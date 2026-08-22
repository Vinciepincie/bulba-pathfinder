// The bot.pathfinder plugin: API-compatible with mineflayer-pathfinder
// (same methods, fields, events, error names and tick-loop semantics), with
// the A* solve moved onto a snapshot + worker thread. Path FOLLOWING is an
// exact port of upstream's monitorMovement (minus dig/place, which this
// package never does), driven by prismarine-physics simulation like
// upstream, so the walking behavior is unchanged.
//
// Improvements (all off by default or strictly additive — see README):
//   - solves run off the event loop (worker_thread), interim partial paths
//     stream back so the bot walks while thinking
//   - keepPathDuringRecompute: keep following the still-valid path while a
//     recompute (goal moved / world changed) is in flight
//   - stuckTimeout / executionTimeout knobs (default off)
//   - stop() releases controls within a tick and never wedges a later goal
import { performance } from 'node:perf_hooks'
import { Vec3 } from 'vec3'
import nbt from 'prismarine-nbt'
import type { Bot } from 'mineflayer'
import { Movements } from './movements.js'
import { Move } from './move.js'
import { Goal } from './goals.js'
import { PhysicsSim } from './physics.js'
import { Solver } from './solver.js'
import type { RawSolveResult } from './solver.js'
import { GoalAdapter } from './goalAdapter.js'
import { getLut } from './lut.js'
import type { BlockLut } from './lut.js'
import { computeDigData, digFingerprint } from './digData.js'
import { fastEvaluator } from './fastEvaluator.js'
import type { DigContext } from './moveGen.js'
import type { DigData, MovementsConfig } from './types.js'
import {
  Snapshot, buildSnapshot, computeBox, applySnapshotBlockUpdate, bakeEntityIndex, nextSnapshotGeneration
} from './snapshot.js'
import type { Box } from './snapshot.js'
import { serializeGoal, goalNeedsRaycast, descriptorTargets } from './goalSerde.js'
import { TAKEOFF_STAND } from './parkourEnvelope.js'
import { getSharedWorkerHost } from './worker/host.js'
import * as geometry from './geometry.js'
import type { PathfinderOptions, GoalDescriptor, PhysicsLike } from './types.js'

// Upstream lib/lock.js.
class Lock {
  private locked = false

  tryAcquire (): boolean {
    if (this.locked) return false
    this.locked = true
    return true
  }

  release (): void {
    this.locked = false
  }
}

export interface ComputedPathResult {
  status: string
  cost: number
  time: number
  visitedNodes: number
  generatedNodes: number
  path: Move[]
  context: { visitedChunks: Set<string>, compute?: () => ComputedPathResult } | null
}

interface BlockLike {
  type: number
  stateId?: number
  position: Vec3
  shapes: number[][]
  boundingBox?: string
}

/**
 * How long server position corrections may keep excusing a lack of progress.
 * Generous enough to cover a real lagback burst, short enough that a bot
 * wedged in geometry still trips the 3.5 s futility check.
 */
const FORCED_MOVE_GRACE_MS = 3000

/**
 * How far from a wall the body has to be for sprinting to be safe. Any
 * positive margin makes exact contact — what prismarine-physics leaves after
 * every horizontal collision — count as "against the wall".
 */
const SPRINT_WALL_MARGIN = 0.03

/**
 * A walking step is at most a diagonal; anything longer is a jump and must
 * not be re-steered along a wall.
 */
const WALK_STEP_REACH = 1.8

/** How far ahead the wall-slide probe looks — over one tick of sprinting. */
const SLIDE_PROBE = 0.35

/**
 * Steering bias AWAY from a wall we are sliding along, as a fraction of the
 * along-wall component. Sliding with a heading exactly parallel is not enough:
 * whatever momentum the bot still carries into the wall gets clamped, and a
 * clamped position sits EXACTLY on the block face — the one value the server
 * disagrees with us about, because prismarine-physics reconstructs the body
 * from `minZ + halfWidth` while the server reconstructs it from the centre,
 * and the two round to opposite sides of the boundary (the 1.21 hitbox
 * precision class again — see geometry.ts). A hair of standoff keeps every
 * position we claim unambiguously outside the block.
 */
const WALL_STANDOFF = 0.25

/**
 * Above this, the next node is something to climb ONTO, not a wall to walk
 * around: a shelf, a step, a ladder exit. Steering away from it there is how a
 * wall-slide turns a one-block step-up into an unreachable node.
 */
const STEP_UP_MIN = 0.1

/** Ground distance one sprinting tick covers, the sprint gate's look-ahead. */
const SPRINT_TICK = 0.3

const DEFAULT_OPTIONS: Required<Omit<PathfinderOptions, 'physicsFactory' | 'onNoPath'>> = {
  useWorkerThreads: true,
  workerEntryPath: '',
  maxSnapshotCells: 8_000_000
}

/**
 * Create the plugin function. `pathfinder` (the default instance) is what
 * `bot.loadPlugin(pathfinder)` takes, exactly like upstream.
 */
export function createPathfinder (options: PathfinderOptions = {}) {
  const opts = { ...DEFAULT_OPTIONS, ...options }

  return function inject (bot: Bot): void {
    const registry = bot.registry
    const waterType: number = registry.blocksByName.water.id
    const ladderId: number = registry.blocksByName.ladder.id
    const vineId: number = registry.blocksByName.vine.id
    const bubbleColumnId: number = registry.blocksByName.bubble_column ? registry.blocksByName.bubble_column.id : -1

    let stateMovements = new Movements(bot)
    let stateGoal: Goal | null = null
    let dynamicGoal = false
    let path: Move[] = []
    let pathUpdated = false
    // Last dispatched solve's inputs, for the onNoPath capture hook.
    let lastSolveStart: { x: number, y: number, z: number } = { x: 0, y: 0, z: 0 }
    /** Block the last solve was dispatched from — see `startSolve` and the
     *  exhausted-path recovery in the tick handler. */
    let lastSolveFrom: { x: number, y: number, z: number } | null = null
    let lastSolveGoal: unknown = null
    let lastSolveCfg: MovementsConfig | null = null
    let placing = false
    let placingBlock: { x: number, y: number, z: number, useOne?: boolean } | null = null
    let digging = false
    let lastNodeTime = performance.now()
    /**
     * When a path node was last genuinely REACHED. `lastNodeTime` is also
     * pushed forward by server position corrections; this one never is, so it
     * can bound how long those corrections are allowed to excuse a lack of
     * progress. See the forcedMove handler.
     */
    let lastNodeArrival = performance.now()
    let goalSetTime = performance.now()
    let stopPathing = false

    const physics = options.physicsFactory ? options.physicsFactory(bot) : new PhysicsSim(bot)
    const lockUseBlock = new Lock()
    const workerHost = getSharedWorkerHost()
    if (opts.workerEntryPath) workerHost.setEntryPath(opts.workerEntryPath)

    // ── async solve state ────────────────────────────────────────────────
    let solveGeneration = 0
    let activeSolveCancel: (() => void) | null = null
    let activeSolveGeneration = -1
    let mainSolver: Solver | null = null
    let growFactor = 1
    let growAttempts = 0
    let lastTouchedChunks: Set<string> | null = null

    // ── snapshot cache ───────────────────────────────────────────────────
    let cachedSnapshot: Snapshot | null = null
    let cachedLut: BlockLut | null = null
    let snapshotStale = true
    const pendingPatches: Array<{ x: number, y: number, z: number, stateId: number }> = []

    const pf: Record<string, unknown> = {}
    ;(bot as unknown as { pathfinder: typeof pf }).pathfinder = pf

    pf.thinkTimeout = 5000 // ms
    pf.tickTimeout = 40 // ms, amount of thinking per tick (parity; also worker partial cadence)
    pf.searchRadius = -1 // cost slack, -1: don't limit the search
    pf.enablePathShortcut = false
    pf.LOSWhenPlacingBlocks = true
    // Improvements, default off / additive:
    pf.stuckTimeout = -1 // ms without reaching the next node → path_stop (goto rejects PathStopped)
    pf.executionTimeout = -1 // ms per goal overall → path_stop
    pf.keepPathDuringRecompute = true // keep walking the valid prefix while replanning

    pf.bestHarvestTool = (block: { digTime: (...args: unknown[]) => number }) => {
      const availableTools = bot.inventory.items()
      const effects = (bot.entity as { effects?: unknown }).effects

      let fastest = Number.MAX_VALUE
      let bestTool = null
      for (const tool of availableTools) {
        const enchants = (tool && tool.nbt) ? (nbt.simplify(tool.nbt) as { Enchantments?: unknown[] }).Enchantments : []
        const digTime = block.digTime(tool ? tool.type : null, false, false, false, enchants, effects)
        if (digTime < fastest) {
          fastest = digTime
          bestTool = tool
        }
      }
      return bestTool
    }

    Object.defineProperties(pf, {
      goal: { get () { return stateGoal } },
      movements: { get () { return stateMovements } }
    })

    pf.isMoving = () => path.length > 0
    pf.isMining = () => digging // only ever true when movements.canDig is enabled
    pf.isBuilding = () => placing // true while activating a door/gate

    // ── absorbed wrapper scaffolding (additive; upstream has no equivalents) ──
    pf.isStuck = () => geometry.isStuck(bot)
    pf.canStandAt = (pos: Vec3) => geometry.canStandAt(bot, pos)
    pf.findEscape = (radius?: number) => geometry.findEscape(bot, radius)
    pf.findNearestOpenStandable = (radius?: number) => geometry.findNearestOpenStandable(bot, radius)
    pf.unstick = async (): Promise<boolean> => {
      // Stop driving first — an active goal's tick loop would fight the
      // escape controls every tick (the wrapper's unstick stopped first too).
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      return geometry.unstick(bot)
    }
    pf.centreInCell = (cell: Vec3, tolerance?: number) => geometry.centreInCell(bot, cell, tolerance)
    pf.jumpOnce = () => geometry.jumpOnce(bot)
    pf.settle = async (): Promise<boolean> => {
      // Come to a verified standstill: drop any goal (listeners see the
      // upstream goal_updated/GoalChanged contract), release controls, then
      // wait for drift to actually end; walk out if embedded.
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      bot.clearControlStates()
      if (geometry.isStuck(bot)) await geometry.unstick(bot)
      return geometry.settle(bot)
    }

    // ── snapshot management ──────────────────────────────────────────────

    function flushPatches (): void {
      // Never touch the SAB while a solve reads it; every solve-completion
      // boundary calls back here, so the queue drains promptly. Updates are
      // ALWAYS queued (never applied directly) so FIFO order guarantees the
      // newest write wins — a direct-write/queued-write interleave used to
      // let a stale queued value overwrite a newer one.
      if (solveInFlight()) return
      if (!cachedSnapshot || !cachedLut) {
        pendingPatches.length = 0
        return
      }
      for (const p of pendingPatches) {
        applySnapshotBlockUpdate(cachedSnapshot, cachedLut, p.x, p.y, p.z, p.stateId)
      }
      pendingPatches.length = 0
    }

    function solveInFlight (): boolean {
      return activeSolveCancel !== null || mainSolver !== null
    }

    function ensureSnapshot (start: Vec3, descriptor: GoalDescriptor | null, needStates: boolean): Snapshot {
      const lut = getLut(bot, stateMovements)
      const slack = pf.searchRadius as number
      const targets = descriptor ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })

      const lutChanged = cachedLut !== lut
      let rebuild = snapshotStale || lutChanged || !cachedSnapshot
      if (!rebuild && cachedSnapshot) {
        // Box still adequate? Start + targets must sit inside with margin.
        const m = cachedSnapshot.meta
        const inside = (x: number, y: number, z: number): boolean =>
          x >= m.x0 + 8 && x <= m.x0 + m.w - 9 && y >= m.y0 + 4 && y <= m.y0 + m.h - 5 &&
          z >= m.z0 + 8 && z <= m.z0 + m.l - 9
        if (!inside(start.x, start.y, start.z)) rebuild = true
        for (const t of targets) {
          if (!inside(t.x, t.y ?? start.y, t.z)) rebuild = true
        }
        if (needStates && !cachedSnapshot.states) rebuild = true
      }

      if (rebuild) {
        const box: Box = computeBox(bot, start, targets, slack, growFactor, opts.maxSnapshotCells)
        cachedSnapshot = buildSnapshot(bot, lut, box, needStates, nextSnapshotGeneration())
        cachedLut = lut
        snapshotStale = false
        pendingPatches.length = 0
      } else {
        flushPatches()
      }
      return cachedSnapshot as Snapshot
    }

    // ── solve orchestration ──────────────────────────────────────────────

    function cancelActiveSolve (): void {
      solveGeneration++
      if (activeSolveCancel) {
        activeSolveCancel()
        activeSolveCancel = null
      }
      activeSolveGeneration = -1
      mainSolver = null
      // Solve is no longer in flight (a cancelled worker's result is
      // discarded by generation, so late SAB reads are harmless) — apply
      // any block updates that queued up during it.
      flushPatches()
    }

    function computeStartMove (startPos: Vec3): { x: number, y: number, z: number } {
      const p = startPos.floored()
      const dy = startPos.y - p.y
      const b = bot.blockAt(p) as BlockLike | null
      // Upstream quirk preserved: uses stateMovements.emptyBlocks regardless
      // of the movements passed to getPathFromTo.
      const offset = (b && dy > 0.001 && bot.entity.onGround && !stateMovements.emptyBlocks.has(b.type)) ? 1 : 0
      return { x: p.x, y: p.y + offset, z: p.z }
    }

    function toResult (raw: RawSolveResult, context: ComputedPathResult['context']): ComputedPathResult {
      return {
        status: raw.status,
        cost: raw.cost,
        time: raw.time,
        visitedNodes: raw.visitedNodes,
        generatedNodes: raw.generatedNodes,
        path: raw.path.map(Move.fromRaw),
        context
      }
    }

    function contextFromRaw (raw: RawSolveResult): ComputedPathResult['context'] {
      const visitedChunks = new Set<string>()
      for (const [cx, cz] of raw.touchedChunks) visitedChunks.add(`${cx},${cz}`)
      return { visitedChunks }
    }

    function handleDriveResult (raw: RawSolveResult, generation: number, final: boolean): void {
      if (generation !== solveGeneration) return // stale (goal changed etc.)

      // Transparent snapshot growth: a boundary-limited noPath is retried
      // with a larger box before anything is emitted.
      if (final && raw.status === 'noPath' && raw.boundaryLimited && growAttempts < 5) {
        const snap = cachedSnapshot
        if (snap && snap.cellCount < opts.maxSnapshotCells) {
          growAttempts++
          growFactor *= 1.8
          snapshotStale = true
          activeSolveCancel = null
          activeSolveGeneration = -1
          mainSolver = null
          flushPatches()
          return // next physicsTick startSolve()s again with the bigger box
        }
      }

      // Terminal no-path (growth exhausted): hand the exact solver inputs to
      // the capture hook BEFORE queued patches mutate the snapshot.
      if (final && raw.status === 'noPath' && opts.onNoPath && cachedSnapshot && lastSolveGoal !== null && lastSolveCfg !== null) {
        try {
          opts.onNoPath({
            meta: cachedSnapshot.meta,
            flags: cachedSnapshot.flags,
            heights: cachedSnapshot.heights,
            start: lastSolveStart,
            goal: lastSolveGoal,
            cfg: lastSolveCfg,
            visitedNodes: raw.visitedNodes
          })
        } catch (error) {
          console.warn('[bulba-pathfinder] onNoPath hook threw:', error)
        }
      }

      if (final) {
        activeSolveCancel = null
        activeSolveGeneration = -1
        flushPatches()
      }

      const results = toResult(raw, contextFromRaw(raw))
      lastTouchedChunks = results.context?.visitedChunks ?? null
      results.path = postProcessPath(results.path)
      pathFromPlayer(results.path)
      bot.emit('path_update' as never, results as never)
      path = results.path
      if (final) pathUpdated = true
      lastNodeTime = performance.now()
      // A freshly installed path deserves a full grace window: the bot has
      // not failed to reach anything on it yet.
      lastNodeArrival = lastNodeTime
    }

    /** Has the bot left the block the last solve was dispatched from? */
    function movedSinceLastSolve (): boolean {
      if (lastSolveFrom === null) return true
      const p = bot.entity.position.floored()
      return p.x !== lastSolveFrom.x || p.y !== lastSolveFrom.y || p.z !== lastSolveFrom.z
    }

    function startSolve (): void {
      if (!stateGoal || !stateMovements) return
      try {
        stateMovements.assertSupported()
      } catch (error) {
        // Post-hoc mutation (movements.canDig = true after setMovements)
        // must not throw inside the physicsTick handler: fail the goal
        // instead (goto rejects PathStopped) and say why, once per goal.
        console.error('[bulba-pathfinder] unsupported movements profile — stopping the goal:', (error as Error).message)
        internalStop()
        return
      }

      const generation = ++solveGeneration
      const startPos = bot.entity.position
      const start = computeStartMove(startPos)

      if (stateMovements.allowEntityDetection) {
        stateMovements.clearCollisionIndex()
        stateMovements.updateCollisionIndex()
      }

      const descriptor = serializeGoal(stateGoal)
      lastSolveStart = start
      lastSolveFrom = startPos.floored()
      lastSolveGoal = descriptor
      lastSolveCfg = stateMovements.toConfig()
      const canUseWorker =
        opts.useWorkerThreads &&
        descriptor !== null &&
        stateMovements.exclusionAreasStep.length === 0 &&
        (!stateMovements.canDig || stateMovements.exclusionAreasBreak.length === 0) &&
        !workerHost.unavailable

      const needStates = (descriptor !== null && goalNeedsRaycast(descriptor)) || stateMovements.canDig
      const snapshot = ensureSnapshot(startPos, descriptor, needStates)
      bakeEntityIndex(snapshot, stateMovements)
      const digData = stateMovements.canDig ? getDigDataCached(stateMovements) : null

      if (canUseWorker && descriptor) {
        const req = {
          snapshot,
          lut: cachedLut as BlockLut,
          cfg: stateMovements.toConfig(),
          goal: descriptor,
          start,
          timeout: pf.thinkTimeout as number,
          searchRadius: pf.searchRadius as number,
          sliceMs: Math.max(10, pf.tickTimeout as number),
          dig: digData,
          onPartial: (raw: RawSolveResult) => handleDriveResult(raw, generation, false)
        }
        // Mark in-flight synchronously so the next tick doesn't double-start.
        activeSolveGeneration = generation
        activeSolveCancel = () => {} // placeholder until the handle arrives
        workerHost.solve(req).then((handle) => {
          if (!handle) {
            // Worker unavailable → main-thread fallback (next tick).
            if (generation === solveGeneration) {
              activeSolveCancel = null
              activeSolveGeneration = -1
              startMainThreadSolve(generation, snapshot, start)
            }
            return
          }
          if (generation !== solveGeneration) {
            handle.cancel()
            return
          }
          activeSolveCancel = handle.cancel
          handle.promise.then(
            (raw) => { handleDriveResult(raw, generation, true) },
            () => {
              // Worker crashed mid-solve: retry this goal on the main thread.
              if (generation === solveGeneration) {
                activeSolveCancel = null
                activeSolveGeneration = -1
                startMainThreadSolve(generation, snapshot, start)
              }
            }
          )
        }, () => {
          if (generation === solveGeneration) {
            activeSolveCancel = null
            activeSolveGeneration = -1
            startMainThreadSolve(generation, snapshot, start)
          }
        })
      } else {
        startMainThreadSolve(generation, snapshot, start)
      }
    }

    function liveStepExclusion (x: number, y: number, z: number): number {
      const block = bot.blockAt(new Vec3(x, y, z), false)
      return block ? stateMovements.exclusionStep(block) : 0
    }

    function liveBreakExclusion (x: number, y: number, z: number): number {
      const block = bot.blockAt(new Vec3(x, y, z), false)
      return block ? stateMovements.exclusionBreak(block) : 0
    }

    // Dig tables cached by (lut × inventory × effects) fingerprint.
    let cachedDigData: DigData | null = null

    function getDigDataCached (movements: Movements): DigData {
      const lut = getLut(bot, movements)
      const fp = digFingerprint(bot, movements, lut)
      if (!cachedDigData || cachedDigData.fingerprint !== fp) {
        cachedDigData = computeDigData(bot, movements, lut)
      }
      return cachedDigData
    }

    function digContextFor (movements: Movements, snapshot: Snapshot, digData: DigData | null): DigContext | null {
      if (!movements.canDig || !digData) return null
      return {
        data: digData,
        states: snapshot.allocStates(),
        breakExclusion: movements.exclusionAreasBreak.length > 0 ? liveBreakExclusion : null
      }
    }

    function startMainThreadSolve (generation: number, snapshot: Snapshot, start: { x: number, y: number, z: number }): void {
      if (!stateGoal) return
      const stepExclusion = stateMovements.exclusionAreasStep.length > 0 ? liveStepExclusion : null
      const digData = stateMovements.canDig ? getDigDataCached(stateMovements) : null
      const descriptor = serializeGoal(stateGoal)
      const evaluator = (descriptor && fastEvaluator(descriptor)) ?? new GoalAdapter(stateGoal)
      const solver = new Solver(
        snapshot,
        stateMovements.toConfig(),
        evaluator,
        start,
        { timeout: pf.thinkTimeout as number, searchRadius: pf.searchRadius as number },
        stepExclusion,
        digContextFor(stateMovements, snapshot, digData)
      )
      // First slice synchronously (upstream computes in-tick); subsequent
      // slices continue from monitorMovement.
      const raw = solver.compute(pf.tickTimeout as number)
      if (raw.status === 'partial') {
        mainSolver = solver
        activeSolveGeneration = generation
      }
      handleDriveResult(raw, generation, raw.status !== 'partial')
    }

    // ── path post-processing (upstream ports) ────────────────────────────

    function getPositionOnTopOf (block: BlockLike | null): Vec3 | null {
      if (!block || block.shapes.length === 0) return null
      const p = new Vec3(0.5, 0, 0.5)
      let n = 1
      for (const shape of block.shapes) {
        const h = shape[4]
        if (h === p.y) {
          p.x += (shape[0] + shape[3]) / 2
          p.z += (shape[2] + shape[5]) / 2
          n++
        } else if (h > p.y) {
          n = 2
          p.x = 0.5 + (shape[0] + shape[3]) / 2
          p.y = h
          p.z = 0.5 + (shape[2] + shape[5]) / 2
        }
      }
      p.x /= n
      p.z /= n
      return block.position.plus(p)
    }

    function postProcessPath (thePath: Move[]): Move[] {
      for (let i = 0; i < thePath.length; i++) {
        const curPoint = thePath[i]
        if (curPoint.toBreak.length > 0 || curPoint.toPlace.length > 0) break
        const b = bot.blockAt(new Vec3(curPoint.x, curPoint.y, curPoint.z)) as BlockLike | null
        if (b && (b.type === waterType || b.type === bubbleColumnId ||
            ((b.type === ladderId || b.type === vineId) && i + 1 < thePath.length && thePath[i + 1].y < curPoint.y))) {
          curPoint.x = Math.floor(curPoint.x) + 0.5
          curPoint.y = Math.floor(curPoint.y)
          curPoint.z = Math.floor(curPoint.z) + 0.5
          continue
        }
        let np = getPositionOnTopOf(b)
        if (np === null) np = getPositionOnTopOf(bot.blockAt(new Vec3(curPoint.x, curPoint.y - 1, curPoint.z)) as BlockLike | null)
        if (np) {
          curPoint.x = np.x
          curPoint.y = np.y
          curPoint.z = np.z
        } else {
          curPoint.x = Math.floor(curPoint.x) + 0.5
          curPoint.y = curPoint.y - 1
          curPoint.z = Math.floor(curPoint.z) + 0.5
        }
      }

      if (!(pf.enablePathShortcut as boolean) || stateMovements.exclusionAreasStep.length !== 0 || thePath.length === 0) return thePath

      const newPath: Move[] = []
      let lastNode: { x: number, y: number, z: number } = bot.entity.position
      for (let i = 1; i < thePath.length; i++) {
        const node = thePath[i]
        if (Math.abs(node.y - lastNode.y) > 0.5 || node.toBreak.length > 0 || node.toPlace.length > 0 ||
            !physics.canStraightLineBetween(new Vec3(lastNode.x, lastNode.y, lastNode.z), node)) {
          newPath.push(thePath[i - 1])
          lastNode = thePath[i - 1]
        }
      }
      newPath.push(thePath[thePath.length - 1])
      return newPath
    }

    function pathFromPlayer (thePath: Move[]): void {
      if (thePath.length === 0) return
      let minI = 0
      let minDistance = 1000
      for (let i = 0; i < thePath.length; i++) {
        const node = thePath[i]
        if (node.toBreak.length !== 0 || node.toPlace.length !== 0) break
        const dist = bot.entity.position.distanceSquared(node)
        if (dist < minDistance) {
          minDistance = dist
          minI = i
        }
      }
      const n1 = thePath[minI]
      const dx = n1.x - bot.entity.position.x
      const dy = n1.y - bot.entity.position.y
      const dz = n1.z - bot.entity.position.z
      const reached = Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1
      if (minI + 1 < thePath.length && n1.toBreak.length === 0 && n1.toPlace.length === 0) {
        const n2 = thePath[minI + 1]
        const d2 = bot.entity.position.distanceSquared(n2)
        const d12 = n1.distanceSquared(n2)
        minI += d12 > d2 || reached ? 1 : 0
      }
      thePath.splice(0, minI)
    }

    function isPositionNearPath (pos: Vec3 | undefined, thePath: Move[]): boolean {
      if (!pos) return false
      let prevNode: Move | null = null
      for (const node of thePath) {
        let comparisonPoint: { x: number, y: number, z: number } | null = null
        if (
          prevNode === null ||
          (
            Math.abs(prevNode.x - node.x) <= 2 &&
            Math.abs(prevNode.y - node.y) <= 2 &&
            Math.abs(prevNode.z - node.z) <= 2
          )
        ) {
          comparisonPoint = node
        } else {
          const minBound = prevNode.min(node)
          const maxBound = prevNode.max(node)
          if (
            pos.x - 0.5 < minBound.x - 1 ||
            pos.x - 0.5 > maxBound.x + 1 ||
            pos.y - 0.5 < minBound.y - 2 ||
            pos.y - 0.5 > maxBound.y + 2 ||
            pos.z - 0.5 < minBound.z - 1 ||
            pos.z - 0.5 > maxBound.z + 1
          ) {
            continue // upstream does NOT advance prevNode here
          }
          comparisonPoint = closestPointOnLineSegment(pos, prevNode, node)
        }

        const dx = Math.abs(comparisonPoint.x - pos.x - 0.5)
        const dy = Math.abs(comparisonPoint.y - pos.y - 0.5)
        const dz = Math.abs(comparisonPoint.z - pos.z - 0.5)
        if (dx <= 1 && dy <= 2 && dz <= 1) return true

        prevNode = node
      }
      return false
    }

    function closestPointOnLineSegment (point: Vec3, segmentStart: Vec3, segmentEnd: Vec3): Vec3 {
      const segmentLength = segmentEnd.minus(segmentStart).norm()
      if (segmentLength === 0) return segmentStart
      let t = (point.minus(segmentStart)).dot(segmentEnd.minus(segmentStart)) / segmentLength
      t = Math.max(0, Math.min(1, t))
      return segmentStart.plus(segmentEnd.minus(segmentStart).scaled(t))
    }

    // ── stop / reset machinery (upstream ports + hardened cancel) ────────

    function fullStop (): void {
      bot.clearControlStates()

      // Force horizontal velocity to 0 (otherwise inertia can move us too far)
      bot.entity.velocity.x = 0
      bot.entity.velocity.z = 0

      const blockX = Math.floor(bot.entity.position.x) + 0.5
      const blockZ = Math.floor(bot.entity.position.z) + 0.5

      if (Math.abs(bot.entity.position.x - blockX) > 0.2) { bot.entity.position.x = blockX }
      if (Math.abs(bot.entity.position.z - blockZ) > 0.2) { bot.entity.position.z = blockZ }
    }

    function internalStop (): void {
      stopPathing = false
      stateGoal = null
      path = []
      pathUpdated = false
      lastSolveFrom = null
      stopDiggingIfNeeded()
      placing = false
      placingBlock = null
      cancelActiveSolve()
      bot.emit('path_stop' as never)
      fullStop()
    }

    // Upstream detectDiggingStopped: clear the digging latch when an
    // interrupted dig settles.
    function detectDiggingStopped (): void {
      digging = false
      bot.removeAllListeners('diggingAborted' as never)
      bot.removeAllListeners('diggingCompleted' as never)
    }

    function stopDiggingIfNeeded (): void {
      if (!digging) return
      bot.on('diggingAborted' as never, detectDiggingStopped as never)
      bot.on('diggingCompleted' as never, detectDiggingStopped as never)
      try {
        (bot as unknown as { stopDigging: () => void }).stopDigging()
      } catch {
        digging = false
      }
    }

    function resetPath (reason: string, clearStates = true): void {
      if (!stopPathing && path.length > 0) bot.emit('path_reset' as never, reason as never)
      path = []
      stopDiggingIfNeeded()
      placing = false
      placingBlock = null
      pathUpdated = false
      cancelActiveSolve()
      lockUseBlock.release()
      stateMovements.clearCollisionIndex()
      if (clearStates) bot.clearControlStates()
      if (stopPathing) internalStop()
    }

    /**
     * Improvement over upstream resetPath(reason, false): keep following the
     * still-valid path while the recompute runs, then splice over. Falls back
     * to the upstream hard reset when disabled or when there is nothing to keep.
     */
    function softReplan (reason: string): void {
      if (!(pf.keepPathDuringRecompute as boolean) || path.length === 0 || placing) {
        resetPath(reason, false)
        return
      }
      if (!stopPathing) bot.emit('path_reset' as never, reason as never)
      pathUpdated = false
      cancelActiveSolve()
      stateMovements.clearCollisionIndex()
      if (stopPathing) internalStop()
    }

    // ── public API ───────────────────────────────────────────────────────

    pf.setGoal = (goal: Goal | null, dynamic = false) => {
      stateGoal = goal
      dynamicGoal = dynamic
      goalSetTime = performance.now()
      growFactor = 1
      growAttempts = 0
      bot.emit('goal_updated' as never, goal as never, dynamic as never)
      resetPath('goal_updated')
    }

    pf.setMovements = (movements: Movements) => {
      movements.assertSupported()
      stateMovements = movements
      snapshotStale = true
      resetPath('movements_updated')
    }

    pf.stop = () => {
      // Improvement: stop() when idle doesn't latch a flag that would kill
      // the NEXT goal (upstream quirk); stopping an active goal still emits
      // path_stop (goto rejects 'PathStopped') within a tick.
      if (!stateGoal && path.length === 0 && !solveInFlight()) return
      stopPathing = true
    }

    pf.goto = (goal: Goal) => {
      return gotoImpl(goal)
    }

    function gotoImpl (goal: Goal): Promise<void> {
      // Port of upstream lib/goto.js — verbatim event contract and error
      // names; the wrapper string-matches these, keep them byte-identical.
      return new Promise((resolve, reject) => {
        function makeError (name: string, message: string): Error {
          const err = new Error(message)
          err.name = name
          return err
        }

        function goalReached (): void {
          cleanup()
        }

        function noPathListener (results: ComputedPathResult): void {
          if (results.path.length === 0) {
            cleanup()
          } else if (results.status === 'noPath') {
            cleanup(makeError('NoPath', 'No path to the goal!'))
          } else if (results.status === 'timeout') {
            cleanup(makeError('Timeout', 'Took to long to decide path to goal!'))
          }
        }

        function goalChangedListener (newGoal: Goal): void {
          if (newGoal !== goal) {
            cleanup(makeError('GoalChanged', 'The goal was changed before it could be completed!'))
          }
        }

        function pathStopped (): void {
          cleanup(makeError('PathStopped', 'Path was stopped before it could be completed! Thus, the desired goal was not reached.'))
        }

        function cleanup (err?: Error): void {
          bot.removeListener('goal_reached' as never, goalReached as never)
          bot.removeListener('path_update' as never, noPathListener as never)
          bot.removeListener('goal_updated' as never, goalChangedListener as never)
          bot.removeListener('path_stop' as never, pathStopped as never)
          setTimeout(() => {
            if (err) reject(err)
            else resolve()
          }, 0)
        }

        bot.on('path_stop' as never, pathStopped as never)
        bot.on('goal_reached' as never, goalReached as never)
        bot.on('path_update' as never, noPathListener as never)
        bot.on('goal_updated' as never, goalChangedListener as never)
        ;(pf.setGoal as (g: Goal | null, dynamic?: boolean) => void)(goal)
      })
    }

    // Synchronous compute API (upstream parity — main-thread, tick-sliced).
    pf.getPathFromTo = function * (movements: Movements, startPos: Vec3, goal: Goal, options: {
      optimizePath?: boolean
      resetEntityIntersects?: boolean
      timeout?: number
      tickTimeout?: number
      searchRadius?: number
      startMove?: { x: number, y: number, z: number }
    } = {}): Generator<{ result: ComputedPathResult, astarContext: unknown }> {
      movements.assertSupported()
      const optimizePath = options.optimizePath ?? true
      const resetEntityIntersects = options.resetEntityIntersects ?? true
      const timeout = options.timeout ?? (pf.thinkTimeout as number)
      const tickTimeout = options.tickTimeout ?? (pf.tickTimeout as number)
      const searchRadius = options.searchRadius ?? (pf.searchRadius as number)

      let start: { x: number, y: number, z: number }
      if (options.startMove) {
        start = options.startMove
      } else {
        start = computeStartMove(startPos)
      }
      if (movements.allowEntityDetection) {
        if (resetEntityIntersects) movements.clearCollisionIndex()
        movements.updateCollisionIndex()
      }

      // A private snapshot for this query (doesn't disturb the drive cache).
      const lut = getLut(bot, movements)
      const descriptor = serializeGoal(goal)
      const targets = descriptor ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })
      const box = computeBox(bot, startPos, targets, searchRadius, 1, opts.maxSnapshotCells)
      const snap = buildSnapshot(bot, lut, box, (descriptor ? goalNeedsRaycast(descriptor) : true) || movements.canDig, nextSnapshotGeneration())
      bakeEntityIndex(snap, movements)

      const stepExclusion = movements.exclusionAreasStep.length > 0
        ? (x: number, y: number, z: number) => {
            const block = bot.blockAt(new Vec3(x, y, z), false)
            return block ? movements.exclusionStep(block) : 0
          }
        : null
      let queryDig: DigContext | null = null
      if (movements.canDig) {
        queryDig = {
          data: getDigDataCached(movements),
          states: snap.allocStates(),
          breakExclusion: movements.exclusionAreasBreak.length > 0
            ? (x: number, y: number, z: number) => {
                const block = bot.blockAt(new Vec3(x, y, z), false)
                return block ? movements.exclusionBreak(block) : 0
              }
            : null
        }
      }
      const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), start, { timeout, searchRadius }, stepExclusion, queryDig)
      const astarContext = {
        get visitedChunks () { return solver.visitedChunks },
        compute: () => toResult(solver.compute(tickTimeout), null)
      }

      // NOTE: unlike drive solves, a read-only query must NOT touch
      // lastTouchedChunks (the drive loop's chunkColumnLoad replan state).
      let raw = solver.compute(tickTimeout)
      let result = toResult(raw, { visitedChunks: solver.visitedChunks })
      if (optimizePath) result.path = postProcessPath(result.path)
      yield { result, astarContext }
      while (result.status === 'partial') {
        raw = solver.compute(tickTimeout)
        result = toResult(raw, { visitedChunks: solver.visitedChunks })
        if (optimizePath) result.path = postProcessPath(result.path)
        yield { result, astarContext }
      }
    }

    pf.getPathTo = (movements: Movements, goal: Goal, timeout?: number) => {
      const generator = (pf.getPathFromTo as (...args: unknown[]) => Generator<{ result: ComputedPathResult, astarContext: unknown }>)(
        movements, bot.entity.position, goal, { timeout }
      )
      const { value } = generator.next()
      return (value as { result: ComputedPathResult }).result
    }

    // ── world-change invalidation ────────────────────────────────────────

    bot.on('blockUpdate' as never, ((oldBlock: BlockLike | null, newBlock: BlockLike | null) => {
      if (!oldBlock || !newBlock) return

      // Keep the snapshot current. Always queue-then-flush (FIFO) so a
      // newer write can never be overwritten by an older queued one.
      if (cachedSnapshot && cachedSnapshot.contains(newBlock.position.x, newBlock.position.y, newBlock.position.z)) {
        pendingPatches.push({
          x: newBlock.position.x,
          y: newBlock.position.y,
          z: newBlock.position.z,
          stateId: newBlock.stateId ?? -1
        })
        flushPatches() // no-op while a solve is reading the SAB
      }

      if (isPositionNearPath(oldBlock.position, path) && oldBlock.type !== newBlock.type) {
        // Hard-clear when the change is imminent on our path; otherwise keep
        // walking the prefix while we replan.
        let imminent = false
        for (let i = 0; i < Math.min(3, path.length); i++) {
          const n = path[i]
          if (Math.abs(n.x - oldBlock.position.x) <= 1 && Math.abs(n.z - oldBlock.position.z) <= 1 &&
              Math.abs(n.y - oldBlock.position.y) <= 2) {
            imminent = true
            break
          }
        }
        if (imminent) resetPath('block_updated', false)
        else softReplan('block_updated')
      }
    }) as never)

    // Lagback / teleport reaction (improvement — upstream walks at a stale
    // node until the 3.5s futility timer). On a server position correction:
    // still near the path → splice to the closest node and keep walking;
    // snapped away → replan immediately from the corrected position.
    bot.on('forcedMove' as never, (() => {
      if (path.length === 0) return
      const at = bot.entity.position.floored()
      if (isPositionNearPath(at, path)) {
        pathFromPlayer(path)
        // A correction is not "stuck" — but only for as long as the bot is
        // otherwise making progress. A bot WEDGED in geometry gets corrected
        // every tick, and refreshing unconditionally meant the 3.5 s futility
        // timer was reset before it could ever expire: measured on 2b2t
        // spawn, 46 s motionless with 34 path nodes left and not one `stuck`
        // reset, in the same spot every run. Upstream has no such handler and
        // walks the same route without trouble. Past the grace window the
        // corrections stop excusing it and the futility check does its job.
        if (performance.now() - lastNodeArrival < FORCED_MOVE_GRACE_MS) {
          lastNodeTime = performance.now()
        }
      } else {
        resetPath('forced_move', false)
      }
    }) as never)

    bot.on('chunkColumnLoad' as never, ((chunk: Vec3) => {
      const cx = chunk.x >> 4
      const cz = chunk.z >> 4
      if (cachedSnapshot) {
        const m = cachedSnapshot.meta
        if ((cx << 4) + 15 >= m.x0 && (cx << 4) <= m.x0 + m.w - 1 &&
            (cz << 4) + 15 >= m.z0 && (cz << 4) <= m.z0 + m.l - 1) {
          snapshotStale = true
        }
      }
      if (lastTouchedChunks && (stateGoal !== null || path.length > 0)) {
        if (lastTouchedChunks.has(`${cx - 1},${cz}`) ||
            lastTouchedChunks.has(`${cx},${cz - 1}`) ||
            lastTouchedChunks.has(`${cx + 1},${cz}`) ||
            lastTouchedChunks.has(`${cx},${cz + 1}`)) {
          softReplan('chunk_loaded')
        }
      }
    }) as never)

    // ── the tick loop (upstream monitorMovement port) ────────────────────

    function monitorMovement (): void {
      // Improvement: a requested stop takes effect within one tick, even
      // mid-edge — controls are always released.
      if (stopPathing) {
        internalStop()
        return
      }

      // Test freemotion
      const freeGoal = stateGoal as (Goal & { entity?: { position: Vec3 }, rangeSq?: number }) | null
      if (stateMovements && stateMovements.allowFreeMotion && freeGoal && freeGoal.entity) {
        const target = freeGoal.entity
        if (physics.canStraightLine([target.position])) {
          bot.lookAt(target.position.offset(0, 1.6, 0))
          // Upstream: `dist > rangeSq` — false when rangeSq is undefined, so
          // an entity goal without rangeSq STOPS at the entity (no ?? 0!).
          const rangeSq = freeGoal.rangeSq
          if (rangeSq !== undefined && target.position.distanceSquared(bot.entity.position) > rangeSq) {
            bot.setControlState('forward', true)
          } else {
            bot.clearControlStates()
          }
          return
        }
      }

      if (stateGoal) {
        if (!stateGoal.isValid()) {
          internalStop()
        } else if (stateGoal.hasChanged()) {
          softReplan('goal_moved')
        }
      }

      // Improvement knobs (default off).
      if (stateGoal) {
        const now = performance.now()
        if ((pf.executionTimeout as number) > 0 && now - goalSetTime > (pf.executionTimeout as number)) {
          internalStop()
          return
        }
        if ((pf.stuckTimeout as number) > 0 && path.length > 0 && !placing && !digging &&
            now - lastNodeTime > (pf.stuckTimeout as number)) {
          internalStop()
          return
        }
      }

      // Main-thread sliced solve continuation (upstream astartTimedout).
      if (mainSolver) {
        const generation = activeSolveGeneration
        const raw = mainSolver.compute(pf.tickTimeout as number)
        if (raw.status !== 'partial') mainSolver = null
        handleDriveResult(raw, generation, raw.status !== 'partial')
      }

      if (path.length === 0) {
        lastNodeTime = performance.now()
        if (stateGoal && stateMovements) {
          if (stateGoal.isEnd(bot.entity.position.floored())) {
            if (!dynamicGoal) {
              cancelActiveSolve()
              bot.emit('goal_reached' as never, stateGoal as never)
              stateGoal = null
              fullStop()
              return
            }
            // Dynamic goal standing at its end: idle until hasChanged fires
            // (upstream else-if structure — no recompute here).
          } else if (!solveInFlight() && (!pathUpdated || movedSinceLastSolve())) {
            // Divergence from upstream, and the reason for it: upstream gates
            // this purely on `!pathUpdated`, which latches true as soon as any
            // final path is delivered. A path that is then walked to
            // exhaustion WITHOUT reaching the goal therefore leaves the bot
            // with nothing to walk, no recompute, and no escape — and because
            // `lastNodeTime` is refreshed above on every empty-path tick, the
            // 3.5 s futility timer can never fire either. The bot stands
            // still forever, waiting for a block change or a new goal.
            // Measured on 2b2t spawn: 83 s of a 97 s run motionless, and runs
            // that ended 29 blocks short of the goal having stopped trying.
            //
            // Re-solving when the bot is on a DIFFERENT block than the last
            // solve started from fixes it while staying bounded: a genuinely
            // unreachable goal still costs exactly one extra solve, because a
            // stationary bot never satisfies the condition again.
            startSolve()
          }
        }
      } else if (stateGoal && stateMovements && !pathUpdated && !solveInFlight()) {
        // Soft replan: keep walking the old prefix while recomputing.
        startSolve()
      }

      if (path.length === 0) return

      let nextPoint: Move = path[0]
      const p = bot.entity.position

      // Handle digging (canDig solves only) — upstream port: equip the best
      // tool, dig with forceLook, stand still until the break lands.
      if (digging || nextPoint.toBreak.length > 0) {
        if (!digging && bot.entity.onGround) {
          digging = true
          const b = nextPoint.toBreak.shift() as Vec3
          const block = bot.blockAt(new Vec3(b.x, b.y, b.z), false)
          if (!block) {
            digging = false
            resetPath('dig_error')
            return
          }
          const tool = (pf.bestHarvestTool as (blk: unknown) => unknown)(block as never)
          fullStop()
          const pathToken = path
          const digBlock = (): void => {
            (bot as unknown as { dig: (blk: unknown, forceLook: boolean) => Promise<void> })
              .dig(block, true)
              .catch(() => {
                if (path === pathToken) resetPath('dig_error')
              })
              .then(() => {
                lastNodeTime = performance.now()
                digging = false
              })
          }
          if (!tool) {
            digBlock()
          } else {
            (bot as unknown as { equip: (item: unknown, dest: string) => Promise<void> })
              .equip(tool, 'hand')
              .catch(() => {})
              .then(() => digBlock())
          }
        }
        return
      }

      // Door/fence-gate activation (activate — never place).
      if (placing || nextPoint.toPlace.length > 0) {
        if (!placing) {
          placing = true
          placingBlock = nextPoint.toPlace.shift() ?? null
          fullStop()
        }
        if (placingBlock?.useOne) {
          if (!lockUseBlock.tryAcquire()) return
          const block = bot.blockAt(new Vec3(placingBlock.x, placingBlock.y, placingBlock.z))
          if (!block) {
            lockUseBlock.release()
            resetPath('place_error')
            return
          }
          // activateBlock settles async — if the goal/path changed meanwhile,
          // its callbacks must not touch the NEW path's state.
          const pathToken = path
          bot.activateBlock(block).then(() => {
            lockUseBlock.release()
            if (path !== pathToken) return
            placingBlock = nextPoint.toPlace.shift() ?? null
            if (!placingBlock) {
              placing = false
              lastNodeTime = performance.now()
            }
          }, (err: unknown) => {
            console.error(err)
            lockUseBlock.release()
            if (path !== pathToken) return
            resetPath('place_error')
          })
          return
        }
        // No other toPlace kind can exist in our paths; clear defensively.
        placing = false
        placingBlock = null
        return
      }

      let dx = nextPoint.x - p.x
      let dy = nextPoint.y - p.y
      let dz = nextPoint.z - p.z
      // Improvement: never finish a path mid-air. The arrival box has no
      // ground requirement, so a goal satisfied at jump apex would cut
      // sprint+forward and drop the bot short of its landing — keep flying
      // the final node until grounded (water and climbables count as landed).
      let airborneHold = false
      if (path.length === 1 && Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1) {
        const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
        if (ent.onGround !== true && ent.isInWater !== true) {
          const feet = bot.blockAt(p) as { type: number } | null
          airborneHold = feet === null || (feet.type !== ladderId && feet.type !== vineId)
        }
      }
      if (!airborneHold && Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1) {
        // arrived at next point
        lastNodeTime = performance.now()
        lastNodeArrival = lastNodeTime
        path.shift()
        if (path.length === 0) { // done
          if (!dynamicGoal && stateGoal && (stateGoal.isEnd(p.floored()) || stateGoal.isEnd(p.floored().offset(0, 1, 0)))) {
            // A soft-replan solve may still be in flight (we kept walking the
            // old path and won the race) — cancel it, or its late result
            // would re-install a path with no goal set.
            cancelActiveSolve()
            bot.emit('goal_reached' as never, stateGoal as never)
            stateGoal = null
          }
          fullStop()
          return
        }
        // not done yet
        nextPoint = path[0]
        if (nextPoint.toBreak.length > 0 || nextPoint.toPlace.length > 0) {
          fullStop()
          return
        }
        dx = nextPoint.x - p.x
        dy = nextPoint.y - p.y
        dz = nextPoint.z - p.z
      }

      // Climb aid (improvement): with the next node directly ABOVE, the look
      // direction is degenerate — steer into an adjacent solid block instead
      // so 'forward' presses against it. Both vanilla and prismarine-physics
      // ascend climbables via horizontal collision, so without a wall to
      // press the bot would drift off a ladder/vine column.
      let climbing = false
      if (nextPoint.y > p.y + 0.1 && Math.abs(dx) < 0.2 && Math.abs(dz) < 0.2) {
        const feet = bot.blockAt(p) as BlockLike | null
        if (feet && (feet.type === ladderId || feet.type === vineId)) {
          const fx = Math.floor(p.x)
          const fy = Math.floor(p.y + 0.001)
          const fz = Math.floor(p.z)
          for (const [ox, oz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
            const nb = bot.blockAt(new Vec3(fx + ox, fy, fz + oz)) as BlockLike | null
            if (nb && nb.boundingBox === 'block') {
              dx = ox
              dz = oz
              climbing = true
              break
            }
          }
        }
      }

      // Sprint gate. A vanilla client cancels sprinting the moment a
      // collision deflects it (LocalPlayer.aiStep). Holding sprint against a
      // wall is not cosmetic: the server REFUSES every position a sprinting
      // client claims there and teleports the bot back — measured, back to
      // back, on the same block: walking along the wall covered 7.1 blocks
      // with zero corrections, sprinting covered 0.0 with 26. Because the bot
      // cannot move it never stops touching the wall, so this is a livelock,
      // and the futility timer cannot break it either: every correction
      // re-arms the forced-move grace.
      //
      // The probe is geometric, and looks a sprinting tick AHEAD as well as at
      // the body, because reacting to isCollidedHorizontally is two ticks too
      // late: by then the server already has the bot down as sprinting, and
      // the touching position it refuses has already been sent. Reacting
      // instead of predicting was worth 7 corrections on this route; looking
      // ahead took it to none.
      const slideLen = Math.hypot(dx, dz)
      const againstWall = !climbing && (geometry.nearWall(bot, SPRINT_WALL_MARGIN) ||
        (slideLen > 0.01 && geometry.playerCollides(
          bot, p.x + dx / slideLen * SPRINT_TICK, p.y, p.z + dz / slideLen * SPRINT_TICK)))
      // The one exception is a parkour jump already in flight: it was
      // approved on a rollout that held sprint the whole way, and a corner
      // nick mid-arc is exactly what that model tolerates. Cutting sprint
      // there would land the bot short of a node the planner routed through.
      const flyingParkour = (nextPoint as { parkour?: boolean }).parkour === true &&
        (bot.entity as { onGround?: boolean }).onGround !== true
      const maySprint = stateMovements.allowSprinting && (flyingParkour || !againstWall)

      // Wall-slide steering. Aiming straight at the next node when a wall is
      // in the way makes the physics clamp the move, and the server refuses a
      // clamped position that also slid along the wall: on the same corner,
      // aiming diagonally into it produced 25 corrections and 0.01 blocks of
      // travel, while aiming along the wall produced none and 7.1 blocks. A
      // player walks along the wall and turns at the corner; so do we.
      //
      // Walking steps only. A jump is flown on the heading the physics
      // rollout approved, and re-steering it mid-flight would land the bot
      // somewhere the planner never routed through.
      if (!climbing && (nextPoint as { parkour?: boolean }).parkour !== true &&
          dy <= STEP_UP_MIN && Math.hypot(dx, dz) <= WALK_STEP_REACH) {
        const sx = Math.sign(dx)
        const sz = Math.sign(dz)
        const xBlocked = sx !== 0 && geometry.playerCollides(bot, p.x + sx * SLIDE_PROBE, p.y, p.z)
        const zBlocked = sz !== 0 && geometry.playerCollides(bot, p.x, p.y, p.z + sz * SLIDE_PROBE)
        if (zBlocked && !xBlocked) dz = -sz * Math.abs(dx) * WALL_STANDOFF
        else if (xBlocked && !zBlocked) dx = -sx * Math.abs(dz) * WALL_STANDOFF
      }

      bot.look(Math.atan2(-dx, -dz), 0)
      bot.setControlState('forward', true)
      bot.setControlState('jump', false)
      // Sneak is only ever engaged by the corner-creep branch below; every
      // other branch (including the jump itself) must take off un-sneaked.
      bot.setControlState('sneak', false)

      if ((bot.entity as { isInWater?: boolean }).isInWater) {
        // prismarine-physics counts bubble columns as water. In a column the
        // drag does the vertical work — and in a DOWN column, holding jump
        // adds enough swim-up acceleration to stall the descent entirely.
        const feet = bot.blockAt(bot.entity.position) as { type: number } | null
        if (feet !== null && feet.type === bubbleColumnId) {
          bot.setControlState('jump', nextPoint.y > bot.entity.position.y + 0.25)
        } else {
          bot.setControlState('jump', true)
        }
        bot.setControlState('sprint', false)
      } else if (maySprint && physics.canStraightLine(path, true)) {
        bot.setControlState('jump', false)
        bot.setControlState('sprint', true)
      } else if (maySprint && physics.canSprintJump(path)) {
        bot.setControlState('jump', true)
        bot.setControlState('sprint', true)
      } else if (physics.canStraightLine(path)) {
        bot.setControlState('jump', false)
        bot.setControlState('sprint', false)
      } else if (physics.canWalkJump(path)) {
        bot.setControlState('jump', true)
        bot.setControlState('sprint', false)
      } else {
        // Improvement: creep to the takeoff CORNER before a standing parkour
        // jump. The sims above test jump-now from the CURRENT position; a
        // player walks to the lip first — for diagonal pillar hops all the
        // way to the corner overhang (the planner's per-axis flightNeeded
        // model, parkourEnvelope.ts, assumes the widest axis reaches
        // TAKEOFF_STAND). Sneak while creeping: the vanilla edge-guard makes
        // walking off the block physically impossible, so the overhang
        // stance is safe; the jump fires from another branch with sneak off.
        let creep = false
        if ((nextPoint as { parkour?: boolean }).parkour === true &&
            (bot.entity as { onGround?: boolean }).onGround === true) {
          const len = Math.sqrt(dx * dx + dz * dz)
          if (len > 0.01) {
            const px = p.x - (Math.floor(p.x) + 0.5)
            const pz = p.z - (Math.floor(p.z) + 0.5)
            const sCap = TAKEOFF_STAND * len / Math.max(Math.abs(dx), Math.abs(dz))
            creep = (px * dx + pz * dz) / len < sCap
          }
        }
        bot.setControlState('forward', creep)
        bot.setControlState('sneak', creep)
        bot.setControlState('sprint', false)
      }

      // check for futility
      if (performance.now() - lastNodeTime > 3500) {
        // should never take this long to go to the next node
        resetPath('stuck')
      }
    }

    bot.on('physicsTick', monitorMovement)
  }
}

/** The plugin instance for `bot.loadPlugin(pathfinder)` — upstream-identical usage. */
export const pathfinder = createPathfinder()
