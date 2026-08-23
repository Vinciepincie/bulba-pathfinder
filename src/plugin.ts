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

/**
 * Arrival box height while sprint-hopping. The hop apex is 1.25 above the
 * take-off, so upstream's |dy| < 1 would fly the bot over a node without
 * consuming it — and a node left behind turns the bot round.
 */
const HOP_ARRIVE_DY = 1.45

/**
 * How many nodes ahead the corner cut will look. One hop covers about four
 * blocks and the nodes are a block apart, so five is a whole arc's worth of
 * line to aim down — past that the scan costs more than the zig it saves,
 * and a cut that long is usually stopped by terrain anyway.
 */
const CUT_LOOKAHEAD = 5

/**
 * How far off the cut line a node may sit and still count as gone by. An
 * eight-direction path approximating a straight run leaves its nodes up to
 * half a block off the line the body actually takes; 0.9 covers that and a
 * hop's landing scatter without reaching sideways for a node on a branch the
 * bot never went down.
 */
const CUT_RETIRE = 0.9

/**
 * How far ABOVE a parkour node the body may be and still retire it in the
 * air. Roughly one tick of fall: at that height the landing is committed, so
 * holding the node buys nothing and costs the momentum the next move wants.
 */
const PARKOUR_LAND_DY = 0.5

/** Horizontal distance from a point to a segment, in the XZ plane. */
function pointToSegment (
  q: { x: number, z: number },
  a: { x: number, z: number },
  b: { x: number, z: number }
): number {
  const abx = b.x - a.x
  const abz = b.z - a.z
  const len2 = abx * abx + abz * abz
  if (len2 < 1e-9) return Math.hypot(q.x - a.x, q.z - a.z)
  let t = ((q.x - a.x) * abx + (q.z - a.z) * abz) / len2
  t = Math.max(0, Math.min(1, t))
  return Math.hypot(q.x - (a.x + abx * t), q.z - (a.z + abz * t))
}

/**
 * Air (of mineflayer's 0..20) below which the executor abandons the node and
 * surfaces. Vanilla's lung is 300 ticks and refills at 4 air-ticks per tick,
 * so a third of it is comfortably more than the time to swim up out of
 * anything the planner can legally route through.
 */
const AIR_RESERVE = 7

/** Movement below this, over a tick, is jitter rather than progress. */
const WEDGE_MOVE = 0.1

/**
 * Ticks of jitter-only movement before the executor stops believing its own
 * rollouts and starts trying escapes. Long enough to sit through a sharp turn
 * (bot.look interpolates) and a landing, short enough to save the run.
 */
const WEDGE_TICKS = 12

/**
 * Consecutive ticks one escape is given before the next is tried. An escape
 * that works stops being needed within a tick or two — the gates approve and
 * the recovery is not called again — and one that is not working has had its
 * chance.
 */
const ESCAPE_PATIENCE = 6

/**
 * Ticks an angled take-off is driven before the node is handed to the blind
 * escapes. A jump leaves the ground on the first tick, so anything past a
 * jump's worth of them means the angle was not the problem.
 */
const ANGLE_PATIENCE = 14

/**
 * Escapes, cheapest first. Cycled per node: an escape that did not help is
 * not the one tried next time the same node wedges. Each is simulated before
 * use (physics.canNudge), so none of them can be the fall it was avoiding.
 */
const ESCAPES: Array<{ back?: boolean, left?: boolean, right?: boolean, forward?: boolean, jump?: boolean }> = [
  { back: true },
  { left: true },
  { right: true }
]

const DEFAULT_OPTIONS: Required<Omit<PathfinderOptions, 'physicsFactory' | 'onNoPath'>> = {
  useWorkerThreads: true,
  workerEntryPath: '',
  maxSnapshotCells: 8_000_000,
  hitboxPrecisionFix: true
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
    /** Sprint-hop latched at take-off, re-pressed on each landing (allowSprintHop). */
    let hopHold = false
    /** The node the corner cut is steering at, held until reached or invalid. */
    let cutTarget: Move | null = null
    // Wedge recovery state — see the tick loop's recovery block.
    let wedgeAnchor: Vec3 | null = null
    let wedgeTicks = 0
    let recoverTicks = 0
    let recoverKind = 0
    let nextEscape = 0
    let recoverNode = ''
    /** Angle-solver state: the offset that lands the current node, if any. */
    let headingNode = ''
    let headingBias: number | null = null
    let headingSprint = false
    let biasFlight = false
    let biasTicks = 0
    /** Cell the parkour corner-creep started from (see the creep branch). */
    let creepCell: { x: number, z: number } | null = null
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
    // ── 1.21.x exact-boundary collision bug (hitboxPrecisionFix) ─────────
    //
    // prismarine-physics builds the body from playerHalfWidth 0.3 and
    // playerHeight 1.8, and every collision resolution leaves the box exactly
    // on a block boundary. On 1.21.x the server's own sweep computes exactly
    // 1.0 for such a move, calls it blocked, and teleports the client back —
    // silently, every tick, for as long as the bot keeps producing that
    // position. It is a physics bug, not a pathfinding one, but it presents
    // as a pathfinder that cannot climb: the bot welds itself to the face of
    // a one-block step and never gets off it.
    //
    // MEASURED on the arena's climb1 riser, walking jump, same controls, only
    // the starting clearance changed:
    //
    // | clearance | stock hitbox    | nudged hitbox |
    // | --------- | --------------- | ------------- |
    // | 0.01      | stuck, 19 corr. | CLIMBED, 0    |
    // | 0.15      | stuck, 17 corr. | CLIMBED, 0    |
    // | 0.20      | CLIMBED, 0      | CLIMBED, 0    |
    // | 0.30 spr. | stuck, 7 corr.  | CLIMBED, 0    |
    //
    // Nudging the dimensions by 1e-5 breaks the alignment and the whole class
    // goes away — 64 corrections across the probe instead of 174. Same fix as
    // PrismarineJS/mineflayer#3911 and mineflayer-pathfinder#364; applied
    // here because this package is what notices. Guarded on the exact stock
    // values so an application that already applies it is not doubled up.
    if (opts.hitboxPrecisionFix) {
      const ph = bot.physics as unknown as { playerHalfWidth?: number, playerHeight?: number }
      if (ph) {
        if (ph.playerHalfWidth === 0.3) ph.playerHalfWidth = 0.30001
        if (ph.playerHeight === 1.8) ph.playerHeight = 1.80001
      }
    }

    const workerHost = getSharedWorkerHost()
    if (opts.workerEntryPath) workerHost.setEntryPath(opts.workerEntryPath)
    // Pay the thread + wasm start-up while the bot is idle, not inside its
    // first goal. See SolverWorkerHost.prewarm.
    if (opts.useWorkerThreads) workerHost.prewarm()

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
          // The retry emits nothing, so nothing downstream clears these — and
          // with pathUpdated still latched from an earlier solve the tick loop
          // takes neither the replan branch nor the walk branch, and the goal
          // sits with an empty path and no solve in flight.
          pathUpdated = false
          lastSolveFrom = null
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
      if (final) {
        pathUpdated = true
        // A freshly installed COMPLETE path deserves a full futility window:
        // the bot has not failed to reach anything on it yet. A streamed
        // PARTIAL does not — partials arrive every tick-slice while a long
        // search runs, so refreshing on those meant the 3.5 s check could
        // never expire on exactly the routes that think hardest.
        lastNodeTime = performance.now()
        // Growth budget is per solve, not per goal: a solve that succeeded
        // must not leave the next one starting from a 1.8^5 box.
        if (raw.status === 'success') {
          growFactor = 1
          growAttempts = 0
        }
      }
      // lastNodeArrival is deliberately NOT touched here. It means "when a
      // node was last genuinely reached", it is what bounds how long server
      // corrections may excuse a lack of progress, and a path_update is not
      // an arrival.
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

    /**
     * Is there a hole just past this landing, in the direction the body is
     * travelling? A cell it can move into with nothing to stand on — a wall
     * is not a hole, because a wall stops the overrun instead of swallowing
     * it. Sampled a block and two blocks out, which is as far as the rest of
     * a descent carries.
     */
    function holeBeyond (node: Move, from: Vec3): boolean {
      const v = bot.entity.velocity
      let ux = v.x
      let uz = v.z
      let len = Math.hypot(ux, uz)
      if (len < 0.05) {
        ux = node.x - from.x
        uz = node.z - from.z
        len = Math.hypot(ux, uz)
        if (len < 1e-6) return false
      }
      ux /= len
      uz /= len
      for (const d of [1, 2]) {
        const x = node.x + ux * d
        const z = node.z + uz * d
        if (!geometry.playerCollides(bot, x, node.y, z) &&
            !geometry.playerCollides(bot, x, node.y - 0.55, z)) return true
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
      hopHold = false
      cutTarget = null
      clearWedge()
      bot.clearControlStates()

      // Force horizontal velocity to 0 (otherwise inertia can move us too far)
      bot.entity.velocity.x = 0
      bot.entity.velocity.z = 0

      const blockX = Math.floor(bot.entity.position.x) + 0.5
      const blockZ = Math.floor(bot.entity.position.z) + 0.5

      if (Math.abs(bot.entity.position.x - blockX) > 0.2) { bot.entity.position.x = blockX }
      if (Math.abs(bot.entity.position.z - blockZ) > 0.2) { bot.entity.position.z = blockZ }
    }

    // ── wedge recovery (improvement) ─────────────────────────────────────

    /** Consecutive ticks the body has not left `wedgeAnchor`. */
    function updateWedge (p: Vec3): number {
      if (wedgeAnchor === null || p.distanceTo(wedgeAnchor) > WEDGE_MOVE) {
        wedgeAnchor = p.clone()
        wedgeTicks = 0
      } else {
        wedgeTicks++
      }
      return wedgeTicks
    }

    function applyRecovery (kind: number): void {
      const c = ESCAPES[kind]
      bot.setControlState('forward', c.forward === true)
      bot.setControlState('back', c.back === true)
      bot.setControlState('left', c.left === true)
      bot.setControlState('right', c.right === true)
      bot.setControlState('jump', c.jump === true)
      bot.setControlState('sprint', false)
      bot.setControlState('sneak', false)
    }

    /**
     * Drive an escape for ONE tick, or report that none is available.
     *
     * One tick at a time, re-decided every tick, because the amount of room
     * a wedge needs is tiny and the cost of taking more is not. With the
     * hitbox-precision fix in place a step-up needs about 0.01 blocks of
     * clearance; a fixed four-tick back-off gives 0.35 and then has to walk
     * all of it back, which on the arena's climb2 staircase showed up as a
     * 0.9-block sideways wobble on every single step and 14 blocks of extra
     * ground against upstream on the identical plan. The escape stops the
     * moment the gates can work with what it made.
     *
     * The escape is only swapped after ESCAPE_PATIENCE consecutive ticks of
     * it not helping, so a recovery does not flicker between directions.
     */
    function driveRecovery (nextPoint: Move): boolean {
      if (recoverNode !== nextPoint.hash) {
        recoverNode = nextPoint.hash
        nextEscape = 0
        recoverTicks = 0
      }
      if (recoverTicks > 0 && recoverTicks < ESCAPE_PATIENCE && physics.canNudge(ESCAPES[recoverKind])) {
        recoverTicks++
        applyRecovery(recoverKind)
        return true
      }
      for (let i = 0; i < ESCAPES.length; i++) {
        const kind = (nextEscape + i) % ESCAPES.length
        if (!physics.canNudge(ESCAPES[kind])) continue
        recoverKind = kind
        nextEscape = (kind + 1) % ESCAPES.length
        recoverTicks = 1
        applyRecovery(kind)
        return true
      }
      // Boxed in on every side: nothing to do but let the futility timer
      // replan, and try a different escape first next time.
      nextEscape = (nextEscape + 1) % ESCAPES.length
      recoverTicks = 0
      wedgeTicks = 0
      return false
    }

    /**
     * The futility check, and it has to be reachable from every path through
     * the tick.
     *
     * Upstream's flat 3.5 s is a WALKING budget: swimming covers 2 blocks a
     * second against 4.3 walking, so a legitimate in-water crossing trips it
     * routinely — and the reset then releases jump and lets the bot sink for
     * the whole re-solve, which is worse than the stall it was called for.
     * Water gets the same distance, not the same time, and keeps swimming
     * while it thinks.
     *
     * Called from the end of the tick AND before every early return, because
     * the recovery branches return without falling through: an escape that
     * never works would otherwise hold the tick loop forever with the timer
     * that exists to break exactly that never once being read.
     */
    function futile (swimming: boolean): boolean {
      if (performance.now() - lastNodeTime <= (swimming ? 8000 : 3500)) return false
      resetPath('stuck', !swimming)
      return true
    }

    function clearWedge (): void {
      wedgeAnchor = null
      wedgeTicks = 0
      recoverTicks = 0
      recoverNode = ''
      nextEscape = 0
      headingNode = ''
      headingBias = null
      headingSprint = false
      biasFlight = false
      biasTicks = 0
    }

    /**
     * Turn to make the jump, before shuffling to make room for it.
     *
     * A player lining up an awkward hop does not stare at the block they
     * want — they turn a few degrees to clear whatever is in the way, and the
     * jump that was impossible head-on goes first try. That is the whole idea
     * behind Leg0shii's ParkourCalculatorMod, whose "angle solver" searches
     * yaw inputs for the ones that land a given jump; this is the same thing
     * at a hundredth the scope, run only on a node the executor is already
     * stuck on, and cached per node so the search is paid once.
     *
     * It is tried before the blind escapes because it is the only one that
     * costs nothing: a heading that lands the jump gets the bot to where it
     * was going, while a step back or a step sideways is ground given up and
     * walked again.
     */
    function driveAngledJump (nextPoint: Move, dx: number, dz: number): boolean {
      if (headingNode !== nextPoint.hash) {
        headingNode = nextPoint.hash
        biasFlight = false
        biasTicks = 0
        headingSprint = false
        headingBias = physics.bestHeading(path, true, false)
        if (headingBias === null && stateMovements.allowSprinting) {
          headingBias = physics.bestHeading(path, true, true)
          headingSprint = headingBias !== null
        }
        // Straight on already works, so this is not the problem — let the
        // escapes have it.
        if (headingBias === 0) headingBias = null
      }
      if (headingBias === null) return false
      // Bounded, because this branch returns without falling through to the
      // gates: an angle the server will not honour would otherwise be driven
      // forever. If the bot has not left the ground within a jump's worth of
      // ticks, the angle is not the answer — hand the node to the escapes.
      if (++biasTicks > ANGLE_PATIENCE) {
        headingBias = null
        return false
      }
      biasFlight = true
      bot.look(Math.atan2(-dx, -dz) + headingBias, 0)
      bot.setControlState('forward', true)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)
      bot.setControlState('sneak', false)
      bot.setControlState('jump', true)
      bot.setControlState('sprint', headingSprint)
      return true
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
      cutTarget = null
      stopDiggingIfNeeded()
      placing = false
      placingBlock = null
      pathUpdated = false
      // Growth budget is per solve. Sharing one across a whole goal meant a
      // route that grew the box once kept the enlarged box (and the spent
      // attempts) for every later replan, so the last few solves of a long
      // run had no growth left when they needed it.
      growFactor = 1
      growAttempts = 0
      clearWedge()
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

      if (path.length === 0) {
        // Release what only the tick loop ever presses. `resetPath(reason,
        // false)` deliberately keeps the controls so the bot coasts through a
        // soft replan, which is right for forward/sprint — but a `back` or a
        // strafe left over from a wedge escape would then be held for the
        // whole recompute and walk the bot away from its own path.
        // Do not sink while thinking. A bot with no path sets no controls at
        // all, and in water that is not "standing still" — it is 2 blocks a
        // second downward, for as long as the solve takes (up to the whole
        // think budget). Every re-solve therefore started from a worse place
        // than the last one, which is how a stall in a pool turns into a
        // drowning.
        if ((bot.entity as { isInWater?: boolean }).isInWater === true) {
          bot.setControlState('jump', true)
        }
        if (recoverTicks > 0) {
          clearWedge()
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
        }
        return
      }

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

      // SWIMMING, not merely wet (improvement). Upstream keys its water
      // branch on `isInWater`, which mineflayer sets for a body touching
      // water anywhere — so one waterlogged step, a puddle, or a splash at
      // head height cancels sprint and holds jump for the whole crossing, and
      // the bot bobs across a ford it could have run through. What actually
      // distinguishes swimming from wading is whether the HEAD cell is water:
      // there the bot has to swim up, and below it, it can walk.
      const head = bot.blockAt(bot.entity.position.offset(0, 1, 0)) as { type: number } | null
      const swimming = (bot.entity as { isInWater?: boolean }).isInWater === true &&
        head !== null && (head.type === waterType || head.type === bubbleColumnId)

      let dx = nextPoint.x - p.x
      let dy = nextPoint.y - p.y
      let dz = nextPoint.z - p.z
      // Arrival. Two divergences from upstream's single shift, both forced by
      // the sprint-hop gait: a hop crosses a node at up to 1.25 blocks above
      // it — outside the |dy| < 1 box — and covers more than one node's worth
      // of ground per tick. A node left behind unconsumed is worse than a
      // wasted one: the executor turns the bot round and walks it back to
      // somewhere it has already flown over. So while hopping the box is as
      // tall as the hop, and EVERY node the body has passed is consumed, not
      // just the first. Walking, `arriveDy` is upstream's 1 and the loop can
      // only ever run once (two nodes inside one 0.7-wide box and within a
      // block of each other would have to be the same cell).
      const arriveDy = hopHold ? HOP_ARRIVE_DY : 1
      for (;;) {
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
        // A PARKOUR node is landed on, never flown over (improvement).
        //
        // The arrival box has no ground requirement, so a jump that passes
        // over its node on the way down satisfies it in mid-air — and the
        // executor then starts flying at the node AFTER it, which on the far
        // side of a gap means steering into the gap. On the arena's basic1 a
        // 4-block drop onto a 2-cell shelf was retired 0.9 blocks above the
        // shelf, and the bot spent the rest of its fall thrusting toward a
        // node across the chasm beyond it: 40 blocks down, one run in eight,
        // on either gait. `landsThere` already refuses to AUTHORISE a jump
        // that only passes through its node; this is the same rule on the
        // other side of the flight.
        //
        // Held back, the node stays the thing being steered at, so the bot
        // spends the descent aiming at the shelf — which is also the fastest
        // way to be standing on it.
        // The threshold is a LANDING's worth of height, not zero. Holding all
        // the way to touchdown also works and is safe, but it costs about a
        // tick per jump across the route book — while the node is held the
        // bot keeps steering at it, which means steering backwards once it is
        // underneath, and that sheds the momentum the next move wants. A body
        // within half a block of the node is committed to it; a body 0.9 up,
        // which is where the arena retired it, is not.
        //
        // And only where flying past would actually cost something. A landing
        // with more ground beyond it is one the bot can overrun harmlessly,
        // which is most of them — holding those too is safe but costs about a
        // tick a jump across the route book (basic2 +0.8 s, 15 jumps). So the
        // hold is spent on the case it was written for: a shelf with a hole
        // after it.
        if ((nextPoint as { parkour?: boolean }).parkour === true &&
            p.y - nextPoint.y > PARKOUR_LAND_DY) {
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
          if (ent.onGround !== true && ent.isInWater !== true && holeBeyond(nextPoint, p)) break
        }
        if (airborneHold) break
        // Inside the box, or simply GONE BY. The corner cut does not steer
        // through every node centre by design, so a node the body has passed
        // is done even though it was never inside its 0.7-wide box; left in
        // place it turns the bot round to collect it, and the bot then cuts
        // forward again — the vibrating-on-flat-ground shuffle.
        //
        // "Passed" is measured against the path's own next leg, not against
        // the body's velocity: a body being nudged backwards by the wedge
        // recovery has velocity pointing away from a node it has NOT passed,
        // and would retire it. Projecting onto the leg cannot be fooled that
        // way — it only reads true once the body is genuinely on the far side
        // of the node. Never the last node: the goal is always arrived at.
        let passed = false
        if (stateMovements.allowCornerCut && !swimming && path.length > 1 && Math.abs(dy) < arriveDy &&
            (nextPoint as { parkour?: boolean }).parkour !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0) {
          const leg = path[1]
          const abx = leg.x - nextPoint.x
          const abz = leg.z - nextPoint.z
          passed = ((p.x - nextPoint.x) * abx + (p.z - nextPoint.z) * abz) > 0 &&
            Math.hypot(dx, dz) <= CUT_RETIRE
        }
        if (!passed && !(Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < arriveDy)) break

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


      // Corner cut (improvement): walk the line, not the staircase.
      //
      // The planner routes cell centre to cell centre over eight directions,
      // so a run a few degrees off a cardinal comes back as an alternating
      // zig-zag — and a follower that steers at each centre in turn walks
      // every zig, and swings its heading ±45° at every one. Sprinting that
      // is merely wasteful; HOPPING it is expensive, because each arc is
      // committed at take-off and the yaw swing happens with only air control
      // to answer it. Measured on the arena's simple1, same plan and same
      // gait: 88.8 blocks walked for a 74.6-block line.
      //
      // So the executor pulls the string: it STEERS at the furthest node it
      // can reach in a straight line the body fits down — full hitbox sampled
      // every quarter block, floor under every sample — and the nodes in
      // between retire as the body goes by them (see the arrival loop). Only
      // across flat plain walking, and only what geometry can vouch for: a
      // rise, a parkour node, anything to break or place, or a profile with
      // exclusion zones stops the scan, because those carry planner reasoning
      // a swept box cannot re-derive.
      //
      // The nodes are STEERED PAST, not spliced out. Dropping them ahead of
      // time leaves the body behind its own path[0], and upstream's lagback
      // recovery reads exactly that — `isPositionNearPath` compares against
      // the first node directly — so a routine sub-block correction stopped
      // splicing and forced a full replan instead.
      // Kept pointing at path[0] whatever the cut does below. The angle
      // solver's offset is measured from the heading to path[0], so adding it
      // to a heading aimed several nodes further on would fly the jump at
      // something nobody solved for.
      const nodeDx = dx
      const nodeDz = dz
      const mayCut = stateMovements.allowCornerCut && !swimming && path.length > 1 &&
        stateMovements.exclusionAreasStep.length === 0 &&
        (nextPoint as { parkour?: boolean }).parkour !== true &&
        nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0 &&
        Math.abs(nextPoint.y - p.y) <= HOP_ARRIVE_DY
      /** Is this node still a legal thing to be steering at from here? */
      const cuttableTo = (n: Move & { parkour?: boolean }, upTo: number): boolean => {
        if (n.parkour === true || n.toBreak.length > 0 || n.toPlace.length > 0) return false
        if (Math.abs(n.y - nextPoint.y) > 0.1) return false
        // Only cut a corner whose skipped nodes can still be COLLECTED. They
        // retire by being gone by rather than by being stood on, and that
        // test has a reach; a node further off the new line than that reach
        // is never reached and never retired, so the bot turns round for it.
        // Cutting without this check made every route LONGER and added
        // replans — simple2 went from 105 blocks and 15.7 s to 111 and 19.9.
        for (let j = 1; j < upTo; j++) {
          if (pointToSegment(path[j], p, n) > CUT_RETIRE) return false
        }
        return geometry.walkableLine(bot, p.x, p.z, n.x, n.z, nextPoint.y, stateMovements.blocksToAvoid)
      }
      if (!mayCut) cutTarget = null
      else {
        // COMMIT to a target. Re-picking the furthest legal node every tick
        // makes the heading flick between path[1] and path[4] as the line
        // scan flickers on and off near terrain, which is a wobble on the
        // screen and a waste of momentum. The target is kept until the body
        // reaches it (it retires like any other node) or it stops being
        // legal, and only then is a new one chosen.
        if (cutTarget !== null) {
          const at = path.indexOf(cutTarget)
          if (at < 1 || !cuttableTo(cutTarget as Move & { parkour?: boolean }, at)) cutTarget = null
        }
        if (cutTarget === null) {
          for (let k = 1; k < Math.min(CUT_LOOKAHEAD, path.length); k++) {
            if (!cuttableTo(path[k] as Move & { parkour?: boolean }, k)) break
            cutTarget = path[k]
          }
        }
        if (cutTarget !== null) {
          // Steer at it; the gates below still reason about path[0], which is
          // on this same line and closer, so nothing is authorised that the
          // node-by-node follower would not have authorised.
          dx = cutTarget.x - p.x
          dz = cutTarget.z - p.z
        }
      }

      // Is the bot getting anywhere? Two triggers, because waiting is only
      // cheap when it is rare. The generic one is slow on purpose (a sharp
      // turn or a landing can hold the body still for a few ticks and neither
      // is a wedge). The fast one asks the physics whether walking at the node
      // from here gets anywhere at all: on a staircase the answer is no the
      // instant the bot lands against the next riser, and paying the full 12
      // ticks there cost ~1.4 s PER STEP on the arena's 46-block climb.
      const stillTicks = updateWedge(p)
      const wedged = stillTicks > WEDGE_TICKS ||
        (stillTicks >= 1 && (bot.entity as { onGround?: boolean }).onGround === true &&
         physics.isGrinding(path))

      // Diagonal squeeze: go round the corner that is actually open
      // (improvement).
      //
      // A diagonal move is priced on the CHEAPER of its two corners — that is
      // upstream's rule and ours by parity — which is honest only if the
      // walker goes AROUND that corner. Aiming at the node centre instead
      // cuts across both, and a 0.6-wide body clips the blocked one: it never
      // arrives, the physics rollouts all refuse (rightly), and the executor
      // stands there. Measured on the arena's climb1 tower: 23 s motionless
      // one cell from the node, with the open corner beside it the whole
      // time, on a move the planner was right to think was walkable.
      //
      // Inserting the open corner turns the squeeze into two ordinary moves —
      // exactly the two the planner priced. It cannot recurse: the waypoint
      // is cardinal from here, and the node is cardinal from the waypoint.
      //
      // ONLY once the bot is actually stuck on the node, though. Most tight
      // diagonals are flown straight through perfectly well — the body clips
      // the corner, the physics resolves it per axis and the bot slides on —
      // and routing every one of them through a waypoint turns a straight
      // climb into a staircase of 1-block sidesteps. Measured on the arena's
      // climb2: a 0.9-block x zig-zag on every step, 127 blocks walked
      // against upstream's 105 on the same 62-node plan, and 4.4 s lost.
      let squeezed = false
      if (wedged && (nextPoint as { parkour?: boolean }).parkour !== true &&
          nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0) {
        const bx = Math.floor(p.x)
        const by = Math.floor(p.y + 0.001)
        const bz = Math.floor(p.z)
        const tx = Math.floor(nextPoint.x)
        const tz = Math.floor(nextPoint.z)
        if (Math.abs(tx - bx) === 1 && Math.abs(tz - bz) === 1 && Math.abs(dy) <= 1.3) {
          const yTest = Math.max(by, Math.floor(nextPoint.y + 0.001))
          const openA = !geometry.playerCollides(bot, tx + 0.5, yTest, bz + 0.5)
          const openB = !geometry.playerCollides(bot, bx + 0.5, yTest, tz + 0.5)
          if (openA !== openB) {
            const wx = openA ? tx : bx
            const wz = openA ? bz : tz
            if (geometry.isStandable(bot, new Vec3(wx, by, wz))) {
              let np = getPositionOnTopOf(bot.blockAt(new Vec3(wx, by, wz)) as BlockLike | null)
              if (np === null) np = getPositionOnTopOf(bot.blockAt(new Vec3(wx, by - 1, wz)) as BlockLike | null)
              const waypoint = new Move(wx, by, wz, nextPoint.remainingBlocks, 1)
              if (np !== null) {
                waypoint.x = np.x
                waypoint.y = np.y
                waypoint.z = np.z
              } else {
                waypoint.x = wx + 0.5
                waypoint.y = by
                waypoint.z = wz + 0.5
              }
              squeezed = true
              clearWedge() // the waypoint IS the escape; do not also nudge
              path.unshift(waypoint)
              nextPoint = waypoint
              dx = nextPoint.x - p.x
              dy = nextPoint.y - p.y
              dz = nextPoint.z - p.z
            }
          }
        }
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
        // The standoff is a fraction of the ALONG-wall component, and it
        // REPLACES the into-wall one. With nothing to slide along — a step
        // square-on to the face — that turns a heading of (0.02, 1.0) into
        // (0.02, -0.005) and hands Math.atan2 a direction made of float
        // noise, so the bot faces sideways at a wall instead of at its node.
        // There is no slide to steer there; leave the heading alone and let
        // the wedge recovery below deal with it if nothing moves.
        if (zBlocked && !xBlocked && Math.abs(dx) >= 0.15) dz = -sz * Math.abs(dx) * WALL_STANDOFF
        else if (xBlocked && !zBlocked && Math.abs(dz) >= 0.15) dx = -sx * Math.abs(dz) * WALL_STANDOFF
      }

      // Air, before anything else can spend it. Vanilla gives 300 ticks of it
      // and then 2 HP a second, and the planner has no idea — it will happily
      // route 40 blocks of submerged corridor, which at swim speed is 20 s
      // against a 15 s lung. Nothing downstream can rescue that, so the
      // executor watches its own breath: below a third of it, the goal stops
      // mattering until the bot has surfaced. Straight up is always the
      // fastest way out (3.5 blocks/s against 2 swimming sideways), and the
      // futility timer is held off because surfacing IS progress even though
      // no node is being reached. It outranks the wedge recovery: a bot both
      // stuck and out of air needs the air first.
      const air = (bot as { oxygenLevel?: number }).oxygenLevel
      if (swimming && air !== undefined && air <= AIR_RESERVE) {
        bot.clearControlStates()
        bot.setControlState('jump', true)
        lastNodeTime = performance.now()
        return
      }

      // ── when the plan and the world disagree ─────────────────────────────
      //
      // Every rollout above answers "does this work?" against
      // prismarine-physics, and prismarine-physics is not the authority. When
      // the two disagree the bot stands still holding a plan it believes in,
      // the futility timer replans, the identical plan comes back, and it
      // stands still again: measured on the arena's climb1 staircase, 55 s in
      // one spot across sixteen `stuck` resets, one cell from a node it was
      // right to want.
      //
      // So the recovery triggers on the SYMPTOM — the body has not moved —
      // rather than on any prediction about why, and then works down a ladder
      // of increasingly expensive answers: turn to make the jump, go round
      // the open corner, make room. Each is simulated before it is used, and
      // none of them run while the bot is making progress.
      //
      // A jump taken at an angle is flown at that angle: re-homing on the
      // node mid-arc would undo the very thing that made it possible.
      if (biasFlight && (bot.entity as { onGround?: boolean }).onGround !== true &&
          headingBias !== null) {
        bot.look(Math.atan2(-nodeDx, -nodeDz) + headingBias, 0)
        bot.setControlState('forward', true)
        bot.setControlState('sprint', headingSprint)
        futile(swimming)
        return
      }
      if (biasFlight && (bot.entity as { onGround?: boolean }).onGround === true) biasFlight = false

      if (!squeezed && wedged) {
        if (driveAngledJump(nextPoint, nodeDx, nodeDz)) { futile(swimming); return }
        if (driveRecovery(nextPoint)) { futile(swimming); return }
      }

      bot.look(Math.atan2(-dx, -dz), 0)
      bot.setControlState('forward', true)
      bot.setControlState('jump', false)
      // Sneak is only ever engaged by the corner-creep branch below; every
      // other branch (including the jump itself) must take off un-sneaked.
      bot.setControlState('sneak', false)
      // Back/strafe are only ever engaged by the wedge recovery.
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)


      if (swimming) {
        // Swim to the level the plan is on, and STAY there.
        //
        // Upstream holds jump for as long as the body is wet, which is a
        // sensible way to not drown and a terrible way to follow a path: the
        // search has no vertical water move at all (moveUp refuses a liquid
        // feet cell, moveDown refuses to go under), so every water node it
        // emits sits at one planned level — and an executor that swims up
        // unconditionally climbs out of its own plan. In a waterfall it rides
        // the column: on the arena's basic1 the bot floated up a stream it
        // was supposed to cross, replanned from higher each time, and never
        // arrived.
        //
        // The rule is the one already used for bubble columns, with the
        // threshold at the current level rather than above it: press up while
        // the node is at or above the body, let buoyancy go when the body has
        // drifted above it. That holds a surface swim level, climbs to a node
        // above, and sinks toward one below, with no separate state.
        hopHold = false
        const feet = bot.blockAt(bot.entity.position) as { type: number } | null
        const inColumn = feet !== null && feet.type === bubbleColumnId
        // A down-column does the vertical work itself, and swim-up in one is
        // strong enough to stall the descent outright — so a column only gets
        // help when the node is clearly above.
        bot.setControlState('jump', nextPoint.y > bot.entity.position.y + (inColumn ? 0.25 : -0.1))
        bot.setControlState('sprint', false)
      } else if (maySprint && physics.canStraightLine(path, true)) {
        // Sprint-hop (improvement, allowSprintHop): the plain sprint upstream
        // uses here is the SLOWEST way a bot with a jump key crosses open
        // ground — 5.56 blocks/s against 6.97 hopping, measured on the arena
        // server, because the jump preserves the sprint boost ground friction
        // eats. The decision is re-taken on every take-off (and only there —
        // it is the one tick it can be acted on), from a rollout of both
        // gaits down this same path: the hop has to actually get further,
        // without losing height, or the bot keeps its feet.
        if ((bot.entity as { onGround?: boolean }).onGround === true) {
          hopHold = stateMovements.allowSprintHop &&
            physics.sprintHopBetter(path, stateMovements.allowLowCeilingHop)
        }
        // PRESS on the landing tick, never hold. prismarine-physics charges a
        // held jump key a 10-tick re-jump cooldown (`autojumpCooldown`) and
        // clears it the instant the key comes up, so a bonked arc — one that
        // lands after 5 ticks because the ceiling is low — spends the rest of
        // the cooldown on the ground, bleeding the sprint boost into friction.
        // Measured on flat stone with the same physics: under a 2-block roof,
        // sprint 5.59 b/s, hold-jump 6.50, press-on-landing 9.68 — faster than
        // bunny-hopping under open sky. Where the arc already outlasts the
        // cooldown the two cadences are identical tick for tick (7.05 both, 34
        // jumps each), so this is never the slower choice and is not optional.
        bot.setControlState('jump', hopHold && (bot.entity as { onGround?: boolean }).onGround === true)
        bot.setControlState('sprint', true)
      } else if (maySprint && physics.canSprintJump(path)) {
        hopHold = false
        bot.setControlState('jump', true)
        bot.setControlState('sprint', true)
      } else if (physics.canStraightLine(path)) {
        hopHold = false
        bot.setControlState('jump', false)
        bot.setControlState('sprint', false)
      } else if (physics.canWalkJump(path)) {
        hopHold = false
        bot.setControlState('jump', true)
        bot.setControlState('sprint', false)
      } else {
        hopHold = false
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
            // Measured against the cell the creep STARTED in, not against
            // whatever cell the body is in now. The overhang stance puts the
            // hitbox centre past the lip by design, so once it crosses the
            // boundary `Math.floor(p)` names the NEXT cell, the offset flips
            // sign, and the bot reads as "not crept far enough" again — it
            // walks off the edge it was carefully standing on.
            creepCell ??= { x: Math.floor(p.x), z: Math.floor(p.z) }
            const px = p.x - (creepCell.x + 0.5)
            const pz = p.z - (creepCell.z + 0.5)
            const sCap = TAKEOFF_STAND * len / Math.max(Math.abs(dx), Math.abs(dz))
            creep = (px * dx + pz * dz) / len < sCap
          }
        }
        if (!creep) creepCell = null
        // A body already in the air keeps flying the heading it took off on.
        // Whatever the gates think from here, releasing forward mid-flight
        // throws away the air control the jump was approved with and lands
        // the bot short of a node the planner routed through — and there is
        // nothing else this branch could usefully do about a bot that is not
        // touching the ground.
        const inFlight = (bot.entity as { onGround?: boolean }).onGround !== true && !swimming

        // Riser standoff: take the pace back a step needs, and ONLY when the
        // step actually needs it.
        //
        // The server refuses every position where the body touches a face
        // while the feet are below its top, so a jump has to start far enough
        // back that the arc clears the lip first. MEASURED on one arena
        // riser, identical controls, only the starting clearance changed:
        // 0.00-0.15 gave 17-19 corrections and never climbed; 0.20 climbed
        // with none. The grind guard in physics.ts already draws that line —
        // it is why every gate above has just refused — so the trigger here
        // is simply "the gates said no to a step up", not a second clearance
        // constant of its own.
        //
        // That distinction is the whole difference between a bot that climbs
        // and one that looks like it is glitching. A walking jump started at
        // 0.2 lands on the next step at 0.2, so a clean staircase chains with
        // no correction at all; an explicit threshold set anywhere above that
        // landing clearance fires on EVERY step instead, and the bot visibly
        // shuffles backwards before each one.
        const stepBack = !creep && !inFlight && !climbing && !swimming &&
          dy > STEP_UP_MIN && (nextPoint as { parkour?: boolean }).parkour !== true &&
          physics.canNudge({ back: true }, 3)

        bot.setControlState('forward', creep || inFlight)
        bot.setControlState('back', stepBack)
        bot.setControlState('sneak', creep)
        bot.setControlState('sprint', flyingParkour)
      }

      futile(swimming)
    }

    bot.on('physicsTick', monitorMovement)
  }
}

/** The plugin instance for `bot.loadPlugin(pathfinder)` — upstream-identical usage. */
export const pathfinder = createPathfinder()
