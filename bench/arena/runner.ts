// One racer: a bot, a pathfinder implementation, and the instrumentation
// that turns a walk into comparable numbers.
//
// The custom engine is loaded from its published entry point (`dist/`), not
// from `src/`, so the benchmark exercises what production runs — worker
// thread, wasm core and all. Running the TypeScript sources directly would
// silently fall back to main-thread solving and flatter the result.
import { createRequire } from 'node:module'
import { performance } from 'node:perf_hooks'
import type { Bot } from 'mineflayer'
import { Vec3 } from 'vec3'

const require2 = createRequire(import.meta.url)

/** No progress for this long counts as stuck, and gets snapshotted. */
export const STUCK_MS = 2500

interface BlockLike {
  name: string
  boundingBox: string
  shapes?: number[][]
  getProperties?: () => Record<string, unknown>
  _properties?: Record<string, unknown>
}

/** The terrain a failure happened on. Air is implied by absence. */
export interface WorldProbe {
  blocks: Array<[number, number, number, string]>
  /** Read through an unloaded chunk — absence here means "unknown", not air. */
  unloaded: Array<[number, number, number]>
  focus: Array<{
    pos: [number, number, number]
    label: string
    name: string
    boundingBox: string
    properties: Record<string, unknown>
    shapes: number[][]
  }>
}

function readProperties (b: BlockLike | null): Record<string, unknown> {
  if (b === null) return {}
  try {
    return typeof b.getProperties === 'function' ? b.getProperties() : (b._properties ?? {})
  } catch {
    return {}
  }
}

/**
 * `bulba` runs the reference JavaScript solver on the main thread;
 * `bulba-wasm` runs the production path — the Rust core in a worker thread.
 * Both are the same algorithm and are pinned bit-identical by the package's
 * differential suite, so racing them apart measures the engine room, not the
 * search.
 *
 * `bulba-nohop` is `bulba` with `allowSprintHop` off and nothing else. The
 * sprint-hop gait is worth ~25% of ground speed in isolation but costs
 * airtime on stepped terrain, so it is raced against its own twin rather than
 * argued about: same route, same tick, same server.
 */
export type Impl = 'upstream' | 'bulba' | 'bulba-wasm' | 'bulba-nohop'

export type Outcome =
  | 'arrived'
  | 'no-path'
  | 'think-timeout'
  | 'timeout'
  | 'died'
  | 'gave-up'
  | 'stopped'
  | 'error'

/**
 * A bot that stopped making progress, caught in the act: where it was, and
 * what the executor was still trying to walk. The path nodes are the useful
 * half — "stalled at x y z" says nothing on its own, "stalled at x y z with a
 * 3-block parkour jump as the next node" is a bug report.
 */
export interface StuckSnapshot {
  atMs: number
  stalledMs: number
  pos: [number, number, number]
  /** Nodes still queued in the live path, nearest first. */
  ahead: Array<{ x: number, y: number, z: number, parkour: boolean, cost: number }>
  remaining: number
}

export interface RunResult {
  impl: Impl
  routeId: string
  outcome: Outcome
  /** What `goto()` claimed, before checking where the bot actually is. */
  promiseOutcome: Outcome
  error?: string
  /** Wall clock from "go" to arrival/failure. The headline number. */
  wallMs: number
  /** Time the first complete solve took, and the sum over every (re)solve. */
  firstSolveMs: number | null
  /**
   * Wall clock from launch to the bot leaving the start block — the
   * engine-agnostic `firstSolveMs`. The engines time different windows
   * (upstream from search start, the wasm core from inside the worker), so
   * only this one covers everything that happens before the bot moves. It is
   * NOT additive with firstSolveMs: the solve happens inside this window.
   *
   * ⚠ Reading the two together, `bulba-wasm` looks contradictory — the better
   * solve number and the worse first move. A race is a COLD benchmark: one
   * solve per fresh process, and the worker path pays a one-time ~140 ms
   * (worker spawn, module load in the worker, wasm instantiate, first LUT and
   * parkour-table upload) that main-thread JS never pays. That cost lands
   * here and in wallMs but not in firstSolveMs, which the core measures from
   * inside the worker. Warm — which is what a long-lived bot and `npm run
   * bench` both see — it inverts: ~1.2 ms for the worker against ~1.6 ms for
   * main-thread JS on a 107-node solve, and far wider on big searches.
   */
  firstMoveMs: number | null
  solveMsTotal: number
  solves: number
  visitedNodes: number
  generatedNodes: number
  firstPathNodes: number | null
  firstPathCost: number | null
  replans: number
  travelled: number
  jumps: number
  /**
   * Server position corrections. A bot that scrapes along geometry the server
   * disagrees about gets teleported back, and a bot that does it in a way the
   * server refuses outright gets teleported back EVERY tick — the difference
   * between "a bit of rubber-banding" and a livelock is a number, so measure
   * it. Upstream is the control: whatever it scores here is the terrain's
   * fault, not the engine's.
   */
  lagbacks: number
  damage: number
  deaths: number
  worstStallMs: number
  /** Every stall past `STUCK_MS`, in order. The auto-debug reads these. */
  stalls: StuckSnapshot[]
  endDistance: number
  /** Where the run actually ended (the death spot, if it died there). */
  endPos: [number, number, number]
  /** Wall clock (ms) at which this racer actually called goto(). */
  startedAt: number
  /** Diagnostics for a run that did not arrive: what the engine kept saying. */
  statuses: Record<string, number>
  resetReasons: Record<string, number>
  gameMode: string
  /** Vanilla death message, filled in by the referee ("PF_Bulba drowned"). */
  deathCause?: string
  /**
   * Forensics for a run that stops somewhere it should not have. The first
   * complete path is snapshotted at emit time (the engines hand back the LIVE
   * array and drain it as the bot walks), and `drainedAt` records where the
   * bot actually was when that array hit zero. If the bot is far from the
   * path's last node at that moment, nodes were consumed without being
   * travelled — which is a very different bug from failing a jump.
   */
  firstPath?: Array<[number, number, number]>
  /** Indices into `firstPath` that the engine flagged as parkour jumps. */
  firstPathParkour?: number[]
  drainedAt?: [number, number, number]
  drainedAfterMs?: number
  /**
   * `[ms, x, y, z, nodesLeft]` every ~200 ms. A run that neither arrives nor
   * stalls is going somewhere — this is what tells the difference between a
   * bot wedged against a wall and one walking a loop.
   */
  trace?: Array<[number, number, number, number, number]>
  /** `[ms, status, pathLen]` per path_update, in order. The counts alone hide
   *  whether a partial arrived after the success and replaced the path. */
  updates?: Array<[number, string, number]>
}

export interface MovementProfile {
  extendedParkour: boolean
  maxDropDown: number
  /** Override for Movements.parkourSafetyMargin (undefined = the package default). */
  safetyMargin?: number
  thinkTimeout: number
}

interface PathUpdate {
  status: string
  cost?: number
  time?: number
  visitedNodes?: number
  generatedNodes?: number
  path?: unknown[]
}

/** Load a pathfinder plugin + goals for one implementation. */
export async function loadImpl (impl: Impl): Promise<{
  plugin: (bot: Bot) => void
  Movements: new (bot: Bot) => Record<string, unknown>
  goals: Record<string, new (...args: number[]) => unknown>
}> {
  if (impl === 'upstream') {
    const mod = require2('mineflayer-pathfinder')
    return { plugin: mod.pathfinder, Movements: mod.Movements, goals: mod.goals }
  }
  let mod: Record<string, unknown>
  try {
    mod = await import('@bulba/pathfinder') as Record<string, unknown>
  } catch (error) {
    throw new Error(
      `could not load the built @bulba/pathfinder — run "npm run build" first (${(error as Error).message})`
    )
  }
  // The worker carries the wasm core; without it the plugin solves on the
  // main thread with the JavaScript reference implementation.
  const createPathfinder = mod.createPathfinder as (o: { useWorkerThreads: boolean }) => (bot: Bot) => void
  return {
    plugin: createPathfinder({ useWorkerThreads: impl === 'bulba-wasm' }),
    Movements: mod.Movements,
    goals: mod.goals
  } as never
}

/**
 * Identical movement rules for both engines, so the comparison is engine vs
 * engine. Digging and block placement stay off: this benchmark measures
 * traversal of the world as it is.
 */
export function applyProfile (
  movements: Record<string, unknown>,
  impl: Impl,
  profile: MovementProfile
): void {
  movements.canDig = false
  movements.allow1by1towers = false
  movements.scafoldingBlocks = []
  movements.allowFreeMotion = false
  movements.allowParkour = true
  movements.allowSprinting = true
  movements.allowEntityDetection = true
  movements.dontCreateFlow = true
  movements.infiniteLiquidDropdownDistance = true
  movements.maxDropDown = profile.maxDropDown
  // The extended repertoire and the sprint-hop gait are the things under
  // test; upstream has no such flags, so this is where the engines are
  // allowed to differ. `--parity` turns both off for an apples-to-apples run.
  if (impl !== 'upstream') {
    movements.allowParkourExtended = profile.extendedParkour
    if (profile.safetyMargin !== undefined) movements.parkourSafetyMargin = profile.safetyMargin
    // `bulba-nohop` is `bulba` with the sprint-hop gait off and nothing else.
    // Racing them side by side is how the gait is measured — same terrain,
    // same tick, same server hitch — instead of across two runs.
    movements.allowSprintHop = profile.extendedParkour && impl !== 'bulba-nohop'
    // Rides with the gait: it only changes how far ahead a ceiling drop
    // vetoes a take-off, so a route with open sky over it cannot tell the
    // difference. 2b2t spawn is mostly open sky — the gait's own numbers come
    // from the offline corridor measurement, and the route book's job here is
    // to show it costs nothing where it does not apply.
    movements.allowLowCeilingHop = movements.allowSprintHop === true
    // Bisect switch: ARENA_NO_CUT=1 races the same build with node-by-node
    // following, which is the only honest way to price the corner cut.
    if (process.env.ARENA_NO_CUT === '1') movements.allowCornerCut = false
  }
}

export class Racer {
  readonly bot: Bot
  readonly impl: Impl
  private readonly goalsNs: Record<string, new (...args: number[]) => unknown>
  private readonly MovementsCtor: new (bot: Bot) => Record<string, unknown>
  private readonly profile: MovementProfile

  private constructor (bot: Bot, impl: Impl, loaded: Awaited<ReturnType<typeof loadImpl>>, profile: MovementProfile) {
    this.bot = bot
    this.impl = impl
    this.goalsNs = loaded.goals
    this.MovementsCtor = loaded.Movements
    this.profile = profile
  }

  static async attach (bot: Bot, impl: Impl, profile: MovementProfile): Promise<Racer> {
    const loaded = await loadImpl(impl)
    bot.loadPlugin(loaded.plugin as never)
      // The 1.21.x exact-boundary collision bug is a mineflayer PHYSICS bug,
    // not a pathfinding one: a body resting exactly on a block boundary makes
    // the server refuse the move and teleport the client back. @bulba/
    // pathfinder applies the nudge itself on inject, so applying it here too
    // for UPSTREAM keeps the race measuring pathfinding rather than handing
    // us a client fix upstream never got. Idempotent — the package's own fix
    // is guarded on the stock values.
    const ph = (bot as unknown as { physics: { playerHalfWidth: number, playerHeight: number } }).physics
    if (ph.playerHalfWidth === 0.3) ph.playerHalfWidth = 0.30001
    if (ph.playerHeight === 1.8) ph.playerHeight = 1.80001

    const racer = new Racer(bot, impl, loaded, profile)
    const pf = (bot as unknown as { pathfinder: Record<string, unknown> }).pathfinder
    pf.thinkTimeout = profile.thinkTimeout
    const movements = new loaded.Movements(bot)
    applyProfile(movements, impl, profile)
    ;(pf.setMovements as (m: unknown) => void)(movements)
    return racer
  }

  private get pf (): Record<string, unknown> {
    return (this.bot as unknown as { pathfinder: Record<string, unknown> }).pathfinder
  }

  /**
   * Solve only — no movement, the head-to-head planning numbers.
   *
   * Drives `getPathFromTo` to completion rather than calling `getPathTo`:
   * upstream's `getPathTo` returns after a single 40 ms slice, so on any real
   * route it reports `partial` and times a fraction of the work. Both engines
   * run their JavaScript solver on this path, which makes this an
   * algorithm-vs-algorithm number; the race phase is where the custom engine's
   * worker thread and wasm core actually run.
   */
  async solveOnly (end: [number, number, number], repeats: number): Promise<{
    ms: number[]
    visited: number
    cost: number | null
    nodes: number | null
    status: string
  }> {
    const Goal = this.goalsNs.GoalBlock
    const movements = new this.MovementsCtor(this.bot)
    applyProfile(movements, this.impl, this.profile)
    const ms: number[] = []
    let last: PathUpdate = { status: 'unknown' }
    for (let i = 0; i < repeats; i++) {
      const t0 = performance.now()
      const gen = (this.pf.getPathFromTo as (m: unknown, p: unknown, g: unknown, o: unknown) => Generator<{ result: PathUpdate }>)(
        movements, this.bot.entity.position.clone(), new Goal(end[0], end[1], end[2]),
        { timeout: this.profile.thinkTimeout, tickTimeout: 40 }
      )
      for (const { result } of gen) {
        last = result
        if (result.status !== 'partial') break
      }
      ms.push(performance.now() - t0)
    }
    return {
      ms,
      visited: last.visitedNodes ?? 0,
      cost: last.cost ?? null,
      nodes: last.path?.length ?? null,
      status: last.status
    }
  }

  /**
   * Steady-state planning cost: what a running bot actually pays per solve.
   *
   * `solveOnly` builds a private snapshot on every call, so it can only ever
   * report the cold number. On a short route that fixed cost dominates and
   * flatters upstream, which builds nothing and lazily reads only the blocks
   * it expands — on one 106-node route the cold figures were upstream 27.1 ms
   * against 34.9 ms here, while the same search off a warm cache took 3 ms.
   * Driving the live path reuses the cached snapshot, so every solve after
   * the first is warm. Upstream has no such cache, so its warm and cold
   * numbers should agree; that agreement is the control.
   *
   * The goal is dropped the moment a path lands, so the bot barely moves.
   */
  async solveWarm (end: [number, number, number], repeats: number): Promise<number[]> {
    const Goal = this.goalsNs.GoalBlock
    const pf = this.pf
    const times: number[] = []
    for (let i = 0; i < repeats; i++) {
      const ms = await new Promise<number>(resolve => {
        const onUpdate = (r: PathUpdate): void => {
          if (r.status === 'partial') return
          finish(r.time ?? 0)
        }
        const timer = setTimeout(() => finish(-1), this.profile.thinkTimeout + 2000)
        const finish = (value: number): void => {
          clearTimeout(timer)
          this.bot.removeListener('path_update' as never, onUpdate as never)
          resolve(value)
        }
        this.bot.on('path_update' as never, onUpdate as never)
        ;(pf.setGoal as (g: unknown) => void)(new Goal(end[0], end[1], end[2]))
      })
      ;(pf.setGoal as (g: unknown) => void)(null)
      this.bot.clearControlStates()
      if (ms >= 0) times.push(ms)
      await new Promise(resolve => setTimeout(resolve, 200))
    }
    return times
  }

  /**
   * Walk to `end`, recording everything worth comparing.
   *
   * `startAt` is a shared wall-clock instant. The parent sends the go message
   * to each racer in turn, and each process schedules its own event loop, so
   * without a common instant the first racer to be told gets a head start
   * that shows up as a visibly earlier launch. Every process reads the same
   * system clock, so waiting for the instant removes that skew; the actual
   * launch time is recorded so the skew can be reported instead of assumed.
   */
  async race (
    routeId: string,
    end: [number, number, number],
    tolerance: number,
    timeoutMs: number,
    startAt = 0
  ): Promise<RunResult> {
    const bot = this.bot
    const pf = this.pf
    const goalCentre = new Vec3(end[0] + 0.5, end[1], end[2] + 0.5)
    const Goal = tolerance > 0 ? this.goalsNs.GoalNear : this.goalsNs.GoalBlock
    const goal = tolerance > 0
      ? new Goal(end[0], end[1], end[2], tolerance)
      : new Goal(end[0], end[1], end[2])

    const result: RunResult = {
      impl: this.impl,
      routeId,
      outcome: 'error',
      promiseOutcome: 'error',
      wallMs: 0,
      firstSolveMs: null,
      firstMoveMs: null,
      solveMsTotal: 0,
      solves: 0,
      visitedNodes: 0,
      generatedNodes: 0,
      firstPathNodes: null,
      firstPathCost: null,
      replans: 0,
      travelled: 0,
      jumps: 0,
      lagbacks: 0,
      damage: 0,
      deaths: 0,
      worstStallMs: 0,
      stalls: [],
      endDistance: 0,
      endPos: [0, 0, 0],
      startedAt: 0,
      statuses: {},
      resetReasons: {},
      gameMode: bot.game?.gameMode ?? 'unknown'
    }

    const raceStart = performance.now()
    let lastPos = bot.entity.position.clone()
    let anchor = bot.entity.position.clone()
    let lastMoveAt = performance.now()
    let lastHealth = bot.health
    let wasOnGround = bot.entity.onGround
    // Filled at the launch instant, not here: the spin-down must not count.
    let launchAt = 0
    let launchPos: Vec3 | null = null
    /** True while the current stall has already been snapshotted. */
    let stalled = false

    const onPathUpdate = (r: PathUpdate): void => {
      result.statuses[r.status] = (result.statuses[r.status] ?? 0) + 1
      if ((result.updates?.length ?? 0) < 200) {
        result.updates?.push([Math.round(performance.now() - raceStart), r.status, r.path?.length ?? -1])
      }
      // The executor walks whatever array arrived last; follow it.
      if (Array.isArray(r.path)) pathRef = r.path as Array<{ x: number, y: number, z: number }>
      if (r.status === 'partial') return
      result.solves++
      result.solveMsTotal += r.time ?? 0
      result.visitedNodes += r.visitedNodes ?? 0
      result.generatedNodes += r.generatedNodes ?? 0
      if (result.firstSolveMs === null) {
        result.firstSolveMs = r.time ?? 0
        result.firstPathNodes = r.path?.length ?? null
        result.firstPathCost = r.cost ?? null
        if (r.status === 'success' && Array.isArray(r.path)) {
          const nodes = r.path as Array<{ x: number, y: number, z: number }>
          result.firstPath = nodes.slice(0, 400).map(n => [n.x, n.y, n.z])
          result.firstPathParkour = nodes
            .slice(0, 400)
            .flatMap((n, i) => (n as { parkour?: boolean }).parkour === true ? [i] : [])
          livePath = nodes // the same array the executor drains
          pathRef = nodes
        }
      }
    }
    // Watch the live array empty, and note where the bot was when it did.
    let livePath: Array<{ x: number, y: number, z: number }> | null = null
    /** Same array, never cleared — used for the node count in the trace. */
    let pathRef: Array<{ x: number, y: number, z: number }> | null = null
    let lastSample = 0
    result.trace = []
    result.updates = []
    const onReset = (reason: string): void => {
      result.replans++
      const key = typeof reason === 'string' ? reason : 'unknown'
      result.resetReasons[key] = (result.resetReasons[key] ?? 0) + 1
    }
    const onTick = (): void => {
      const p = bot.entity.position
      const d = p.distanceTo(lastPos)
      const now = performance.now()
      if (d > 0.02) { result.travelled += d; lastPos = p.clone() }
      // 0.3 blocks clears the jitter of a bot the server keeps nudging, and a
      // real first step covers it inside two ticks.
      if (result.firstMoveMs === null && launchPos !== null && p.distanceTo(launchPos) > 0.3) {
        result.firstMoveMs = now - launchAt
      }
      // Stall means "made no PROGRESS", not "did not twitch". A wedged bot
      // that the server keeps nudging jitters constantly: measured against a
      // per-tick delta it looked like a 0.9 s stall while it sat in the same
      // spot for 46 s. Anchor on a position and only move the anchor once the
      // bot has genuinely left it.
      if (p.distanceTo(anchor) > 0.75) { anchor = p.clone(); lastMoveAt = now; stalled = false } else {
        const stalledMs = now - lastMoveAt
        result.worstStallMs = Math.max(result.worstStallMs, stalledMs)
        // Caught in the act, once per stall: the remaining path is the half
        // that says what the bot was failing to do, and it is gone by the
        // time the run ends.
        if (!stalled && stalledMs > STUCK_MS && result.stalls.length < 8) {
          stalled = true
          result.stalls.push({
            atMs: Math.round(now - raceStart),
            stalledMs: Math.round(stalledMs),
            pos: [Number(p.x.toFixed(2)), Number(p.y.toFixed(2)), Number(p.z.toFixed(2))],
            ahead: (pathRef ?? []).slice(0, 6).map(n => ({
              x: n.x,
              y: n.y,
              z: n.z,
              parkour: (n as { parkour?: boolean }).parkour === true,
              cost: (n as { cost?: number }).cost ?? 0
            })),
            remaining: pathRef?.length ?? -1
          })
        }
      }
      if (wasOnGround && !bot.entity.onGround && bot.entity.velocity.y > 0.3) result.jumps++
      wasOnGround = bot.entity.onGround
      if (livePath !== null && livePath.length === 0) {
        result.drainedAt = [p.x, p.y, p.z]
        result.drainedAfterMs = now - raceStart
        livePath = null
      }
      if (now - lastSample >= 200 && (result.trace?.length ?? 0) < 900) {
        lastSample = now
        result.trace?.push([
          Math.round(now - raceStart),
          Number(p.x.toFixed(1)), Number(p.y.toFixed(1)), Number(p.z.toFixed(1)),
          pathRef?.length ?? -1
        ])
      }
    }
    const onForced = (): void => { result.lagbacks++ }
    const onHealth = (): void => {
      if (bot.health < lastHealth) result.damage += lastHealth - bot.health
      lastHealth = bot.health
    }
    let died = false
    // Respawn teleports the bot to world spawn, which would report a wildly
    // wrong "distance left" — one run said 28.9 blocks and its twin 192.2
    // for the identical journey. Freeze the position at the moment of death.
    let deathPos: Vec3 | null = null
    const onDeath = (): void => {
      died = true
      result.deaths++
      deathPos ??= bot.entity.position.clone()
    }

    bot.on('path_update' as never, onPathUpdate as never)
    bot.on('path_reset' as never, onReset as never)
    bot.on('physicsTick', onTick)
    bot.on('forcedMove', onForced)
    bot.on('health', onHealth)
    bot.on('death', onDeath)

    // Spin down to the shared instant, then launch. The last stretch is a
    // tight poll so the launch lands on the instant rather than on the far
    // side of a timer's scheduling slop.
    while (startAt > 0 && Date.now() < startAt) {
      const remaining = startAt - Date.now()
      if (remaining > 20) await new Promise(resolve => setTimeout(resolve, remaining - 15))
      else await new Promise(resolve => setImmediate(resolve))
    }

    result.startedAt = Date.now()
    const t0 = performance.now()
    launchAt = t0
    launchPos = bot.entity.position.clone()
    try {
      const goto = (pf.goto as (g: unknown) => Promise<void>)(goal)
      let timer: NodeJS.Timeout | undefined
      let deathTimer: NodeJS.Timeout | undefined
      const guard = new Promise<Outcome>(resolve => {
        timer = setTimeout(() => resolve('timeout'), timeoutMs)
        // goto() has no idea the bot died; without this the run would hang
        // until the timeout and score a death as a slow success.
        deathTimer = setInterval(() => { if (died) resolve('died') }, 100)
      })
      const outcome = await Promise.race([
        goto.then<Outcome>(() => 'arrived').catch((error: Error): Outcome => {
          result.error = error.message
          // Upstream puts the machine-readable tag in `name` and prose in
          // `message` ("Took to long to decide path to goal!"), so matching
          // only the message misfiles every timeout as a plain stop.
          const tag = `${error.name} ${error.message}`
          if (/NoPath/i.test(tag)) return 'no-path'
          if (/Timeout/i.test(tag)) return 'think-timeout'
          return 'stopped'
        }),
        guard
      ])
      clearTimeout(timer)
      clearInterval(deathTimer)
      result.outcome = outcome
      result.promiseOutcome = outcome
      if (outcome === 'timeout' || outcome === 'died') {
        ;(pf.setGoal as (g: unknown) => void)(null)
        goto.catch(() => {}) // the abort makes goto reject; already accounted for
      }
    } catch (error) {
      result.outcome = 'error'
      result.promiseOutcome = 'error'
      result.error = (error as Error).message
    } finally {
      result.wallMs = performance.now() - t0
      const last = deathPos ?? bot.entity.position
      result.endDistance = last.distanceTo(goalCentre)
      result.endPos = [Number(last.x.toFixed(2)), Number(last.y.toFixed(2)), Number(last.z.toFixed(2))]
      bot.removeListener('path_update' as never, onPathUpdate as never)
      bot.removeListener('path_reset' as never, onReset as never)
      bot.removeListener('physicsTick', onTick)
      bot.removeListener('forcedMove', onForced)
      bot.removeListener('health', onHealth)
      bot.removeListener('death', onDeath)
      bot.clearControlStates()
    }

    // Where the bot is beats what goto() said. Upstream's goto resolves
    // SUCCESSFULLY on any path_update carrying an empty path, and its noPath
    // results carry `path: []` — so an unreachable goal reports "arrived" in
    // 90 ms without the bot moving. Scoring the promise would hand upstream a
    // win for giving up fastest.
    // A stand on a fence post, pot or head has its node in the cell ABOVE
    // the block while the feet rest in the upper part of the block's own
    // cell (260.5 on a pot whose node is 261), so the feet may floor to one
    // below the end cell as long as they are clearly off that cell's floor.
    const endPos = deathPos ?? bot.entity.position
    const f = endPos.floored()
    const onEndColumn = f.x === end[0] && f.z === end[2]
    const atEndHeight = f.y === end[1] || (f.y === end[1] - 1 && endPos.y - f.y >= 0.3)
    const reached = tolerance > 0
      ? result.endDistance <= tolerance + 0.5
      : onEndColumn && atEndHeight
    if (reached) {
      result.outcome = 'arrived'
    } else if (result.outcome === 'arrived' || result.outcome === 'stopped') {
      result.outcome = (result.statuses.noPath ?? 0) > 0
        ? 'no-path'
        : (result.statuses.timeout ?? 0) > 0 ? 'think-timeout' : 'gave-up'
    }
    return result
  }

  /**
   * Read the terrain the run actually failed on, from the bot that failed on
   * it. The referee cannot do this: it sits at the finish on a 2-chunk view
   * distance, so the blocks around a stall halfway down the route are simply
   * not loaded for it. The racer still has them.
   *
   * `shapes` is carried for the focus blocks on purpose. `boundingBox` says
   * `block` for carpets, slabs and snow layers alike, so it cannot explain a
   * bot resting at y .4; the collision shapes can.
   */
  probeWorld (focus: Array<{ pos: [number, number, number], label: string }>, radius = 3, below = 2, above = 3): WorldProbe {
    const probe: WorldProbe = { blocks: [], unloaded: [], focus: [] }
    const seen = new Set<string>()
    const air = new Set(['air', 'cave_air', 'void_air'])
    const read = (x: number, y: number, z: number): BlockLike | null =>
      this.bot.blockAt(new Vec3(x, y, z)) as BlockLike | null

    for (const { pos, label } of focus) {
      const [cx, cy, cz] = pos.map(Math.floor) as [number, number, number]
      for (const [dy, role] of [[-1, `${label}: under`], [0, `${label}: feet`], [1, `${label}: head`]] as const) {
        const b = read(cx, cy + dy, cz)
        probe.focus.push({
          pos: [cx, cy + dy, cz],
          label: role,
          name: b?.name ?? 'unloaded',
          boundingBox: b?.boundingBox ?? 'unknown',
          properties: readProperties(b),
          shapes: b?.shapes ?? []
        })
      }
      for (let dx = -radius; dx <= radius; dx++) {
        for (let dz = -radius; dz <= radius; dz++) {
          for (let dy = -below; dy <= above; dy++) {
            const x = cx + dx
            const y = cy + dy
            const z = cz + dz
            const key = `${x},${y},${z}`
            if (seen.has(key)) continue
            seen.add(key)
            const b = read(x, y, z)
            if (b === null) { probe.unloaded.push([x, y, z]); continue }
            if (air.has(b.name)) continue
            probe.blocks.push([x, y, z, b.name])
          }
        }
      }
    }
    return probe
  }

  stop (): void {
    try { (this.pf.setGoal as (g: unknown) => void)(null) } catch { /* already idle */ }
    this.bot.clearControlStates()
  }
}
