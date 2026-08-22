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

/**
 * `bulba` runs the reference JavaScript solver on the main thread;
 * `bulba-wasm` runs the production path — the Rust core in a worker thread.
 * Both are the same algorithm and are pinned bit-identical by the package's
 * differential suite, so racing them apart measures the engine room, not the
 * search.
 */
export type Impl = 'upstream' | 'bulba' | 'bulba-wasm'

export type Outcome =
  | 'arrived'
  | 'no-path'
  | 'think-timeout'
  | 'timeout'
  | 'died'
  | 'gave-up'
  | 'stopped'
  | 'error'

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
  solveMsTotal: number
  solves: number
  visitedNodes: number
  generatedNodes: number
  firstPathNodes: number | null
  firstPathCost: number | null
  replans: number
  travelled: number
  jumps: number
  damage: number
  deaths: number
  worstStallMs: number
  endDistance: number
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
  // The extended repertoire is the thing under test; upstream has no such
  // flag, so this is where the engines are allowed to differ.
  if (impl !== 'upstream') movements.allowParkourExtended = profile.extendedParkour
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
      solveMsTotal: 0,
      solves: 0,
      visitedNodes: 0,
      generatedNodes: 0,
      firstPathNodes: null,
      firstPathCost: null,
      replans: 0,
      travelled: 0,
      jumps: 0,
      damage: 0,
      deaths: 0,
      worstStallMs: 0,
      endDistance: 0,
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
      // Stall means "made no PROGRESS", not "did not twitch". A wedged bot
      // that the server keeps nudging jitters constantly: measured against a
      // per-tick delta it looked like a 0.9 s stall while it sat in the same
      // spot for 46 s. Anchor on a position and only move the anchor once the
      // bot has genuinely left it.
      if (p.distanceTo(anchor) > 0.75) { anchor = p.clone(); lastMoveAt = now } else {
        result.worstStallMs = Math.max(result.worstStallMs, now - lastMoveAt)
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
      result.endDistance = (deathPos ?? bot.entity.position).distanceTo(goalCentre)
      bot.removeListener('path_update' as never, onPathUpdate as never)
      bot.removeListener('path_reset' as never, onReset as never)
      bot.removeListener('physicsTick', onTick)
      bot.removeListener('health', onHealth)
      bot.removeListener('death', onDeath)
      bot.clearControlStates()
    }

    // Where the bot is beats what goto() said. Upstream's goto resolves
    // SUCCESSFULLY on any path_update carrying an empty path, and its noPath
    // results carry `path: []` — so an unreachable goal reports "arrived" in
    // 90 ms without the bot moving. Scoring the promise would hand upstream a
    // win for giving up fastest.
    const f = (deathPos ?? bot.entity.position).floored()
    const reached = tolerance > 0
      ? result.endDistance <= tolerance + 0.5
      : f.x === end[0] && f.y === end[1] && f.z === end[2]
    if (reached) {
      result.outcome = 'arrived'
    } else if (result.outcome === 'arrived' || result.outcome === 'stopped') {
      result.outcome = (result.statuses.noPath ?? 0) > 0
        ? 'no-path'
        : (result.statuses.timeout ?? 0) > 0 ? 'think-timeout' : 'gave-up'
    }
    return result
  }

  stop (): void {
    try { (this.pf.setGoal as (g: unknown) => void)(null) } catch { /* already idle */ }
    this.bot.clearControlStates()
  }
}
