// Cooperative interrupts: how another plugin borrows the bot mid-path.
//
// The problem this solves is the one every mineflayer stack hits eventually.
// A second plugin — auto-eat, a totem swapper, an armour manager, a trade
// handler — wants the bot's hands for a moment. It has no idea what the
// pathfinder is doing, so it acts whenever its own trigger fires. If that
// moment happens to be the tick the executor committed to a sprint-jump, the
// bot eats in mid-air: item use and sprinting cancel each other in vanilla,
// the held item is swapped away from the tool the next node needs, and the
// ~1.6 s of eating lands the bot somewhere the planner never routed through.
// The pathfinder then replans from wherever it fell, which is how "the bot
// randomly walks back on itself" happens.
//
// The fix is not to teach the pathfinder about eating. It is to give
// borrowers a way to ASK, and to answer only at a moment where handing over
// the controls is harmless: on the ground, not mid-dig, not mid-interaction.
// A request that arrives while the bot is standing on a block is granted on
// the next tick — so "wait before jumping" is not a special case, it is what
// falls out: the executor yields before it reaches the take-off decision.
//
// The path is NOT reset while an interrupt is held. No `path_reset`, no
// re-solve, no `goto()` rejection — the bot stands still on its own path and
// carries on from the same node when the holder releases.
//
// Interrupts are EXCLUSIVE by default, and that is the other half of the
// point: a bot has one pair of hands. While `pathfinder.dig()` holds the
// controls, an auto-eat request queues behind it instead of swapping the
// pickaxe out from under a swing.
import { performance } from 'node:perf_hooks'

/**
 * What the executor is doing right now. Read off `bot.pathfinder.motion`;
 * `paused` outranks the rest (an interrupt is held).
 */
export type MotionPhase =
  | 'idle'
  | 'walking'
  | 'airborne'
  | 'climbing'
  | 'swimming'
  | 'digging'
  | 'interacting'
  | 'paused'

/** Handle returned by `bot.pathfinder.interrupt()`. Release it or the bot never walks again. */
export interface InterruptHandle {
  readonly reason: string
  /** performance.now() at grant. */
  readonly grantedAt: number
  /** True until released. */
  readonly active: boolean
  /** Nothing else may hold the controls while this is held. */
  readonly exclusive: boolean
  /** Hand the controls back. Idempotent. */
  release: () => void
}

export interface InterruptOptions {
  /**
   * How long to wait for a safe hand-over point before taking the controls
   * anyway (ms, default 5000). The wait exists to avoid interrupting a jump;
   * it must not be able to stop a starving bot from eating, so it expires.
   * `0` waits indefinitely for a genuinely safe window.
   *
   * The deadline does NOT override exclusivity — a request still queues
   * behind a holder that has the hands.
   */
  timeout?: number
  /** Skip the wait for a safe moment; take the controls on the next tick. */
  force?: boolean
  /**
   * Refuse to share (default true). Pass `false` for a passive observer that
   * only wants the bot to stand still and will not touch its hands.
   */
  exclusive?: boolean
  /** Abort the *request*. A granted interrupt is released, not aborted. */
  signal?: AbortSignal
}

interface Waiter {
  reason: string
  force: boolean
  exclusive: boolean
  deadline: number
  resolve: (h: InterruptHandle) => void
  reject: (e: Error) => void
  signal?: AbortSignal
  onAbort?: () => void
  timer?: ReturnType<typeof setTimeout>
}

interface Hooks {
  /**
   * Is right now a harmless moment to stop driving? The executor answers from
   * its own state — feet on something, nothing half-finished.
   */
  isSafe: () => boolean
  /** Controls are being handed over: release everything the tick loop presses. */
  onPause: (reasons: string[]) => void
  /** Controls are coming back after `pausedMs` of standing still. */
  onResume: (pausedMs: number) => void
}

interface EmitterLike {
  emit: (event: string, ...args: unknown[]) => boolean
}

function makeError (name: string, message: string): Error {
  const err = new Error(message)
  err.name = name
  return err
}

/**
 * Owns the pause protocol. One per injected bot; the executor calls
 * {@link gate} once per tick and consumers call {@link acquire}.
 */
export class InterruptController {
  private readonly held = new Set<InterruptHandle>()
  private waiters: Waiter[] = []
  private pausedSince = 0

  constructor (private readonly bot: EmitterLike, private readonly hooks: Hooks) {}

  /** True while at least one interrupt is held (the executor is not driving). */
  get paused (): boolean {
    return this.held.size > 0
  }

  /** True while someone is waiting for a hand-over that hasn't happened yet. */
  get wanted (): boolean {
    return this.waiters.length > 0
  }

  /** Reasons of every current holder — for diagnostics and `motion.phase`. */
  get holders (): string[] {
    const out: string[] = []
    for (const h of this.held) out.push(h.reason)
    return out
  }

  /**
   * Ask for the controls. Resolves once the executor has stopped driving and
   * nothing else holds them — on the next tick if the bot is already standing
   * somewhere safe, otherwise as soon as it lands (or when `timeout` expires,
   * whichever comes first).
   */
  async acquire (reason: string, options: InterruptOptions = {}): Promise<InterruptHandle> {
    const timeout = options.timeout ?? 5000
    const signal = options.signal

    if (signal?.aborted === true) {
      throw makeError('InterruptAborted', `Interrupt request '${reason}' was aborted before it was granted.`)
    }

    this.bot.emit('pathfinder:interrupt_requested', reason)

    return await new Promise<InterruptHandle>((resolve, reject) => {
      const waiter: Waiter = {
        reason,
        force: options.force === true,
        exclusive: options.exclusive !== false,
        deadline: timeout > 0 ? performance.now() + timeout : Number.POSITIVE_INFINITY,
        resolve,
        reject,
        signal
      }
      if (signal) {
        waiter.onAbort = () => {
          this.drop(waiter)
          reject(makeError('InterruptAborted', `Interrupt request '${reason}' was aborted before it was granted.`))
        }
        signal.addEventListener('abort', waiter.onAbort, { once: true })
      }
      // The deadline is enforced from gate(), which only runs on a physics
      // tick. Nudge ourselves once past it so a bot that is not ticking (not
      // spawned yet, or a stalled connection) cannot hang the caller forever.
      if (isFinite(waiter.deadline)) {
        waiter.timer = setTimeout(() => { this.gate() }, timeout + 16)
        ;(waiter.timer as unknown as { unref?: () => void }).unref?.()
      }
      this.waiters.push(waiter)
      // Granting happens only in gate(), so there is exactly one code path
      // that hands over control and it is the one the executor drives.
    })
  }

  /**
   * Acquire, run `fn`, always release. Prefer this over a manual release — a
   * thrown error that skips `release()` wedges the bot for good.
   */
  async run<T> (reason: string, fn: () => Promise<T>, options: InterruptOptions = {}): Promise<T> {
    const handle = await this.acquire(reason, options)
    try {
      return await fn()
    } finally {
      handle.release()
    }
  }

  /**
   * Called once per tick by the executor, before it drives anything. Returns
   * true when the executor must not touch the controls this tick.
   */
  gate (): boolean {
    if (this.waiters.length > 0) this.grantWhatWeCan()
    return this.held.size > 0
  }

  private grantWhatWeCan (): void {
    const now = performance.now()
    const wasPaused = this.held.size > 0
    let paused = wasPaused
    const granted: Array<{ waiter: Waiter, handle: InterruptHandle }> = []

    // FIFO with head-of-line blocking: the first request that cannot be
    // granted stops the pass, so a stream of shared requests can never
    // starve an exclusive one waiting behind them.
    while (this.waiters.length > 0) {
      const w = this.waiters[0]
      if (this.hasExclusiveHolder()) break
      if (w.exclusive && this.held.size > 0) break
      // Once the bot is stopped there is nothing left to wait for.
      if (!paused && !this.hooks.isSafe() && !w.force && now < w.deadline) break

      this.waiters.shift()
      if (w.signal && w.onAbort) w.signal.removeEventListener('abort', w.onAbort)
      if (w.timer) clearTimeout(w.timer)
      const handle = this.makeHandle(w.reason, w.exclusive)
      this.held.add(handle)
      granted.push({ waiter: w, handle })
      paused = true
    }

    if (granted.length === 0) return

    if (!wasPaused) {
      this.pausedSince = now
      const reasons = granted.map(g => g.waiter.reason)
      this.hooks.onPause(reasons)
      this.bot.emit('pathfinder:paused', reasons)
    }
    // Resolve only after the executor has been told to stop, so a holder that
    // acts synchronously on resolution cannot race this tick's control writes.
    for (const { waiter, handle } of granted) waiter.resolve(handle)
  }

  private hasExclusiveHolder (): boolean {
    for (const h of this.held) if (h.exclusive) return true
    return false
  }

  private drop (waiter: Waiter): void {
    this.waiters = this.waiters.filter(w => w !== waiter)
    if (waiter.timer) clearTimeout(waiter.timer)
  }

  private makeHandle (reason: string, exclusive: boolean): InterruptHandle {
    let active = true
    const grantedAt = performance.now()
    const self = this
    const handle: InterruptHandle = {
      reason,
      grantedAt,
      exclusive,
      get active () { return active },
      release () {
        if (!active) return
        active = false
        self.held.delete(handle)
        if (self.held.size === 0) {
          const pausedMs = performance.now() - self.pausedSince
          self.pausedSince = 0
          self.hooks.onResume(pausedMs)
          self.bot.emit('pathfinder:resumed', reason, pausedMs)
        }
        // Someone may have been queued behind this holder; let them in
        // without waiting for the next tick.
        if (self.waiters.length > 0) self.grantWhatWeCan()
      }
    }
    return handle
  }
}
