// Item use cancels sprinting, so plugins acquire controls at a safe stopping point.
// Interrupts preserve the path; exclusive holders prevent concurrent control or equipment changes.
import { performance } from 'node:perf_hooks'

/** Current executor phase from bot.pathfinder.motion;
 * paused takes precedence while an interrupt is held. */
export type MotionPhase =
  | 'idle'
  | 'walking'
  | 'airborne'
  | 'climbing'
  | 'swimming'
  | 'digging'
  | 'interacting'
  | 'paused'

/** Control lease returned by bot.pathfinder.interrupt(); movement stays paused until release. */
export interface InterruptHandle {
  readonly reason: string
  /** performance.now() at grant. */
  readonly grantedAt: number
  /** True until released. */
  readonly active: boolean
  /** Nothing else may hold the controls while this is held. */
  readonly exclusive: boolean
  /** Return the controls; repeated calls have no effect. */
  release: () => void
}

export interface InterruptOptions {
  /** Wait for a safe stopping point before forcing a pause (milliseconds, default 5000).
   * 0 waits indefinitely. Expiry never overrides an existing holder's exclusivity. */
  timeout?: number
  /** Skip the wait for a safe moment; take the controls on the next tick. */
  force?: boolean
  /** Require sole access to the controls (default true); false permits sharing
   * with passive holders that only need the bot to stand still. */
  exclusive?: boolean
  /** Abort a pending request; granted interrupts must be released. */
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
  /** Check that the body is supported and no digging or interaction is in progress. */
  isSafe: () => boolean
  /** Release executor controls before handing them to interrupt holders. */
  onPause: (reasons: string[]) => void
  /** Resume driving after pausedMs milliseconds of interruption. */
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

/** Per-bot pause controller; the executor calls gate each tick and consumers request control through acquire. */
export class InterruptController {
  private readonly held = new Set<InterruptHandle>()
  private waiters: Waiter[] = []
  private pausedSince = 0
  private closed: string | null = null

  constructor (private readonly bot: EmitterLike, private readonly hooks: Hooks) {}

  /** True while at least one interrupt is held (the executor is not driving). */
  get paused (): boolean {
    return this.held.size > 0
  }

  /** True while an exclusive holder owns the controls, and nothing else may write them. */
  get exclusive (): boolean {
    return this.hasExclusiveHolder()
  }

  /** True while a control request is pending. */
  get wanted (): boolean {
    return this.waiters.length > 0
  }

  /** Current holders' reasons, used for diagnostics and motion.phase. */
  get holders (): string[] {
    const out: string[] = []
    for (const h of this.held) out.push(h.reason)
    return out
  }

  /** Request control once the executor stops at a safe point or the timeout expires.
   * Exclusive holders still block the request; compatible shared requests can overlap. */
  async acquire (reason: string, options: InterruptOptions = {}): Promise<InterruptHandle> {
    const timeout = options.timeout ?? 5000
    const signal = options.signal

    const refuseIfOver = (): void => {
      if (signal?.aborted === true) {
        throw makeError('InterruptAborted', `Interrupt request '${reason}' was aborted before it was granted.`)
      }
      if (this.closed !== null) throw makeError('InterruptAborted', this.closed)
    }
    refuseIfOver()

    this.bot.emit('pathfinder:interrupt_requested', reason)

    // A listener may have disposed the plugin or aborted this request.
    refuseIfOver()

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
      // Enforce the deadline even without physics ticks, such as before spawn or during a stalled connection.
      if (isFinite(waiter.deadline)) {
        waiter.timer = setTimeout(() => { this.gate() }, timeout + 16)
        ;(waiter.timer as unknown as { unref?: () => void }).unref?.()
      }
      this.waiters.push(waiter)
      // Keep control handover in gate() so requests cannot interrupt executor writes.
    })
  }

  /** Acquire control, run fn and release even on errors, preventing a permanently paused bot. */
  async run<T> (reason: string, fn: () => Promise<T>, options: InterruptOptions = {}): Promise<T> {
    const handle = await this.acquire(reason, options)
    try {
      return await fn()
    } finally {
      handle.release()
    }
  }

  /** Run before executor control writes each tick; true means the executor must leave controls untouched. */
  gate (): boolean {
    if (this.waiters.length > 0) this.grantWhatWeCan()
    return this.held.size > 0
  }

  private grantWhatWeCan (): void {
    const now = performance.now()
    const wasPaused = this.held.size > 0
    let paused = wasPaused
    const granted: Array<{ waiter: Waiter, handle: InterruptHandle }> = []

    // FIFO blocking prevents later shared requests from starving an exclusive request.
    while (this.waiters.length > 0) {
      const w = this.waiters[0]
      if (this.hasExclusiveHolder()) break
      if (w.exclusive && this.held.size > 0) break
      // A paused executor already provides a stopping point for compatible shared requests.
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
    // Stop executor writes before resolving, so holders cannot race this tick's controls.
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

  /** The bot is gone: reject every waiter and all later requests, since no tick will ever grant them. */
  close (message: string): void {
    this.closed = message
    const waiters = this.waiters
    this.waiters = []
    for (const w of waiters) {
      if (w.timer) clearTimeout(w.timer)
      if (w.signal && w.onAbort) w.signal.removeEventListener('abort', w.onAbort)
      w.reject(makeError('InterruptAborted', message))
    }
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
        // Grant queued requests immediately when the blocking holder releases.
        if (self.waiters.length > 0) self.grantWhatWeCan()
      }
    }
    return handle
  }
}
