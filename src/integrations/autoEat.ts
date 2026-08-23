// mineflayer-auto-eat, taught to wait for a safe moment.
//
// Out of the box the plugin eats from its own `physicsTick` handler the
// instant hunger crosses its threshold. That is fine for a bot standing in a
// field and wrong for a bot walking a path: eating swaps the main hand, and
// item use and sprinting cancel each other in vanilla — so a bite taken at
// take-off drops the sprint the jump needed, and a bite taken in flight lands
// the bot short with the wrong item held. The pathfinder then replans from
// wherever it fell.
//
// This adapter turns the plugin's automatic mode off and drives it through
// the pathfinder's interrupt protocol instead: the bot lands, stops on its
// own path, eats, and walks on from the same node. Nothing is re-planned and
// `goto()` never rejects — from the caller's point of view the walk just took
// a second and a half longer.
//
// ```js
// import { pathfinder, autoEatIntegration } from '@bulba/pathfinder'
// import { loader as autoEat } from 'mineflayer-auto-eat'
//
// bot.loadPlugin(pathfinder)
// bot.loadPlugin(autoEat)
// bot.loadPlugin(autoEatIntegration({ startAt: 16 }))
// ```
//
// It is deliberately defensive about the plugin's own API: 3.x has shipped
// both `enableAuto`/`disableAuto` and `enable`/`disable`, and both `opts` and
// `options`. Whichever this bot has, the adapter finds it.
import type { InterruptOptions } from '../interrupt.js'

export interface AutoEatIntegrationOptions {
  /** Food level (0–20) at or below which the bot eats. Default 16. */
  startAt?: number
  /** Also eat when health drops to or below this, even if not hungry. Default 0 (off). */
  healthBelow?: number
  /** How often to check, in ticks. Default 20 (once a second). */
  checkEveryTicks?: number
  /**
   * How long to wait for a safe moment before eating anyway (ms, default
   * 10000). A bot that is genuinely wedged still has to eat.
   */
  waitForSafe?: number
  /** Don't eat while a window is open — clicking through a chest UI. Default true. */
  skipWhenWindowOpen?: boolean
  /** Options handed to the plugin's own `eat()`. */
  eatOptions?: Record<string, unknown>
  /** Reason string used for the interrupt. Default 'autoeat'. */
  reason?: string
}

interface AutoEatLike {
  isEating?: boolean
  enableAuto?: () => void
  disableAuto?: () => void
  enable?: () => void
  disable?: () => void
  eat: (opts?: unknown) => Promise<unknown>
  cancelEat?: () => void
  opts?: Record<string, unknown>
  options?: Record<string, unknown>
}

interface PathfinderLike {
  interrupt: (reason: string, options?: InterruptOptions) => Promise<{ release: () => void }>
}

interface BotLike {
  autoEat?: AutoEatLike
  pathfinder?: PathfinderLike
  food?: number
  health?: number
  currentWindow?: unknown
  on: (event: string, listener: (...args: unknown[]) => void) => unknown
  removeListener: (event: string, listener: (...args: unknown[]) => void) => unknown
  emit: (event: string, ...args: unknown[]) => boolean
}

/** Runtime handle installed at `bot.pathfinderAutoEat`. */
export interface AutoEatControl {
  /** Eat now (still waiting for a safe moment). Resolves when the bite is done. */
  eatNow: () => Promise<boolean>
  /** Stop the automatic checks; nested calls are counted. */
  suspend: () => void
  /** Undo one `suspend()`. */
  resume: () => void
  readonly suspended: boolean
  /** Is a bite in flight right now? */
  readonly eating: boolean
  /** Detach the adapter and hand automatic eating back to the plugin. */
  detach: () => void
  options: Required<Omit<AutoEatIntegrationOptions, 'eatOptions'>> & { eatOptions: Record<string, unknown> }
}

const DEFAULTS = {
  startAt: 16,
  healthBelow: 0,
  checkEveryTicks: 20,
  waitForSafe: 10_000,
  skipWhenWindowOpen: true,
  reason: 'autoeat'
}

/**
 * Build the plugin function. Load it AFTER both the pathfinder and
 * mineflayer-auto-eat; it tolerates being loaded before auto-eat finishes
 * injecting and picks it up on the first tick it appears.
 */
export function autoEatIntegration (options: AutoEatIntegrationOptions = {}) {
  return function inject (bot: BotLike): void {
    const opts = {
      ...DEFAULTS,
      ...stripUndefined(options),
      eatOptions: options.eatOptions ?? {}
    }

    let suspensions = 0
    let eating = false
    let ticks = 0
    let takenOver = false
    let detached = false

    function autoEat (): AutoEatLike | null {
      return bot.autoEat ?? null
    }

    /** Turn the plugin's own physicsTick eating off — we drive it ourselves. */
    function takeOver (): void {
      const ae = autoEat()
      if (!ae || takenOver) return
      takenOver = true
      if (typeof ae.disableAuto === 'function') ae.disableAuto()
      else if (typeof ae.disable === 'function') ae.disable()
      // Keep the plugin's own threshold from second-guessing ours if a later
      // version re-enables itself.
      const bag = ae.opts ?? ae.options
      if (bag) {
        if ('startAt' in bag) bag.startAt = opts.startAt
        if ('minHunger' in bag) bag.minHunger = opts.startAt
      }
    }

    function handBack (): void {
      const ae = autoEat()
      if (!ae || !takenOver) return
      takenOver = false
      if (typeof ae.enableAuto === 'function') ae.enableAuto()
      else if (typeof ae.enable === 'function') ae.enable()
    }

    function hungry (): boolean {
      const food = bot.food
      if (typeof food === 'number' && food <= opts.startAt) return true
      if (opts.healthBelow > 0 && typeof bot.health === 'number' && bot.health <= opts.healthBelow) return true
      return false
    }

    /**
     * Ask the pathfinder for a safe moment, then eat. The interrupt is what
     * makes this safe: it is granted on the ground, never mid-jump, and never
     * while `pathfinder.dig()` or another exclusive holder has the hands.
     */
    async function eatNow (): Promise<boolean> {
      const ae = autoEat()
      if (!ae || eating) return false
      const pf = bot.pathfinder
      eating = true
      let handle: { release: () => void } | null = null
      try {
        if (pf && typeof pf.interrupt === 'function') {
          handle = await pf.interrupt(opts.reason, { timeout: opts.waitForSafe })
        }
        bot.emit('pathfinder:autoeat_start')
        await ae.eat(opts.eatOptions)
        bot.emit('pathfinder:autoeat_finish')
        return true
      } catch (err) {
        bot.emit('pathfinder:autoeat_failed', err)
        return false
      } finally {
        handle?.release()
        eating = false
      }
    }

    function onTick (): void {
      if (detached) return
      takeOver()
      if (++ticks < opts.checkEveryTicks) return
      ticks = 0
      if (suspensions > 0 || eating) return
      const ae = autoEat()
      if (!ae || ae.isEating === true) return
      if (opts.skipWhenWindowOpen && bot.currentWindow) return
      if (!hungry()) return
      void eatNow()
    }

    bot.on('physicsTick', onTick)

    const control: AutoEatControl = {
      eatNow,
      suspend: () => { suspensions++ },
      resume: () => { if (suspensions > 0) suspensions-- },
      get suspended () { return suspensions > 0 },
      get eating () { return eating },
      detach: () => {
        if (detached) return
        detached = true
        bot.removeListener('physicsTick', onTick)
        handBack()
      },
      options: opts
    }
    ;(bot as unknown as { pathfinderAutoEat: AutoEatControl }).pathfinderAutoEat = control
  }
}

function stripUndefined<T extends object> (o: T): Partial<T> {
  const out: Partial<T> = {}
  for (const k of Object.keys(o) as Array<keyof T>) {
    if (o[k] !== undefined) out[k] = o[k]
  }
  return out
}
