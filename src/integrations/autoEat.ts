// Eating changes the held item and cancels sprint, which can shorten a jump.
// Wait for a grounded interrupt before eating, then resume the path.
import type { Bot } from 'mineflayer'
import type { InterruptOptions } from '../interrupt.js'

export interface AutoEatIntegrationOptions {
  /** Food level (0–20) at or below which the bot eats. Default 16. */
  startAt?: number
  /** Also eat when health drops to or below this, even if not hungry. Default 0 (off). */
  healthBelow?: number
  /** How often to check, in ticks. Default 20 (once a second). */
  checkEveryTicks?: number
  /** Maximum wait for a safe moment before eating anyway, in ms (default 10000). */
  waitForSafe?: number
  /** Skip eating while a window is open (default true). */
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
  entity?: { isInWater?: boolean }
  setControlState?: (control: string, state: boolean) => void
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

/** Build an integration loaded after pathfinder and mineflayer-auto-eat.
 * Delayed auto-eat injection is detected on a later tick. */
export function autoEatIntegration (options: AutoEatIntegrationOptions = {}) {
  // Typed as mineflayer's Bot, so that bot.loadPlugin accepts it.
  return function inject (botArg: Bot): void {
    const bot = botArg as unknown as BotLike
    const opts = {
      ...DEFAULTS,
      ...stripUndefined(options),
      eatOptions: options.eatOptions ?? {}
    }

    let suspensions = 0
    let eating = false
    let swimming = false
    let ticks = 0
    let takenOver = false
    let detached = false

    function autoEat (): AutoEatLike | null {
      return bot.autoEat ?? null
    }

    /** Disable the plugin's automatic eating so interrupts control its timing. */
    function takeOver (): void {
      const ae = autoEat()
      if (!ae || takenOver) return
      takenOver = true
      if (typeof ae.disableAuto === 'function') ae.disableAuto()
      else if (typeof ae.disable === 'function') ae.disable()
      // Keep the plugin's threshold consistent if it re-enables itself.
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

    /** Eat during a grounded interrupt, after exclusive actions release the bot's hands. */
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
        // The pause leaves every control to this holder, so keep afloat while eating in water.
        swimming = handle !== null && bot.entity?.isInWater === true
        if (swimming) bot.setControlState?.('jump', true)
        bot.emit('pathfinder:autoeat_start')
        await ae.eat(opts.eatOptions)
        bot.emit('pathfinder:autoeat_finish')
        return true
      } catch (err) {
        bot.emit('pathfinder:autoeat_failed', err)
        return false
      } finally {
        if (swimming) bot.setControlState?.('jump', false)
        swimming = false
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
