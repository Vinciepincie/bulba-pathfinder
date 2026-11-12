// Bound window-open waits and remove abandoned listeners so later windows cannot satisfy them.
// Accept any window type and use target updates to distinguish processed clicks from silent drops.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { BlockLike, OpenOptions, WindowLike } from './types.js'
import type { ActionContext } from './context.js'
import {
  cellCentre, fmt, hasLineOfSight, inReach, pickFace, reachToCell, eyePos,
  settleAim, waitTicks
} from './reach.js'
import { deadlineFrom, throwIfAborted } from './dig.js'

/** Escalations, cheapest first. `resync` rungs change the bot's stance. */
interface Rung {
  name: string
  resync: boolean
  prepare?: (ctx: ActionContext, pos: Vec3, reach: number, signal?: AbortSignal) => Promise<void>
}

const RUNGS: Rung[] = [
  { name: 'as-is', resync: false },
  { name: 're-aim', resync: false },
  {
    name: 'jump (position resync)',
    resync: true,
    prepare: async (ctx) => {
      // A hop sends fresh position data before retrying an interaction from stale state.
      ctx.bot.setControlState('jump', true)
      await waitTicks(ctx.bot, 1)
      ctx.bot.setControlState('jump', false)
      await waitTicks(ctx.bot, 6)
    }
  },
  {
    name: 'walk closer',
    resync: true,
    prepare: async (ctx, pos, reach, signal) => {
      await ctx.approach(pos, Math.min(reach, 3), signal)
    }
  }
]

export function createOpen (ctx: ActionContext) {
  /** Most recent window opened by this bot, for target-specific reuse. */
  let lastOpened: { window: WindowLike, pos: Vec3 } | null = null

  return async function open (block: BlockLike, options: OpenOptions = {}): Promise<WindowLike> {
    const bot = ctx.bot
    const cfg = ctx.config
    const pos = block.position.floored()
    const reach = options.reach ?? cfg.reach
    const windowTimeout = options.windowTimeout ?? cfg.windowTimeout
    const deadline = deadlineFrom(options.timeout ?? cfg.timeout)
    const maxRungs = Math.min(RUNGS.length, Math.max(1, options.retries ?? RUNGS.length))
    throwIfAborted(options.signal, pos)

    // Reopening the same window costs round trips and can race.
    if (options.reuse !== false && bot.currentWindow && lastOpened?.window === bot.currentWindow &&
        lastOpened.pos.equals(pos) && inReach(bot, pos, reach)) {
      return bot.currentWindow as WindowLike
    }

    // A stale window makes the server drop the next open in silence.
    if (options.closeStale !== false && bot.currentWindow) {
      try { bot.closeWindow?.(bot.currentWindow) } catch { /* best effort */ }
      await waitTicks(bot, 2)
    }

    // Sneaking turns an open into a placement.
    bot.setControlState('sneak', false)

    const tried: string[] = []
    let lastError: Error | null = null

    for (let i = 0; i < maxRungs; i++) {
      const rung = RUNGS[i]
      throwIfAborted(options.signal, pos)
      if (Date.now() > deadline) break
      // approach: false means the bot stays where it is.
      if (rung.name === 'walk closer' && options.approach === false) break

      if (rung.prepare) await rung.prepare(ctx, pos, reach, options.signal)

      const live = bot.blockAt(pos, false)
      if (!live) {
        throw new ActionError(ActionErrors.NO_TARGET, `No block loaded at ${fmt(pos)}.`, pos)
      }

      if (!inReach(bot, pos, reach)) {
        if (options.approach === false) {
          throw new ActionError(
            ActionErrors.OUT_OF_REACH,
            `${live.name} at ${fmt(pos)} is ${reachToCell(eyePos(bot), pos).toFixed(2)} away (reach ${reach}).`,
            pos, true
          )
        }
        const ok = await ctx.approach(pos, reach, options.signal)
        if (!ok) {
          throw new ActionError(
            ActionErrors.UNREACHABLE, `No path to a spot within ${reach} of ${fmt(pos)}.`, pos
          )
        }
      }

      tried.push(rung.name)
      try {
        const window = await attemptOpen(ctx, live, pos, reach, windowTimeout, options.signal)
        lastOpened = { window, pos }
        bot.emit('pathfinder:open_finish', pos, window)
        return window
      } catch (err) {
        if (err instanceof ActionError && err.name === ActionErrors.ABORTED) throw err
        lastError = err as Error
        bot.emit('pathfinder:open_retry', pos, rung.name, lastError.message)
      }
    }

    throw new ActionError(
      ActionErrors.OPEN_FAILED,
      `Could not open ${block.name} at ${fmt(pos)} after ${tried.length} attempt(s) [${tried.join(', ')}]: ${lastError?.message ?? 'no window'}`,
      pos
    )
  }
}

async function attemptOpen (
  ctx: ActionContext,
  block: BlockLike,
  pos: Vec3,
  reach: number,
  windowTimeout: number,
  signal?: AbortSignal
): Promise<WindowLike> {
  const bot = ctx.bot

  const aim = pickFace(bot, pos, { requireCursor: false, allowGeometric: true, reach })
  const aimPoint = aim ? aim.point : cellCentre(pos)
  if (aim && !hasLineOfSight(bot, pos, aimPoint, reach)) {
    throw new ActionError(ActionErrors.OCCLUDED, `No line of sight to ${fmt(pos)}.`, pos, true)
  }

  await settleAim(bot, pos, aimPoint, { minTicks: 2, maxTicks: 8, reach })

  if (ctx.msSinceForcedMove() < ctx.config.forcedMoveGrace) {
    await waitTicks(bot, 8)
  }

  bot.emit('pathfinder:open_start', pos)
  throwIfAborted(signal, pos)
  return await clickAndWait(ctx, block, pos, aim ? aim.face : new Vec3(0, 1, 0), windowTimeout, signal)
}

/** Open any window with mineflayer's helpers; remove its listener on timeout.
 * Otherwise a later unrelated window could resolve the abandoned request. */
async function clickAndWait (
  ctx: ActionContext,
  block: BlockLike,
  pos: Vec3,
  face: Vec3,
  timeoutMs: number,
  signal?: AbortSignal
): Promise<WindowLike> {
  const bot = ctx.bot
  const neighbour = pos.plus(face)
  let sawBlockUpdate = false

  const onBlockUpdate = (...args: unknown[]): void => {
    const updated = args[1] as { position?: Vec3 } | null
    if (updated?.position && (updated.position.equals(pos) || updated.position.equals(neighbour))) {
      sawBlockUpdate = true
    }
  }
  bot.on('blockUpdate', onBlockUpdate)

  const listenersBefore = new Set(listenersOf(bot, 'windowOpen'))

  try {
    const cursor = new Vec3(0.5 + face.x * 0.5, 0.5 + face.y * 0.5, 0.5 + face.z * 0.5)
    const opening: Promise<WindowLike> = typeof bot.openBlock === 'function'
      ? (bot.openBlock(block, face, cursor) as Promise<WindowLike>)
      : activateAndWait(ctx, block, face, cursor)
    // The timed-out operation may still reject later; consume that rejection.
    opening.catch(() => {})

    return await raceWindow(opening, timeoutMs, signal, () => {
      const verdict = sawBlockUpdate
        ? 'the server processed the click and refused it (block update, no window — interaction cooldown, pending position correction, or a plugin cancel?)'
        : 'the server never processed the click (no block update — out of its range, a rotation/hit mismatch, or a cancel?)'
      const err = new ActionError(
        ActionErrors.OPEN_FAILED, `No window after ${timeoutMs}ms: ${verdict}`, pos, true
      )
      ;(err as ActionError & { silentTimeout: boolean }).silentTimeout = !sawBlockUpdate
      return err
    })
  } finally {
    bot.removeListener('blockUpdate', onBlockUpdate)
    // Remove the listener left by openBlock's pending window wait.
    for (const l of listenersOf(bot, 'windowOpen')) {
      if (!listenersBefore.has(l)) bot.removeListener('windowOpen', l)
    }
  }
}

/** Fallback when the bot has no openBlock helper. */
async function activateAndWait (
  ctx: ActionContext,
  block: BlockLike,
  face: Vec3,
  cursor: Vec3
): Promise<WindowLike> {
  const bot = ctx.bot
  return await new Promise<WindowLike>((resolve, reject) => {
    const onOpen = (...args: unknown[]): void => {
      bot.removeListener('windowOpen', onOpen)
      const window = args[0] as WindowLike
      if (typeof window.close !== 'function') {
        window.close = () => { bot.closeWindow?.(window) }
      }
      resolve(window)
    }
    bot.on('windowOpen', onOpen)
    Promise.resolve(bot.activateBlock(block, face, cursor)).catch((err: unknown) => {
      bot.removeListener('windowOpen', onOpen)
      reject(err as Error)
    })
  })
}

async function raceWindow (
  opening: Promise<WindowLike>,
  timeoutMs: number,
  signal: AbortSignal | undefined,
  makeError: () => Error
): Promise<WindowLike> {
  let timer: ReturnType<typeof setTimeout> | null = null
  let onAbort: (() => void) | null = null
  try {
    return await Promise.race([
      opening,
      new Promise<never>((_resolve, reject) => {
        timer = setTimeout(() => reject(makeError()), timeoutMs)
        if (signal) {
          onAbort = () => reject(new ActionError(ActionErrors.ABORTED, 'Open was aborted.'))
          if (signal.aborted) onAbort()
          else signal.addEventListener('abort', onAbort, { once: true })
        }
      })
    ])
  } finally {
    if (timer) clearTimeout(timer)
    if (signal && onAbort) signal.removeEventListener('abort', onAbort)
  }
}

function listenersOf (bot: unknown, event: string): Array<(...args: unknown[]) => void> {
  const fn = (bot as { listeners?: (e: string) => Array<(...a: unknown[]) => void> }).listeners
  if (typeof fn !== 'function') return []
  return fn.call(bot, event) ?? []
}
