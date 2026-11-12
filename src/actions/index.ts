import { ActionError, ActionErrors, DEFAULT_ACTION_CONFIG } from './types.js'
import type { ActionConfig, ActionName, ActionTable, ActivateOptions, BlockLike } from './types.js'
import type { ActionContext } from './context.js'
import { createDig, throwIfAborted } from './dig.js'
import { createPlace } from './place.js'
import { createOpen } from './open.js'
import { Pacer } from './pacing.js'
import {
  cellCentre, fmt, hasLineOfSight, inReach, pickFace, reachToCell, eyePos, settleAim, untilStopped, waitTicks
} from './reach.js'

type AnyAction = (...args: unknown[]) => Promise<unknown>

/** One `actions.wrap`, linked to its neighbours so that it can leave from anywhere in the stack. */
interface WrapLayer {
  call: AnyAction
  below: AnyAction
  above: WrapLayer | null
}

export function createActionTable (host: Omit<ActionContext, 'equip'>): ActionTable {
  const ctx: ActionContext = { ...host, equip: async (item, destination) => await table.equip(item, destination) }
  const slots = (): Record<ActionName, AnyAction> => table as unknown as Record<ActionName, AnyAction>
  const layers = new WeakMap<AnyAction, WrapLayer>()
  const table: ActionTable = {
    config: ctx.config,
    dig: createDig(ctx),
    place: createPlace(ctx),
    open: createOpen(ctx),
    activate: createActivate(ctx),
    equip: async (item: unknown, destination = 'hand') => {
      await untilStopped(ctx.bot, () => ctx.bot.equip(item, destination))
    },
    wrap (name, wrapper) {
      const layer: WrapLayer = { call: async () => {}, below: slots()[name], above: null }
      const wrapped = (wrapper as unknown as (next: AnyAction) => AnyAction)(async (...args) => await layer.below(...args))
      let active = true
      // The pass-through serves anyone who kept a reference to this layer before it left.
      layer.call = async (...args) => await (active ? wrapped(...args) : layer.below(...args))
      const under = layers.get(layer.below)
      if (under !== undefined) under.above = layer
      layers.set(layer.call, layer)
      slots()[name] = layer.call
      return () => {
        if (!active) return
        active = false
        const lower = layers.get(layer.below)
        if (layer.above !== null) {
          layer.above.below = layer.below
          if (lower !== undefined) lower.above = layer.above
        } else if (slots()[name] === layer.call) {
          slots()[name] = layer.below
          if (lower !== undefined) lower.above = null
        }
      }
    }
  }
  return table
}

/** Right-click without waiting for a window, using the same reach and aim guards as other actions. */
function createActivate (ctx: ActionContext) {
  return async function activate (block: BlockLike, options: ActivateOptions = {}): Promise<void> {
    const bot = ctx.bot
    const cfg = ctx.config
    const pos = block.position.floored()
    const reach = options.reach ?? cfg.reach
    const retries = Math.max(1, options.retries ?? cfg.retries)

    let lastError: Error | null = null
    for (let attempt = 1; attempt <= retries; attempt++) {
      throwIfAborted(options.signal, pos)
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
        continue
      }

      const settle = options.settle !== false
      const aim = settle ? pickFace(bot, pos, { requireCursor: false, allowGeometric: true, reach }) : null
      if (settle) {
        const point = aim ? aim.point : cellCentre(pos)
        if (aim && !hasLineOfSight(bot, pos, point, reach)) {
          lastError = new ActionError(ActionErrors.OCCLUDED, `No line of sight to ${fmt(pos)}.`, pos, true)
          await waitTicks(bot, 4)
          continue
        }
        await settleAim(bot, pos, point, { minTicks: 2, maxTicks: 6, reach })
      }

      const sneak = options.sneak === true
      try {
        if (sneak) {
          bot.setControlState('sneak', true)
          await waitTicks(bot, 2)
        }
        await untilStopped(bot, () => bot.activateBlock(live, aim ? aim.face : undefined), options.signal)
        bot.emit('pathfinder:activate', pos, live)
        return
      } catch (err) {
        if (err instanceof ActionError && err.name === ActionErrors.ABORTED) throw err
        lastError = err as Error
        await waitTicks(bot, 4)
      } finally {
        if (sneak) bot.setControlState('sneak', false)
      }
    }

    throw new ActionError(
      ActionErrors.ACTIVATE_FAILED,
      `Could not activate the block at ${fmt(pos)}: ${lastError?.message ?? 'unknown'}`, pos
    )
  }
}

export { Pacer, DEFAULT_ACTION_CONFIG }
export type { ActionConfig, ActionContext, ActionTable }
