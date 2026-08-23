// Assembles the overridable interaction table.
import { ActionError, ActionErrors, DEFAULT_ACTION_CONFIG } from './types.js'
import type { ActionConfig, ActionTable, ActivateOptions, BlockLike } from './types.js'
import type { ActionContext } from './context.js'
import { createDig } from './dig.js'
import { createPlace } from './place.js'
import { createOpen } from './open.js'
import { Pacer } from './pacing.js'
import {
  cellCentre, fmt, hasLineOfSight, inReach, pickFace, reachToCell, eyePos, settleAim, waitTicks
} from './reach.js'

export function createActionTable (ctx: ActionContext): ActionTable {
  const table: ActionTable = {
    config: ctx.config,
    dig: createDig(ctx),
    place: createPlace(ctx),
    open: createOpen(ctx),
    activate: createActivate(ctx),
    equip: async (item: unknown, destination = 'hand') => {
      await ctx.bot.equip(item, destination)
    }
  }
  return table
}

/**
 * Right-click a block without expecting a window: a lever, a button, a door,
 * a note block. Same reach and aim discipline as the rest; no window wait.
 */
function createActivate (ctx: ActionContext) {
  return async function activate (block: BlockLike, options: ActivateOptions = {}): Promise<void> {
    const bot = ctx.bot
    const cfg = ctx.config
    const pos = block.position.floored()
    const reach = options.reach ?? cfg.reach
    const retries = Math.max(1, options.retries ?? cfg.retries)

    let lastError: Error | null = null
    for (let attempt = 1; attempt <= retries; attempt++) {
      if (options.signal?.aborted === true) {
        throw new ActionError(ActionErrors.ABORTED, `Activate on ${fmt(pos)} was aborted.`, pos)
      }
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
      if (sneak) {
        bot.setControlState('sneak', true)
        await waitTicks(bot, 2)
      }
      try {
        await bot.activateBlock(live, aim ? aim.face : undefined)
        bot.emit('pathfinder:activate', pos, live)
        return
      } catch (err) {
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
