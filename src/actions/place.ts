// Sneak against interactable supports so placement does not open them.
// Verify world state even after rejection; a block-update wait can time out after placement lands.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { PlaceOptions } from './types.js'
import type { ActionContext } from './context.js'
import { interactableBlocks } from '../interactableBlocks.js'
import {
  bodyOverlaps, facePoint, fmt, hasLineOfSight, inReach, reachToCell, eyePos,
  settleAim, settleGrounded, untilStopped, waitTicks
} from './reach.js'
import { deadlineFrom, throwIfAborted } from './dig.js'

const INTERACTABLE = new Set(interactableBlocks)

/** Supporting directions in preference order: below, sides, then above.
 * A support below leaves a placed shulker box opening upward. */
const DEFAULT_FACES: Vec3[] = [
  new Vec3(0, -1, 0),
  new Vec3(0, 0, 1),
  new Vec3(0, 0, -1),
  new Vec3(1, 0, 0),
  new Vec3(-1, 0, 0),
  new Vec3(0, 1, 0)
]

const AIR_NAMES = new Set(['air', 'cave_air', 'void_air'])

export function createPlace (ctx: ActionContext) {
  return async function place (target: Vec3, options: PlaceOptions = {}): Promise<void> {
    const cfg = ctx.config
    const bot = ctx.bot
    const pos = target.floored()
    const retries = options.retries ?? cfg.retries
    const reach = options.reach ?? cfg.reach
    const deadline = deadlineFrom(options.timeout ?? cfg.timeout)

    const item = resolveItem(ctx, options.item)
    if (!item) {
      throw new ActionError(
        ActionErrors.MISSING_ITEM,
        `Nothing to place at ${fmt(pos)}${options.item ? ` — no ${describeItem(options.item)} in the inventory` : ' — no item given and the hand is empty'}.`,
        pos
      )
    }
    const wantName = item.name ?? null

    let lastError: Error | null = null

    for (let attempt = 1; attempt <= Math.max(1, retries); attempt++) {
      throwIfAborted(options.signal, pos)
      if (Date.now() > deadline) break

      const occupant = bot.blockAt(pos, false)
      if (occupant && !AIR_NAMES.has(occupant.name)) {
        // Existing blocks are accepted only when they match; placement must not break replacements.
        if (wantName === null || placedFrom(bot, occupant, item)) return
        throw new ActionError(
          ActionErrors.PLACE_FAILED,
          `${fmt(pos)} already holds ${occupant.name}; refusing to replace it.`, pos
        )
      }

      // The bot cannot stand where it is placing.
      if (bodyOverlaps(bot, pos)) {
        throw new ActionError(
          ActionErrors.PLACE_FAILED, `Standing in ${fmt(pos)}; step out before placing.`, pos, true
        )
      }

      const support = findSupport(ctx, pos, options, item.name ?? '', reach)
      if (!support) {
        // Not refused: walking elsewhere changes nothing about the faces around the cell.
        throw new ActionError(
          ActionErrors.PLACE_FAILED,
          `No supporting face for ${fmt(pos)} that leaves the block usable.`, pos
        )
      }

      if (!inReach(bot, support.ref, reach)) {
        if (options.approach === false) {
          throw new ActionError(
            ActionErrors.OUT_OF_REACH,
            `Support ${fmt(support.ref)} is ${reachToCell(eyePos(bot), support.ref).toFixed(2)} away (reach ${reach}).`,
            support.ref, true
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

      try {
        await attemptPlace(ctx, pos, support, item, { ...options, reach })
        return
      } catch (err) {
        lastError = err as Error
        if (err instanceof ActionError && !err.refused) throw err
        await waitTicks(bot, 4)
      }
    }

    throw lastError ?? new ActionError(
      ActionErrors.PLACE_FAILED, `Could not place at ${fmt(pos)}.`, pos
    )
  }
}

interface Support {
  /** Position of the block being clicked. */
  ref: Vec3
  /** Face vector as `placeBlock` wants it: from the ref block toward the target. */
  faceVector: Vec3
  /** True when the reference block opens a window if clicked without sneaking. */
  interactable: boolean
}

/**
 * Find a supporting face, in preference order, preferring one the bot can reach and see now.
 * A shulker box opens away from its support, so for one the opposite side must be clear.
 */
function findSupport (ctx: ActionContext, pos: Vec3, options: PlaceOptions, itemName: string, reach: number): Support | null {
  const bot = ctx.bot
  const faces = options.faces ?? DEFAULT_FACES
  const requireOpenable = options.requireOpenable ?? itemName.endsWith('shulker_box')

  let first: Support | null = null
  for (const dir of faces) {
    const refPos = pos.plus(dir)
    const ref = bot.blockAt(refPos, false)
    // boundingBox includes all air variants, including cave_air.
    if (!ref || ref.boundingBox !== 'block') continue

    if (requireOpenable) {
      const openingPos = pos.minus(dir)
      const opening = bot.blockAt(openingPos, false)
      if (opening && opening.boundingBox !== 'empty') continue
    }

    const support: Support = {
      ref: refPos,
      faceVector: dir.scaled(-1), // ref → target
      interactable: INTERACTABLE.has(ref.name)
    }
    if (inReach(bot, refPos, reach) && hasLineOfSight(bot, refPos, facePoint(refPos, support.faceVector), reach)) return support
    first ??= support
  }
  return first
}

async function attemptPlace (
  ctx: ActionContext,
  pos: Vec3,
  support: Support,
  item: { type: number, name?: string },
  options: PlaceOptions & { reach: number }
): Promise<void> {
  const bot = ctx.bot
  const refBlock = bot.blockAt(support.ref, false)
  if (!refBlock) {
    throw new ActionError(ActionErrors.PLACE_FAILED, `Support at ${fmt(support.ref)} vanished.`, pos, true)
  }

  const aimPoint = facePoint(support.ref, support.faceVector)
  if (!hasLineOfSight(bot, support.ref, aimPoint, options.reach)) {
    throw new ActionError(
      ActionErrors.OCCLUDED, `No line of sight to the support face at ${fmt(support.ref)}.`, pos, true
    )
  }

  if (bot.heldItem == null || bot.heldItem.type !== item.type) {
    await ctx.equip(item, 'hand')
  }

  if (options.requireGrounded !== false) await settleGrounded(bot, 20)
  await settleAim(bot, support.ref, aimPoint, { minTicks: 3, maxTicks: 8, reach: options.reach })

  if (ctx.msSinceForcedMove() < ctx.config.forcedMoveGrace) {
    throw new ActionError(
      ActionErrors.PLACE_FAILED, 'Server corrected our position just now; holding the click.', pos, true
    )
  }

  // Sneaking prevents interactable reference blocks from opening during placement.
  const sneak = options.sneak !== false && (support.interactable || options.sneak === true)
  bot.emit('pathfinder:place_start', pos, item)
  try {
    if (sneak) {
      bot.setControlState('sneak', true)
      await waitTicks(bot, 2)
    }
    await untilStopped(bot, async () => {
      ctx.pacer.noteAction(item.name ?? null)
      await bot.placeBlock(refBlock, support.faceVector)
    }, options.signal)
  } catch (err) {
    if (err instanceof ActionError && err.name === ActionErrors.ABORTED) throw err
    // A delayed update can reject after placement succeeds; verify the world below.
  } finally {
    if (sneak) bot.setControlState('sneak', false)
  }

  await waitTicks(bot, options.settleTicks ?? 10)

  const placed = bot.blockAt(pos, false)
  const ok = placed && !AIR_NAMES.has(placed.name) && (item.name === undefined || placedFrom(bot, placed, item))
  if (!ok) {
    throw new ActionError(
      ActionErrors.PLACE_FAILED,
      `Nothing appeared at ${fmt(pos)} after the click${placed ? ` (still ${placed.name})` : ''}.`,
      pos, true
    )
  }

  // Breaks and placements share the same server action-rate budget.
  await ctx.pacer.cooldown(options.pacing ?? ctx.config.pacing, t => waitTicks(bot, t))
  bot.emit('pathfinder:place_finish', pos, placed)
}

/**
 * Whether this block is what placing `item` makes. Most items place a block of their own name; some place
 * another (torch on a wall, redstone, string, seeds, signs on a wall). Those drop the item, and no item
 * carries their name. That rule rejects stone for cobblestone, though stone drops it.
 */
export function placedFrom (bot: ActionContext['bot'], block: { name: string, type: number }, item: { type: number, name?: string }): boolean {
  if (item.name === undefined || block.name === item.name) return true
  if (block.name === 'wheat' && item.name === 'wheat_seeds') return true
  const registry = (bot as { registry?: { blocks?: Record<number, { drops?: unknown[] }>, itemsByName?: Record<string, unknown> } }).registry
  if (registry?.blocks === undefined || registry.itemsByName === undefined) return false
  if (registry.itemsByName[block.name] !== undefined) return false
  // minecraft-data lists a drop as an item id, or as { drop: id } or { drop: { id } }.
  const dropId = (d: unknown): number | undefined => {
    if (typeof d === 'number') return d
    const drop = (d as { drop?: unknown }).drop
    return typeof drop === 'number' ? drop : (drop as { id?: number } | undefined)?.id
  }
  return (registry.blocks[block.type]?.drops ?? []).some(d => dropId(d) === item.type)
}

function resolveItem (
  ctx: ActionContext,
  want: PlaceOptions['item']
): { type: number, name?: string } | null {
  const items = ctx.bot.inventory.items()
  if (want === undefined || want === null) {
    return ctx.bot.heldItem ?? items[0] ?? null
  }
  if (typeof want === 'object') return want
  if (typeof want === 'number') return items.find(i => i.type === want) ?? null
  return items.find(i => i.name === want) ?? null
}

function describeItem (want: NonNullable<PlaceOptions['item']>): string {
  if (typeof want === 'object') return want.name ?? String(want.type)
  return String(want)
}
