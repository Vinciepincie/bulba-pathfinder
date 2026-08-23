// The default place.
//
// `bot.placeBlock(ref, face)` clicks a face and waits for a block update. Two
// things about that are wrong often enough to matter:
//
// 1. **Right-clicking an interactable block opens it.** Chests, barrels,
//    hoppers, crafting tables, furnaces — clicking one with a block in hand
//    opens its window instead of placing, exactly as it does in vanilla. The
//    fix is the same as in vanilla: sneak. A shulker station sitting on a
//    hopper is unplaceable without it, and the failure is silent — a window
//    opens, no block appears, and the next action finds a window it did not
//    open.
// 2. **`placeBlock` rejecting is not the same as the placement failing.** Its
//    block-update wait times out on a lagging server after the placement
//    landed. Treating the rejection as failure makes the bot place a second
//    block, or "fail" a placement that worked. The world is the only judge.
//
// The face order also matters for shulker boxes specifically: a shulker
// opens along the face it was placed against, and a shulker whose opening is
// sealed by another block can never be opened again. Preferring the support
// BELOW (so it opens upward) and refusing any face whose opening is blocked
// is what stops the bot from bricking its own storage.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { PlaceOptions } from './types.js'
import type { ActionContext } from './context.js'
import { interactableBlocks } from '../interactableBlocks.js'
import {
  bodyOverlaps, facePoint, fmt, hasLineOfSight, inReach, reachToCell, eyePos,
  settleAim, settleGrounded, waitTicks
} from './reach.js'
import { deadlineFrom, throwIfAborted } from './dig.js'

const INTERACTABLE = new Set(interactableBlocks)

/**
 * Faces to try, best first: the block below (opens upward), then the four
 * sides, then hung from a ceiling. Expressed as the direction from the target
 * cell TOWARD the supporting block.
 */
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
        // Already filled. If it is what we wanted, we are done; if not, the
        // caller asked for something this action must not do (break it).
        if (wantName === null || occupant.name === wantName) return
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

      const support = findSupport(ctx, pos, options)
      if (!support) {
        throw new ActionError(
          ActionErrors.PLACE_FAILED,
          `No supporting face for ${fmt(pos)} that leaves the block usable.`, pos, true
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
 * Find a block to place against. Rejects any face whose "opening" side is
 * obstructed, which is what keeps a placed shulker box openable.
 */
function findSupport (ctx: ActionContext, pos: Vec3, options: PlaceOptions): Support | null {
  const bot = ctx.bot
  const faces = options.faces ?? DEFAULT_FACES
  const requireOpenable = options.requireOpenable !== false

  for (const dir of faces) {
    const refPos = pos.plus(dir)
    const ref = bot.blockAt(refPos, false)
    // boundingBox, not the name: `cave_air` is not `air`, and a name check
    // misses half the world.
    if (!ref || ref.boundingBox !== 'block') continue

    if (requireOpenable) {
      const openingPos = pos.minus(dir)
      const opening = bot.blockAt(openingPos, false)
      if (opening && opening.boundingBox !== 'empty') continue
    }

    return {
      ref: refPos,
      faceVector: dir.scaled(-1), // ref → target
      interactable: INTERACTABLE.has(ref.name)
    }
  }
  return null
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
    await bot.equip(item, 'hand')
  }

  if (options.requireGrounded !== false) await settleGrounded(bot, 20)
  await settleAim(bot, support.ref, aimPoint, { minTicks: 3, maxTicks: 8, reach: options.reach })

  if (ctx.msSinceForcedMove() < ctx.config.forcedMoveGrace) {
    throw new ActionError(
      ActionErrors.PLACE_FAILED, 'Server corrected our position just now; holding the click.', pos, true
    )
  }

  // Sneak whenever the reference block would otherwise open. Doing it
  // unconditionally is also fine and slightly safer, which is why it is the
  // default — sneaking never prevents a placement.
  const sneak = options.sneak !== false && (support.interactable || options.sneak === true)
  bot.emit('pathfinder:place_start', pos, item)
  if (sneak) {
    bot.setControlState('sneak', true)
    await waitTicks(bot, 2)
  }
  try {
    ctx.pacer.noteAction(item.name ?? null)
    await bot.placeBlock(refBlock, support.faceVector)
  } catch {
    // Deliberately swallowed: placeBlock's block-update wait rejects on a
    // lagging server even when the placement landed. The verify below is the
    // only judge.
  } finally {
    if (sneak) bot.setControlState('sneak', false)
  }

  await waitTicks(bot, options.settleTicks ?? 10)

  const placed = bot.blockAt(pos, false)
  const ok = placed && !AIR_NAMES.has(placed.name) && (item.name === undefined || placed.name === item.name)
  if (!ok) {
    throw new ActionError(
      ActionErrors.PLACE_FAILED,
      `Nothing appeared at ${fmt(pos)} after the click${placed ? ` (still ${placed.name})` : ''}.`,
      pos, true
    )
  }

  // Placements share the break pacing window: a checker counting block
  // actions does not care which kind they were.
  await ctx.pacer.cooldown(options.pacing ?? ctx.config.pacing, t => waitTicks(bot, t))
  bot.emit('pathfinder:place_finish', pos, placed)
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
