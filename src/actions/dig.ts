// Refuse invalid swings and verify the world update; client-side dig completion is not proof of a break.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { BlockLike, DigOptions } from './types.js'
import type { ActionContext } from './context.js'
import { tpsScale } from './pacing.js'
import {
  cellCentre, fmt, inReach, pickFace, reachToCell, eyePos,
  settleAim, settleGrounded, waitTicks
} from './reach.js'

/** How far the bot may drift mid-dig before the swing is abandoned. */
const DISPLACEMENT_LIMIT = 0.4

/** Consecutive airborne samples that mean the footing is gone. */
const AIRBORNE_SAMPLES = 2

/** How often the mid-dig watchdog looks, ms. */
const WATCH_INTERVAL = 50

/** Slack added to the dig-time estimate before the wall-clock timeout fires. */
const DIG_TIMEOUT_SLACK = 2000

/** Ticks a break waits for the server's answer when no pacing cooldown already gives it the time. */
const REFUSAL_TICKS = 4

export function createDig (ctx: ActionContext) {
  return async function dig (block: BlockLike, options: DigOptions = {}): Promise<void> {
    const cfg = ctx.config
    const retries = options.retries ?? cfg.retries
    const reach = options.reach ?? cfg.reach
    const pacing = options.pacing ?? cfg.pacing
    const deadline = deadlineFrom(options.timeout ?? cfg.timeout)
    const pos = block.position.floored()
    const wantName = block.name

    let lastError: Error | null = null

    for (let attempt = 1; attempt <= Math.max(1, retries); attempt++) {
      throwIfAborted(options.signal, pos)
      if (Date.now() > deadline) {
        throw new ActionError(
          ActionErrors.DIG_FAILED,
          `Dig at ${fmt(pos)} ran out of time after ${attempt - 1} attempt(s).`, pos
        )
      }

      // Retries need fresh state because the target can change while waiting.
      const live = ctx.bot.blockAt(pos, false)
      // Unloaded is not air: nothing is known about that cell.
      if (live === null || live === undefined) {
        throw new ActionError(ActionErrors.NO_TARGET, `The chunk at ${fmt(pos)} is not loaded.`, pos, true)
      }
      if (live.name === 'air' || live.name === 'cave_air' || live.name === 'void_air') {
        return // An absent block already satisfies the request.
      }
      if (live.name !== wantName) {
        // A replacement block is outside the requested action.
        throw new ActionError(
          ActionErrors.NO_TARGET,
          `Block at ${fmt(pos)} is ${live.name}, expected ${wantName}.`, pos, true
        )
      }

      try {
        await attemptDig(ctx, live, pos, { ...options, reach, pacing })
        return
      } catch (err) {
        // Cancelling a swing makes it fail, and that is not a failure to retry or to report.
        throwIfAborted(options.signal, pos)
        lastError = err as Error
        const refused = err instanceof ActionError && err.refused
        if (!refused && !(err instanceof ActionError)) {
          // Structural dig errors cannot be repaired by retrying.
          throw err
        }
        if (err instanceof ActionError && err.name === ActionErrors.OUT_OF_REACH && options.approach !== false) {
          const ok = await ctx.approach(pos, reach, options.signal)
          if (!ok) {
            throw new ActionError(
              ActionErrors.UNREACHABLE,
              `No path to a spot within ${reach} of ${fmt(pos)}.`, pos
            )
          }
          continue
        }
        // Retrying in the same tick would see the same state.
        await waitTicks(ctx.bot, 4)
      }
    }

    throw lastError ?? new ActionError(
      ActionErrors.DIG_FAILED, `Could not break the block at ${fmt(pos)}.`, pos
    )
  }
}

async function attemptDig (
  ctx: ActionContext,
  block: BlockLike,
  pos: Vec3,
  options: DigOptions & { reach: number }
): Promise<void> {
  const bot = ctx.bot
  const reach = options.reach

  if (!inReach(bot, pos, reach)) {
    throw new ActionError(
      ActionErrors.OUT_OF_REACH,
      `Block at ${fmt(pos)} is ${reachToCell(eyePos(bot), pos).toFixed(2)} away (reach ${reach}).`,
      pos, true
    )
  }

  // Equip before aiming so a held-item change does not interrupt rotation and swing ordering.
  if (options.equipTool !== false) {
    const tool = ctx.bestHarvestTool(block)
    if (tool && (bot.heldItem == null || (bot.heldItem as { type: number }).type !== (tool as { type: number }).type)) {
      await ctx.equip(tool, 'hand').catch(() => {})
    }
  }

  await settleAim(bot, pos, cellCentre(pos), { minTicks: 3, maxTicks: 5, reach })

  const aim = pickFace(bot, pos, {
    requireCursor: options.requireCursor !== false,
    allowGeometric: options.allowGeometric === true,
    reach
  })
  if (!aim) {
    throw new ActionError(
      ActionErrors.OCCLUDED,
      `No clickable face on ${fmt(pos)} — the cursor lands elsewhere.`, pos, true
    )
  }

  // Falling sends a swing from a position the server may not have accepted.
  if (options.requireGrounded !== false) {
    const grounded = await settleGrounded(bot, 20)
    if (!grounded) {
      throw new ActionError(
        ActionErrors.DIG_FAILED, `Still airborne over ${fmt(pos)}; not swinging.`, pos, true
      )
    }
  }

  const aimed = await settleAim(bot, pos, aim.point, {
    minTicks: 3, maxTicks: 8, wantFace: aim.id, reach
  })
  if (!aimed) {
    throw new ActionError(
      ActionErrors.OCCLUDED, `Could not settle the cursor on face ${aim.id} of ${fmt(pos)}.`, pos, true
    )
  }

  // Slow aiming leaves time for leaves or falling blocks to change.
  const stillThere = bot.blockAt(pos, false)
  if (!stillThere || stillThere.name !== block.name) {
    throw new ActionError(
      ActionErrors.NO_TARGET,
      `Block at ${fmt(pos)} changed while aiming (now ${stillThere ? stillThere.name : 'nothing'}).`,
      pos, true
    )
  }

  if (ctx.msSinceForcedMove() < ctx.config.forcedMoveGrace) {
    throw new ActionError(
      ActionErrors.DIG_FAILED,
      `Server corrected our position ${Math.round(ctx.msSinceForcedMove())}ms ago; holding the swing.`,
      pos, true
    )
  }
  if (!inReach(bot, pos, reach)) {
    throw new ActionError(
      ActionErrors.OUT_OF_REACH, `Drifted out of reach of ${fmt(pos)} during the settle.`, pos, true
    )
  }

  bot.emit('pathfinder:dig_start', stillThere)
  // After the event: a listener can abort, and a swing would miss a signal that has already fired.
  throwIfAborted(options.signal, pos)
  await swingWatched(ctx, stillThere, pos, aim.face, options)

  // Mineflayer writes air into its own world as it sends the finish, so the world proves nothing yet.
  // A server that refuses the break puts the block back a round trip later: wait for that first.
  const pacing = options.pacing ?? ctx.config.pacing
  await ctx.pacer.cooldown(pacing, t => waitTicks(bot, t))
  if (pacing === 'none') await waitTicks(bot, REFUSAL_TICKS)
  const after = bot.blockAt(pos, false)
  if (after === null || after === undefined) {
    throw new ActionError(ActionErrors.DIG_FAILED, `The chunk at ${fmt(pos)} unloaded during the dig; the result is unknown.`, pos, true)
  }
  const broke = after.name === 'air' || after.name === 'cave_air' || after.name === 'void_air'
  if (!broke) {
    throw new ActionError(
      ActionErrors.DIG_FAILED,
      `Dig at ${fmt(pos)} resolved but ${after.name} is still there.`, pos, true
    )
  }

  bot.emit('pathfinder:dig_finish', pos)
}

/** Cancel a dig when correction, lost footing or drift invalidates its reach and aim. */
async function swingWatched (
  ctx: ActionContext,
  block: BlockLike,
  pos: Vec3,
  face: Vec3,
  options: DigOptions
): Promise<void> {
  const bot = ctx.bot
  const from = bot.entity.position.clone()
  let aborted: string | null = null

  const abort = (why: string): void => {
    if (aborted) return
    aborted = why
    try { bot.stopDigging?.() } catch { /* best effort */ }
  }

  const onForcedMove = (): void => abort('server lagback')
  bot.on('forcedMove', onForcedMove)

  let airSamples = 0
  const watch = setInterval(() => {
    const e = bot.entity
    if (!e || aborted) return
    airSamples = e.onGround === false ? airSamples + 1 : 0
    if (airSamples >= AIRBORNE_SAMPLES) { abort('footing gone mid-dig'); return }
    if (e.position.distanceTo(from) > DISPLACEMENT_LIMIT) abort('displaced mid-dig')
  }, WATCH_INTERVAL)

  const onAbortSignal = (): void => abort('aborted by caller')
  options.signal?.addEventListener('abort', onAbortSignal, { once: true })

  // Pad client dig timing for slow servers, with a wall-clock backstop.
  const original = typeof block.digTime === 'function' ? block.digTime.bind(block) : null
  const scale = tpsScale(ctx.serverTps())
  let budget = DIG_TIMEOUT_SLACK
  if (original) {
    ;(block as { digTime: (...a: unknown[]) => number }).digTime = (...args: unknown[]) =>
      Math.ceil(original(...args) * 1.3 * scale) + 100
    // The backstop must outlast the wait mineflayer will make, with water, footing, enchantments and effects.
    const helper = (bot as { digTime?: (b: unknown) => number }).digTime
    try {
      const wait = typeof helper === 'function'
        ? helper.call(bot, block)
        : Math.ceil(original(bot.heldItem ? bot.heldItem.type : null) * 1.3 * scale) + 100
      budget = Math.ceil(wait) + DIG_TIMEOUT_SLACK
    } catch { /* keep the default */ }
  }

  let digError: unknown = null
  try {
    ctx.pacer.noteAction(bot.heldItem?.name ?? null)
    await withTimeout(ctx.rawDig(block, true, face), budget, () => {
      abort('dig timed out')
    })
  } catch (err) {
    digError = err
  } finally {
    clearInterval(watch)
    bot.removeListener('forcedMove', onForcedMove)
    options.signal?.removeEventListener('abort', onAbortSignal)
    if (original) (block as { digTime: (...a: unknown[]) => number }).digTime = original
  }

  if (aborted) {
    // START and CANCEL still count toward server break pacing.
    await ctx.pacer.cooldown(options.pacing ?? ctx.config.pacing, t => waitTicks(bot, t))
    bot.emit('pathfinder:dig_aborted', pos, aborted)
    throw new ActionError(ActionErrors.DIG_FAILED, `Dig at ${fmt(pos)} abandoned: ${aborted}.`, pos, true)
  }
  if (digError) throw digError
}

function deadlineFrom (timeout: number): number {
  return timeout > 0 ? Date.now() + timeout : Number.POSITIVE_INFINITY
}

function throwIfAborted (signal: AbortSignal | undefined, pos: Vec3): void {
  if (signal?.aborted === true) {
    throw new ActionError(ActionErrors.ABORTED, `Action on ${fmt(pos)} was aborted.`, pos)
  }
}

async function withTimeout<T> (promise: Promise<T>, ms: number, onTimeout: () => void): Promise<T> {
  if (!isFinite(ms) || ms <= 0) return await promise
  let timer: ReturnType<typeof setTimeout> | null = null
  try {
    return await Promise.race([
      promise,
      new Promise<never>((_resolve, reject) => {
        timer = setTimeout(() => {
          onTimeout()
          const err = new Error(`timed out after ${ms}ms`)
          err.name = 'Timeout'
          reject(err)
        }, ms)
      })
    ])
  } finally {
    if (timer) clearTimeout(timer)
  }
}

export { withTimeout, throwIfAborted, deadlineFrom }
