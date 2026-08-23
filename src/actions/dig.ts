// The default dig.
//
// `bot.dig(block)` is one packet sequence with no guards around it. Every
// failure below is one this file exists to stop, and each one presents to the
// application as "the bot just stood there":
//
//   - out of the server's reach, so the START packet is dropped in silence
//   - aimed at a face the server derives differently from the rotation it has
//   - swung while airborne or sliding, from a position not yet agreed
//   - swung within the correction window after a lagback
//   - the block decayed/changed between the decision and the swing
//   - the footing broke mid-dig and the bot is now falling somewhere else
//   - the dig "succeeded" client-side but the block is still there
//
// The contract: this either breaks the block, or throws. A refusal (out of
// reach, occluded, lagback) throws with `refused = true` so the caller can
// reposition and retry rather than treating it as a hard failure. It never
// re-hammers a block the server has already dropped a click for.
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

      // Re-read every attempt: the world moved on while we were failing.
      const live = ctx.bot.blockAt(pos, false)
      if (!live || live.name === 'air' || live.name === 'cave_air' || live.name === 'void_air') {
        return // already gone — that is a success, whoever did it
      }
      if (live.name !== wantName) {
        // Something else is there now. Breaking it would be a different act
        // than the caller asked for.
        throw new ActionError(
          ActionErrors.NO_TARGET,
          `Block at ${fmt(pos)} is ${live.name}, expected ${wantName}.`, pos, true
        )
      }

      try {
        await attemptDig(ctx, live, pos, { ...options, reach, pacing })
        return
      } catch (err) {
        lastError = err as Error
        const refused = err instanceof ActionError && err.refused
        if (!refused && !(err instanceof ActionError)) {
          // A genuine dig error from mineflayer — surface it rather than
          // spending the remaining attempts on something structural.
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
        // Give the world a moment to settle before the next attempt; a retry
        // fired on the same tick would hit the same state.
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

  // 1. Vanilla range, measured the way the server measures it.
  if (!inReach(bot, pos, reach)) {
    throw new ActionError(
      ActionErrors.OUT_OF_REACH,
      `Block at ${fmt(pos)} is ${reachToCell(eyePos(bot), pos).toFixed(2)} away (reach ${reach}).`,
      pos, true
    )
  }

  // 2. Tool first: equipping AFTER aiming re-sends a held-item change between
  // the rotation and the swing, and the swing is what the server times.
  if (options.equipTool !== false) {
    const tool = ctx.bestHarvestTool(block)
    if (tool && (bot.heldItem == null || (bot.heldItem as { type: number }).type !== (tool as { type: number }).type)) {
      await bot.equip(tool, 'hand').catch(() => {})
    }
  }

  // 3. Aim at the block and let the rotation reach the server.
  await settleAim(bot, pos, cellCentre(pos), { minTicks: 3, maxTicks: 5, reach })

  // 4. Pick the face the server will agree with.
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

  // 5. Feet planted. A swing sent while falling is sent from a position the
  // server has not accepted yet.
  if (options.requireGrounded !== false) {
    const grounded = await settleGrounded(bot, 20)
    if (!grounded) {
      throw new ActionError(
        ActionErrors.DIG_FAILED, `Still airborne over ${fmt(pos)}; not swinging.`, pos, true
      )
    }
  }

  // 6. Re-aim precisely at the chosen face and confirm the cursor agrees.
  const aimed = await settleAim(bot, pos, aim.point, {
    minTicks: 3, maxTicks: 8, wantFace: aim.id, reach
  })
  if (!aimed) {
    throw new ActionError(
      ActionErrors.OCCLUDED, `Could not settle the cursor on face ${aim.id} of ${fmt(pos)}.`, pos, true
    )
  }

  // 7. Decay guard: leaves and falling blocks change under a slow aim.
  const stillThere = bot.blockAt(pos, false)
  if (!stillThere || stillThere.name !== block.name) {
    throw new ActionError(
      ActionErrors.NO_TARGET,
      `Block at ${fmt(pos)} changed while aiming (now ${stillThere ? stillThere.name : 'nothing'}).`,
      pos, true
    )
  }

  // 8. Stance guard: never swing inside the correction window.
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

  // 9. Swing, watched.
  bot.emit('pathfinder:dig_start', stillThere)
  await swingWatched(ctx, stillThere, pos, aim.face, options)

  // 10. Judge by the world, not by the promise. mineflayer resolves its dig
  // on its own timer; the block is the only authority on whether it broke.
  const after = bot.blockAt(pos, false)
  const broke = !after || after.name === 'air' || after.name === 'cave_air' || after.name === 'void_air'
  if (!broke) {
    throw new ActionError(
      ActionErrors.DIG_FAILED,
      `Dig at ${fmt(pos)} resolved but ${after.name} is still there.`, pos, true
    )
  }

  await ctx.pacer.cooldown(options.pacing ?? ctx.config.pacing, t => waitTicks(bot, t))
  bot.emit('pathfinder:dig_finish', pos)
}

/**
 * Run the dig with a watchdog that cancels it the moment the swing stops
 * being valid — a lagback, the footing decaying out from under the bot, or
 * being pushed. Letting the dig run on in those cases is how a bot keeps
 * mining a block it can no longer see.
 */
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

  // Pad the dig-time estimate so mineflayer's own timer does not fire early
  // on a server running behind, and derive a wall-clock backstop from it.
  const original = typeof block.digTime === 'function' ? block.digTime.bind(block) : null
  const scale = tpsScale(ctx.serverTps())
  let budget = DIG_TIMEOUT_SLACK
  if (original) {
    try { budget = Math.ceil(original(bot.heldItem ? bot.heldItem.type : null) * scale) + DIG_TIMEOUT_SLACK } catch { /* keep the default */ }
    ;(block as { digTime: (...a: unknown[]) => number }).digTime = (...args: unknown[]) =>
      Math.ceil(original(...args) * 1.3 * scale) + 100
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
    // The START (and its CANCEL) still counted against the server's break
    // pacing, so pay the cooldown even though nothing broke.
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
