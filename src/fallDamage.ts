// A damaging landing removes speed through a later server velocity packet.

import { SIM_SLIME, SIM_CUSHION, SIM_BED } from './playerSim.js'

/** Blocks a body falls before a landing can hurt (the safe_fall_distance attribute). */
export const SAFE_FALL = 3

/**
 * The deepest drop a plan may choose with `spare` health to lose. A jump's rise adds up to
 * 1.25 blocks to the fall, so a chosen drop never costs all of `spare`, and 2 blocks never hurt.
 */
export function dropForHealth (spare: number): number {
  return Math.max(2, SAFE_FALL - 2 + Math.floor(spare))
}

/** Hop searches reject touchdowns causing this many fall-damage points or more.
 * Sneaked slime landings take the whole fall, including in deep bounce searches. */
export const FALL_VETO = 5

/** The hop veto with `spare` health to lose: a touchdown may not cost all of it. */
export function fallVetoForHealth (spare: number): number {
  return Math.min(FALL_VETO, Math.max(1, Math.ceil(spare)))
}

/**
 * Whether a landing hurts: `fall` sums descent, `last` is its landing-tick share, `kind` holds support flags.
 * With `whole` (1.21.5+), count the full fall and round down; otherwise exclude `last` and round up.
 */
export function landingHurts (fall: number, last: number, kind: number, sneaking: boolean, whole: boolean): boolean {
  return fallDamagePoints(fall, last, kind, sneaking, whole) >= 1
}

/** Landing damage in health points, using the same fall accounting as landingHurts. */
export function fallDamagePoints (fall: number, last: number, kind: number, sneaking: boolean, whole: boolean): number {
  // Sneaking disables the slime bounce and its fall-damage protection.
  if ((kind & SIM_SLIME) !== 0 && !sneaking) return 0
  const reach = (kind & SIM_BED) !== 0 ? 0.5 : 1
  const kept = (kind & SIM_CUSHION) !== 0 ? 0.2 : 1
  if (whole) return Math.max(0, Math.floor((fall * reach + 1e-6 - SAFE_FALL) * kept))
  return Math.max(0, Math.ceil(((fall - last) * reach - SAFE_FALL) * kept))
}
