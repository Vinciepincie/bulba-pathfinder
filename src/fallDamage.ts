// Which landings hurt: the server's fall accounting (Entity.checkFallDamage,
// LivingEntity.calculateFallDamage and the landing block's fallOn), read from
// the 1.21.11 server. A landing that hurts is one whose speed the server
// takes away, a velocity packet later — see docs/PhysicsHops.md.

import { SIM_SLIME, SIM_CUSHION, SIM_BED } from './playerSim.js'

/** Blocks a body falls before a landing can hurt (the safe_fall_distance attribute). */
export const SAFE_FALL = 3

/**
 * Does this landing hurt? `fall`: every tick's descent since the ground was
 * left; `last`: the landing tick's share of it; `kind`: the SIM_* bits of
 * what the feet came down on.
 *
 * From 1.21.5 (`whole`) the whole fall counts and the damage is rounded DOWN,
 * so the first point of it takes a full block past the safe three. Before,
 * the landing tick's descent is never counted and any excess is rounded UP.
 */
export function landingHurts (fall: number, last: number, kind: number, sneaking: boolean, whole: boolean): boolean {
  // (slime spares a body that bounces; sneaking is what refuses the bounce)
  if ((kind & SIM_SLIME) !== 0 && !sneaking) return false
  const reach = (kind & SIM_BED) !== 0 ? 0.5 : 1
  const kept = (kind & SIM_CUSHION) !== 0 ? 0.2 : 1
  if (whole) return Math.floor((fall * reach + 1e-6 - SAFE_FALL) * kept) >= 1
  return Math.ceil(((fall - last) * reach - SAFE_FALL) * kept) >= 1
}
