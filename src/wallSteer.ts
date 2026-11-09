// Shared steering keeps simulated ground movement consistent with the executor.

/** Collision probe for the body's box with feet at (x, y, z). */
export type BoxProbe = (x: number, y: number, z: number) => boolean

/** Walking heading with the step's component lengths, without normalisation. */
export interface Heading { dx: number, dz: number }

/** Height difference above which the node is a step or climb; steering around it can make it unreachable. */
export const STEP_UP_MIN = 0.1
/** Maximum walking-step length; longer steps are jumps and must retain their heading. */
export const WALK_STEP_REACH = 1.8
/** Collision-box inset on each side, matching geometry.ts EPS. */
export const PROBE_INSET = 0.02
/** Side probe exceeds the sprint margin plus box inset to detect flush wall contact. */
export const SIDE_TOUCH = 0.06
/**
 * Standoff as a fraction of along-wall motion keeps reported positions outside the wall.
 * Client box reconstruction and server centre-based rounding can disagree at exact contact.
 */
export const WALL_STANDOFF = 0.25
/** Wall-probe look-ahead, slightly more than one tick of sprinting. */
export const SLIDE_PROBE = 0.35
/** Ignore small heading components so near-parallel travel does not trigger wall sliding. */
export const SLIDE_AXIS_MIN = 0.2
/** Sprint clearance margin; exact contact after a horizontal collision counts as against the wall. */
export const SPRINT_WALL_MARGIN = 0.03
/** Sprint gate look-ahead distance for one tick. */
export const SPRINT_TICK = 0.3

/** Whether floor supports a landing at offset (ox, oz) from the body. */
function floorAway (hit: BoxProbe, x: number, y: number, z: number, ox: number, oz: number): boolean {
  return hit(x + ox, y - 0.55, z + oz)
}

/** Steer away from flush side-wall contact by WALL_STANDOFF, only where floor continues. */
export function sideStandoff (hit: BoxProbe, x: number, y: number, z: number, h: Heading): void {
  const negX = hit(x - SIDE_TOUCH, y, z)
  const posX = hit(x + SIDE_TOUCH, y, z)
  const negZ = hit(x, y, z - SIDE_TOUCH)
  const posZ = hit(x, y, z + SIDE_TOUCH)
  if (negX !== posX && Math.abs(h.dz) >= Math.abs(h.dx)) {
    const away = negX ? 1 : -1
    if (floorAway(hit, x, y, z, away * 0.3, 0)) h.dx += away * Math.abs(h.dz) * WALL_STANDOFF
  } else if (negZ !== posZ && Math.abs(h.dx) >= Math.abs(h.dz)) {
    const away = negZ ? 1 : -1
    if (floorAway(hit, x, y, z, 0, away * 0.3)) h.dz += away * Math.abs(h.dx) * WALL_STANDOFF
  }
}

/**
 * Slide along walls obstructing (slideDx, slideDz), using the node heading
 * (nodeDx, nodeDz) with outward standoff where supported. Return whether a wall blocks the step.
 */
export function wallSlide (
  hit: BoxProbe, x: number, y: number, z: number,
  slideDx: number, slideDz: number, nodeDx: number, nodeDz: number, h: Heading
): boolean {
  const len = Math.hypot(slideDx, slideDz) || 1
  const sx = Math.abs(slideDx) / len < SLIDE_AXIS_MIN ? 0 : Math.sign(slideDx)
  const sz = Math.abs(slideDz) / len < SLIDE_AXIS_MIN ? 0 : Math.sign(slideDz)
  const xBlocked = sx !== 0 && hit(x + sx * SLIDE_PROBE, y, z)
  const zBlocked = sz !== 0 && hit(x, y, z + sz * SLIDE_PROBE)
  if (!zBlocked && !xBlocked) return false
  h.dx = nodeDx
  h.dz = nodeDz
  // Replace into-wall motion with standoff only when there is an along-wall component.
  if (zBlocked && !xBlocked && Math.abs(h.dx) >= 0.15 && floorAway(hit, x, y, z, 0, -sz * 0.3)) h.dz = -sz * Math.abs(h.dx) * WALL_STANDOFF
  else if (xBlocked && !zBlocked && Math.abs(h.dz) >= 0.15 && floorAway(hit, x, y, z, -sx * 0.3, 0)) h.dx = -sx * Math.abs(h.dz) * WALL_STANDOFF
  return true
}

/** Whether (dx, dz) reaches a wall within a sprinting tick; `near` means current wall contact. */
export function againstWall (hit: BoxProbe, near: boolean, x: number, y: number, z: number, dx: number, dz: number): boolean {
  if (near) return true
  const len = Math.hypot(dx, dz)
  return len > 0.01 && hit(x + (dx / len) * SPRINT_TICK, y, z + (dz / len) * SPRINT_TICK)
}
