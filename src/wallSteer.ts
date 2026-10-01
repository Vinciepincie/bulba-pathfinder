// Steering along walls: the executor's two corrections to a walking heading,
// as functions of collision probes alone — so the kernel flies the ground the
// way the executor walks it (gaitSearch.ts), from the same code. The reasons
// for each, and what they measured, are with their call sites in plugin.ts.

/** Does the body's box, its feet at (x, y, z), overlap a block (geometry.playerCollides)? */
export type BoxProbe = (x: number, y: number, z: number) => boolean

/** A walking heading being corrected (not normalised: its parts are the step's). */
export interface Heading { dx: number, dz: number }

/**
 * Above this, the next node is something to climb ONTO, not a wall to walk
 * around: a shelf, a step, a ladder exit. Steering away from it there is how a
 * wall-slide turns a one-block step-up into an unreachable node.
 */
export const STEP_UP_MIN = 0.1
/**
 * A walking step is at most a diagonal; anything longer is a jump and must
 * not be re-steered along a wall.
 */
export const WALK_STEP_REACH = 1.8
/** The inset of a box probe on every side (geometry.ts EPS). */
export const PROBE_INSET = 0.02
/**
 * Sideways probe for a body FLUSH with a wall it is walking along. Slightly
 * more than SPRINT_WALL_MARGIN plus the probe's own inset, so it fires on
 * exactly the contact the sprint gate refuses.
 */
export const SIDE_TOUCH = 0.06
/**
 * Steering bias AWAY from a wall being slid along, as a fraction of the
 * along-wall component. Sliding with a heading exactly parallel is not enough:
 * whatever momentum the body still carries into the wall gets clamped, and a
 * clamped position sits EXACTLY on the block face — the one value the server
 * disagrees about, because prismarine-physics reconstructs the body from
 * `minZ + halfWidth` while the server reconstructs it from the centre, and
 * the two round to opposite sides of the boundary (the 1.21 hitbox precision
 * class — see geometry.ts). A hair of standoff keeps every position claimed
 * unambiguously outside the block.
 */
export const WALL_STANDOFF = 0.25
/** How far ahead the wall-slide probe looks — over one tick of sprinting. */
export const SLIDE_PROBE = 0.35
/**
 * The wall-slide probes only the axes the step actually moves along: a
 * component below this fraction of it is a line running a few degrees off a
 * wall, not into it.
 */
export const SLIDE_AXIS_MIN = 0.2
/**
 * How far from a wall the body has to be for sprinting to be safe. Any
 * positive margin makes exact contact — what prismarine-physics leaves after
 * every horizontal collision — count as "against the wall".
 */
export const SPRINT_WALL_MARGIN = 0.03
/** Ground distance one sprinting tick covers, the sprint gate's look-ahead. */
export const SPRINT_TICK = 0.3

/** Is there floor a body could land on at the body's level, `ox`, `oz` off its place? */
function floorAway (hit: BoxProbe, x: number, y: number, z: number, ox: number, oz: number): boolean {
  return hit(x + ox, y - 0.55, z + oz)
}

/**
 * Side-wall standoff: a body flush with a wall it is walking ALONG is
 * steered off it, by WALL_STANDOFF of the along-wall part, where the ground
 * continues that way.
 */
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
 * Wall-slide: a wall in the way of the step (`slideDx`, `slideDz`: the line
 * being walked) turns the heading along it — the step to the node itself
 * (`nodeDx`, `nodeDz`), its into-wall part replaced by a standoff where the
 * ground continues away from the wall. Returns whether a wall was in the way
 * (the caller's corner cut ends there).
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
  // The standoff is a fraction of the ALONG-wall part and REPLACES the
  // into-wall one; with nothing to slide along (a step square-on to the
  // face) the heading is left alone.
  if (zBlocked && !xBlocked && Math.abs(h.dx) >= 0.15 && floorAway(hit, x, y, z, 0, -sz * 0.3)) h.dz = -sz * Math.abs(h.dx) * WALL_STANDOFF
  else if (xBlocked && !zBlocked && Math.abs(h.dz) >= 0.15 && floorAway(hit, x, y, z, -sx * 0.3, 0)) h.dx = -sx * Math.abs(h.dz) * WALL_STANDOFF
  return true
}

/** Would a body heading (`dx`, `dz`) touch a wall within a sprinting tick's travel? (`near`: it is touching one now.) */
export function againstWall (hit: BoxProbe, near: boolean, x: number, y: number, z: number, dx: number, dz: number): boolean {
  if (near) return true
  const len = Math.hypot(dx, dz)
  return len > 0.01 && hit(x + (dx / len) * SPRINT_TICK, y, z + (dz / len) * SPRINT_TICK)
}
