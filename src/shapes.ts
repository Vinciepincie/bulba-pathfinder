// Port of mineflayer-pathfinder/lib/shapes.js (MIT), verbatim math — plus
// the top-catch classification shared by the LUT and the executor.
import { Vec3 } from 'vec3'

/** Centered landing half-extent per topCatchClass, in blocks: full/wide,
 * head/wall post, flower pot, fence post. */
export const CATCH_HALF: readonly number[] = [0.5, 0.25, 0.1875, 0.125]

/**
 * How much of the top of this shape set can actually CATCH a landing body.
 *
 * Every face within step height (0.6) of the collision top is projected
 * onto XZ, and the largest centered square inside any ONE of them gives the
 * class: 0 = full/wide (slabs, stairs, chests — any face a landing can
 * catch and walk up), 1 = the 0.5-wide class (heads, wall posts), 2 = the
 * flower-pot class (0.375 wide), 3 = the fence-post class (0.25 wide) — and
 * anything narrower than that, or a top that does not even hold the body's
 * centre (a ladder's 3/16 edge), which the solver refuses by its CLIMBABLE
 * flag. One box at a time, not the union: a fence with arms projects to a
 * cross whose bounding box is the whole cell, but a body landing off the
 * post's line finds nothing under it. The solver plans narrow landings with
 * the reduced per-axis credit (parkourEnvelope.ts) and the executor caps
 * its takeoff creep with it.
 */
export function topCatchClass (shapes: readonly number[][]): number {
  let top = 0
  for (const s of shapes) {
    if (s[4] > top) top = s[4]
  }
  if (top <= 0) return 0
  const lim = top - 0.6
  let c = -1
  for (const s of shapes) {
    if (s[4] <= lim) continue
    const ci = Math.min(0.5 - s[0], s[3] - 0.5, 0.5 - s[2], s[5] - 0.5)
    if (ci > c) c = ci
  }
  return c >= 0.4 ? 0 : c >= 0.2 ? 1 : c >= 0.15 ? 2 : 3
}

export function getShapeFaceCenters (shapes: number[][], direction: Vec3, half: 'top' | 'bottom' | null = null): Vec3[] {
  const faces: Vec3[] = []
  for (const shape of shapes) {
    const halfsize = new Vec3(shape[3] - shape[0], shape[4] - shape[1], shape[5] - shape[2]).scale(0.5)
    let center = new Vec3(shape[0] + shape[3], shape[1] + shape[4], shape[2] + shape[5]).scale(0.5)
    center = center.offset(halfsize.x * direction.x, halfsize.y * direction.y, halfsize.z * direction.z)

    if (half === 'top' && center.y <= 0.5) {
      if (Math.abs(direction.y) === 0) center.y += halfsize.y - 0.001
      if (center.y <= 0.5) continue
    } else if (half === 'bottom' && center.y >= 0.5) {
      if (Math.abs(direction.y) === 0) center.y -= halfsize.y - 0.001
      if (center.y >= 0.5) continue
    }

    faces.push(center)
  }
  return faces
}
