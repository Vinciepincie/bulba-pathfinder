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
 * onto XZ, and the largest centered square inside their UNION gives the
 * class: 0 = full/wide (slabs, stairs, chests — any face a landing can
 * catch and walk up), 1 = the 0.5-wide class (heads, wall posts), 2 = the
 * flower-pot class (0.375 wide), 3 = the fence-post class (0.25 wide) — and
 * anything narrower than that, or a top that does not even hold the body's
 * centre (a ladder's 3/16 edge), which the solver refuses by its CLIMBABLE
 * flag.
 *
 * The union, not each box alone: an upside-down stair is two boxes that TILE
 * its cell, and its top is as flat and as full as a slab's — measured one box
 * at a time it came out a fence post (class 3) and lost a landing the whole
 * top can catch. A fence keeps its class either way: its arms sit half a block
 * under the post, so the union is a cross and the largest centred square in a
 * cross is still the post. The solver plans narrow landings with the reduced
 * per-axis credit (parkourEnvelope.ts) and the executor caps its takeoff
 * creep with it.
 */
export function topCatchClass (shapes: readonly number[][]): number {
  let top = 0
  for (const s of shapes) {
    if (s[4] > top) top = s[4]
  }
  if (top <= 0) return 0
  const lim = top - 0.6
  const faces = shapes.filter(s => s[4] > lim)
  return covers(faces, 0.4) ? 0 : covers(faces, 0.2) ? 1 : covers(faces, 0.15) ? 2 : 3
}

/**
 * Is the centred square of half-extent `h` fully covered by the faces' XZ
 * projections? Exact for axis-aligned rects: the faces' own edges cut the
 * square into cells that each lie wholly inside a face or wholly outside
 * every one, so testing cell centres decides it.
 */
function covers (faces: ReadonlyArray<readonly number[]>, h: number): boolean {
  const lo = 0.5 - h
  const hi = 0.5 + h
  const cuts = (i0: number, i1: number): number[] => {
    const v = [lo, hi]
    for (const f of faces) {
      if (f[i0] > lo && f[i0] < hi) v.push(f[i0])
      if (f[i1] > lo && f[i1] < hi) v.push(f[i1])
    }
    v.sort((p, q) => p - q)
    return v
  }
  const xs = cuts(0, 3)
  const zs = cuts(2, 5)
  for (let i = 0; i + 1 < xs.length; i++) {
    const cx = (xs[i] + xs[i + 1]) / 2
    if (xs[i + 1] - xs[i] < 1e-9) continue
    for (let j = 0; j + 1 < zs.length; j++) {
      if (zs[j + 1] - zs[j] < 1e-9) continue
      const cz = (zs[j] + zs[j + 1]) / 2
      let inside = false
      for (const f of faces) {
        if (cx > f[0] && cx < f[3] && cz > f[2] && cz < f[5]) { inside = true; break }
      }
      if (!inside) return false
    }
  }
  return true
}

/**
 * Where a NARROW support carries the body relative to its cell, per side —
 * the direction-dependent half of the catch class above. The class says how
 * wide the top is; it says nothing about where it sits, and the planner's
 * per-axis credits assume the middle. Three spare bits of the LutSpecial byte
 * (CARRY_SHIFT) carry one of seven patterns:
 *
 *   0  centred (or class 0): the class model as it stands — posts, heads,
 *      pots, bars, a lone pane, anything wide.
 *   1–4  an EDGE PANEL: the face is a strip against the cell's W/E/N/S face
 *      (an open trapdoor, a ladder's top). Toward the panel's own side the
 *      body is carried like a full block (0.8 past centre); toward the far
 *      side it must already be past centre (credit 0); along the panel, full.
 *   5, 6  a LINE along x / along z (a pane or fence line with both arms, a
 *      pane row): full along the line, the class across it.
 *
 * Anything else — an L of arms, a single arm — stays 0: the class model,
 * conservative where it is wrong. Only class > 0 supports get a code, so a
 * full block's plans cannot change.
 */
export const CARRY_SHIFT = 5
export const CARRY_W_NONE = 1
export const CARRY_E_NONE = 2
export const CARRY_N_NONE = 3
export const CARRY_S_NONE = 4
export const CARRY_LINE_X = 5
export const CARRY_LINE_Z = 6
/**
 * A narrow-TOPPED block whose body is wider than its top (a brewing stand's
 * base plate, a pane with one arm): the class model as a support, exactly as
 * code 0 — and never a thin obstacle a flight may pass beside (moveGen.ts
 * thinClear), which is what telling it apart from 0 is for.
 */
export const CARRY_WIDE = 7

export function carryCode (shapes: readonly number[][]): number {
  const cls = shapes.length === 0 ? 0 : topCatchClass(shapes)
  if (cls === 0) return 0
  const code = topCarryCode(shapes)
  // Every box of the block — not only the ones that make its top — has to
  // sit inside the footprint the code stands for, or it is CARRY_WIDE.
  let x0 = 1; let x1 = 0; let z0 = 1; let z1 = 0
  for (const b of shapes) {
    if (b[0] < x0) x0 = b[0]
    if (b[3] > x1) x1 = b[3]
    if (b[2] < z0) z0 = b[2]
    if (b[5] > z1) z1 = b[5]
  }
  const r = code === 0
    ? [0.5 - CATCH_HALF[cls], 0.5 + CATCH_HALF[cls], 0.5 - CATCH_HALF[cls], 0.5 + CATCH_HALF[cls]]
    : THIN_FOOTPRINT[code]
  const E = 1e-6
  return x0 >= r[0] - E && x1 <= r[1] + E && z0 >= r[2] - E && z1 <= r[3] + E ? code : CARRY_WIDE
}

/** Footprint [x0, x1, z0, z1] a carry code 1-6 stands for as an OBSTACLE (a line is a fence's post wide). */
export const THIN_FOOTPRINT: ReadonlyArray<readonly [number, number, number, number]> = [
  [0, 1, 0, 1],
  [0.8125, 1, 0, 1],
  [0, 0.1875, 0, 1],
  [0, 1, 0.8125, 1],
  [0, 1, 0, 0.1875],
  [0, 1, 0.375, 0.625],
  [0.375, 0.625, 0, 1]
]

function topCarryCode (shapes: readonly number[][]): number {
  let top = 0
  for (const s of shapes) if (s[4] > top) top = s[4]
  const lim = top - 0.6
  let x0 = 1; let x1 = 0; let z0 = 1; let z1 = 0
  for (const s of shapes) {
    if (s[4] <= lim) continue
    if (s[0] < x0) x0 = s[0]
    if (s[3] > x1) x1 = s[3]
    if (s[2] < z0) z0 = s[2]
    if (s[5] > z1) z1 = s[5]
  }
  // per side: 2 = the face reaches this edge, 1 = the face is entirely on
  // the other side of the centre, 0 = neither (the class model)
  const side = (near: number, far: boolean): number => far ? (near >= 0.999 ? 2 : near <= 0.5 ? 1 : 0) : (near <= 0.001 ? 2 : near >= 0.5 ? 1 : 0)
  const w = side(x0, false); const e = side(x1, true); const n = side(z0, false); const s = side(z1, true)
  if (w === 1 && e === 2 && n === 2 && s === 2) return CARRY_W_NONE
  if (e === 1 && w === 2 && n === 2 && s === 2) return CARRY_E_NONE
  if (n === 1 && s === 2 && w === 2 && e === 2) return CARRY_N_NONE
  if (s === 1 && n === 2 && w === 2 && e === 2) return CARRY_S_NONE
  if (w === 2 && e === 2 && n === 0 && s === 0) return CARRY_LINE_X
  if (n === 2 && s === 2 && w === 0 && e === 0) return CARRY_LINE_Z
  return 0
}

/**
 * The carry on one side of a support with carry code `code`, for a body
 * moving along signed axis step (dx, dz) — the credit the planner may take
 * toward that side: 2 full (a full block's), 1 none, 0 the class model.
 * `dx`/`dz` is the direction of the side asked about: (+1, 0) is the east
 * side, (0, −1) the north side.
 */
export function carrySide (code: number, dx: number, dz: number): number {
  if (code === 0 || code === CARRY_WIDE) return 0
  if (code === CARRY_LINE_X) return dx !== 0 ? 2 : 0
  if (code === CARRY_LINE_Z) return dz !== 0 ? 2 : 0
  // an edge panel: one side none, the other three full
  const none = code === CARRY_W_NONE ? (dx < 0) : code === CARRY_E_NONE ? (dx > 0) : code === CARRY_N_NONE ? (dz < 0) : (dz > 0)
  return none ? 1 : 2
}

/**
 * THIN footprint byte of a block (improvement, allowParkourExtended): where
 * its collision boxes sit in the cell, so a parkour flight can pass BESIDE a
 * pane post, a fence arm, an open trapdoor's panel or a door instead of
 * treating the whole cell as solid. Low nibble = the x interval's class, high
 * nibble = the z interval's; 0 = both FULL = an ordinary solid cell (and what
 * every full cube, slab, stair and empty block gets). Interval classes, in
 * sixteenths of a block — the smallest one that CONTAINS the boxes' extent:
 *   0 FULL [0,16]   1 LOW EDGE [0,3]   2 HIGH EDGE [13,16]
 *   3-6 CENTRED, half-width 1/2/3/4   7-10 LOW HALF [0, 8+hw]   11-14 HIGH HALF [8-hw, 16]
 * Self-describing (no table travels with the snapshot): thinInterval decodes.
 */
const THIN_INTERVALS: ReadonlyArray<readonly [number, number]> = [
  [0, 16], [0, 3], [13, 16],
  [7, 9], [6, 10], [5, 11], [4, 12],
  [0, 9], [0, 10], [0, 11], [0, 12],
  [7, 16], [6, 16], [5, 16], [4, 16]
]

function thinClass (lo: number, hi: number): number {
  // lo/hi in blocks; pick the narrowest listed interval that contains them
  const a = Math.round(lo * 16 * 1000) / 1000
  const b = Math.round(hi * 16 * 1000) / 1000
  let best = 0
  let bestW = 16
  for (let k = 1; k < THIN_INTERVALS.length; k++) {
    const [l, h] = THIN_INTERVALS[k]
    if (a >= l - 1e-6 && b <= h + 1e-6 && h - l < bestW) { best = k; bestW = h - l }
  }
  return best
}

/** The thin footprint byte of a block's collision boxes (0 = solid / nothing to pass beside). */
export function thinFootprint (shapes: readonly number[][]): number {
  if (shapes.length === 0) return 0
  let x0 = 1; let x1 = 0; let z0 = 1; let z1 = 0
  for (const b of shapes) {
    if (b[0] < x0) x0 = b[0]
    if (b[3] > x1) x1 = b[3]
    if (b[2] < z0) z0 = b[2]
    if (b[5] > z1) z1 = b[5]
  }
  return thinClass(x0, x1) | (thinClass(z0, z1) << 4)
}

/** [lo, hi] in blocks of an interval class (one nibble of a thin footprint byte). */
export function thinInterval (cls: number): readonly [number, number] {
  const iv = THIN_INTERVALS[cls] ?? THIN_INTERVALS[0]
  return [iv[0] / 16, iv[1] / 16]
}

/** Nominal top-face rect [x0, x1, z0, z1] per carry code (panels 3/16, lines 2/16). */
const CARRY_RECT: ReadonlyArray<readonly [number, number, number, number]> = [
  [0, 1, 0, 1],
  [0.8125, 1, 0, 1], // W none: the strip is against the east face
  [0, 0.1875, 0, 1],
  [0, 1, 0.8125, 1], // N none: against the south face
  [0, 1, 0, 0.1875],
  [0, 1, 0.4375, 0.5625],
  [0.4375, 0.5625, 0, 1],
  [0, 1, 0, 1] // CARRY_WIDE: the class model, ordinary ground
]
/** How far past a face's edge the hitbox centre is still carried, less a margin. */
const CARRY_REACH = 0.275

/**
 * Is a walk from a support with carry code `a` to one with code `b` at cell
 * offset (dx, dz) continuous ground — is there a body position both faces
 * carry? Two ledges round the OUTER corner of one wall meet at the corner;
 * the ledges of two walls across an inner corner do not, and the step
 * between them is a hop.
 */
export function carryTouch (a: number, b: number, dx: number, dz: number): boolean {
  const ra = CARRY_RECT[a]
  const rb = CARRY_RECT[b]
  const r = 2 * CARRY_REACH
  return ra[0] - r < rb[1] + dx && rb[0] + dx - r < ra[1] && ra[2] - r < rb[3] + dz && rb[2] + dz - r < ra[3]
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
