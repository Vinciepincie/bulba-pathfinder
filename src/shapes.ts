// Port of mineflayer-pathfinder/lib/shapes.js (MIT), verbatim math — plus
// the top-catch classification shared by the LUT and the executor.
import { Vec3 } from 'vec3'

/** Centered landing half-extent per topCatchClass, in blocks: full/wide,
 * head/wall post, flower pot, fence post. */
export const CATCH_HALF: readonly number[] = [0.5, 0.25, 0.1875, 0.125]

/** Classify landing width from the union of faces within 0.6 blocks of the top.
 * Classes: 0 full/wide, 1 half-block, 2 flower pot, 3 fence post or narrower.
 * Union coverage keeps tiled stair boxes full-width; fence arms leave only the post centred. */
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

/** Test centred-square coverage by face union; rectangle edges partition it into uniform cells. */
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

/** Carry codes describe where a narrow support sits, in three LutSpecial bits.
 * 0 uses centred class credit; 1-4 have no W/E/N/S credit respectively; 5/6 are X/Z lines.
 * Panels carry fully toward their edge and along it, with no credit away; lines carry fully along them.
 * Other footprints keep conservative class credit. Full-width supports always use 0. */
export const CARRY_SHIFT = 5
export const CARRY_W_NONE = 1
export const CARRY_E_NONE = 2
export const CARRY_N_NONE = 3
export const CARRY_S_NONE = 4
export const CARRY_LINE_X = 5
export const CARRY_LINE_Z = 6
/** A narrow top with a wider body keeps class credit but cannot be passed as a thin obstacle. */
export const CARRY_WIDE = 7

export function carryCode (shapes: readonly number[][]): number {
  const cls = shapes.length === 0 ? 0 : topCatchClass(shapes)
  if (cls === 0) return 0
  const code = topCarryCode(shapes)
  // The entire obstacle must fit the footprint, including boxes below the support face.
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

/** Obstacle footprint [x0, x1, z0, z1] for carry codes 1-6; lines are fence-post wide. */
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
  // Per side: 2 reaches the edge, 1 lies across the centre, 0 keeps class credit.
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

/** Carry credit toward signed axis (dx, dz): 2 full, 1 none, 0 class-based. */
export function carrySide (code: number, dx: number, dz: number): number {
  if (code === 0 || code === CARRY_WIDE) return 0
  if (code === CARRY_LINE_X) return dx !== 0 ? 2 : 0
  if (code === CARRY_LINE_Z) return dz !== 0 ? 2 : 0
  const none = code === CARRY_W_NONE ? (dx < 0) : code === CARRY_E_NONE ? (dx > 0) : code === CARRY_N_NONE ? (dz < 0) : (dz > 0)
  return none ? 1 : 2
}

/** Packed X/Z obstacle intervals for flights past partial blocks; 0 treats the cell as solid.
 * Low/high nibbles encode X/Z bounds in sixteenths using the narrowest containing interval.
 * Classes cover full, edge, centred and half-cell intervals; thinInterval decodes them. */
const THIN_INTERVALS: ReadonlyArray<readonly [number, number]> = [
  [0, 16], [0, 3], [13, 16],
  [7, 9], [6, 10], [5, 11], [4, 12],
  [0, 9], [0, 10], [0, 11], [0, 12],
  [7, 16], [6, 16], [5, 16], [4, 16]
]

function thinClass (lo: number, hi: number): number {
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

/** Whether supports at offset (dx, dz) share a body position, allowing continuous walking.
 * Ledges meet around an outer corner; ledges across an inner corner require a hop. */
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

/** Encode a full-footprint block raised k/16 above its cell floor as a lid byte.
 * High nibble 15 distinguishes lids from footprints; low nibble stores k.
 * Partial footprints retain their footprint byte and count as reaching the cell floor. */
export const LID_TAG = 0xF0
export function lidByte (shapes: readonly number[][]): number {
  if (shapes.length === 0 || thinFootprint(shapes) !== 0) return 0
  let bot = 1
  for (const s of shapes) if (s[1] < bot) bot = s[1]
  const k = Math.floor(bot * 16 + 1e-9)
  return k >= 1 ? LID_TAG | Math.min(k, 15) : 0
}
/** True for a lid byte (not a footprint): thin consumers treat it as 0. */
export function isLidByte (b: number): boolean {
  return (b & 0xF0) === 0xF0
}
