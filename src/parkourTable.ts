// Generate swept corridors once in JavaScript and pass the same table to the wasm core,
// so both solvers use identical flight geometry.
import {
  MAX_OFFSET_MAJOR, J_RUNNING, FLIGHT_STANDING, FLIGHT_RUNNING,
  FLIGHT_LOW_STANDING, FLIGHT_LOW_RUNNING, feetAt,
  flightNeeded, takeoffStand, takeoffRun, TAKEOFF_STAND, TAKEOFF_RUN, LAND_HALF, LIP_STRIDE, J_LID8_RUN, J_LID13_RUN, J_LID3_RUN, FLIGHT_ROW } from './parkourEnvelope.js'

export interface ParkourExtEntry {
  /** Landing offset, in multiples of the direction signs (sx, sz). */
  tx: number
  tz: number
  /** First nLine cell pairs lie on the flight line; walkable floor there makes the jump unnecessary. */
  nLine: number
  /** Swept (ax, az) cell pairs: flight-line cells in order, then clipped corners.
   * Takeoff and landing cells are excluded. */
  cells: number[]
  /** Minimum feet height above takeoff level per swept cell, for standing and running starts.
   * Bounds obstacle height and required clearance depth while the body overlaps the cell. */
  mfStand: number[]
  mfRun: number[]
  /** Minimum feet heights for ceiling-clipped flight under two-block headroom. */
  mfLowStand: number[]
  mfLowRun: number[]
  /** Minimum feet height over each cell for takeoffs in [LAND_HALF, LAND_HALF + LIP_STRIDE].
   * The latest takeoff bounds the rising arc; the earliest bounds the falling arc. */
  mfLip: number[]
  mfLowLip: number[]
  /** Required flight from standing corner-creep and running delayed takeoffs, with per-axis
   * support credit; compared with the corresponding reach row for the landing height. */
  fnStand: number
  fnRun: number
  /** Run-up cell: the backward continuation of the flight line. */
  runX: number
  runZ: number
  /** Center-to-center distance (cost basis). */
  dist: number
  /** Base cost max(dist + 0.5, octile distance), plus an aimed-flight premium. */
  cost: number
  /** Lateral takeoff and landing offsets in blocks along the perpendicular (-tz, tx)/dist.
   * Both zero gives the centre line; equal offsets give a parallel line. */
  pOff: number
  qOff: number
  /** Waypoint offset beside the centre line at fraction wFrac, for a pillar-dodging jump; wOff = 0 gives a straight line. */
  wOff: number
  wFrac: number
  /** How much longer than the centre line the aimed line is (added to the flight needed). */
  extra: number
  /** 1-based index into AIM_VARIANTS; 0 on a centred entry. Travels to the executor with the move. */
  aimIndex: number
  /** Flight path as [x, z] pairs: takeoff, optional waypoint, landing; cell (i,j) spans [i,i+1] x [j,j+1].
   * Collision checks use actual thin-block footprints so empty parts of a cell do not veto a jump. */
  pts: number[]
  /** Per-cell mask of variants whose swept hitboxes miss the cell; bit v refers to AIM_VARIANTS[v].
   * A blocked cell preserves only those variants, allowing the planner to prune the rest of the family. */
  avoid: number[]
  /** Per-variant cell mask of siblings whose flight lines miss the cell; bit v refers to AIM_VARIANTS[v].
   * Floor under the line makes a jump a walk for every line crossing it. Centred entries use zeros. */
  lineAvoid: number[]
  /** Aimed variants in trial order, empty on a variant; tried only when the centred
   * flight is blocked by corridor geometry. */
  variants: ParkourExtEntry[]
  /** Head-sweep (ax, az) cell pairs: takeoff first, then path order, landing last.
   * Vertical collision resolves first, so ceiling checks use HEAD_HALF without corner-nick tolerance. */
  headCells: number[]
  /** Distances along pts from takeoff where the head hitbox enters and leaves each head cell. */
  headIn: number[]
  headOut: number[]
}

/** Head-sweep half-width: the 0.3-block hitbox radius minus 0.01 blocks of grace. */
const HEAD_HALF = 0.29
/** Maximum feet height over distances [a, b] from the jump point.
 * Vertical motion resolves before horizontal motion, so interval [d_(i-1), d_i] uses height y_i.
 * Before takeoff (a < 0), feet are at ground level. */
export function maxFeetStep (curve: readonly number[], a: number, b: number): number {
  let m = a < 0 ? 0 : -Infinity
  let prev = 0
  for (let i = 0; i < curve.length; i += 2) {
    const d = curve[i]
    if (d >= a && prev <= b && curve[i + 1] > m) m = curve[i + 1]
    prev = d
    if (prev > b) break
  }
  if (m === -Infinity) m = curve[curve.length - 1]
  return m
}

/** Flight variants [takeoff offset, landing offset, waypoint offset, waypoint fraction],
 * tried in order to clear obstacles clipped by the centre line. */
export const AIM_VARIANTS: ReadonlyArray<readonly [number, number, number, number]> = [
  [0.2, 0.2, 0, 0], [-0.2, -0.2, 0, 0], [0.35, 0.35, 0, 0], [-0.35, -0.35, 0, 0],
  [0.35, 0, 0, 0], [-0.35, 0, 0, 0], [0, 0.65, 0, 0], [0, -0.65, 0, 0],
  [0.35, 0.65, 0, 0], [-0.35, -0.65, 0, 0], [-0.35, 0.65, 0, 0], [0.35, -0.65, 0, 0],
  [0.65, 0, 0, 0], [-0.65, 0, 0, 0], [0.65, 0.65, 0, 0], [-0.65, -0.65, 0, 0],
  // A 0.95-block waypoint offset puts the whole hitbox beside a full cube,
  // allowing the flight to return behind the pillar.
  [0.65, 0.65, 0.95, 0.5], [-0.65, -0.65, -0.95, 0.5],
  [0.65, 0.65, 0.95, 0.35], [-0.65, -0.65, -0.95, 0.35],
  [0.65, 0.65, 0.95, 0.65], [-0.65, -0.65, -0.95, 0.65],
  [0.35, 0.65, 0.9, 0.5], [-0.35, -0.65, -0.9, 0.5]
]
/** Variants with centred takeoff (pOff = 0) are the only ones narrow supports can start;
 * other offsets place the body beside the available support. */
export const AIM_P0_MASK = AIM_VARIANTS.reduce((m, v, i) => v[0] === 0 ? m | (1 << i) : m, 0)
/** Parallel flight offsets in blocks. */
export const LATERAL_SHIFT = 0.35
export const LATERAL_SHIFT_SMALL = 0.2
/** Cost premium for lining up off-centre, with an extra penalty for non-parallel flight. */
function aimCost (p: number, q: number, w: number): number {
  if (w !== 0) return 3 // Edge alignment and waypoint steering add to the cost.
  const m = Math.max(Math.abs(p), Math.abs(q))
  return (m <= 0.2 ? 0.5 : m <= 0.35 ? 1 : 1.5) + (p === q ? 0 : 0.5)
}
/** Aimed flights allow 0.1-block corner nicks so the body can slide behind a pillar.
 * The executor checks the actual collision in a physics rollout before takeoff. */
const SHIFT_SWEPT_HALF = 0.2
export interface ParkourExtTable {
  /** Offsets with both components ≥ 1, applied per diagonal quadrant. */
  diag: ParkourExtEntry[]
  /** Pure-x offsets (tz = 0), applied per x cardinal direction. */
  cardX: ParkourExtEntry[]
  /** Pure-z offsets (tx = 0), applied per z cardinal direction. */
  cardZ: ParkourExtEntry[]
}

const HITBOX_HALF = 0.3
/** Vanilla resolves collisions per axis, so shallow corner clips can slide past a block.
 * Shrink the sweep by this tolerance; the executor checks the actual graze before takeoff. */
const CORNER_NICK = 0.15
const SWEPT_HALF = HITBOX_HALF - CORNER_NICK
/** Grazing contact (segment touching a cell edge exactly) does not count. */
const GRAZE_EPS = 1e-9

/** Clip P -> P+D against an open rectangle inflated by m; m = 0 tests the centre line.
 * Returns entry and exit parameters, or null for a miss or grazing contact. */
function clipRect (px: number, pz: number, dx: number, dz: number, x0: number, x1: number, z0: number, z1: number, m: number): [number, number] | null {
  let t0 = 0
  let t1 = 1
  for (const [o, d, lo, hi] of [
    [px, dx, x0 - m, x1 + m],
    [pz, dz, z0 - m, z1 + m]
  ]) {
    if (d === 0) {
      if (o <= lo || o >= hi) return null
      continue
    }
    let e = (lo - o) / d
    let x = (hi - o) / d
    if (e > x) { const sw = e; e = x; x = sw }
    if (e > t0) t0 = e
    if (x < t1) t1 = x
  }
  return t1 - t0 > GRAZE_EPS ? [t0, t1] : null
}

/** Octile XZ heuristic; move costs must meet this lower bound to preserve admissibility. */
function octile (a: number, b: number): number {
  const dx = Math.abs(a)
  const dz = Math.abs(b)
  return Math.abs(dx - dz) + Math.SQRT2 * Math.min(dx, dz)
}

function buildEntry (a: number, b: number, pOff = 0, qOff = 0, aimIndex = 0, wOff = 0, wFrac = 0): ParkourExtEntry {
  interface Cell { ax: number, az: number, line: boolean, t: number, tIn: number, tOut: number }
  const found: Cell[] = []
  const centreDist = Math.hypot(a, b)
  // Lateral offsets use the perpendicular (-b, a)/dist in cell coordinates.
  const at = (f: number, off: number): [number, number] => [0.5 + a * f - off * b / centreDist, 0.5 + b * f + off * a / centreDist]
  const pts: Array<[number, number]> = wOff !== 0 ? [at(0, pOff), at(wFrac, wOff), at(1, qOff)] : [at(0, pOff), at(1, qOff)]
  const segLen: number[] = []
  for (let k = 0; k + 1 < pts.length; k++) segLen.push(Math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]))
  const dist = segLen.reduce((n, v) => n + v, 0)
  const centred = pOff === 0 && qOff === 0 && wOff === 0
  const sweptHalf = centred ? SWEPT_HALF : SHIFT_SWEPT_HALF
  /** Swept interval over the whole path as fractions of its length, or null for a miss. */
  const sweep = (x0: number, x1: number, z0: number, z1: number, m: number): [number, number] | null => {
    let lo = Infinity
    let hi = -Infinity
    let before = 0
    for (let k = 0; k < segLen.length; k++) {
      const hit = clipRect(pts[k][0], pts[k][1], pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1], x0, x1, z0, z1, m)
      if (hit !== null) {
        lo = Math.min(lo, (before + hit[0] * segLen[k]) / dist)
        hi = Math.max(hi, (before + hit[1] * segLen[k]) / dist)
      }
      before += segLen[k]
    }
    return hi >= lo ? [lo, hi] : null
  }
  const reach = wOff !== 0 ? 2 : 1
  for (let i = Math.min(0, a) - reach; i <= Math.max(0, a) + reach; i++) {
    for (let j = Math.min(0, b) - reach; j <= Math.max(0, b) + reach; j++) {
      if ((i === 0 && j === 0) || (i === a && j === b)) continue
      const swept = sweep(i, i + 1, j, j + 1, sweptHalf)
      if (swept === null) continue
      const lineHit = sweep(i, i + 1, j, j + 1, 0)
      found.push({
        ax: i,
        az: j,
        line: lineHit !== null,
        t: lineHit !== null ? lineHit[0] : swept[0],
        tIn: swept[0],
        tOut: swept[1]
      })
    }
  }
  const line = found.filter(c => c.line).sort((p, q) => p.t - q.t)
  const corners = found.filter(c => !c.line).sort((p, q) => p.t - q.t)
  // Shift cell progress by the actual takeoff offset: cells behind it are crossed
  // at ground level, so they must still reject obstacles in the walking approach.
  const sStand = TAKEOFF_STAND * centreDist / Math.max(Math.abs(a), Math.abs(b))
  const sRun = TAKEOFF_RUN
  const cells: number[] = []
  const mfStand: number[] = []
  const mfRun: number[] = []
  const mfLowStand: number[] = []
  const mfLowRun: number[] = []
  const mfLip: number[] = []
  const mfLowLip: number[] = []
  const sLipNear = LAND_HALF
  const sLipFar = LAND_HALF + LIP_STRIDE
  for (const c of [...line, ...corners]) {
    cells.push(c.ax, c.az)
    // The arc rises then falls, so minimum feet height occurs at a swept interval's endpoint.
    mfStand.push(Math.min(feetAt(FLIGHT_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_STANDING, c.tOut * dist - sStand)))
    mfRun.push(Math.min(feetAt(FLIGHT_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_RUNNING, c.tOut * dist - sRun)))
    mfLowStand.push(Math.min(feetAt(FLIGHT_LOW_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_LOW_STANDING, c.tOut * dist - sStand)))
    mfLowRun.push(Math.min(feetAt(FLIGHT_LOW_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_LOW_RUNNING, c.tOut * dist - sRun)))
    // Bound the takeoff window at both ends and include the running curve:
    // a narrow support's earlier takeoff can be lower on the falling arc.
    mfLip.push(Math.min(mfRun[mfRun.length - 1], feetAt(FLIGHT_RUNNING, c.tIn * dist - sLipFar), feetAt(FLIGHT_RUNNING, c.tOut * dist - sLipNear)))
    mfLowLip.push(Math.min(mfLowRun[mfLowRun.length - 1], feetAt(FLIGHT_LOW_RUNNING, c.tIn * dist - sLipFar), feetAt(FLIGHT_LOW_RUNNING, c.tOut * dist - sLipNear)))
  }
  const headCells: number[] = []
  const headIn: number[] = []
  const headOut: number[] = []
  {
    const hc: Array<[number, number, number, number]> = []
    for (let i = Math.min(0, a) - reach; i <= Math.max(0, a) + reach; i++) {
      for (let j = Math.min(0, b) - reach; j <= Math.max(0, b) + reach; j++) {
        const hit = sweep(i, i + 1, j, j + 1, HEAD_HALF)
        if (hit === null) continue
        hc.push([i, j, hit[0] * dist, hit[1] * dist])
      }
    }
    const rank = (c: [number, number, number, number]): number => c[0] === 0 && c[1] === 0 ? 0 : c[0] === a && c[1] === b ? 2 : 1
    hc.sort((p, q) => rank(p) - rank(q) || p[2] - q[2])
    for (const [i, j, sIn, sOut] of hc) {
      headCells.push(i, j)
      headIn.push(sIn)
      headOut.push(sOut)
    }
  }
  const variants = centred ? AIM_VARIANTS.map(([vp, vq, vw, vf], k) => buildEntry(a, b, vp, vq, k + 1, vw, vf)) : []
  /** Bit v marks a variant that misses cell (ax, az) among its first upTo(v) cells. */
  const avoiders = (ax: number, az: number, upTo: (v: ParkourExtEntry) => number): number => {
    let mask = 0
    variants.forEach((v, vi) => {
      let has = false
      const n = upTo(v)
      for (let c = 0; c < n; c++) {
        if (v.cells[c * 2] === ax && v.cells[c * 2 + 1] === az) { has = true; break }
      }
      if (!has) mask |= 1 << vi
    })
    return mask
  }
  const sweptCells = (v: ParkourExtEntry): number => v.cells.length / 2
  const lineCells = (v: ParkourExtEntry): number => v.nLine
  const avoid: number[] = []
  for (let k = 0; k * 2 < cells.length; k++) avoid.push(avoiders(cells[k * 2], cells[k * 2 + 1], sweptCells))
  for (const v of variants) {
    v.avoid.length = 0
    v.lineAvoid.length = 0
    for (let k = 0; k * 2 < v.cells.length; k++) {
      v.avoid.push(avoiders(v.cells[k * 2], v.cells[k * 2 + 1], sweptCells))
      v.lineAvoid.push(avoiders(v.cells[k * 2], v.cells[k * 2 + 1], lineCells))
    }
  }
  let centreFirst: [number, number] = [0, 0]
  if (!centred) {
    let bestT = Infinity
    for (let i = Math.min(0, a) - 1; i <= Math.max(0, a) + 1; i++) {
      for (let j = Math.min(0, b) - 1; j <= Math.max(0, b) + 1; j++) {
        if ((i === 0 && j === 0) || (i === a && j === b)) continue
        const hit = clipRect(0.5, 0.5, a, b, i, i + 1, j, j + 1, 0)
        if (hit !== null && hit[0] < bestT) { bestT = hit[0]; centreFirst = [i, j] }
      }
    }
  }
  const [tsx, tsz] = takeoffStand(a, b)
  const [trx, trz] = takeoffRun(a, b)
  return {
    tx: a,
    tz: b,
    nLine: line.length,
    cells,
    mfStand,
    mfRun,
    mfLowStand,
    mfLowRun,
    mfLip,
    mfLowLip,
    fnStand: flightNeeded(a, b, tsx, tsz),
    fnRun: flightNeeded(a, b, trx, trz),
    // Run-up follows the centre line; an aimed flight's first swept cell can lie to the side.
    runX: centred ? -cells[0] : -centreFirst[0],
    runZ: centred ? -cells[1] : -centreFirst[1],
    dist: centreDist,
    // The 0.5 pad prevents jumps from replacing equivalent walks for free.
    // The octile floor preserves heuristic admissibility where the pad is insufficient.
    cost: Math.max(centreDist + 0.5, octile(a, b)) + (centred ? 0 : aimCost(pOff, qOff, wOff)),
    pOff,
    qOff,
    wOff,
    wFrac,
    extra: Math.max(0, dist - centreDist),
    aimIndex,
    pts: pts.flat(),
    avoid,
    lineAvoid: new Array<number>(cells.length / 2).fill(0),
    variants,
    headCells,
    headIn,
    headOut
  }
}

let cached: ParkourExtTable | null = null

export function getParkourExtTable (): ParkourExtTable {
  if (cached !== null) return cached
  // Offsets beyond even the deepest running reach cannot be feasible.
  const maxFlight = J_RUNNING[J_RUNNING.length - 1]
  const diag: ParkourExtEntry[] = []
  const cardX: ParkourExtEntry[] = []
  const cardZ: ParkourExtEntry[] = []
  for (let a = 2; a <= MAX_OFFSET_MAJOR; a++) {
    const e = buildEntry(a, 0)
    if (e.fnRun > maxFlight) continue
    cardX.push(e)
    cardZ.push(buildEntry(0, a))
  }
  for (let a = 1; a <= MAX_OFFSET_MAJOR; a++) {
    for (let b = 1; b <= MAX_OFFSET_MAJOR; b++) {
      if (Math.hypot(a, b) < 2) continue // adjacent hops are walk/jump moves
      const e = buildEntry(a, b)
      if (e.fnRun > maxFlight) continue
      diag.push(e)
    }
  }
  cached = { diag, cardX, cardZ }
  return cached
}

/** Serialize shared geometry and reach envelopes for the wasm core in little-endian order.
 * Integer metadata, cells and masks precede aligned f64 reach rows, entry geometry,
 * clearance bounds, ceiling rows, flight curves and head-cell spans; matches lib.rs parse_ext. */
export function serializeParkourTable (
  table: ParkourExtTable,
  run: ReadonlyArray<readonly number[]>,
  lowRun: ReadonlyArray<readonly number[]>,
  chain: readonly number[]
): ArrayBuffer {
  const main = [...table.diag, ...table.cardX, ...table.cardZ]
  const nVar = main[0].variants.length
  const entries = [...main, ...main.flatMap(e => e.variants)]
  const cellsLen = entries.reduce((n, e) => n + e.cells.length, 0)
  const intCount = 5 + entries.length * 8 + cellsLen + cellsLen
  const f64Off = Math.ceil((intCount * 4) / 8) * 8
  // The f64 tail includes ceiling rows, length-prefixed flight curves and head-cell spans.
  const flightLen = FLIGHT_ROW.reduce((n, c) => n + c.length, 0)
  const headLen = entries.reduce((n, e) => n + 1 + (e.headCells.length / 2) * 4, 0)
  const buf = new ArrayBuffer(f64Off + 8 * (150 + entries.length * 13 + cellsLen * 3 + 210 + FLIGHT_ROW.length + flightLen + headLen))
  const view = new DataView(buf)
  let o = 0
  const i32 = (v: number): void => { view.setInt32(o, v, true); o += 4 }
  i32(table.diag.length); i32(table.cardX.length); i32(table.cardZ.length); i32(cellsLen); i32(nVar)
  let cellsOff = 0
  for (const e of entries) {
    i32(e.tx); i32(e.tz); i32(e.nLine); i32(e.cells.length / 2); i32(cellsOff); i32(e.runX); i32(e.runZ); i32(e.pts.length / 2 - 1)
    cellsOff += e.cells.length / 2
  }
  for (const e of entries) for (const c of e.cells) i32(c)
  for (const e of entries) for (const m of e.avoid) i32(m)
  for (const e of entries) for (const m of e.lineAvoid) i32(m)
  o = f64Off
  const f64 = (v: number): void => { view.setFloat64(o, v, true); o += 8 }
  for (const row of run) for (const v of row) f64(v)
  for (const row of lowRun) for (const v of row) f64(v)
  for (const v of chain) f64(v)
  for (const e of entries) {
    f64(e.dist); f64(e.cost); f64(e.fnStand); f64(e.fnRun); f64(e.pOff); f64(e.qOff); f64(e.extra)
    const n = e.pts.length
    for (let k = 0; k < 6; k++) f64(e.pts[k < n ? k : n - 2 + (k & 1)])
  }
  for (const e of entries) for (const v of e.mfStand) f64(v)
  for (const e of entries) for (const v of e.mfRun) f64(v)
  for (const e of entries) for (const v of e.mfLowStand) f64(v)
  for (const e of entries) for (const v of e.mfLowRun) f64(v)
  for (const e of entries) for (const v of e.mfLip) f64(v)
  for (const e of entries) for (const v of e.mfLowLip) f64(v)
  for (const rows of [J_LID8_RUN, J_LID13_RUN, J_LID3_RUN]) for (const row of rows) for (const v of row) f64(v)
  for (const c of FLIGHT_ROW) f64(c.length)
  for (const c of FLIGHT_ROW) for (const v of c) f64(v)
  for (const e of entries) {
    f64(e.headCells.length / 2)
    for (let h = 0; h * 2 < e.headCells.length; h++) { f64(e.headCells[h * 2]); f64(e.headCells[h * 2 + 1]); f64(e.headIn[h]); f64(e.headOut[h]) }
  }
  return buf
}
