// Generative extended-parkour table: every integer landing offset within the
// physics envelope (parkourEnvelope.ts), each with its swept-corridor cells
// computed from the 0.6-wide hitbox moving along the flight line — nothing
// hand-picked. Built once (lazy singleton) in JS ONLY: the wasm core receives
// this exact table with the solve parameters, so there is no second
// implementation of the geometry to keep bit-identical.
import {
  MAX_OFFSET_MAJOR, J_RUNNING, FLIGHT_STANDING, FLIGHT_RUNNING,
  FLIGHT_LOW_STANDING, FLIGHT_LOW_RUNNING, feetAt,
  flightNeeded, takeoffStand, takeoffRun, TAKEOFF_STAND, TAKEOFF_RUN, LAND_HALF, LIP_STRIDE
} from './parkourEnvelope.js'

export interface ParkourExtEntry {
  /** Landing offset, in multiples of the direction signs (sx, sz). */
  tx: number
  tz: number
  /** cells[0..2*nLine): flight-line cells — walkable floor there voids the jump. */
  nLine: number
  /** (ax, az) pairs the swept hitbox crosses: line cells first (flight
   * order), then corner-clipped cells; start and landing excluded. */
  cells: number[]
  /** Per cell (same order as cells): minimum feet height (blocks above the
   * takeoff walk level) while the hitbox overlaps that cell, from the flight
   * curve — standing and running takeoff classes. Bounds both the poke-in
   * height and how deep the corridor must be clear over that cell. */
  mfStand: number[]
  mfRun: number[]
  /** Same, on the head-hitter (2-high, bonked) flight curves. */
  mfLowStand: number[]
  mfLowRun: number[]
  /** Same for a LIP take-off (allowParkourMomentum): the lowest feet over
   * the cell for any take-off in [LAND_HALF, LAND_HALF + LIP_STRIDE] along
   * the line — a later take-off is lower on the rising arc, an earlier one
   * on the falling arc, so both ends of the window bound it. */
  mfLip: number[]
  mfLowLip: number[]
  /** Flight needed from the standing corner-creep takeoff / the running
   * delayed-jump takeoff, per-axis credited (parkourEnvelope.flightNeeded).
   * Compared against J_STANDING / J_RUNNING (or the J_LOW rows when the
   * corridor has a lid) per landing bucket. */
  fnStand: number
  fnRun: number
  /** Run-up cell: the backward continuation of the flight line. */
  runX: number
  runZ: number
  /** Center-to-center distance (cost basis). */
  dist: number
  /** Move cost: dist + 0.5 (≥ the octile heuristic; above walk cost). */
  cost: number
  /**
   * Lateral offsets (blocks, table space, signed along the perpendicular
   * (-b, a)/dist) of the TAKE-OFF point and the LANDING point from their
   * cell centres; both 0 = the centre line. Equal = a parallel (shifted)
   * line, different = an aimed one. See AIM_VARIANTS.
   */
  pOff: number
  qOff: number
  /** A NEO's waypoint: wOff beside the centre line at fraction wFrac of it; 0 = a straight line. */
  wOff: number
  wFrac: number
  /** How much longer than the centre line the aimed line is (added to the flight needed). */
  extra: number
  /** 1-based index into AIM_VARIANTS; 0 on a centred entry. Travels to the executor with the move. */
  aimIndex: number
  /**
   * The flown path in table space: take-off point, optional neo waypoint,
   * landing point, as [x, z, x, z, (x, z)] with cell (i,j) spanning
   * [i,i+1]x[j,j+1]. The planner tests it against the THIN footprint of a
   * corridor cell that is not passable (moveGen.ts thinClear): a pane post,
   * a fence arm or an open trapdoor's panel vetoes the line only where the
   * hitbox actually touches it.
   */
  pts: number[]
  /**
   * Per cell: which aimed variants of the entry's family do not sweep that
   * cell at all (bit v = AIM_VARIANTS[v]). On the CENTRED entry a blocked
   * cell is worth exactly those variants and no others: a pillar in the line
   * selects the lines that go round it, a wall across the flight selects
   * none. On a variant it is the SIBLINGS a solid block in that cell does not
   * refuse as well: the planner drops the rest of the family unseen.
   */
  avoid: number[]
  /**
   * Per cell of a VARIANT: the siblings whose flight LINE misses that cell
   * (bit v = AIM_VARIANTS[v]). Walkable floor under a line cell makes the
   * jump a walk — for every line over it alike — so the planner drops those
   * siblings with this one. Zeros on a centred entry (its refusal offers no
   * variant: the jump is a walk).
   */
  lineAvoid: number[]
  /** The aimed lines of a centred entry, in the order they are tried; empty
   * on a variant. The planner tries them only where the centred line is
   * refused by a corridor pass. */
  variants: ParkourExtEntry[]
}

/**
 * AIMED flight lines: the same jump flown from a take-off point and to a
 * landing point that sit beside their cell centres, which is how a player
 * clears what the centre line clips. [pOff, qOff], tried in this order:
 *  - parallel lines 0.2 (the body flush with its own cell at both ends) and
 *    0.35 to either side: a pillar at the corner of the line (the arena's
 *    Lateral Leaps);
 *  - a take-off from one side of the block, a landing on the far EDGE of the
 *    landing block (0.65: the body overhangs it by 0.15), and their
 *    combinations: the neo, past a pillar that stands in the line and in
 *    behind it (mcc-8-1), and the jump taken from the edge of the block.
 * Full-block ends only (moveGen.ts): a post or a panel has no room beside
 * its centre.
 */
export const AIM_VARIANTS: ReadonlyArray<readonly [number, number, number, number]> = [
  [0.2, 0.2, 0, 0], [-0.2, -0.2, 0, 0], [0.35, 0.35, 0, 0], [-0.35, -0.35, 0, 0],
  [0.35, 0, 0, 0], [-0.35, 0, 0, 0], [0, 0.65, 0, 0], [0, -0.65, 0, 0],
  [0.35, 0.65, 0, 0], [-0.35, -0.65, 0, 0], [-0.35, 0.65, 0, 0], [0.35, -0.65, 0, 0],
  [0.65, 0, 0, 0], [-0.65, 0, 0, 0], [0.65, 0.65, 0, 0], [-0.65, -0.65, 0, 0],
  // The NEO: [pOff, qOff, wOff, f] — out beside a pillar that stands IN the
  // line and back in behind it, steered at a waypoint wOff beside the line
  // at fraction f of it (0.95 puts the whole hitbox past a full cube).
  [0.65, 0.65, 0.95, 0.5], [-0.65, -0.65, -0.95, 0.5],
  [0.65, 0.65, 0.95, 0.35], [-0.65, -0.65, -0.95, 0.35],
  [0.65, 0.65, 0.95, 0.65], [-0.65, -0.65, -0.95, 0.65],
  [0.35, 0.65, 0.9, 0.5], [-0.35, -0.65, -0.9, 0.5]
]
/**
 * The aimed lines whose TAKE-OFF is centred (pOff 0) — the only ones a narrow
 * support can start, since the rest stand the body beside the cell's centre
 * where a post, a pane or a panel has nothing to stand on.
 */
export const AIM_P0_MASK = AIM_VARIANTS.reduce((m, v, i) => v[0] === 0 ? m | (1 << i) : m, 0)
/** The parallel offsets, by name (tests, docs). */
export const LATERAL_SHIFT = 0.35
export const LATERAL_SHIFT_SMALL = 0.2
/** An aimed jump costs more, by how far the body lines up off-centre; a non-parallel line a little more again. */
function aimCost (p: number, q: number, w: number): number {
  if (w !== 0) return 3 // a neo: lined up on the block's edge and steered through a waypoint
  const m = Math.max(Math.abs(p), Math.abs(q))
  return (m <= 0.2 ? 0.5 : m <= 0.35 ? 1 : 1.5) + (p === q ? 0 : 0.5)
}
/**
 * An aimed line is swept with a TEN-centimetre corner-nick allowance: less
 * than the centre line's (it is only offered where that one hits something,
 * so it has to clear most of what it was offered for), and not none — the
 * neo tucks in behind its pillar by sliding the last few centimetres along
 * the pillar's face, which is a corner clipped by 0.06-0.1 on the MINOR
 * axis (mcc-8-1). The executor's rollout flies the real collision before
 * it commits.
 */
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
/**
 * Corner-nick tolerance. Vanilla resolves collisions PER AXIS: a box that
 * clips the corner of a block by a few centimetres loses that much travel on
 * one axis and slides on — it does not stop the jump (and a player aims a
 * hair off-line anyway). Clipping the cell rect inflated by
 * HITBOX_HALF − this is exactly "the box penetrates the cell by ≥ this on
 * BOTH axes", so shallow corner nicks stop vetoing while any real cut
 * through a block still does. The executor's per-tick physics rollout is the
 * backstop: it simulates the actual graze before committing the jump.
 */
const CORNER_NICK = 0.15
const SWEPT_HALF = HITBOX_HALF - CORNER_NICK
/** Grazing contact (segment touching a cell edge exactly) does not count. */
const GRAZE_EPS = 1e-9

/**
 * Clip the flight segment P -> P+D against the rect [x0,x1] x [z0,z1] inflated
 * by `m` (Minkowski sum with the hitbox = swept-area test; m = 0 = centre-line
 * test). Table space: cell (i,j) spans [i,i+1] x [j,j+1]. Returns the
 * [entry, exit] parameters, or null when the segment misses the open rect.
 */
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

/** The solver's own XZ heuristic (goals.ts distanceXZ) — the floor a move
 *  cost may never go under, or A* stops being admissible. */
function octile (a: number, b: number): number {
  const dx = Math.abs(a)
  const dz = Math.abs(b)
  return Math.abs(dx - dz) + Math.SQRT2 * Math.min(dx, dz)
}

function buildEntry (a: number, b: number, pOff = 0, qOff = 0, aimIndex = 0, wOff = 0, wFrac = 0): ParkourExtEntry {
  interface Cell { ax: number, az: number, line: boolean, t: number, tIn: number, tOut: number }
  const found: Cell[] = []
  const centreDist = Math.hypot(a, b)
  // The flown path: take-off point P, optional waypoint W, landing point Q —
  // the cell centres (and the point wFrac along the centre line) moved pOff /
  // wOff / qOff along the perpendicular (-b, a)/dist. The sweep is the
  // corner-nick model: the centre line's own, or the aimed lines' tighter one.
  const at = (f: number, off: number): [number, number] => [0.5 + a * f - off * b / centreDist, 0.5 + b * f + off * a / centreDist]
  const pts: Array<[number, number]> = wOff !== 0 ? [at(0, pOff), at(wFrac, wOff), at(1, qOff)] : [at(0, pOff), at(1, qOff)]
  const segLen: number[] = []
  for (let k = 0; k + 1 < pts.length; k++) segLen.push(Math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]))
  const dist = segLen.reduce((n, v) => n + v, 0)
  const centred = pOff === 0 && qOff === 0 && wOff === 0
  const sweptHalf = centred ? SWEPT_HALF : SHIFT_SWEPT_HALF
  /** Swept interval of a rect inflated by m over the whole path, as fractions of its length; null = missed. */
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
  // The jump fires from the takeoff point, not the cell center: standing
  // creeps sStand along the flight line, running delays the jump to sRun.
  // Corridor cells are parameterized center-to-center, so the curve lookup
  // shifts by the takeoff offset — cells behind it are crossed WALKING at
  // ground level (feetAt clamps to 0 there), which both restores the
  // near-takeoff poke veto and references apex heights from the true launch.
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
    // The flight arc rises then falls (unimodal), so the minimum feet height
    // over the cell's swept interval is at one of its endpoints.
    mfStand.push(Math.min(feetAt(FLIGHT_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_STANDING, c.tOut * dist - sStand)))
    mfRun.push(Math.min(feetAt(FLIGHT_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_RUNNING, c.tOut * dist - sRun)))
    mfLowStand.push(Math.min(feetAt(FLIGHT_LOW_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_LOW_STANDING, c.tOut * dist - sStand)))
    mfLowRun.push(Math.min(feetAt(FLIGHT_LOW_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_LOW_RUNNING, c.tOut * dist - sRun)))
    // Over the take-off WINDOW x the cell's sweep the minimum is still at an
    // end: the latest take-off at the cell's entry, the earliest at its exit.
    // Floored by the running curve too, so the lip corridor is never more
    // permissive than today's on the falling arc (a narrow support's lip is
    // earlier than a full block's, which is lower there) — only honest about
    // the later, lower rising arc.
    mfLip.push(Math.min(mfRun[mfRun.length - 1], feetAt(FLIGHT_RUNNING, c.tIn * dist - sLipFar), feetAt(FLIGHT_RUNNING, c.tOut * dist - sLipNear)))
    mfLowLip.push(Math.min(mfLowRun[mfLowRun.length - 1], feetAt(FLIGHT_LOW_RUNNING, c.tIn * dist - sLipFar), feetAt(FLIGHT_LOW_RUNNING, c.tOut * dist - sLipNear)))
  }
  const variants = centred ? AIM_VARIANTS.map(([vp, vq, vw, vf], k) => buildEntry(a, b, vp, vq, k + 1, vw, vf)) : []
  /** Bit v set where variant v does NOT have cell (ax, az) among its first `upTo(v)` cells. */
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
  // First cell the centre line enters (its own cells[0]), for the variants' run-up cell.
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
    // The run-up cell is the backward continuation of the CENTRE line for
    // every variant (an aimed line's own first cell can be a lateral one).
    runX: centred ? -cells[0] : -centreFirst[0],
    runZ: centred ? -cells[1] : -centreFirst[1],
    dist: centreDist,
    // `dist + 0.5`: euclidean plus enough pad to clear the worst
    // octile-minus-euclidean deficit, so every entry stays at or above the
    // heuristic. Pricing a jump at PAR instead (cost = octile, no pad) was
    // tried and measured worse: A* then bought jump-heavy lines that were
    // cheaper by the model but longer on the ground, and the arena's simple3
    // went 9.51s -> 9.74s. The pad is doing real tie-breaking work — it keeps
    // a jump from displacing a walk of the same displacement for free.
    //
    // The pad is not quite enough at the enumeration limit, though: the
    // deficit peaks at ~0.082 x major, so (2,6), (3,6), (6,2) and (6,3) come
    // out BELOW the octile heuristic by up to 0.035 and make it inadmissible.
    // Floor them at the heuristic rather than lowering MAX_OFFSET_MAJOR,
    // which would delete real reach (and the entries envelope.test.ts pins).
    //
    // MEASURED AND REJECTED (2026-09-18): flooring every jump at the ~11-tick
    // arc (3.1 walking blocks, JUMP_MIN_COST) on the theory that a short hop
    // is slower than walking round it. The route book said the opposite —
    // basic1 +2.49 s, basic3 +3.68 s, tunnel1 +1.47 s, because the bot then
    // walked 5-8 blocks round gaps it used to cross in stride. A hop is only
    // 11 dead ticks when the body has to STOP and line up for it; a sprint-hop
    // never stops, and covers more ground per tick than sprinting does. Any
    // future attempt has to price the LINE-UP, not the jump.
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
    variants
  }
}

let cached: ParkourExtTable | null = null

export function getParkourExtTable (): ParkourExtTable {
  if (cached !== null) return cached
  // Feasibility cap: reachable at the deepest running bucket at all.
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

/**
 * Serialize for the wasm core (little-endian, matches lib.rs parse_ext):
 *   i32: nDiag, nCardX, nCardZ, cellsLen, nVar (shifted variants per entry)
 *   i32 ×8 per entry (diag, cardX, cardZ order, then every entry's variants
 *        in the same order, nVar each): tx, tz, nLine, nCells, cellsOff,
 *        runX, runZ, nSeg (path segments: 1, or 2 for a neo)
 *   i32 ×cellsLen: cells
 *   i32 ×(cellsLen/2): avoid masks, one per cell (ParkourExtEntry.avoid)
 *   i32 ×(cellsLen/2): line-avoid masks, one per cell (ParkourExtEntry.lineAvoid)
 *   f64 ×150: J_RUN (7 run rows × 10 buckets), then J_LOW_RUN, then J_CHAIN
 *        (10) — the envelope lives beside the table in the blob, so the core
 *        never carries a pasted copy that can drift from the pinned rows
 *   f64 ×13 per entry: dist, cost, fnStand, fnRun, pOff, qOff, extra, then the
 *        path's three points x,z (the last repeated for a straight line)
 *   f64 ×(cellsLen/2) ×6: mfStand, mfRun, mfLowStand, mfLowRun, mfLip,
 *        mfLowLip per cell, each block indexed by cellsOff + k
 */
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
  const buf = new ArrayBuffer(f64Off + 8 * (150 + entries.length * 13 + cellsLen * 3))
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
  return buf
}
