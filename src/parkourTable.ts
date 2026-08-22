// Generative extended-parkour table: every integer landing offset within the
// physics envelope (parkourEnvelope.ts), each with its swept-corridor cells
// computed from the 0.6-wide hitbox moving along the flight line — nothing
// hand-picked. Built once (lazy singleton) in JS ONLY: the wasm core receives
// this exact table with the solve parameters, so there is no second
// implementation of the geometry to keep bit-identical.
import {
  MAX_OFFSET_MAJOR, J_RUNNING, FLIGHT_STANDING, FLIGHT_RUNNING,
  FLIGHT_LOW_STANDING, FLIGHT_LOW_RUNNING, feetAt,
  flightNeeded, takeoffStand, takeoffRun, TAKEOFF_STAND, TAKEOFF_RUN
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
}

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
 * Clip the flight segment (0.5,0.5)→(a+0.5,b+0.5) against cell (i,j)'s rect
 * inflated by `m` (Minkowski sum with the hitbox = swept-area test; m = 0 =
 * center-line test). Returns the entry parameter t, or NaN when the segment
 * misses the open rect.
 */
function clipSegment (a: number, b: number, i: number, j: number, m: number): [number, number] | null {
  let t0 = 0
  let t1 = 1
  for (const [p, lo, hi] of [
    [a, i - m - 0.5, i + 1 + m - 0.5],
    [b, j - m - 0.5, j + 1 + m - 0.5]
  ]) {
    if (p === 0) {
      if (lo >= 0 || hi <= 0) return null
      continue
    }
    let e = lo / p
    let x = hi / p
    if (e > x) { const s = e; e = x; x = s }
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

function buildEntry (a: number, b: number): ParkourExtEntry {
  interface Cell { ax: number, az: number, line: boolean, t: number, tIn: number, tOut: number }
  const found: Cell[] = []
  for (let i = Math.min(0, a) - 1; i <= Math.max(0, a) + 1; i++) {
    for (let j = Math.min(0, b) - 1; j <= Math.max(0, b) + 1; j++) {
      if ((i === 0 && j === 0) || (i === a && j === b)) continue
      const swept = clipSegment(a, b, i, j, SWEPT_HALF)
      if (swept === null) continue
      const lineHit = clipSegment(a, b, i, j, 0)
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
  const dist = Math.hypot(a, b)
  // The jump fires from the takeoff point, not the cell center: standing
  // creeps sStand along the flight line, running delays the jump to sRun.
  // Corridor cells are parameterized center-to-center, so the curve lookup
  // shifts by the takeoff offset — cells behind it are crossed WALKING at
  // ground level (feetAt clamps to 0 there), which both restores the
  // near-takeoff poke veto and references apex heights from the true launch.
  const sStand = TAKEOFF_STAND * dist / Math.max(Math.abs(a), Math.abs(b))
  const sRun = TAKEOFF_RUN
  const cells: number[] = []
  const mfStand: number[] = []
  const mfRun: number[] = []
  const mfLowStand: number[] = []
  const mfLowRun: number[] = []
  for (const c of [...line, ...corners]) {
    cells.push(c.ax, c.az)
    // The flight arc rises then falls (unimodal), so the minimum feet height
    // over the cell's swept interval is at one of its endpoints.
    mfStand.push(Math.min(feetAt(FLIGHT_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_STANDING, c.tOut * dist - sStand)))
    mfRun.push(Math.min(feetAt(FLIGHT_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_RUNNING, c.tOut * dist - sRun)))
    mfLowStand.push(Math.min(feetAt(FLIGHT_LOW_STANDING, c.tIn * dist - sStand), feetAt(FLIGHT_LOW_STANDING, c.tOut * dist - sStand)))
    mfLowRun.push(Math.min(feetAt(FLIGHT_LOW_RUNNING, c.tIn * dist - sRun), feetAt(FLIGHT_LOW_RUNNING, c.tOut * dist - sRun)))
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
    fnStand: flightNeeded(a, b, tsx, tsz),
    fnRun: flightNeeded(a, b, trx, trz),
    runX: -cells[0],
    runZ: -cells[1],
    dist,
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
    cost: Math.max(dist + 0.5, octile(a, b))
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
 *   i32: nDiag, nCardX, nCardZ, cellsLen
 *   i32 ×7 per entry (diag, cardX, cardZ order): tx, tz, nLine, nCells,
 *        cellsOff, runX, runZ
 *   i32 ×cellsLen: cells
 *   f64 ×40: J_STANDING, J_RUNNING, J_LOW_STANDING, J_LOW_RUNNING (written
 *        by the caller — the envelope lives beside the table in the blob)
 *   f64 ×4 per entry: dist, cost, fnStand, fnRun
 *   f64 ×(cellsLen/2) ×4: mfStand, mfRun, mfLowStand, mfLowRun per cell,
 *        each block indexed by cellsOff + k
 */
export function serializeParkourTable (
  table: ParkourExtTable,
  standing: readonly number[],
  running: readonly number[],
  lowStanding: readonly number[],
  lowRunning: readonly number[]
): ArrayBuffer {
  const entries = [...table.diag, ...table.cardX, ...table.cardZ]
  const cellsLen = entries.reduce((n, e) => n + e.cells.length, 0)
  const intCount = 4 + entries.length * 7 + cellsLen
  const f64Off = Math.ceil((intCount * 4) / 8) * 8
  const buf = new ArrayBuffer(f64Off + 8 * (40 + entries.length * 4 + cellsLen * 2))
  const view = new DataView(buf)
  let o = 0
  const i32 = (v: number): void => { view.setInt32(o, v, true); o += 4 }
  i32(table.diag.length); i32(table.cardX.length); i32(table.cardZ.length); i32(cellsLen)
  let cellsOff = 0
  for (const e of entries) {
    i32(e.tx); i32(e.tz); i32(e.nLine); i32(e.cells.length / 2); i32(cellsOff); i32(e.runX); i32(e.runZ)
    cellsOff += e.cells.length / 2
  }
  for (const e of entries) for (const c of e.cells) i32(c)
  o = f64Off
  const f64 = (v: number): void => { view.setFloat64(o, v, true); o += 8 }
  for (const v of standing) f64(v)
  for (const v of running) f64(v)
  for (const v of lowStanding) f64(v)
  for (const v of lowRunning) f64(v)
  for (const e of entries) { f64(e.dist); f64(e.cost); f64(e.fnStand); f64(e.fnRun) }
  for (const e of entries) for (const v of e.mfStand) f64(v)
  for (const e of entries) for (const v of e.mfRun) f64(v)
  for (const e of entries) for (const v of e.mfLowStand) f64(v)
  for (const e of entries) for (const v of e.mfLowRun) f64(v)
  return buf
}
