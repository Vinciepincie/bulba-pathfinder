// The fixed jump table cannot represent curved flights or every thin landing.
// Simulated programs must stay on the landing and tolerate nudged starts.

import { PlayerSim, newBody, copyBody, SIM, SIM_CLIMBABLE, SIM_SLIME, SIM_LIQUID, SIM_HAZARD, isVanillaJump } from './playerSim.js'
import { fallDamagePoints, FALL_VETO, SAFE_FALL } from './fallDamage.js'
import type { SimBody, SimControl, SimWorld } from './playerSim.js'

/** How one phase of a program is steered: at a point, re-aimed every tick, or along a fixed unit direction. */
export interface Heading {
  fixed: boolean
  x: number
  z: number
}

/** A tick-for-tick control program from rest at (sx, sy, sz), or from the exact live `start` state. */
export interface HopProgram {
  sx: number
  sy: number
  sz: number
  /** Exact initial body state for a live program; absent when starting from rest at the start point. */
  start?: SimBody
  /** Initial live yaw, used to reproduce bot.look's heading rounding. */
  yaw0?: number
  /** Supported retreat to make room at a landing flush against a wall; forward held for at most four ticks. */
  groundPrefix?: Array<{ yaw: number, sprint: boolean, sneak: boolean }>
  /** Per-tick [yaw (NaN: no look), key flags (F_FWD | F_SPRINT | F_JUMP)] pairs for a beam program; replaces phase controls. */
  seq?: number[]
  /** A touchdown short of the target starts the next leg instead of ending the flight. */
  legs?: boolean
  /** Heading on the ground, until the jump fires. */
  run: Heading
  /** Tick (from the start) the jump is pressed on; held until the body leaves the ground. */
  jumpAt: number
  /** Jump on the first grounded tick at or after `jumpAt` with no support under the box.
   * Y moves first, so this is the last tick a jump can leave the lip; an event avoids dependence on the starting offset. */
  jumpOnLeave?: boolean
  /** Heading for the first `turnAir` air ticks (or until a slime rebound). */
  air1: Heading
  /** Face `air1` from `jumpAt` so the sprint-jump boost follows the jump heading rather than the run. */
  kick?: boolean
  /** Precision program verified with LIVE_NUDGE and flown only from the exact line-up, never a walking line-up. */
  precise?: boolean
  turnAir: number
  /** Heading after the turn, to touchdown. */
  air2: Heading
  /** Headings after successive slime rebounds; `air2` applies after the last, or after the first when absent. */
  bounces?: Heading[]
  /** Sprint held throughout (a walking jump otherwise). */
  sprint: boolean
  /** Ticks from the start to touchdown (catch). */
  ticks: number
  /** Landing node (planner convention) and whether it is a climbable catch. */
  tx: number
  ty: number
  tz: number
  catchTarget: boolean
  family: string
  /** Take off from a ladder or vine: horizontal speed is clamped to 0.15 and holding jump climbs.
   * The program fires once the body leaves the climbable and becomes airborne. */
  climb?: boolean
  /** Sneak on slime touchdown to suppress its rebound. */
  slimeLand?: boolean
  /** Bounce from slime more than BOUNCE_DROP below take-off, checked by commitGate and re-verified live each tick. */
  deep?: boolean
  /** Hold jump on the grounded tick after a rebound; vanilla preserves the rebound and adds the sprint boost. */
  reboundJump?: boolean
  /** Abort targets inside a deep bounce's slime pool, at least a block and a half from its edge. */
  basin?: Array<[number, number, number]>
  /** Required resting place within the landing node; only its walk-connected region counts as a landing.
   * A pane rail and the floors on either side share a planner node but form separate regions. */
  land?: { x: number, y: number, z: number }
}

/** Whether to sneak on slime touchdown to suppress the rebound; longer sneaking costs 70% of air control.
 * Uses the tick's initial state because Y moves before horizontal motion. */
export function sneakLanding (p: HopProgram, onGround: boolean, x: number, y: number, z: number, vy: number, hw: number): boolean {
  if (p.slimeLand !== true || onGround || vy >= 0 || y < p.ty - 1e-9 || y + vy > p.ty + 1e-9) return false
  return x + hw > p.tx && x - hw < p.tx + 1 && z + hw > p.tz && z - hw < p.tz + 1
}

export interface HopEdge {
  fromX: number
  fromY: number
  fromZ: number
  program: HopProgram
}

interface Stand { x: number, y: number, z: number, catch: boolean, under?: boolean }

/** Walk-connected stands on a node. `under` marks rests beside a thin block in the cell below, outside the planner's footing. */
export interface Region { stands: Stand[], y: number, under: boolean }

/** Start-position nudge every accepted program must survive, per axis (blocks). */
export const ROBUST_NUDGE = 0.03
/** Live start-position tolerance in blocks; exact-state replay only needs to cover model mismatch. */
export const LIVE_NUDGE = 0.01
const LIVE_NUDGE_SAFE = process.env.PF_LIVE_NUDGE_SAFE !== '0'
// Enables supported retreats before live jumps from flush landings.
export const LIVE_PREFIX = process.env.PF_LIVE_PREFIX !== '0'
export const LIVE_PREFIX_TICKS = 80_000
const PREFIX_EXPIRED = Symbol('prefix budget')
/** Stricter start-position tolerance for goal-side bridges, which can cross voids. */
const SIDE_NUDGE = 0.06
/** Letting go after touchdown: ticks the body must stay on the landing node. */
export const STAY_TICKS = 8
/** Enables settling region landings to their required level before counting arrival ticks. */
export const REGION_SETTLE = process.env.PF_REGION_SETTLE !== '0'
/** Allows neighbouring collision boxes to support a landing over its own column. */
const NEIGHBOUR_CARRY = process.env.PF_NEIGHBOUR_CARRY !== '0'
/** Planned climbable take-off heights above the cell floor. */
const HANG_HEIGHTS = [0.1, 0.6]

/** LutFlags bits required by the planner's node convention. */
const F_SAFE = 1
const F_PHYSICAL = 2
const F_CLIMBABLE = 8

/** The unit direction a heading steers along from (bx, bz); null when a point heading is reached. */
export function headingDir (h: Heading, bx: number, bz: number): [number, number] | null {
  if (h.fixed) return [h.x, h.z]
  const dx = h.x - bx
  const dz = h.z - bz
  const d = Math.sqrt(dx * dx + dz * dz)
  return d > 1e-9 ? [dx / d, dz / d] : null
}

/** The heading for the program's current phase counters. */
export function programHeading (p: HopProgram, fired: boolean, air: number, rebounds: number, t: number): Heading {
  // A kick's sprint boost must follow the jump heading.
  if (!fired) return p.kick === true && t >= p.jumpAt ? p.air1 : p.run
  if (rebounds === 0) return air < p.turnAir ? p.air1 : p.air2
  return p.bounces !== undefined && rebounds <= p.bounces.length ? p.bounces[rebounds - 1] : p.air2
}

const point = (x: number, z: number): Heading => ({ fixed: false, x, z })
const dir = (x: number, z: number): Heading => ({ fixed: true, x, z })

/** Programs sharing a run heading branch on jump tick, turn tick, then landing heading.
 * Incrementally traversed tick lists must be ascending. */
interface ProgramClass {
  run: Heading
  /** A kick class: the first air heading, pressed from the jump tick on (HopProgram.kick). */
  kick?: Heading
  sprint: boolean
  jumpAts: readonly number[]
  turnAirs: readonly number[]
  air2s: readonly Heading[]
  /** Also branch on the headings aimed at the landing (aimHeadings). */
  aim: boolean
  /** A bounce class whose landing no single rebound reaches: branch on to further slimes at the rebound (chainOn). */
  chain?: boolean
  /** Jump on the latched tick (HopProgram.jumpOnLeave). */
  leave?: boolean
  /** A deep bounce class (HopProgram.deep). */
  deep?: boolean
}

type Family = 'straight' | 'run-turn' | 'air-turn' | 'yaw' | 'bounce' | 'leave' | 'kick' | 'pivot'

const NO_TURN = [0]
const STRAIGHT_JUMP_AT = [0, 1, 2, 3, 4, 5, 6, 8, 10]
const RUN_TURN_JUMP_AT = [1, 2, 3, 4, 5, 6, 8]
const AXES8: ReadonlyArray<[number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1], [Math.SQRT1_2, Math.SQRT1_2], [Math.SQRT1_2, -Math.SQRT1_2], [-Math.SQRT1_2, Math.SQRT1_2], [-Math.SQRT1_2, -Math.SQRT1_2]]
const AIR_TURN_JUMP_AT = [0, 2, 4, 6]
/** The leave family arms immediately and turns at take-off or shortly after it. */
const LEAVE_AT = [0]
const LEAVE_TURN = [0, 2, 4]
const AIR_TURN_TICKS = [2, 4, 6, 8]
// Yaw offsets in radians from the landing bearing, gentlest first, for turns around pillars.
const YAW_RUN = [0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6, 0.9, -0.9]
const YAW_TURN = [0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2, 1.5, -1.5]
const YAW_TURN_TICKS = [1, 2, 3, 4, 6, 8]
const YAW_JUMP_AT = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14]
// Kick runs follow world axes within 60 degrees of the landing bearing.
// Leaning toward the landing and turning at the lip allow a run along a narrow strip.
const KICK_AXES: ReadonlyArray<[number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1]]
const KICK_AXIS_COS = 0.5
const KICK_LEAN = [0, 0.09, 0.17, 0.26]
const KICK_OUT = [0.35, 0.52, 0.7, 0.87, 1.05]
const KICK_JUMP_AT = Array.from({ length: 17 }, (_, i) => i)
const KICK_TURN_TICKS = [1, 2, 3, 4, 5, 6, 8]
// Enables steering back onto a planned kick lane during live searches.
export const LANE_KICK_TICKS = process.env.PF_LANE_KICK === '0' ? 0 : 40_000
// Pivot runs follow a world axis but face away from it at take-off to redirect the sprint boost.
// These precision programs are reserved for narrow landings.
const PIVOT_JUMP = Array.from({ length: 38 }, (_, i) => (i % 2 === 0 ? 1 : -1) * (0.3 + 0.05 * Math.floor(i / 2)))
const PIVOT_MIN = 0.25
const PIVOT_TURN = [0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2, 1.5, -1.5, 1.8, -1.8, 2.1, -2.1, 2.4, -2.4]
const PIVOT_TURN_TICKS = [1, 2, 3, 4, 5, 6, 7, 8]
const PIVOT_JUMP_AT = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16]
/** Stand points spanning less than this across one axis are a narrow landing, or take-off (a 3/16 strip spans 0). */
const PIVOT_NARROW = 0.25
/** pointAims: how far a stand point may lie off the landing disc's circle and still be aimed at. */
const PIVOT_POINT_MISS = 0.3
/** The spread of points along one axis (0: x, 1: z). */
function spanOf (ps: ReadonlyArray<{ x: number, z: number }>, axis: number): number {
  let lo = Infinity
  let hi = -Infinity
  for (const q of ps) { const v = axis === 0 ? q.x : q.z; lo = Math.min(lo, v); hi = Math.max(hi, v) }
  return ps.length === 0 ? 0 : hi - lo
}
// Bounce candidates are slime tops within reach of both legs, ranked by their combined distance.
const BOUNCE_REACH = 7
const BOUNCE_DROP = 14
const BOUNCE_SLIMES = 12
/** Maximum drop below take-off for deep bounce candidates, in blocks. */
const BOUNCE_DROP_LONG = 72
/** How far (blocks, horizontally) a sprint jump carries while falling `drop` below its take-off: an over-estimate (0.6 a tick at take-off, a block of slack). */
const fallMemo = new Map<number, number>()
function fallReach (drop: number): number {
  const hit = fallMemo.get(drop)
  if (hit !== undefined) return hit
  let y = 0
  let vy = 0.42
  let t = 0
  while (y > -drop && t < 400) { y += vy; vy = (vy - 0.08) * 0.98; t++ }
  let d = 0
  let v = 0.6
  for (let k = 0; k < t; k++) { d += v; v = v * 0.91 + 0.02548 }
  fallMemo.set(drop, d + 1)
  return d + 1
}
const BOUNCE_JUMP_AT = [0, 1, 2, 3, 4, 6, 8, 12]
/** How far short of a slime top's centre (blocks, along the way in) the bounce family's first phase also aims. */
const BOUNCE_SHORT = [0, 0.3]
/** Candidate limits for bounce chains when a single rebound cannot reach the landing. */
const BOUNCE_CHAIN_FIRST = 4
/** Optimistic rebound reach from fall height `drop` to landing height `rise`, or -1 if unreachable.
 * Assumes sprint air acceleration from speed 0.4 and adds one block of slack. */
const reboundMemo = new Map<number, number>()
/** Deep rebound reach plus vanilla's sprint boost: 0.2 decaying by 0.91 per tick, bounded by 2.3 blocks. */
function deepReach (drop: number, rise: number): number {
  const r = reboundReach(drop, rise)
  return r < 0 ? r : r + (isVanillaJump() ? 2.3 : 0)
}
function reboundReach (drop: number, rise: number): number {
  const key = Math.round(drop * 64) * 4096 + Math.round(rise * 64)
  const hit = reboundMemo.get(key)
  if (hit !== undefined) return hit
  let y = drop
  let vy = 0.42
  for (let t = 0; t < 200; t++) {
    const ny = y + vy
    if (ny <= 0) break
    y = ny
    vy = (vy - 0.08) * 0.98
  }
  let up = -vy
  y = 0
  let ticks = 0
  let apex = 0
  for (; ticks < 200;) {
    y += up
    up = (up - 0.08) * 0.98
    ticks++
    apex = Math.max(apex, y)
    if (up < 0 && y <= rise) break
  }
  let reach = -1
  if (apex >= rise) {
    let d = 0
    let v = 0.4
    for (let k = 0; k < ticks; k++) { d += v; v = v * 0.91 + 0.02548 }
    reach = d + 1
  }
  reboundMemo.set(key, reach)
  return reach
}
const BOUNCE_CHAIN_NEXT = 3
const BOUNCE_CHAIN = 2
/** Finite mark beyond any air count, so advance stops at the next rebound. */
const TO_NEXT_REBOUND = Number.MAX_SAFE_INTEGER

const FAMILIES: Record<HopFamilies, readonly Family[]> = {
  simple: ['straight', 'run-turn', 'air-turn', 'bounce'],
  yaw: ['yaw'],
  yawKick: ['yaw', 'kick'],
  kick: ['kick'],
  // Pivots stay out of the ordinary live sweep; seeded searches give them a separate budget.
  pivot: ['pivot'],
  // Deep landing candidates use only deep bounce classes.
  deep: ['bounce'],
  // Try cheap bounce candidates before the yaw grid so live searches leave time for slime routes.
  all: ['straight', 'run-turn', 'air-turn', 'bounce', 'yaw', 'leave'],
  beam: [],
  bounce: ['bounce']
}

/** Flight body and phase counters; `top` tracks peak height since support for the fall-damage veto. */
interface Flight { b: SimBody, t: number, fired: boolean, air: number, rebounds: number, yaw: number, leg: number, top: number }

/** Initial search state: at rest on a stand, or at an exact live body state. */
interface Pad { f: Flight, live: SimBody | undefined }

const newFlight = (): Flight => ({ b: newBody(0, 0, 0), t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: -Infinity })

function flightInto (dst: Flight, src: Flight): Flight {
  copyBody(dst.b, src.b)
  dst.t = src.t
  dst.fired = src.fired
  dst.air = src.air
  dst.rebounds = src.rebounds
  dst.yaw = src.yaw
  dst.leg = src.leg
  dst.top = src.top
  return dst
}

/** advance(): the flight reached its mark and goes on. */
const FLYING = -2
/** beamSearch: a nudged twin let go at the end of the lead body's first leg. */
const RELEASED = -3
/** A chain's first real leg ends a block or more from its start (robust(), beamSearch). */
const LEG_MIN = 1
/** Key flags of a beam program's tick (HopProgram.seq). */
export const F_FWD = 1
export const F_SPRINT = 2
export const F_JUMP = 4
/** bot.look rounds yaw changes to whole 0.15-degree steps. */
export const LOOK_STEP = ((((Math.PI / 180) * -0.15 + Math.PI) % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2) - Math.PI

/** Beam state: an ensemble of flying or landed bodies and the control that reaches it. */
interface BeamNode { fs: Flight[], done: number[], parent: number, want: number, flags: number, pad: number, score: number, wp: number }
/** Reserve grounded beam states so early jumps cannot crowd out later take-offs.
 * Ranks use optimistic sprint-jump speed and landing-disc margins. */
const BEAM_WIDTH = 160
const BEAM_WIDTH_GROUND = 60
export const BEAM_DEPTH = 80
const BEAM_OFFSETS = [0, 0.15, -0.15, 0.35, -0.35, 0.6, -0.6, 0.9, -0.9, 1.25, -1.25, 1.65, -1.65, 2.2, -2.2, Math.PI]
/** World axes keep runs aligned with rails and ledges regardless of the landing bearing. */
const BEAM_AXES = [0, Math.PI / 2, Math.PI, -Math.PI / 2]
const BEAM_SPEED = 0.36
const BEAM_SLACK = 0.02
const BEAM_MISS_WEIGHT = 6
/** A block of height to climb back, in ticks; and how near (blocks) a corridor waypoint counts as passed. */
const BEAM_CLIMB = 10
const BEAM_WAYPOINT = 0.8
/** Sprinting ground speed (blocks/tick) a grounded rank expects a body to have (spinUp). */
const BEAM_SPRINT_SPEED = 0.28
/** How far (blocks) below a leg's landing height something may carry it and the leg still count (footing). */
const BEAM_FOOTING = 3
/** Landings per layer the beam re-flies for robustness (the rest of a layer's are near copies). */
const BEAM_CHECKS_PER_LAYER = 24
/** Enables per-layer beam diagnostics on stderr. */
const BEAM_DEBUG = typeof process !== 'undefined' && process.env.PF_BEAM_DEBUG === '1'
/** Enables ranking by obstacle detours when the beam has no corridor. */
const BEAM_DETOUR = typeof process === 'undefined' || process.env.PF_BEAM_DETOUR !== '0'
/** Detour grid spacing, bounds, jump clearance and height sampling, in blocks.
 * Unreachable points receive a capped penalty rather than being discarded. */
const DETOUR_STEP = 0.125
const DETOUR_MARGIN = 3
const DETOUR_HALF = 7
const DETOUR_CLEAR = 1
const DETOUR_DY = 0.5
const DETOUR_CAP = 6
const DETOUR_EPS = 0.05

/** Obstacle detour cost: free-space octile distance minus the unobstructed distance; zero outside the grid.
 * Blocked points use the cheapest free neighbour or DETOUR_CAP. */
class DetourField {
  constructor (readonly x0: number, readonly z0: number, readonly nx: number, readonly nz: number, readonly free: Uint8Array, readonly excess: Float32Array) {}

  at (x: number, z: number): number {
    const i = Math.round((x - this.x0) / DETOUR_STEP)
    const j = Math.round((z - this.z0) / DETOUR_STEP)
    if (i < 0 || j < 0 || i >= this.nx || j >= this.nz) return 0
    const k = j * this.nx + i
    if (this.free[k] === 1) return this.excess[k]
    let best = DETOUR_CAP
    for (let dj = -1; dj <= 1; dj++) {
      for (let di = -1; di <= 1; di++) {
        const ii = i + di
        const jj = j + dj
        if (ii < 0 || jj < 0 || ii >= this.nx || jj >= this.nz) continue
        const kk = jj * this.nx + ii
        if (this.free[kk] === 1 && this.excess[kk] < best) best = this.excess[kk]
      }
    }
    return best
  }
}

/** The octile length of (dx, dz): an 8-neighbour grid's shortest way with nothing in it. */
const octile = (dx: number, dz: number): number => {
  const a = Math.abs(dx)
  const b = Math.abs(dz)
  return Math.max(a, b) + (Math.SQRT2 - 1) * Math.min(a, b)
}

/** Quantized beam merge key: coarse ground positions preserve take-off variety; velocity preserves chain momentum.
 * Air positions and velocities use finer bins; integer tuples avoid allocating strings per child. */
function beamKeyInto (f: Flight, out: Int32Array, at: number): void {
  const b = f.b
  if (!f.fired) {
    out[at] = 0
    out[at + 1] = Math.round(b.x * 8); out[at + 2] = Math.round(b.y * 8); out[at + 3] = Math.round(b.z * 8)
    out[at + 4] = Math.round(b.vx * 20); out[at + 5] = Math.round(b.vz * 20); out[at + 6] = 0
    return
  }
  out[at] = b.onGround ? 2 : 1
  out[at + 1] = Math.round(b.x * 16); out[at + 2] = Math.round(b.y * 16); out[at + 3] = Math.round(b.z * 16)
  out[at + 4] = Math.round(b.vx * 64); out[at + 5] = Math.round(b.vy * 32); out[at + 6] = Math.round(b.vz * 64)
}
const BEAM_KEY_LEN = 7
/** Hash of the beam key; collisions are resolved by comparing the full tuples. */
function beamHash (k: Int32Array, at: number): number {
  let h = 0x811c9dc5
  for (let i = 0; i < BEAM_KEY_LEN; i++) h = Math.imul(h ^ k[at + i], 0x01000193)
  return h
}
/** Ticks jump may be held without taking off (a ceiling, a ladder) before the program is given up. */
const JUMP_GRACE = 6
/** Enables detecting jump firing through cooldown re-arming when a ceiling blocks the rise. */
export const BONK_FIRE = process.env.PF_BONK_FIRE !== '0'
export function jumpRearmed (before: number | undefined, after: number | undefined, cooldown: number = SIM.autojumpCooldown): boolean {
  return BONK_FIRE && before !== undefined && after === cooldown && before < cooldown
}
/** Longest run on the ground, and longest flight, a program may have (ticks). */
const MAX_RUN = 40
/** Ticks a leave program may run before its body comes off the edge it runs at. */
const LEAVE_RUN = 20
const MAX_AIR = 60
/** Caps rebound air-time credits so a body trapped over slime cannot bounce forever. */
const REBOUND_CAP = process.env.PF_REBOUND_CAP !== '0'
const REBOUND_CREDIT = 4
/** Enables hazard checks on touch-and-go landings. */
const TOUCH_WET = process.env.PF_TOUCH_WET !== '0'
/** Enables a total flight limit because touch-and-go legs reset their own limits. */
export const FLIGHT_CAP = process.env.PF_FLIGHT_CAP !== '0'
export const FLIGHT_TICKS_MAX = 600
/** A bounce's first air phase lasts to the rebound: a finite mark past any flight, so advance() stops there. */
const TO_REBOUND = [MAX_AIR]

/** Air acceleration along the heading with forward held (prismarine: 0.98 input, +30% sprinting). */
const airAccel = (sprint: boolean): number => (SIM.airborneAcceleration + (sprint ? SIM.airborneAcceleration * 0.3 : 0)) * 0.98
/** Headings aimHeadings sweeps (a full turn), and how far inside the column a touchdown must be. */
const AIM_STEPS = 720
const AIM_EDGE = 0.001
/** Minimum footprint overlap with the landing column; the body's centre may remain outside it. */
export const LAND_OVERLAP = 0.1
const AIM_COS = Float64Array.from({ length: AIM_STEPS }, (_, i) => Math.cos((i / AIM_STEPS) * 2 * Math.PI))
const AIM_SIN = Float64Array.from({ length: AIM_STEPS }, (_, i) => Math.sin((i / AIM_STEPS) * 2 * Math.PI))
/** Where across a landing arc the extra headings go, and how far (sweep steps) from the best one they must be. */
const AIM_SPREAD = [0.2, 0.8]
/** Enables pruning flights that cannot reach the landing. */
const REACH_PRUNE = typeof process === 'undefined' || process.env.PF_REACH_PRUNE !== '0'
const AIM_DISTINCT = 4

function pushAim (out: Heading[], i: number): void {
  out.push(dir(AIM_COS[i], AIM_SIN[i]))
}

export class HopOracle {
  private readonly sim: PlayerSim
  private readonly world: SimWorld
  private readonly flagsAt: (x: number, y: number, z: number) => number
  private readonly standCache = new Map<number, Stand[]>()
  /** Exposed slime tops within bounce reach below a take-off cell, by cell (slimeTops). */
  private readonly slimeCache = new Map<number, Array<[number, number, number]>>()
  private readonly deepSlimeCache = new Map<number, Array<[number, number, number]>>()
  /** Scratch flights of the search tree: run, launch, first air phase, branch. */
  private readonly flights: Flight[] = [newFlight(), newFlight(), newFlight(), newFlight()]
  /** Scratch of aimHeadings' sweep. */
  private readonly aimIns = new Uint8Array(AIM_STEPS)
  private readonly aimScore = new Float64Array(AIM_STEPS)
  private readonly aimMargin = new Float64Array(AIM_STEPS)
  /** Scratch of landingDisc, and of tickFlight's stay test on a copy. */
  private readonly disc = new Float64Array(7)
  private readonly probe = newBody(0, 0, 0)
  private readonly aimPicked = new Int32Array(8)
  private readonly ctl: SimControl = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false, hx: 0, hz: -1 }
  /** Moving line-up checks replay controls on a bounded budget instead of running a live search. */
  private anchorDeadline: number | null = null
  private prefixBudget: { end: number, deadline: number } | null = null
  /** The body's half-width (the kernel's). */
  get halfWidth (): number { return this.sim.halfWidth }
  /** Kernel ticks flown by the last discover() (diagnostics). */
  ticksFlown = 0
  /** The last hop search has no verdict: its budget expires or a take-off or landing stand is unavailable.
   * False means all requested families finish searching. */
  noVerdict = false
  /** The last hop lacks a take-off or landing stand; also sets noVerdict, since this does not disprove the jump. */
  noStand = false
  /** Kernel tick budget in addition to the deadline; 0 disables it.
   * A work limit keeps live-search results independent of machine load. */
  tickLimit = 0
  /** discover(): the hop being flown is a goal-side bridge (robust() holds it to SIDE_NUDGE). */
  private sideBridge = false
  /** Obstacle detour ranking enabled for this oracle. */
  beamDetour = BEAM_DETOUR
  /** Reject touchdowns costing this many health points or more; lowered by a health floor. */
  fallVeto = FALL_VETO

  constructor (world: SimWorld, flagsAt: (x: number, y: number, z: number) => number, halfWidth: number, height: number, speedBase = 0.1) {
    this.world = world
    this.flagsAt = flagsAt
    this.sim = new PlayerSim(world, halfWidth, height, speedBase)
  }

  private kindAt (x: number, y: number, z: number): number {
    const s = this.sim.stateAt(x, y, z)
    return s < 0 ? 0 : this.world.kind[s]
  }

  /** The top of the cell's highest collision box, relative to the cell (0 when it has none). */
  private cellTop (x: number, y: number, z: number): number {
    const w = this.world
    const s = this.sim.stateAt(x, y, z)
    if (s < 0 || w.shapeStart[s] < 0) return 0
    let top = 0
    for (let k = 0; k < w.shapeCount[s]; k++) top = Math.max(top, w.boxes[(w.shapeStart[s] + k) * 6 + 4])
    return top
  }

  /** Whether a cardinal neighbour has no collision at feet, head or below, and no footing a block lower. */
  openDrop (x: number, y: number, z: number): boolean {
    for (const [ex, ez] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const nx = x + ex
      const nz = z + ez
      if (!this.shaped(nx, y, nz) && !this.shaped(nx, y + 1, nz) && !this.shaped(nx, y - 1, nz) &&
          this.stands(nx, y, nz).length === 0 && this.stands(nx, y - 1, nz).length === 0) return true
    }
    return false
  }

  /** Whether the node rests on ice with slipperiness at least 0.9, where a line-up slides. */
  onIce (x: number, y: number, z: number): boolean {
    const s = this.sim.stateAt(x, y - 1, z)
    return s >= 0 && this.world.slip[s] >= 0.9
  }

  /** Does the cell hold any collision box? */
  private shaped (x: number, y: number, z: number): boolean {
    const s = this.sim.stateAt(x, y, z)
    return s >= 0 && this.world.shapeStart[s] >= 0 && this.world.shapeCount[s] > 0
  }

  /** Planner node for a body at `feetY`: one cell above a physical block unless it is a thin floor. */
  nodeY (x: number, feetY: number, z: number): number {
    const fy = Math.floor(feetY + 1e-9)
    const f = this.flagsAt(x, fy, z)
    const physical = (f & F_PHYSICAL) !== 0
    const thinFloor = (f & (F_SAFE | F_PHYSICAL | F_CLIMBABLE)) === (F_SAFE | F_PHYSICAL)
    return physical && !thinFloor ? fy + 1 : fy
  }

  // Stand discovery and landing checks share the retreat's hard budget.
  private kernelStep (b: SimBody, c: SimControl, counted = true): void {
    const budget = this.prefixBudget
    if (budget !== null && (this.ticksFlown >= budget.end || performance.now() > budget.deadline ||
        (this.abort !== null && this.abort()))) throw PREFIX_EXPIRED
    this.sim.step(b, c)
    if (counted || budget !== null) this.ticksFlown++
  }

  private settle (b: SimBody): boolean {
    const c = this.ctl
    c.forward = false; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = false
    for (let t = 0; t < 30; t++) {
      this.kernelStep(b, c, false)
      if (b.inLiquid) return false
      if (b.onGround && Math.abs(b.vx) < 1e-3 && Math.abs(b.vz) < 1e-3) return true
    }
    return false
  }

  /** Where a body can stand on node (x, y, z): rest positions carried by that column; a climbable catch. */
  stands (x: number, y: number, z: number): Stand[] {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.standCache.get(k)
    if (hit !== undefined) return hit
    const out: Stand[] = []
    if (this.sim.stateAt(x, y, z) >= 0 && this.sim.stateAt(x, y + 1, z) >= 0) {
      if ((this.kindAt(x, y, z) & SIM_CLIMBABLE) !== 0) {
        out.push({ x: x + 0.5, y, z: z + 0.5, catch: true })
      } else if (this.shaped(x, y, z) || this.shaped(x, y - 1, z) || this.shaped(x, y - 2, z)) {
        // Drop from just above the support boxes to avoid starting inside a ceiling, fence or wall.
        const drop = y + Math.max(this.cellTop(x, y, z), this.cellTop(x, y - 1, z) - 1) + 0.001
        const tryAt = (ox: number, oz: number): void => {
          const b = newBody(x + ox, drop, z + oz)
          // Start clear of every box; a body dropped inside a pane or panel can fall through and appear to rest embedded.
          if (this.sim.collides(b)) return
          if (!this.settle(b)) return
          if (Math.floor(b.x) !== x || Math.floor(b.z) !== z) return
          if (this.nodeY(x, b.y, z) !== y) return
          if (this.sim.collides(b) || !this.sim.supportedIn(b, x, z)) return
          if (!out.some(p => Math.abs(p.x - b.x) < 0.05 && Math.abs(p.z - b.z) < 0.05)) out.push({ x: b.x, y: b.y, z: b.z, catch: false })
        }
        for (const ox of [0.2, 0.5, 0.8]) for (const oz of [0.2, 0.5, 0.8]) tryAt(ox, oz)
        // Sample support edges too: the grid can miss narrow ledges such as an open trapdoor's top edge.
        const w = this.world
        const e = this.sim.halfWidth - LAND_OVERLAP
        for (const yy of [y - 1, y]) {
          const s = this.sim.stateAt(x, yy, z)
          if (s < 0 || w.shapeStart[s] < 0) continue
          for (let k = 0; k < w.shapeCount[s]; k++) {
            const o = (w.shapeStart[s] + k) * 6
            // A neighbouring box can leave only a narrow window beside the ledge.
            const x0 = w.boxes[o]; const x1 = w.boxes[o + 3]
            const z0 = w.boxes[o + 2]; const z1 = w.boxes[o + 5]
            for (const ox of [(x0 + x1) / 2, x0 - e, x0 + e, x1 - e, x1 + e]) {
              for (const oz of [(z0 + z1) / 2, z0 - e, z0 + e, z1 - e, z1 + e]) {
                if (ox > 0.01 && ox < 0.99 && oz > 0.01 && oz < 0.99) tryAt(ox, oz)
              }
            }
          }
        }
        // Sample beside obstacles too: a post can leave a standable sliver outside every grid point.
        const low = y + Math.max(0, this.cellTop(x, y - 1, z) - 1) + 0.001
        const hw = this.sim.halfWidth
        const tryLow = (ox: number, oz: number): void => {
          const b = newBody(x + ox, low, z + oz)
          if (this.sim.collides(b)) return
          if (!this.settle(b)) return
          if (Math.floor(b.x) !== x || Math.floor(b.z) !== z) return
          if (this.nodeY(x, b.y, z) !== y) return
          if (this.sim.collides(b) || !this.sim.supportedIn(b, x, z)) return
          if (!out.some(p => Math.abs(p.x - b.x) < 0.05 && Math.abs(p.z - b.z) < 0.05)) out.push({ x: b.x, y: b.y, z: b.z, catch: false })
        }
        // Cross box-face offsets to find corners left free by posts with arms.
        const oxs = new Set<number>([0.5])
        const ozs = new Set<number>([0.5])
        for (const yy of [y, y + 1]) {
          const s = this.sim.stateAt(x, yy, z)
          if (s < 0 || w.shapeStart[s] < 0) continue
          for (let k = 0; k < w.shapeCount[s]; k++) {
            const o = (w.shapeStart[s] + k) * 6
            for (const ox of [w.boxes[o] - hw - 0.01, w.boxes[o + 3] + hw + 0.01]) if (ox > 0.01 && ox < 0.99) oxs.add(ox)
            for (const oz of [w.boxes[o + 2] - hw - 0.01, w.boxes[o + 5] + hw + 0.01]) if (oz > 0.01 && oz < 0.99) ozs.add(oz)
          }
        }
        // Require a full support top; a sliver off a fence or post can leave no room for take-off.
        let fullTop = false
        {
          const s = this.sim.stateAt(x, y - 1, z)
          if (s >= 0 && w.shapeStart[s] >= 0) {
            const top = this.cellTop(x, y - 1, z)
            for (let k = 0; k < w.shapeCount[s] && !fullTop; k++) {
              const o = (w.shapeStart[s] + k) * 6
              fullTop = w.boxes[o + 4] >= top - 1e-9 && w.boxes[o] <= 1e-9 && w.boxes[o + 2] <= 1e-9 && w.boxes[o + 3] >= 1 - 1e-9 && w.boxes[o + 5] >= 1 - 1e-9
            }
          }
        }
        if (fullTop && (oxs.size > 1 || ozs.size > 1)) for (const ox of oxs) for (const oz of ozs) tryLow(ox, oz)
      }
    }
    this.standCache.set(k, out)
    return out
  }

  /** Unobstructed touchdown at `landY` under heading `h`, or null if not reached within MAX_AIR.
   * Used to reject flights whose landing misses by more than BALLISTIC_MISS. */
  private ballisticLanding (b: SimBody, h: Heading, sprint: boolean, landY: number): [number, number] | null {
    let x = b.x; let y = b.y; let z = b.z
    let vx = b.vx; let vy = b.vy; let vz = b.vz
    const accel = airAccel(sprint)
    for (let t = 0; t < MAX_AIR; t++) {
      const d = headingDir(h, x, z)
      if (d !== null) { vx += accel * d[0]; vz += accel * d[1] }
      const py = y
      x += vx; y += vy; z += vz
      if (vy < 0 && py >= landY && y <= landY) return [x, z]
      vx *= SIM.airborneInertia; vz *= SIM.airborneInertia
      vy = (vy - SIM.gravity) * SIM.airdrag
      if (y < landY - 3) return null
    }
    return null
  }

  /** Unobstructed landing disc at `landY`: [tick, cx, cz, radius, previous cx, previous cz, previous radius], or null.
   * Fixed headings land on its circle; mixed headings land inside. The previous disc locates support before Y moves. */
  private landingDisc (b: SimBody, sprint: boolean, landY: number): Float64Array | null {
    let y = b.y
    let vy = b.vy
    let s = 0
    for (;;) {
      if (s >= MAX_AIR) return null
      const py = y
      y += vy
      s++
      if (vy < 0 && py >= landY && y <= landY) break
      if (vy < 0 && y < landY) return null
      vy = (vy - SIM.gravity) * SIM.airdrag
    }
    const drag = SIM.airborneInertia
    const a = airAccel(sprint)
    const sum = (1 - Math.pow(drag, s)) / (1 - drag)
    const sum1 = (1 - Math.pow(drag, s - 1)) / (1 - drag)
    const d = this.disc
    d[0] = s
    d[1] = b.x + b.vx * sum
    d[2] = b.z + b.vz * sum
    d[3] = a * (sum + (s - sum) / (1 - drag))
    // Y moves first, so support is determined at the start of the crossing tick.
    d[4] = b.x + b.vx * sum1
    d[5] = b.z + b.vz * sum1
    d[6] = a * (sum1 + (s - 1 - sum1) / (1 - drag))
    return d
  }

  /** Whether the remaining fall time, carried speed and air acceleration can reach the landing column.
   * Uses a lower landing level when a step-up can carry the body onto it. */
  private canReach (b: SimBody, sprint: boolean, tx: number, ty: number, tz: number): boolean {
    let y = b.y
    let vy = b.vy
    let n = 0
    while (n < MAX_AIR && !(vy < 0 && y < ty)) {
      y += vy
      vy = (vy - SIM.gravity) * SIM.airdrag
      n++
    }
    const drag = SIM.airborneInertia
    const sum = (1 - Math.pow(drag, n)) / (1 - drag)
    const reach = Math.hypot(b.vx, b.vz) * sum + airAccel(sprint) * (n - drag * sum) / (1 - drag)
    // Allow the footprint to overhang the column by the landing overlap margin.
    const e = this.sim.halfWidth - LAND_OVERLAP
    const dx = Math.max(tx - e - b.x, 0, b.x - tx - 1 - e)
    const dz = Math.max(tz - e - b.z, 0, b.z - tz - 1 - e)
    return dx * dx + dz * dz <= (reach + 1e-6) * (reach + 1e-6)
  }

  /** Is there a box with its top a step under `y` (and below it) in column (tx, tz) or one beside it (canReach's step-up allowance)? */
  private stepUnder (tx: number, tz: number, y: number): boolean {
    const w = this.world
    for (let x = tx - 1; x <= tx + 1; x++) {
      for (let z = tz - 1; z <= tz + 1; z++) {
        for (let yy = Math.floor(y - SIM.stepHeight) - 1; yy <= Math.floor(y); yy++) {
          const s = this.sim.stateAt(x, yy, z)
          if (s < 0 || w.shapeStart[s] < 0) continue
          for (let k = 0; k < w.shapeCount[s]; k++) {
            const top = yy + w.boxes[(w.shapeStart[s] + k) * 6 + 4]
            if (top < y - 1e-6 && top >= y - SIM.stepHeight - 1e-6) return true
          }
        }
      }
    }
    return false
  }

  /** Is the search out of budget: its work (tickLimit), or the clock at `deadline`? */
  private expired (deadline: number): boolean {
    return (this.tickLimit > 0 && this.ticksFlown >= this.tickLimit) || performance.now() > deadline || (this.abort !== null && this.abort())
  }

  /** The search's owner has given it up (the worker's cancel flag): every search ends as if out of time. */
  abort: (() => boolean) | null = null

  /** Append fixed headings toward stands near the landing circle; narrow strips need finer aim than column targeting. */
  private pointAims (b: SimBody, sprint: boolean, landY: number, lands: readonly Stand[], out: Heading[]): void {
    const disc = this.landingDisc(b, sprint, landY)
    if (disc === null) return
    for (const st of lands) {
      const dx = st.x - disc[1]
      const dz = st.z - disc[2]
      const d = Math.hypot(dx, dz)
      if (d < 1e-9 || Math.abs(d - disc[3]) > PIVOT_POINT_MISS) continue
      out.push(dir(dx / d, dz / d))
    }
  }

  private aimHeadings (b: SimBody, sprint: boolean, tx: number, tz: number, landY: number, out: Heading[]): void {
    out.length = 0
    const disc = this.landingDisc(b, sprint, landY)
    if (disc === null) return
    const s = disc[0]
    const cx = disc[1]
    const cz = disc[2]
    const r = disc[3]
    const a = airAccel(sprint)
    const drag = SIM.airborneInertia
    // Rest drift includes landing-tick air drag, then ground friction: velocity times 1/(1 - slip*0.91).
    const under = this.sim.stateAt(tx, Math.floor(landY - 1), tz)
    const slip = under >= 0 && this.world.slip[under] > 0 ? this.world.slip[under] : SIM.defaultSlipperiness
    const slide = drag / (1 - slip * SIM.airborneInertia)
    const q1 = Math.pow(drag, s - 1)
    // w_{s-1} = q1·(v0 + a·u) + a·u·(1 - q1)/(1 - drag)
    const rr = r + slide * a * (q1 + (1 - q1) / (1 - drag))
    const rcx = cx + slide * q1 * b.vx
    const rcz = cz + slide * q1 * b.vz
    // A landing arc exists only where the circle crosses the column, widened by the allowed footprint overhang.
    const e = this.sim.halfWidth - LAND_OVERLAP
    const lox = tx - e - cx
    const hix = lox + 1 + 2 * e
    const loz = tz - e - cz
    const hiz = loz + 1 + 2 * e
    const nx = lox > 0 ? lox : hix < 0 ? hix : 0
    const nz = loz > 0 ? loz : hiz < 0 ? hiz : 0
    const fx = Math.max(-lox, hix)
    const fz = Math.max(-loz, hiz)
    if (r * r < nx * nx + nz * nz || r * r > fx * fx + fz * fz) return
    let base = 0
    let n = AIM_STEPS
    if (nx !== 0 || nz !== 0) {
      const t0 = Math.atan2(loz, lox)
      let dMin = 0
      let dMax = 0
      for (let c = 1; c < 4; c++) {
        let d = Math.atan2(c < 2 ? loz : hiz, (c & 1) === 1 ? hix : lox) - t0
        if (d > Math.PI) d -= 2 * Math.PI
        else if (d < -Math.PI) d += 2 * Math.PI
        if (d < dMin) dMin = d
        if (d > dMax) dMax = d
      }
      const step = (2 * Math.PI) / AIM_STEPS
      base = (((Math.floor((t0 + dMin) / step) - 1) % AIM_STEPS) + AIM_STEPS) % AIM_STEPS
      n = Math.min(AIM_STEPS, Math.ceil((dMax - dMin) / step) + 3)
    }
    const midX = tx + 0.5
    const midZ = tz + 0.5
    const ins = this.aimIns
    const score = this.aimScore
    const margin = this.aimMargin
    let outAt = -1
    for (let j = 0; j < n; j++) {
      const i = (base + j) % AIM_STEPS
      const ux = AIM_COS[i]
      const uz = AIM_SIN[i]
      const lx = cx + r * ux
      const lz = cz + r * uz
      if (lx > tx - e + AIM_EDGE && lx < tx + 1 + e - AIM_EDGE && lz > tz - e + AIM_EDGE && lz < tz + 1 + e - AIM_EDGE) {
        ins[i] = 1
        const ex = rcx + rr * ux - midX
        const ez = rcz + rr * uz - midZ
        score[i] = ex * ex + ez * ez
        margin[i] = Math.min(lx - tx + e, tx + 1 + e - lx, lz - tz + e, tz + 1 + e - lz)
      } else {
        ins[i] = 0
        outAt = j
      }
    }
    // Sample the deepest touchdown, the most central rest and two spread headings per arc.
    // Obstacles can block part of an otherwise valid landing arc.
    const from = base + (n === AIM_STEPS ? outAt + 1 : 0)
    let j = 0
    while (j < n) {
      if (ins[(from + j) % AIM_STEPS] === 0) { j++; continue }
      let len = 0
      let best = (from + j) % AIM_STEPS
      let widest = best
      while (j + len < n && ins[(from + j + len) % AIM_STEPS] === 1) {
        const k = (from + j + len) % AIM_STEPS
        if (score[k] < score[best]) best = k
        if (margin[k] > margin[widest]) widest = k
        len++
      }
      const first = out.length
      const pick = (k: number): void => {
        for (let q = first; q < out.length; q++) {
          const gap = Math.abs(this.aimPicked[q - first] - k)
          if (Math.min(gap, AIM_STEPS - gap) < AIM_DISTINCT) return
        }
        this.aimPicked[out.length - first] = k
        pushAim(out, k)
      }
      pick(widest)
      pick(best)
      for (const f of AIM_SPREAD) pick((from + j + Math.round(f * (len - 1))) % AIM_STEPS)
      j += len
    }
  }

  /** A body come to rest at (x, y, z), as a flight at its first tick; null if it cannot rest there. */
  private pad (x: number, y: number, z: number): Flight | null {
    const b = newBody(x, y + 0.001, z)
    if (!this.settle(b)) return null
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: b.y }
  }

  /** Initial climbable hold: sneak with no movement for one tick to retain its hold velocity.
   * Ladders clamp vertical speed to zero; returns null outside a climbable. */
  private hang (x: number, y: number, z: number): Flight | null {
    const b = newBody(x, y, z)
    if (!this.sim.onClimbable(b)) return null
    const c = this.ctl
    c.forward = false; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = true
    this.sim.step(b, c)
    c.sneak = false
    if (b.inLiquid || !this.sim.onClimbable(b)) return null
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: b.y }
  }

  /** Initial states at each stand, or hanging against a climbable at two heights.
   * Planned hops use these states; live take-offs use the body's exact state. */
  private padsAt (x: number, y: number, z: number): Pad[] {
    const out: Pad[] = []
    for (const s of this.stands(x, y, z)) {
      if (!s.catch) {
        const f = this.pad(s.x, s.y, s.z)
        if (f !== null) out.push({ f, live: undefined })
        continue
      }
      // against the climbable's box (a ladder's 3/16 slab on one face);
      // anywhere in the cell for one without a box (a vine)
      let hx = x + 0.5
      let hz = z + 0.5
      const w = this.world
      const st = this.sim.stateAt(x, y, z)
      if (st >= 0 && w.shapeStart[st] >= 0 && w.shapeCount[st] > 0) {
        const o = w.shapeStart[st] * 6
        const hw = this.sim.halfWidth
        if (w.boxes[o + 3] - w.boxes[o] < 0.5) hx = w.boxes[o] < 0.01 ? x + w.boxes[o + 3] + hw : x + w.boxes[o] - hw
        if (w.boxes[o + 5] - w.boxes[o + 2] < 0.5) hz = w.boxes[o + 2] < 0.01 ? z + w.boxes[o + 5] + hw : z + w.boxes[o + 2] - hw
      }
      for (const fy of HANG_HEIGHTS) {
        const f = this.hang(hx, y + fy, hz)
        if (f !== null && !this.sim.collides(f.b)) out.push({ f, live: undefined })
      }
    }
    return out
  }

  /** Is a first state hanging on a climbable (a take-off off it: HopProgram.climb)? */
  private climbing (b: SimBody): boolean {
    return !b.onGround && this.sim.onClimbable(b)
  }

  /** Is node t's landing a slime top, sneaked onto (HopProgram.slimeLand)? */
  private slimeLanding (t: readonly [number, number, number], catchTarget: boolean): boolean {
    return !catchTarget && (this.kindAt(t[0], t[1] - 1, t[2]) & SIM_SLIME) !== 0
  }

  // A body flush against a wall cannot use an inward nudge that starts inside it.
  // Keep both sides when neither is clear, or when the original start already overlaps a box.
  private requiredNudges (p: Pick<HopProgram, 'start'>, n: number): Array<[number, number]> {
    const offsets: Array<[number, number]> = [[n, 0], [-n, 0], [0, n], [0, -n]]
    const start = p.start
    if (!LIVE_NUDGE_SAFE || start === undefined || n === 0 || this.sim.collides(start)) return offsets
    const b = copyBody(newBody(0, 0, 0), start)
    const hits = offsets.map(([nx, nz]) => {
      b.x = start.x + nx
      b.z = start.z + nz
      return this.sim.collides(b)
    })
    return offsets.filter((_, i) => !hits[i] || hits[i ^ 1])
  }

  /** Program `p`'s first state nudged by (nx, nz): its live start, or a body come to rest (or hanging, `climb`) at its start point. */
  private origin (p: HopProgram, nx: number, nz: number): Flight | null {
    if (p.start === undefined) return p.climb === true ? this.hang(p.sx + nx, p.sy, p.sz + nz) : this.pad(p.sx + nx, p.sy, p.sz + nz)
    const b = copyBody(newBody(0, 0, 0), p.start)
    b.x += nx
    b.z += nz
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: p.yaw0 ?? NaN, leg: 0, top: b.y }
  }

  /** Replay `p` from its nudged start; returns ticks to a stable landing or -1.
   * `track` receives positions after each tick up to touchdown. */
  fly (p: HopProgram, nx = 0, nz = 0, track: number[] | null = null): number {
    const f = this.origin(p, nx, nz)
    if (f === null) return -1
    const r = this.advance(f, p, Infinity, Infinity, track)
    return r === FLYING ? -1 : r
  }

  /** Advance the deterministic flight in place until landing, failure or a ground/air phase mark.
   * Returns landing ticks, -1 or FLYING; a finite air mark also stops at a slime rebound. */
  private advance (f: Flight, p: HopProgram, groundUntil: number, airUntil: number, track: number[] | null): number {
    const b = f.b
    const c = this.ctl
    const rebounds0 = f.rebounds
    const prefix = LIVE_PREFIX ? p.groundPrefix : undefined
    if (prefix !== undefined && prefix.length > 4) return -1
    for (;;) {
      if (this.anchorDeadline !== null && this.expired(this.anchorDeadline)) return -1
      if (f.fired) {
        if (f.air >= airUntil || (f.rebounds > rebounds0 && airUntil !== Infinity)) return FLYING
        if (f.air === p.turnAir && f.rebounds === 0 && !p.catchTarget && !b.onGround && !this.comesDownNear(b, p)) return -1
      } else if (f.t >= groundUntil) {
        return FLYING
      }
      const ground = prefix?.[f.t]
      if (ground !== undefined) {
        if (!this.groundTick(f, ground, p.sy, track)) return -1
        continue
      }
      if (p.seq !== undefined) {
        const k = Math.min(f.t, (p.seq.length >> 1) - 1) << 1
        this.seqControl(f, c, p.seq[k], p.seq[k + 1])
      } else {
        const d = headingDir(programHeading(p, f.fired, f.air, f.rebounds, f.t), b.x, b.z)
        if (d !== null) this.face(f, c, d[0], d[1])
        c.forward = true; c.back = false; c.left = false; c.right = false
        c.sprint = p.sprint
        c.jump = (!f.fired && f.t >= p.jumpAt && (p.jumpOnLeave !== true || (b.onGround && !this.sim.carried(b)))) ||
          (p.reboundJump === true && f.fired && f.rebounds > 0 && b.onGround)
      }
      c.sneak = sneakLanding(p, b.onGround, b.x, b.y, b.z, b.vy, this.sim.halfWidth)
      // Hold jump and steer toward the column centre to climb a caught vine curtain.
      // Without a wall to press against, the flight's own keys can slide the body off.
      if (p.deep === true && p.catchTarget && f.fired && b.y < p.ty && Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz && this.sim.onClimbable(b)) {
        const dx = p.tx + 0.5 - b.x
        const dz = p.tz + 0.5 - b.z
        const dd = Math.hypot(dx, dz)
        if (dd > 1e-9) this.face(f, c, dx / dd, dz / dd)
        c.forward = dd > CLIMB_ON_SLACK; c.sprint = false; c.jump = true; c.sneak = false
      }
      const r = this.tickFlight(f, p, track)
      if (r !== FLYING) return r
    }
  }

  /** Face (hx, hz), rounding known live yaw to bot.look's LOOK_STEP increments.
   * This keeps simulated headings identical to replay headings. */
  private face (f: Flight, c: SimControl, hx: number, hz: number): void {
    if (Number.isNaN(f.yaw)) { c.hx = hx; c.hz = hz; return }
    this.turnTo(f, c, Math.atan2(-hx, -hz))
  }

  private turnTo (f: Flight, c: SimControl, want: number): void {
    f.yaw += Math.round((want - f.yaw) / LOOK_STEP) * LOOK_STEP
    c.hx = -Math.sin(Math.PI - f.yaw)
    c.hz = Math.cos(Math.PI - f.yaw)
  }

  /** One tick of explicit controls (a beam program's): look at yaw `want` (NaN: no look), keys from `flags`. */
  private seqControl (f: Flight, c: SimControl, want: number, flags: number): void {
    if (!Number.isNaN(want)) {
      if (Number.isNaN(f.yaw)) { c.hx = -Math.sin(Math.PI - want); c.hz = Math.cos(Math.PI - want) } else this.turnTo(f, c, want)
    }
    c.forward = (flags & F_FWD) !== 0
    c.sprint = (flags & F_SPRINT) !== 0
    c.jump = (flags & F_JUMP) !== 0
    c.back = false; c.left = false; c.right = false; c.sneak = false
  }

  /** Step a flight and its phase counters under this.ctl; returns stable-landing ticks, -1 or FLYING. */
  private tickFlight (f: Flight, p: HopProgram, track: number[] | null): number {
    const b = f.b
    const jumpTicks = b.jumpTicks
    this.kernelStep(b, this.ctl)
    if (track !== null) track.push(b.x, b.y, b.z)
    const t = f.t++
    if (b.inLiquid) return -1
    // Repeated ceiling bonks can reset leg limits without ending the flight.
    if (FLIGHT_CAP && t >= FLIGHT_TICKS_MAX) return -1
    // Sneaking onto slime takes the whole fall, so its touchdown is subject to the fall-damage veto.
    if (b.onGround) {
      if (b.y < f.top - SAFE_FALL && fallDamagePoints(f.top - b.y, 0, this.kindAt(Math.floor(b.x), Math.floor(b.y - 0.2), Math.floor(b.z)), this.ctl.sneak, true) >= this.fallVeto) return -1
      f.top = b.y
    } else if (b.y > f.top || this.sim.onClimbable(b)) {
      f.top = b.y
    }
    if (!f.fired) {
      // Holding jump on a climbable only climbs; take-off occurs when the body leaves it.
      const onClimb = p.climb === true && !b.onGround && this.sim.onClimbable(b)
      if (b.vy > 0.3 || (!b.onGround && b.vy < -0.3) || (p.climb === true && !b.onGround && !onClimb) ||
          (!onClimb && jumpRearmed(jumpTicks, b.jumpTicks))) f.fired = true
      else if ((t >= p.jumpAt + JUMP_GRACE && !onClimb && p.jumpOnLeave !== true) || t - f.leg >= (p.jumpOnLeave === true ? LEAVE_RUN : MAX_RUN)) return -1
      else if (p.legs === true && f.leg > 0 && b.onGround && Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz &&
               this.nodeY(p.tx, b.y, p.tz) === p.ty && this.stays(copyBody(this.probe, b), p)) {
        // touch and go's last leg can be a slide: down short of the node on
        // ice, and carried into it on the ground
        return t + 1
      }
      return FLYING
    }
    // A body boxed in over slime can rebound forever without a cap.
    const credit = REBOUND_CAP ? Math.min(f.rebounds, REBOUND_CREDIT + (p.bounces?.length ?? 0)) : f.rebounds
    if (++f.air > MAX_AIR * (1 + credit + (p.deep === true ? 1 : 0))) return -1
    const inCol = Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz
    if (p.catchTarget) {
      if (inCol && b.y >= p.ty - 0.001 && b.y < p.ty + 1) return this.stays(b, p) ? t + 1 : -1
    }
    if (b.onGround) {
      if (b.vy > 0.05 && (this.kindAt(Math.floor(b.x), Math.floor(b.y - 0.2), Math.floor(b.z)) & SIM_SLIME) !== 0) {
        if (LIVE_PREFIX && (this.prefixBudget !== null || p.groundPrefix !== undefined)) return -1
        f.rebounds++
        if (this.reboundIsLanding && f.rebounds === 1) return TOUCH_WET && this.sim.touchesWet(b) ? -1 : t + 1
        return FLYING
      }
      if (!this.overCol(b, p.tx, p.tz) || this.nodeY(p.tx, b.y, p.tz) !== p.ty) {
        if (p.legs !== true) return -1
        // A nudged chain twin must finish a real leg, at least a block from its start, rather than hop in place.
        if (this.touchIsLanding && Math.hypot(b.x - p.sx, b.z - p.sz) >= LEG_MIN) return TOUCH_WET && this.sim.touchesWet(b) ? -1 : t + 1
        f.fired = false
        f.air = 0
        f.rebounds = 0
        f.leg = f.t
        return FLYING
      }
      // A chain stone needs a touchdown that preserves momentum rather than a stop.
      if (this.goLanding) {
        // The kernel checks hazards before moving, so the final move into lava needs a separate check.
        if (TOUCH_WET && this.sim.touchesWet(b)) return -1
        if (this.touch !== null) copyBody(this.touch, b)
        return t + 1
      }
      const offLevel = this.offLevel(b, p)
      if (!this.stays(b, p)) return -1
      // Arrival ticks include settling to the required region's level.
      return offLevel ? t + 1 + (this.stayAt >= 0 ? this.stayAt + 1 : STAY_TICKS) : t + 1
    }
    if (b.y < Math.min(p.sy, p.ty) - 14 - (p.deep === true ? BOUNCE_DROP_LONG : 0)) return -1
    return FLYING
  }

  /** Beam search over per-tick controls for flights outside the program families.
   * Each state includes nudged twins on identical controls, so accepted programs satisfy LIVE_NUDGE.
   * Ranks use the worst body's earliest landing; breadth-first layers find the quickest robust programs. */
  private beamSearch (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], deadline: number, corridor: ReadonlyArray<{ x: number, y: number, z: number }> = [], land?: { x: number, y: number, z: number }, depthLimit = BEAM_DEPTH): HopProgram | null {
    if (pads.length === 0 || lands.length === 0) return null
    const catchTarget = lands[0].catch
    const landY = lands[0].y
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const c = this.ctl
    // Corridor waypoints preserve the momentum chain along the planner's partial path.
    const way: Array<[number, number, number]> = [...corridor.map(n => [n.x, n.y, n.z] as [number, number, number]), [t[0], landY, t[2]]]
    const tail = new Float64Array(way.length)
    for (let i = way.length - 2; i >= 0; i--) tail[i] = tail[i + 1] + Math.hypot(way[i + 1][0] - way[i][0], way[i + 1][2] - way[i][2])
    const reach = (b: SimBody, wp: number): number => {
      while (wp < way.length - 1 && Math.hypot(way[wp][0] + 0.5 - b.x, way[wp][2] + 0.5 - b.z) <= BEAM_WAYPOINT && Math.abs(way[wp][1] - b.y) < 1.5) wp++
      return wp
    }
    const tpl: HopProgram = {
      sx: 0, sy: pads[0].f.b.y, sz: 0, run: point(tcx, tcz), jumpAt: Infinity, air1: point(tcx, tcz), turnAir: Infinity,
      air2: point(tcx, tcz), sprint: true, ticks: 0, tx: t[0], ty: t[1], tz: t[2], catchTarget, family: 'beam', legs: true,
      climb: this.climbing(pads[0].f.b), slimeLand: this.slimeLanding(t, catchTarget), land
    }
    const field = corridor.length === 0 ? this.detourField(t, landY, pads) : null
    const nodes: BeamNode[] = []
    let layer: number[] = []
    pads.forEach((pad, i) => {
      // Use the same nudged ensemble as robustness verification.
      const start: HopProgram = { ...tpl, sx: pad.f.b.x, sy: pad.f.b.y, sz: pad.f.b.z, start: pad.live, yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw }
      const nudges = this.requiredNudges(start, LIVE_NUDGE)
      const fs: Flight[] = [flightInto(newFlight(), pad.f)]
      for (const [nx, nz] of nudges) {
        const f = this.origin(start, nx, nz)
        if (f !== null) fs.push(f)
      }
      if (fs.length !== nudges.length + 1) return // a required start cannot rest there: no robust program from this pad
      nodes.push({ fs, done: fs.map(() => FLYING), parent: -1, want: NaN, flags: 0, pad: i, score: 0, wp: reach(pad.f.b, 0) })
      layer.push(nodes.length - 1)
    })
    const found: HopProgram[] = []
    const seen = new Map<number, number[]>()
    let keys = new Int32Array(1024 * BEAM_KEY_LEN)
    for (let depth = 0; depth < depthLimit && layer.length > 0 && found.length === 0; depth++) {
      if (this.expired(deadline)) { this.noVerdict = true; break }
      const children: BeamNode[] = []
      seen.clear()
      for (const ni of layer) {
        const n = nodes[ni]
        const lead = n.fs[0]
        const n0x = pads[n.pad].f.b.x
        const n0z = pads[n.pad].f.b.z
        // Jump on a climbable can gain height before pushing off.
        const grounded = n.fs.some((f, k) => n.done[k] === FLYING && (f.b.onGround || (tpl.climb === true && !f.fired && this.sim.onClimbable(f.b))))
        const aim = way[n.wp]
        const bearing = Math.atan2(aim[2] + 0.5 - lead.b.z, aim[0] + 0.5 - lead.b.x)
        const nh = BEAM_OFFSETS.length + BEAM_AXES.length
        for (let a = 0; a <= nh; a++) {
          const coast = a === nh
          for (let j = 0; j < (!coast && grounded ? 2 : 1); j++) {
            let want = NaN
            let flags = 0
            if (!coast) {
              const th = a < BEAM_OFFSETS.length ? bearing + BEAM_OFFSETS[a] : BEAM_AXES[a - BEAM_OFFSETS.length]
              want = Math.atan2(-Math.cos(th), -Math.sin(th))
              flags = F_FWD | F_SPRINT | (j === 1 ? F_JUMP : 0)
            }
            const fs: Flight[] = []
            const done = n.done.slice()
            let dead = false
            let flying = 0
            for (let k = 0; k < n.fs.length && !dead; k++) {
              // Landed bodies are no longer stepped, so their states can be shared.
              if (done[k] !== FLYING) { fs.push(n.fs[k]); continue } // landed already: the replay has moved on
              // Release chain twins after the lead body completes its first real leg.
              if (k > 0 && fs[0].leg > 0 && Math.hypot(fs[0].b.x - n0x, fs[0].b.z - n0z) >= LEG_MIN) { done[k] = RELEASED; fs.push(n.fs[k]); continue }
              const f = flightInto(newFlight(), n.fs[k])
              fs.push(f)
              this.seqControl(f, c, want, flags)
              c.sneak = sneakLanding(tpl, f.b.onGround, f.b.x, f.b.y, f.b.z, f.b.vy, this.sim.halfWidth)
              const r = this.tickFlight(f, tpl, null)
              if (r === -1) dead = true
              else if (r >= 0) done[k] = r
              else flying++
            }
            if (dead) continue
            const child: BeamNode = { fs, done, parent: ni, want, flags, pad: n.pad, score: 0, wp: reach(fs[0].b, n.wp) }
            if (flying === 0) {
              const p = this.beamProgram(nodes, child, pads, tpl, done[0])
              const ticks = this.robust(p)
              if (ticks >= 0) found.push({ ...p, ticks })
              continue
            }
            let score = -Infinity
            for (let k = 0; k < fs.length; k++) {
              if (done[k] !== FLYING) continue
              score = Math.max(score, child.wp === way.length - 1
                ? this.beamScore(fs[k], t, landY, catchTarget, 0, field)
                : this.beamScore(fs[k], [way[child.wp][0], way[child.wp][1], way[child.wp][2]], way[child.wp][1], false, tail[child.wp]))
            }
            if (score === Infinity) continue
            child.score = score
            // Keep the best child per full merge tuple; hash collisions must not merge distinct classes.
            const at = children.length * BEAM_KEY_LEN
            if (keys.length < at + BEAM_KEY_LEN) { const grown = new Int32Array(Math.max(keys.length * 2, at + BEAM_KEY_LEN)); grown.set(keys); keys = grown }
            beamKeyInto(fs[0], keys, at)
            const hk = beamHash(keys, at)
            const bucket = seen.get(hk)
            let prev = -1
            if (bucket !== undefined) {
              for (const ci of bucket) {
                let same = true
                for (let q = 0; q < BEAM_KEY_LEN && same; q++) same = keys[ci * BEAM_KEY_LEN + q] === keys[at + q]
                if (same) { prev = ci; break }
              }
            }
            if (prev >= 0) {
              if (children[prev].score > score) children[prev] = child
              continue
            }
            if (bucket !== undefined) bucket.push(children.length)
            else seen.set(hk, [children.length])
            children.push(child)
          }
        }
      }
      children.sort((p, q) => p.score - q.score)
      if (BEAM_DEBUG) {
        const air = children.filter(ch => ch.fs[0].fired).length
        const top = children.slice(0, 3).map(ch => `${ch.score.toFixed(1)}${ch.fs[0].fired ? 'a' : 'g'}@${ch.fs[0].b.x.toFixed(2)},${ch.fs[0].b.y.toFixed(2)},${ch.fs[0].b.z.toFixed(2)}`).join(' ')
        console.warn(`[beam] depth ${depth}: ${children.length} children (${air} airborne), ${found.length} robust; top ${top}`)
      }
      layer = []
      let ground = 0
      const taken = new Uint8Array(children.length)
      for (let i = 0; i < children.length && layer.length < BEAM_WIDTH; i++) {
        if (!children[i].fs[0].fired) {
          if (ground >= BEAM_WIDTH_GROUND) continue
          ground++
        }
        taken[i] = 1
        nodes.push(children[i])
        layer.push(nodes.length - 1)
      }
      // Reserve ground states even when airborne states rank better, so later take-offs remain possible.
      for (let i = 0; i < children.length && ground < BEAM_WIDTH_GROUND; i++) {
        if (taken[i] === 1 || children[i].fs[0].fired) continue
        ground++
        nodes.push(children[i])
        layer.push(nodes.length - 1)
      }
    }
    found.sort((p, q) => p.ticks - q.ticks)
    return found.length > 0 ? found[0] : null
  }

  /** Optimistic arrival-time rank using the landing disc in air and sprint-jump speed for remaining legs.
   * Falling short can be progress in a momentum chain; detour costs account for obstacles the disc ignores.
   * `t` is the next corridor waypoint and `after` is the remaining corridor length. */
  private beamScore (f: Flight, t: [number, number, number], landY: number, catchTarget: boolean, after: number, field: DetourField | null = null): number {
    const b = f.b
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const detour = field !== null ? field.at(b.x, b.z) : 0
    const toGo = Math.hypot(tcx - b.x, tcz - b.z) + detour + after
    // below the landing, a body climbs back about a block a jump
    const climb = Math.max(0, landY - b.y) * BEAM_CLIMB
    if (catchTarget) return f.t + toGo / BEAM_SPEED + climb
    if (!f.fired) return f.t + toGo / BEAM_SPEED + climb + this.spinUp(b, tcx, tcz)
    if (detour > DETOUR_EPS) {
      if (b.vy < 0 && b.y < landY - 0.5 && !this.slimeUnder(b)) return Infinity
      return f.t + toGo / BEAM_SPEED + climb
    }
    const disc = this.landingDisc(b, true, landY)
    if (disc === null) {
      // A falling body below the landing can only recover if slime below it provides a rebound.
      if (b.vy < 0 && b.y < landY - 0.5 && !this.slimeUnder(b)) return Infinity
      return f.t + toGo / BEAM_SPEED + climb
    }
    const s = disc[0]
    const cx = disc[1]
    const cz = disc[2]
    const r = disc[3]
    // Support uses the previous disc because Y moves before horizontal motion.
    const cx1 = disc[4]
    const cz1 = disc[5]
    const r1 = disc[6]
    const hw = this.sim.halfWidth
    const nx = Math.max(t[0] - hw, Math.min(cx1, t[0] + 1 + hw))
    const nz = Math.max(t[2] - hw, Math.min(cz1, t[2] + 1 + hw))
    const gap = Math.hypot(nx - cx1, nz - cz1) - r1
    if (gap > BEAM_SLACK) {
      // A short flight can become another leg only if something supports its nearest reachable touchdown.
      const d = Math.hypot(tcx - cx1, tcz - cz1)
      const px = d > 1e-9 ? cx1 + ((tcx - cx1) / d) * r1 : cx1
      const pz = d > 1e-9 ? cz1 + ((tcz - cz1) / d) * r1 : cz1
      if (!this.footing(px, landY, pz)) return Infinity
      return f.t + s + (gap + after) / BEAM_SPEED
    }
    return f.t + s + after / BEAM_SPEED + BEAM_MISS_WEIGHT * Math.max(0, Math.hypot(tcx - cx, tcz - cz) - r)
  }

  /** Build obstacle detour costs over a grid between the pads and landing, charging work to ticksFlown.
   * A point is free if the body clears boxes at any sampled height, including jump clearance.
   * Returns null when no obstacle or meaningful detour exists. */
  private detourField (t: [number, number, number], landY: number, pads: readonly Pad[]): DetourField | null {
    if (!this.beamDetour) return null
    const w = this.world
    const hw = this.sim.halfWidth
    const height = this.sim.height
    let lo = landY
    let hi = landY
    let ax = t[0]
    let bx = t[0] + 1
    let az = t[2]
    let bz = t[2] + 1
    for (const p of pads) {
      const b = p.f.b
      lo = Math.min(lo, b.y)
      hi = Math.max(hi, b.y)
      ax = Math.min(ax, b.x)
      bx = Math.max(bx, b.x)
      az = Math.min(az, b.z)
      bz = Math.max(bz, b.z)
    }
    hi += DETOUR_CLEAR
    const x0 = Math.max(Math.floor(ax) - DETOUR_MARGIN, t[0] - DETOUR_HALF)
    const x1 = Math.min(Math.ceil(bx) + DETOUR_MARGIN, t[0] + 1 + DETOUR_HALF)
    const z0 = Math.max(Math.floor(az) - DETOUR_MARGIN, t[2] - DETOUR_HALF)
    const z1 = Math.min(Math.ceil(bz) + DETOUR_MARGIN, t[2] + 1 + DETOUR_HALF)
    const nx = Math.round((x1 - x0) / DETOUR_STEP) + 1
    const nz = Math.round((z1 - z0) / DETOUR_STEP) + 1
    const n = nx * nz
    this.ticksFlown += Math.ceil(n / 4)
    const blocked = new Uint16Array(n)
    const mark = new Uint16Array(n)
    let heights = 0
    let any = false
    // Include exact stand heights so a landing under a low ceiling can be free only at its floor.
    const ys: number[] = [hi, landY]
    for (let y = lo; y < hi; y += DETOUR_DY) ys.push(y)
    for (const p of pads) ys.push(p.f.b.y)
    ys.sort((a, b) => a - b)
    for (let s = 0; s < ys.length; s++) {
      if (s > 0 && ys[s] - ys[s - 1] < 1e-6) continue
      const fy = ys[s]
      heights++
      for (let cy = Math.floor(fy) - 1; cy <= Math.floor(fy + height); cy++) {
        for (let cz = z0 - 1; cz <= z1; cz++) {
          for (let cx = x0 - 1; cx <= x1; cx++) {
            const st = this.sim.stateAt(cx, cy, cz)
            if (st < 0 || w.shapeStart[st] < 0) continue
            for (let q = 0; q < w.shapeCount[st]; q++) {
              const o = (w.shapeStart[st] + q) * 6
              if (cy + w.boxes[o + 4] <= fy + 1e-7 || cy + w.boxes[o + 1] >= fy + height - 1e-7) continue
              const i0 = Math.max(0, Math.floor((cx + w.boxes[o] - hw - x0) / DETOUR_STEP) + 1)
              const i1 = Math.min(nx - 1, Math.ceil((cx + w.boxes[o + 3] + hw - x0) / DETOUR_STEP) - 1)
              const j0 = Math.max(0, Math.floor((cz + w.boxes[o + 2] - hw - z0) / DETOUR_STEP) + 1)
              const j1 = Math.min(nz - 1, Math.ceil((cz + w.boxes[o + 5] + hw - z0) / DETOUR_STEP) - 1)
              for (let j = j0; j <= j1; j++) {
                for (let i = i0; i <= i1; i++) {
                  const k = j * nx + i
                  if (mark[k] === heights) continue
                  mark[k] = heights
                  blocked[k]++
                  any = true
                }
              }
            }
          }
        }
      }
    }
    if (!any) return null
    const free = new Uint8Array(n)
    let walls = false
    for (let k = 0; k < n; k++) {
      free[k] = blocked[k] < heights ? 1 : 0
      if (free[k] === 0) walls = true
    }
    if (!walls) return null
    const cx = t[0] + 0.5
    const cz = t[2] + 0.5
    const reach = hw - LAND_OVERLAP
    const g = new Float64Array(n).fill(Infinity)
    let hk = new Float64Array(1024)
    let hv = new Int32Array(1024)
    let hn = 0
    const push = (key: number, v: number): void => {
      if (hn === hk.length) {
        const k2 = new Float64Array(hn * 2); k2.set(hk); hk = k2
        const v2 = new Int32Array(hn * 2); v2.set(hv); hv = v2
      }
      let i = hn++
      while (i > 0) {
        const p = (i - 1) >> 1
        if (hk[p] <= key) break
        hk[i] = hk[p]; hv[i] = hv[p]; i = p
      }
      hk[i] = key; hv[i] = v
    }
    const pop = (): number => {
      const top = hv[0]
      const key = hk[--hn]
      const v = hv[hn]
      let i = 0
      for (;;) {
        let c = 2 * i + 1
        if (c >= hn) break
        if (c + 1 < hn && hk[c + 1] < hk[c]) c++
        if (hk[c] >= key) break
        hk[i] = hk[c]; hv[i] = hv[c]; i = c
      }
      hk[i] = key; hv[i] = v
      return top
    }
    const ci0 = Math.max(0, Math.ceil((t[0] - reach - x0) / DETOUR_STEP))
    const ci1 = Math.min(nx - 1, Math.floor((t[0] + 1 + reach - x0) / DETOUR_STEP))
    const cj0 = Math.max(0, Math.ceil((t[2] - reach - z0) / DETOUR_STEP))
    const cj1 = Math.min(nz - 1, Math.floor((t[2] + 1 + reach - z0) / DETOUR_STEP))
    for (let j = cj0; j <= cj1; j++) {
      for (let i = ci0; i <= ci1; i++) {
        const k = j * nx + i
        if (free[k] === 0) continue
        g[k] = octile(x0 + i * DETOUR_STEP - cx, z0 + j * DETOUR_STEP - cz)
        push(g[k], k)
      }
    }
    // No free landing point leaves the field without a destination.
    if (hn === 0) return null
    const diag = DETOUR_STEP * Math.SQRT2
    while (hn > 0) {
      const key = hk[0]
      const k = pop()
      if (key > g[k]) continue
      const i = k % nx
      const j = (k - i) / nx
      for (let dj = -1; dj <= 1; dj++) {
        for (let di = -1; di <= 1; di++) {
          if (di === 0 && dj === 0) continue
          const ii = i + di
          const jj = j + dj
          if (ii < 0 || jj < 0 || ii >= nx || jj >= nz) continue
          const kk = jj * nx + ii
          if (free[kk] === 0) continue
          // Do not cut a diagonal corner between blocked points.
          if (di !== 0 && dj !== 0 && (free[j * nx + ii] === 0 || free[jj * nx + i] === 0)) continue
          const v = key + (di !== 0 && dj !== 0 ? diag : DETOUR_STEP)
          if (v < g[kk]) { g[kk] = v; push(v, kk) }
        }
      }
    }
    const excess = new Float32Array(n)
    let most = 0
    for (let j = 0; j < nz; j++) {
      for (let i = 0; i < nx; i++) {
        const k = j * nx + i
        const e = g[k] === Infinity ? DETOUR_CAP : Math.min(DETOUR_CAP, Math.max(0, g[k] - octile(x0 + i * DETOUR_STEP - cx, z0 + j * DETOUR_STEP - cz)))
        excess[k] = e
        if (free[k] === 1 && e > most) most = e
      }
    }
    if (BEAM_DEBUG) console.warn(`[beam] detour field ${nx}x${nz} over y ${lo.toFixed(2)}..${hi.toFixed(2)}: most ${most.toFixed(2)}`)
    if (most <= DETOUR_EPS) return null
    return new DetourField(x0, z0, nx, nz, free, excess)
  }

  /** Ticks needed to reach sprint speed toward (px, pz), using the current block's acceleration. */
  private spinUp (b: SimBody, px: number, pz: number): number {
    const dx = px - b.x
    const dz = pz - b.z
    const d = Math.hypot(dx, dz)
    const along = d > 1e-6 ? (b.vx * dx + b.vz * dz) / d : 0
    if (along >= BEAM_SPRINT_SPEED) return 0
    const under = this.sim.stateAt(Math.floor(b.x), Math.floor(b.y - 1), Math.floor(b.z))
    const slip = (under >= 0 && this.world.slip[under] > 0 ? this.world.slip[under] : SIM.defaultSlipperiness) * SIM.airborneInertia
    const accel = this.sim.speedBase * 1.3 * 0.98 * (0.1627714 / (slip * slip * slip))
    return (BEAM_SPRINT_SPEED - along) / Math.max(accel * (1 - slip) * 3, 1e-3)
  }

  /** Whether a collision box within BEAM_FOOTING below `y` can support the body's footprint at (x, z). */
  private footing (x: number, y: number, z: number): boolean {
    const hw = this.sim.halfWidth
    for (let yy = Math.floor(y - 1e-6); yy >= Math.floor(y) - BEAM_FOOTING; yy--) {
      for (let cx = Math.floor(x - hw); cx <= Math.floor(x + hw); cx++) {
        for (let cz = Math.floor(z - hw); cz <= Math.floor(z + hw); cz++) {
          if (this.shaped(cx, yy, cz)) return true
        }
      }
    }
    return false
  }

  /** Is the first collision box down the body's column (within BOUNCE_DROP + 6) a slime block's? */
  private slimeUnder (b: SimBody): boolean {
    const x = Math.floor(b.x)
    const z = Math.floor(b.z)
    for (let y = Math.floor(b.y); y >= Math.floor(b.y) - BOUNCE_DROP - 6; y--) {
      if (!this.shaped(x, y, z)) continue
      return (this.kindAt(x, y, z) & SIM_SLIME) !== 0
    }
    return false
  }

  /** A landing the beam found, as a replayable program: the controls from its pad to the landing, tick for tick. */
  private beamProgram (nodes: BeamNode[], leaf: BeamNode, pads: readonly Pad[], tpl: HopProgram, ticks: number): HopProgram {
    const rev: number[] = []
    for (let n: BeamNode = leaf; n.parent >= 0; n = nodes[n.parent]) rev.push(n.flags, n.want)
    const seq = rev.reverse()
    const pad = pads[leaf.pad]
    return {
      ...tpl,
      sx: pad.f.b.x,
      sy: pad.f.b.y,
      sz: pad.f.b.z,
      start: pad.live,
      yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw,
      seq,
      ticks
    }
  }

  /** Does the unobstructed rest of this flight, on the program's last heading, come down within BALLISTIC_MISS of the landing column? */
  private comesDownNear (b: SimBody, p: HopProgram): boolean {
    const land = this.ballisticLanding(b, p.air2, p.sprint, p.ty - 0.5)
    if (land === null) return false
    const mx = Math.max(p.tx - land[0], 0, land[0] - (p.tx + 1))
    const mz = Math.max(p.tz - land[1], 0, land[1] - (p.tz + 1))
    return mx * mx + mz * mz <= BALLISTIC_MISS * BALLISTIC_MISS
  }

  /** The body's box over column (tx, tz) by LAND_OVERLAP at least (a catch is in its column: the caller's test). */
  private overCol (b: SimBody, tx: number, tz: number): boolean {
    const e = 0.5 + this.sim.halfWidth - LAND_OVERLAP
    return Math.abs(b.x - tx - 0.5) < e && Math.abs(b.z - tz - 0.5) < e
  }

  /** Whether a non-deep, non-sequence region landing needs to settle to its required level. */
  private offLevel (b: SimBody, p: HopProgram): boolean {
    return REGION_SETTLE && p.land !== undefined && p.deep !== true && p.seq === undefined && Math.abs(b.y - p.land.y) >= 0.5
  }

  /** Tick when the last stays() settles at the required region level, or -1. */
  private stayAt = -1
  private stays (b: SimBody, p: HopProgram): boolean {
    if (this.touch !== null) copyBody(this.touch, b)
    this.stayAt = -1
    const off = this.offLevel(b, p)
    const c = this.ctl
    c.forward = false; c.sprint = false; c.jump = false; c.sneak = true
    for (let k = 0; k < STAY_TICKS; k++) {
      if (this.anchorDeadline !== null && this.expired(this.anchorDeadline)) { c.sneak = false; return false }
      this.kernelStep(b, c, this.anchorDeadline !== null)
      if (this.stayAt < 0 && p.land !== undefined && b.onGround && Math.abs(b.y - p.land.y) < 0.5) {
        this.stayAt = k
        // The next leg must start where the arrival tick places the body.
        if (off && this.touch !== null) copyBody(this.touch, b)
      }
    }
    c.sneak = false
    if (p.catchTarget) return Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz && b.y >= p.ty - 1.001 && b.y < p.ty + 1
    // Liquid contact freezes the kernel body, so a slide into liquid can falsely appear stable.
    if (b.inLiquid) return false
    if (!this.overCol(b, p.tx, p.tz)) return false
    return b.onGround && this.nodeY(p.tx, b.y, p.tz) === p.ty && this.carriedAt(b, p.tx, p.ty, p.tz) &&
      (p.land === undefined || this.inRegion(b, p))
  }

  /** Whether the landing column supports the body, directly or through a neighbour at most a step above.
   * Neighbour support also requires footing in the landing column within a step below. */
  private carriedAt (b: SimBody, tx: number, ty: number, tz: number): boolean {
    if (this.sim.supportedIn(b, tx, tz)) return true
    if (!NEIGHBOUR_CARRY || Math.floor(b.x) !== tx || Math.floor(b.z) !== tz || !this.sim.carried(b)) return false
    let floor = Infinity
    for (const st of this.stands(tx, ty, tz)) if (!st.catch) floor = Math.min(floor, st.y)
    return b.y >= floor && b.y <= floor + SIM.stepHeight && this.footingUnder(b, tx, tz)
  }

  /** Whether landing-column footing lies under the body within a step below its feet.
   * A body resting on the take-off lip across a gap has no such footing. */
  private footingUnder (b: SimBody, tx: number, tz: number): boolean {
    const w = this.world
    const hw = this.sim.halfWidth
    for (let yy = Math.floor(b.y - SIM.stepHeight) - 1; yy <= Math.floor(b.y); yy++) {
      const s = this.sim.stateAt(tx, yy, tz)
      if (s < 0 || w.shapeStart[s] < 0) continue
      for (let k = 0; k < w.shapeCount[s]; k++) {
        const o = (w.shapeStart[s] + k) * 6
        const top = yy + w.boxes[o + 4]
        if (top > b.y + 1e-6 || top < b.y - SIM.stepHeight - 1e-6) continue
        if (b.x - hw < tx + w.boxes[o + 3] && b.x + hw > tx + w.boxes[o] && b.z - hw < tz + w.boxes[o + 5] && b.z + hw > tz + w.boxes[o + 2]) return true
      }
    }
    return false
  }

  /** Verify the program from its start and all required nudges.
   * Live and beam programs use LIVE_NUDGE because replay starts from the exact body state. */
  private robust (p: HopProgram): number {
    const t0 = this.fly(p)
    if (t0 < 0) return -1
    // Precision programs require the exact line-up.
    const n = p.start !== undefined || p.seq !== undefined || p.precise === true ? LIVE_NUDGE : this.sideBridge ? SIDE_NUDGE : ROBUST_NUDGE
    // Chain twins only need the first touchdown; small nudges diverge across multiple rebounds.
    this.touchIsLanding = p.legs === true
    // Bounce-chain twins likewise stop at the first rebound.
    this.reboundIsLanding = p.bounces !== undefined && p.bounces.length > 0
    try {
      for (const [nx, nz] of this.requiredNudges(p, n)) {
        if (this.fly(p, nx, nz) < 0) return -1
      }
    } finally {
      this.touchIsLanding = false
      this.reboundIsLanding = false
    }
    // Check the deep bounce's committed window here so rejection can leave slower candidates available.
    if (p.deep === true && !this.commitGate(p)) return -1
    return t0
  }

  /** Whether aborting from `f` by steering without sprint, jump or sneak lands inside the slime pool.
   * Targets the nearest basin point and requires at least a block and a half of edge clearance. */
  private abortsOnSlime (f: Flight, tops: ReadonlyArray<[number, number, number]>): boolean {
    const b = f.b
    const c = this.ctl
    for (let t = 0; t < 400; t++) {
      let best = -1
      let bd = Infinity
      for (let i = 0; i < tops.length; i++) {
        const d = (tops[i][0] + 0.5 - b.x) ** 2 + (tops[i][2] + 0.5 - b.z) ** 2
        if (d < bd) { bd = d; best = i }
      }
      if (best < 0) return false
      const dx = tops[best][0] + 0.5 - b.x
      const dz = tops[best][2] + 0.5 - b.z
      const dd = Math.hypot(dx, dz)
      c.hx = dd > 1e-9 ? dx / dd : 0; c.hz = dd > 1e-9 ? dz / dd : 0
      c.forward = dd > 0.2; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = false
      this.sim.step(b, c)
      this.ticksFlown++
      if (b.inLiquid) return false
      if (b.onGround) return (this.kindAt(Math.floor(b.x), Math.floor(b.y - 0.2), Math.floor(b.z)) & SIM_SLIME) !== 0 && this.inPool(b.x, Math.floor(b.y - 0.2), b.z)
    }
    return false
  }

  /** Is (x, z) at least SLIME_EDGE from any non-slime column at level y (a slime pool's inside)? */
  private inPool (x: number, y: number, z: number): boolean {
    for (let k = 0; k < 16; k++) {
      const a = (k / 16) * 2 * Math.PI
      for (const r of [0.5, 1, SLIME_EDGE]) {
        if ((this.kindAt(Math.floor(x + Math.cos(a) * r), y, Math.floor(z + Math.sin(a) * r)) & SIM_SLIME) === 0) return false
      }
    }
    return true
  }

  /** Verify the deep bounce's non-abortable ticks against velocity nudges and position nudges on entry.
   * Earlier ticks are re-verified live; the rebound must land inside the slime pool.
   * Scan rather than bisect because abortable intervals can have holes. */
  private commitGate (p: HopProgram): boolean {
    const f = this.origin(p, 0, 0)
    if (f === null) return false
    // The nearest slime top may be on the rim, so aborts target interior points.
    const tops = this.deepSlimeTops(Math.floor(p.sx), Math.floor(p.sy + 1e-9), Math.floor(p.sz))
      .filter(t => this.inPool(t[0] + 0.5, t[1], t[2] + 0.5))
    if (tops.length === 0) return false
    const states: Flight[] = []
    let reboundAt = -1
    for (let i = 0; i < 2000; i++) {
      states.push(flightInto(newFlight(), f))
      const before = f.rebounds
      // Advance one tick at a time; the zero air mark is met on the firing tick.
      const r = this.advance(f, p, f.t + 1, f.fired ? f.air + 1 : 0, null)
      if (f.rebounds > before && reboundAt < 0) {
        reboundAt = states.length
        if (!this.inPool(f.b.x, Math.floor(f.b.y - 0.2), f.b.z)) return false
      }
      if (r !== FLYING) {
        if (r < 0) return false
        break
      }
    }
    if (reboundAt < 0) return false
    // Check velocity disturbances on every non-abortable tick and position disturbances only on entry.
    // Abortable intervals between them are re-verified live.
    let committed = false
    for (let i = 0; i < states.length; i++) {
      const st = states[i]
      if ((!st.fired && st.b.onGround) || this.abortsOnSlime(flightInto(newFlight(), st), tops)) { committed = false; continue }
      const entering = !committed
      committed = true
      for (const [ax, az, vx, vy, vz] of COMMIT_KICKS) {
        if ((ax !== 0 || az !== 0) && !entering) continue
        const g = flightInto(newFlight(), st)
        g.b.x += ax; g.b.z += az; g.b.vx += vx; g.b.vy += vy; g.b.vz += vz
        if (this.sim.collides(g.b)) continue
        if (this.advance(g, p, Infinity, Infinity, null) < 0) return false
      }
    }
    p.basin = tops
    return true
  }

  /** Recheck the remaining deep bounce and its basin abort from the current live state.
   * Both simulations use copies of the body. */
  deepVerdict (p: HopProgram, body: SimBody, t: number, fired: boolean, air: number, rebounds: number, yaw: number): { arrives: boolean, abortable: boolean } {
    const f = newFlight()
    copyBody(f.b, body)
    f.t = t; f.fired = fired; f.air = air; f.rebounds = rebounds; f.yaw = yaw; f.top = body.y
    const g = flightInto(newFlight(), f)
    const arrives = this.advance(f, p, Infinity, Infinity, null) >= 0
    const abortable = p.basin !== undefined && p.basin.length > 0 && this.abortsOnSlime(g, p.basin)
    return { arrives, abortable }
  }

  /** robust(): a chain's twin counts the end of its first real leg as its landing. */
  private touchIsLanding = false
  /** robust(): a bounce chain's twin counts its first rebound (off the slime its first heading aims at) as its landing. */
  private reboundIsLanding = false

  /** The ticks program `p` takes if it lands robustly (robust(): nudged starts, a chain to its first leg), else -1. */
  verify (p: HopProgram): number {
    return this.robust(p)
  }

  /** Slime block coordinates within BOUNCE_REACH whose first collision below take-off is within BOUNCE_DROP. */
  slimeTops (x: number, y: number, z: number): Array<[number, number, number]> {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.slimeCache.get(k)
    if (hit !== undefined) return hit
    const out: Array<[number, number, number]> = []
    for (let dx = -BOUNCE_REACH; dx <= BOUNCE_REACH; dx++) {
      for (let dz = -BOUNCE_REACH; dz <= BOUNCE_REACH; dz++) {
        if (dx * dx + dz * dz > BOUNCE_REACH * BOUNCE_REACH) continue
        for (let yy = y - 1; yy >= y - BOUNCE_DROP; yy--) {
          if (!this.shaped(x + dx, yy, z + dz)) continue
          if ((this.kindAt(x + dx, yy, z + dz) & SIM_SLIME) !== 0) out.push([x + dx, yy, z + dz])
          break
        }
      }
    }
    this.slimeCache.set(k, out)
    return out
  }

  /** Slime tops within fall reach, more than BOUNCE_DROP and at most BOUNCE_DROP_LONG below take-off.
   * Kept separate from shallow bounce and chain candidates. */
  deepSlimeTops (x: number, y: number, z: number): Array<[number, number, number]> {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.deepSlimeCache.get(k)
    if (hit !== undefined) return hit
    const out: Array<[number, number, number]> = []
    const R = Math.ceil(fallReach(BOUNCE_DROP_LONG))
    // A live grid is only a cache window; unloaded cells below it appear empty.
    const floor = y - BOUNCE_DROP_LONG
    for (let dx = -R; dx <= R; dx++) {
      for (let dz = -R; dz <= R; dz++) {
        const d2 = dx * dx + dz * dz
        if (d2 > R * R) continue
        for (let yy = y - 1; yy >= floor; yy--) {
          if (!this.shaped(x + dx, yy, z + dz)) continue
          if (y - yy > BOUNCE_DROP && (this.kindAt(x + dx, yy, z + dz) & SIM_SLIME) !== 0 && d2 <= fallReach(y - yy) ** 2) out.push([x + dx, yy, z + dz])
          break
        }
      }
    }
    this.deepSlimeCache.set(k, out)
    return out
  }

  /** One family's program classes from start `st`, simplest (gentlest) first. */
  private * classes (family: Family, st: { x: number, y: number, z: number }, aims: ReadonlyArray<[number, number]>, tcx: number, tcz: number, walk: boolean, grid: boolean, ty: number, runAxes: ReadonlyArray<[number, number]> = AXES8, hint?: HopProgram): Generator<ProgramClass> {
    if (family === 'straight') {
      for (const a of aims) {
        yield { run: point(a[0], a[1]), sprint: true, jumpAts: STRAIGHT_JUMP_AT, turnAirs: NO_TURN, air2s: [point(a[0], a[1])], aim: false }
      }
      if (walk) for (const a of aims) yield { run: point(a[0], a[1]), sprint: false, jumpAts: STRAIGHT_JUMP_AT, turnAirs: NO_TURN, air2s: [point(a[0], a[1])], aim: false }
    } else if (family === 'leave') {
      // Turn a run across a strip's width toward the landing at the lip or shortly after take-off.
      const air2s = aims.slice(0, 3).map(a => point(a[0], a[1]))
      for (const sprint of [true, false]) {
        for (const a of aims) yield { run: point(a[0], a[1]), sprint, jumpAts: LEAVE_AT, turnAirs: LEAVE_TURN, air2s, aim: true, leave: true }
        for (const [ux, uz] of AXES8) yield { run: dir(ux, uz), sprint, jumpAts: LEAVE_AT, turnAirs: LEAVE_TURN, air2s, aim: true, leave: true }
      }
    } else if (family === 'kick') {
      if (hint !== undefined) {
        // A parallel run preserves sideways error; aim past the planned take-off to return to its lane.
        const f = this.origin(hint, 0, 0)
        if (f === null || this.advance(f, hint, hint.jumpAt, 0, null) !== FLYING ||
            f.fired || !f.b.onGround || f.t !== hint.jumpAt) return
        const u = hint.run
        const ax = Math.atan2(u.z, u.x)
        const out = Math.atan2(u.x * hint.air1.z - u.z * hint.air1.x, u.x * hint.air1.x + u.z * hint.air1.z)
        const side = out >= 0 ? 1 : -1
        const neighbours = KICK_OUT.map(a => side * a).filter(a => Math.abs(a - out) > 1e-6)
          .sort((a, b) => Math.abs(a - out) - Math.abs(b - out)).slice(0, 2)
        const kicks = [hint.air1, ...neighbours.map(a => dir(Math.cos(ax + a), Math.sin(ax + a)))]
        const runs = [point(f.b.x + 1.5 * u.x, f.b.z + 1.5 * u.z), point(f.b.x + 2 * u.x, f.b.z + 2 * u.z), u]
        const bearing = Math.atan2(tcz - st.z, tcx - st.x)
        const air2s = [hint.air2, ...YAW_TURN.map(a => dir(Math.cos(bearing + a), Math.sin(bearing + a)))]
        for (const kick of kicks) {
          for (const run of runs) {
            yield { run, kick, sprint: hint.sprint, jumpAts: KICK_JUMP_AT, turnAirs: KICK_TURN_TICKS, air2s, aim: true }
          }
        }
        return
      }
      // Face away from the strip's axis at take-off to redirect the sprint boost toward the landing.
      const bearing = Math.atan2(tcz - st.z, tcx - st.x)
      const bx = Math.cos(bearing)
      const bz = Math.sin(bearing)
      for (const sprint of walk ? [true, false] : [true]) {
        for (const [ux, uz] of KICK_AXES) {
          if (ux * bx + uz * bz < KICK_AXIS_COS) continue
          const side = ux * bz - uz * bx >= 0 ? 1 : -1
          const ax = Math.atan2(uz, ux)
          const air2s = grid ? YAW_TURN.map(o => dir(Math.cos(bearing + o), Math.sin(bearing + o))) : []
          for (const lean of KICK_LEAN) {
            const run = dir(Math.cos(ax + side * lean), Math.sin(ax + side * lean))
            for (const out of KICK_OUT) {
              yield { run, kick: dir(Math.cos(ax + side * out), Math.sin(ax + side * out)), sprint, jumpAts: KICK_JUMP_AT, turnAirs: KICK_TURN_TICKS, air2s, aim: true }
            }
          }
        }
      }
    } else if (family === 'pivot') {
      const bearing = Math.atan2(tcz - st.z, tcx - st.x)
      const air2s = PIVOT_TURN.map(o => dir(Math.cos(bearing + o), Math.sin(bearing + o)))
      for (const [ux, uz] of runAxes) {
        if (ux * Math.cos(bearing) + uz * Math.sin(bearing) <= 0) continue
        const ra = Math.atan2(uz, ux)
        for (const pj of PIVOT_JUMP) {
          const ja = bearing + pj
          if (Math.abs(Math.atan2(Math.sin(ja - ra), Math.cos(ja - ra))) < PIVOT_MIN) continue
          yield { run: dir(ux, uz), kick: dir(Math.cos(ja), Math.sin(ja)), sprint: true, jumpAts: PIVOT_JUMP_AT, turnAirs: PIVOT_TURN_TICKS, air2s, aim: true }
        }
      }
    } else if (family === 'run-turn') {
      const air2s = aims.slice(0, 3).map(a => point(a[0], a[1]))
      for (const [ux, uz] of AXES8) yield { run: dir(ux, uz), sprint: true, jumpAts: RUN_TURN_JUMP_AT, turnAirs: NO_TURN, air2s, aim: true }
    } else if (family === 'air-turn') {
      const dx = tcx - st.x
      const dz = tcz - st.z
      const d = Math.hypot(dx, dz) || 1
      const px = -dz / d
      const pz = dx / d
      const air2s = [point(tcx, tcz)]
      for (const f of [0.35, 0.5, 0.65]) {
        for (const off of [-1.5, -1, -0.6, 0.6, 1, 1.5]) {
          yield { run: point(st.x + dx * f + px * off, st.z + dz * f + pz * off), sprint: true, jumpAts: AIR_TURN_JUMP_AT, turnAirs: AIR_TURN_TICKS, air2s, aim: true }
        }
      }
    } else if (family === 'bounce') {
      const all = this.slimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z))
        .map(([x, y, z]) => [x, z, Math.hypot(x + 0.5 - st.x, z + 0.5 - st.z) + Math.hypot(tcx - x - 0.5, tcz - z - 0.5), y])
      // Keep the fixed bounce ring and also allow tops whose calculated rebound reaches farther.
      let tops = all.filter(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <=
        Math.max(BOUNCE_REACH, reboundReach(st.y - (t[3] + 1), ty - (t[3] + 1))))
      const chain = tops.length === 0
      // Try direct widened-reach candidates before chains from the fixed ring.
      const chainsToo = !chain && !all.some(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <= BOUNCE_REACH)
      tops = (chain ? all : tops).sort((p, q) => p[2] - q[2]).slice(0, chain ? BOUNCE_CHAIN_FIRST : BOUNCE_SLIMES)
      const air2s = [point(tcx, tcz)]
      // Aim short as well as centrally: carried momentum can overshoot the first touchdown and redirect the rebound.
      for (const sprint of this.deepOnly ? [] : walk ? [true, false] : [true]) {
        for (const [x, z] of tops) {
          const cx = x + 0.5
          const cz = z + 0.5
          const d = Math.hypot(cx - st.x, cz - st.z) || 1
          for (const short of BOUNCE_SHORT) {
            yield { run: point(cx - ((cx - st.x) / d) * short, cz - ((cz - st.z) / d) * short), sprint, jumpAts: BOUNCE_JUMP_AT, turnAirs: TO_REBOUND, air2s, aim: true, chain }
          }
        }
      }
      const deep = this.deepSlimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z))
        .map(([x, y, z]) => [x, z, Math.hypot(x + 0.5 - st.x, z + 0.5 - st.z) + Math.hypot(tcx - x - 0.5, tcz - z - 0.5), y])
        .filter(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <= Math.max(BOUNCE_REACH, deepReach(st.y - (t[3] + 1), ty - (t[3] + 1))))
        .sort((p, q) => p[2] - q[2]).slice(0, BOUNCE_SLIMES)
      if (chainsToo && !this.deepOnly) {
        const firsts = [...all].sort((p, q) => p[2] - q[2]).slice(0, BOUNCE_CHAIN_FIRST)
        for (const sprint of walk ? [true, false] : [true]) {
          for (const [x, z] of firsts) {
            const cx = x + 0.5
            const cz = z + 0.5
            const d = Math.hypot(cx - st.x, cz - st.z) || 1
            for (const short of BOUNCE_SHORT) {
              yield { run: point(cx - ((cx - st.x) / d) * short, cz - ((cz - st.z) / d) * short), sprint, jumpAts: BOUNCE_JUMP_AT, turnAirs: TO_REBOUND, air2s, aim: true, chain: true }
            }
          }
        }
      }
      for (const sprint of walk ? [true, false] : [true]) {
        for (const [x, z] of deep) {
          const cx = x + 0.5
          const cz = z + 0.5
          const d = Math.hypot(cx - st.x, cz - st.z) || 1
          for (const short of BOUNCE_SHORT) {
            yield { run: point(cx - ((cx - st.x) / d) * short, cz - ((cz - st.z) / d) * short), sprint, jumpAts: BOUNCE_JUMP_AT, turnAirs: TO_REBOUND, air2s, aim: true, deep: true }
          }
        }
      }
    } else {
      const bearing = Math.atan2(tcz - st.z, tcx - st.x)
      // The grid includes headings that need a collision to slide into the landing.
      const air2s = grid ? YAW_TURN.map(b => dir(Math.cos(bearing + b), Math.sin(bearing + b))) : []
      for (const sprint of walk ? [true, false] : [true]) {
        for (const a of YAW_RUN) {
          yield { run: dir(Math.cos(bearing + a), Math.sin(bearing + a)), sprint, jumpAts: YAW_JUMP_AT, turnAirs: YAW_TURN_TICKS, air2s, aim: true }
        }
      }
    }
  }

  /** Search one family's programs as a tree, sharing run and first-air ticks across later branches.
   * Returns the quickest robust program. */
  private search (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], family: Family, deadline: number, walk: boolean, grid: boolean, land?: { x: number, y: number, z: number }, hint?: HopProgram, pick?: (found: HopProgram[]) => HopProgram | null): HopProgram | null {
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const catchTarget = lands[0].catch
    const landY = lands[0].y
    // Use the lowest stand height for reach pruning; the planner node can sit above the actual support top.
    let reachY = lands.reduce((m, s) => Math.min(m, s.y), Infinity)
    // Allow a lower approach where neighbouring footing can step up onto the landing.
    // Climbable catches require the body to remain in their column.
    if (!catchTarget && this.stepUnder(t[0], t[2], reachY)) reachY -= SIM.stepHeight
    const aimsRaw: Array<[number, number]> = [[tcx, tcz], ...lands.map(p => [p.x, p.z] as [number, number])]
    const aims = aimsRaw.filter((p, i) => aimsRaw.findIndex(q => Math.abs(q[0] - p[0]) < 0.05 && Math.abs(q[1] - p[1]) < 0.05) === i)
    const [run, launch, air, branch] = this.flights
    const aimed: Heading[] = []
    const found: HopProgram[] = []
    const select = (): HopProgram | null => pick !== undefined ? pick(found) : this.quickestRobust(found, deadline)
    const slimeLand = this.slimeLanding(t, catchTarget)
    let runAxes: ReadonlyArray<[number, number]> = AXES8
    if (family === 'pivot') {
      if (catchTarget || Math.min(spanOf(lands, 0), spanOf(lands, 1)) >= PIVOT_NARROW) return null
      // A narrow take-off allows a run along its length only.
      const px = spanOf(pads.map(q => q.f.b), 0)
      const pz = spanOf(pads.map(q => q.f.b), 1)
      if (pz < PIVOT_NARROW && px >= PIVOT_NARROW) runAxes = [[1, 0], [-1, 0]]
      else if (px < PIVOT_NARROW && pz >= PIVOT_NARROW) runAxes = [[0, 1], [0, -1]]
    }
    let n = 0
    for (const pad of pads) {
      const st = pad.f.b
      const climb = this.climbing(st)
      // Reach pruning assumes a falling flight; slime rebounds and climbable recovery invalidate that bound.
      const falls = REACH_PRUNE && family !== 'bounce' && !climb && !slimeLand &&
        this.slimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z)).length === 0
      for (const cls of this.classes(family, st, aims, tcx, tcz, walk, grid, t[1], runAxes, hint)) {
        if ((++n & 3) === 0 && this.expired(deadline)) { this.noVerdict = true; return select() }
        const base: HopProgram = {
          sx: st.x, sy: st.y, sz: st.z, start: pad.live, yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw, run: cls.run, jumpAt: Infinity, air1: cls.kick ?? cls.run, turnAir: Infinity, air2: cls.kick ?? cls.run, kick: cls.kick !== undefined ? true : undefined, precise: family === 'pivot' ? true : undefined,
          sprint: cls.sprint, ticks: 0, tx: t[0], ty: t[1], tz: t[2], catchTarget, family, climb, slimeLand, land, deep: cls.deep === true ? true : undefined
        }
        flightInto(run, pad.f)
        for (const j of cls.jumpAts) {
          if (hint !== undefined && this.expired(deadline)) { this.noVerdict = true; return select() }
          if (this.advance(run, base, j, 0, null) !== FLYING) break
          // Leaving the edge before this jump tick also rules out every later jump tick.
          const walkedOff = run.fired
          const pj: HopProgram = cls.leave === true ? { ...base, jumpAt: j, jumpOnLeave: true } : { ...base, jumpAt: j }
          // No turn can recover a take-off already out of reach.
          if (this.advance(flightInto(launch, run), pj, Infinity, 0, null) === FLYING &&
              (!falls || this.canReach(launch.b, cls.sprint, t[0], reachY, t[2]))) {
            flightInto(air, launch)
            for (const k of cls.turnAirs) {
              const ra = this.advance(air, pj, Infinity, k, null)
              if (ra !== FLYING) {
                // A flight ending before the turn is identical for every later turn tick.
                if (ra >= 0) found.push({ ...pj, turnAir: k, ticks: ra })
                break
              }
              // A later turn cannot recover a flight already out of reach on its first heading.
              if (falls && !this.canReach(air.b, cls.sprint, t[0], reachY, t[2])) break
              aimed.length = 0
              if (cls.aim) {
                if (catchTarget) aimed.push(point(tcx, tcz))
                else this.aimHeadings(air.b, cls.sprint, t[0], t[2], landY, aimed)
              }
              if (family === 'pivot' && !catchTarget) this.pointAims(air.b, cls.sprint, landY, lands, aimed)
              // Try both rebound controls where the physics preserves rebound velocity when jump is held.
              const rjs = cls.deep === true && air.rebounds > 0 && isVanillaJump() ? REBOUND_JUMPS : NO_REBOUND_JUMP
              for (let h = 0; h < cls.air2s.length + aimed.length; h++) {
                for (const rj of rjs) {
                  const pk: HopProgram = { ...pj, turnAir: k, air2: h < cls.air2s.length ? cls.air2s[h] : aimed[h - cls.air2s.length], reboundJump: rj ? true : undefined }
                  const rb = this.advance(flightInto(branch, air), pk, Infinity, Infinity, null)
                  if (rb >= 0) found.push({ ...pk, ticks: rb })
                }
              }
              if (cls.chain === true && air.rebounds === 1) this.chainOn(air, { ...pj, turnAir: k }, [], t, landY, catchTarget, cls.sprint, found, deadline)
              // A rebound turns every program whose scheduled turn comes later.
              if (air.rebounds > 0) break
            }
          }
          if (walkedOff) break
        }
      }
    }
    return select()
  }

  /** Extend a bounce chain toward reachable slime tops, then the landing, within BOUNCE_CHAIN.
   * Rebounds lose little height to drag, allowing successive slime crossings. Appends landings to `found`. */
  private chainOn (at: Flight, pj: HopProgram, chain: readonly Heading[], t: [number, number, number], landY: number, catchTarget: boolean, sprint: boolean, found: HopProgram[], deadline: number): void {
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const last = chain.length + 1 >= BOUNCE_CHAIN
    const f = newFlight()
    const g = newFlight()
    const aimed: Heading[] = []
    for (const [sx, sz] of this.nextSlimes(at.b, tcx, tcz, last)) {
      if (this.expired(deadline)) { this.noVerdict = true; return }
      const bounces = [...chain, point(sx + 0.5, sz + 0.5)]
      const pv: HopProgram = { ...pj, bounces, air2: point(tcx, tcz) }
      const r = this.advance(flightInto(f, at), pv, Infinity, TO_NEXT_REBOUND, null)
      if (r !== FLYING) {
        // down on the landing on the way: this chain is that program
        if (r >= 0) found.push({ ...pv, ticks: r })
        continue
      }
      if (f.rebounds !== chain.length + 2) continue // came down somewhere without a rebound (an edge)
      if (catchTarget) aimed.length = 0
      else this.aimHeadings(f.b, sprint, t[0], t[2], landY, aimed)
      aimed.push(point(tcx, tcz))
      for (const h of aimed) {
        const pk: HopProgram = { ...pv, air2: h }
        const rb = this.advance(flightInto(g, f), pk, Infinity, Infinity, null)
        if (rb >= 0) found.push({ ...pk, ticks: rb })
      }
      if (!last) this.chainOn(f, pv, bounces, t, landY, catchTarget, sprint, found, deadline)
    }
  }

  /** Next slime tops below the rebound apex and within BOUNCE_REACH, excluding the current top.
   * The final bounce must also be within reach of the landing; returns [x, z] for the nearest candidates. */
  private nextSlimes (b: SimBody, tcx: number, tcz: number, last: boolean): Array<[number, number]> {
    let y = b.y
    let vy = b.vy
    while (vy > 0) { y += vy; vy = (vy - SIM.gravity) * SIM.airdrag }
    const cx = Math.floor(b.x)
    const cz = Math.floor(b.z)
    return this.slimeTops(cx, Math.floor(y), cz)
      .filter(([x, , z]) => (x !== cx || z !== cz) && (!last || Math.hypot(tcx - x - 0.5, tcz - z - 0.5) <= BOUNCE_REACH))
      .map(([x, , z]): [number, number, number] => [x, z, Math.hypot(x + 0.5 - b.x, z + 0.5 - b.z) + Math.hypot(tcx - x - 0.5, tcz - z - 0.5)])
      .sort((p, q) => p[2] - q[2])
      .slice(0, BOUNCE_CHAIN_NEXT)
      .map(([x, z]): [number, number] => [x, z])
  }

  /** Select the quickest robust landing, breaking ties by discovery order.
   * Reserve QR_GRACE_TICKS for verification so budget expiry does not discard unverified finds. */
  private quickestRobust (found: HopProgram[], deadline: number): HopProgram | null {
    const order = found.map((p, i) => i).sort((a, b) => found[a].ticks - found[b].ticks || a - b)
    let stop = Infinity
    let first: HopProgram | null = null
    for (const i of order) {
      if (stop === Infinity && this.expired(deadline)) { this.noVerdict = true; stop = this.ticksFlown + QR_GRACE_TICKS }
      if (this.ticksFlown >= stop) break
      // (collecting for the next-leg ranking: the robust ones within its slack of the quickest too)
      if (first !== null && (this.collect === null || found[i].ticks > first.ticks + this.collectSlack || this.collect.length >= this.collectCap)) break
      const ticks = this.robust(found[i])
      if (ticks < 0) continue
      const r = { ...found[i], ticks }
      if (this.collect !== null) this.collect.push(r)
      if (first === null) first = r
    }
    return first
  }

  /** quickestRobust(): when set, also receives the robust programs within collectSlack of each search's quickest (hopFrom's next-leg ranking), collectCap in all. */
  private collect: HopProgram[] | null = null
  private collectSlack = NEXT_SLACK
  private collectCap = NEXT_COLLECT
  /** stays(): when set, receives the body at the touchdown it judges (touchdownOf). */
  private touch: SimBody | null = null
  /** A touchdown over the node is the landing, the body going on at once (hopChain). */
  private goLanding = false

  /** Whether one flight lands with enough fall damage to hurt, measured from its peak height.
   * Chain legs start separate falls; returns false when there is no landing. */
  landingHurts (p: HopProgram, whole = true): boolean {
    // A climbable catch has no touchdown or fall damage.
    if (p.catchTarget) return false
    const track: number[] = []
    if (this.fly(p, 0, 0, track) < 0 || track.length < 6) return false
    let top = p.sy
    for (let i = 0; i < track.length; i += 3) {
      // Touching a climbable ends the fall, as on the server.
      if ((this.kindAt(Math.floor(track[i]), Math.floor(track[i + 1]), Math.floor(track[i + 2])) & SIM_CLIMBABLE) !== 0) top = track[i + 1]
      else top = Math.max(top, track[i + 1])
    }
    const x = track[track.length - 3]
    const y = track[track.length - 2]
    const z = track[track.length - 1]
    const kind = this.kindAt(Math.floor(x), Math.floor(y - 0.2), Math.floor(z))
    // Slime touchdown is sneaked onto, so the whole fall applies.
    const sneaking = p.slimeLand === true && (kind & SIM_SLIME) !== 0
    return fallDamagePoints(top - y, Math.max(0, track[track.length - 5] - y), kind, sneaking, whole) >= 1
  }

  /** The body at program `p`'s touchdown, flown from its start, or null. */
  private touchdownOf (p: HopProgram): SimBody | null {
    const out = newBody(0, 0, 0)
    this.touch = out
    try {
      return this.fly(p) >= 0 ? out : null
    } finally {
      this.touch = null
    }
  }

  /** Ticks for the quickest simple next leg from moving touchdown `b` to `t`, or -1.
   * Search work is limited to NEXT_LEG_TICKS. */
  private nextLegTicks (b: SimBody, t: [number, number, number], deadline: number): number {
    const lands = this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return -1
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: b.y }, live: copyBody(newBody(0, 0, 0), b) }]
    const limit = this.tickLimit
    this.tickLimit = this.ticksFlown + NEXT_LEG_TICKS
    try {
      let best = -1
      for (const family of NEXT_FAMILIES) {
        if (this.expired(deadline)) break
        const q = this.search(pads, lands, t, family, deadline, true, false)
        if (q !== null && (best < 0 || q.ticks < best)) best = q.ticks
      }
      return best
    } finally {
      this.tickLimit = limit
    }
  }

  /** Choose the lowest total of arrival ticks and next-leg ticks among near-quickest robust candidates.
   * Only rank pinned touchdowns: retained speed toward `next` usually makes further search unnecessary. */
  private nextLegPick (best: HopProgram, cands: HopProgram[], next: [number, number, number], deadline: number): HopProgram {
    const pool = cands.filter(p => p.ticks <= best.ticks + NEXT_SLACK).sort((p, q) => p.ticks - q.ticks)
    if (pool.length < 2) return best
    const td0 = this.touchdownOf(best)
    if (td0 === null) return best
    if (NEXT_ALONG) {
      const dx = next[0] + 0.5 - td0.x
      const dz = next[2] + 0.5 - td0.z
      const d = Math.hypot(dx, dz)
      if ((d > 1e-6 ? (td0.vx * dx + td0.vz * dz) / d : Math.hypot(td0.vx, td0.vz)) >= NEXT_PINNED) return best
    } else if (Math.hypot(td0.vx, td0.vz) >= NEXT_PINNED) return best
    const seen: SimBody[] = []
    const stop = this.ticksFlown + NEXT_TOTAL_TICKS
    let pick = best
    let pickTotal = Infinity
    let bestTotal = Infinity
    for (const p of [best, ...pool.filter(q => q !== best)]) {
      if (seen.length >= NEXT_CANDS || this.ticksFlown >= stop || performance.now() > deadline) break
      const td = this.touchdownOf(p)
      if (td === null) continue
      if (seen.some(s => Math.abs(s.x - td.x) < 0.05 && Math.abs(s.z - td.z) < 0.05 && Math.abs(s.vx - td.vx) < 0.02 && Math.abs(s.vz - td.vz) < 0.02)) continue
      seen.push(td)
      const n = this.nextLegTicks(td, next, deadline)
      const total = n < 0 ? Infinity : p.ticks + n
      if (p === best) bestTotal = total
      if (total < pickTotal) { pickTotal = total; pick = p }
    }
    return pickTotal < bestTotal ? pick : best
  }

  /** Robust hop from a stand on `r` to `t`, or null; returns the quickest of the first family that lands.
   * `families` selects the search families; simpler programs take priority. `grid` adds fixed yaw headings. */
  hop (r: [number, number, number], t: [number, number, number], deadline: number, families: HopFamilies = 'simple', grid = false, corridor: ReadonlyArray<{ x: number, y: number, z: number }> = [], depthLimit = BEAM_DEPTH): HopProgram | null {
    this.noVerdict = true
    this.noStand = true
    const lands = this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return null
    const pads = this.padsAt(r[0], r[1], r[2])
    if (pads.length === 0) return null
    this.noVerdict = false
    this.noStand = false
    if (families === 'beam') return this.beamSearch(pads, lands, t, deadline, corridor, undefined, depthLimit)
    this.deepOnly = families === 'deep'
    try {
      for (const family of FAMILIES[families]) {
        if (this.expired(deadline)) { this.noVerdict = true; break }
        const p = this.search(pads, lands, t, family, deadline, false, grid)
        if (p !== null) return p
      }
    } finally {
      this.deepOnly = false
    }
    return null
  }

  /** hop() on the deep family: classes('bounce') yields its deep classes alone. */
  private deepOnly = false

  /** Whether every nudged start lands on the target in one flight.
   * Touch-and-go verification normally stops twins at the first touchdown, which can fall short of the target. */
  singleFlight (p: HopProgram): boolean {
    if (p.legs !== true) return true
    return this.robust({ ...p, legs: false }) >= 0
  }

  /** Re-anchor the same controls at body `b` and known `yaw`; returns null unless they remain robust.
   * A small line-up offset can then reuse a planned program without another search. */
  anchored (p: HopProgram, b: SimBody, yaw = NaN): HopProgram | null {
    const q: HopProgram = {
      ...p,
      sx: b.x,
      sy: b.y,
      sz: b.z,
      start: copyBody(newBody(0, 0, 0), b),
      yaw0: Number.isNaN(yaw) ? undefined : yaw
    }
    const ticks = this.robust(q)
    return ticks >= 0 ? { ...q, ticks } : null
  }

  /** Find one or two sneaking presses, rounded to LOOK_STEP, whose settled rest re-anchors `p`.
   * Precision programs need finer placement than a walking line-up; returns presses, rest and program, or null. */
  lineUp (p: HopProgram, b: SimBody, yaw: number, deadline: number, moving?: { reach: number, tried: boolean }): { yaws: number[], delay?: number, rest: SimBody, prog: HopProgram } | null {
    if (moving !== undefined) {
      moving.tried = false
      return MOVING_LINEUP ? this.movingLineUp(p, b, yaw, deadline, moving) : null
    }
    const c = this.ctl
    const steps = Math.round((2 * Math.PI) / Math.abs(LOOK_STEP))
    // Match turnTo arithmetic so replay reproduces each rounded yaw exactly.
    const turned = (y: number, m: number): number => y + m * LOOK_STEP
    const press = (q: SimBody, y: number): void => {
      c.forward = true; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = true
      c.hx = -Math.sin(Math.PI - y); c.hz = Math.cos(Math.PI - y)
      this.sim.step(q, c)
      this.ticksFlown++
    }
    const coast = (q: SimBody): boolean => {
      c.forward = false
      for (let k = 0; k < LINEUP_COAST; k++) {
        this.sim.step(q, c)
        this.ticksFlown++
        if (q.vx === 0 && q.vz === 0) return q.onGround
      }
      return false
    }
    const cands: Array<{ err: number, yaws: number[], rest: SimBody }> = []
    const keep = (q: SimBody, yaws: number[]): void => {
      if (q.inLiquid) return
      const err = Math.hypot(q.x - p.sx, q.z - p.sz)
      if (err < LINEUP_ACCEPT && Math.abs(q.y - p.sy) < 1e-6) cands.push({ err, yaws, rest: q })
    }
    for (let k = 0; k < steps && !this.expired(deadline); k += LINEUP_STRIDE) {
      const y = turned(yaw, k)
      const q = copyBody(newBody(0, 0, 0), b)
      press(q, y)
      if (coast(q)) keep(q, [y])
    }
    const ex = p.sx - b.x
    const ez = p.sz - b.z
    for (let k = 0; k < steps && !this.expired(deadline); k += 2 * LINEUP_STRIDE) {
      const y1 = turned(yaw, k)
      const q1 = copyBody(newBody(0, 0, 0), b)
      press(q1, y1)
      const rx = ex + Math.sin(y1) * LINEUP_PRESS
      const rz = ez + Math.cos(y1) * LINEUP_PRESS
      const m0 = Math.round((Math.atan2(-rx, -rz) - y1) / LOOK_STEP)
      for (let dm = -LINEUP_SPREAD; dm <= LINEUP_SPREAD; dm += 2) {
        const y2 = turned(y1, m0 + dm)
        const q = copyBody(newBody(0, 0, 0), q1)
        press(q, y2)
        if (coast(q)) keep(q, [y1, y2])
      }
    }
    cands.sort((u, v) => u.err - v.err)
    // Always verify the best candidate even if the sweep exhausts its budget.
    for (let i = 0; i < cands.length && i < LINEUP_VERIFY && (i === 0 || !this.expired(deadline)); i++) {
      const cd = cands[i]
      const q = this.anchored(p, cd.rest, cd.yaws[cd.yaws.length - 1])
      if (q !== null) return { yaws: cd.yaws, rest: cd.rest, prog: q }
    }
    return null
  }

  // Aim from the predicted glide rest but start presses at the current body state.
  // Spend the approach's single search only when that rest is near.
  private movingLineUp (p: HopProgram, b: SimBody, yaw: number, deadline: number, moving: { reach: number, tried: boolean }): ReturnType<HopOracle['lineUp']> {
    const clone = (q: SimBody): SimBody => copyBody(newBody(0, 0, 0), q)
    // anchored() mutates this.ctl; coasting must use independent controls with every movement key released.
    const c: SimControl = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: true, hx: 0, hz: -1 }
    const safe = (q: SimBody): boolean => {
      if (!q.onGround || q.vy > 0 || q.inLiquid || Math.abs(q.y - p.sy) > 1e-6 || !this.sim.carried(q)) return false
      const hw = this.sim.halfWidth
      for (let y = Math.floor(q.y + 0.001); y <= Math.floor(q.y + this.sim.height - 0.001); y++) {
        for (let z = Math.floor(q.z - hw + 0.001); z <= Math.floor(q.z + hw - 0.001); z++) {
          for (let x = Math.floor(q.x - hw + 0.001); x <= Math.floor(q.x + hw - 0.001); x++) {
            const s = this.sim.stateAt(x, y, z)
            if (s < 0 || (this.world.kind[s] & (SIM_LIQUID | SIM_HAZARD)) !== 0) return false
          }
        }
      }
      return true
    }
    const step = (q: SimBody, look?: number): boolean => {
      if (this.expired(deadline)) return false
      c.forward = look !== undefined
      if (look !== undefined) { c.hx = -Math.sin(Math.PI - look); c.hz = Math.cos(Math.PI - look) }
      this.sim.step(q, c)
      this.ticksFlown++
      // onGround can remain set on the tick the body slides off a thin panel.
      return safe(q)
    }
    const coast = (q: SimBody): number => {
      for (let k = 1; k <= LINEUP_COAST; k++) {
        if (!step(q)) return -1
        if (q.vx === 0 && q.vz === 0) return k
      }
      return -1
    }
    const anchor = (q: SimBody, look: number): HopProgram | null => {
      if (this.expired(deadline)) return null
      const limit = this.tickLimit
      const previous = this.anchorDeadline
      this.tickLimit = Math.min(limit > 0 ? limit : Infinity, this.ticksFlown + LINEUP_ANCHOR_TICKS)
      this.anchorDeadline = deadline
      try { return this.anchored(p, q, look) } finally { this.tickLimit = limit; this.anchorDeadline = previous }
    }
    if (p.climb === true || p.deep === true || !safe(b)) return null
    const glide = clone(b)
    const glideTicks = coast(glide)
    if (glideTicks < 0 || Math.hypot(glide.x - p.sx, glide.z - p.sz) > moving.reach) return null
    moving.tried = true
    // Verify the prefix from nudged starts too; a coast skimming a panel edge can be too fragile.
    const holds = (delay: number, yaws: number[]): boolean => {
      for (const [nx, nz] of [[LIVE_NUDGE, 0], [-LIVE_NUDGE, 0], [0, LIVE_NUDGE], [0, -LIVE_NUDGE]]) {
        const q = clone(b)
        q.x += nx
        q.z += nz
        if (this.sim.collides(q) && !this.sim.collides(b)) continue
        for (let k = 0; k < delay; k++) if (!step(q)) return false
        for (const y of yaws) if (!step(q, y)) return false
        if (coast(q) < 0) return false
      }
      return true
    }
    const zero = anchor(glide, yaw)
    if (zero !== null && holds(0, [])) return { yaws: [], delay: 0, rest: glide, prog: zero }
    const steps = Math.round((2 * Math.PI) / Math.abs(LOOK_STEP))
    const aimed = (from: SimBody, look: number): number =>
      look + Math.round((Math.atan2(from.x - p.sx, from.z - p.sz) - look) / LOOK_STEP) * LOOK_STEP
    const first = aimed(glide, yaw)
    const cands: Array<{ ticks: number, err: number, delay: number, yaws: number[], rest: SimBody }> = []
    const keep = (q: SimBody, delay: number, yaws: number[]): void => {
      const ticks = coast(q)
      if (ticks < 0) return
      const err = Math.hypot(q.x - p.sx, q.z - p.sz)
      if (err < LINEUP_ACCEPT) cands.push({ ticks: delay + yaws.length + ticks, err, delay, yaws, rest: q })
    }
    // Reserve enough budget to check the candidate controls.
    const searchEnd = this.tickLimit > 0 ? this.ticksFlown + Math.floor((this.tickLimit - this.ticksFlown) * 0.75) : Infinity
    const searching = (): boolean => this.ticksFlown < searchEnd && !this.expired(deadline)
    for (let presses = 1; presses <= 2 && searching(); presses++) {
      for (let delay = 0; delay <= 1 && searching(); delay++) {
        const start = clone(b)
        if (delay !== 0 && !step(start)) continue
        for (let k = 0; k < steps && searching(); k += presses * LINEUP_STRIDE) {
          const y1 = first + k * LOOK_STEP
          const q1 = clone(start)
          if (!step(q1, y1)) continue
          if (presses === 1) { keep(q1, delay, [y1]); continue }
          const end = clone(q1)
          if (coast(end) < 0) continue
          const second = aimed(end, y1)
          for (let dm = -LINEUP_SPREAD; dm <= LINEUP_SPREAD && searching(); dm += 2) {
            const y2 = second + dm * LOOK_STEP
            const q = clone(q1)
            if (step(q, y2)) keep(q, delay, [y1, y2])
          }
        }
      }
    }
    cands.sort((u, v) => u.ticks - v.ticks || u.err - v.err)
    let best: ReturnType<HopOracle['lineUp']> = null
    let cost = Infinity
    for (let i = 0; i < cands.length && i < LINEUP_VERIFY && !this.expired(deadline); i++) {
      const cd = cands[i]
      const q = anchor(cd.rest, cd.yaws[cd.yaws.length - 1])
      if (q !== null && cd.ticks + q.ticks < cost && holds(cd.delay, cd.yaws)) {
        cost = cd.ticks + q.ticks
        best = { yaws: cd.yaws, delay: cd.delay, rest: cd.rest, prog: q }
      }
    }
    return best
  }

  // onGround can outlive support, and the kernel checks hazards before moving.
  private prefixSafe (b: SimBody, y: number): boolean {
    if (!b.onGround || b.vy > 0 || b.inLiquid || Math.abs(b.y - y) > 1e-6 ||
        this.sim.collides(b) || !this.sim.carried(b) || this.sim.onClimbable(b)) return false
    const hw = this.sim.halfWidth
    for (let x = Math.floor(b.x - hw + 1e-7); x <= Math.floor(b.x + hw - 1e-7); x++) {
      for (let z = Math.floor(b.z - hw + 1e-7); z <= Math.floor(b.z + hw - 1e-7); z++) {
        for (let yy = Math.floor(b.y + 1e-7); yy <= Math.floor(b.y + this.sim.height - 1e-7); yy++) {
          const state = this.sim.stateAt(x, yy, z)
          if (state < 0 || (this.world.kind[state] & (SIM_LIQUID | SIM_HAZARD)) !== 0) return false
        }
      }
    }
    return true
  }

  private groundTick (f: Flight, control: { yaw: number, sprint: boolean, sneak: boolean }, y: number, track: number[] | null): boolean {
    if (!Number.isFinite(control.yaw) || !this.prefixSafe(f.b, y)) return false
    this.seqControl(f, this.ctl, control.yaw, F_FWD | (control.sprint ? F_SPRINT : 0))
    this.ctl.sneak = control.sneak
    this.kernelStep(f.b, this.ctl)
    if (track !== null) track.push(f.b.x, f.b.y, f.b.z)
    f.t++
    f.leg = f.t
    return this.prefixSafe(f.b, y)
  }

  // Retreat from the collision face; a planned stand supplies only a direction.
  private prefixYaws (b: SimBody, t: readonly [number, number, number], yaw: number, hint?: HopProgram): number[] {
    const out: number[] = []
    const keys = new Set<number>()
    const add = (dx: number, dz: number): void => {
      if (Math.hypot(dx, dz) < 1e-6) return
      const want = Math.atan2(-dx, -dz)
      const key = Math.round((want - yaw) / LOOK_STEP)
      if (!keys.has(key)) { keys.add(key); out.push(want) }
    }
    if (hint !== undefined && Math.abs(hint.sy - b.y) < 1e-6) add(hint.sx - b.x, hint.sz - b.z)
    const faces: Array<{ dx: number, dz: number, d: number }> = []
    const w = this.world
    const r = this.sim.halfWidth + 0.5
    for (let x = Math.floor(b.x - r); x <= Math.floor(b.x + r); x++) {
      for (let z = Math.floor(b.z - r); z <= Math.floor(b.z + r); z++) {
        for (let y = Math.floor(b.y) - 1; y <= Math.floor(b.y + this.sim.height); y++) {
          const state = this.sim.stateAt(x, y, z)
          if (state < 0 || w.shapeStart[state] < 0) continue
          for (let k = 0; k < w.shapeCount[state]; k++) {
            const o = (w.shapeStart[state] + k) * 6
            if (y + w.boxes[o + 4] <= b.y + 1e-6 || y + w.boxes[o + 1] >= b.y + this.sim.height) continue
            const dx = b.x - Math.max(x + w.boxes[o], Math.min(b.x, x + w.boxes[o + 3]))
            const dz = b.z - Math.max(z + w.boxes[o + 2], Math.min(b.z, z + w.boxes[o + 5]))
            const d = Math.hypot(dx, dz)
            if (d > 1e-6 && d <= r) faces.push({ dx, dz, d })
          }
        }
      }
    }
    faces.sort((a, b) => a.d - b.d)
    for (const f of faces) {
      if (out.length >= 4) break
      add(f.dx, f.dz)
      const side = f.dz * (t[0] + 0.5 - b.x) - f.dx * (t[2] + 0.5 - b.z) >= 0 ? 1 : -1
      if (out.length < 4) add(f.dx + side * f.dz, f.dz - side * f.dx)
    }
    add(b.x - t[0] - 0.5, b.z - t[2] - 0.5)
    return out
  }

  /** Try a short supported retreat followed by an ordinary jump from its endpoint. */
  hopPrefix (b: SimBody, t: [number, number, number], deadline: number, yaw: number,
    land?: { x: number, y: number, z: number }, hint?: HopProgram): HopProgram | null {
    if (!LIVE_PREFIX || this.prefixBudget !== null || !Number.isFinite(yaw) || Math.hypot(b.vx, b.vz) >= 0.01 ||
        !this.prefixSafe(b, b.y)) return null
    const limit = this.tickLimit
    const anchor = this.anchorDeadline
    const end = Math.min(limit > 0 ? limit : Infinity, this.ticksFlown + LIVE_PREFIX_TICKS)
    this.prefixBudget = { end, deadline }
    this.anchorDeadline = deadline
    let best: HopProgram | null = null
    try {
      if (this.expired(deadline)) return null
      const lands = land !== undefined ? this.regionOf(t, land) : this.stands(t[0], t[1], t[2])
      if (lands.length === 0) return null
      const start = copyBody(newBody(0, 0, 0), b)
      const offsets: Array<[number, number]> = [[0, 0], ...this.requiredNudges({ start }, LIVE_NUDGE)]
      const prefixes: Array<{ controls: NonNullable<HopProgram['groundPrefix']>, pad: Pad }> = []
      for (const want of this.prefixYaws(start, t, yaw, hint)) {
        for (const [sprint, sneak] of [[false, false], [true, false], [false, true]]) {
          const fs = offsets.map(([nx, nz]): Flight => {
            const body = copyBody(newBody(0, 0, 0), start)
            body.x += nx; body.z += nz
            return { b: body, t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0, top: b.y }
          })
          const controls: NonNullable<HopProgram['groundPrefix']> = []
          for (let n = 1; n <= 4; n++) {
            const control = { yaw: want, sprint, sneak }
            if (!fs.every(f => this.groundTick(f, control, start.y, null))) break
            controls.push(control)
            const f = flightInto(newFlight(), fs[0])
            f.t = 0; f.leg = 0
            if (Math.hypot(f.b.x - start.x, f.b.z - start.z) < 0.02) continue
            prefixes.push({ controls: controls.slice(), pad: { f, live: copyBody(newBody(0, 0, 0), f.b) } })
          }
        }
      }
      // Leave budget for air turns from every retreat endpoint and for verification.
      const families: Array<[Family, number]> = [['air-turn', 6], ['run-turn', 1], ['straight', 1], ['yaw', 1]]
      let jobs = prefixes.length * 9
      for (const [family, weight] of families) {
        for (const prefix of prefixes) {
          if (this.ticksFlown >= end || performance.now() > deadline) return best
          const share = Math.floor((end - this.ticksFlown) * weight / jobs)
          jobs -= weight
          const stop = this.ticksFlown + share
          this.tickLimit = this.ticksFlown + Math.floor(share * 0.75)
          this.search([prefix.pad], lands, t, family, deadline, true, true, land, undefined, found => {
            // Verify the original nudged starts through the retreat, rather than introducing new nudges at its endpoint.
            this.tickLimit = stop
            found.sort((a, b) => a.ticks - b.ticks)
            for (const suffix of found) {
              if (this.expired(deadline)) break
              const p: HopProgram = {
                ...suffix, sx: start.x, sy: start.y, sz: start.z, start, yaw0: yaw,
                groundPrefix: prefix.controls, jumpAt: suffix.jumpAt + prefix.controls.length,
                ticks: suffix.ticks + prefix.controls.length
              }
              if (best !== null && p.ticks >= best.ticks) break
              let ticks = -1
              let ok = true
              for (let i = 0; i < offsets.length; i++) {
                const r = this.fly(p, offsets[i][0], offsets[i][1])
                if (r < 0) { ok = false; break }
                if (i === 0) ticks = r
              }
              if (ok && (best === null || ticks < best.ticks)) best = { ...p, ticks }
            }
            return null
          })
        }
      }
      return best
    } catch (error) {
      if (error !== PREFIX_EXPIRED) throw error
      return best
    } finally {
      this.prefixBudget = null
      this.anchorDeadline = anchor
      this.tickLimit = limit
    }
  }

  /** Robust live hop from exact body state `b` to `t`, including walking jumps and optional beam fallback.
   * `yaw` reproduces bot.look rounding (NaN: unknown). Returns the quickest landing found, or null. */
  hopFrom (b: SimBody, t: [number, number, number], deadline: number, yaw = NaN, land?: { x: number, y: number, z: number }, next?: [number, number, number], beam = true,
    seed?: ReadonlyArray<'kick' | 'pivot'>, seedTicks = 0, seedFirst = true, hint?: HopProgram): HopProgram | null {
    // Keep a planned region landing's live search in that region.
    const lands = land !== undefined ? this.regionOf(t, land) : this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return null
    const live = copyBody(newBody(0, 0, 0), b)
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0, top: b.y }, live }]
    let best: HopProgram | null = null
    // Collect near-quickest programs when the next leg affects touchdown choice.
    const cands: HopProgram[] | null = next !== undefined ? [] : null
    // Seed kick and pivot families can recover starts beside their planned lines that ordinary live families miss.
    // Give them their own allowance, before ordinary families when seedFirst is set, or after failure otherwise.
    const seeded = (): void => {
      if (seed === undefined || seed.length === 0) return
      const limit = this.tickLimit
      const from = this.ticksFlown
      this.tickLimit = seedTicks > 0 ? from + seedTicks : limit
      try {
        for (const family of seed) {
          if (this.expired(deadline)) break
          const p = this.search(pads, lands, t, family, deadline, true, true, land)
          if (p !== null && (best === null || p.ticks < best.ticks)) best = p
          if (p !== null) break
        }
      } finally {
        this.tickLimit = limit > 0 && seedTicks > 0 ? limit + (this.ticksFlown - from) : limit
      }
    }
    this.collect = cands
    try {
      // Try the planned lane within this pass's rollout and verification allowance.
      if (LANE_KICK_TICKS > 0 && hint?.family === 'kick' && hint.kick === true && hint.seq === undefined &&
          hint.run.fixed && hint.air1.fixed && hint.jumpOnLeave !== true && hint.climb !== true && hint.deep !== true &&
          Number.isInteger(hint.jumpAt) && hint.jumpAt >= 0 && hint.jumpAt <= 16 &&
          hint.tx === t[0] && hint.ty === t[1] && hint.tz === t[2]) {
        const limit = this.tickLimit
        const from = this.ticksFlown
        this.tickLimit = from + LANE_KICK_TICKS
        try {
          if (!this.expired(deadline)) best = this.search(pads, lands, t, 'kick', deadline, false, true, land, hint)
        } finally {
          this.tickLimit = limit > 0 ? limit + (this.ticksFlown - from) : limit
        }
      }
      if (seedFirst) seeded()
      for (const family of FAMILIES.all) {
        if (this.expired(deadline)) break
        if (family === 'leave' && best !== null) break
        const p = this.search(pads, lands, t, family, deadline, true, true, land)
        if (p !== null && (best === null || p.ticks < best.ticks)) best = p
      }
      if (!seedFirst && best === null) seeded()
    } finally {
      this.collect = null
    }
    if (best === null) return beam ? this.beamSearch(pads, lands, t, deadline, [], land) : null
    // A touchdown that retains forward speed can make the next leg quicker than an equally fast pinned landing.
    return cands !== null && next !== undefined ? this.nextLegPick(best, cands, next, deadline) : best
  }

  /** Choose a robust touchdown on stone `t` by its arrival time plus the next leg's time.
   * Thin stones need momentum, so stopping is not required. Returns null without a viable next leg.
   * `nextGo` also treats the next landing as a touch-and-go stone. */
  hopChain (b: SimBody, t: [number, number, number], next: [number, number, number], deadline: number, yaw = NaN, nextGo = false): HopProgram | null {
    const lands = this.stands(t[0], t[1], t[2])
    if (lands.length === 0 || this.stands(next[0], next[1], next[2]).length === 0) return null
    const live = copyBody(newBody(0, 0, 0), b)
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0, top: b.y }, live }]
    // Share the caller's budget between searching and ranking, reserving work for the yaw round.
    const limit = this.tickLimit
    const end = limit > 0 ? limit : Infinity
    const share = (ticks: number, until: number): number => {
      const at = Math.min(until, this.ticksFlown + ticks)
      return at === Infinity ? 0 : at
    }
    const reserve = CHAIN_SEARCH_TICKS + QR_GRACE_TICKS + CHAIN_YAW_LEGS * NEXT_LEG_TICKS
    const seen: SimBody[] = []
    let pick: HopProgram | null = null
    let pickTotal = Infinity
    const round = (families: readonly Family[], until: number): void => {
      const cands: HopProgram[] = []
      this.collect = cands
      this.goLanding = true
      this.tickLimit = share(CHAIN_SEARCH_TICKS, until)
      for (const family of families) {
        if (this.expired(deadline)) break
        this.collectCap = cands.length + CHAIN_PICK_COLLECT
        this.search(pads, lands, t, family, deadline, true, family === 'yaw')
      }
      this.collect = null
      cands.sort((p, q) => p.ticks - q.ticks)
      const stop = Math.min(until, this.ticksFlown + CHAIN_PICK_TICKS)
      this.tickLimit = limit
      let weighed = 0
      for (const p of cands) {
        // An unfinished next-leg search gives no verdict, so start one only with enough budget to finish.
        if (weighed >= CHAIN_PICK_CANDS || this.ticksFlown + NEXT_LEG_TICKS > stop || performance.now() > deadline) break
        // Stop when no later touchdown can beat the best total even with the minimum next-leg time.
        if (p.ticks + CHAIN_PICK_LEAST >= pickTotal) break
        this.goLanding = true
        const td = this.touchdownOf(p)
        this.goLanding = nextGo
        if (td === null) continue
        if (seen.some(s => Math.abs(s.x - td.x) < 0.02 && Math.abs(s.z - td.z) < 0.02 && Math.abs(s.vx - td.vx) < 0.01 && Math.abs(s.vz - td.vz) < 0.01)) continue
        seen.push(td)
        weighed++
        const n = this.nextLegTicks(td, next, deadline)
        if (n >= 0 && p.ticks + n < pickTotal) { pickTotal = p.ticks + n; pick = p }
      }
    }
    this.collectSlack = CHAIN_PICK_SLACK
    try {
      round(NEXT_FAMILIES, end - reserve)
      // Try the expensive yaw family only if simple touchdowns have no viable next leg and budget remains.
      if (pick === null && !this.expired(deadline) && end - this.ticksFlown >= CHAIN_SEARCH_TICKS + NEXT_LEG_TICKS) round(CHAIN_YAW_FAMILY, end)
      return pick
    } finally {
      this.collect = null
      this.collectSlack = NEXT_SLACK
      this.collectCap = NEXT_COLLECT
      this.goLanding = false
      this.tickLimit = limit
    }
  }


  private readonly underCache = new Map<number, Stand[]>()
  private readonly regionsCache = new Map<number, Region[]>()
  private readonly regionOfCache = new Map<string, Stand[]>()
  private readonly linkBody = newBody(0, 0, 0)

  /** Rests beside a thin block below the node, where the planner assumes the block's top.
   * Exclude rests carried by another column, since those already have a planner node. */
  underStands (x: number, y: number, z: number): Stand[] {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.underCache.get(k)
    if (hit !== undefined) return hit
    const out: Stand[] = []
    const fb = this.flagsAt(x, y - 1, z)
    const thinFloor = (fb & (F_SAFE | F_PHYSICAL | F_CLIMBABLE)) === (F_SAFE | F_PHYSICAL)
    if ((fb & F_PHYSICAL) !== 0 && !thinFloor && this.sim.stateAt(x, y - 1, z) >= 0 && this.sim.stateAt(x, y, z) >= 0 && this.shaped(x, y - 2, z) && !this.fillsFootprint(x, y - 1, z)) {
      const low = y - 1 + Math.max(0, this.cellTop(x, y - 2, z) - 1) + 0.001
      const hw = this.sim.halfWidth
      const w = this.world
      const oxs = new Set<number>([0.2, 0.5, 0.8])
      const ozs = new Set<number>([0.2, 0.5, 0.8])
      for (const yy of [y - 1, y]) {
        const s = this.sim.stateAt(x, yy, z)
        if (s < 0 || w.shapeStart[s] < 0) continue
        for (let q = 0; q < w.shapeCount[s]; q++) {
          const o = (w.shapeStart[s] + q) * 6
          for (const e of [0.001, 0.07]) {
            oxs.add(w.boxes[o] - hw - e); oxs.add(w.boxes[o + 3] + hw + e)
            ozs.add(w.boxes[o + 2] - hw - e); ozs.add(w.boxes[o + 5] + hw + e)
          }
        }
      }
      for (const ox of oxs) {
        for (const oz of ozs) {
          if (!(ox > 0.01 && ox < 0.99 && oz > 0.01 && oz < 0.99)) continue
          const b = newBody(x + ox, low, z + oz)
          if (this.sim.collides(b) || !this.settle(b)) continue
          if (Math.floor(b.x) !== x || Math.floor(b.z) !== z) continue
          if (b.y >= y - 0.5 || this.nodeY(x, b.y, z) !== y) continue
          if (this.sim.collides(b) || !this.sim.supportedIn(b, x, z) || this.carriedBeside(b, x, z)) continue
          if (!out.some(p => Math.abs(p.x - b.x) < 0.05 && Math.abs(p.z - b.z) < 0.05)) out.push({ x: b.x, y: b.y, z: b.z, catch: false, under: true })
        }
      }
    }
    this.underCache.set(k, out)
    return out
  }

  /** Does a box of the cell cover its whole footprint (nothing stands beside it in its column)? */
  private fillsFootprint (x: number, y: number, z: number): boolean {
    const s = this.sim.stateAt(x, y, z)
    if (s < 0 || this.world.shapeStart[s] < 0) return false
    const w = this.world
    for (let q = 0; q < w.shapeCount[s]; q++) {
      const o = (w.shapeStart[s] + q) * 6
      if (w.boxes[o] <= 1e-9 && w.boxes[o + 2] <= 1e-9 && w.boxes[o + 3] >= 1 - 1e-9 && w.boxes[o + 5] >= 1 - 1e-9) return true
    }
    return false
  }

  /** Is the body carried by a column other than (x, z) too? */
  private carriedBeside (b: SimBody, x: number, z: number): boolean {
    const hw = this.sim.halfWidth
    for (let cx = Math.floor(b.x - hw); cx <= Math.floor(b.x + hw); cx++) {
      for (let cz = Math.floor(b.z - hw); cz <= Math.floor(b.z + hw); cz++) {
        if ((cx !== x || cz !== z) && this.sim.supportedIn(b, cx, cz)) return true
      }
    }
    return false
  }

  /** Does a body at rest at `a` walk straight to (bx, bz) on the level: nothing in the way, carried all along? */
  walkLink (a: { x: number, y: number, z: number }, bx: number, bz: number): boolean {
    const d = Math.hypot(bx - a.x, bz - a.z)
    const n = Math.max(1, Math.ceil(d / 0.05))
    const p = this.linkBody
    for (let i = 0; i <= n; i++) {
      p.x = a.x + ((bx - a.x) * i) / n
      p.y = a.y
      p.z = a.z + ((bz - a.z) * i) / n
      if (this.sim.collides(p) || !this.sim.carried(p)) return false
    }
    return true
  }

  /** Walk-connected regions of a node's stands and underStands; a climbable catch is one region.
   * A pane rail separates its top and the floors on either side. */
  regions (x: number, y: number, z: number): Region[] {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.regionsCache.get(k)
    if (hit !== undefined) return hit
    const out: Region[] = []
    const all = [...this.stands(x, y, z), ...this.underStands(x, y, z)]
    const catches = all.filter(s => s.catch)
    if (catches.length > 0) out.push({ stands: catches, y: catches[0].y, under: false })
    const left = all.filter(s => !s.catch)
    while (left.length > 0) {
      const comp = [left.shift() as Stand]
      for (let i = 0; i < comp.length; i++) {
        for (let j = left.length - 1; j >= 0; j--) {
          if (Math.abs(left[j].y - comp[i].y) < 1e-3 && this.walkLink(comp[i], left[j].x, left[j].z)) comp.push(left.splice(j, 1)[0])
        }
      }
      // Under regions lie beside a block; the top of a low block remains the planner's own region.
      out.push({ stands: comp, y: comp[0].y, under: comp.some(s => s.under === true) })
    }
    this.regionsCache.set(k, out)
    return out
  }

  /** Whether a rest lies beside the block below node `t`, rather than on its top. */
  besideRest (t: readonly [number, number, number], rest: { x: number, y: number, z: number }): boolean {
    if (rest.y >= t[1] - 0.5 || this.underStands(t[0], t[1], t[2]).length === 0) return false
    return rest.y < t[1] - 1 + this.cellTop(t[0], t[1] - 1, t[2]) - 1e-6
  }

  /** The stands of node t a body at rest at `land` walks to, `land` itself first. */
  regionOf (t: readonly [number, number, number], land: { x: number, y: number, z: number }): Stand[] {
    const key = `${t[0]},${t[1]},${t[2]}:${Math.round(land.x * 256)},${Math.round(land.y * 256)},${Math.round(land.z * 256)}`
    const hit = this.regionOfCache.get(key)
    if (hit !== undefined) return hit
    const out: Stand[] = [{ x: land.x, y: land.y, z: land.z, catch: false }]
    for (const r of this.regions(t[0], t[1], t[2])) {
      if (Math.abs(r.y - land.y) > 1e-3 || r.stands[0].catch) continue
      if (r.stands.some(s => this.walkLink(land, s.x, s.z))) out.push(...r.stands)
    }
    this.regionOfCache.set(key, out)
    return out
  }

  /** Is a body at rest in program p's landing region (HopProgram.land)? */
  private inRegion (b: SimBody, p: HopProgram): boolean {
    const land = p.land as { x: number, y: number, z: number }
    if (Math.abs(b.y - land.y) > 1e-3) return false
    for (const s of this.regionOf([p.tx, p.ty, p.tz], land)) if (this.walkLink(s, b.x, b.z)) return true
    return false
  }

  /** Where program p's body comes to rest: its landing, let go (sneak, no keys) until it stops; null if it does not land. */
  restOf (p: HopProgram): SimBody | null {
    const f = this.origin(p, 0, 0)
    if (f === null) return null
    if (this.advance(f, p, Infinity, Infinity, null) < 0) return null
    const c = this.ctl
    c.forward = false; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = true
    for (let t = 0; t < 30 && !(f.b.onGround && Math.abs(f.b.vx) < 1e-3 && Math.abs(f.b.vz) < 1e-3); t++) this.sim.step(f.b, c)
    c.sneak = false
    return f.b
  }

  /** Take-off pads at `rest` and stands reachable from it by a straight walk, matching executor line-up. */
  padsFromRest (n: readonly [number, number, number], rest: { x: number, y: number, z: number }): Pad[] {
    const out: Pad[] = []
    for (const s of this.regionOf(n, rest)) {
      if (s.x !== rest.x || s.z !== rest.z) { if (!this.walkLink(rest, s.x, s.z)) continue }
      const f = this.pad(s.x, s.y, s.z)
      if (f !== null && !out.some(q => Math.abs(q.f.b.x - f.b.x) < 0.02 && Math.abs(q.f.b.z - f.b.z) < 0.02)) out.push({ f, live: undefined })
    }
    return out
  }

  /** Discover hops from `rest` or stands it can walk to, targeting each region within hop range.
   * Includes other regions of the same node. `skipTop` excludes planner regions already reached. */
  discoverFromRest (
    src: { x: number, y: number, z: number },
    rest: { x: number, y: number, z: number },
    heuristic: (x: number, y: number, z: number) => number,
    budgetMs: number,
    skipTop: ((x: number, y: number, z: number) => boolean) | null,
    maxEdges = MAX_EDGES,
    families: HopFamilies = 'simple'
  ): HopEdge[] {
    const deadline = performance.now() + budgetMs
    this.ticksFlown = 0
    const out: HopEdge[] = []
    const pads = this.padsFromRest([src.x, src.y, src.z], rest)
    if (pads.length === 0) return out
    const own = this.regionOf([src.x, src.y, src.z], rest)
    const hR = heuristic(src.x, src.y, src.z)
    const cands: Array<[number, number, number, number]> = []
    for (let dx = -HOP_REACH; dx <= HOP_REACH; dx++) {
      for (let dz = -HOP_REACH; dz <= HOP_REACH; dz++) {
        if (dx * dx + dz * dz > HOP_REACH_SQ) continue
        for (let dy = HOP_DROP; dy <= HOP_RISE; dy++) {
          const h = heuristic(src.x + dx, src.y + dy, src.z + dz)
          if (h > hR + MAX_REGRESS) continue
          cands.push([src.x + dx, src.y + dy, src.z + dz, h])
        }
      }
    }
    cands.sort((p, q) => p[3] - q[3])
    let tried = 0
    for (const [x, y, z] of cands) {
      if (this.expired(deadline)) break
      for (const reg of this.regions(x, y, z)) {
        if (this.expired(deadline)) break
        if (x === src.x && y === src.y && z === src.z && reg.stands.some(s => own.includes(s))) continue
        if (!reg.under && skipTop !== null && skipTop(x, y, z)) continue
        if ((families === 'yaw' && ++tried > YAW_CANDIDATES) || (families === 'beam' && ++tried > BEAM_CANDIDATES)) return out
        const t: [number, number, number] = [x, y, z]
        const land = reg.stands[0].catch ? undefined : { x: reg.stands[0].x, y: reg.stands[0].y, z: reg.stands[0].z }
        let p: HopProgram | null = null
        this.noVerdict = false
        if (families === 'beam') p = this.beamSearch(pads, reg.stands, t, deadline, [], land)
        else {
          for (const family of FAMILIES[families]) {
            if (this.expired(deadline)) break
            p = this.search(pads, reg.stands, t, family, deadline, false, false, land)
            if (p !== null) break
          }
        }
        if (p !== null) out.push({ fromX: src.x, fromY: src.y, fromZ: src.z, program: p })
        if (out.length >= maxEdges) return out
      }
    }
    return out
  }

  /** Whether a standable, unreached candidate lies within hop range and MAX_REGRESS of (x, y, z). */
  opens (x: number, y: number, z: number, heuristic: (x: number, y: number, z: number) => number, reached: (x: number, y: number, z: number) => boolean, goalSide: ((x: number, y: number, z: number) => boolean) | null = null): boolean {
    const hR = heuristic(x, y, z)
    const cands: Array<[number, number, number, number]> = []
    for (let dx = -HOP_REACH; dx <= HOP_REACH; dx++) {
      for (let dz = -HOP_REACH; dz <= HOP_REACH; dz++) {
        if (dx * dx + dz * dz > HOP_REACH_SQ || (dx === 0 && dz === 0)) continue
        for (let dy = HOP_DROP; dy <= HOP_RISE; dy++) {
          const h = heuristic(x + dx, y + dy, z + dz)
          const side = goalSide !== null && goalSide(x + dx, y + dy, z + dz)
          if ((h > hR + MAX_REGRESS && !side) || reached(x + dx, y + dy, z + dz)) continue
          cands.push([x + dx, y + dy, z + dz, side ? h - GOAL_SIDE_FIRST : h])
        }
      }
    }
    cands.sort((p, q) => p[3] - q[3])
    for (const [cx, cy, cz] of cands) if (this.stands(cx, cy, cz).length > 0) return true
    return false
  }

  /** Index of the last firm-ground node before a path ending on ice, within CHAIN_BACK nodes, or -1.
   * Ice pads give a stationary body no run-up; later gaps can require momentum carried across earlier legs. */
  chainStart (path: ReadonlyArray<{ x: number, y: number, z: number }>): number {
    const n = path.length
    if (n < 2 || !this.onIce(path[n - 1].x, path[n - 1].y, path[n - 1].z)) return -1
    for (let i = n - 2; i >= 0 && i >= n - 1 - CHAIN_BACK; i--) {
      if (!this.onIce(path[i].x, path[i].y, path[i].z)) return i
    }
    return -1
  }

  /** Beam chains from `from` along `corridor` to unreached, in-range landings within MAX_REGRESS.
   * Depth covers the corridor at chain speed plus CHAIN_SLACK. */
  chain (
    from: { x: number, y: number, z: number },
    corridor: ReadonlyArray<{ x: number, y: number, z: number }>,
    heuristic: (x: number, y: number, z: number) => number,
    budgetMs: number,
    skip: ((x: number, y: number, z: number) => boolean) | null = null,
    maxEdges = MAX_EDGES
  ): HopEdge[] {
    const deadline = performance.now() + budgetMs
    this.ticksFlown = 0
    const out: HopEdge[] = []
    if (corridor.length === 0 || this.stands(from.x, from.y, from.z).length === 0) return out
    const r = corridor[corridor.length - 1]
    let len = Math.hypot(corridor[0].x - from.x, corridor[0].z - from.z)
    for (let i = 1; i < corridor.length; i++) len += Math.hypot(corridor[i].x - corridor[i - 1].x, corridor[i].z - corridor[i - 1].z)
    const hR = heuristic(r.x, r.y, r.z)
    const cands: Array<[number, number, number, number]> = []
    for (let dx = -HOP_REACH; dx <= HOP_REACH; dx++) {
      for (let dz = -HOP_REACH; dz <= HOP_REACH; dz++) {
        if (dx * dx + dz * dz > HOP_REACH_SQ || (dx === 0 && dz === 0)) continue
        for (let dy = HOP_DROP; dy <= HOP_RISE; dy++) {
          const x = r.x + dx
          const y = r.y + dy
          const z = r.z + dz
          const h = heuristic(x, y, z)
          if (h > hR + MAX_REGRESS) continue
          if (skip !== null && skip(x, y, z)) continue
          cands.push([x, y, z, h])
        }
      }
    }
    cands.sort((u, v) => u[3] - v[3])
    let tried = 0
    for (const [x, y, z] of cands) {
      if (this.expired(deadline) || tried >= BEAM_CANDIDATES) break
      if (this.stands(x, y, z).length === 0) continue
      tried++
      const depth = Math.max(BEAM_DEPTH, Math.ceil(CHAIN_SLACK * (len + Math.hypot(x - r.x, z - r.z)) / BEAM_SPEED))
      const q = this.hop([from.x, from.y, from.z], [x, y, z], deadline, 'beam', false, corridor, depth)
      if (q !== null) out.push({ fromX: from.x, fromY: from.y, fromZ: from.z, program: q })
      if (out.length >= maxEdges) break
    }
    return out
  }

  /** Discover up to `maxEdges` hops from `sources` within `budgetMs`, excluding `skip` and excessive retreat.
   * Sources are ordered nearest the goal last. `families` selects simple or escalated searches. */
  discover (
    sources: ReadonlyArray<{ x: number, y: number, z: number }>,
    heuristic: (x: number, y: number, z: number) => number,
    budgetMs: number,
    skip: ((x: number, y: number, z: number) => boolean) | null = null,
    maxEdges = MAX_EDGES,
    families: HopFamilies = 'simple',
    goalSide: ((x: number, y: number, z: number) => boolean) | null = null
  ): HopEdge[] {
    const deadline = performance.now() + budgetMs
    this.ticksFlown = 0
    const out: HopEdge[] = []
    const seenTargets = new Set<number>()
    for (let si = sources.length - 1; si >= 0 && !this.expired(deadline); si--) {
      const r = sources[si]
      if (this.stands(r.x, r.y, r.z).length === 0) continue
      const hR = heuristic(r.x, r.y, r.z)
      const cands: Array<[number, number, number, number]> = []
      for (let dx = -HOP_REACH; families !== 'bounce' && dx <= HOP_REACH; dx++) {
        for (let dz = -HOP_REACH; dz <= HOP_REACH; dz++) {
          if (dx * dx + dz * dz > HOP_REACH_SQ || (dx === 0 && dz === 0)) continue
          for (let dy = HOP_DROP; dy <= HOP_RISE; dy++) {
            const x = r.x + dx
            const y = r.y + dy
            const z = r.z + dz
            const h = heuristic(x, y, z)
            // Goal-side landings complete the route, so they bypass the retreat limit and are tried first.
            const side = goalSide !== null && goalSide(x, y, z)
            if (h > hR + MAX_REGRESS && !side) continue
            if (skip !== null && skip(x, y, z)) continue
            cands.push([x, y, z, side ? Math.hypot(dx, dz) + Math.max(0, dy) - GOAL_SIDE_FIRST : h])
          }
        }
      }
      const seenCand = new Set<number>(cands.map(([x, y, z]) => ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)))
      for (const [sx, sy, sz] of this.slimeTops(r.x, r.y, r.z)) {
        for (let dx = -BOUNCE_REACH; dx <= BOUNCE_REACH; dx++) {
          for (let dz = -BOUNCE_REACH; dz <= BOUNCE_REACH; dz++) {
            if (dx * dx + dz * dz > BOUNCE_REACH * BOUNCE_REACH) continue
            for (let y = sy + 1; y <= r.y + HOP_RISE; y++) {
              const x = sx + dx
              const z = sz + dz
              const key = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
              if (seenCand.has(key)) continue
              seenCand.add(key)
              const h = heuristic(x, y, z)
              if (h > hR + MAX_REGRESS) continue
              if (skip !== null && skip(x, y, z)) continue
              cands.push([x, y, z, h])
            }
          }
        }
      }
      // Deep slime candidates use deep bounces only, with reach computed for each landing height.
      // Try each column's nearest top once per pool level.
      const deepCand = new Set<number>()
      const deepTops = families === 'simple' || families === 'bounce' || families === 'all' ? this.deepSlimeTops(r.x, r.y, r.z) : []
      const levels = new Set(deepTops.map(t => t[1]))
      for (const sy of levels) {
        if (this.expired(deadline)) break
        const level = deepTops.filter(t => t[1] === sy)
        const drop = r.y - (sy + 1)
        const RR = Math.ceil(Math.max(BOUNCE_REACH, deepReach(drop, 0)))
        let x0 = Infinity; let x1 = -Infinity; let z0 = Infinity; let z1 = -Infinity
        for (const [sx, , sz] of level) { x0 = Math.min(x0, sx - RR); x1 = Math.max(x1, sx + RR); z0 = Math.min(z0, sz - RR); z1 = Math.max(z1, sz + RR) }
        const w = x1 - x0 + 1
        const near = new Float64Array(w * (z1 - z0 + 1)).fill(Infinity)
        for (const [sx, , sz] of level) {
          for (let dx = -RR; dx <= RR; dx++) {
            for (let dz = -RR; dz <= RR; dz++) {
              const d2 = dx * dx + dz * dz
              const i = (sz + dz - z0) * w + (sx + dx - x0)
              if (d2 <= RR * RR && d2 < near[i]) near[i] = d2
            }
          }
        }
        for (let y = sy + 1; y <= r.y + HOP_RISE; y++) {
          const rr2 = Math.max(BOUNCE_REACH, deepReach(drop, y - (sy + 1))) ** 2
          for (let i = 0; i < near.length; i++) {
            if (near[i] > rr2) continue
            const x = x0 + (i % w)
            const z = z0 + Math.floor(i / w)
            const key = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
            if (seenCand.has(key)) continue
            seenCand.add(key)
            const h = heuristic(x, y, z)
            if (h > hR + MAX_REGRESS) continue
            if (skip !== null && skip(x, y, z)) continue
            deepCand.add(key)
            cands.push([x, y, z, h])
          }
        }
      }
      cands.sort((p, q) => p[3] - q[3])
      let tried = 0
      let sideTried = 0
      // Only a slime rebound can reach a goal-side landing above the jump limit.
      const srcStands = this.stands(r.x, r.y, r.z)
      const srcTop = srcStands.length > 0 ? Math.max(...srcStands.map(p => p.y)) : r.y
      const rises = this.slimeTops(r.x, r.y, r.z).length === 0
      for (const [x, y, z] of cands) {
        if (this.expired(deadline)) break
        const key = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
        // Try goal-side landings from every take-off; reachability depends on the source.
        const sideT = goalSide !== null && goalSide(x, y, z)
        // Mark escalated targets seen only after landing; another source may reach a target this one cannot.
        const escalation = families === 'yaw' || families === 'yawKick' || families === 'pivot'
        if (!sideT && seenTargets.has(key)) continue
        const lands = this.stands(x, y, z)
        if (lands.length === 0) continue
        if (families === 'pivot' && (lands[0].catch || Math.min(spanOf(lands, 0), spanOf(lands, 1)) >= PIVOT_NARROW)) continue
        if (sideT) {
          if (rises && Math.min(...lands.map(p => p.y)) - srcTop > GOAL_SIDE_RISE) continue
          if (++sideTried > GOAL_SIDE_PER_SOURCE) continue
        }
        if (!escalation) seenTargets.add(key)
        if ((families === 'pivot' && ++tried > PIVOT_CANDIDATES) || (escalation && families !== 'pivot' && ++tried > YAW_CANDIDATES) || (families === 'beam' && ++tried > BEAM_CANDIDATES)) break
        // Follow the partial-path corridor to carry momentum through a chain.
        this.sideBridge = sideT
        let p: HopProgram | null
        try {
          p = this.hop([r.x, r.y, r.z], [x, y, z], deadline, deepCand.has(key) ? 'deep' : families, false, families === 'beam' ? sources.slice(si + 1) : [])
        } finally {
          this.sideBridge = false
        }
        if (p !== null && escalation) seenTargets.add(key)
        if (p !== null) out.push({ fromX: r.x, fromY: r.y, fromZ: r.z, program: p })
        if (out.length >= maxEdges) return out
      }
    }
    return out
  }
}

/** Unobstructed landing miss allowed before pruning, in blocks; collisions can deflect a grazing flight. */
const BALLISTIC_MISS = 1

/** Kernel ticks quickestRobust may still spend verifying found programs once a search is out of budget. */
const QR_GRACE_TICKS = 8000

/** Which program families a search flies (HopOracle.hop). */
export type HopFamilies = 'simple' | 'yaw' | 'yawKick' | 'all' | 'beam' | 'bounce' | 'kick' | 'pivot' | 'deep'

/** Horizontal hop range (blocks) and its square: a 6.5 disc covers every vanilla gap jump. */
const HOP_REACH = 6
const HOP_REACH_SQ = 6.5 * 6.5
/** Maximum horizontal reach of one hop or beam flight, in blocks. */
export const ONE_FLIGHT_REACH = 6.5
/** Landing heights relative to the take-off node. */
const HOP_DROP = -6
const HOP_RISE = 2
/** Allowed heuristic retreat from the goal; a useful hop can lead away before its follow-up finds progress. */
const MAX_REGRESS = 3
/** Sort bonus that puts goal-side landings (hopPipeline.ts goalSide) ahead of every other candidate. */
const GOAL_SIDE_FIRST = 1e6
/** Goal-side rise limit above the take-off stand, just beyond the 1.2522-block jump apex.
 * Try at most GOAL_SIDE_PER_SOURCE landings per take-off, nearest first. */
const GOAL_SIDE_RISE = 1.26
const GOAL_SIDE_PER_SOURCE = 8
/** Edges one discover() returns at most. */
const MAX_EDGES = 4
/** Candidate landings per source the yaw escalation flies, and the beam escalation searches (discover). */
const YAW_CANDIDATES = 12
/** A deep bounce's take-off with and without the rebound jump (search()). */
const REBOUND_JUMPS = [false, true]
const NO_REBOUND_JUMP = [false]
/** The climb-on: how far off the column's centre a body caught on the curtain is pressed back (advance()). */
export const CLIMB_ON_SLACK = 0.12
/** A slime touchdown this far inside the pool at least (commitGate, abortsOnSlime). */
const SLIME_EDGE = 1.5
/** Next-leg ranking (hopFrom next): programs within NEXT_SLACK ticks of the quickest, NEXT_COLLECT kept a search, NEXT_CANDS distinct touchdowns weighed. */
const NEXT_SLACK = 1
/** Horizontal speed below which a touchdown is pinned and worth ranking by its next leg. */
const NEXT_PINNED = 0.12
const NEXT_ALONG = typeof process === 'undefined' || process.env.PF_NEXT_ALONG !== '0'
const NEXT_COLLECT = 6
const NEXT_CANDS = 4
/** Per-leg and total kernel-work budgets for simple next-leg searches. */
const NEXT_LEG_TICKS = 10_000
const NEXT_TOTAL_TICKS = 40_000
const NEXT_FAMILIES: readonly Family[] = ['straight', 'run-turn', 'air-turn']
/** Two-leg selection limits: near-quickest candidates, distinct touchdowns and per-round work budgets.
 * CHAIN_PICK_LEAST bounds the next-leg time so ranking can stop when no later touchdown can win. */
const CHAIN_YAW_FAMILY: readonly Family[] = ['yaw']
const CHAIN_PICK_SLACK = 2
const CHAIN_PICK_COLLECT = 12
const CHAIN_PICK_CANDS = 24
const CHAIN_SEARCH_TICKS = 60_000
const CHAIN_PICK_TICKS = 110_000
const CHAIN_PICK_LEAST = 5
/** Next legs the first round leaves room for the yaw round to weigh (hopChain's reserve). */
const CHAIN_YAW_LEGS = 3
/** The committed window's disturbances (commitGate): position COMMIT_POS each way on x and z on entering it, velocity COMMIT_VEL each way on x, y and z at every tick. */
const COMMIT_POS = 0.5
const COMMIT_VEL = 0.05
const COMMIT_KICKS: ReadonlyArray<[number, number, number, number, number]> = [
  [COMMIT_POS, 0, 0, 0, 0], [-COMMIT_POS, 0, 0, 0, 0], [0, COMMIT_POS, 0, 0, 0], [0, -COMMIT_POS, 0, 0, 0],
  [0, 0, COMMIT_VEL, 0, 0], [0, 0, -COMMIT_VEL, 0, 0], [0, 0, 0, COMMIT_VEL, 0], [0, 0, 0, -COMMIT_VEL, 0],
  [0, 0, 0, 0, COMMIT_VEL], [0, 0, 0, 0, -COMMIT_VEL]
]
const BEAM_CANDIDATES = 4
/** Narrow landing candidates per source for the expensive pivot search. */
const PIVOT_CANDIDATES = 2
/** Exact line-up limits: coast ticks, yaw stride and spread, approximate press distance in blocks,
 * rest acceptance distance and verification candidate count. */
const LINEUP_COAST = 60
const LINEUP_STRIDE = 4
const LINEUP_SPREAD = 12
const LINEUP_PRESS = 0.059
const LINEUP_ACCEPT = 0.01
const LINEUP_VERIFY = 6
/** Enables moving line-up checks of chain controls while reserving work for other prefixes. */
const MOVING_LINEUP = typeof process === 'undefined' || process.env.PF_MOVING_LINEUP !== '0'
const LINEUP_ANCHOR_TICKS = 4000
/** How many nodes back a carry chain may start (chainStart), and the beam depth it gets per tick of its corridor at chain speed (chain). */
const CHAIN_BACK = 16
const CHAIN_SLACK = 1.4
