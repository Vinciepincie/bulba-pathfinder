// Physics-verified hops (improvement, allowParkourPhysics): the jumps the
// table model cannot express, found by flying them in the exact kernel
// (src/playerSim.ts, bit-identical to prismarine-physics) against the exact
// collision boxes of the snapshot.
//
// The table (parkourTable.ts) plans a jump from a handful of numbers per cell
// — flags, a height byte, a catch class, a thin footprint — and a fixed set
// of flight lines per offset. That is fast and it covers ordinary parkour,
// but a line through a 2/16 pane post needs an offset the set does not have,
// a jump round an L-wall needs a flight that turns, and a run along a ledge
// that turns into the jump at the lip is no straight line at all. Here the
// question "can a body get from this stand to that one" is answered by the
// physics itself.
//
// A CONTROL PROGRAM is three phases, each steered by a heading — toward a
// point (re-aimed every tick) or along a fixed direction: the RUN on the
// ground until the jump fires (pressed on tick `jumpAt`), the first air
// phase for `turnAir` air ticks (or until a slime rebound), the second air
// phase to touchdown. Forward and sprint are held throughout. Every jump a
// player makes between two stands is some program of this shape; the search
// tries them in families, simplest first:
//
//   straight  one point for all three phases (run at it, jump, keep aiming)
//   run-turn  run along an axis, then aim at the landing
//   air-turn  aim at a waypoint beside the line, turn to the landing mid-air
//   yaw       fixed headings off the bearing: run and first air phase on one,
//             turn onto the other (the neo: out along the pillar's face,
//             back in behind it — the executor's old runtime script search)
//   bounce    at a slime top below until the rebound, then at the landing
//             (a drop into a slime pit and up onto a ladder)
//
// A program is accepted only if it lands and STAYS (eight ticks of letting go)
// and it still does so from start positions nudged ROBUST_NUDGE either way —
// the tolerance the executor's line-up is given (plugin.ts program branch).
// Where no family lands, a beam over per-tick controls (beamSearch). The
// whole design, and what it measured: docs/PhysicsHops.md.

import { PlayerSim, newBody, copyBody, SIM, SIM_CLIMBABLE, SIM_SLIME, SIM_LIQUID, SIM_HAZARD, isVanillaJump } from './playerSim.js'
import { fallDamagePoints, SAFE_FALL } from './fallDamage.js'
import type { SimBody, SimControl, SimWorld } from './playerSim.js'

/** How one phase of a program is steered: at a point, re-aimed every tick, or along a fixed unit direction. */
export interface Heading {
  fixed: boolean
  x: number
  z: number
}

/** A control program: what the executor replays, tick for tick, from rest at (sx, sy, sz) — or, a LIVE one, from exactly `start`. */
export interface HopProgram {
  sx: number
  sy: number
  sz: number
  /** A live program's exact first state (the body where the executor stood when it was found); absent: from rest at the start point. */
  start?: SimBody
  /** A live program's first yaw: its headings are then the ones bot.look turns to (HopOracle.face). */
  yaw0?: number
  /** mcc-6-2: make room at the flush rest before the suffix runs and jumps. Forward held, at most four ticks. */
  groundPrefix?: Array<{ yaw: number, sprint: boolean, sneak: boolean }>
  /** A beam program's controls, tick for tick: [yaw to look at (NaN: none), key flags (F_FWD | F_SPRINT | F_JUMP)] pairs; the phases above are then unused. */
  seq?: number[]
  /** Touch and go: a landing short of the node starts the next leg (a beam program's chain of jumps) instead of ending the flight. */
  legs?: boolean
  /** Heading on the ground, until the jump fires. */
  run: Heading
  /** Tick (from the start) the jump is pressed on; held until the body leaves the ground. */
  jumpAt: number
  /**
   * The jump is pressed on the LATCHED tick instead: the first tick, from
   * `jumpAt` on, that starts on the ground with nothing under the box
   * (PlayerSim.carried) — the run's Y move went first, against the edge it
   * was leaving. The last tick a jump can be taken off a lip, and an event,
   * not a count: a start a few hundredths along the run gets there a tick
   * sooner or later and still jumps from it (the 'leave' family).
   */
  jumpOnLeave?: boolean
  /** Heading for the first `turnAir` air ticks (or until a slime rebound). */
  air1: Heading
  /**
   * A KICK: the jump is pressed already facing `air1` (from tick `jumpAt`
   * on), so the sprint-jump boost goes along it, not along the run - a run
   * along a strip's axis turned into its jump at the lip (arena mcc-8-2's
   * hanger pipes: a body on the far side of a post from its arm, kicked out
   * past the next post's other side).
   */
  kick?: boolean
  /**
   * A PRECISION program (the pivot family): it lands only from within about
   * a hundredth of its start, so it is held to LIVE_NUDGE and flown only from
   * the exact line-up (HopOracle.lineUp), never from a walking one.
   */
  precise?: boolean
  turnAir: number
  /** Heading after the turn, to touchdown. */
  air2: Heading
  /**
   * A bounce CHAIN: the heading held after the first rebound until the
   * second (at the next slime top), and so on; `air2` after the last. Absent:
   * `air2` from the first rebound on.
   */
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
  /**
   * Taken off from a climbable: the body hangs on a ladder or a vine, the
   * run is the push off it (horizontal speed clamped to the ladder's 0.15,
   * jump held climbs), and the program has fired once the body is off it and
   * airborne.
   */
  climb?: boolean
  /** The landing is a slime top: sneak on the touchdown tick (sneakLanding), or the block throws the body back up. */
  slimeLand?: boolean
  /**
   * A DEEP bounce (deepSlimeTops: slime more than BOUNCE_DROP below the
   * take-off, a fall any miss of which kills): flown with the climb-on, gated
   * by its committed window (commitGate), and re-verified every tick live.
   */
  deep?: boolean
  /** Jump held on the grounded tick after a rebound: vanilla keeps the rebound and adds the sprint boost (playerSim.ts isVanillaJump). */
  reboundJump?: boolean
  /** A deep bounce's abort targets (commitGate): the pool's slime tops a block and a half inside it, steered at by the executor's basin (plugin.ts). */
  basin?: Array<[number, number, number]>
  /**
   * A REGION landing (stitchHops on a node whose stands fall apart): the
   * rest the program was planned to come to. A landing counts only in that
   * rest's region of the node — the stands a body walks to from it
   * (HopOracle.regionOf) — not anywhere nodeY files under the node: beside a
   * pane rail, the floor east of it, the rail's top and the floor west of it
   * are one planner node and three places.
   */
  land?: { x: number, y: number, z: number }
}

/**
 * Sneak on this tick? A program landing on a slime top presses it on the
 * touchdown tick only — prismarine's moveEntity reverses a vertical collision
 * on slime unless sneaking, and sneak held any longer costs 70% of the air
 * control. Decided from the state the tick starts in (the kernel and the
 * executor both have it): in the air, coming down, the feet reaching the
 * landing's top this tick, the box over the landing column — Y moves first,
 * from where the tick starts.
 */
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

/** A place on a node: stands a body walks between (HopOracle.regions). `under`: beside a thin block in the cell below, where the planner never stands. */
export interface Region { stands: Stand[], y: number, under: boolean }

/** Start-position nudge every accepted program must survive, per axis (blocks). */
export const ROBUST_NUDGE = 0.03
/**
 * ...and a LIVE program's: its replay starts from the very state it was
 * found from and turns the very yaws (HopOracle.face), so only a model's
 * last hair of mismatch is left to cover.
 */
export const LIVE_NUDGE = 0.01
const LIVE_NUDGE_SAFE = process.env.PF_LIVE_NUDGE_SAFE !== '0'
// mcc-6-2's second neo needs room before its ordinary live flight.
export const LIVE_PREFIX = process.env.PF_LIVE_PREFIX !== '0'
export const LIVE_PREFIX_TICKS = 80_000
const PREFIX_EXPIRED = Symbol('prefix budget')
/**
 * A goal-side bridge (discover's goalSide) is held to this nudge: it is a hop
 * the pipeline would not try without the exemption, and the ones it opens
 * are over voids (arena opk2-1's slot drop holds at 0.06 by an air-turn 13t;
 * a 6-block ice jump on mcc-7-2 that 14 of all brute-force starts land does not).
 */
const SIDE_NUDGE = 0.06
/** Letting go after touchdown: ticks the body must stay on the landing node. */
export const STAY_TICKS = 8
/**
 * A region program down over its column off its place's level settles into it:
 * the executor no-keys sneaks there (plugin.ts), and the program's ticks run on to
 * the level. Priced at the touchdown, mcc-6-1's 3t bonk onto a stair top (11 to its
 * place) beat a 7t walk-off that was there first. PF_REGION_SETTLE=0 off.
 */
export const REGION_SETTLE = process.env.PF_REGION_SETTLE !== '0'
/** PF_NEIGHBOUR_CARRY=0: a landing must be carried by its own column's boxes (A/B of HopOracle.carriedAt). */
const NEIGHBOUR_CARRY = process.env.PF_NEIGHBOUR_CARRY !== '0'
/** Feet heights in a climbable's cell (above its floor) a planned take-off from it hangs at (padsAt). */
const HANG_HEIGHTS = [0.1, 0.6]

/** LutFlags bits (types.ts) the node convention needs. */
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

/** The heading a program steers by, given its phase counters (HopOracle.fly's order). */
export function programHeading (p: HopProgram, fired: boolean, air: number, rebounds: number, t: number): Heading {
  // (`t` the program's tick, required: a kick replayed with the boost along the run falls)
  if (!fired) return p.kick === true && t >= p.jumpAt ? p.air1 : p.run
  if (rebounds === 0) return air < p.turnAir ? p.air1 : p.air2
  // after rebound k: the chain's k-th heading (at the next slime), then the landing's
  return p.bounces !== undefined && rebounds <= p.bounces.length ? p.bounces[rebounds - 1] : p.air2
}

const point = (x: number, z: number): Heading => ({ fixed: false, x, z })
const dir = (x: number, z: number): Heading => ({ fixed: true, x, z })

/**
 * The programs of one family that share a run heading, as a tree: the run
 * (and the first air phase, which keeps the run's heading) branches on the
 * jump tick, the flight on the turn tick, the turn on the second heading.
 * Every value list is ascending where the tree walks it incrementally
 * (jumpAts, turnAirs).
 */
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

// Straight and run-turn turn onto their second heading at take-off (turnAir
// 0); straight's second heading is its first.
const NO_TURN = [0]
const STRAIGHT_JUMP_AT = [0, 1, 2, 3, 4, 5, 6, 8, 10]
const RUN_TURN_JUMP_AT = [1, 2, 3, 4, 5, 6, 8]
const AXES8: ReadonlyArray<[number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1], [Math.SQRT1_2, Math.SQRT1_2], [Math.SQRT1_2, -Math.SQRT1_2], [-Math.SQRT1_2, Math.SQRT1_2], [-Math.SQRT1_2, -Math.SQRT1_2]]
const AIR_TURN_JUMP_AT = [0, 2, 4, 6]
/** The leave family (HopProgram.jumpOnLeave): armed from the start, turned at take-off or a few air ticks on. */
const LEAVE_AT = [0]
const LEAVE_TURN = [0, 2, 4]
const AIR_TURN_TICKS = [2, 4, 6, 8]
// The yaw family's grid (radians off the bearing to the landing), measured on
// a one-block neo: the run along the pillar's face goes out 0.15-0.9, the
// turn comes back 0.3-1.5 within the first few air ticks, and the jump is
// taken after a run of up to 14 ticks. Gentlest first.
const YAW_RUN = [0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6, 0.9, -0.9]
const YAW_TURN = [0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2, 1.5, -1.5]
const YAW_TURN_TICKS = [1, 2, 3, 4, 6, 8]
const YAW_JUMP_AT = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14]
// The kick family's grid: runs along the world axes within 60 degrees of the
// bearing (KICK_AXIS_COS), leaning toward the landing's side (radians), kicks
// off the axis toward it, every jump tick to the end of a four-block run (the
// strip's lip is wherever the run leaves it), the turns after. Measured on
// mcc-8-2: kicks of 0.35-1.22 with leans of 0-0.26 all still plan it.
const KICK_AXES: ReadonlyArray<[number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1]]
const KICK_AXIS_COS = 0.5
const KICK_LEAN = [0, 0.09, 0.17, 0.26]
const KICK_OUT = [0.35, 0.52, 0.7, 0.87, 1.05]
const KICK_JUMP_AT = Array.from({ length: 17 }, (_, i) => i)
const KICK_TURN_TICKS = [1, 2, 3, 4, 5, 6, 8]
// mcc-8-2: a body beside a kick's lane needs to steer back onto it.
// PF_LANE_KICK=0: the live search's old families and budgets (A/B).
export const LANE_KICK_TICKS = process.env.PF_LANE_KICK === '0' ? 0 : 40_000
// The pivot family (HopProgram.precise): a run along a world axis toward the
// landing (a strip or a rail runs along one), the jump pressed facing
// PIVOT_JUMP off the bearing (at least PIVOT_MIN off the run: the sprint-jump
// boost goes along the facing, HopProgram.kick's replay), then turned onto
// PIVOT_TURN. For narrow landings alone (PIVOT_NARROW): arena mcc-7-1's 3/16
// trapdoor strips, whose jump faces some 80 degrees off the run.
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
// Slime bounces: aim at a slime top until the rebound, then at the landing.
// Slime tops within BOUNCE_REACH (horizontally) of both the take-off and
// the landing and up to BOUNCE_DROP below the take-off, the nearest
// BOUNCE_SLIMES by the two legs together.
const BOUNCE_REACH = 7
const BOUNCE_DROP = 14
const BOUNCE_SLIMES = 12
/** DEEP bounces (HopProgram.deep): slime more than BOUNCE_DROP below the take-off, down to this. */
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
/**
 * Bounce CHAINS, for a landing no single rebound reaches (HopProgram.bounces):
 * the first slimes tried, the next slimes tried from each rebound, and how
 * many slimes may follow the first.
 */
const BOUNCE_CHAIN_FIRST = 4
/**
 * How far (blocks, horizontally) one rebound off a slime top can carry a
 * body that came down `drop` above it, to a landing `rise` above it: the
 * fall from the take-off (a jump), the rebound flown back up to the
 * landing's height, and the air speed gained over those ticks from a running
 * 0.4 — an over-estimate by construction (air acceleration at its sprinting
 * best, a block of slack), -1 when the rebound does not reach that height.
 * BOUNCE_REACH is a fixed 7; a nine-block drop throws the body more than
 * nine, and a ladder 7.5-8.5 from the slime (arena mcc-3-1) was only ever
 * tried as a chain — at ten times the search cost, and never found.
 */
const reboundMemo = new Map<number, number>()
/** A deep bounce's reach: reboundReach, widened by the sprint boost a rebound jump adds where the bot's physics keeps the rebound (vanilla: 0.2 decaying 0.91 a tick, at most 2.3 blocks). */
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
/** advance(): fly to the next rebound (a finite mark no air count reaches). */
const TO_NEXT_REBOUND = Number.MAX_SAFE_INTEGER

const FAMILIES: Record<HopFamilies, readonly Family[]> = {
  simple: ['straight', 'run-turn', 'air-turn', 'bounce'],
  yaw: ['yaw'],
  // (the hop pipeline's escalation: the yaw family, then the kicks where it lands nothing)
  yawKick: ['yaw', 'kick'],
  kick: ['kick'],
  // (the pipeline's narrow-landing step alone: never live, where 0.5M ticks a strip is too slow)
  pivot: ['pivot'],
  // (the deep ring's landings, discover(): deep bounces alone, classes() deepOnly)
  deep: ['bounce'],
  // (the bounce family before the yaw family's grid, as in `simple`: it
  // costs nothing where no slime is in reach, and a live search that runs
  // out of time must not have spent it all on the neos first)
  all: ['straight', 'run-turn', 'air-turn', 'bounce', 'yaw', 'leave'],
  // (the beam alone: HopOracle.hop)
  beam: [],
  // (the slime ring alone: discover() from drop-edge take-offs, hopPipeline.ts dropTakeoffs)
  bounce: ['bounce']
}

/** A flight in progress: the body and the counters a program steers by (the executor keeps the same). */
/** `top`: the highest the body has been since its last support (the fall veto: tickFlight). */
interface Flight { b: SimBody, t: number, fired: boolean, air: number, rebounds: number, yaw: number, leg: number, top: number }

/** A first state the search flies from: a body at rest on a stand point, or a live one (`live` = its exact state). */
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
/**
 * mineflayer's bot.look step: a yaw change is rounded to whole steps of
 * 0.15° (conversions.fromNotchianPitch(0.15), computed the same way).
 */
export const LOOK_STEP = ((((Math.PI / 180) * -0.15 + Math.PI) % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2) - Math.PI

/** A state of the beam search, and the control that led to it. */
/** A state of the beam search — an ensemble of bodies, each flying or down (landing ticks) — and the control that led to it. */
interface BeamNode { fs: Flight[], done: number[], parent: number, want: number, flags: number, pad: number, score: number, wp: number }
/**
 * The beam (HopOracle.beamSearch): states kept per tick (of them, at most
 * BEAM_WIDTH_GROUND still before take-off — kept apart, or the first jumps
 * off crowd out every later one), ticks at most, headings off the bearing
 * to the landing, the speed a rank assumes to the landing before take-off
 * (sprint-jumping's, an optimistic bound like the airborne ranks' exact
 * landing tick), how far (blocks) a landing disc may miss the column and
 * live, and what a disc that holds the centre only at a stretch costs per
 * block.
 */
const BEAM_WIDTH = 160
const BEAM_WIDTH_GROUND = 60
export const BEAM_DEPTH = 80
const BEAM_OFFSETS = [0, 0.15, -0.15, 0.35, -0.35, 0.6, -0.6, 0.9, -0.9, 1.25, -1.25, 1.65, -1.65, 2.2, -2.2, Math.PI]
/** ...and the world axes, whatever the bearing: blocks line up on them, and a run along a rail or a ledge holds one exactly. */
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
/** PF_BEAM_DEBUG=1: one line per beam layer on stderr (diagnostic). */
const BEAM_DEBUG = typeof process !== 'undefined' && process.env.PF_BEAM_DEBUG === '1'
/**
 * The beam's detour field (DetourField): a beam with no corridor ranks a
 * body by the way round what stands between it and the landing column, not
 * the straight line through it. PF_BEAM_DETOUR=0: the straight line (A/B).
 */
const BEAM_DETOUR = typeof process === 'undefined' || process.env.PF_BEAM_DETOUR !== '0'
/**
 * Its grid step (blocks), the margin round the pads and the column, the most
 * blocks either side of the column, the clearance a body may jump over (a
 * box under the higher level plus this is no obstacle), the step between the
 * body heights tried, the most a detour counts (a point the grid cannot
 * reach: demoted, never dropped), and the least that counts at all.
 */
const DETOUR_STEP = 0.125
const DETOUR_MARGIN = 3
const DETOUR_HALF = 7
const DETOUR_CLEAR = 1
const DETOUR_DY = 0.5
const DETOUR_CAP = 6
const DETOUR_EPS = 0.05

/**
 * What a detour round the obstacles adds to the straight way to a landing
 * column, over a grid of points (HopOracle.detourField): at each point the
 * free-space octile geodesic to the column less the octile line, 0 outside
 * the grid. A point inside an obstacle (the body cannot stand there at any
 * height tried) reads the least of its free neighbours, or DETOUR_CAP.
 */
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

/**
 * A beam state's dedupe key. In the air: position to 1/16, velocity to 1/64
 * (vertical 1/32). Before take-off, position to 1/8 and velocity to 1/20: the
 * ground share is spread over PLACES — the corner a neo leaves from ranks
 * worse on distance than the spot pressed into the pillar, and fine velocity
 * variants of that one spot would otherwise fill the share — but not over
 * speed, which a chain across a stepping stone lives on.
 */
/**
 * The beam's merge class of a flight, as BEAM_KEY_LEN integers into `out` at
 * `at`: before take-off the ground position in eighths and the horizontal
 * velocity in twentieths; in the air the position in sixteenths and the
 * velocity (and whether it is down) - the same classes the string key had,
 * with no string built per child (beamSearch's hottest line).
 */
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
/** beamKeyInto's integers folded to one number (a Map key; equal tuples, equal hash - collisions compared exactly). */
function beamHash (k: Int32Array, at: number): number {
  let h = 0x811c9dc5
  for (let i = 0; i < BEAM_KEY_LEN; i++) h = Math.imul(h ^ k[at + i], 0x01000193)
  return h
}
/** Ticks jump may be held without taking off (a ceiling, a ladder) before the program is given up. */
const JUMP_GRACE = 6
/** climb2: a ceiling eats the rise, but the jump still re-arms its cooldown. PF_BONK_FIRE=0: velocity only. */
export const BONK_FIRE = process.env.PF_BONK_FIRE !== '0'
export function jumpRearmed (before: number | undefined, after: number | undefined, cooldown: number = SIM.autojumpCooldown): boolean {
  return BONK_FIRE && before !== undefined && after === cooldown && before < cooldown
}
/** Longest run on the ground, and longest flight, a program may have (ticks). */
const MAX_RUN = 40
/** Ticks a leave program may run before its body comes off the edge it runs at. */
const LEAVE_RUN = 20
const MAX_AIR = 60
/**
 * Rebounds that each buy MAX_AIR more air ticks, past a bounce chain's own
 * (tickFlight). Uncapped, a reboundJump flight trapped over slime never ran
 * out of air and advance() spun on the main thread. PF_REBOUND_CAP=0: uncapped.
 */
const REBOUND_CAP = process.env.PF_REBOUND_CAP !== '0'
const REBOUND_CREDIT = 4
/** A touchdown taken as the landing without a stay (goLanding, a chain twin's leg end) must not touch lava/fire: PF_TOUCH_WET=0 off. */
const TOUCH_WET = process.env.PF_TOUCH_WET !== '0'
/**
 * The most ticks any one flight may run (tickFlight), legs and rebounds and all.
 * Each leg of a touch-and-go flight restarts its own limits, so a beam ensemble
 * twin pinned under a climb2 roof with jump held (re-fired by BONK_FIRE on every
 * bonk) or at a wall flew for ever. The longest program seen is ~131t. PF_FLIGHT_CAP=0: none.
 */
export const FLIGHT_CAP = process.env.PF_FLIGHT_CAP !== '0'
export const FLIGHT_TICKS_MAX = 600
/** A bounce's first air phase lasts to the rebound: a finite mark past any flight, so advance() stops there. */
const TO_REBOUND = [MAX_AIR]

/** Air acceleration along the heading with forward held (prismarine: 0.98 input, +30% sprinting). */
const airAccel = (sprint: boolean): number => (SIM.airborneAcceleration + (sprint ? SIM.airborneAcceleration * 0.3 : 0)) * 0.98
/** Headings aimHeadings sweeps (a full turn), and how far inside the column a touchdown must be. */
const AIM_STEPS = 720
const AIM_EDGE = 0.001
/**
 * A landing is the body SUPPORTED by its node's block: the box over the
 * column by this much at least, the centre up to 0.2 outside it (what the
 * planner's shifted lines land on, Move.takeoff — 8-1's strip past a two-high
 * post has no landing with the centre inside at all).
 */
export const LAND_OVERLAP = 0.1
const AIM_COS = Float64Array.from({ length: AIM_STEPS }, (_, i) => Math.cos((i / AIM_STEPS) * 2 * Math.PI))
const AIM_SIN = Float64Array.from({ length: AIM_STEPS }, (_, i) => Math.sin((i / AIM_STEPS) * 2 * Math.PI))
/** Where across a landing arc the extra headings go, and how far (sweep steps) from the best one they must be. */
const AIM_SPREAD = [0.2, 0.8]
/** The search's reach test (HopOracle.canReach); PF_REACH_PRUNE=0 turns it off for A/B runs. */
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
  /** mcc-7-2's chain: a moving line-up checks its own controls on a bounded share, never a live search. */
  private anchorDeadline: number | null = null
  private prefixBudget: { end: number, deadline: number } | null = null
  /** The body's half-width (the kernel's). */
  get halfWidth (): number { return this.sim.halfWidth }
  /** Kernel ticks flown by the last discover() (diagnostics). */
  ticksFlown = 0
  /**
   * Did the last hop() give no verdict: stopped on its deadline, or had no
   * stand to land on or take off from (a landing the model does not know is
   * not a jump the physics refuses)? False: every program of every family
   * flew, and what came back is the answer.
   */
  noVerdict = false
  /** The last hop() had no stand to fly from or to: the oracle is blind here, which says nothing of the jump (noVerdict is set too). */
  noStand = false
  /**
   * A budget in kernel ticks, on top of the deadline (0: none). A live
   * search is stopped by its work, not by the clock: the same body in the
   * same place gets the same answer whatever else the machine is doing. By
   * the clock a search flew 30 000 ticks or 110 000 in its 40 ms, and the
   * one program that lands arena opk-6's second jump was found or missed by
   * a few flights — missed, the bot stood twelve seconds re-solving.
   */
  tickLimit = 0
  /** discover(): the hop being flown is a goal-side bridge (robust() holds it to SIDE_NUDGE). */
  private sideBridge = false
  /** The beam's detour field (BEAM_DETOUR; a test switches it per oracle). */
  beamDetour = BEAM_DETOUR

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

  /**
   * Is (x, y, z) at the edge of a drop: a cardinal neighbour with no box at
   * feet, head or below, and nothing to stand on there or a block lower — the
   * lip a body walks or jumps off (hopPipeline.ts dropTakeoffs)?
   */
  openDrop (x: number, y: number, z: number): boolean {
    for (const [ex, ez] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const nx = x + ex
      const nz = z + ez
      if (!this.shaped(nx, y, nz) && !this.shaped(nx, y + 1, nz) && !this.shaped(nx, y - 1, nz) &&
          this.stands(nx, y, nz).length === 0 && this.stands(nx, y - 1, nz).length === 0) return true
    }
    return false
  }

  /**
   * Does node (x, y, z) stand on ice (slipperiness 0.9 or more under it)?
   * A program from it starts from a line-up that slides (hopPipeline.ts).
   */
  onIce (x: number, y: number, z: number): boolean {
    const s = this.sim.stateAt(x, y - 1, z)
    return s >= 0 && this.world.slip[s] >= 0.9
  }

  /** Does the cell hold any collision box? */
  private shaped (x: number, y: number, z: number): boolean {
    const s = this.sim.stateAt(x, y, z)
    return s >= 0 && this.world.shapeStart[s] >= 0 && this.world.shapeCount[s] > 0
  }

  /**
   * The planner's node for a body resting with its feet at `feetY` in column
   * (x, z): the feet cell, one up when that cell holds a PHYSICAL block that
   * is not a thin floor — upstream puts a slab's or a dirt path's node on top
   * of the block, not in it.
   */
  nodeY (x: number, feetY: number, z: number): number {
    const fy = Math.floor(feetY + 1e-9)
    const f = this.flagsAt(x, fy, z)
    const physical = (f & F_PHYSICAL) !== 0
    const thinFloor = (f & (F_SAFE | F_PHYSICAL | F_CLIMBABLE)) === (F_SAFE | F_PHYSICAL)
    return physical && !thinFloor ? fy + 1 : fy
  }

  // mcc-6-2: stand discovery and landing holds share the retreat's hard allowance.
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
        // (nothing to stand on within reach of the node: open air, a pit)
        // Dropped from just above the feet cell's own boxes (a thin floor):
        // from any higher, a ceiling two blocks up (a hoop's top, a
        // headhitter) has the body start embedded in it.
        // ...and above what reaches up into it from the cell below: a fence
        // or a wall is a box 1.5 tall, its top half a block into the feet
        // cell, and a body dropped from the cell's floor started inside it —
        // no stand on any post (parkouradv1's fence posts: every take-off
        // search onto one came back empty, and the run-up line-up flew them)
        const drop = y + Math.max(this.cellTop(x, y, z), this.cellTop(x, y - 1, z) - 1) + 0.001
        const tryAt = (ox: number, oz: number): void => {
          const b = newBody(x + ox, drop, z + oz)
          // a body dropped INTO a pane or a panel falls through it and
          // "rests" embedded; start clear of everything
          if (this.sim.collides(b)) return
          if (!this.settle(b)) return
          if (Math.floor(b.x) !== x || Math.floor(b.z) !== z) return
          if (this.nodeY(x, b.y, z) !== y) return
          if (this.sim.collides(b) || !this.sim.supportedIn(b, x, z)) return
          if (!out.some(p => Math.abs(p.x - b.x) < 0.05 && Math.abs(p.z - b.z) < 0.05)) out.push({ x: b.x, y: b.y, z: b.z, catch: false })
        }
        for (const ox of [0.2, 0.5, 0.8]) for (const oz of [0.2, 0.5, 0.8]) tryAt(ox, oz)
        // and over the carrying cells' own boxes, by the box overlap a
        // landing needs (overCol): a thin ledge — an open trapdoor's top
        // edge is 3/16 wide, spiral3-b lands on one — meets no point of the
        // grid above
        const w = this.world
        const e = this.sim.halfWidth - LAND_OVERLAP
        for (const yy of [y - 1, y]) {
          const s = this.sim.stateAt(x, yy, z)
          if (s < 0 || w.shapeStart[s] < 0) continue
          for (let k = 0; k < w.shapeCount[s]; k++) {
            const o = (w.shapeStart[s] + k) * 6
            // (off each edge by the overlap margin, both ways: beside a ledge
            // a neighbour's box may leave a window a few hundredths wide)
            const x0 = w.boxes[o]; const x1 = w.boxes[o + 3]
            const z0 = w.boxes[o + 2]; const z1 = w.boxes[o + 5]
            for (const ox of [(x0 + x1) / 2, x0 - e, x0 + e, x1 - e, x1 + e]) {
              for (const oz of [(z0 + z1) / 2, z0 - e, z0 + e, z1 - e, z1 + e]) {
                if (ox > 0.01 && ox < 0.99 && oz > 0.01 && oz < 0.99) tryAt(ox, oz)
              }
            }
          }
        }
        // ...and BESIDE what stands in the feet and head cells, dropped from
        // the support's own top: a fence post on a plank strip leaves a
        // sliver the body can stand on, hanging over the strip's edge, and
        // every point of the grid above is inside the post or the fence over
        // it (arena opk-1: no stand at all on its first strip, so no hop to it)
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
        // (the offsets of every box's faces, crossed: a post with arms — a
        // cross of boxes — leaves only the cell's corners free)
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
        // (only over a support with a full top: off a fence or a post the
        // sliver is a perch no take-off leaves from — opk-6's fence arm, where
        // the next jump found nothing and the bot gave up and replanned)
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

  /**
   * Where the feet come down through height `landY`, flying from `b` with
   * heading `h` held (forward, sprint as `sprint`) and NOTHING in the way: the
   * kernel's air kinematics without collision — a dozen multiply-adds a
   * tick, no block reads. Null when the body is not coming down there within
   * MAX_AIR. Collisions only ever shorten or deflect a flight, so a program
   * whose unobstructed landing misses the node by more than BALLISTIC_MISS is
   * not worth flying on (the pre-screen).
   */
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

  /**
   * The fixed headings that bring the rest of this flight down in column
   * (tx, tz) and to REST near its centre, unobstructed. Held along a fixed
   * unit heading u, the flight's air kinematics are affine in u: the feet
   * come down (at the tick the vertical motion alone decides) at C + R·u,
   * and a body let go there slides on to C' + R'·u — two circles about the
   * drift points. So the headings that land are arcs of the first circle,
   * found by sweeping the angle the column spans; each arc gives a few
   * headings (below), and the kernel then flies them for real.
   */
  /**
   * Where an airborne body comes down through height `landY`, forward held on
   * any mix of headings and nothing in the way: at step s (the vertical
   * motion alone decides it) within r of the drift point (cx, cz) — on the
   * circle for a fixed heading, x_s = x0 + v0·sum + a·u·(sum + (s - sum)/(1 -
   * drag)), anywhere inside for a mix. [s, cx, cz, r, cx1, cz1, r1] in a
   * scratch array — the last three the disc a step earlier, where the
   * crossing tick begins — or null when the body is not coming down through
   * landY.
   */
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
    // and a step earlier: prismarine moves Y first, so what carries the
    // body is what is under it when the crossing tick BEGINS
    d[4] = b.x + b.vx * sum1
    d[5] = b.z + b.vz * sum1
    d[6] = a * (sum1 + (s - 1 - sum1) / (1 - drag))
    return d
  }

  /**
   * Can a flight from airborne `b` still come down in column (tx, tz) at
   * level `ty` or above? The most ground it covers before it has fallen
   * past that level — forward held on any headings — is the speed it carries
   * and its air acceleration, summed over the ticks the fall takes, and
   * nothing in the way adds to either. (PF_REACH_PRUNE=0 flies everything,
   * for A/B runs: the programs found are the same.) The level is search()'s
   * reachY, a step lower where a step-up can carry the body onto the landing.
   */
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
    // (to the column widened by what a box may hang over its edge: overCol)
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

  /**
   * Fixed headings that bring the rest of this flight down (unobstructed) on
   * each stand point the landing disc's circle passes near: toward it from
   * the drift point. A strip's window is narrower than the column
   * aimHeadings aims into (the pivot family). Appended to `out`.
   */
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
    // the slide: the landing tick's air drag, then ground friction on the
    // landing block to rest — velocity (drag · w_{s-1}) times 1/(1 - slip·0.91)
    const under = this.sim.stateAt(tx, Math.floor(landY - 1), tz)
    const slip = under >= 0 && this.world.slip[under] > 0 ? this.world.slip[under] : SIM.defaultSlipperiness
    const slide = drag / (1 - slip * SIM.airborneInertia)
    const q1 = Math.pow(drag, s - 1)
    // w_{s-1} = q1·(v0 + a·u) + a·u·(1 - q1)/(1 - drag)
    const rr = r + slide * a * (q1 + (1 - q1) / (1 - drag))
    const rcx = cx + slide * q1 * b.vx
    const rcz = cz + slide * q1 * b.vz
    // the circle meets the column only if its radius lies between the
    // nearest and the farthest the column comes to the drift point; then the
    // arcs lie within the angle the column spans from there
    // the column widened by what a box may hang over its edge (overCol)
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
    // per arc (a full turn is walked from just past a miss, so no arc wraps):
    // the touchdown furthest inside the column (what a start or heading
    // error eats into), the rest nearest the centre, and two more spread
    // across the arc — an obstacle (the pillar a neo turns round) cuts into
    // an arc from one side, and the kernel flight says which part is clear
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

  /**
   * A body HANGING on a climbable with its feet at (x, y, z): held there the
   * way the executor holds it — sneak and no keys, which on a ladder keeps
   * the body from sliding (prismarine clamps vy at 0) — one tick, so it
   * carries the hold's own velocity. Null if (x, y, z) is not on a climbable.
   */
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

  /**
   * The first states a hop from node (x, y, z) flies from: a body at rest on
   * each of its stand points, or — a climbable — hanging on it, pressed
   * against the ladder's face as a climb leaves it, at two heights in the
   * cell. (The executor takes off from wherever its climb has the body, by a
   * live search: these are what a planned hop is proved from.)
   */
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

  // mcc-6-2's neo lands flush against a wall: its inward twin starts inside
  // the wall and flies a path no live body can. Keep both sides if neither
  // is clear, and every twin of a start already overlapping a box.
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

  /**
   * Fly program `p` from its first state nudged by (nx, nz): ticks to a
   * landing that stays on (tx, ty, tz), or -1. `track`, when given, receives
   * the position after every tick up to touchdown. The reference the search
   * (and the executor's replay) must agree with.
   */
  fly (p: HopProgram, nx = 0, nz = 0, track: number[] | null = null): number {
    const f = this.origin(p, nx, nz)
    if (f === null) return -1
    const r = this.advance(f, p, Infinity, Infinity, track)
    return r === FLYING ? -1 : r
  }

  /**
   * THE flight loop: step flight `f` under program `p`, in place, until it
   * ends — ticks to a landing that stays, or -1 — or reaches its mark and
   * returns FLYING: on the ground at tick `groundUntil`, or once taken off,
   * at `airUntil` air ticks (a finite `airUntil` also marks a slime rebound,
   * where the program turns early). fly() runs it without marks; the search
   * runs it between marks, from copies, which is the same flight because the
   * kernel is deterministic.
   */
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
        // the pre-screen, as the program commits to its last heading
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
      // a deep bounce caught on a climbable in the target's column below it:
      // jump held climbs it (vanilla, prismarine's climbUsingJump, the kernel:
      // a free-hanging vine curtain has no wall to press into), steered at the
      // column's centre - its own keys there slide it off, 26 blocks down
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

  /**
   * Face heading (hx, hz): exactly — or, when the flight knows its yaw (a
   * live program), the way the executor's bot.look turns: mineflayer moves
   * the yaw in whole LOOK_STEPs from where it is, and prismarine takes the
   * heading from that yaw. So a live program flies the headings the replay
   * will, to the bit.
   */
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

  /**
   * One tick of a flight under the controls in this.ctl: the step, the
   * phase counters, and whether the flight has ended — ticks to a landing on
   * (tx, ty, tz) that stays, -1, or FLYING.
   */
  private tickFlight (f: Flight, p: HopProgram, track: number[] | null): number {
    const b = f.b
    const jumpTicks = b.jumpTicks
    this.kernelStep(b, this.ctl)
    if (track !== null) track.push(b.x, b.y, b.z)
    const t = f.t++
    if (b.inLiquid) return -1
    // (every flight ends: a touch-and-go leg pinned under a roof re-fires on each bonk - FLIGHT_CAP)
    if (FLIGHT_CAP && t >= FLIGHT_TICKS_MAX) return -1
    // a touchdown that costs FALL_VETO or more is no landing, whatever it lands on
    // (sneaking as the program presses it: onto slime that is the whole fall)
    if (b.onGround) {
      if (b.y < f.top - SAFE_FALL && fallDamagePoints(f.top - b.y, 0, this.kindAt(Math.floor(b.x), Math.floor(b.y - 0.2), Math.floor(b.z)), this.ctl.sneak, true) >= FALL_VETO) return -1
      f.top = b.y
    } else if (b.y > f.top || this.sim.onClimbable(b)) {
      f.top = b.y
    }
    if (!f.fired) {
      // (off a climbable, the take-off is the body leaving it: jump held
      // there only climbs, and a push off it never gets near 0.3 either way)
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
    // (each rebound buys more air, but only so many: a rebound-jump body boxed in over slime bounces for ever - REBOUND_CAP)
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
        // (a nudged twin of a chain only has to get this far: robust())
        if (this.reboundIsLanding && f.rebounds === 1) return TOUCH_WET && this.sim.touchesWet(b) ? -1 : t + 1
        return FLYING
      }
      if (!this.overCol(b, p.tx, p.tz) || this.nodeY(p.tx, b.y, p.tz) !== p.ty) {
        if (p.legs !== true) return -1
        // (a nudged twin of a chain only has to get this far — a real leg's
        // end, a block or more from its start, not a hop on the spot: robust())
        if (this.touchIsLanding && Math.hypot(b.x - p.sx, b.z - p.sz) >= LEG_MIN) return TOUCH_WET && this.sim.touchesWet(b) ? -1 : t + 1
        // touch and go: down short of the node, and the next leg starts here
        f.fired = false
        f.air = 0
        f.rebounds = 0
        f.leg = f.t
        return FLYING
      }
      // (a stone the plan goes on from: its touchdown is the landing, not a stop on it - hopChain)
      if (this.goLanding) {
        // (the kernel tests liquid and fire before it moves: a twin's last move into lava is seen only here - TOUCH_WET)
        if (TOUCH_WET && this.sim.touchesWet(b)) return -1
        if (this.touch !== null) copyBody(this.touch, b)
        return t + 1
      }
      const offLevel = this.offLevel(b, p)
      if (!this.stays(b, p)) return -1
      // (at its place only once settled there: the arrival the executor retires, not the touch)
      return offLevel ? t + 1 + (this.stayAt >= 0 ? this.stayAt + 1 : STAY_TICKS) : t + 1
    }
    if (b.y < Math.min(p.sy, p.ty) - 14 - (p.deep === true ? BOUNCE_DROP_LONG : 0)) return -1
    return FLYING
  }

  /**
   * The general search, for the jumps no program family flies — a neo off a
   * one-wide stair onto a pane rail, a slot threaded after a rebound: a beam
   * over per-tick controls in the exact kernel. Each tick either faces a
   * heading off the bearing to the landing (BEAM_OFFSETS) with forward and
   * sprint held, jumping or not while grounded, or coasts (no keys).
   *
   * Every state is an ENSEMBLE: the body, and the same body nudged LIVE_NUDGE
   * either way on each axis, all flown on the same controls (a beam program is
   * precision work — the executor re-derives it live from the exact rest
   * state rather than replay it from a line-up, so the live tolerance is the
   * one it has to meet). A state lives
   * while all of them do and lands when all of them have — so what the beam
   * finds is robust by construction, where a beam over the body alone finds
   * a precise jump's one fragile line first and nothing else.
   *
   * An airborne body is judged by its landing disc — held on any mix of
   * headings, it comes down within R of its drift point C (the flight is
   * affine in the headings: landingDisc) — so one whose disc misses the
   * landing column is dead, and a state ranks by its worst body's sooner
   * landing. Breadth-first by tick: the first layer that lands holds the
   * quickest programs, replayed tick for tick (HopProgram.seq).
   */
  private beamSearch (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], deadline: number, corridor: ReadonlyArray<{ x: number, y: number, z: number }> = [], land?: { x: number, y: number, z: number }, depthLimit = BEAM_DEPTH): HopProgram | null {
    if (pads.length === 0 || lands.length === 0) return null
    const catchTarget = lands[0].catch
    const landY = lands[0].y
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const c = this.ctl
    // the waypoints on the way (the planner's partial path, when a chain has
    // to follow it) and the landing last, with the length left after each
    const way: Array<[number, number, number]> = [...corridor.map(n => [n.x, n.y, n.z] as [number, number, number]), [t[0], landY, t[2]]]
    const tail = new Float64Array(way.length)
    for (let i = way.length - 2; i >= 0; i--) tail[i] = tail[i + 1] + Math.hypot(way[i + 1][0] - way[i][0], way[i + 1][2] - way[i][2])
    const reach = (b: SimBody, wp: number): number => {
      while (wp < way.length - 1 && Math.hypot(way[wp][0] + 0.5 - b.x, way[wp][2] + 0.5 - b.z) <= BEAM_WAYPOINT && Math.abs(way[wp][1] - b.y) < 1.5) wp++
      return wp
    }
    // what tickFlight reads the landing and the fall limit from
    const tpl: HopProgram = {
      sx: 0, sy: pads[0].f.b.y, sz: 0, run: point(tcx, tcz), jumpAt: Infinity, air1: point(tcx, tcz), turnAir: Infinity,
      air2: point(tcx, tcz), sprint: true, ticks: 0, tx: t[0], ty: t[1], tz: t[2], catchTarget, family: 'beam', legs: true,
      // (a source's pads are all of a kind: a climbable's hang, a floor's rests)
      climb: this.climbing(pads[0].f.b), slimeLand: this.slimeLanding(t, catchTarget), land
    }
    // (a beam to the landing alone ranks by the way round what is in it: BEAM_DETOUR)
    const field = corridor.length === 0 ? this.detourField(t, landY, pads) : null
    const nodes: BeamNode[] = []
    let layer: number[] = []
    pads.forEach((pad, i) => {
      // the ensemble, the way robust() will fly the program: its start, nudged
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
    // (merge classes: beamHash -> the children holding it, their tuples in keys)
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
        // (jump held on a climbable climbs: a choice before the push off it too)
        const grounded = n.fs.some((f, k) => n.done[k] === FLYING && (f.b.onGround || (tpl.climb === true && !f.fired && this.sim.onClimbable(f.b))))
        // headings off the bearing to the next waypoint
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
              // (a body down already, or let go: never stepped again, so shared, not copied)
              if (done[k] !== FLYING) { fs.push(n.fs[k]); continue } // landed already: the replay has moved on
              // past the end of the lead's first real leg a chain's twins are released (robust())
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
              // every body down on the node: the program, flown as robust() will
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
            // (the merge class: one child a class, the better-scored - exactly as by
            // the string key; a hash shared by different classes keeps them apart)
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
      // the best, at most BEAM_WIDTH_GROUND of them still before take-off...
      for (let i = 0; i < children.length && layer.length < BEAM_WIDTH; i++) {
        if (!children[i].fs[0].fired) {
          if (ground >= BEAM_WIDTH_GROUND) continue
          ground++
        }
        taken[i] = 1
        nodes.push(children[i])
        layer.push(nodes.length - 1)
      }
      // ...and that ground share whatever the airborne ranks: a later take-off is always an option
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

  /**
   * The beam's rank of a state: its time so far and the soonest it could be
   * down on the node (lower is better). Before take-off, and for a catch
   * (which may come on the way up): the distance at BEAM_SPEED. In the air,
   * the landing disc: the tick it comes down, and when the disc holds the
   * column nothing more (the nearer it holds the centre, the more room to
   * steer). When it does not, this leg comes down short and the rest is
   * another (touch and go): the distance the disc leaves, at BEAM_SPEED —
   * which is a jump chain's speed, so a leg in the air ranks as the progress
   * it is, not as a detour (a chain of ice pads is legs that each fall short
   * of the last, and a surcharge on them left the beam standing on the first).
   * `t` is the next waypoint when the beam follows a corridor, and `after` the
   * corridor's length beyond it. With a detour field (to the landing alone,
   * BEAM_DETOUR) the distance is the way round what is in it; and a body in
   * the air whose straight line is blocked ranks by that way, not by its
   * disc, which is blind to the obstacle the body is about to meet.
   */
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
      // falling, and already under the landing: it cannot get back up —
      // unless the column it drops down holds a slime top to rebound off
      if (b.vy < 0 && b.y < landY - 0.5 && !this.slimeUnder(b)) return Infinity
      return f.t + toGo / BEAM_SPEED + climb
    }
    const s = disc[0]
    const cx = disc[1]
    const cz = disc[2]
    const r = disc[3]
    // what carries the body is under it as the crossing tick begins (the
    // disc a step earlier), within its half-width of the column
    const cx1 = disc[4]
    const cz1 = disc[5]
    const r1 = disc[6]
    const hw = this.sim.halfWidth
    const nx = Math.max(t[0] - hw, Math.min(cx1, t[0] + 1 + hw))
    const nz = Math.max(t[2] - hw, Math.min(cz1, t[2] + 1 + hw))
    const gap = Math.hypot(nx - cx1, nz - cz1) - r1
    if (gap > BEAM_SLACK) {
      // short of the column: at best it comes down at the (earlier) disc's
      // point nearest it — a leg, if something there can carry the body,
      // and otherwise a fall with nothing under it
      const d = Math.hypot(tcx - cx1, tcz - cz1)
      const px = d > 1e-9 ? cx1 + ((tcx - cx1) / d) * r1 : cx1
      const pz = d > 1e-9 ? cz1 + ((tcz - cz1) / d) * r1 : cz1
      if (!this.footing(px, landY, pz)) return Infinity
      return f.t + s + (gap + after) / BEAM_SPEED
    }
    return f.t + s + after / BEAM_SPEED + BEAM_MISS_WEIGHT * Math.max(0, Math.hypot(tcx - cx, tcz - cz) - r)
  }

  /**
   * The beam's detour field to landing column `t` (DetourField), over a grid
   * round the pads and the column: a point is free when a body centred there
   * clears every collision box at some height from the lower of the pads and
   * the landing to the higher plus DETOUR_CLEAR (a one-high step or a slab is
   * jumped, a pane or a fence is not), and its detour is the octile geodesic
   * over free points to the column (widened by what a landing may hang over)
   * less the octile line. Null when no point is blocked, or no detour counts:
   * the beam ranks as it always has. The work is charged to ticksFlown.
   * Arena opk-2: two panes across a one-wide strip, where the straight line
   * ranked the bodies diving at the first pane as landing.
   */
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
    // the heights a point is blocked at (a box overlaps the body's there), counted
    const blocked = new Uint16Array(n)
    const mark = new Uint16Array(n)
    let heights = 0
    let any = false
    // (every DETOUR_DY from the lower level to the higher plus the clearance,
    // and the levels the bodies stand at: a landing under a head-hitter is
    // free at its floor alone)
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
              // the points strictly inside the box widened by the half-width
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
    // Dijkstra from the column (its points seeded at their octile to the centre)
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
    // (no free point to land at: the field has nothing to rank by)
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
          // (no corner cut between two blocked points)
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

  /**
   * The ticks a grounded body still needs to reach sprinting speed toward
   * (px, pz), at the acceleration of the block it is on (ice is slow to get
   * going): what a slow body pays over a fast one at the same place.
   */
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

  /** Could a body coming down at (x, z) through height `y` be carried by something — a box under its footprint within BEAM_FOOTING below? */
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

  /** After touchdown: let go (sneak, no keys) and check the body is still on the node. */
  /** The body's box over column (tx, tz) by LAND_OVERLAP at least (a catch is in its column: the caller's test). */
  private overCol (b: SimBody, tx: number, tz: number): boolean {
    const e = 0.5 + this.sim.halfWidth - LAND_OVERLAP
    return Math.abs(b.x - tx - 0.5) < e && Math.abs(b.z - tz - 0.5) < e
  }

  /** A region program down off its place's level, which the executor settles (plugin.ts): not a deep flight, not a beam's sequence. */
  private offLevel (b: SimBody, p: HopProgram): boolean {
    return REGION_SETTLE && p.land !== undefined && p.deep !== true && p.seq === undefined && Math.abs(b.y - p.land.y) >= 0.5
  }

  /** The stay tick the last stays() came down at its region's level (-1: not): REGION_SETTLE's price. */
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
        // (the next leg leaves from where the price says it arrived, not the stair top it slid off)
        if (off && this.touch !== null) copyBody(this.touch, b)
      }
    }
    c.sneak = false
    if (p.catchTarget) return Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz && b.y >= p.ty - 1.001 && b.y < p.ty + 1
    // (a landing that slid into liquid did not stay: the kernel freezes a
    // body at liquid contact, so it would "stay" in the lava beside the
    // ledge it came down on — arena opk-9)
    if (b.inLiquid) return false
    if (!this.overCol(b, p.tx, p.tz)) return false
    return b.onGround && this.nodeY(p.tx, b.y, p.tz) === p.ty && this.carriedAt(b, p.tx, p.ty, p.tz) &&
      (p.land === undefined || this.inRegion(b, p))
  }

  /**
   * Is a body at rest carried at node (tx, ty, tz): by its column's own boxes
   * (PlayerSim.supportedIn) - or centred over the column, on a neighbour's
   * box no more than a step above the node's own stands, with the column's
   * own footing under its box a step down at most (footingUnder: off the
   * neighbour it comes down on it). Arena mcc-8-2's strip of 3/16 panels runs
   * beside a closed trapdoor 0.19 higher: every hop onto the strip comes down
   * on the trapdoor by box overlap, centred over the strip's column, and
   * none was a landing - the bot stood, gave up and re-planned three times
   * (9 s) for the one cell its gait could reach. The executor arrives there
   * as it does (plugin.ts progLanded).
   */
  private carriedAt (b: SimBody, tx: number, ty: number, tz: number): boolean {
    if (this.sim.supportedIn(b, tx, tz)) return true
    if (!NEIGHBOUR_CARRY || Math.floor(b.x) !== tx || Math.floor(b.z) !== tz || !this.sim.carried(b)) return false
    let floor = Infinity
    for (const st of this.stands(tx, ty, tz)) if (!st.catch) floor = Math.min(floor, st.y)
    return b.y >= floor && b.y <= floor + SIM.stepHeight && this.footingUnder(b, tx, tz)
  }

  /**
   * Does a box of column (tx, tz) lie under body `b`'s box, its top at the
   * feet or up to a step below? (A rest on the take-off's own lip, centred
   * past a gap from the node's footing, has none: carriedAt.)
   */
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

  /**
   * Lands from the start AND from every nudged start: the tolerance the
   * executor's line-up gets — or LIVE_NUDGE, for a live program and for a
   * beam program, which the executor never replays from a line-up but
   * re-derives from the body's exact rest state (plugin.ts program branch).
   */
  private robust (p: HopProgram): number {
    const t0 = this.fly(p)
    if (t0 < 0) return -1
    // (a precise program is flown only from the exact line-up: HopOracle.lineUp)
    const n = p.start !== undefined || p.seq !== undefined || p.precise === true ? LIVE_NUDGE : this.sideBridge ? SIDE_NUDGE : ROBUST_NUDGE
    // a chain (touch and go) is held to its first touch-down: past it the
    // replay is the kernel's own flight to the bit, and twins nudged a
    // hundredth apart part ways over a few bounces as any two players would
    this.touchIsLanding = p.legs === true
    // a bounce chain likewise to its first rebound, off the slime it aims at
    this.reboundIsLanding = p.bounces !== undefined && p.bounces.length > 0
    try {
      for (const [nx, nz] of this.requiredNudges(p, n)) {
        if (this.fly(p, nx, nz) < 0) return -1
      }
    } finally {
      this.touchIsLanding = false
      this.reboundIsLanding = false
    }
    // (a deep bounce: its committed window too - inside robust(), so a
    // rejected program lets the search go on to a slower take-off)
    if (p.deep === true && !this.commitGate(p)) return -1
    return t0
  }

  /**
   * The abort a deep bounce's executor flies (plugin.ts 'basin'): steer at the
   * nearest of the pool's inside tops (HopProgram.basin), no sprint, jump or sneak, until down. Is
   * the first touchdown from flight state `f` (a copy) on slime, a block and
   * a half inside the pool?
   */
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

  /**
   * The committed window of deep bounce `p` (the review of arena opk2-2): the
   * ticks from the last one the executor's abort (abortsOnSlime) still lands
   * on slime to the landing. Live, every tick up to it re-flies the rest from
   * the body as it is (deepVerdict), so what the window must stand is what
   * arises inside it: at every one of its ticks the velocity kicked COMMIT_VEL
   * either way on each axis, and on entering it the body moved COMMIT_POS
   * either way on x and z, must still arrive on the program's own keys (half
   * a block at every tick fails every program on opk2-2: a teleport ticks
   * before a one-wide catch misses by construction). And the rebound must
   * come down a block and a half inside the pool. Scanned, not bisected: the
   * margin has holes.
   */
  private commitGate (p: HopProgram): boolean {
    const f = this.origin(p, 0, 0)
    if (f === null) return false
    // (steered at the pool's inside: the nearest top of all is its rim)
    const tops = this.deepSlimeTops(Math.floor(p.sx), Math.floor(p.sy + 1e-9), Math.floor(p.sz))
      .filter(t => this.inPool(t[0] + 0.5, t[1], t[2] + 0.5))
    if (tops.length === 0) return false
    // the flight, a state per tick (before it), to its landing
    const states: Flight[] = []
    let reboundAt = -1
    for (let i = 0; i < 2000; i++) {
      states.push(flightInto(newFlight(), f))
      const before = f.rebounds
      // (one tick a call: before the fire the air mark is 0, met on the fire tick itself)
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
    // every tick off the ground the abort no longer lands from holds against
    // the disturbances (the moves on entering such a stretch only: the
    // abortable ticks between are re-flown live from the body as it is)
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

  /**
   * A deep bounce's live check (plugin.ts, every tick of its replay): from
   * body `body` at the replay's counters, does the rest of program `p` still
   * arrive, and does the abort (abortsOnSlime, at HopProgram.basin) still
   * come down inside the pool? Both flown in copies.
   */
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

  /**
   * The slime tops a body taking off from cell (x, y, z) could come down on:
   * columns within BOUNCE_REACH whose first collision box below the take-off
   * (within BOUNCE_DROP) is a slime block. [x, y, z] of the block.
   */
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

  /**
   * The DEEP slime tops a body taking off from cell (x, y, z) could come down
   * on: columns within the fall's reach whose first collision box below the
   * take-off is a slime block more than BOUNCE_DROP down (to
   * BOUNCE_DROP_LONG; arena opk2-2's pool is 61 below its arch). Apart from
   * slimeTops: the chains and the shallow classes never see them.
   */
  deepSlimeTops (x: number, y: number, z: number): Array<[number, number, number]> {
    const k = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
    const hit = this.deepSlimeCache.get(k)
    if (hit !== undefined) return hit
    const out: Array<[number, number, number]> = []
    const R = Math.ceil(fallReach(BOUNCE_DROP_LONG))
    // (no floor but the drop's: a grid is a box in the worker but only a cache
    // window live, where the world goes on below it; unloaded cells read empty)
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
      // off a lip on the latched tick: at the landing (and its stands), or
      // along an axis, then aimed at the landing from the take-off on; and
      // the same turned a few air ticks later (a body coming off a strip
      // across its width wants its push turned onto the landing)
      const air2s = aims.slice(0, 3).map(a => point(a[0], a[1]))
      for (const sprint of [true, false]) {
        for (const a of aims) yield { run: point(a[0], a[1]), sprint, jumpAts: LEAVE_AT, turnAirs: LEAVE_TURN, air2s, aim: true, leave: true }
        for (const [ux, uz] of AXES8) yield { run: dir(ux, uz), sprint, jumpAts: LEAVE_AT, turnAirs: LEAVE_TURN, air2s, aim: true, leave: true }
      }
    } else if (family === 'kick') {
      if (hint !== undefined) {
        // mcc-8-2: a parallel run keeps the landing's sideways miss.
        // Aim past the plan's take-off, measured before its jump tick moves.
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
      // a run along a world axis (the strip a body is on), turned into the
      // jump at the lip: the jump pressed already facing KICK_OUT off the
      // axis toward the landing's side, held KICK_TURN_TICKS, then the
      // landing's; the run itself may lean KICK_LEAN toward that side
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
      // a run along a world axis toward the landing, the jump pressed facing
      // off it (replayed as a kick), then turned
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
      // the slime tops nearest by both legs; the first heading holds to the rebound
      const all = this.slimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z))
        .map(([x, y, z]) => [x, z, Math.hypot(x + 0.5 - st.x, z + 0.5 - st.z) + Math.hypot(tcx - x - 0.5, tcz - z - 0.5), y])
      // the tops whose rebound may carry to the landing: within BOUNCE_REACH,
      // or as far as the rebound off that top physically throws a body
      // (reboundReach) — a widening, never a narrowing of the fixed ring
      let tops = all.filter(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <=
        Math.max(BOUNCE_REACH, reboundReach(st.y - (t[3] + 1), ty - (t[3] + 1))))
      // no rebound from here carries to the landing: a chain (chainOn), off
      // the slimes nearest by both legs
      const chain = tops.length === 0
      // (where only the widened reach admits a top, the chains the fixed
      // ring would have flown come after its direct classes, as before)
      const chainsToo = !chain && !all.some(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <= BOUNCE_REACH)
      tops = (chain ? all : tops).sort((p, q) => p[2] - q[2]).slice(0, chain ? BOUNCE_CHAIN_FIRST : BOUNCE_SLIMES)
      const air2s = [point(tcx, tcz)]
      // at the slime's centre, and short of it along the way in: a take-off
      // that carries (a jump, a fall from a ledge's end) lands past a centre
      // it aims at, and one off the centre throws the rebound elsewhere —
      // a bounce is two landings, and the first one is a choice too
      // (the deep family: none of these)
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
      // DEEP bounces (deepSlimeTops), after every shallow and chain class
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
      // the aimed turns are the unobstructed landings; the grid adds the ones
      // a collision helps home (a slide along the pillar into the landing)
      const air2s = grid ? YAW_TURN.map(b => dir(Math.cos(bearing + b), Math.sin(bearing + b))) : []
      for (const sprint of walk ? [true, false] : [true]) {
        for (const a of YAW_RUN) {
          yield { run: dir(Math.cos(bearing + a), Math.sin(bearing + a)), sprint, jumpAts: YAW_JUMP_AT, turnAirs: YAW_TURN_TICKS, air2s, aim: true }
        }
      }
    }
  }

  /**
   * Every program of one family from these first states to node `t`, flown
   * as a tree — the run once per class, a copy per jump tick, the first air
   * phase once per jump tick, a copy per turn tick and second heading — then
   * the quickest robust one. A tick of the run is shared by every jump taken
   * after it and every flight after that, so the family costs its branches,
   * not its programs × their length.
   */
  private search (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], family: Family, deadline: number, walk: boolean, grid: boolean, land?: { x: number, y: number, z: number }, hint?: HopProgram, pick?: (found: HopProgram[]) => HopProgram | null): HopProgram | null {
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const catchTarget = lands[0].catch
    const landY = lands[0].y
    // (the reach test's height: the lowest stand of the target column - a
    // slab's, a closed trapdoor's or a pipe's node is a cell above its top,
    // a carpet's or a wall top's below it; the node's y pruned flights that
    // land on mcc-8-2's pipes, feet 279.19 under nodes at 280)
    let reachY = lands.reduce((m, s) => Math.min(m, s.y), Infinity)
    // (a step lower where a box a step under the landing stands in or beside
    // its column: a body coming down on it steps up onto the landing - arena
    // mcc-8-2, off a 3/16 panel up the pipe's 0.19; the hop the bot stood
    // nine seconds for. Not a catch: that holds the body in its column.)
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
      // a pivot is for a NARROW landing (a strip, a ledge, a rail): its stands lie on a line
      if (catchTarget || Math.min(spanOf(lands, 0), spanOf(lands, 1)) >= PIVOT_NARROW) return null
      // and runs along a narrow take-off's length (the body cannot run across it)
      const px = spanOf(pads.map(q => q.f.b), 0)
      const pz = spanOf(pads.map(q => q.f.b), 1)
      if (pz < PIVOT_NARROW && px >= PIVOT_NARROW) runAxes = [[1, 0], [-1, 0]]
      else if (px < PIVOT_NARROW && pz >= PIVOT_NARROW) runAxes = [[0, 1], [0, -1]]
    }
    let n = 0
    for (const pad of pads) {
      const st = pad.f.b
      const climb = this.climbing(st)
      // The reach test (canReach) holds for a flight that only falls: not
      // where slime may throw it back up, nor off a climbable it may regain.
      const falls = REACH_PRUNE && family !== 'bounce' && !climb && !slimeLand &&
        this.slimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z)).length === 0
      for (const cls of this.classes(family, st, aims, tcx, tcz, walk, grid, t[1], runAxes, hint)) {
        if ((++n & 3) === 0 && this.expired(deadline)) { this.noVerdict = true; return select() }
        // the run (no jump) and the first air phase (no turn) of this class
        const base: HopProgram = {
          sx: st.x, sy: st.y, sz: st.z, start: pad.live, yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw, run: cls.run, jumpAt: Infinity, air1: cls.kick ?? cls.run, turnAir: Infinity, air2: cls.kick ?? cls.run, kick: cls.kick !== undefined ? true : undefined, precise: family === 'pivot' ? true : undefined,
          sprint: cls.sprint, ticks: 0, tx: t[0], ty: t[1], tz: t[2], catchTarget, family, climb, slimeLand, land, deep: cls.deep === true ? true : undefined
        }
        flightInto(run, pad.f)
        for (const j of cls.jumpAts) {
          if (hint !== undefined && this.expired(deadline)) { this.noVerdict = true; return select() }
          if (this.advance(run, base, j, 0, null) !== FLYING) break
          // ran off an edge before tick j: no later jump tick is pressed either
          const walkedOff = run.fired
          const pj: HopProgram = cls.leave === true ? { ...base, jumpAt: j, jumpOnLeave: true } : { ...base, jumpAt: j }
          // (a take-off the landing is out of reach of: no turn brings it in)
          if (this.advance(flightInto(launch, run), pj, Infinity, 0, null) === FLYING &&
              (!falls || this.canReach(launch.b, cls.sprint, t[0], reachY, t[2]))) {
            flightInto(air, launch)
            for (const k of cls.turnAirs) {
              const ra = this.advance(air, pj, Infinity, k, null)
              if (ra !== FLYING) {
                // ended on the first heading: every later turn is this program
                if (ra >= 0) found.push({ ...pj, turnAir: k, ticks: ra })
                break
              }
              // flown out of reach on the first heading: so is every later turn
              if (falls && !this.canReach(air.b, cls.sprint, t[0], reachY, t[2])) break
              aimed.length = 0
              if (cls.aim) {
                if (catchTarget) aimed.push(point(tcx, tcz))
                else this.aimHeadings(air.b, cls.sprint, t[0], t[2], landY, aimed)
              }
              if (family === 'pivot' && !catchTarget) this.pointAims(air.b, cls.sprint, landY, lands, aimed)
              // (a deep bounce flown twice where the bot's physics keeps a rebound: and with the rebound jump)
              const rjs = cls.deep === true && air.rebounds > 0 && isVanillaJump() ? REBOUND_JUMPS : NO_REBOUND_JUMP
              for (let h = 0; h < cls.air2s.length + aimed.length; h++) {
                for (const rj of rjs) {
                  const pk: HopProgram = { ...pj, turnAir: k, air2: h < cls.air2s.length ? cls.air2s[h] : aimed[h - cls.air2s.length], reboundJump: rj ? true : undefined }
                  const rb = this.advance(flightInto(branch, air), pk, Infinity, Infinity, null)
                  if (rb >= 0) found.push({ ...pk, ticks: rb })
                }
              }
              if (cls.chain === true && air.rebounds === 1) this.chainOn(air, { ...pj, turnAir: k }, [], t, landY, catchTarget, cls.sprint, found, deadline)
              // a rebound turns every program with a later turn tick here
              if (air.rebounds > 0) break
            }
          }
          if (walkedOff) break
        }
      }
    }
    return select()
  }

  /**
   * A bounce chain on from rebound `at` (the flight of program `pj`, whose
   * chain so far is `chain`): at each slime top it could come down on next
   * (nextSlimes) until that rebound, then at the landing — the point and the
   * aimed headings, as at a first rebound — or, BOUNCE_CHAIN allowing, on to
   * another slime. Landings go to `found`. (A chain of rebounds loses little
   * height to drag, so a room of slime a drop starts is crossed rebound by
   * rebound: the arena's Ten Ways slime room, west wall to east.)
   */
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

  /**
   * The slime tops a body rebounding at `b` could come down on next: under
   * the apex of its rebound, within BOUNCE_REACH, not the one it is on — the
   * last of a chain within BOUNCE_REACH of the landing too — nearest by both
   * legs, the BOUNCE_CHAIN_NEXT best. [x, z] of each.
   */
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

  /**
   * The quickest of these landings that is also robust (ties: the first
   * found, the simplest). Past the budget the found ones are still checked,
   * for QR_GRACE_TICKS more: dropped unverified, a program found at 102k of
   * a 100k budget came and went with where the expiry check fell (arena
   * opk-6's yaw 15t; 2026-10-10 audit, the finds of a 200k budget at 100k).
   */
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

  /**
   * Does program `p`'s landing hurt (fallDamagePoints over the fall from the
   * top of its flight)? For one flight (singleFlight): a chain's legs each
   * start their own fall. False when it does not land.
   */
  landingHurts (p: HopProgram, whole = true): boolean {
    // (a catch hangs on its climbable: no landing, no fall)
    if (p.catchTarget) return false
    const track: number[] = []
    if (this.fly(p, 0, 0, track) < 0 || track.length < 6) return false
    let top = p.sy
    for (let i = 0; i < track.length; i += 3) {
      // (a climbable touched on the way ends the fall there, as the server counts it)
      if ((this.kindAt(Math.floor(track[i]), Math.floor(track[i + 1]), Math.floor(track[i + 2])) & SIM_CLIMBABLE) !== 0) top = track[i + 1]
      else top = Math.max(top, track[i + 1])
    }
    const x = track[track.length - 3]
    const y = track[track.length - 2]
    const z = track[track.length - 1]
    const kind = this.kindAt(Math.floor(x), Math.floor(y - 0.2), Math.floor(z))
    // (a slime landing is sneaked onto, as the replay does - sneakLanding - and takes the whole fall)
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

  /**
   * The quickest next leg's ticks from body `b` (a touchdown, moving) to node
   * `t`, the simple families, within NEXT_LEG_TICKS of kernel work; -1 if
   * none lands.
   */
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

  /**
   * Of take-off programs `cands` (the robust ones within NEXT_SLACK of the
   * quickest, `best`), the one whose landing leaves the quickest next leg to
   * node `next`: ticks plus that leg's (distinct touchdowns only, at most
   * NEXT_CANDS). The quickest stands unless another's total is strictly
   * less - one with no next leg found counts as none. Only where the
   * quickest comes down all but stopped (NEXT_PINNED: pinned on a post, a
   * wall): one that keeps its speed goes on as quick as any, and the
   * ranking's search work (a fifth to a half more per take-off) buys nothing.
   * The speed is the one along the way to `next`: arena opk-2's second neo
   * came down at 0.122, sideways, 0.057 of it toward the finish, and ran four
   * ticks to get going (PF_NEXT_ALONG=0: the whole speed, for A/B).
   */
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

  /**
   * A robust program from node `r` to node `t`, from rest at one of r's
   * stand points: the quickest of the first family that lands, or null.
   * `families`: the simple ones ('simple', what a discovery weighs every
   * candidate with), the yaw family alone ('yaw', for the jumps known to
   * need it), all of them, or the beam alone ('beam', beamSearch). Families go simplest first: a simpler program
   * that lands is worth more than a tick or two. `grid`: the yaw family
   * also turns onto a fixed grid of headings, not only the aimed ones (the
   * executor's take-off searches, one node at a time).
   */
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

  /**
   * Does program `p` come down on its own landing in one flight, robustly? A
   * touch-and-go program (HopProgram.legs: every beam program) holds its nudged
   * twins only to their first touchdown (robust), and the first touchdown may
   * be short of the node: a skip past a planned node must land past it, every
   * twin (plugin.ts SKIP_AHEAD).
   */
  singleFlight (p: HopProgram): boolean {
    if (p.legs !== true) return true
    return this.robust({ ...p, legs: false }) >= 0
  }

  /**
   * Program `p` re-anchored at body `b` as it is (yaw `yaw`): the same
   * controls, flown from here — the program, live, if it still lands robustly
   * (LIVE_NUDGE), else null. The executor's first resort at a lined-up rest:
   * a line-up leaves the body a few hundredths off the planned start, which
   * most programs shrug off, and then no search is needed at all.
   */
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

  /**
   * The exact line-up: one or two one-tick sneaking presses from body `b`
   * at rest (yaw `yaw`), each turned in whole LOOK_STEPs as bot.look turns,
   * coasted to rest, whose rest re-anchors precision program `p`
   * (anchored()). A planned beam chain or pivot lands only from within a
   * hundredth or so of its start, finer than a walking line-up places a body
   * (a press moves its rest ~0.06); the kernel places it to a few
   * thousandths. The presses' yaws, the rest they leave and the program
   * from it, or null.
   */
  lineUp (p: HopProgram, b: SimBody, yaw: number, deadline: number, moving?: { reach: number, tried: boolean }): { yaws: number[], delay?: number, rest: SimBody, prog: HopProgram } | null {
    if (moving !== undefined) {
      moving.tried = false
      return MOVING_LINEUP ? this.movingLineUp(p, b, yaw, deadline, moving) : null
    }
    const c = this.ctl
    const steps = Math.round((2 * Math.PI) / Math.abs(LOOK_STEP))
    // (the yaw after m more LOOK_STEPs from y: HopOracle.turnTo's arithmetic, so the replay's looks reproduce it to the bit)
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
    // one press, every LINEUP_STRIDE-th heading
    for (let k = 0; k < steps && !this.expired(deadline); k += LINEUP_STRIDE) {
      const y = turned(yaw, k)
      const q = copyBody(newBody(0, 0, 0), b)
      press(q, y)
      if (coast(q)) keep(q, [y])
    }
    // two presses: the second at the start from about where the first leaves the body
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
    // (the best one is always verified: a sweep that ran to the budget keeps its find)
    for (let i = 0; i < cands.length && i < LINEUP_VERIFY && (i === 0 || !this.expired(deadline)); i++) {
      const cd = cands[i]
      const q = this.anchored(p, cd.rest, cd.yaws[cd.yaws.length - 1])
      if (q !== null) return { yaws: cd.yaws, rest: cd.rest, prog: q }
    }
    return null
  }

  // mcc-7-2's chain need not stop before lining up: aim from the glide's rest.
  // 'tried' spends the approach's one search only once that rest is near;
  // the presses still start at b, not at the predicted rest.
  private movingLineUp (p: HopProgram, b: SimBody, yaw: number, deadline: number, moving: { reach: number, tried: boolean }): ReturnType<HopOracle['lineUp']> {
    const clone = (q: SimBody): SimBody => copyBody(newBody(0, 0, 0), q)
    // anchored() changes this.ctl; mcc-7-2's next coast must release every key.
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
      // mcc-8-2's panel can set onGround on the tick the box slides off it.
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
    // (the prefix held from the nudged bodies too, not the exact one alone: a
    // coast that skims a 3/16 panel's edge by less than LIVE_NUDGE is no line-up)
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
    // Leave work for the checks: mcc-7-2's long chain must fit the same 60k.
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

  // mcc-6-2: onGround alone can outlive the support, and a hazard is tested before the kernel moves.
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

  // mcc-6-2: retreat from the collision face, with a planned stand only a directional hint.
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

  /** mcc-6-2: a short supported retreat, then one ordinary jump from the body it leaves. */
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
      // mcc-6-2's known flight turns in the air; leave work for every endpoint and for verification.
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
            // mcc-6-2: verify the original twins, never new nudges at the retreat's endpoint.
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

  /**
   * A robust LIVE program to node `t` from body state `b` exactly as it is
   * (the executor's take-off, where no gate would fly the jump): every
   * family (the yaw family on the grid of turns too), walking jumps too, and
   * the quickest that lands — the body is already where the program starts,
   * so there is no line-up to weigh it against. `yaw`: the body's yaw now,
   * which makes the headings the ones the replay's bot.look will turn to
   * (NaN: unknown). When no family lands, the beam (beamSearch). Null if
   * nothing lands.
   */
  hopFrom (b: SimBody, t: [number, number, number], deadline: number, yaw = NaN, land?: { x: number, y: number, z: number }, next?: [number, number, number], beam = true,
    seed?: ReadonlyArray<'kick' | 'pivot'>, seedTicks = 0, seedFirst = true, hint?: HopProgram): HopProgram | null {
    // (a planned region landing's live search lands in that region: HopProgram.land)
    const lands = land !== undefined ? this.regionOf(t, land) : this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return null
    const live = copyBody(newBody(0, 0, 0), b)
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0, top: b.y }, live }]
    let best: HopProgram | null = null
    // (with the node after: the programs within NEXT_SLACK of each family's quickest, for the next-leg ranking)
    const cands: HopProgram[] | null = next !== undefined ? [] : null
    // Seed families (the kick and the pivot are no live families: FAMILIES.all)
    // on `seedTicks` of their own, in order until one lands, the families'
    // budget left as it was. From a body come down beside a kick or pivot
    // plan's line the live families find nothing in all their budget, and the
    // plan's own family flies it: arena mcc-7-1 a pivot 13t from the landing
    // (36 ticks to touchdown flown), mcc-8-2 a pivot 17t (28); a kick plan's
    // landing at mcc-8-2 has a pivot 23t and no kick. `seedFirst`: before the
    // families, else only where they find nothing.
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
      // mcc-8-2: try the plan's lane before spending the seed families' work.
      // The rollout and verification belong to this pass's own allowance.
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
    // (the beam where no family lands - unless the caller asked the families alone: plugin.ts SKIP_AHEAD)
    if (best === null) return beam ? this.beamSearch(pads, lands, t, deadline, [], land) : null
    // the landing that leaves the quickest next leg (arena opk-1: the
    // quickest 9t onto the first arm pins the body on its fence post, the
    // next leg from rest 15t; a 9t landing still moving goes on in 9)
    return cands !== null && next !== undefined ? this.nextLegPick(best, cands, next, deadline) : best
  }

  /**
   * The leg onto a stone the plan goes straight on from, picked for the leg
   * after it: a robust program from body `b` to stone `t` that touches down
   * over it and goes on (no stop on it: stays() is the wrong test, a stone this
   * thin stops nothing at the speed the next jump needs), the least total of
   * its ticks and the next leg's from that touchdown (the same search the
   * arrival will run, the kernel's flight being the body's to the bit). Null
   * if no touchdown has a next leg. Arena mcc-5-2, a skull to a 0.125 bar to a
   * skull: the jump past the bar needs a second ground tick, a quarter of the
   * tick phases; the gates' touchdown 0.09 further on had none, and 73 ticks
   * were lost on the bar. `nextGo`: the next leg's landing is a stone too.
   */
  hopChain (b: SimBody, t: [number, number, number], next: [number, number, number], deadline: number, yaw = NaN, nextGo = false): HopProgram | null {
    const lands = this.stands(t[0], t[1], t[2])
    // (no stand on the node after: no next leg to weigh a touchdown by)
    if (lands.length === 0 || this.stands(next[0], next[1], next[2]).length === 0) return null
    const live = copyBody(newBody(0, 0, 0), b)
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0, top: b.y }, live }]
    // (one budget, the caller's: the searches and the weighing each a share,
    // the first round leaving the yaw round enough to search and weigh a few)
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
        // (a next leg is never begun where it cannot end: one cut short is no verdict)
        if (weighed >= CHAIN_PICK_CANDS || this.ticksFlown + NEXT_LEG_TICKS > stop || performance.now() > deadline) break
        // (no touchdown a tick later can beat the total in hand on a next leg of a run and a flight)
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
      // (the yaw family, five times the work, where no simple touchdown has a
      // next leg - and there is room left for it to search and weigh some)
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

  // ── regions: the places one planner node holds ─────────────────────────

  private readonly underCache = new Map<number, Stand[]>()
  private readonly regionsCache = new Map<number, Region[]>()
  private readonly regionOfCache = new Map<string, Stand[]>()
  private readonly linkBody = newBody(0, 0, 0)

  /**
   * Rests BESIDE a thin block in the cell under the node (a pane rail, a
   * post): the feet on the floor a block down, filed under this node by
   * nodeY (the feet cell holds a physical block that is no floor), where the
   * planner never stands — it reads the node as the top of that block. Only
   * where no other column carries the body: there the rest is that column's
   * node, which the planner has.
   */
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

  /**
   * The places on node (x, y, z): its stands and underStands, joined where a
   * body walks between them (walkLink) — the floor either side of a pane
   * rail and the rail's top are three. A climbable's catch is one place.
   */
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
      // (under: made of underStands, beside the block; a rest ON a low block,
      // a bottom trapdoor or a flower pot's top, is the planner's own place)
      out.push({ stands: comp, y: comp[0].y, under: comp.some(s => s.under === true) })
    }
    this.regionsCache.set(k, out)
    return out
  }

  /**
   * Is a rest filed under node t (nodeY) BESIDE the block in the cell under
   * it, not on its top: the node has under places and the feet are below
   * that block's top (a bottom trapdoor's top is the planner's own place).
   */
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

  /**
   * Take-off pads in the region of a body at rest at `rest` on node `n`: the
   * rest itself, then the stands it walks to in a straight line (the
   * executor's line-up walks straight at a program's start).
   */
  padsFromRest (n: readonly [number, number, number], rest: { x: number, y: number, z: number }): Pad[] {
    const out: Pad[] = []
    for (const s of this.regionOf(n, rest)) {
      if (s.x !== rest.x || s.z !== rest.z) { if (!this.walkLink(rest, s.x, s.z)) continue }
      const f = this.pad(s.x, s.y, s.z)
      if (f !== null && !out.some(q => Math.abs(q.f.b.x - f.b.x) < 0.02 && Math.abs(q.f.b.z - f.b.z) < 0.02)) out.push({ f, live: undefined })
    }
    return out
  }

  /**
   * discover() from a hop's REST (stitchHops): take-offs from the rest and
   * the stands it walks to (padsFromRest), landings in each region of the
   * nodes in hop range — the rest's own node too, its other places — each
   * held to that region (HopProgram.land). `skipTop`: nodes whose planner
   * place (a region that is not under) is reached already.
   */
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

  /**
   * Does (x, y, z) look out on ground the failed search never reached: a
   * standable cell within hop range of it that discover() would try (not
   * `reached`, no more than MAX_REGRESS further from the goal)? The test
   * behind hopPipeline.ts frontierSources.
   */
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

  /**
   * Hops out of the failed search's reach: from each of `sources` (the tail
   * of the best partial path, nearest the goal last), to nodes within hop
   * range not already reached (`skip`) and no more than MAX_REGRESS further
   * from the goal, most promising first. Returns up to `maxEdges` edges
   * within `budgetMs`. `families`: the simple ones for every candidate, or
   * ('yaw') the yaw family for the YAW_CANDIDATES most promising per source —
   * the escalation when no simple program opens the way.
   */
  /**
   * Where a carry chain over ice starts: when `path` ENDS on ice (onIce), the
   * index of its last node on firm ground before that run (at most CHAIN_BACK
   * nodes back), else -1. Speed carried across ice is never stopped and taken
   * up again: a body at rest on an ice pad gets no run-up, and the gap the
   * run ends at may be crossed only on the carry of every leg before it
   * (arena mcc-7-2: nothing lands from rest on the last pads, a chain from
   * the stone before the first does).
   */
  chainStart (path: ReadonlyArray<{ x: number, y: number, z: number }>): number {
    const n = path.length
    if (n < 2 || !this.onIce(path[n - 1].x, path[n - 1].y, path[n - 1].z)) return -1
    for (let i = n - 2; i >= 0 && i >= n - 1 - CHAIN_BACK; i--) {
      if (!this.onIce(path[i].x, path[i].y, path[i].z)) return i
    }
    return -1
  }

  /**
   * Chains out of the failed search's reach: the beam from `from` along
   * `corridor` (the partial path after it) to the landings discover()
   * would try from the corridor's last node (hop range, not `skip`ped, no
   * more than MAX_REGRESS further from the goal), its depth the corridor's
   * length at chain speed with CHAIN_SLACK to spare.
   */
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
    // nearest the goal first: the best partial path ends at the closest node
    for (let si = sources.length - 1; si >= 0 && !this.expired(deadline); si--) {
      const r = sources[si]
      if (this.stands(r.x, r.y, r.z).length === 0) continue
      const hR = heuristic(r.x, r.y, r.z)
      const cands: Array<[number, number, number, number]> = []
      // (a bounce-only discovery flies the slime ring alone)
      for (let dx = -HOP_REACH; families !== 'bounce' && dx <= HOP_REACH; dx++) {
        for (let dz = -HOP_REACH; dz <= HOP_REACH; dz++) {
          if (dx * dx + dz * dz > HOP_REACH_SQ || (dx === 0 && dz === 0)) continue
          for (let dy = HOP_DROP; dy <= HOP_RISE; dy++) {
            const x = r.x + dx
            const y = r.y + dy
            const z = r.z + dz
            const h = heuristic(x, y, z)
            // (a cell the planner reaches the goal from is worth any regress, and is tried first, nearest first)
            const side = goalSide !== null && goalSide(x, y, z)
            if (h > hR + MAX_REGRESS && !side) continue
            if (skip !== null && skip(x, y, z)) continue
            cands.push([x, y, z, side ? Math.hypot(dx, dz) + Math.max(0, dy) - GOAL_SIDE_FIRST : h])
          }
        }
      }
      // and round the slime a drop from here could bounce off (the bounce family)
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
      // and round the deep slime (deep bounces): landings within the deep
      // rebound's reach, flown as deep bounces alone (deepCand: no other
      // family's landing 15-72 down is one). Per pool level, each column's
      // nearest top once, then the reach per landing height.
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
      // (a goal-side landing above a jump's reach is no hop at all — unless slime throws the body)
      const srcStands = this.stands(r.x, r.y, r.z)
      const srcTop = srcStands.length > 0 ? Math.max(...srcStands.map(p => p.y)) : r.y
      const rises = this.slimeTops(r.x, r.y, r.z).length === 0
      for (const [x, y, z] of cands) {
        if (this.expired(deadline)) break
        const key = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
        // (a goal-side landing is worth trying from every take-off: which one flies it is the question)
        const sideT = goalSide !== null && goalSide(x, y, z)
        // (an escalation family marks a target seen only once it lands it: the
        // first source to try one may be the one that cannot - mcc-8-2's tail
        // claimed every pipe target its arm side forbids, and the frontier
        // cell beside it that flies them was never asked)
        const escalation = families === 'yaw' || families === 'yawKick' || families === 'pivot'
        if (!sideT && seenTargets.has(key)) continue
        const lands = this.stands(x, y, z)
        if (lands.length === 0) continue
        // (the pivot family flies narrow landings alone)
        if (families === 'pivot' && (lands[0].catch || Math.min(spanOf(lands, 0), spanOf(lands, 1)) >= PIVOT_NARROW)) continue
        if (sideT) {
          if (rises && Math.min(...lands.map(p => p.y)) - srcTop > GOAL_SIDE_RISE) continue
          if (++sideTried > GOAL_SIDE_PER_SOURCE) continue
        }
        if (!escalation) seenTargets.add(key)
        if ((families === 'pivot' && ++tried > PIVOT_CANDIDATES) || (escalation && families !== 'pivot' && ++tried > YAW_CANDIDATES) || (families === 'beam' && ++tried > BEAM_CANDIDATES)) break
        // (the beam follows the partial path on from here: a chain carries its momentum along it)
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

/**
 * How far (blocks) outside the landing cell an unobstructed flight may come
 * down and still be flown out in full: collisions deflect a flight by a few
 * tenths at most (a graze, a corner slide), not by a block.
 */
const BALLISTIC_MISS = 1

/** Kernel ticks quickestRobust may still spend verifying found programs once a search is out of budget. */
const QR_GRACE_TICKS = 8000

/** Which program families a search flies (HopOracle.hop). */
export type HopFamilies = 'simple' | 'yaw' | 'yawKick' | 'all' | 'beam' | 'bounce' | 'kick' | 'pivot' | 'deep'

/** Horizontal hop range (blocks) and its square: a 6.5 disc covers every vanilla gap jump. */
const HOP_REACH = 6
const HOP_REACH_SQ = 6.5 * 6.5
/** The reach of one flight, a hop or a beam program (HOP_REACH_SQ's radius): a live search re-finds nothing farther (plugin.ts futileChain). */
export const ONE_FLIGHT_REACH = 6.5
/** Landing heights relative to the take-off node. */
const HOP_DROP = -6
const HOP_RISE = 2
/**
 * How much FURTHER from the goal (heuristic) a landing may be and still be
 * tried: courses wind, and the hop that opens the way (round an L-wall) can
 * lead away from the goal first. The caller judges a hop by what it opens
 * (a follow-up search), not by this.
 */
const MAX_REGRESS = 3
/** Sort bonus that puts goal-side landings (hopPipeline.ts goalSide) ahead of every other candidate. */
const GOAL_SIDE_FIRST = 1e6
/**
 * A goal-side landing more than this above the take-off's stand is out of a
 * jump's reach (apex 1.2522); and at most GOAL_SIDE_PER_SOURCE are flown per
 * take-off, nearest first.
 */
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
/**
 * A touchdown costing this much fall damage or more (fallDamagePoints, the
 * whole-fall rule; sneaking as the program presses it, so onto slime the
 * whole fall) is no landing (tickFlight). The candidate rings reach six
 * below a take-off (HOP_DROP: four points at most with the jump's rise);
 * the deep ring, the drops past BOUNCE_DROP, reaches 72 down, and the
 * oracle has no other fall model.
 */
const FALL_VETO = 5
/** A slime touchdown this far inside the pool at least (commitGate, abortsOnSlime). */
const SLIME_EDGE = 1.5
/** Next-leg ranking (hopFrom next): programs within NEXT_SLACK ticks of the quickest, NEXT_COLLECT kept a search, NEXT_CANDS distinct touchdowns weighed. */
const NEXT_SLACK = 1
/** A touchdown slower than this horizontally is pinned (nextLegPick ranks only then): opk-1's post-pinned 0.08 against 0.26 moving. */
const NEXT_PINNED = 0.12
const NEXT_ALONG = typeof process === 'undefined' || process.env.PF_NEXT_ALONG !== '0'
const NEXT_COLLECT = 6
const NEXT_CANDS = 4
/** ...each next leg searched within NEXT_LEG_TICKS of kernel work, all of them within NEXT_TOTAL_TICKS, by the simple families. */
const NEXT_LEG_TICKS = 10_000
const NEXT_TOTAL_TICKS = 40_000
const NEXT_FAMILIES: readonly Family[] = ['straight', 'run-turn', 'air-turn']
/**
 * The two-leg pick (hopChain): programs within CHAIN_PICK_SLACK ticks of a
 * family's quickest, CHAIN_PICK_COLLECT a family, on CHAIN_SEARCH_TICKS of
 * kernel work a round; CHAIN_PICK_CANDS distinct touchdowns weighed on
 * CHAIN_PICK_TICKS a round, all within the caller's budget; CHAIN_PICK_LEAST the
 * least a next leg takes (the pick stops where no later touchdown can win).
 */
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
/** ...and the narrow landings per source the pivot step flies (0.5M kernel ticks a strip). */
const PIVOT_CANDIDATES = 2
/**
 * The exact line-up (HopOracle.lineUp): ticks a press is coasted to rest at
 * most, the heading stride of its sweep (LOOK_STEPs), the second press's
 * spread round its aim (LOOK_STEPs), the move of one sneaking press from rest
 * (blocks, measured on stone), how near the start a rest is worth
 * re-anchoring at, and how many of the nearest are.
 */
const LINEUP_COAST = 60
const LINEUP_STRIDE = 4
const LINEUP_SPREAD = 12
const LINEUP_PRESS = 0.059
const LINEUP_ACCEPT = 0.01
const LINEUP_VERIFY = 6
/** mcc-7-2's moving line-up checks the chain's controls, with room left for other prefixes. */
const MOVING_LINEUP = typeof process === 'undefined' || process.env.PF_MOVING_LINEUP !== '0'
const LINEUP_ANCHOR_TICKS = 4000
/** How many nodes back a carry chain may start (chainStart), and the beam depth it gets per tick of its corridor at chain speed (chain). */
const CHAIN_BACK = 16
const CHAIN_SLACK = 1.4
