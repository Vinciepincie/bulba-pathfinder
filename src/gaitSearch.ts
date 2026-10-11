// Gait schedules in the exact kernel (improvement, allowHopIntoJump).
//
// On the ground the executor decides one thing a tick: jump now, or keep its
// feet. A hop is the faster gait — the jump keeps the sprint boost ground
// friction eats — but it is ballistic: for the length of its arc the body
// goes where it was thrown. Whether to throw it NOW depends on what the arc
// comes down in: the far side of a gap, the lip the next jump leaves from,
// the inside of a corner, the last node of the path. The old answers compared
// two fixed policies (always hop, never hop) a fixed horizon out, and left
// every jump to a gate that took it on the first tick it was possible. They
// took a hop into a one-wide corner because hopping is the faster way down
// the straight before it (arena tunnel1), refused two hops in a row under a
// low roof because the first alone gained a single tick, and jumped a gap a
// tick early into a ditch on its far side.
//
// Here the question is asked exactly. Of the SCHEDULES of jumps the body
// could fly from here — ticks on the ground, a jump, ticks on the ground, a
// jump — which reaches a node some ten blocks on soonest, and does it begin
// with a jump? Every schedule is flown in the kernel (playerSim.ts) as the
// executor will replay it: on the ground steering at the next node not yet
// passed, the way the executor walks (wallSteer.ts); in the air at that node,
// tick for tick. A jump over a gap or up a rise is flown like any hop: what
// tells them apart is that the path asks for it, so it may be taken on any
// tick, once, and over a gap has to hold from a hair either side.
//
// Depth first, jumps first, and cut three ways — none of which changes the
// answer where the path turns, drops or ends:
//
// - the bound: a schedule is dropped once it cannot beat the best so far,
//   going on at the fastest any gait goes round the corners left (togo);
// - dominance: a ground state is not flown from twice, nor where a state
//   explored before it was further down a straight run no later (dominated);
// - the horizon: as far as the hops a schedule holds reach, and a little
//   (scout). Past them every schedule is the same run.

import { SIM, SIM_SLIME, SIM_CLIMBABLE, copyBody, newBody } from './playerSim.js'
import type { PlayerSim, SimBody, SimControl } from './playerSim.js'
import { sideStandoff, wallSlide, againstWall, SPRINT_WALL_MARGIN, STEP_UP_MIN, WALK_STEP_REACH, PROBE_INSET } from './wallSteer.js'
import type { Heading } from './wallSteer.js'
import { landingHurts, SAFE_FALL } from './fallDamage.js'

export interface GaitNode {
  x: number
  y: number
  z: number
  /** A node the plan jumps to: retired as the executor retires a landing (Search.pass). */
  parkour?: boolean
  /** ...that the body walks to all the same (kWalkable): no jump is asked for it. */
  walked?: boolean
}

export interface GaitGoal {
  /**
   * Index of the goal node in the path: the schedule's end is the body past
   * it, down and carried. The search ends its schedules sooner where the way
   * there takes more jumps than a schedule holds (GAIT_HOPS): at the node the
   * last but one of them lands on.
   */
  index: number
  /** The goal node ends the path: it is arrived at on the ground, never flown over. */
  last: boolean
  /** The node retired before the path's first: where the flight to a parkour node that IS the first began. */
  from?: { x: number, z: number } | null
  /** The server counts the whole fall (fallDamage.ts landingHurts); true when absent. */
  wholeFall?: boolean
  /** Fly and run with the 45° strafe (kStrafe). */
  strafe?: boolean
  /** The executor retires a parkour node the body came down past (allowLandingRetire); true when absent. */
  landRetire?: boolean
  /**
   * Index of the node a jump catches a climbable for (a ladder or vine in
   * its column, at its feet or under them): a flight caught there is a
   * landing, and the schedule climbs on, as the executor's catch branch
   * climbs, to the goal's level (Search.climb). The goal is the catch node
   * or a climb node above it in the same column. Absent or -1: none.
   */
  catchAt?: number
}

/** A tick on the ground, as the kernel ran it: where the body looks, and the keys beside forward. */
export interface GaitStep { yaw: number, sprint: boolean, strafe: boolean }

export interface GaitResult {
  /** Ticks to the goal of the best schedule that jumps now; Infinity when none gets there. */
  hop: number
  /**
   * ...and of a schedule that keeps its feet this tick: one that does as well
   * (where a jump lies ahead) or a tick better (over open ground) when `hop`
   * is finite, any that gets there when it is not; Infinity when there is none.
   */
  run: number
  /**
   * The search ran out of work looking for a schedule that keeps its feet.
   * With `hop` finite the jump stands unrefuted: the likeliest refutations
   * (the same jump a tick or two later) are the first flown, and the jump is
   * part of a schedule that gets there. With no jump on offer there is no
   * verdict at all.
   */
  spent: boolean
  /** Kernel ticks flown. */
  work: number
  /**
   * The jump now, as flown: the look of every tick from the take-off to the
   * touchdown. The executor replays it — the schedule was judged on this
   * flight, and a take-off on any other heading is another flight.
   */
  flight: number[]
  /** ...its ticks after the take-off's with the strafe key held (GaitGoal.strafe). */
  strafe: boolean
  /**
   * ...and the tick of it from which forward (and the strafe key) is let go:
   * the flight coasts down onto a node it would fly past (Search.jump).
   * EASE_NEVER: held to the touchdown.
   */
  ease: number
  /** The tick on the ground the feet's schedule begins with (`run` finite), for the executor to take as it was run. */
  step: GaitStep | null
}

/** A flight that touches a wall must land with the heading this far either side too (the legacy gates' graze rule). */
export const GRAZE_YAW = 0.02
/**
 * Cosine of the largest angle between the body's velocity and the commanded
 * heading at which a jump the path asks for may take off (20°), and a hop of
 * the gait (45°). Into a turn a jump flies the OLD heading; the kernel flies
 * it exactly, so the angles are a cut on the search's work, not a model: a
 * flight thrown across the line is one more to refute, and past 45° a hop
 * is never the way to the next node (measured offline: 45° took 73 ticks off
 * 21 routes for a quarter more work; 90° and 180° took less off for more).
 * The 20° stays the executor's own gate on a legacy take-off, and holds a
 * jump the path asks for to a line the body has turned onto: those have one
 * landing to make. PF_HOP_TURN_DEG and PF_GAIT_TURN_DEG for A/B runs.
 */
export const HOP_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_HOP_TURN_DEG : undefined) ?? 20) || 20) * Math.PI / 180)
export const GAIT_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_GAIT_TURN_DEG : undefined) ?? 45) || 45) * Math.PI / 180)
/** How far below the path a jump may come down (the lower of the node behind and the node ahead). */
export const HOP_MAX_DIP = 0.5
/**
 * How far ABOVE a parkour node the body may be and still retire it in the
 * air. Roughly one tick of fall: at that height the landing is committed, so
 * holding the node buys nothing and costs the momentum the next move wants.
 */
export const PARKOUR_LAND_DY = 0.5

/**
 * Ticks on the ground a schedule may wait before a hop over ground that runs
 * on, and the hops it may hold.
 */
const GAIT_DELAY = 3
const GAIT_HOPS = 4
/**
 * The jumps the path may ASK of a schedule — a gap, a rise over a step. One
 * for each, on any tick (the lip is where it is) and not counted against
 * GAIT_HOPS; the schedules end at the node the last of them lands on. Three:
 * a landing is judged by the take-off it leaves for the jump after it, and
 * that one by its own (arena spiral3-a, a row of gaps: with two the body
 * came down where the third could not be thrown).
 */
const GAIT_LIPS = 3
/** Jumps deep a schedule may go, of both kinds (the scratch bodies' depth). */
const GAIT_DEPTH = GAIT_HOPS + 2 * GAIT_LIPS
/**
 * The scout's chain of hops (Search.scout): the ticks it may take, and the
 * blocks past its last touchdown the schedules run on — what that landing
 * shows in: a corner overshot, a wall run into.
 */
const SCOUT_TICKS = 40
const SCOUT_TAIL = 3
/** Ticks a schedule may take. */
const GAIT_BUDGET = 80
/**
 * Kernel ticks a search may fly: on the schedules that jump now, and then on
 * the ones that do not. A search costs some hundreds; these are for the few
 * with three jumps in a row to time, where one cut short takes the first
 * jump it found for the best. Doubled (offline, climb2: 9 spent searches
 * to 1, -9 ticks): a long tick costs no route time, mineflayer catches up.
 */
const GAIT_WORK_HOP = 18000
const GAIT_WORK_RUN = 15000
/**
 * Ticks a landing that hurts stands for the server's packet (plugin.ts
 * HURT_WAIT_TICKS: one, where it comes at once), and the ticks of ground a
 * body that lost its speed there has lost on one that kept it.
 */
const HURT_STAND = 1
const HURT_REGAIN = 2
/**
 * ...and what the health it takes is worth, in ticks: a schedule comes down
 * hurt only where that saves it this much. (Timed alone, a hop off a ledge
 * into a fall that hurts beat running off it by a tick or two, whenever the
 * run came down slowly: half a heart for a tenth of a second.)
 */
const HURT_COST = 6
/** As far as any wall probe reaches from the body (SLIDE_PROBE), and a hair. */
const WALL_REACH = 0.36
/**
 * No gait covers ground faster than this (blocks a tick): the bound's
 * optimism. A body with no hop left in its schedule runs: at the speed its
 * push settles to on the block that carries a runner furthest (RUN_SETTLE
 * times the push: 0.216 / (s^3 (1 - 0.91 s)) at blue ice's s), or the speed
 * it has and a tick's push (RUN_PUSH: sprinting, on plain ground).
 */
const GAIT_VMAX = 0.5
const RUN_PUSH = 1.3
const RUN_SETTLE = 2.2326
/**
 * A parkour node is come down past within LAND_PAST of it, or within
 * LAND_PAST_WALKED on its own platform: ground walked back over to the node
 * (kWalkable). The executor's numbers (plugin.ts).
 */
const LAND_PAST = 1
const LAND_PAST_WALKED = 2.5
/** A node is passed from within this of it at the least: the corner of its box, and a parkour node come down past (Search.pass). */
const PASS_REACH = 0.5
const PASS_REACH_LANDED = LAND_PAST_WALKED
/** Ticks sooner a schedule that keeps its feet must pass a node over open ground to be preferred to the hop (Search.run). */
const GAIT_RUN_GAIN = 1
/**
 * The second ground policy (Search.roundMode, gaitSearch's `pick`): how much
 * sooner (ticks) it must reach the goal to be taken, and how far (blocks) the
 * corner it rounds may lie off the leg it is on. Offline over 24 routes
 * (driveSim, the arena's traces) the guarded pair took 56 ticks off and lost
 * none above 5; unguarded — a 0 margin, no stickiness, no corridor — it fell
 * on basic3 and stalled on spiral3-a, the two policies flapping into states
 * neither schedules.
 */
/**
 * The aim node of a turn test closer than this (blocks, horizontally) is
 * passed for the turn's purposes: the heading to it is float noise. A body
 * riding a block above its floor nodes (basic3's y237 tops beside y236 nodes)
 * never passes them (|dy| < 1), and the hop at the next node was refused on
 * a cos of 0.60 against a node 0.04 away. PF_GAIT_NEARAIM=0 bisects.
 */
const NEAR_AIM = 0.35
const GAIT_NEARAIM = typeof process === 'undefined' || process.env.PF_GAIT_NEARAIM !== '0'
/**
 * A touchdown further than this (blocks, either axis) from the last parkour
 * node its flight retired flew over that node (Search.hop): the eased flights
 * are tried for it. A node retired in the air (PARKOUR_LAND_DY) was never
 * 'over', so a stair of 2-deep steps was hopped to the far edge of each and
 * the next riser bonked (arena opk-9: -5 ticks offline; it fired on no other
 * route of the book).
 */
const EASE_PAST = 1
const ROUND_MARGIN = 2
const ROUND_CORRIDOR = 0.5
/** How far ahead (blocks) the rounding policy looks for a box in the way, and the body band it looks in. */
const ROUND_REACH = 2.5
const ROUND_BAND = 0.61
/** PF_GAIT_ROUND=0 bisects the second policy. */
const GAIT_ROUND = typeof process === 'undefined' || process.env.PF_GAIT_ROUND !== '0'
/** The strafe fallback (gaitSearch): how many ticks sooner the search without the strafe must be; PF_STRAFE_FALLBACK=0 bisects. */
const STRAFE_MARGIN = 1
const STRAFE_FALLBACK = typeof process === 'undefined' || process.env.PF_STRAFE_FALLBACK !== '0'
/**
 * A jump is taken only if it comes down the same from starts this far either
 * side too (hopOracle.ts LIVE_NUDGE): the replay is exact, the look the
 * executor turns to is not (mineflayer turns in 0.15° steps).
 */
const HOP_NUDGE = 0.01
/** GaitResult.ease of a flight with forward held all the way. */
export const EASE_NEVER = 99
/**
 * A caught flight (Search.climb): the executor's catch branch steers the
 * along-wall drift back to the column's centre (plugin.ts CLIMB_DRIFT,
 * CLIMB_HOLD); the climb ends at the goal's level, CATCH_LEVEL under it. A
 * nudged flight holds where it catches the same column no further down than
 * CATCH_NUDGE_DY and climbs to the goal's level too.
 */
const CATCH_DRIFT = 0.91 / 0.09
const CATCH_HOLD = 0.15
const CATCH_LEVEL = 0.01
const CATCH_NUDGE_DY = 1
/** The ladder's climb, blocks a tick: (ladderClimbSpeed - gravity) * airdrag. */
const LADDER_RISE = (SIM.ladderClimbSpeed - SIM.gravity) * SIM.airdrag
const WALL_SIDES: ReadonlyArray<readonly [number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1]]
/** PF_GAIT_CATCH=0 bisects the catch landing. */
export const GAIT_CATCH = typeof process === 'undefined' || process.env.PF_GAIT_CATCH !== '0'
/** A body at a schedule's end has stopped below this speed, and is given this many ticks to (Search.stops). */
const STOP_SPEED = 0.05
const STOP_TICKS = 8
/**
 * Dominance (Search.dominated). Bodies on levels further apart than
 * DOM_LEVEL are not compared. A state is THE SAME as another within DOM_SAME
 * (blocks, blocks a tick). Over DOM_CLEAR blocks of straight path a lead of
 * DOM_LEAD blocks per block-a-tick of speed is one the slower body ahead
 * keeps: a surplus of speed decays by the air's friction at the least,
 * 1 / (1 - 0.91) ticks of it.
 */
const DOM_LEVEL = 0.6
const DOM_SAME = 1e-3
const DOM_CLEAR = 5
const DOM_LEAD = 11.2
/** The ground states a phase has explored (labels): where, how fast, when, with what left to do, and how it came out. */
const LABELS = 4096
const OPEN = 0
const ALIVE = 1
const DEAD = 2
/** ...and the labels of a subtree under a landing that did not hold (Search.verified): they count for nothing. */
const VOID = 3
const labelX = new Float64Array(LABELS)
const labelY = new Float64Array(LABELS)
const labelZ = new Float64Array(LABELS)
const labelVx = new Float64Array(LABELS)
const labelVz = new Float64Array(LABELS)
const labelT = new Float64Array(LABELS)
/** (progress down the path and speed, the measures of a lead; and is it on a straight run) */
const labelS = new Float64Array(LABELS)
const labelV = new Float64Array(LABELS)
const labelClear = new Uint8Array(LABELS)
const labelHops = new Uint8Array(LABELS)
const labelWait = new Uint8Array(LABELS)
const labelState = new Uint8Array(LABELS)
const NUDGES: ReadonlyArray<readonly [number, number]> = [[HOP_NUDGE, 0], [-HOP_NUDGE, 0], [0, HOP_NUDGE], [0, -HOP_NUDGE]]

/**
 * The 45° strafe, on a control already aimed down its line: the look turned
 * an eighth of a turn to the right and the left key bringing the push back
 * onto the line. Forward alone is an input 0.98 long; forward and strafe are
 * normalised to 1 — 2% more push, on the ground and in the air (5.58 to 5.69
 * b/s sprinting, 6.97 to 7.04 hopping, measured in the kernel). Never on a
 * jump's own tick: a sprint jump's boost goes where the body LOOKS.
 */
export function kStrafe (c: SimControl): void {
  const hx = c.hx
  c.hx = (hx - c.hz) * Math.SQRT1_2
  c.hz = (c.hz + hx) * Math.SQRT1_2
  c.left = true
}

/** The yaw that looks down a line of travel yaw `yaw` for the strafe (kStrafe, with the left key). */
export function strafeYaw (yaw: number): number {
  return yaw - Math.PI / 4
}

/** Aim control `c` from `b` at (px, pz). */
export function kAim (c: SimControl, b: SimBody, px: number, pz: number): void {
  const dx = px - b.x
  const dz = pz - b.z
  const d = Math.sqrt(dx * dx + dz * dz)
  if (d <= 1e-9) return
  c.hx = dx / d
  c.hz = dz / d
}

/** Which policy the last decision took (per PhysicsSim): the rounding one is kept until the plain one beats it by ROUND_MARGIN. */
export interface GaitPick { round: boolean }

class Search {
  /**
   * The second ground policy: a run tick whose straight step to its node has
   * a box in the way at body height (within ROUND_REACH) steers at that box's
   * nearest corner instead — round it, sprinting — where the plain policy
   * walks the wall it meets (sideStandoff, wallSlide, no sprint against it).
   * On opk-4's diagonal pillar squeezes the plain run was refuted (the
   * standoff pushed the body off its support) and the gait threw 12-tick
   * hops pinned against the pillar face.
   */
  roundMode = false
  private best = Infinity
  private work = 0
  /** The work the phase being searched may reach, and whether it got there. */
  private limit = GAIT_WORK_HOP
  private spent = false
  /** The run-first search ends at the first schedule it finds. */
  private firstWins = false
  private done = false
  private readonly c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: -1 }
  private readonly win: SimBody[] = []
  private readonly fly: SimBody[] = []
  /** The best jump from the root: its looks (GaitResult.flight) and its ease. */
  private readonly flight: number[] = []
  private ease = EASE_NEVER
  /** The looks of the jump being flown that has to hold, by depth (Search.holds). */
  private readonly looks: number[][] = [[]]
  /**
   * Per depth, of the jump flown there: must it hold (Search.verified), from
   * which take-off, with what ease, landing at what level and carried; is
   * it known to (-1 not yet, 0 no, 1 yes); the labels before its subtree;
   * and did its subtree find it not to.
   */
  private readonly heldAt: boolean[] = []
  private readonly holdFrom: SimBody[] = []
  private readonly holdEase: number[] = []
  private readonly holdY: number[] = []
  private readonly holdCarried: boolean[] = []
  private readonly holdCatch: boolean[] = []
  private readonly holdOk: number[] = []
  /** The catch (GaitGoal.catchAt): its node index, its column, and the bottom of the climbable under it. */
  private catchAt = -1
  private catchX = 0
  private catchZ = 0
  private catchLow = 0
  /** The wall a climbable is on (wallAt). */
  private wallX = 0
  private wallZ = 0
  private readonly labelsAt: number[] = []
  private readonly unheld: boolean[] = []
  /** Per depth, of the flight last flown there: did it fly past a node it was for (and which), and its ticks. */
  private readonly over: boolean[] = []
  private readonly overAt: number[] = []
  private readonly flew: number[] = []
  private readonly nudged = newBody(0, 0, 0)
  /** The first tick of the feet's schedule (GaitResult.step): taken down while `stepWanted`. */
  private step: GaitStep | null = null
  private stepWanted = false
  /** Where the body stood before the tick just flown (passedAt). */
  private px = 0
  private pz = 0
  /** When the last stride ended. */
  private at = 0
  /** The executor's collision probe, on the kernel; and the heading being steered. */
  private readonly hit = (x: number, y: number, z: number): boolean => this.k.probeHit(x, y, z, -PROBE_INSET, PROBE_INSET)
  private readonly h: Heading = { dx: 0, dz: 0 }
  private readonly strafe: boolean
  /** Per node: does getting to it take a jump (a parkour node, a rise over a step)? */
  private readonly lip: boolean[] = []
  private rootY = 0
  /** Per node: the path's length to it from the root, through the nodes before it; and from it on to the path's first turn. */
  private readonly cum: number[] = []
  private readonly clear: number[] = []
  /** Per node: the blocks from passing it to passing the goal at the least, through the nodes between; is there a lip from it on; its PASS_REACH. */
  private readonly rest: number[] = []
  private readonly lipAhead: boolean[] = []
  private readonly reach: number[] = []
  /** The speed a runner settles to at the most, and a tick's push (togo). */
  private vRun = 0
  private aRun = 0
  /** The labels of this phase (dominated), the ones each call of dfs opened, and whether the hop from each came to anything. */
  private labels = 0
  private readonly opened: Int32Array[] = []
  private readonly hopped: Uint8Array[] = []
  /** The measures of the state dominated was last asked about, for label. */
  private stateS = 0
  private stateV = 0
  private stateClear = false
  /** Did the stride that ended its schedule end it alive (at the goal), not lost? */
  private live = false
  /** Scouting (Search.scout): a hop ends at its touchdown, short of the goal — the node index there and the time. */
  private scouting = false
  private landI = -1
  private landT = 0
  /** The goal, as far as the schedules are flown (GaitGoal.index, or sooner: GAIT_LIPS). */
  private index = 0
  private last = false

  constructor (
    private readonly k: PlayerSim,
    private readonly path: ReadonlyArray<GaitNode>,
    private readonly goal: GaitGoal
  ) {
    this.strafe = goal.strafe === true
  }

  run (b0: SimBody): GaitResult {
    const root = copyBody(newBody(0, 0, 0), b0)
    this.rootY = root.y
    let y = root.y
    let lips = 0
    let px = root.x
    let pz = root.z
    let cum = 0
    this.index = this.goal.index
    for (let j = 0; j <= this.goal.index; j++) {
      const n = this.path[j]
      this.lip[j] = (n.parkour === true && n.walked !== true) || n.y > y + SIM.stepHeight
      y = n.y
      cum += Math.hypot(n.x - px, n.z - pz)
      this.cum[j] = cum
      px = n.x
      pz = n.z
      if (this.lip[j] && ++lips >= GAIT_LIPS) { this.index = j; break }
    }
    this.last = this.goal.last && this.index === this.goal.index
    // the catch, inside the schedules' reach: the column, and how far down it the climbable runs
    this.catchAt = GAIT_CATCH && this.goal.catchAt !== undefined && this.goal.catchAt >= 0 && this.goal.catchAt <= this.index ? this.goal.catchAt : -1
    if (this.catchAt >= 0) {
      const n = this.path[this.catchAt]
      this.catchX = Math.floor(n.x)
      this.catchZ = Math.floor(n.z)
      let y = Math.floor(n.y + 0.001)
      if ((this.k.kindAt(n.x, y, n.z) & SIM_CLIMBABLE) === 0) y--
      while ((this.k.kindAt(n.x, y - 1, n.z) & SIM_CLIMBABLE) !== 0 && y > n.y - 16) y--
      this.catchLow = y
    }
    const i0 = this.pass(root, 0, 1, false)
    const none: GaitResult = { hop: Infinity, run: Infinity, spent: false, work: 0, flight: this.flight, strafe: this.strafe, ease: EASE_NEVER, step: null }
    // the goal from here, this tick: nothing to schedule
    if (i0 > this.index) return none
    this.scout(root, i0)
    this.vRun = this.k.speedBase * RUN_PUSH * RUN_SETTLE
    this.aRun = this.k.speedBase * RUN_PUSH
    for (let j = this.index; j >= 0; j--) {
      const n = this.path[j]
      this.reach[j] = n.parkour === true ? PASS_REACH_LANDED : PASS_REACH
      this.lipAhead[j] = this.lip[j] || (j < this.index && this.lipAhead[j + 1])
      let rest = 0
      for (let m = j + 1; m <= this.index; m++) {
        const o = this.path[m]
        rest = Math.max(rest, Math.max(0, Math.hypot(o.x - n.x, o.z - n.z) - this.reach[j] - this.reach[m]) + this.rest[m])
      }
      this.rest[j] = rest
    }
    // (past the goal is past the search: first there is all it asks, unless the path ends there)
    this.clear[this.index] = this.last ? 0 : Infinity
    for (let j = this.index - 1; j >= 0; j--) {
      const a = j > 0 ? this.path[j - 1] : root
      const n = this.path[j]
      const m = this.path[j + 1]
      const ax = n.x - a.x
      const az = n.z - a.z
      const bx = m.x - n.x
      const bz = m.z - n.z
      const al = Math.hypot(ax, az)
      const bl = Math.hypot(bx, bz)
      const turn = al > 0.35 && bl > 1e-6 && (ax * bx + az * bz) / (al * bl) < HOP_TURN_COS
      this.clear[j] = turn ? 0 : bl + this.clear[j + 1]
    }
    const jumps = this.lipAhead[0]

    let hop = Infinity
    this.best = GAIT_BUDGET + 1
    const asked = this.needsJump(root, i0)
    if (this.mayHop(root, i0, asked)) {
      this.jump(root, i0, 0, asked ? 0 : 1, 0, asked ? i0 : -1, asked)
      if (this.best <= GAIT_BUDGET) hop = this.best
    }
    const hopSpent = this.spent

    // Keeping its feet this tick: does any schedule do as well? Where a jump
    // lies ahead a tie keeps them: the goal is reached the same either way,
    // and the ground has the options. (A node passed is timed to the
    // fraction of a tick, passedAt: in whole ticks a hop now ties with the
    // same hop a tick later, the tick after that ties again, and the body
    // runs where it should be flying — arena simple1, four ticks on the spot
    // the hop was due.)
    //
    // Over open ground the feet have to EARN the tick (GAIT_RUN_GAIN). The
    // goal there is a horizon, not a place: hopping at every touchdown is
    // the fastest gait there is, and which of two schedules is first past a
    // node eight blocks on turns on where its hops happen to fall about that
    // node — half a tick either way, for nothing (arena simple3: two ticks
    // run after every landing, a tick lost on each). A corner, a lip or the
    // path's end costs whole ticks, and shows.
    //
    // With no jump now on offer the feet's first schedule is the answer to
    // another question: does running on get there at all?
    const bound = hop === Infinity ? GAIT_BUDGET + 1 : jumps ? hop + 1e-6 : hop - GAIT_RUN_GAIN + 1e-6
    this.best = bound
    // (the first schedule under the bound answers "can the feet do as well";
    // with no jump on offer the question is the best way on, not any)
    this.firstWins = hop !== Infinity || !GAIT_NEARAIM
    this.heldAt[0] = false
    this.limit = this.work + GAIT_WORK_RUN
    this.spent = false
    this.labels = 0
    this.stepWanted = true
    const i = this.stride(root, i0, 0, 0)
    if (i >= 0) this.dfs(root, i, this.at, 0, 1, -1)
    const run = this.best < bound ? this.best : Infinity
    return {
      hop,
      run,
      spent: run === Infinity && (this.spent || hopSpent),
      work: this.work,
      flight: this.flight,
      strafe: this.strafe,
      ease: this.ease,
      step: run === Infinity ? null : this.step
    }
  }

  /**
   * The horizon, in hops. A schedule holds GAIT_HOPS hops and runs the rest
   * of the way; under a low roof, where a hop is three ticks and a block and
   * a half, that rest is most of the way, the same run behind every one of
   * the schedules — and all of the search's work. So the goal is no further
   * than the chain of hops goes: the body hopped at every touchdown from the
   * root (run where it may not hop: into a turn), the node its last hop
   * comes down short of, and SCOUT_TAIL blocks. A jump the path asks for on
   * the way, a chain that is lost or reaches the goal: the goal stays.
   */
  private scout (root: SimBody, i0: number): void {
    const b = copyBody(this.scratch(this.fly, 0), root)
    const best = this.best
    this.best = Infinity
    this.scouting = true
    let i = i0
    let t = 0
    for (let hops = 0; hops < GAIT_HOPS && i >= 0;) {
      if (this.lip[i] || t > SCOUT_TICKS) { i = -1; break }
      if (this.mayHop(b, i, false)) {
        this.landI = -1
        this.hop(b, b, i, t, hops + 1, 1, -1, false, EASE_NEVER, false)
        i = this.landI
        t = this.landT
        hops++
      } else {
        i = this.stride(b, i, t, 0)
        t = this.at
      }
    }
    this.scouting = false
    this.best = best
    if (i < 0) return
    while (i < this.index && this.cum[i] - this.cum[this.landI] < SCOUT_TAIL) i++
    if (i < this.index) {
      this.index = i
      this.last = false
    }
  }

  /**
   * Does the jump from `from`, flown with `looks` to land as `h` did, come
   * down the same from a hair either side (HOP_NUDGE)? Its looks replayed,
   * as the executor replays them: on the ground at the level it came down
   * on, carried as it was. Asked of the jump the executor is handed, and of
   * every jump the path asks for: a schedule that keeps its feet for a
   * take-off a tick on is no schedule if that take-off is one in a hundred.
   */
  private holds (from: SimBody, looks: number[], ease: number, y: number, carried: boolean, caught: boolean): boolean {
    const c = this.c
    const n = looks.length
    for (const [nx, nz] of NUDGES) {
      const b = copyBody(this.nudged, from)
      b.x += nx
      b.z += nz
      // (a start a wall is in the way of is not one the body can be at)
      if (this.k.collides(b)) continue
      let down = false
      for (let f = 0; f < n + 2 && !down; f++) {
        const yaw = looks[Math.min(f, n - 1)]
        c.hx = -Math.sin(yaw)
        c.hz = -Math.cos(yaw)
        c.jump = f === 0
        c.sprint = true
        c.forward = f < ease
        c.left = this.strafe && f > 0 && f < ease
        this.k.step(b, c)
        this.work++
        c.forward = true
        if (b.inLiquid) return false
        down = f > 0 && (b.onGround || (caught && this.catches(b)))
      }
      // (a catch holds where the nudged flight catches the same column, about
      // as high, and climbs to the goal's level from there too: the chosen
      // climbs ran within a hundredth of the column's edge)
      if (caught) {
        if (!down || b.onGround || b.y < y - CATCH_NUDGE_DY || !this.climbsUp(b)) return false
        continue
      }
      if (!down || Math.abs(b.y - y) > 1e-6 || this.k.carried(b) !== carried) return false
    }
    return true
  }

  /**
   * One tick with the feet down from time `t`, and the fall off what it
   * walked off, to the ground: the node index there (the time in `at`), -1
   * when the schedule ended on the way — lost, or past the goal and recorded.
   */
  private stride (b: SimBody, i: number, t: number, depth: number): number {
    let fall = 0
    let drop = 0
    let passed = -1
    let air = !b.onGround
    do {
      const y0 = b.y
      i = this.runTick(b, i, air)
      t++
      this.live = false
      if (i < 0 || t > GAIT_BUDGET) return -1
      if (i > this.index && passed < 0) passed = this.passedAt(b, t)
      drop = Math.max(0, y0 - b.y)
      fall += drop
      air = !b.onGround
    } while (air)
    if (fall > 0 && (this.k.kindAt(b.x, b.y - 0.2, b.z) & SIM_SLIME) !== 0) return -1
    const stood = this.landed(b, fall, drop)
    if (passed >= 0) {
      this.live = this.end(b, passed, t, stood, depth)
      return -1
    }
    this.at = t + stood
    return i
  }

  /**
   * A schedule's end: the goal passed at `passed`, the body down at `t`.
   * Past the goal is past the search, so it counts only where the body could
   * be stopped there: carried, and still carried once it has braked (a post
   * landed on at a hop's speed is slid off the far side). A landing that hurt
   * is charged for the speed it lost. False when the schedule is lost there.
   */
  private end (b: SimBody, passed: number, t: number, stood: number, depth: number): boolean {
    const cost = stood > 0 ? Math.max(passed, t) + stood + HURT_REGAIN : passed
    if (cost >= this.best) return true
    if (!this.stops(b)) return false
    if (!this.verified(depth)) return false
    this.record(cost)
    return true
  }

  /**
   * Do the jumps above depth `depth` that have to hold (holds) hold? Asked
   * of a schedule at its end, as it is about to be the best: a nudge test is
   * four flights, and paid at every landing explored it was most of the work
   * of a search with three gaps to time — paid at the candidates, it is a
   * few. A jump found not to hold voids its subtree (unheld): the hop that
   * threw it is lost, and the states explored under it are no evidence.
   */
  /** Is airborne body `b` caught on the climbable in the catch node's column (GaitGoal.catchAt)? */
  private catches (b: SimBody): boolean {
    return !b.onGround && Math.floor(b.x) === this.catchX && Math.floor(b.z) === this.catchZ &&
      (this.k.kindAt(b.x, b.y, b.z) & SIM_CLIMBABLE) !== 0
  }

  /**
   * The wall the climbable the body is in hangs on (plugin.ts climbWall):
   * the side of the cell its thin box is on, or a full block beside it (a
   * vine). Into wallX/wallZ; false when there is none.
   */
  private wallAt (b: SimBody): boolean {
    const k = this.k
    const w = k.world
    const fx = Math.floor(b.x)
    const fy = Math.floor(b.y + 0.001)
    const fz = Math.floor(b.z)
    const s = k.stateAt(fx, fy, fz)
    if (s >= 0 && w.shapeStart[s] >= 0) {
      for (let j = 0; j < w.shapeCount[s]; j++) {
        const o = (w.shapeStart[s] + j) * 6
        if (w.boxes[o + 3] - w.boxes[o] < 0.5) { this.wallX = w.boxes[o] > 0.5 ? 1 : -1; this.wallZ = 0; return true }
        if (w.boxes[o + 5] - w.boxes[o + 2] < 0.5) { this.wallX = 0; this.wallZ = w.boxes[o + 2] > 0.5 ? 1 : -1; return true }
      }
    }
    for (const [ox, oz] of WALL_SIDES) {
      const t = k.stateAt(fx + ox, fy, fz + oz)
      if (t < 0 || (k.kindAt(fx + ox, fy, fz + oz) & SIM_CLIMBABLE) !== 0 || w.shapeStart[t] < 0 || w.shapeCount[t] !== 1) continue
      const o = w.shapeStart[t] * 6
      if (w.boxes[o] === 0 && w.boxes[o + 1] === 0 && w.boxes[o + 2] === 0 && w.boxes[o + 3] === 1 && w.boxes[o + 4] === 1 && w.boxes[o + 5] === 1) {
        this.wallX = ox
        this.wallZ = oz
        return true
      }
    }
    return false
  }

  /** The catch branch's heading on a climbable: into its wall, and against the along-wall drift off the column's centre. */
  private climbHeading (b: SimBody, goal: GaitNode): void {
    const c = this.c
    if (!this.wallAt(b)) { kAim(c, b, goal.x, goal.z); return }
    const ax = this.wallZ !== 0 ? 1 : 0
    const az = this.wallX !== 0 ? 1 : 0
    const along = (Math.floor(b.x) + 0.5 - b.x) * ax + (Math.floor(b.z) + 0.5 - b.z) * az - (b.vx * ax + b.vz * az) * CATCH_DRIFT
    let hx = this.wallX
    let hz = this.wallZ
    if (Math.abs(along) > CATCH_HOLD) { hx += ax * Math.sign(along); hz += az * Math.sign(along) }
    const l = Math.hypot(hx, hz)
    c.hx = hx / l
    c.hz = hz / l
  }

  /** Does caught body `b` (consumed) climb to the goal's level without leaving the climbable (climb's strict rule, no clock)? */
  private climbsUp (b: SimBody): boolean {
    const goal = this.path[this.index]
    const c = this.c
    for (let t = 0; t < GAIT_BUDGET; t++) {
      if (b.y >= goal.y - CATCH_LEVEL) return true
      if (!this.k.onClimbable(b) || b.inLiquid || (b.onGround && b.y < goal.y - CATCH_LEVEL)) return false
      c.forward = true; c.back = false; c.left = false; c.jump = false; c.sprint = false; c.sneak = true
      this.climbHeading(b, goal)
      this.k.step(b, c)
      this.work++
      c.sneak = false
      c.sprint = true
    }
    return false
  }

  /**
   * A flight caught on the climbable in the catch node's column at time `t`
   * (body `h`, node index `i`): the climb on, as the executor's catch
   * branch climbs (plugin.ts, `caughtOnClimbable`) - pressed into the wall
   * and steered against the drift toward the column's centre, sneaking
   * (which holds the height and does not stop the climb) - to the goal's
   * level. That is the schedule's end: the time the goal's level is reached,
   * to the fraction of a tick. Lost where the body leaves the climbable
   * short of it (a body sliding off a one-wide column is a fall).
   */
  private climb (h: SimBody, i: number, t: number, depth: number): boolean {
    const goal = this.path[this.index]
    const c = this.c
    let y0 = h.y
    for (;;) {
      if (h.y >= goal.y - CATCH_LEVEL) {
        const rise = h.y - y0
        const passed = rise > 1e-9 && t > 0 ? t - 1 + Math.min(1, Math.max(0, (goal.y - CATCH_LEVEL - y0) / rise)) : t
        if (passed >= this.best) return true
        if (!this.verified(depth)) return false
        this.record(passed)
        return true
      }
      // (the bound: no faster than the ladder climbs)
      if (t + (goal.y - CATCH_LEVEL - h.y) / LADDER_RISE >= this.best) return true
      if (!this.k.onClimbable(h)) return false
      c.forward = true
      c.back = false
      c.left = false
      c.jump = false
      c.sprint = false
      c.sneak = true
      this.climbHeading(h, goal)
      y0 = h.y
      this.k.step(h, c)
      this.work++
      t++
      c.sneak = false
      c.sprint = true
      if (h.inLiquid || t > GAIT_BUDGET) return false
      // (stood on something short of the goal: a ledge under the climb, not the climb)
      if (h.onGround && h.y < goal.y - CATCH_LEVEL) return false
      i = this.pass(h, i, 1.45, false)
    }
  }

  private verified (depth: number): boolean {
    for (let d = 0; d < depth; d++) {
      if (!this.heldAt[d]) continue
      if (this.holdOk[d] < 0) this.holdOk[d] = this.holds(this.holdFrom[d], this.looks[d], this.holdEase[d], this.holdY[d], this.holdCarried[d], this.holdCatch[d]) ? 1 : 0
      if (this.holdOk[d] === 0) { this.unheld[d] = true; return false }
    }
    return true
  }

  /**
   * Is ground state `b` (aiming at node `i`, at time `t`; `hops` taken,
   * `wait` ticks since its landing) one not worth exploring, for a state
   * explored before it — no later, and with as much left to it (no more hops
   * taken, no longer waited)? Two ways:
   *
   * - it is that state: there, at that velocity. What `b` does from here,
   *   that one did sooner.
   * - both on a straight run of the path (DOM_CLEAR ahead, and moving down
   *   it), that one is ahead by more than the speed `b` has on it could
   *   close (DOM_LEAD). Short of a turn or the path's end the faster body is
   *   not the better off — it is the one that overshoots — and a stride's
   *   difference in where the turn is met is ticks: there, states are not
   *   compared.
   *
   * ALIVE when that state's schedules got somewhere (the goal, or the bound:
   * no better than the best). DEAD when they are still being flown — `b` is
   * a state the body came round to again, no further for it (run against a
   * wall, down a hole) — or all came to nothing and `b` is that very state.
   * A state whose schedules were all lost dominates nothing else: where it
   * was too far, or too fast, `b` may not be.
   */
  private dominated (b: SimBody, i: number, t: number, hops: number, wait: number): number {
    const n = this.path[i]
    const dist = Math.hypot(n.x - b.x, n.z - b.z)
    const s = this.cum[i] - dist
    const v = Math.hypot(b.vx, b.vz)
    const clear = dist + this.clear[i] >= DOM_CLEAR && (v <= 1e-6 || dist <= 1e-6 || (b.vx * (n.x - b.x) + b.vz * (n.z - b.z)) / (v * dist) >= HOP_TURN_COS)
    this.stateS = s
    this.stateV = v
    this.stateClear = clear
    let open = false
    for (let a = 0; a < this.labels; a++) {
      const state = labelState[a]
      if (state === VOID || labelT[a] > t || labelHops[a] > hops || labelWait[a] > wait || Math.abs(labelY[a] - b.y) > DOM_LEVEL) continue
      const same = Math.abs(labelX[a] - b.x) < DOM_SAME && Math.abs(labelZ[a] - b.z) < DOM_SAME &&
        Math.abs(labelVx[a] - b.vx) < DOM_SAME && Math.abs(labelVz[a] - b.vz) < DOM_SAME
      if (state === DEAD) {
        if (same) return DEAD
        continue
      }
      if (!same && !(clear && labelClear[a] === 1 && labelS[a] - s >= DOM_LEAD * Math.max(0, v - labelV[a]))) continue
      if (state === ALIVE) return ALIVE
      open = true
    }
    return open ? DEAD : OPEN
  }

  /** Take down ground state `b`, as dominated measured it: explored from time `t` on. */
  private label (b: SimBody, t: number, hops: number, wait: number): number {
    const a = this.labels
    if (a >= LABELS) return -1
    labelX[a] = b.x
    labelY[a] = b.y
    labelZ[a] = b.z
    labelVx[a] = b.vx
    labelVz[a] = b.vz
    labelT[a] = t
    labelS[a] = this.stateS
    labelV[a] = this.stateV
    labelClear[a] = this.stateClear ? 1 : 0
    labelHops[a] = hops
    labelWait[a] = wait
    labelState[a] = OPEN
    this.labels = a + 1
    return a
  }

  /** Does body `b`, down, come to a stand where it is — braked as the executor brakes (back held along its motion)? */
  private stops (b: SimBody): boolean {
    const k = this.k
    if (!k.carried(b)) return false
    const s = copyBody(this.halt, b)
    const c = this.c
    for (let t = 0; t < STOP_TICKS; t++) {
      const speed = Math.hypot(s.vx, s.vz)
      if (speed < STOP_SPEED) break
      c.hx = s.vx / speed
      c.hz = s.vz / speed
      c.forward = false
      c.back = true
      c.sprint = false
      c.jump = false
      c.left = false
      k.step(s, c)
      this.work++
      c.forward = true
      c.back = false
      if (!s.onGround || s.inLiquid) return false
    }
    return k.carried(s)
  }

  private readonly halt = newBody(0, 0, 0)

  /**
   * A landing after `fall` (`drop` of it this tick): what it costs when it
   * hurt, in ticks — the tick it stands (the server takes the speed of a
   * body it hurts, and the executor decides nothing until it has: plugin.ts
   * HURT_WAIT_TICKS) and what the health is worth (HURT_COST).
   */
  private landed (b: SimBody, fall: number, drop: number): number {
    if (fall <= SAFE_FALL) return 0
    const kind = this.k.kindAt(b.x, b.y - 0.2, b.z)
    if (!landingHurts(fall, drop, kind, false, this.goal.wholeFall !== false)) return 0
    b.vx = 0
    b.vz = 0
    return HURT_STAND + HURT_COST
  }

  private scratch (pool: SimBody[], depth: number): SimBody {
    while (pool.length <= depth) pool.push(newBody(0, 0, 0))
    return pool[depth]
  }

  private record (t: number): void {
    if (t >= this.best) return
    this.best = t
    if (this.firstWins) this.done = true
  }

  /** The node the body steers at: the next not yet passed. */
  private aim (i: number): GaitNode {
    return this.path[Math.min(i, this.path.length - 1)]
  }

  /** The lowest a body between node `i - 1` and node `i` belongs: under it, it has left the path. */
  private low (i: number): number {
    const a = i > 0 ? this.path[Math.min(i, this.path.length) - 1].y : this.rootY
    return Math.min(a, this.aim(i).y)
  }

  /**
   * Nodes passed by body `b` from index `i` on, as the executor retires them
   * (plugin.ts, the arrival loop): inside the arrival box (`dy` its height)
   * or down on the leg to the next node, past it (onLegPast);
   * a parkour node in it only from low over it (PARKOUR_LAND_DY: higher, a
   * node with a hole beyond is held to the landing) or, `landing` (down this
   * tick), within a block and past it. A landing short of a parkour node is a
   * take-off for it still (arena tenways-headhitters: counted as landed, the
   * next jump left from the lip into a head-hitter, 12 ticks a platform).
   */
  private pass (b: SimBody, i: number, dy: number, landing: boolean): number {
    while (i <= this.index) {
      const n = this.path[i]
      const boxed = Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < dy
      if (n.parkour === true) {
        if (!(boxed && b.y - n.y <= PARKOUR_LAND_DY) && !(landing && this.cameDownPast(b, i))) break
      } else if (!boxed && !(b.onGround && i + 1 < this.path.length && Math.abs(n.y - b.y) < dy && onLegPast(b.x, b.z, n, this.path[i + 1]))) break
      // (the path's end is arrived at: down, and in the node's own cell — a goal is a cell, and a box round a node off its centre is two)
      if (this.last && i === this.index && (!b.onGround || Math.floor(b.x) !== Math.floor(n.x) || Math.floor(b.z) !== Math.floor(n.z))) break
      i++
    }
    return i
  }

  /**
   * The executor's landing retirement (plugin.ts allowLandingRetire): has
   * body `b`, down this tick, come down within a block of parkour node `i`
   * and past it along the flight from the node before?
   */
  private cameDownPast (b: SimBody, i: number): boolean {
    if (this.goal.landRetire === false || i >= this.path.length - 1) return false
    const node = this.path[i]
    const from = i > 0 ? this.path[i - 1] : this.goal.from
    if (from === undefined || from === null) return false
    const far = Math.hypot(node.x - b.x, node.z - b.z)
    if (far > LAND_PAST_WALKED || Math.abs(node.y - b.y) >= 1) return false
    const fx = node.x - from.x
    const fz = node.z - from.z
    if (!(fx * fx + fz * fz > 1 && (b.x - node.x) * fx + (b.z - node.z) * fz > 0)) return false
    return far <= LAND_PAST || kWalkable(this.k, b, node)
  }

  /**
   * When the goal node was passed, to the fraction of a tick: the tick
   * before `t`, and how far into the move from there the body crossed into
   * the node's box. Whole ticks tie too often to choose by — a hop now and
   * the same hop a tick later pass a node eight blocks on in the same tick.
   */
  private passedAt (b: SimBody, t: number): number {
    const n = this.path[this.index]
    const ax = Math.abs(n.x - this.px)
    const az = Math.abs(n.z - this.pz)
    const dx = Math.abs(b.x - this.px)
    const dz = Math.abs(b.z - this.pz)
    let f = 0
    if (ax > 0.35 && dx > 1e-9) f = Math.max(f, (ax - 0.35) / dx)
    if (az > 0.35 && dz > 1e-9) f = Math.max(f, (az - 0.35) / dz)
    return t - 1 + Math.min(1, f)
  }

  /** May a jump be taken off here: a hop of the gait, or (`asked`) one the path asks for — not into a turn past its angle (GAIT_TURN_COS, HOP_TURN_COS)? */
  private mayHop (b: SimBody, i: number, asked: boolean): boolean {
    let n = this.aim(i)
    let dx = n.x - b.x
    let dz = n.z - b.z
    let head = Math.hypot(dx, dz)
    // (a node the body is all but on: the turn is the one to the node after it)
    if (GAIT_NEARAIM && head < NEAR_AIM && i + 1 <= this.index) {
      n = this.aim(i + 1)
      dx = n.x - b.x
      dz = n.z - b.z
      head = Math.hypot(dx, dz)
    }
    const speed = Math.hypot(b.vx, b.vz)
    return !(speed > 0.1 && head > 1e-6 && (b.vx * dx + b.vz * dz) / (speed * head) < (asked ? HOP_TURN_COS : GAIT_TURN_COS))
  }

  /**
   * Ticks the rest of the way takes at the least, from node `i` on: the
   * nodes are passed in turn, each from within its reach. `running`: the
   * body has no hop left to take.
   */
  private togo (b: SimBody, i: number, running: boolean): number {
    let far = 0
    for (let m = i; m <= this.index; m++) {
      const n = this.path[m]
      const d = Math.hypot(n.x - b.x, n.z - b.z) - this.reach[m]
      if (d > 0 && d + this.rest[m] > far) far = d + this.rest[m]
    }
    return far / (running ? Math.max(this.vRun, Math.hypot(b.vx, b.vz) + this.aRun) : GAIT_VMAX)
  }

  /**
   * One tick with the feet down (or falling off what they walked off): the
   * node index after it, -1 when the schedule is lost. `air`: the body was
   * off the ground before the tick (a touchdown in it is a landing).
   */
  private runTick (b: SimBody, i: number, air: boolean): number {
    // steered as the executor walks a step (plugin.ts, wallSteer.ts): off a
    // wall it runs along, no sprint against one, along one in its way; and
    // falling, as the executor falls (fallAim)
    const n = air ? this.path[fallAim(this.path, Math.min(i, this.path.length - 1), b.x, b.z, b.vx, b.vz)] : this.aim(i)
    const c = this.c
    const h = this.h
    const nodeDx = n.x - b.x
    const nodeDz = n.z - b.z
    h.dx = nodeDx
    h.dz = nodeDz
    c.sprint = true
    c.forward = true
    // (the rounding policy: round the first box in the way, if its corner
    // lies within ROUND_CORRIDOR of the leg the body is on)
    let rounded = false
    if (this.roundMode && !air && this.roundCorner(b, nodeDx, nodeDz, n.y, this.round)) {
      const px = b.x + this.round.dx
      const pz = b.z + this.round.dz
      const pv = i > 0 ? this.path[Math.min(i, this.path.length) - 1] : null
      let ok = true
      if (pv !== null) {
        const ax = n.x - pv.x
        const az = n.z - pv.z
        const l2 = ax * ax + az * az
        const tt = l2 > 1e-9 ? Math.max(0, Math.min(1, ((px - pv.x) * ax + (pz - pv.z) * az) / l2)) : 0
        ok = Math.hypot(pv.x + ax * tt - px, pv.z + az * tt - pz) <= ROUND_CORRIDOR
      }
      if (ok) {
        h.dx = this.round.dx
        h.dz = this.round.dz
        rounded = true
      }
    }
    // (one gathering of the boxes round the body for all the probes; none
    // within a probe's reach at body level, and there is no wall to steer by)
    this.k.probeBegin(b.x, b.y, b.z, WALL_REACH, 0.6)
    if (!rounded && !air && this.k.probeHit(b.x, b.y, b.z, WALL_REACH, PROBE_INSET)) {
      const walk = (n.parkour !== true || n.walked === true) && n.y - b.y <= STEP_UP_MIN
      if (walk && Math.hypot(nodeDx, nodeDz) > 0.15) sideStandoff(this.hit, b.x, b.y, b.z, h)
      c.sprint = !againstWall(this.hit, this.k.probeHit(b.x, b.y, b.z, SPRINT_WALL_MARGIN, PROBE_INSET), b.x, b.y, b.z, h.dx, h.dz)
      if (walk && Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH) wallSlide(this.hit, b.x, b.y, b.z, nodeDx, nodeDz, nodeDx, nodeDz, h)
    }
    const len = Math.hypot(h.dx, h.dz)
    if (len > 1e-9) { c.hx = h.dx / len; c.hz = h.dz / len }
    c.left = false
    // (the strafe is a sprinting gait: against a wall the body walks, and looks where it goes)
    if (this.strafe && c.sprint && !air) kStrafe(c)
    c.jump = false
    if (this.stepWanted) {
      this.stepWanted = false
      this.step = { yaw: Math.atan2(-c.hx, -c.hz), sprint: c.sprint, strafe: c.left }
    }
    this.px = b.x
    this.pz = b.z
    this.k.step(b, c)
    this.work++
    if (b.inLiquid || b.y < this.low(i) - 1) return -1
    return this.pass(b, i, 1, air && b.onGround)
  }

  private readonly round = { dx: 0, dz: 0 }

  /**
   * The first box at body height (ROUND_BAND up to the head) in the way of
   * the straight step (ndx, ndz) within ROUND_REACH, by a slab test against
   * the box grown by the half-width: the heading at its grown corner that
   * minimises the way to the node (the corner whose midpoint with the body is
   * outside it), into `h`. False when nothing is in the way, or the corner is
   * where the body already is. A box no higher than the node's feet (`ny`) is
   * the step the path climbs, not a box in the way: rounding the riser it was
   * to jump onto cost spiral3-c 13 ticks and climb2 a hop a step.
   */
  private roundCorner (b: SimBody, ndx: number, ndz: number, ny: number, h: { dx: number, dz: number }): boolean {
    const k = this.k
    const lo = Math.max(b.y, ny) + ROUND_BAND
    const hw = k.halfWidth + 0.01
    const d = Math.hypot(ndx, ndz)
    if (d < 0.3) return false
    const reach = Math.min(d, ROUND_REACH)
    const ux = ndx / d
    const uz = ndz / d
    const ex = b.x + ux * reach
    const ez = b.z + uz * reach
    if (lo >= b.y + k.height - 0.01) return false
    const n = k.boxesIn(Math.min(b.x, ex) - hw - 0.1, lo, Math.min(b.z, ez) - hw - 0.1, Math.max(b.x, ex) + hw + 0.1, b.y + k.height - 0.01, Math.max(b.z, ez) + hw + 0.1)
    const buf = k.scratchBoxes
    let bestT = Infinity
    let bi = -1
    for (let j = 0; j < n; j++) {
      const o = j * 6
      if (!(buf[o + 4] > lo - 0.01 && buf[o + 1] < b.y + k.height)) continue
      const x0 = buf[o] - hw
      const x1 = buf[o + 3] + hw
      const z0 = buf[o + 2] - hw
      const z1 = buf[o + 5] + hw
      // (already against it: not in the way, the walk rules have it)
      if (b.x > x0 + 1e-6 && b.x < x1 - 1e-6 && b.z > z0 + 1e-6 && b.z < z1 - 1e-6) continue
      let t0 = 0
      let t1 = reach
      if (Math.abs(ux) < 1e-9) {
        if (b.x <= x0 || b.x >= x1) continue
      } else {
        let a = (x0 - b.x) / ux
        let c = (x1 - b.x) / ux
        if (a > c) { const q = a; a = c; c = q }
        t0 = Math.max(t0, a)
        t1 = Math.min(t1, c)
      }
      if (Math.abs(uz) < 1e-9) {
        if (b.z <= z0 || b.z >= z1) continue
      } else {
        let a = (z0 - b.z) / uz
        let c = (z1 - b.z) / uz
        if (a > c) { const q = a; a = c; c = q }
        t0 = Math.max(t0, a)
        t1 = Math.min(t1, c)
      }
      if (t0 < t1 - 1e-6 && t0 < bestT) { bestT = t0; bi = j }
    }
    if (bi < 0) return false
    const o = bi * 6
    const cx0 = buf[o] - hw
    const cx1 = buf[o + 3] + hw
    const cz0 = buf[o + 2] - hw
    const cz1 = buf[o + 5] + hw
    let best = Infinity
    let px = 0
    let pz = 0
    for (const x of [cx0, cx1]) {
      for (const z of [cz0, cz1]) {
        const mx = (b.x + x) / 2
        const mz = (b.z + z) / 2
        if (mx > cx0 + 1e-6 && mx < cx1 - 1e-6 && mz > cz0 + 1e-6 && mz < cz1 - 1e-6) continue
        const len = Math.hypot(x - b.x, z - b.z) + Math.hypot(b.x + ndx - x, b.z + ndz - z)
        if (len < best) { best = len; px = x; pz = z }
      }
    }
    if (best === Infinity) return false
    h.dx = px - b.x
    h.dz = pz - b.z
    return Math.hypot(h.dx, h.dz) >= 0.05
  }

  /**
   * From a grounded state: every schedule on from here (`hops` of the gait
   * taken; `used`: the last node a jump the path asked for was taken at).
   * Consumes nothing of `b`. False when all of them were lost (dominated).
   */
  private dfs (b: SimBody, i: number, t: number, hops: number, depth: number, used: number): boolean {
    const w = copyBody(this.scratch(this.win, depth), b)
    while (this.opened.length <= depth) {
      this.opened.push(new Int32Array(GAIT_BUDGET + 2))
      this.hopped.push(new Uint8Array(GAIT_BUDGET + 2))
    }
    const opened = this.opened[depth]
    const hopped = this.hopped[depth]
    let n = 0
    let alive = false
    for (let d = 0; ; d++) {
      // (the ticks waited since the landing: past GAIT_DELAY there is no hop of the gait left in this stretch of ground)
      const wait = Math.min(d, GAIT_DELAY + 1)
      const gait = wait <= GAIT_DELAY && hops < GAIT_HOPS
      if (this.done || t + this.togo(w, i, !gait && !this.lipAhead[i]) >= this.best) { alive = true; break }
      if (this.work > this.limit) { this.spent = true; alive = true; break }
      const dom = this.dominated(w, i, t, hops, wait)
      if (dom !== OPEN) { alive = dom === ALIVE; break }
      opened[n] = this.label(w, t, hops, wait)
      hopped[n] = 0
      n++
      // (the jump the path asks for here, on any tick and once; or a hop of the gait, counted and timed)
      const asked = i > used && this.needsJump(w, i)
      if ((asked || gait) && depth < GAIT_DEPTH && t + 1 < this.best && this.mayHop(w, i, asked)) {
        if (this.jump(w, i, t, asked ? hops : hops + 1, depth, asked ? i : used, asked)) hopped[n - 1] = 1
        if (this.done) { alive = true; break }
      }
      i = this.stride(w, i, t, depth)
      if (i < 0) { alive = this.live; break }
      t = this.at
    }
    // (a state's schedules: the jump from it, and all of the states after it)
    for (let j = n - 1; j >= 0; j--) {
      alive = alive || hopped[j] === 1
      if (opened[j] >= 0) labelState[opened[j]] = alive ? ALIVE : DEAD
    }
    return alive
  }

  /** Does body `b` need a jump to get to node `i`: a parkour node, a rise above its step? */
  private needsJump (b: SimBody, i: number): boolean {
    if (!this.lip[i]) return false
    const n = this.path[i]
    return (n.parkour === true && n.walked !== true) || n.y > b.y + SIM.stepHeight
  }

  /**
   * The jump from grounded body `w` (kept), as thrown — forward held to the
   * touchdown — and, where that flies past the node it is for, eased:
   * forward let go at the latest tick of the flight that brings it down
   * short of the node's far side (found on bare flights, by bisection: the
   * longer forward is held, the further the flight). For a jump the path
   * asks for, and for the path's last node: thrown at full stretch the body
   * comes down past the landing and walks back to it (arena mcc-6-2a, three
   * ticks at the end of every run), or past a platform's far edge. A hop of
   * the gait is not eased: one that comes down later does the same. False
   * when all were lost.
   */
  private jump (w: SimBody, i: number, t: number, hops: number, depth: number, used: number, asked: boolean): boolean {
    let before = this.best
    let alive = this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, EASE_NEVER, false)
    if (depth === 0 && this.best < before) this.keep(EASE_NEVER)
    if (!this.over[depth] || this.done || !(asked || (this.last && this.overAt[depth] === this.index))) return alive
    let lo = 1
    let hi = this.flew[depth] - 1
    let ease = 0
    while (lo <= hi) {
      const mid = (lo + hi) >> 1
      this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, mid, true)
      if (this.over[depth]) hi = mid - 1
      else { ease = mid; lo = mid + 1 }
    }
    if (ease === 0) return alive
    before = this.best
    if (this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, ease, false)) alive = true
    if (depth === 0 && this.best < before) this.keep(ease)
    return alive
  }

  /** The root's jump just flown is the best so far: its looks and its ease are the result's. */
  private keep (ease: number): void {
    const looks = this.looks[0]
    this.flight.length = 0
    for (let f = 0; f < looks.length; f++) this.flight.push(looks[f])
    this.ease = ease
  }

  /**
   * A jump from grounded body `h` (consumed; `from` is it, kept), flown with
   * forward held up to tick `ease` of it, then every schedule on from its
   * landing. `hops`: the gait's count with this one; `used`: the last node a
   * jump the path asked for was taken at; `asked`: this is one. False when
   * all were lost; `over` and `flew` say how it flew. `bare`: the flight
   * alone, for those two.
   */
  private hop (h: SimBody, from: SimBody, i: number, t: number, hops: number, depth: number, used: number, asked: boolean, ease: number, bare: boolean): boolean {
    const c = this.c
    const root = depth === 0
    while (this.looks.length <= depth) this.looks.push([])
    const looks = this.looks[depth]
    this.over[depth] = false
    this.flew[depth] = 0
    // (short of a gap's far side is a fall; short of a rise is a tick against it)
    const held = (root || (asked && this.path[i].parkour === true)) && !this.scouting && !bare
    if (held) looks.length = 0
    this.heldAt[depth] = held
    this.holdFrom[depth] = from
    this.holdEase[depth] = ease
    this.holdCatch[depth] = false
    this.holdOk[depth] = -1
    this.unheld[depth] = false
    this.labelsAt[depth] = this.labels
    let left = false
    let passed = -1
    let fall = 0
    let lastP = -1
    for (let f = 0; ; f++) {
      const n = this.aim(i)
      kAim(c, h, n.x, n.z)
      c.jump = f === 0
      c.sprint = true
      c.left = false
      c.forward = f < ease
      if (this.strafe && f > 0 && f < ease) kStrafe(c)
      if (held) looks.push(Math.atan2(-c.hx, -c.hz))
      this.px = h.x
      this.pz = h.z
      const y0 = h.y
      this.k.step(h, c)
      this.work++
      c.forward = true
      this.flew[depth] = f + 1
      t++
      const drop = Math.max(0, y0 - h.y)
      fall += drop
      if (h.inLiquid || t > GAIT_BUDGET) return false
      if (passed < 0 && t >= this.best) return true
      if (!h.onGround) left = true
      // (the hop's own box is as tall as the hop: plugin.ts HOP_ARRIVE_DY)
      const low = this.low(i)
      const i1 = i
      i = this.pass(h, i, 1.45, left && h.onGround)
      for (let m = i1; m < i && m <= this.index; m++) if (this.path[m].parkour === true) lastP = m
      if (left && h.onGround && !this.over[depth] && lastP >= 0) {
        const o = this.path[lastP]
        if (Math.abs(o.x - h.x) > EASE_PAST || Math.abs(o.z - h.z) > EASE_PAST) { this.over[depth] = true; this.overAt[depth] = lastP }
      }
      // (a jump that never left the ground: under a roof too low to rise under)
      if (!left) { if (f > 0) return false; continue }
      // (past the node it is for: the node not passed, and behind the body's motion)
      if (i <= this.index && !this.over[depth]) {
        const o = this.path[i]
        const ox = o.x - h.x
        const oz = o.z - h.z
        if (ox * h.vx + oz * h.vz < 0 && (Math.abs(ox) > 0.35 || Math.abs(oz) > 0.35)) { this.over[depth] = true; this.overAt[depth] = i }
      }
      if (i > this.index && passed < 0) passed = this.passedAt(h, t)
      // (caught on the climbable the path jumps for: a landing, and the climb on to the goal)
      const catching = this.catchAt >= 0 && i >= this.catchAt
      if (catching && this.catches(h)) {
        if (bare) return false
        this.holdY[depth] = h.y
        this.holdCarried[depth] = false
        this.holdCatch[depth] = true
        if (this.scouting) return false
        const alive = this.climb(h, i, t, depth + 1)
        if (!this.unheld[depth]) return alive
        for (let a = this.labelsAt[depth]; a < this.labels; a++) labelState[a] = VOID
        return false
      }
      if (h.onGround) {
        this.holdCatch[depth] = false
        if (h.y < low - HOP_MAX_DIP) return false
        // (slime throws a body back up: its landings are the oracle's, sneak and all)
        if ((this.k.kindAt(h.x, h.y - 0.2, h.z) & SIM_SLIME) !== 0) return false
        if (bare) return false
        this.holdY[depth] = h.y
        this.holdCarried[depth] = this.k.carried(h)
        const stood = this.landed(h, fall, drop)
        if (this.scouting) {
          if (passed < 0) { this.landI = i; this.landT = t + stood }
          return false
        }
        const alive = passed >= 0 ? this.end(h, passed, t, stood, depth + 1) : this.dfs(h, i, t + stood, hops, depth + 1, used)
        if (!this.unheld[depth]) return alive
        for (let a = this.labelsAt[depth]; a < this.labels; a++) labelState[a] = VOID
        return false
      }
      // (falling to the climbable's foot is falling to the catch)
      if (h.y < (catching ? Math.min(low, this.catchLow) : low) - 1) return false
    }
  }
}

/**
 * The node a body falling off what it walked off steers at (an index into
 * `path`): the next one, `i` — or, once that one is behind its motion, the
 * first after it on the same floor that is not. A node below a ledge is come
 * down past, not circled back to in the air: steering at it all the way, a
 * run off a ledge came down at a standstill on top of it, and a hop off the
 * ledge into a fall that hurts was the faster way down.
 */
export function fallAim (path: ReadonlyArray<GaitNode>, i: number, x: number, z: number, vx: number, vz: number): number {
  while (i + 1 < path.length) {
    const n = path[i]
    const m = path[i + 1]
    if ((n.x - x) * vx + (n.z - z) * vz >= 0) break
    if ((m.parkour === true && m.walked !== true) || Math.abs(m.y - n.y) > SIM.stepHeight) break
    i++
  }
  return i
}

/**
 * Is a body at (x, z) on the leg from node `n` to node `m`, level with `n`
 * or past it: within the half-width of a node's box of the line, beyond the
 * node along it? The planner's own line, further along — a node there is
 * passed (plugin.ts, the arrival loop; Search.pass). The line of a jump too:
 * a body that came down past its take-off node, toward the gap, takes the
 * jump from there (arena spiral3-d: it walked back to the node's box and
 * turned round again, nine ticks).
 */
export function onLegPast (x: number, z: number, n: { x: number, z: number }, m: { x: number, z: number }): boolean {
  const lx = m.x - n.x
  const lz = m.z - n.z
  const len = Math.hypot(lx, lz)
  if (len < 0.3) return false
  const px = x - n.x
  const pz = z - n.z
  return px * lx + pz * lz >= 0 && Math.abs(px * lz - pz * lx) <= 0.35 * len
}

const walker = newBody(0, 0, 0)
const walkCtl: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: -1 }

/**
 * Is node `n` walked to from `from` — sprinting at it, feet down, the
 * arrival box reached on the ground at its level? A parkour node that is, is
 * a jump only in the plan (the planner prices a sprint jump over flat ground
 * below the steps it replaces), and the gait treats it as ground.
 */
export function kWalkable (k: PlayerSim, from: { x: number, y: number, z: number }, n: GaitNode): boolean {
  if (Math.abs(n.y - from.y) > 0.6) return false
  const b = walker
  b.x = from.x; b.y = from.y; b.z = from.z
  b.vx = 0; b.vy = 0; b.vz = 0
  b.onGround = true; b.collidedH = false; b.collidedV = true; b.jumpTicks = 0; b.inLiquid = false
  const c = walkCtl
  const low = Math.min(from.y, n.y) - 0.6
  const ticks = Math.ceil(Math.hypot(n.x - from.x, n.z - from.z) / 0.2) + 6
  for (let t = 0; t < ticks; t++) {
    kAim(c, b, n.x, n.z)
    k.step(b, c)
    if (b.inLiquid || b.y < low) return false
    if (b.onGround && Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < 0.5) return true
  }
  return false
}

/**
 * The gait from grounded body `b0` down `path` to `goal`: the ticks of the
 * best schedule that jumps now and of one that keeps its feet this tick
 * (GaitResult). A jump is worth taking when `hop` is finite and `run` is not;
 * a tick on the ground is the kernel's to take (`step`) when `run` is finite;
 * with neither, the kernel has no way there and the tick is someone else's.
 */
function policySearch (k: PlayerSim, b0: SimBody, path: ReadonlyArray<GaitNode>, goal: GaitGoal, pick?: GaitPick): GaitResult {
  const a = new Search(k, path, goal).run(b0)
  if (pick === undefined || !GAIT_ROUND) return a
  // (and the rounding policy: taken where it is ROUND_MARGIN sooner, kept
  // once taken until the plain one is ROUND_MARGIN sooner — no flapping)
  const s2 = new Search(k, path, goal)
  s2.roundMode = true
  const b = s2.run(b0)
  const ma = Math.min(a.hop, a.run)
  const mb = Math.min(b.hop, b.run)
  const takeB = pick.round ? !(ma + ROUND_MARGIN + 1e-6 < mb) : mb + ROUND_MARGIN + 1e-6 < ma
  pick.round = takeB && mb !== Infinity
  if (takeB && mb !== Infinity) return { ...b, flight: b.flight.slice(), work: a.work + b.work }
  return { ...a, work: a.work + b.work }
}
/**
 * The strafe is an option, never a restriction: where the strafed search
 * finds a hop but no run (spent or not), the search without it is asked too
 * and taken when more than STRAFE_MARGIN ticks sooner. From arena
 * spiral3-c's riser leg the strafed search, run to exhaustion, found no run
 * better than r67.8 where the plain one found r48.9, and hopped away from
 * the node and back (+8 ticks on both cores; 2026-10-10 audit, driveSim
 * from the traced state: -11 on the segment). Asked on every tick it took
 * -17 there for half again the gait work.
 */
export function gaitSearch (k: PlayerSim, b0: SimBody, path: ReadonlyArray<GaitNode>, goal: GaitGoal, pick?: GaitPick): GaitResult {
  const r = policySearch(k, b0, path, goal, pick)
  // (only between two kernel schedules: a strafed search with no schedule at
  // all leaves the tick to the gates, as before - asked there, the plain hop
  // it found put mcc-5-2 0.09 off its line, and two jumps on the momentum
  // chain onto a bar arm was not signed and the brake ran off the arm)
  if (!STRAFE_FALLBACK || goal.strafe !== true || r.run !== Infinity || r.hop === Infinity) return r
  const u = new Search(k, path, { ...goal, strafe: false }).run(b0)
  if (Math.min(u.hop, u.run) + STRAFE_MARGIN + 1e-6 < Math.min(r.hop, r.run)) return { ...u, flight: u.flight.slice(), work: r.work + u.work }
  return { ...r, work: r.work + u.work }
}

