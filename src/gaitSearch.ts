// Jump timing depends on the landing and the next take-off, so schedules use
// the executor's physics and steering rather than a fixed hopping policy.

import { SIM, SIM_SLIME, SIM_CLIMBABLE, SIM_HAZARD, copyBody, newBody } from './playerSim.js'
import type { PlayerSim, SimBody, SimControl } from './playerSim.js'
import { sideStandoff, wallSlide, againstWall, SPRINT_WALL_MARGIN, STEP_UP_MIN, WALK_STEP_REACH, PROBE_INSET } from './wallSteer.js'
import type { Heading } from './wallSteer.js'
import { landingHurts, SAFE_FALL } from './fallDamage.js'

export interface GaitNode {
  x: number
  y: number
  z: number
  /** Jump destination, retired using the executor's landing rules. */
  parkour?: boolean
  /** The destination is reachable on foot, so it does not require a jump. */
  walked?: boolean
}

export interface GaitGoal {
  /** Goal node index; the search may stop earlier at its jump or scouting horizon. */
  index: number
  /** The goal ends the path and requires a grounded arrival. */
  last: boolean
  /** Previous node, used as the take-off origin when the first node is a jump destination. */
  from?: { x: number, z: number } | null
  /** Whether the server counts the whole fall for damage; defaults to true. */
  wholeFall?: boolean
  /** Enable the 45° strafe on the ground and in the air. */
  strafe?: boolean
  /** Retire parkour nodes when landing past them; defaults to true. */
  landRetire?: boolean
  /**
   * Climbable catch node index; absent or -1 disables catches. A catch counts as
   * a landing, followed by a climb to the goal in the same column.
   */
  catchAt?: number
  /** Find the fastest ground-first schedule instead of stopping at the first qualifying one. */
  best?: boolean
}

/** Ground control for one tick, with forward held. */
export interface GaitStep { yaw: number, sprint: boolean, strafe: boolean }

export interface GaitResult {
  /** Best arrival time in ticks when jumping now; Infinity if none succeeds. */
  hop: number
  /** Actual goal index, which may precede GaitGoal.index at the search horizon. */
  goal: number
  goalLast: boolean
  /**
   * Arrival time when staying grounded this tick; Infinity if no schedule qualifies.
   * It must match the hop with jumps ahead, or beat it over open ground.
   */
  run: number
  /**
   * The work budget is exhausted. A finite hop is viable but may have a better
   * ground-first alternative; without a hop, the result is inconclusive.
   */
  spent: boolean
  /** Number of simulated physics ticks. */
  work: number
  /** Yaw for each tick of the chosen jump; the executor must replay these headings. */
  flight: number[]
  /** Hold strafe after take-off while forward is held. */
  strafe: boolean
  /** Tick when forward and strafe are released; EASE_NEVER holds them through touchdown. */
  ease: number
  /** First ground control to replay when `run` is finite. */
  step: GaitStep | null
}

/** Wall-grazing flights must also land with headings offset by this angle on either side. */
export const GRAZE_YAW = 0.02
/**
 * Take-off alignment limits: 20° for the executor's gate, 45° for gait hops.
 * A sprint jump boosts along the current heading, so these limits prune turns.
 */
export const HOP_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_HOP_TURN_DEG : undefined) ?? 20) || 20) * Math.PI / 180)
export const GAIT_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_GAIT_TURN_DEG : undefined) ?? 45) || 45) * Math.PI / 180)
/** Allow path-required jumps within 45° of the body's velocity, including turns between risers. */
const ASKED_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_ASKED_TURN_DEG : undefined) ?? 45) || 45) * Math.PI / 180)
/** Retry the root jump without the angle cut when no schedule succeeds. */
const GAIT_ROOT_UNCUT = typeof process === 'undefined' || process.env.PF_GAIT_ROOT_UNCUT !== '0'
/** Maximum landing depth below the lower of the adjacent path nodes. */
export const HOP_MAX_DIP = 0.5
/**
 * Maximum height above a parkour node for airborne retirement. The landing is
 * committed this close to the ground; holding the node sacrifices momentum.
 */
export const PARKOUR_LAND_DY = 0.5
/** Allow airborne retirement of parkour nodes with ground beyond them. */
const GAIT_GROUND_PASS = typeof process === 'undefined' || process.env.PF_GAIT_GROUND_PASS !== '0'

/** Ground delay before an optional hop, and the optional hops allowed per schedule. */
const GAIT_DELAY = 3
const GAIT_HOPS = 4
/**
 * Path-required jumps do not consume GAIT_HOPS. Looking three jumps ahead
 * judges each landing by the take-off it leaves for subsequent jumps.
 */
const GAIT_LIPS = 3
/** Maximum combined jump depth, also used for scratch-body storage. */
const GAIT_DEPTH = GAIT_HOPS + 2 * GAIT_LIPS
/** The scouting hop chain's tick limit and extra distance for detecting overshot corners or walls. */
const SCOUT_TICKS = 40
const SCOUT_TAIL = 3
/** Maximum ticks per schedule. */
const GAIT_BUDGET = 80
/** Separate simulation budgets for jump-first and ground-first schedules with several jumps to time. */
const GAIT_WORK_HOP = 18000
const GAIT_WORK_RUN = 15000
/** Tick costs for waiting on the server's hurt packet and regaining lost speed. */
const HURT_STAND = 1
const HURT_REGAIN = 2
/** Health penalty in ticks: a damaging landing must save at least this much time. */
const HURT_COST = 6
/** Total tick penalty for a damaging landing: packet wait, health and lost speed. */
export const HURT_TICKS = HURT_STAND + HURT_COST + HURT_REGAIN
/** Maximum wall-probe reach, with a small margin. */
const WALL_REACH = 0.36
/**
 * Optimistic speed bound in blocks per tick. With no hops left, use the larger
 * of the blue-ice running equilibrium and current speed plus one tick's push.
 */
const GAIT_VMAX = 0.5
/** Ice under low ceilings permits faster hops, so pruning needs a higher speed bound. */
const GAIT_VMAX_SLIP = 1.4
const GAIT_VMAX_STATE = process.env.PF_GAIT_VMAX_SLIP !== '0'
/** Slipperiness threshold for the higher speed bound; stone's default reads as 0. */
const SLIP_FAST = 0.6
const RUN_PUSH = 1.3
const RUN_SETTLE = 2.2326
/** Landing retirement distance, extended for nodes reachable on foot on the same platform. */
const LAND_PAST = 1
const LAND_PAST_WALKED = 2.5
/** Minimum node reach for its arrival box and landings past parkour nodes. */
const PASS_REACH = 0.5
const PASS_REACH_LANDED = LAND_PAST_WALKED
/** Tick advantage required to prefer running over hopping on open ground. */
const GAIT_RUN_GAIN = 1
/** Ignore nearby aim nodes in turn tests, where tiny position changes make headings unstable. */
const NEAR_AIM = 0.35
const GAIT_NEARAIM = typeof process === 'undefined' || process.env.PF_GAIT_NEARAIM !== '0'
/** Distance past a retired parkour node that triggers easing, leaving room for the next riser. */
const EASE_PAST = 1
/** Rounding policy time margin and corridor width prevent switching between incompatible schedules. */
const ROUND_MARGIN = 2
const ROUND_CORRIDOR = 0.5
/** Obstacle look-ahead distance and body-height band for corner rounding. */
const ROUND_REACH = 2.5
const ROUND_BAND = 0.61
/** Radius in blocks for gathering hazard cells around the body. */
const HAZARD_SCAN = 12
/** Enable the corner-rounding policy. */
const GAIT_ROUND = typeof process === 'undefined' || process.env.PF_GAIT_ROUND !== '0'
// Reuse the plain search only if every ground step, including scouting,
// makes the same choice under the rounding policy.
const GAIT_REUSE = typeof process === 'undefined' || process.env.PF_GAIT_REUSE !== '0'
/** Tick advantage required to choose the search without strafe. */
const STRAFE_MARGIN = 1
const STRAFE_FALLBACK = typeof process === 'undefined' || process.env.PF_STRAFE_FALLBACK !== '0'
/**
 * Flights must tolerate this starting-position offset on either side because
 * mineflayer rounds the replayed look to 0.15° steps.
 */
const HOP_NUDGE = 0.01
/** GaitResult.ease value for holding forward through touchdown. */
export const EASE_NEVER = 99
/**
 * Climb steering keeps the body centred on the column and ends just below
 * goal height. Nudged catches must reach the same column within CATCH_NUDGE_DY.
 */
const CATCH_DRIFT = 0.91 / 0.09
const CATCH_HOLD = 0.15
const CATCH_LEVEL = 0.01
const CATCH_NUDGE_DY = 1
/** Ladder rise in blocks per tick: (ladderClimbSpeed - gravity) * airdrag. */
const LADDER_RISE = (SIM.ladderClimbSpeed - SIM.gravity) * SIM.airdrag
const WALL_SIDES: ReadonlyArray<readonly [number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1]]
/** Enable climbable catches as landings. */
export const GAIT_CATCH = typeof process === 'undefined' || process.env.PF_GAIT_CATCH !== '0'
/** Enable holding climbable catches before wall contact. */
export const CLIMB_HOLD_ENABLED = typeof process === 'undefined' || process.env.PF_CLIMB_HOLD !== '0'
/** Speed threshold and braking tick limit for a stopped schedule. */
const STOP_SPEED = 0.05
const STOP_TICKS = 8
/**
 * Compare nearby levels and equivalent states; lead dominance requires a clear straight.
 * DOM_LEAD bounds extra travel from decaying speed: 1 / (1 - 0.91) ticks of it.
 */
const DOM_LEVEL = 0.6
const DOM_SAME = 1e-3
const DOM_CLEAR = 5
const DOM_LEAD = 11.2
/**
 * Enable easing before a riser scrape: a shorter, slower landing leaves room
 * for the next jump to clear the riser.
 */
const RISER_EASE = typeof process === 'undefined' || process.env.PF_GAIT_RISER !== '0'
const RISER_TICKS = 3
/** Disable lead dominance before rises. */
const RISE_DOM = typeof process !== 'undefined' && process.env.PF_GAIT_RISEDOM === '1'
const RISER_EASES = 6
/** Fraction of remaining work available for eased flights, preserving the main search's budget. */
const RISER_SHARE = 0.4
const RISER_DEPTH = Number((typeof process !== 'undefined' ? process.env.PF_GAIT_RISER_DEPTH : undefined) ?? 1)
/** Ground-state capacity per phase, including position, velocity, time and remaining choices. */
const LABELS = 4096
const OPEN = 0
const ALIVE = 1
const DEAD = 2
/** Labels under a landing that fails verification cannot dominate other states. */
const VOID = 3
const labelX = new Float64Array(LABELS)
const labelY = new Float64Array(LABELS)
const labelZ = new Float64Array(LABELS)
const labelVx = new Float64Array(LABELS)
const labelVz = new Float64Array(LABELS)
const labelT = new Float64Array(LABELS)
/** Path progress, speed and straight-run status used for lead dominance. */
const labelS = new Float64Array(LABELS)
const labelV = new Float64Array(LABELS)
const labelClear = new Uint8Array(LABELS)
const labelHops = new Uint8Array(LABELS)
const labelWait = new Uint8Array(LABELS)
const labelState = new Uint8Array(LABELS)
const NUDGES: ReadonlyArray<readonly [number, number]> = [[HOP_NUDGE, 0], [-HOP_NUDGE, 0], [0, HOP_NUDGE], [0, -HOP_NUDGE]]

/**
 * Rotate 45° and hold left to keep the push on the travel line. Combined input
 * normalises to 1 rather than 0.98; skip take-off because its boost follows yaw.
 */
export function kStrafe (c: SimControl): void {
  const hx = c.hx
  c.hx = (hx - c.hz) * Math.SQRT1_2
  c.hz = (c.hz + hx) * Math.SQRT1_2
  c.left = true
}

/** Look yaw for travel yaw `yaw` with the left strafe key held. */
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

/** Selected steering policy; rounding persists until the plain policy wins by ROUND_MARGIN ticks. */
export interface GaitPick {
  round: boolean
  /** Set false to repeat the rounding search even when its steering matches the plain policy. */
  reuse?: boolean
}

class Search {
  /** Round obstructing boxes instead of sliding along walls, which can push the body off narrow supports. */
  roundMode = false
  /** Still identical at every rounding opportunity visited by the plain search. */
  roundSame = false
  private best = Infinity
  private work = 0
  /** Simulation work limit for this phase and whether it is exhausted. */
  private limit = GAIT_WORK_HOP
  private spent = false
  /** Stop the ground-first search at its first qualifying schedule. */
  private firstWins = false
  private done = false
  private readonly c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: -1 }
  private readonly win: SimBody[] = []
  private readonly fly: SimBody[] = []
  /** Best root jump's headings and forward-release tick. */
  private readonly flight: number[] = []
  private ease = EASE_NEVER
  /** Headings by depth for flights requiring verification. */
  private readonly looks: number[][] = [[]]
  /** Per-depth replay state, verification status (-1 unknown, 0 failed, 1 passed), and subtree validity. */
  private readonly heldAt: boolean[] = []
  private readonly holdFrom: SimBody[] = []
  private readonly holdEase: number[] = []
  private readonly holdY: number[] = []
  private readonly holdCarried: boolean[] = []
  private readonly holdCatch: boolean[] = []
  private readonly holdOk: number[] = []
  /** Catch node index, column and lowest climbable cell. */
  private catchAt = -1
  private catchX = 0
  private catchZ = 0
  private catchLow = 0
  /** The wall a climbable is on (wallAt). */
  private wallX = 0
  private wallZ = 0
  private readonly labelsAt: number[] = []
  private readonly unheld: boolean[] = []
  /** Per-depth overshot node and flight duration. */
  private readonly over: boolean[] = []
  private readonly overAt: number[] = []
  private readonly flew: number[] = []
  /** Whether the next jump from each landing scrapes a riser while rising. */
  private readonly scraped: boolean[] = []
  /** Eased landings bypass lead dominance because faster bodies ahead may hit the riser. */
  private riserEasing = false
  private readonly nudged = newBody(0, 0, 0)
  /** First ground control, recorded while stepWanted is true. */
  private step: GaitStep | null = null
  private stepWanted = false
  /** Position before the last tick, for fractional arrival timing. */
  private px = 0
  private pz = 0
  /** When the last stride ended. */
  private at = 0
  /** Collision probe and steering heading shared with the executor. */
  private readonly hit = (x: number, y: number, z: number): boolean => this.k.probeHit(x, y, z, -PROBE_INSET, PROBE_INSET)
  private readonly h: Heading = { dx: 0, dz: 0 }
  private readonly strafe: boolean
  /** Whether each node requires a jump: parkour or a rise above step height. */
  private readonly lip: boolean[] = []
  private rootY = 0
  /** Per-node path distance from the root and clear distance to the next turn. */
  private readonly cum: number[] = []
  private readonly clear: number[] = []
  /** Per-node minimum distance to the goal, remaining-jump flag and arrival reach. */
  private readonly rest: number[] = []
  private readonly lipAhead: boolean[] = []
  private readonly reach: number[] = []
  /** Maximum running equilibrium speed and one tick's push for the lower bound. */
  private vRun = 0
  /** Optimistic speed bound for this search. */
  private vMax = GAIT_VMAX
  private aRun = 0
  /** Phase labels, labels opened by each dfs call, and whether each hop remains viable. */
  private labels = 0
  private readonly opened: Int32Array[] = []
  private readonly hopped: Uint8Array[] = []
  /** State measures from the last dominance check, for recording a label. */
  private stateS = 0
  private stateV = 0
  private stateClear = false
  /** Whether the last stride reaches the goal successfully. */
  private live = false
  /** Scouting stops each hop at touchdown and records its node index and time. */
  private scouting = false
  private landI = -1
  private landT = 0
  /** Goal index after applying the jump and scouting horizons. */
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
    const none: GaitResult = { hop: Infinity, goal: this.index, goalLast: this.last, run: Infinity, spent: false, work: 0, flight: this.flight, strafe: this.strafe, ease: EASE_NEVER, step: null }
    if (i0 > this.index) return none
    this.scout(root, i0)
    this.vRun = this.k.speedBase * RUN_PUSH * RUN_SETTLE
    this.aRun = this.k.speedBase * RUN_PUSH
    // Slippery ground within reach needs the higher pruning bound.
    this.vMax = GAIT_VMAX
    if (GAIT_VMAX_STATE) {
      let slip = this.k.slipUnder(root.x, root.y, root.z) > SLIP_FAST
      for (let j = 0; j <= this.index && !slip; j++) slip = this.k.slipUnder(this.path[j].x, this.path[j].y, this.path[j].z) > SLIP_FAST
      this.vMax = slip ? GAIT_VMAX_SLIP : GAIT_VMAX
    }
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
    // Beyond an intermediate goal the search needs only arrival; a final goal requires stopping.
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
      // A faster body can hit a riser that a slower body clears.
      const rise = RISE_DOM && m.y > n.y + SIM.stepHeight
      this.clear[j] = turn || rise ? 0 : bl + this.clear[j + 1]
    }
    const jumps = this.lipAhead[0]

    let hop = Infinity
    this.best = GAIT_BUDGET + 1
    const asked = this.needsJump(root, i0)
    const rootCut = !this.mayHop(root, i0, asked)
    if (!rootCut) {
      this.jump(root, i0, 0, asked ? 0 : 1, 0, asked ? i0 : -1, asked)
      if (this.best <= GAIT_BUDGET) hop = this.best
    }
    const hopSpent = this.spent

    // With jumps ahead, ties favour the ground's remaining choices. Over open
    // ground, require a gain so arbitrary horizon placement does not delay hops.
    const bound = hop === Infinity ? GAIT_BUDGET + 1 : jumps ? hop + 1e-6 : hop - GAIT_RUN_GAIN + 1e-6
    this.best = bound
    // A qualifying schedule settles the hop comparison; without a hop, seek the fastest run.
    this.firstWins = this.goal.best !== true && (hop !== Infinity || !GAIT_NEARAIM)
    this.heldAt[0] = false
    this.limit = this.work + GAIT_WORK_RUN
    this.spent = false
    this.labels = 0
    this.stepWanted = true
    // stride mutates root, so retain the take-off state for the fallback jump.
    const held = rootCut && GAIT_ROOT_UNCUT ? copyBody(newBody(0, 0, 0), root) : null
    const i = this.stride(root, i0, 0, 0)
    if (i >= 0) this.dfs(root, i, this.at, 0, 1, -1)
    const run = this.best < bound ? this.best : Infinity
    const runSpent = this.spent
    // Angle cuts limit work, not physical feasibility; retry when both searches fail.
    if (held !== null && hop === Infinity && run === Infinity) {
      this.best = GAIT_BUDGET + 1
      this.firstWins = false
      this.done = false
      this.limit = this.work + GAIT_WORK_HOP
      this.spent = false
      this.labels = 0
      this.stepWanted = false
      this.jump(held, i0, 0, asked ? 0 : 1, 0, asked ? i0 : -1, asked)
      if (this.best <= GAIT_BUDGET) hop = this.best
    }
    return {
      hop,
      goal: this.index,
      goalLast: this.last,
      run,
      spent: run === Infinity && (this.spent || hopSpent || runSpent),
      work: this.work,
      flight: this.flight,
      strafe: this.strafe,
      ease: this.ease,
      step: run === Infinity ? null : this.step
    }
  }

  /** Limit the horizon to the hop chain plus a tail; otherwise short hops leave redundant running work. */
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

  /** Replay required flights from nudged starts; brittle future take-offs cannot justify a schedule. */
  private holds (from: SimBody, looks: number[], ease: number, y: number, carried: boolean, caught: boolean): boolean {
    const c = this.c
    const n = looks.length
    for (const [nx, nz] of NUDGES) {
      const b = copyBody(this.nudged, from)
      b.x += nx
      b.z += nz
        // A nudged start inside a wall is not a possible body position.
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
      // Nudged catches must still reach the goal without falling off the column.
      if (caught) {
        if (!down || b.onGround || b.y < y - CATCH_NUDGE_DY || !this.climbsUp(b)) return false
        continue
      }
      if (!down || Math.abs(b.y - y) > 1e-6 || this.k.carried(b) !== carried) return false
    }
    return true
  }

  /** Simulate a ground tick and any resulting fall; return the landing node or -1 when the schedule ends. */
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
   * A completed schedule must remain supported after braking; fast landings can
   * slide off narrow supports. Damaging landings also pay for lost speed.
   */
  private end (b: SimBody, passed: number, t: number, stood: number, depth: number): boolean {
    const cost = stood > 0 ? Math.max(passed, t) + stood + HURT_REGAIN : passed
    if (cost >= this.best) return true
    if (!this.stops(b)) return false
    if (!this.verified(depth)) return false
    this.record(cost)
    return true
  }

  /** Whether airborne body `b` catches a climbable in the catch node's column. */
  private catches (b: SimBody): boolean {
    return !b.onGround && Math.floor(b.x) === this.catchX && Math.floor(b.z) === this.catchZ &&
      (this.k.kindAt(b.x, b.y, b.z) & SIM_CLIMBABLE) !== 0
  }

  /** Set wallX/wallZ from the climbable's thin box or a neighbouring full block for vines. */
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

  /** Climb heading presses into the wall and counters drift away from the column's centre. */
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

  /** Consume caught body `b` to check that it reaches goal height without leaving the climbable. */
  private climbsUp (b: SimBody): boolean {
    const goal = this.path[this.index]
    const c = this.c
    for (let t = 0; t < GAIT_BUDGET; t++) {
      if (b.y >= goal.y - CATCH_LEVEL) return true
      if (!this.k.onClimbable(b) || b.inLiquid || (b.onGround && b.y < goal.y - CATCH_LEVEL)) return false
      c.forward = true; c.back = false; c.left = false; c.jump = CLIMB_HOLD_ENABLED; c.sprint = false; c.sneak = !CLIMB_HOLD_ENABLED || b.vy < 0
      this.climbHeading(b, goal)
      this.k.step(b, c)
      this.work++
      c.sneak = false
      c.sprint = true
    }
    return false
  }

  /**
   * Climb to goal height, timed to a fraction of a tick. Steering counters drift;
   * sneaking arrests falls. Leaving the climbable below the goal loses the schedule.
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
      // Remaining climb time cannot beat the ladder's rise speed.
      if (t + (goal.y - CATCH_LEVEL - h.y) / LADDER_RISE >= this.best) return true
      if (!this.k.onClimbable(h)) return false
      c.forward = true
      c.back = false
      c.left = false
      c.jump = CLIMB_HOLD_ENABLED
      c.sprint = false
      // Sneaking arrests the initial fall but weakens steering while rising.
      c.sneak = !CLIMB_HOLD_ENABLED || h.vy < 0
      this.climbHeading(h, goal)
      y0 = h.y
      this.k.step(h, c)
      this.work++
      t++
      c.sneak = false
      c.sprint = true
      if (h.inLiquid || t > GAIT_BUDGET) return false
      // A ledge below the goal is not a completed climb.
      if (h.onGround && h.y < goal.y - CATCH_LEVEL) return false
      i = this.pass(h, i, 1.45, false)
    }
  }

  /** Verify candidate schedules lazily to avoid replaying every landing; failed flights invalidate their subtrees. */
  private verified (depth: number): boolean {
    for (let d = 0; d < depth; d++) {
      if (!this.heldAt[d]) continue
      if (this.holdOk[d] < 0) this.holdOk[d] = this.holds(this.holdFrom[d], this.looks[d], this.holdEase[d], this.holdY[d], this.holdCarried[d], this.holdCatch[d]) ? 1 : 0
      if (this.holdOk[d] === 0) { this.unheld[d] = true; return false }
    }
    return true
  }

  /**
   * Earlier states dominate equivalent states or safe straight-line leads, with enough choices left.
   * Turns can favour slower bodies; failed states match only exactly, and unresolved matches break cycles.
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
      if (!same && !(clear && !this.riserEasing && labelClear[a] === 1 && labelS[a] - s >= DOM_LEAD * Math.max(0, v - labelV[a]))) continue
      if (state === ALIVE) return ALIVE
      open = true
    }
    return open ? DEAD : OPEN
  }

  /** Record ground state `b` and the measures from its dominance check at time `t`. */
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

  /** Check that body `b` stays supported while braking against its velocity. */
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

  /** Damaging landings reset horizontal speed and cost packet-wait and health-penalty ticks. */
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

  /** Lowest allowed height between adjacent nodes; below it, the body leaves the path. */
  private low (i: number): number {
    const a = i > 0 ? this.path[Math.min(i, this.path.length) - 1].y : this.rootY
    return Math.min(a, this.aim(i).y)
  }

  /** Ground beyond a parkour node permits airborne retirement without steering back at a passed node. */
  private holeBeyond (n: GaitNode, b: SimBody): boolean {
    let ux = b.vx
    let uz = b.vz
    let len = Math.hypot(ux, uz)
    if (len < 0.05) {
      ux = n.x - b.x
      uz = n.z - b.z
      len = Math.hypot(ux, uz)
      if (len < 1e-6) return false
    }
    ux /= len
    uz /= len
    for (let d = 1; d <= 2; d++) {
      const x = n.x + ux * d
      const z = n.z + uz * d
      if (!this.k.boxHits(x, n.y, z, -PROBE_INSET, PROBE_INSET) && !this.k.boxHits(x, n.y - 0.55, z, -PROBE_INSET, PROBE_INSET)) return true
    }
    return false
  }

  /**
   * Retire nodes using the executor's arrival and landing rules. Hold parkour
   * nodes over holes until near landing; landing short still requires a take-off.
   */
  private pass (b: SimBody, i: number, dy: number, landing: boolean): number {
    while (i <= this.index) {
      const n = this.path[i]
      const boxed = Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < dy
      if (n.parkour === true) {
        if (!(boxed && (b.y - n.y <= PARKOUR_LAND_DY || (GAIT_GROUND_PASS && i < this.index && !this.holeBeyond(n, b)))) &&
            !(landing && this.cameDownPast(b, i))) break
      } else if (!boxed && !(b.onGround && i + 1 < this.path.length && Math.abs(n.y - b.y) < dy && onLegPast(b.x, b.z, n, this.path[i + 1]))) break
      // Final arrival requires the goal's cell; an off-centre node's arrival box can span two cells.
      if (this.last && i === this.index && (!b.onGround || Math.floor(b.x) !== Math.floor(n.x) || Math.floor(b.z) !== Math.floor(n.z))) break
      i++
    }
    return i
  }

  /** Whether a touchdown is within a block of the parkour node and past it along the incoming flight. */
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

  /** Fractional arrival time avoids whole-tick ties that repeatedly defer a jump. */
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

  /** Whether a gait hop or path-required jump meets the take-off alignment limit. */
  private mayHop (b: SimBody, i: number, asked: boolean): boolean {
    let n = this.aim(i)
    let dx = n.x - b.x
    let dz = n.z - b.z
    let head = Math.hypot(dx, dz)
    // Nearby nodes give unstable headings; test the turn toward the next node instead.
    if (GAIT_NEARAIM && head < NEAR_AIM && i + 1 <= this.index) {
      n = this.aim(i + 1)
      dx = n.x - b.x
      dz = n.z - b.z
      head = Math.hypot(dx, dz)
    }
    const speed = Math.hypot(b.vx, b.vz)
    return !(speed > 0.1 && head > 1e-6 && (b.vx * dx + b.vz * dz) / (speed * head) < (asked ? ASKED_TURN_COS : GAIT_TURN_COS))
  }

  /** Minimum remaining ticks through the nodes' arrival reaches; running means no hops remain. */
  private togo (b: SimBody, i: number, running: boolean): number {
    let far = 0
    for (let m = i; m <= this.index; m++) {
      const n = this.path[m]
      const d = Math.hypot(n.x - b.x, n.z - b.z) - this.reach[m]
      if (d > 0 && d + this.rest[m] > far) far = d + this.rest[m]
    }
    return far / (running ? Math.max(this.vRun, Math.hypot(b.vx, b.vz) + this.aRun) : this.vMax)
  }

  /** Simulate one ground or falling tick; return the next node index, or -1 for a lost schedule. */
  private runTick (b: SimBody, i: number, air: boolean): number {
    // Match the executor's wall steering, sprint restrictions and falling aim.
    const n = air ? this.path[fallAim(this.path, Math.min(i, this.path.length - 1), b.x, b.z, b.vx, b.vz)] : this.aim(i)
    const c = this.c
    const h = this.h
    const nodeDx = n.x - b.x
    const nodeDz = n.z - b.z
    h.dx = nodeDx
    h.dz = nodeDz
    c.sprint = true
    c.forward = true
    let rounded = false
    if ((this.roundMode || this.roundSame) && !air && this.roundCorner(b, nodeDx, nodeDz, n.y, this.round)) {
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
        if (this.roundMode) {
          h.dx = this.round.dx
          h.dz = this.round.dz
          rounded = true
        } else {
          this.roundSame = false
        }
      }
    }
    // Gather boxes once for all probes; distant boxes cannot affect wall steering.
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
    // Against a wall, walking uses a forward heading rather than the sprinting strafe.
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
  /** Nearest obstructing box, expanded by body width, and its distance along the step. */
  private readonly inWay = new Float64Array(5)
  /** Hazard cell coordinates, gathered once per search. */
  private hazards: number[] | null = null

  /** Gather hazards across the search window; they lack collision boxes but touching them stops simulation. */
  private hazardCells (b: SimBody): number[] {
    if (this.hazards !== null) return this.hazards
    let x0 = b.x
    let x1 = b.x
    let y0 = b.y
    let y1 = b.y
    let z0 = b.z
    let z1 = b.z
    for (let j = 0; j <= this.index && j < this.path.length; j++) {
      const n = this.path[j]
      x0 = Math.min(x0, n.x); x1 = Math.max(x1, n.x)
      y0 = Math.min(y0, n.y); y1 = Math.max(y1, n.y)
      z0 = Math.min(z0, n.z); z1 = Math.max(z1, n.z)
    }
    // Shortcut nodes can lie far away; scan only as far as a schedule can reach.
    const r = HAZARD_SCAN
    x0 = Math.max(x0, b.x - r); x1 = Math.min(x1, b.x + r)
    z0 = Math.max(z0, b.z - r); z1 = Math.min(z1, b.z + r)
    y0 = Math.max(y0, b.y - r); y1 = Math.min(y1, b.y + r)
    const m = ROUND_REACH + 1
    const out: number[] = []
    for (let y = Math.floor(y0) - 1; y <= Math.floor(y1 + this.k.height); y++) {
      for (let z = Math.floor(z0 - m); z <= Math.floor(z1 + m); z++) {
        for (let x = Math.floor(x0 - m); x <= Math.floor(x1 + m); x++) {
          if ((this.k.kindAt(x + 0.5, y + 0.5, z + 0.5) & SIM_HAZARD) !== 0) out.push(x, y, z)
        }
      }
    }
    this.hazards = out
    return out
  }

  /** Find a nearer obstruction along the step; existing wall contact is handled by walking rules. */
  private takeInWay (b: SimBody, ux: number, uz: number, reach: number, x0: number, z0: number, x1: number, z1: number): void {
    if (b.x > x0 + 1e-6 && b.x < x1 - 1e-6 && b.z > z0 + 1e-6 && b.z < z1 - 1e-6) return
    let t0 = 0
    let t1 = reach
    if (Math.abs(ux) < 1e-9) {
      if (b.x <= x0 || b.x >= x1) return
    } else {
      let a = (x0 - b.x) / ux
      let c = (x1 - b.x) / ux
      if (a > c) { const q = a; a = c; c = q }
      t0 = Math.max(t0, a)
      t1 = Math.min(t1, c)
    }
    if (Math.abs(uz) < 1e-9) {
      if (b.z <= z0 || b.z >= z1) return
    } else {
      let a = (z0 - b.z) / uz
      let c = (z1 - b.z) / uz
      if (a > c) { const q = a; a = c; c = q }
      t0 = Math.max(t0, a)
      t1 = Math.min(t1, c)
    }
    const w = this.inWay
    if (t0 < t1 - 1e-6 && t0 < w[4]) { w[0] = x0; w[1] = z0; w[2] = x1; w[3] = z1; w[4] = t0 }
  }

  /**
   * Aim around the nearest obstruction expanded by body width, excluding risers the path climbs.
   * Hazards obstruct even at foot height: they stop simulation without a collision box.
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
    const w = this.inWay
    w[4] = Infinity
    for (let j = 0; j < n; j++) {
      const o = j * 6
      if (!(buf[o + 4] > lo - 0.01 && buf[o + 1] < b.y + k.height)) continue
      this.takeInWay(b, ux, uz, reach, buf[o] - hw, buf[o + 2] - hw, buf[o + 3] + hw, buf[o + 5] + hw)
    }
    const hz = this.hazardCells(b)
    for (let j = 0; j < hz.length; j += 3) {
      if (!(hz[j + 1] + 1 > b.y + 1e-3 && hz[j + 1] < b.y + k.height)) continue
      this.takeInWay(b, ux, uz, reach, hz[j] - hw, hz[j + 2] - hw, hz[j] + 1 + hw, hz[j + 2] + 1 + hw)
    }
    if (w[4] === Infinity) return false
    const cx0 = w[0]
    const cx1 = w[2]
    const cz0 = w[1]
    const cz1 = w[3]
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

  /** Search all schedules from `b` without consuming it; false means none remains viable. */
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
      const wait = Math.min(d, GAIT_DELAY + 1)
      const gait = wait <= GAIT_DELAY && hops < GAIT_HOPS
      if (this.done || t + this.togo(w, i, !gait && !this.lipAhead[i]) >= this.best) { alive = true; break }
      if (this.work > this.limit) { this.spent = true; alive = true; break }
      const dom = this.dominated(w, i, t, hops, wait)
      if (dom !== OPEN) { alive = dom === ALIVE; break }
      opened[n] = this.label(w, t, hops, wait)
      hopped[n] = 0
      n++
      // Path-required jumps are allowed once per node; optional hops consume the gait budget.
      const asked = i > used && this.needsJump(w, i)
      if ((asked || gait) && depth < GAIT_DEPTH && t + 1 < this.best && this.mayHop(w, i, asked)) {
        if (this.jump(w, i, t, asked ? hops : hops + 1, depth, asked ? i : used, asked)) hopped[n - 1] = 1
        if (this.done) { alive = true; break }
      }
      i = this.stride(w, i, t, depth)
      if (i < 0) { alive = this.live; break }
      t = this.at
    }
    for (let j = n - 1; j >= 0; j--) {
      alive = alive || hopped[j] === 1
      if (opened[j] >= 0) labelState[opened[j]] = alive ? ALIVE : DEAD
    }
    return alive
  }

  /** Whether node `i` requires a parkour jump or a rise above step height. */
  private needsJump (b: SimBody, i: number): boolean {
    if (!this.lip[i]) return false
    const n = this.path[i]
    return (n.parkour === true && n.walked !== true) || n.y > b.y + SIM.stepHeight
  }

  /**
   * Try the full jump, then ease required jumps or final arrivals that overshoot.
   * Bisection finds the latest release that lands short; optional hops can start later instead.
   */
  private jump (w: SimBody, i: number, t: number, hops: number, depth: number, used: number, asked: boolean): boolean {
    let before = this.best
    this.scraped[depth] = false
    let alive = this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, EASE_NEVER, false)
    if (depth === 0 && this.best < before) this.keep(EASE_NEVER)
    const scraped = this.scraped[depth]
    const flew = this.flew[depth]
    if (this.over[depth] && !this.done && (asked || (this.last && this.overAt[depth] === this.index))) {
      let lo = 1
      let hi = this.flew[depth] - 1
      let ease = 0
      while (lo <= hi) {
        const mid = (lo + hi) >> 1
        this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, mid, true)
        if (this.over[depth]) hi = mid - 1
        else { ease = mid; lo = mid + 1 }
      }
      if (ease > 0) {
        before = this.best
        if (this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, ease, false)) alive = true
        if (depth === 0 && this.best < before) this.keep(ease)
      }
    }
    // Easing the preceding flight leaves room to clear the riser on the next jump.
    if (scraped && RISER_EASE && depth <= RISER_DEPTH) {
      // Limit optional easing work so the main search retains its budget.
      const was = this.riserEasing
      const limit0 = this.limit
      const spent0 = this.spent
      const labels0 = this.labels
      const alive0 = alive
      const best0 = this.best
      this.riserEasing = true
      this.limit = Math.min(limit0, this.work + Math.max(0, limit0 - this.work) * RISER_SHARE)
      try {
        for (let ease = flew - 1; ease >= Math.max(1, flew - RISER_EASES) && !this.done && this.work <= this.limit; ease--) {
          before = this.best
          if (this.hop(copyBody(this.scratch(this.fly, depth), w), w, i, t, hops, depth, used, asked, ease, false)) alive = true
          if (depth === 0 && this.best < before) this.keep(ease)
        }
      } finally {
        // Incomplete exploration cannot justify dominance; retain only recorded successes.
        if (this.spent && !spent0) {
          for (let a = labels0; a < this.labels; a++) labelState[a] = VOID
          alive = alive0 || this.best < best0
        }
        this.riserEasing = was
        this.limit = limit0
        this.spent = spent0
      }
    }
    return alive
  }

  /** Store the best root jump's headings and forward-release tick. */
  private keep (ease: number): void {
    const looks = this.looks[0]
    this.flight.length = 0
    for (let f = 0; f < looks.length; f++) this.flight.push(looks[f])
    this.ease = ease
  }

  /**
   * Consume `h` to simulate a jump with forward held until `ease`, then search its landings.
   * `bare` simulates only the flight, recording its duration and whether it overshoots.
   */
  private hop (h: SimBody, from: SimBody, i: number, t: number, hops: number, depth: number, used: number, asked: boolean, ease: number, bare: boolean): boolean {
    const c = this.c
    const root = depth === 0
    while (this.looks.length <= depth) this.looks.push([])
    const looks = this.looks[depth]
    this.over[depth] = false
    this.flew[depth] = 0
    // Undershooting a gap is a fall; undershooting a rise can leave a viable take-off.
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
      // Only rising contact triggers riser easing; a fall against a gap's far wall does not.
      if (RISER_EASE && asked && !bare && depth > 0 && depth - 1 <= RISER_DEPTH && f < RISER_TICKS && h.collidedH && !h.onGround && h.vy > 0) this.scraped[depth - 1] = true
      t++
      const drop = Math.max(0, y0 - h.y)
      fall += drop
      if (h.inLiquid || t > GAIT_BUDGET) return false
      if (passed < 0 && t >= this.best) return true
      if (!h.onGround) left = true
      // The airborne arrival box spans the jump's height.
      const low = this.low(i)
      const i1 = i
      i = this.pass(h, i, 1.45, left && h.onGround)
      for (let m = i1; m < i && m <= this.index; m++) if (this.path[m].parkour === true) lastP = m
      if (left && h.onGround && !this.over[depth] && lastP >= 0) {
        const o = this.path[lastP]
        if (Math.abs(o.x - h.x) > EASE_PAST || Math.abs(o.z - h.z) > EASE_PAST) { this.over[depth] = true; this.overAt[depth] = lastP }
      }
      // A low roof can prevent take-off entirely.
      if (!left) { if (f > 0) return false; continue }
      if (i <= this.index && !this.over[depth]) {
        const o = this.path[i]
        const ox = o.x - h.x
        const oz = o.z - h.z
        if (ox * h.vx + oz * h.vz < 0 && (Math.abs(ox) > 0.35 || Math.abs(oz) > 0.35)) { this.over[depth] = true; this.overAt[depth] = i }
      }
      if (i > this.index && passed < 0) passed = this.passedAt(h, t)
      // A climbable catch counts as a landing followed by a climb to the goal.
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
        // Slime bounces require the dedicated landing simulation, including sneak control.
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
      // The climbable's foot remains a valid catch height.
      if (h.y < (catching ? Math.min(low, this.catchLow) : low) - 1) return false
    }
  }
}

/**
 * Falling aim index: skip nodes behind the body's motion when the next is on
 * the same floor. Steering back at a node below a ledge destroys forward speed.
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
 * Whether (x, z) is past `n` along the leg to `m`, within the node corridor.
 * This also permits take-off past a jump's origin without walking back to it.
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
 * Whether sprinting on foot reaches `n` on the ground at its level. The planner
 * can label flat ground as parkour because sprint jumps cost less than walking steps.
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

/** Compare jump-first and ground-first schedules; a finite `run` supplies the next ground control. */
function policySearch (k: PlayerSim, b0: SimBody, path: ReadonlyArray<GaitNode>, goal: GaitGoal, pick?: GaitPick): GaitResult {
  const s1 = new Search(k, path, goal)
  // Identical steering decisions let the rounding policy reuse the plain search.
  s1.roundSame = GAIT_REUSE && GAIT_ROUND && pick !== undefined && pick.reuse !== false
  const a = s1.run(b0)
  if (pick === undefined || !GAIT_ROUND) return a
  if (s1.roundSame) {
    pick.round = pick.round && Math.min(a.hop, a.run) !== Infinity
    return a
  }
  // Keep the selected policy until the alternative wins by ROUND_MARGIN to avoid oscillation.
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
 * Compare jump-first and ground-first schedules with optional steering-policy state.
 * If strafe finds only a hop, prefer a plain schedule when it wins by STRAFE_MARGIN ticks.
 */
export function gaitSearch (k: PlayerSim, b0: SimBody, path: ReadonlyArray<GaitNode>, goal: GaitGoal, pick?: GaitPick): GaitResult {
  const r = policySearch(k, b0, path, goal, pick)
  // Compare viable kernel schedules only; without one, leave control to the executor's gates.
  if (!STRAFE_FALLBACK || goal.strafe !== true || r.run !== Infinity || r.hop === Infinity) return r
  const u = new Search(k, path, { ...goal, strafe: false }).run(b0)
  if (Math.min(u.hop, u.run) + STRAFE_MARGIN + 1e-6 < Math.min(r.hop, r.run)) return { ...u, flight: u.flight.slice(), work: r.work + u.work }
  return { ...r, work: r.work + u.work }
}

