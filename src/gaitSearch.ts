// Gait schedules in the exact kernel (improvement, allowHopIntoJump).
//
// On the ground the executor decides one thing a tick: hop now, or keep its
// feet. A hop is the faster gait — the jump keeps the sprint boost ground
// friction eats — but it is ballistic: for the length of its arc the body
// goes where it was thrown. Whether to throw it NOW depends on what the arc
// comes down in: the lip a jump is taken from, the inside of a corner, the
// last node of the path. The old answers compared two fixed policies (always
// hop, never hop) a fixed horizon out, or one hop against none. They took a
// hop into a one-wide corner because hopping is the faster way down the
// straight before it (arena tunnel1: into the wall past the corner, ten
// ticks walking back out of the pocket), and refused two hops in a row under
// a low roof because the first alone gained a single tick.
//
// Here the question is asked exactly. Of the SCHEDULES of hops the body could
// fly from here — up to GAIT_DELAY ticks on the ground before each hop, up to
// GAIT_HOPS hops, then its feet — which reaches the goal soonest, and does it
// begin with a hop? Every schedule is flown in the kernel (playerSim.ts) the
// way the executor would fly it: steering at the next node not yet passed, no
// sprint against a wall, and the jump at a take-off taken the tick the
// executor's own gate would take it. Depth first, hops first, bounded by the
// best so far; a run-first schedule that does as well ends the search.

import { SIM, copyBody, newBody } from './playerSim.js'
import type { PlayerSim, SimBody, SimControl } from './playerSim.js'
import { sideStandoff, wallSlide, againstWall, SPRINT_WALL_MARGIN, STEP_UP_MIN, WALK_STEP_REACH, PROBE_INSET } from './wallSteer.js'
import type { Heading } from './wallSteer.js'
import { landingHurts, SAFE_FALL } from './fallDamage.js'

export interface GaitNode { x: number, y: number, z: number, parkour?: boolean }

export interface GaitGoal {
  /** Index of the goal node in the path. */
  index: number
  /** The goal is a parkour jump: reached when the executor's gate takes it and lands (else: the node is passed). */
  jump: boolean
  /** The goal node ends the path: it is arrived at on the ground, never flown over. */
  last: boolean
  /** The node retired before the path's first: where the flight to a jump that IS the first began. */
  from?: { x: number, z: number } | null
  /** The server counts the whole fall (fallDamage.ts landingHurts); true when absent. */
  wholeFall?: boolean
  /** Fly and run with the 45° strafe (kStrafe). */
  strafe?: boolean
}

export interface GaitResult {
  /** Ticks to the goal of the best schedule that hops now; Infinity when none gets there. */
  hop: number
  /**
   * ...and of a schedule that keeps its feet this tick and does as well
   * (toward a jump) or better (over open ground); Infinity when there is none.
   */
  run: number
  /**
   * The search ran out of work before the feet had their say: no verdict.
   * (A hop is thrown for its whole arc, a tick on the ground is asked again
   * the next: undecided, the feet stay down.)
   */
  spent: boolean
  /** Kernel ticks flown. */
  work: number
  /**
   * The hop now, as flown: the yaw of every tick from the take-off to the
   * touchdown. The executor replays it — the schedule was judged on this
   * flight, and a take-off on any other heading is another flight.
   */
  flight: number[]
  /** ...its ticks after the take-off's with the strafe key held (GaitGoal.strafe). */
  strafe: boolean
}

/** A flight that touches a wall must land with the heading this far either side too (the gates' graze rule). */
export const GRAZE_YAW = 0.02
/**
 * Cosine of the largest angle between the body's velocity and the commanded
 * heading at which a hop may still take off (20°): into a sharper turn it
 * flies the OLD heading and lands off the line. PF_HOP_TURN_DEG for A/B runs.
 */
export const HOP_TURN_COS = Math.cos((Number((typeof process !== 'undefined' ? process.env.PF_HOP_TURN_DEG : undefined) ?? 20) || 20) * Math.PI / 180)
/** How far below the path's floor a hop may come down. */
export const HOP_MAX_DIP = 0.5
/**
 * How far ABOVE a parkour node the body may be and still retire it in the
 * air. Roughly one tick of fall: at that height the landing is committed, so
 * holding the node buys nothing and costs the momentum the next move wants.
 */
export const PARKOUR_LAND_DY = 0.5

/** Ticks on the ground a schedule may wait before each hop, and the hops it may hold. */
const GAIT_DELAY = 3
const GAIT_HOPS = 4
/** Ticks a schedule may take, and grounded ticks a take-off may be refused before its schedule is dropped. */
const GAIT_BUDGET = 80
const GAIT_TAKEOFF = 10
/** Kernel ticks a search may fly: on the schedules that hop now, and then on the ones that do not. */
const GAIT_WORK_HOP = 3000
const GAIT_WORK_RUN = 2000
/**
 * Ticks a landing that hurts stands for the server's packet (plugin.ts
 * HURT_WAIT_TICKS: one, where it comes at once), and the ticks of ground a
 * body that lost its speed there has lost on one that kept it.
 */
const HURT_STAND = 1
const HURT_REGAIN = 2
/** As far as any wall probe reaches from the body (SLIDE_PROBE), and a hair. */
const WALL_REACH = 0.36
/** No gait covers ground faster than this (blocks a tick): the bound's optimism. */
const GAIT_VMAX = 0.5
/** Ticks sooner a schedule that keeps its feet must pass a node over open ground to be preferred to the hop (Search.run). */
const GAIT_RUN_GAIN = 1

const flight = newBody(0, 0, 0)
const gateCtl: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: -1 }

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

/** Aim control `c` from `b` at (px, pz), turned `offset` radians the way a yaw offset turns it. */
export function kAim (c: SimControl, b: SimBody, px: number, pz: number, offset = 0): void {
  const dx = px - b.x
  const dz = pz - b.z
  const d = Math.sqrt(dx * dx + dz * dz)
  if (d <= 1e-9) return
  const hx = dx / d
  const hz = dz / d
  if (offset === 0) { c.hx = hx; c.hz = hz; return }
  const co = Math.cos(offset)
  const si = Math.sin(offset)
  c.hx = hx * co + hz * si
  c.hz = hz * co - hx * si
}

/** Landed, by any of the means the planner counts: ground that carries it, liquid, a climbable at the feet (PhysicsSim.caught). */
export function kCaught (k: PlayerSim, b: SimBody): boolean {
  if (b.inLiquid) return true
  if (b.onGround) return k.carried(b)
  const s = k.stateAt(Math.floor(b.x), Math.floor(b.y), Math.floor(b.z))
  return s >= 0 && (k.world.kind[s] & 1) !== 0
}

/** The executor's againstWall toward `n` (wallSteer.againstWall, on the kernel's probes). */
export function kAgainstWall (k: PlayerSim, b: SimBody, n: { x: number, z: number }): boolean {
  return againstWall(
    (x, y, z) => k.boxHits(x, y, z, -PROBE_INSET, PROBE_INSET),
    k.boxHits(b.x, b.y, b.z, SPRINT_WALL_MARGIN, PROBE_INSET), b.x, b.y, b.z, n.x - b.x, n.z - b.z)
}

let grazed = false

/** Ticks a jump that lands is given to come to stand in its node, where the node ends the path (jumpLandsAt). */
const ARRIVE_TICKS = 40

/**
 * One jump rollout of the gate (PhysicsSim.jumpLands): ticks to the landing,
 * or -1; sets `grazed`. With `arrive` (the node ends the path) the ticks
 * are to STANDING in the node's box: the gate asks only that the jump lands
 * within a block, the path ends in the box — and a landing a block past it
 * walks back (arena mcc-6-2a: hopped into its last jump, 0.8 over, a jump
 * back, and the path ended on the box's far corner, out of tolerance).
 */
function jumpLandsAt (k: PlayerSim, b0: SimBody, n: GaitNode, offset: number, arrive = false): number {
  const b = copyBody(flight, b0)
  const c = gateCtl
  c.sprint = true
  grazed = false
  let fired = false
  let landed = false
  let pressed = false
  let t = 0
  let reached = false
  for (; t < 45; t++) {
    // (getController's latch: rising, or the jump fired and bonked)
    if (!fired) { if (b.vy > 0 || (pressed && b.jumpTicks === SIM.autojumpCooldown)) fired = true } else if (b.onGround) landed = true
    kAim(c, b, n.x, n.z, offset)
    c.jump = !landed
    pressed = c.jump
    k.step(b, c)
    if (b.inLiquid) return -1
    if (b.collidedH && !b.onGround) grazed = true
    if (Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < 1) { reached = true; t++; break }
  }
  if (!reached) return -1
  if (!kCaught(k, b)) {
    const s = settle(k, b, n)
    if (s < 0) return -1
    t += s
  }
  if (!arrive) return t
  c.jump = false
  for (let a = 0; a < ARRIVE_TICKS; a++) {
    if (b.onGround && Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < 1) return t + a
    kAim(c, b, n.x, n.z)
    k.step(b, c)
    if (b.inLiquid) break
  }
  // (the gate takes the jump all the same: a schedule that ends so is the slowest there is, not none)
  return t + ARRIVE_TICKS
}

/** landsThere's settle: ticks until the body is caught within a block of `n`, or -1. Consumes `b`. */
function settle (k: PlayerSim, b: SimBody, n: GaitNode): number {
  const c = gateCtl
  c.jump = false
  c.sprint = true
  for (let s = 0; s < 12; s++) {
    kAim(c, b, n.x, n.z)
    k.step(b, c)
    if (kCaught(k, b)) return Math.hypot(n.x - b.x, n.z - b.z) <= 1 && Math.abs(n.y - b.y) < 1 ? s + 1 : -1
  }
  return -1
}

/**
 * The executor's sprint-jump gate at node `n` from body `b0`, in the kernel:
 * ticks from now to the landing, or -1. The gate as the rollouts fly it —
 * maySprint first (against a wall the executor does not sprint, and its
 * sprint-jump gate is never asked), one jump and the run-in, the settle, and
 * the graze rule: a flight to a parkour node that touches a wall must land
 * with the heading GRAZE_YAW either side too.
 */
export function kJumpLands (k: PlayerSim, b0: SimBody, n: GaitNode, arrive = false): number {
  if (kAgainstWall(k, b0, n)) return -1
  const r = jumpLandsAt(k, b0, n, 0, arrive)
  if (r < 0 || !grazed || n.parkour !== true) return r
  return jumpLandsAt(k, b0, n, GRAZE_YAW) >= 0 && jumpLandsAt(k, b0, n, -GRAZE_YAW) >= 0 ? r : -1
}

class Search {
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
  /** The yaws of the hop from the root (GaitResult.flight). */
  private readonly flight: number[] = []
  /** Where the body stood before the tick just flown (passedAt). */
  private px = 0
  private pz = 0
  /** The executor's collision probe, on the kernel; and the heading being steered. */
  private readonly hit = (x: number, y: number, z: number): boolean => this.k.probeHit(x, y, z, -PROBE_INSET, PROBE_INSET)
  private readonly h: Heading = { dx: 0, dz: 0 }

  constructor (
    private readonly k: PlayerSim,
    private readonly path: ReadonlyArray<GaitNode>,
    private readonly goal: GaitGoal,
    private readonly floor: number
  ) {
    this.strafe = goal.strafe === true
  }

  private readonly strafe: boolean

  run (b0: SimBody): GaitResult {
    const root = copyBody(newBody(0, 0, 0), b0)
    const i0 = this.pass(root, 0, 1)
    // the goal from here, this tick: nothing to schedule
    const now = this.terminal(root, i0, 0)
    if (now >= 0) return { hop: Infinity, run: now, spent: false, work: this.work, flight: this.flight, strafe: this.strafe }
    // (the hop now is the caller's to allow: the executor has its own guard on a take-off into a turn)
    this.best = GAIT_BUDGET + 1
    this.hop(copyBody(this.scratch(this.fly, 0), root), i0, 0, 0, 0)
    const hop = this.best <= GAIT_BUDGET ? this.best : Infinity
    if (hop === Infinity) return { hop, run: Infinity, spent: this.spent, work: this.work, flight: this.flight, strafe: this.strafe }
    // Keeping its feet this tick: does any schedule do as well? A tie keeps
    // them: the goal is reached the same either way, and the ground has the
    // options. (A node passed is timed to the fraction of a tick, passedAt:
    // in whole ticks a hop now ties with the same hop a tick later, the tick
    // after that ties again, and the body runs where it should be flying —
    // arena simple1, four ticks on the spot the hop was due.)
    //
    // Over open ground the feet have to EARN the tick (GAIT_RUN_GAIN). The
    // goal there is a horizon, not a place: hopping at every touchdown is
    // the fastest gait there is, and which of two schedules is first past a
    // node eight blocks on turns on where its hops happen to fall about that
    // node — half a tick either way, for nothing (arena simple3: two ticks
    // run after every landing, a tick lost on each). A corner, a lip or the
    // path's end costs whole ticks, and shows.
    const bound = this.goal.jump ? hop + 1e-6 : hop - GAIT_RUN_GAIN + 1e-6
    this.best = bound
    this.firstWins = true
    this.limit = this.work + GAIT_WORK_RUN
    this.spent = false
    const i = this.stride(root, i0, 0)
    if (i >= 0) this.dfs(root, i, this.at, 0, 1)
    const run = this.best < bound ? this.best : Infinity
    return { hop, run, spent: run === Infinity && this.spent, work: this.work, flight: this.flight, strafe: this.strafe }
  }

  /** When the last stride ended. */
  private at = 0

  /**
   * One tick with the feet down from time `t`, and the fall off what it
   * walked off, to the ground: the node index there (the time in `at`), -1
   * when the schedule ended on the way — lost, or past the goal and recorded.
   */
  private stride (b: SimBody, i: number, t: number): number {
    let fall = 0
    let drop = 0
    do {
      const y0 = b.y
      i = this.runTick(b, i)
      t++
      if (i < 0 || t > GAIT_BUDGET) return -1
      if (!this.goal.jump && i > this.goal.index) { this.record(this.passedAt(b, t)); return -1 }
      drop = Math.max(0, y0 - b.y)
      fall += drop
    } while (!b.onGround)
    this.at = t + this.landed(b, fall, drop)
    return i
  }

  /**
   * A landing after `fall` (`drop` of it this tick): the ticks it stands
   * when it hurt. The server takes the speed of a body it hurts, and the
   * executor decides nothing until it has (plugin.ts HURT_WAIT_TICKS).
   */
  private landed (b: SimBody, fall: number, drop: number): number {
    if (fall <= SAFE_FALL) return 0
    const kind = this.k.kindAt(b.x, b.y - 0.2, b.z)
    if (!landingHurts(fall, drop, kind, false, this.goal.wholeFall !== false)) return 0
    b.vx = 0
    b.vz = 0
    return HURT_STAND
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

  /** Nodes passed by body `b` from index `i` on (the arrival box; `dy` its height). */
  private pass (b: SimBody, i: number, dy: number): number {
    const g = this.goal
    const lim = g.jump ? g.index : g.index + 1
    while (i < lim) {
      const n = this.path[i]
      if (Math.abs(n.x - b.x) > 0.35 || Math.abs(n.z - b.z) > 0.35 || Math.abs(n.y - b.y) >= dy) break
      if (g.last && i === g.index && !b.onGround) break
      i++
    }
    return i
  }

  /**
   * When the goal node was passed, to the fraction of a tick: the tick
   * before `t`, and how far into the move from there the body crossed into
   * the node's box. Whole ticks tie too often to choose by — a hop now and
   * the same hop a tick later pass a node eight blocks on in the same tick.
   */
  private passedAt (b: SimBody, t: number): number {
    const n = this.path[this.goal.index]
    const ax = Math.abs(n.x - this.px)
    const az = Math.abs(n.z - this.pz)
    const dx = Math.abs(b.x - this.px)
    const dz = Math.abs(b.z - this.pz)
    let f = 0
    if (ax > 0.35 && dx > 1e-9) f = Math.max(f, (ax - 0.35) / dx)
    if (az > 0.35 && dz > 1e-9) f = Math.max(f, (az - 0.35) / dz)
    return t - 1 + Math.min(1, f)
  }

  /** At a grounded state: ticks at which the goal is reached from here with no further choice, or -1. */
  private terminal (b: SimBody, i: number, t: number): number {
    const g = this.goal
    if (!g.jump) return i > g.index ? t : -1
    if (i < g.index) return -1
    const r = kJumpLands(this.k, b, this.path[g.index], g.index === this.path.length - 1)
    this.work += 45
    return r < 0 ? -1 : t + r
  }

  /**
   * The executor's landing retirement (plugin.ts allowLandingRetire): has
   * body `h`, down this tick, come down within a block of parkour node
   * `node` and past it along the flight from the node before?
   */
  private cameDownPast (h: SimBody, node: GaitNode): boolean {
    const g = this.goal
    const from = g.index > 0 ? this.path[g.index - 1] : g.from
    if (from === undefined || from === null || g.index >= this.path.length - 1) return false
    if (Math.hypot(node.x - h.x, node.z - h.z) > 1 || Math.abs(node.y - h.y) >= 1) return false
    const fx = node.x - from.x
    const fz = node.z - from.z
    return fx * fx + fz * fz > 1 && (h.x - node.x) * fx + (h.z - node.z) * fz > 0
  }

  /** May a hop be taken off here (never into a turn: the arc flies the velocity)? */
  private mayHop (b: SimBody, i: number): boolean {
    const n = this.aim(i)
    const dx = n.x - b.x
    const dz = n.z - b.z
    const head = Math.hypot(dx, dz)
    const speed = Math.hypot(b.vx, b.vz)
    return !(speed > 0.1 && head > 1e-6 && (b.vx * dx + b.vz * dz) / (speed * head) < HOP_TURN_COS)
  }

  /** Ticks the rest of the way takes at the least. */
  private togo (b: SimBody): number {
    const n = this.path[this.goal.index]
    return Math.max(0, Math.hypot(n.x - b.x, n.z - b.z) - 0.5) / GAIT_VMAX
  }

  /** One tick with the feet down (or falling off what they walked off): the node index after it, -1 when the schedule is lost. */
  private runTick (b: SimBody, i: number): number {
    // steered as the executor walks a step (plugin.ts, wallSteer.ts): off a
    // wall it runs along, no sprint against one, along one in its way
    const n = this.aim(i)
    const c = this.c
    const h = this.h
    const nodeDx = n.x - b.x
    const nodeDz = n.z - b.z
    h.dx = nodeDx
    h.dz = nodeDz
    c.sprint = true
    // (one gathering of the boxes round the body for all the probes; none
    // within a probe's reach at body level, and there is no wall to steer by)
    this.k.probeBegin(b.x, b.y, b.z, WALL_REACH, 0.6)
    if (this.k.probeHit(b.x, b.y, b.z, WALL_REACH, PROBE_INSET)) {
      const walk = n.parkour !== true && n.y - b.y <= STEP_UP_MIN
      if (walk && Math.hypot(nodeDx, nodeDz) > 0.15) sideStandoff(this.hit, b.x, b.y, b.z, h)
      c.sprint = !againstWall(this.hit, this.k.probeHit(b.x, b.y, b.z, SPRINT_WALL_MARGIN, PROBE_INSET), b.x, b.y, b.z, h.dx, h.dz)
      if (walk && Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH) wallSlide(this.hit, b.x, b.y, b.z, nodeDx, nodeDz, nodeDx, nodeDz, h)
    }
    const len = Math.hypot(h.dx, h.dz)
    if (len > 1e-9) { c.hx = h.dx / len; c.hz = h.dz / len }
    c.left = false
    // (the executor strafes its sprinting ticks: against a wall it walks, and looks where it goes)
    if (this.strafe && c.sprint) kStrafe(c)
    c.jump = false
    this.px = b.x
    this.pz = b.z
    this.k.step(b, c)
    this.work++
    if (b.inLiquid || b.y < this.floor - 1) return -1
    return this.pass(b, i, 1)
  }

  /** From a grounded state: every schedule on from here. Consumes nothing of `b`. */
  private dfs (b: SimBody, i: number, t: number, hops: number, depth: number): void {
    const w = copyBody(this.scratch(this.win, depth), b)
    let waited = 0
    for (let d = 0; ; d++) {
      if (this.done || t + this.togo(w) >= this.best) return
      if (this.work > this.limit) { this.spent = true; return }
      const at = this.terminal(w, i, t)
      if (at >= 0) { this.record(at); return }
      if (this.goal.jump && i >= this.goal.index && ++waited > GAIT_TAKEOFF) return
      if (d <= GAIT_DELAY && hops < GAIT_HOPS && this.mayHop(w, i)) {
        this.hop(copyBody(this.scratch(this.fly, depth), w), i, t, hops, depth)
        if (this.done) return
      }
      i = this.stride(w, i, t)
      if (i < 0) return
      t = this.at
    }
  }

  /** A hop from grounded body `h` (consumed), then every schedule on from its landing. */
  private hop (h: SimBody, i: number, t: number, hops: number, depth: number): void {
    const g = this.goal
    const c = this.c
    let left = false
    let passed = -1
    let fall = 0
    for (let f = 0; ; f++) {
      const n = this.aim(i)
      kAim(c, h, n.x, n.z)
      c.jump = f === 0
      c.sprint = true
      c.left = false
      if (this.strafe && f > 0) kStrafe(c)
      if (depth === 0 && hops === 0) this.flight.push(Math.atan2(-c.hx, -c.hz))
      this.px = h.x
      this.pz = h.z
      const y0 = h.y
      this.k.step(h, c)
      this.work++
      t++
      const drop = Math.max(0, y0 - h.y)
      fall += drop
      if (h.inLiquid || t > GAIT_BUDGET || (passed < 0 && t >= this.best)) return
      if (!h.onGround) left = true
      // (the hop's own box is as tall as the hop: plugin.ts HOP_ARRIVE_DY)
      i = this.pass(h, i, 1.45)
      if (!left) { if (f > 0) return; continue }
      if (g.jump && passed < 0 && i >= g.index) {
        // The hop flies the jump itself, where the executor's arrival would
        // retire its node: in the air low over it (higher than
        // PARKOUR_LAND_DY a node with a hole beyond is held to the landing,
        // and a body that came down past it is turned round for it — arena
        // tenways-headhitters: onto the step after, and 22 ticks back), or
        // down within a block of it and BEYOND it (a landing short of the
        // node is a take-off for it still: the same course, a hop that came
        // down on the lip before each node was counted as the jump landed,
        // and the jump the gate then took from there spent its arc against
        // the next head-hitter, 12 ticks a platform).
        const node = this.path[g.index]
        if (Math.abs(node.x - h.x) <= 0.35 && Math.abs(node.z - h.z) <= 0.35 && Math.abs(node.y - h.y) < 1.45 &&
            h.y - node.y <= PARKOUR_LAND_DY) {
          i = g.index + 1
        } else if (h.onGround && this.cameDownPast(h, node)) {
          i = g.index + 1
          passed = t
        }
      }
      if (i > g.index && passed < 0) passed = this.passedAt(h, t)
      if (h.onGround) {
        const low = g.jump ? Math.min(this.floor, this.path[g.index].y) : this.floor
        if (h.y < low - HOP_MAX_DIP) return
        const stood = this.landed(h, fall, drop)
        // (passed in the air: the goal's tick is the passing, once the hop
        // has come down on the path — and with its speed, or the ground
        // that costs past the goal is the schedule's too)
        if (passed >= 0) this.record(stood > 0 ? Math.max(passed, t) + stood + HURT_REGAIN : passed)
        else this.dfs(h, i, t + stood, hops + 1, depth + 1)
        return
      }
      if (h.y < Math.min(this.floor, this.path[g.index].y) - 1) return
    }
  }
}

const walker = newBody(0, 0, 0)

/**
 * Is node `n` walked to from `from` — sprinting at it, feet down, the
 * arrival box reached on the ground at its level? A parkour node that is, is
 * a jump only in the plan: the executor runs it (its walking gate comes
 * first), and the gait treats it as ground.
 */
export function kWalkable (k: PlayerSim, from: { x: number, y: number, z: number }, n: GaitNode): boolean {
  if (Math.abs(n.y - from.y) > 0.6) return false
  const b = copyBody(walker, flight)
  b.x = from.x; b.y = from.y; b.z = from.z
  b.vx = 0; b.vy = 0; b.vz = 0
  b.onGround = true; b.collidedH = false; b.collidedV = true; b.jumpTicks = 0; b.inLiquid = false
  const c = gateCtl
  c.jump = false
  c.sprint = true
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
 * best schedule that hops now, and of one that keeps its feet this tick and
 * does at least as well. A hop is worth taking when `hop` is finite and
 * `run` is not. `floor`: the lowest the path goes on the way (a schedule
 * that comes down under it is lost).
 */
export function gaitSearch (k: PlayerSim, b0: SimBody, path: ReadonlyArray<GaitNode>, goal: GaitGoal, floor: number): GaitResult {
  return new Search(k, path, goal, floor).run(b0)
}
