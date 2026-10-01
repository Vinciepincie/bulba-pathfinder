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

import { PlayerSim, newBody, copyBody, SIM, SIM_CLIMBABLE, SIM_SLIME } from './playerSim.js'
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
  /** A beam program's controls, tick for tick: [yaw to look at (NaN: none), key flags (F_FWD | F_SPRINT | F_JUMP)] pairs; the phases above are then unused. */
  seq?: number[]
  /** Touch and go: a landing short of the node starts the next leg (a beam program's chain of jumps) instead of ending the flight. */
  legs?: boolean
  /** Heading on the ground, until the jump fires. */
  run: Heading
  /** Tick (from the start) the jump is pressed on; held until the body leaves the ground. */
  jumpAt: number
  /** Heading for the first `turnAir` air ticks (or until a slime rebound). */
  air1: Heading
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

interface Stand { x: number, y: number, z: number, catch: boolean }

/** Start-position nudge every accepted program must survive, per axis (blocks). */
export const ROBUST_NUDGE = 0.03
/**
 * ...and a LIVE program's: its replay starts from the very state it was
 * found from and turns the very yaws (HopOracle.face), so only a model's
 * last hair of mismatch is left to cover.
 */
export const LIVE_NUDGE = 0.01
/** Letting go after touchdown: ticks the body must stay on the landing node. */
const STAY_TICKS = 8
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
export function programHeading (p: HopProgram, fired: boolean, air: number, rebounds: number): Heading {
  if (!fired) return p.run
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
  sprint: boolean
  jumpAts: readonly number[]
  turnAirs: readonly number[]
  air2s: readonly Heading[]
  /** Also branch on the headings aimed at the landing (aimHeadings). */
  aim: boolean
  /** A bounce class whose landing no single rebound reaches: branch on to further slimes at the rebound (chainOn). */
  chain?: boolean
}

type Family = 'straight' | 'run-turn' | 'air-turn' | 'yaw' | 'bounce'

// Straight and run-turn turn onto their second heading at take-off (turnAir
// 0); straight's second heading is its first.
const NO_TURN = [0]
const STRAIGHT_JUMP_AT = [0, 1, 2, 3, 4, 5, 6, 8, 10]
const RUN_TURN_JUMP_AT = [1, 2, 3, 4, 5, 6, 8]
const AXES8: ReadonlyArray<[number, number]> = [[1, 0], [-1, 0], [0, 1], [0, -1], [Math.SQRT1_2, Math.SQRT1_2], [Math.SQRT1_2, -Math.SQRT1_2], [-Math.SQRT1_2, Math.SQRT1_2], [-Math.SQRT1_2, -Math.SQRT1_2]]
const AIR_TURN_JUMP_AT = [0, 2, 4, 6]
const AIR_TURN_TICKS = [2, 4, 6, 8]
// The yaw family's grid (radians off the bearing to the landing), measured on
// a one-block neo: the run along the pillar's face goes out 0.15-0.9, the
// turn comes back 0.3-1.5 within the first few air ticks, and the jump is
// taken after a run of up to 14 ticks. Gentlest first.
const YAW_RUN = [0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6, 0.9, -0.9]
const YAW_TURN = [0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2, 1.5, -1.5]
const YAW_TURN_TICKS = [1, 2, 3, 4, 6, 8]
const YAW_JUMP_AT = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14]
// Slime bounces: aim at a slime top until the rebound, then at the landing.
// Slime tops within BOUNCE_REACH (horizontally) of both the take-off and
// the landing and up to BOUNCE_DROP below the take-off, the nearest
// BOUNCE_SLIMES by the two legs together.
const BOUNCE_REACH = 7
const BOUNCE_DROP = 14
const BOUNCE_SLIMES = 12
const BOUNCE_JUMP_AT = [0, 1, 2, 3, 4, 6, 8, 12]
/** How far short of a slime top's centre (blocks, along the way in) the bounce family's first phase also aims. */
const BOUNCE_SHORT = [0, 0.3]
/**
 * Bounce CHAINS, for a landing no single rebound reaches (HopProgram.bounces):
 * the first slimes tried, the next slimes tried from each rebound, and how
 * many slimes may follow the first.
 */
const BOUNCE_CHAIN_FIRST = 4
const BOUNCE_CHAIN_NEXT = 3
const BOUNCE_CHAIN = 2
/** advance(): fly to the next rebound (a finite mark no air count reaches). */
const TO_NEXT_REBOUND = Number.MAX_SAFE_INTEGER

const FAMILIES: Record<HopFamilies, readonly Family[]> = {
  simple: ['straight', 'run-turn', 'air-turn', 'bounce'],
  yaw: ['yaw'],
  // (the bounce family before the yaw family's grid, as in `simple`: it
  // costs nothing where no slime is in reach, and a live search that runs
  // out of time must not have spent it all on the neos first)
  all: ['straight', 'run-turn', 'air-turn', 'bounce', 'yaw'],
  // (the beam alone: HopOracle.hop)
  beam: []
}

/** A flight in progress: the body and the counters a program steers by (the executor keeps the same). */
interface Flight { b: SimBody, t: number, fired: boolean, air: number, rebounds: number, yaw: number, leg: number }

/** A first state the search flies from: a body at rest on a stand point, or a live one (`live` = its exact state). */
interface Pad { f: Flight, live: SimBody | undefined }

const newFlight = (): Flight => ({ b: newBody(0, 0, 0), t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0 })

function flightInto (dst: Flight, src: Flight): Flight {
  copyBody(dst.b, src.b)
  dst.t = src.t
  dst.fired = src.fired
  dst.air = src.air
  dst.rebounds = src.rebounds
  dst.yaw = src.yaw
  dst.leg = src.leg
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
const BEAM_DEPTH = 80
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
 * A beam state's dedupe key. In the air: position to 1/16, velocity to 1/64
 * (vertical 1/32). Before take-off, position to 1/8 and velocity to 1/20: the
 * ground share is spread over PLACES — the corner a neo leaves from ranks
 * worse on distance than the spot pressed into the pillar, and fine velocity
 * variants of that one spot would otherwise fill the share — but not over
 * speed, which a chain across a stepping stone lives on.
 */
function beamKey (f: Flight): string {
  const b = f.b
  if (!f.fired) return `g${Math.round(b.x * 8)},${Math.round(b.y * 8)},${Math.round(b.z * 8)},${Math.round(b.vx * 20)},${Math.round(b.vz * 20)}`
  return `${Math.round(b.x * 16)},${Math.round(b.y * 16)},${Math.round(b.z * 16)},${Math.round(b.vx * 64)},${Math.round(b.vy * 32)},${Math.round(b.vz * 64)},${b.onGround ? 1 : 0}${f.fired ? 1 : 0}`
}
/** Ticks jump may be held without taking off (a ceiling, a ladder) before the program is given up. */
const JUMP_GRACE = 6
/** Longest run on the ground, and longest flight, a program may have (ticks). */
const MAX_RUN = 40
const MAX_AIR = 60
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
  /** Kernel ticks flown by the last discover() (diagnostics). */
  ticksFlown = 0
  /**
   * Did the last hop() give no verdict: stopped on its deadline, or had no
   * stand to land on or take off from (a landing the model does not know is
   * not a jump the physics refuses)? False: every program of every family
   * flew, and what came back is the answer.
   */
  noVerdict = false

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

  private settle (b: SimBody): boolean {
    const c = this.ctl
    c.forward = false; c.back = false; c.left = false; c.right = false; c.jump = false; c.sprint = false; c.sneak = false
    for (let t = 0; t < 30; t++) {
      this.sim.step(b, c)
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
   * for A/B runs: the programs found are the same.)
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
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0 }
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
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0 }
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

  /** Program `p`'s first state nudged by (nx, nz): its live start, or a body come to rest (or hanging, `climb`) at its start point. */
  private origin (p: HopProgram, nx: number, nz: number): Flight | null {
    if (p.start === undefined) return p.climb === true ? this.hang(p.sx + nx, p.sy, p.sz + nz) : this.pad(p.sx + nx, p.sy, p.sz + nz)
    const b = copyBody(newBody(0, 0, 0), p.start)
    b.x += nx
    b.z += nz
    return { b, t: 0, fired: false, air: 0, rebounds: 0, yaw: p.yaw0 ?? NaN, leg: 0 }
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
    for (;;) {
      if (f.fired) {
        if (f.air >= airUntil || (f.rebounds > rebounds0 && airUntil !== Infinity)) return FLYING
        // the pre-screen, as the program commits to its last heading
        if (f.air === p.turnAir && f.rebounds === 0 && !p.catchTarget && !b.onGround && !this.comesDownNear(b, p)) return -1
      } else if (f.t >= groundUntil) {
        return FLYING
      }
      if (p.seq !== undefined) {
        const k = Math.min(f.t, (p.seq.length >> 1) - 1) << 1
        this.seqControl(f, c, p.seq[k], p.seq[k + 1])
      } else {
        const d = headingDir(programHeading(p, f.fired, f.air, f.rebounds), b.x, b.z)
        if (d !== null) this.face(f, c, d[0], d[1])
        c.forward = true; c.back = false; c.left = false; c.right = false
        c.sprint = p.sprint
        c.jump = !f.fired && f.t >= p.jumpAt
      }
      c.sneak = sneakLanding(p, b.onGround, b.x, b.y, b.z, b.vy, this.sim.halfWidth)
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
    this.sim.step(b, this.ctl)
    this.ticksFlown++
    if (track !== null) track.push(b.x, b.y, b.z)
    const t = f.t++
    if (b.inLiquid) return -1
    if (!f.fired) {
      // (off a climbable, the take-off is the body leaving it: jump held
      // there only climbs, and a push off it never gets near 0.3 either way)
      const onClimb = p.climb === true && !b.onGround && this.sim.onClimbable(b)
      if (b.vy > 0.3 || (!b.onGround && b.vy < -0.3) || (p.climb === true && !b.onGround && !onClimb)) f.fired = true
      else if ((t >= p.jumpAt + JUMP_GRACE && !onClimb) || t - f.leg >= MAX_RUN) return -1
      else if (p.legs === true && f.leg > 0 && b.onGround && Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz &&
               this.nodeY(p.tx, b.y, p.tz) === p.ty && this.stays(copyBody(this.probe, b), p)) {
        // touch and go's last leg can be a slide: down short of the node on
        // ice, and carried into it on the ground
        return t + 1
      }
      return FLYING
    }
    if (++f.air > MAX_AIR * (1 + f.rebounds)) return -1
    const inCol = Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz
    if (p.catchTarget) {
      if (inCol && b.y >= p.ty - 0.001 && b.y < p.ty + 1) return this.stays(b, p) ? t + 1 : -1
    }
    if (b.onGround) {
      if (b.vy > 0.05 && (this.kindAt(Math.floor(b.x), Math.floor(b.y - 0.2), Math.floor(b.z)) & SIM_SLIME) !== 0) {
        f.rebounds++
        // (a nudged twin of a chain only has to get this far: robust())
        if (this.reboundIsLanding && f.rebounds === 1) return t + 1
        return FLYING
      }
      if (!this.overCol(b, p.tx, p.tz) || this.nodeY(p.tx, b.y, p.tz) !== p.ty) {
        if (p.legs !== true) return -1
        // (a nudged twin of a chain only has to get this far — a real leg's
        // end, a block or more from its start, not a hop on the spot: robust())
        if (this.touchIsLanding && Math.hypot(b.x - p.sx, b.z - p.sz) >= LEG_MIN) return t + 1
        // touch and go: down short of the node, and the next leg starts here
        f.fired = false
        f.air = 0
        f.rebounds = 0
        f.leg = f.t
        return FLYING
      }
      return this.stays(b, p) ? t + 1 : -1
    }
    if (b.y < Math.min(p.sy, p.ty) - 14) return -1
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
  private beamSearch (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], deadline: number, corridor: ReadonlyArray<{ x: number, y: number, z: number }> = []): HopProgram | null {
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
      climb: this.climbing(pads[0].f.b), slimeLand: this.slimeLanding(t, catchTarget)
    }
    const nodes: BeamNode[] = []
    let layer: number[] = []
    pads.forEach((pad, i) => {
      // the ensemble, the way robust() will fly the program: its start, nudged
      const start: HopProgram = { ...tpl, sx: pad.f.b.x, sy: pad.f.b.y, sz: pad.f.b.z, start: pad.live, yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw }
      const nudge = LIVE_NUDGE
      const fs: Flight[] = [flightInto(newFlight(), pad.f)]
      for (const [nx, nz] of [[nudge, 0], [-nudge, 0], [0, nudge], [0, -nudge]]) {
        const f = this.origin(start, nx, nz)
        if (f !== null) fs.push(f)
      }
      if (fs.length < 5) return // a nudged start that cannot rest there: no robust program from this pad
      nodes.push({ fs, done: [FLYING, FLYING, FLYING, FLYING, FLYING], parent: -1, want: NaN, flags: 0, pad: i, score: 0, wp: reach(pad.f.b, 0) })
      layer.push(nodes.length - 1)
    })
    const found: HopProgram[] = []
    const seen = new Map<string, number>()
    for (let depth = 0; depth < BEAM_DEPTH && layer.length > 0 && found.length === 0; depth++) {
      if (performance.now() > deadline) { this.noVerdict = true; break }
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
              const f = flightInto(newFlight(), n.fs[k])
              fs.push(f)
              if (done[k] !== FLYING) continue // landed already: the replay has moved on
              // past the end of the lead's first real leg a chain's twins are released (robust())
              if (k > 0 && fs[0].leg > 0 && Math.hypot(fs[0].b.x - n0x, fs[0].b.z - n0z) >= LEG_MIN) { done[k] = RELEASED; continue }
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
                ? this.beamScore(fs[k], t, landY, catchTarget, 0)
                : this.beamScore(fs[k], [way[child.wp][0], way[child.wp][1], way[child.wp][2]], way[child.wp][1], false, tail[child.wp]))
            }
            if (score === Infinity) continue
            child.score = score
            const key = beamKey(fs[0])
            const prev = seen.get(key)
            if (prev !== undefined) {
              if (children[prev].score > score) children[prev] = child
              continue
            }
            seen.set(key, children.length)
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
   * corridor's length beyond it.
   */
  private beamScore (f: Flight, t: [number, number, number], landY: number, catchTarget: boolean, after: number): number {
    const b = f.b
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const toGo = Math.hypot(tcx - b.x, tcz - b.z) + after
    // below the landing, a body climbs back about a block a jump
    const climb = Math.max(0, landY - b.y) * BEAM_CLIMB
    if (catchTarget) return f.t + toGo / BEAM_SPEED + climb
    if (!f.fired) return f.t + toGo / BEAM_SPEED + climb + this.spinUp(b, tcx, tcz)
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

  private stays (b: SimBody, p: HopProgram): boolean {
    const c = this.ctl
    c.forward = false; c.sprint = false; c.jump = false; c.sneak = true
    for (let k = 0; k < STAY_TICKS; k++) this.sim.step(b, c)
    c.sneak = false
    if (p.catchTarget) return Math.floor(b.x) === p.tx && Math.floor(b.z) === p.tz && b.y >= p.ty - 1.001 && b.y < p.ty + 1
    if (!this.overCol(b, p.tx, p.tz)) return false
    return b.onGround && this.nodeY(p.tx, b.y, p.tz) === p.ty && this.sim.supportedIn(b, p.tx, p.tz)
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
    const n = p.start !== undefined || p.seq !== undefined ? LIVE_NUDGE : ROBUST_NUDGE
    // a chain (touch and go) is held to its first touch-down: past it the
    // replay is the kernel's own flight to the bit, and twins nudged a
    // hundredth apart part ways over a few bounces as any two players would
    this.touchIsLanding = p.legs === true
    // a bounce chain likewise to its first rebound, off the slime it aims at
    this.reboundIsLanding = p.bounces !== undefined && p.bounces.length > 0
    try {
      for (const [nx, nz] of [[n, 0], [-n, 0], [0, n], [0, -n]]) {
        if (this.fly(p, nx, nz) < 0) return -1
      }
    } finally {
      this.touchIsLanding = false
      this.reboundIsLanding = false
    }
    return t0
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

  /** One family's program classes from start `st`, simplest (gentlest) first. */
  private * classes (family: Family, st: { x: number, y: number, z: number }, aims: ReadonlyArray<[number, number]>, tcx: number, tcz: number, walk: boolean, grid: boolean): Generator<ProgramClass> {
    if (family === 'straight') {
      for (const a of aims) {
        yield { run: point(a[0], a[1]), sprint: true, jumpAts: STRAIGHT_JUMP_AT, turnAirs: NO_TURN, air2s: [point(a[0], a[1])], aim: false }
      }
      if (walk) for (const a of aims) yield { run: point(a[0], a[1]), sprint: false, jumpAts: STRAIGHT_JUMP_AT, turnAirs: NO_TURN, air2s: [point(a[0], a[1])], aim: false }
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
        .map(([x, , z]) => [x, z, Math.hypot(x + 0.5 - st.x, z + 0.5 - st.z) + Math.hypot(tcx - x - 0.5, tcz - z - 0.5)])
      let tops = all.filter(t => Math.hypot(tcx - t[0] - 0.5, tcz - t[1] - 0.5) <= BOUNCE_REACH)
      // no rebound from here carries to the landing: a chain (chainOn), off
      // the slimes nearest by both legs
      const chain = tops.length === 0
      tops = (chain ? all : tops).sort((p, q) => p[2] - q[2]).slice(0, chain ? BOUNCE_CHAIN_FIRST : BOUNCE_SLIMES)
      const air2s = [point(tcx, tcz)]
      // at the slime's centre, and short of it along the way in: a take-off
      // that carries (a jump, a fall from a ledge's end) lands past a centre
      // it aims at, and one off the centre throws the rebound elsewhere —
      // a bounce is two landings, and the first one is a choice too
      for (const sprint of walk ? [true, false] : [true]) {
        for (const [x, z] of tops) {
          const cx = x + 0.5
          const cz = z + 0.5
          const d = Math.hypot(cx - st.x, cz - st.z) || 1
          for (const short of BOUNCE_SHORT) {
            yield { run: point(cx - ((cx - st.x) / d) * short, cz - ((cz - st.z) / d) * short), sprint, jumpAts: BOUNCE_JUMP_AT, turnAirs: TO_REBOUND, air2s, aim: true, chain }
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
  private search (pads: readonly Pad[], lands: readonly Stand[], t: [number, number, number], family: Family, deadline: number, walk: boolean, grid: boolean): HopProgram | null {
    const tcx = t[0] + 0.5
    const tcz = t[2] + 0.5
    const catchTarget = lands[0].catch
    const landY = lands[0].y
    const aimsRaw: Array<[number, number]> = [[tcx, tcz], ...lands.map(p => [p.x, p.z] as [number, number])]
    const aims = aimsRaw.filter((p, i) => aimsRaw.findIndex(q => Math.abs(q[0] - p[0]) < 0.05 && Math.abs(q[1] - p[1]) < 0.05) === i)
    const [run, launch, air, branch] = this.flights
    const aimed: Heading[] = []
    const found: HopProgram[] = []
    const slimeLand = this.slimeLanding(t, catchTarget)
    let n = 0
    for (const pad of pads) {
      const st = pad.f.b
      const climb = this.climbing(st)
      // The reach test (canReach) holds for a flight that only falls: not
      // where slime may throw it back up, nor off a climbable it may regain.
      const falls = REACH_PRUNE && family !== 'bounce' && !climb && !slimeLand &&
        this.slimeTops(Math.floor(st.x), Math.floor(st.y + 1e-9), Math.floor(st.z)).length === 0
      for (const cls of this.classes(family, st, aims, tcx, tcz, walk, grid)) {
        if ((++n & 3) === 0 && performance.now() > deadline) { this.noVerdict = true; return this.quickestRobust(found, deadline) }
        // the run (no jump) and the first air phase (no turn) of this class
        const base: HopProgram = {
          sx: st.x, sy: st.y, sz: st.z, start: pad.live, yaw0: Number.isNaN(pad.f.yaw) ? undefined : pad.f.yaw, run: cls.run, jumpAt: Infinity, air1: cls.run, turnAir: Infinity, air2: cls.run,
          sprint: cls.sprint, ticks: 0, tx: t[0], ty: t[1], tz: t[2], catchTarget, family, climb, slimeLand
        }
        flightInto(run, pad.f)
        for (const j of cls.jumpAts) {
          if (this.advance(run, base, j, 0, null) !== FLYING) break
          // ran off an edge before tick j: no later jump tick is pressed either
          const walkedOff = run.fired
          const pj: HopProgram = { ...base, jumpAt: j }
          // (a take-off the landing is out of reach of: no turn brings it in)
          if (this.advance(flightInto(launch, run), pj, Infinity, 0, null) === FLYING &&
              (!falls || this.canReach(launch.b, cls.sprint, t[0], t[1], t[2]))) {
            flightInto(air, launch)
            for (const k of cls.turnAirs) {
              const ra = this.advance(air, pj, Infinity, k, null)
              if (ra !== FLYING) {
                // ended on the first heading: every later turn is this program
                if (ra >= 0) found.push({ ...pj, turnAir: k, ticks: ra })
                break
              }
              // flown out of reach on the first heading: so is every later turn
              if (falls && !this.canReach(air.b, cls.sprint, t[0], t[1], t[2])) break
              aimed.length = 0
              if (cls.aim) {
                if (catchTarget) aimed.push(point(tcx, tcz))
                else this.aimHeadings(air.b, cls.sprint, t[0], t[2], landY, aimed)
              }
              for (let h = 0; h < cls.air2s.length + aimed.length; h++) {
                const pk: HopProgram = { ...pj, turnAir: k, air2: h < cls.air2s.length ? cls.air2s[h] : aimed[h - cls.air2s.length] }
                const rb = this.advance(flightInto(branch, air), pk, Infinity, Infinity, null)
                if (rb >= 0) found.push({ ...pk, ticks: rb })
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
    return this.quickestRobust(found, deadline)
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
      if (performance.now() > deadline) { this.noVerdict = true; return }
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

  /** The quickest of these landings that is also robust (ties: the first found, the simplest). */
  private quickestRobust (found: HopProgram[], deadline: number): HopProgram | null {
    const order = found.map((p, i) => i).sort((a, b) => found[a].ticks - found[b].ticks || a - b)
    for (const i of order) {
      if (performance.now() > deadline) { this.noVerdict = true; break }
      const ticks = this.robust(found[i])
      if (ticks >= 0) return { ...found[i], ticks }
    }
    return null
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
  hop (r: [number, number, number], t: [number, number, number], deadline: number, families: HopFamilies = 'simple', grid = false, corridor: ReadonlyArray<{ x: number, y: number, z: number }> = []): HopProgram | null {
    this.noVerdict = true
    const lands = this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return null
    const pads = this.padsAt(r[0], r[1], r[2])
    if (pads.length === 0) return null
    this.noVerdict = false
    if (families === 'beam') return this.beamSearch(pads, lands, t, deadline, corridor)
    for (const family of FAMILIES[families]) {
      if (performance.now() > deadline) { this.noVerdict = true; break }
      const p = this.search(pads, lands, t, family, deadline, false, grid)
      if (p !== null) return p
    }
    return null
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
   * A robust LIVE program to node `t` from body state `b` exactly as it is
   * (the executor's take-off, where no gate would fly the jump): every
   * family (the yaw family on the grid of turns too), walking jumps too, and
   * the quickest that lands — the body is already where the program starts,
   * so there is no line-up to weigh it against. `yaw`: the body's yaw now,
   * which makes the headings the ones the replay's bot.look will turn to
   * (NaN: unknown). When no family lands, the beam (beamSearch). Null if
   * nothing lands.
   */
  hopFrom (b: SimBody, t: [number, number, number], deadline: number, yaw = NaN): HopProgram | null {
    const lands = this.stands(t[0], t[1], t[2])
    if (lands.length === 0) return null
    const live = copyBody(newBody(0, 0, 0), b)
    const pads: Pad[] = [{ f: { b: copyBody(newBody(0, 0, 0), b), t: 0, fired: false, air: 0, rebounds: 0, yaw, leg: 0 }, live }]
    let best: HopProgram | null = null
    for (const family of FAMILIES.all) {
      if (performance.now() > deadline) break
      const p = this.search(pads, lands, t, family, deadline, true, true)
      if (p !== null && (best === null || p.ticks < best.ticks)) best = p
    }
    return best ?? this.beamSearch(pads, lands, t, deadline)
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
  discover (
    sources: ReadonlyArray<{ x: number, y: number, z: number }>,
    heuristic: (x: number, y: number, z: number) => number,
    budgetMs: number,
    skip: ((x: number, y: number, z: number) => boolean) | null = null,
    maxEdges = MAX_EDGES,
    families: HopFamilies = 'simple'
  ): HopEdge[] {
    const deadline = performance.now() + budgetMs
    this.ticksFlown = 0
    const out: HopEdge[] = []
    const seenTargets = new Set<number>()
    // nearest the goal first: the best partial path ends at the closest node
    for (let si = sources.length - 1; si >= 0 && performance.now() < deadline; si--) {
      const r = sources[si]
      if (this.stands(r.x, r.y, r.z).length === 0) continue
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
      cands.sort((p, q) => p[3] - q[3])
      let tried = 0
      for (const [x, y, z] of cands) {
        if (performance.now() > deadline) break
        const key = ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
        if (seenTargets.has(key)) continue
        if (this.stands(x, y, z).length === 0) continue
        seenTargets.add(key)
        if ((families === 'yaw' && ++tried > YAW_CANDIDATES) || (families === 'beam' && ++tried > BEAM_CANDIDATES)) break
        // (the beam follows the partial path on from here: a chain carries its momentum along it)
        const p = this.hop([r.x, r.y, r.z], [x, y, z], deadline, families, false, families === 'beam' ? sources.slice(si + 1) : [])
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

/** Which program families a search flies (HopOracle.hop). */
export type HopFamilies = 'simple' | 'yaw' | 'all' | 'beam'

/** Horizontal hop range (blocks) and its square: a 6.5 disc covers every vanilla gap jump. */
const HOP_REACH = 6
const HOP_REACH_SQ = 6.5 * 6.5
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
/** Edges one discover() returns at most. */
const MAX_EDGES = 4
/** Candidate landings per source the yaw escalation flies, and the beam escalation searches (discover). */
const YAW_CANDIDATES = 12
const BEAM_CANDIDATES = 4
