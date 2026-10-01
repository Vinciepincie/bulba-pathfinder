// Port of mineflayer-pathfinder/lib/physics.js (MIT): the executor's
// sprint/jump decisions come from short prismarine-physics simulations
// against the LIVE world — same engine the bot moves with, so path following
// inherits none of the solver's approximations.
import { PlayerState } from 'prismarine-physics'
import type { SimControl } from 'prismarine-physics'
import type { Bot } from 'mineflayer'
import { Vec3 } from 'vec3'
import { carryCode, CARRY_W_NONE, CARRY_S_NONE } from './shapes.js'
import { PlayerSim, SIM, newBody, simTablesFromLut } from './playerSim.js'
import type { SimBody, SimGrid, SimWorld } from './playerSim.js'
import { gaitSearch, kWalkable, GRAZE_YAW, HOP_MAX_DIP } from './gaitSearch.js'
import type { GaitGoal, GaitNode } from './gaitSearch.js'
import type { BlockLut } from './lut.js'
import { HopOracle } from './hopOracle.js'
import type { HopProgram } from './hopOracle.js'

/** prismarine-physics' sprinting modifier id (index.js physics.sprintingUUID). */
const SPRINTING_UUID = '662a6b8d-da3e-4c1c-8813-96ea6097278d'

type Controller = (state: PlayerState, tick: number) => void
type Refusal = (state: PlayerState) => boolean

/**
 * Horizontal headway, per tick, below which a body already pressed against a
 * block face counts as GRINDING rather than sliding. See `grindGuard`.
 */
const GRIND_PROGRESS = 0.01

/**
 * How much further along the path a hop has to get, over the comparison
 * horizon, before it is worth leaving the ground. A whole node of lead keeps
 * the gait from flickering on ground the two are level on.
 */
const HOP_MARGIN = 0.5


/**
 * Take-off heading offsets the angle solver tries, in radians, smallest
 * first. +-30 degrees covers "turn to clear the corner in the way" without
 * ever aiming at something the planner did not route through; anything wider
 * is a different move, not a better line.
 */
const HEADING_OFFSETS = [0, 0.13, -0.13, 0.26, -0.26, 0.39, -0.39, 0.52, -0.52]

/**
 * Blocks a sprint-hop must not lift the head into. Walking never reaches this
 * airspace, so the planner has no opinion about it — but a hop puts the head
 * three blocks above the node, and finding a floating water block up there
 * costs the crossing (swim drag, and the breath clock starts) while finding
 * lava costs the bot.
 */
const OVERHEAD_HAZARDS = new Set([
  'water', 'flowing_water', 'bubble_column', 'lava', 'flowing_lava'
])

/**
 * Feet blocks that catch a falling body without ever grounding it. The
 * extended repertoire jumps at these deliberately — see `caught`.
 */
const CAUGHT_FEET = new Set(['ladder', 'vine'])

/**
 * In-air control options, strongest forward first. Every rollout used to
 * hold forward + sprint to touchdown, and so did the flight itself: right
 * for a landing with ground to overrun, and the one thing that cannot land a
 * 4-block jump on a pane post — the arc only ever overshoots it. A player
 * lets go of W, or taps S, in the air.
 */
export interface AirControl { forward: boolean, back: boolean, sprint: boolean }
const AIR_OPTS: readonly AirControl[] = [
  { forward: true, back: false, sprint: true },
  { forward: true, back: false, sprint: false },
  { forward: false, back: false, sprint: false },
  { forward: false, back: true, sprint: false }
]
/** Air ticks of held forward before the tail option, tried by the take-off gate. */
const AIR_RELEASE_TICKS = [8, 6, 4, 2, 0]
/** Along-track miss (blocks) past which a flight counts as over- or undershooting its node. */
/** PF_AIR_GATE=0 turns the controlled take-off gate off for A/B runs. */
const AIR_GATE = process.env.PF_AIR_GATE !== '0'
const AIR_MISS_TOL = 0.05
/** A predicted touchdown within this of the node's aim is left alone by the in-flight controller. */
const AIR_CENTRE_TOL = 0.1

/**
 * A SCRIPTED jump (improvement): the controls of one jump, found by search in
 * the physics sim and replayed tick for tick. Headings are absolute yaws:
 * `yawA` from now until `turnTick` ticks after leaving the ground, `yawB`
 * from then to touchdown — the mid-air turn of a neo, of a jump threaded
 * through a gap, of a landing that has to come in from the side. `track` is
 * the predicted position after each tick, which the executor holds the body
 * to.
 */
export interface JumpScript {
  yawA: number
  yawB: number
  turnTick: number
  sprint: boolean
  jumpAfter: number
  /** Ticks to touchdown. */
  ticks: number
  track: Array<{ x: number, y: number, z: number }>
  /** How far from the node's aim the body comes to rest. */
  miss: number
}
/** Heading offsets (radians) from the bearing to the node, for the two phases of a scripted jump. */
// Measured on a one-block neo (a 2-high pillar in the line, landing right
// behind it): what lands it is a RUN of 6-14 ticks pressed along the pillar's
// face and out to the corner of the block, the jump on the last supported
// tick, and a turn of 0.6-1.5 rad back in within the first four air ticks.
const SCRIPT_YAW_A = [0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6, 0.9, -0.9]
const SCRIPT_YAW_B = [0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.2, -1.2, 1.5, -1.5]
const SCRIPT_TURN_TICKS = [1, 2, 3, 4, 6, 8]
const SCRIPT_JUMP_AFTER = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14]
/** A script that comes to rest this close to the node's aim ends the search early. */
const SCRIPT_GOOD_MISS = 0.45

/** Half-extent of the per-tick block cache around the body, in blocks. */
const CACHE_REACH = 64
/** The kernel's live window (PhysicsSim.kGrid): cells across, and up. */
const KGRID_W = 64
const KGRID_H = 48

/** Height tolerance for reaching a WALKING node in a rollout (see getReached). */
const WALK_REACH_DY = 0.5
/** How far under the lower of the body and the node a straight line may dip: what a step climbs back out of (canStraightLine). */
const STRAIGHT_MAX_DIP = 0.6

/**
 * Nodes ahead the sprint-hop gait needs backed by path and free of rises
 * and jumps (see sprintHopBetter). PF_HOP_LOOK overrides it for A/B runs.
 */
const HOP_LOOK = Math.max(2, Number(process.env.PF_HOP_LOOK ?? 6) || 6)

/** Path nodes the gait scans for the jump it flies the approach into (gaitVerdict). */
const HOP_JUMP_LOOK = 12
/** Path nodes the gait over ground that takes no jump is flown to (gaitVerdict, gaitSearch). */
const GAIT_LOOK = 9


export class PhysicsSim {
  private readonly bot: Bot
  private readonly world: { getBlock: (pos: Vec3) => unknown }
  /** Set by the last simulateUntil that hit its refusal predicate. */
  private refused = false
  /**
   * Per-tick caches (improvement). Every rollout tick asks the world for the
   * ~30 blocks around the body, and every rollout rebuilds a PlayerState —
   * effect lookups and a boots-NBT parse each time. A refused parkour node
   * runs ~1,300 simulated ticks and ~15 rollouts per executor tick, all
   * against a world that cannot change inside the tick. `beginTick` opens
   * a tick: blocks are memoised by cell and the state is built once and
   * cloned. Without it (tests, out-of-tick callers) nothing is cached.
   */
  private tickSerial = 0
  private cacheX0 = 0
  private cacheY0 = 0
  private cacheZ0 = 0
  private readonly blockCache = new Map<number, unknown>()
  private seed: PlayerState | null = null
  /** Why the last sprintHopBetter said no ('' = it said yes). Trace diagnostics. */
  hopRefusal = ''

  constructor (bot: Bot) {
    this.bot = bot
    this.world = { getBlock: (pos: Vec3) => this.blockAt(pos) }
  }

  // ── kernel (src/playerSim.ts) over the live world ─────────────────────
  //
  // The exact physics prismarine-physics runs, allocation-free: rollouts that
  // would cost a millisecond each in prismarine cost microseconds here, which
  // is what lets a decision look past the next take-off instead of stopping
  // at it. Built lazily from the LUT's shape table (setShapeTables); absent
  // until the plugin hands one over, and every caller falls back without it.
  private kernel: PlayerSim | null = null
  private kTables: Omit<SimWorld, 'stateAt'> | null = null
  private kTablesLut: BlockLut | null = null
  /**
   * The kernel's window on the live world: a lazy grid round the body, read
   * through once per cell and forgotten every tick (SimGrid.stamps) — array
   * reads where a map lookup per cell was most of a rollout's cost.
   */
  private readonly kGrid: SimGrid & { stamps: Int32Array, serial: number } = {
    states: new Int32Array(KGRID_W * KGRID_H * KGRID_W),
    stamps: new Int32Array(KGRID_W * KGRID_H * KGRID_W).fill(-1),
    serial: 1,
    x0: 0,
    y0: 0,
    z0: 0,
    w: KGRID_W,
    h: KGRID_H,
    l: KGRID_W
  }
  private readonly kProbe = { x: 0, y: 0, z: 0 }

  /** Hand the kernel its per-state tables (shapes, slipperiness, liquids). Cheap to repeat. */
  setShapeTables (lut: BlockLut): void {
    if (this.kTablesLut === lut) return
    this.kTablesLut = lut
    this.kTables = simTablesFromLut(lut)
    this.kernel = null
  }

  /** State id at a cell in the live world; -1 unloaded. (The kernel reads it through kGrid.) */
  private kStateAt (x: number, y: number, z: number): number {
    let s = -1
    try {
      const col = (this.bot.world as unknown as { getColumn: (cx: number, cz: number) => { getBlockStateId: (p: { x: number, y: number, z: number }) => number } | null | undefined }).getColumn(x >> 4, z >> 4)
      if (col) {
        this.kProbe.x = x & 15
        this.kProbe.y = y
        this.kProbe.z = z & 15
        const v = col.getBlockStateId(this.kProbe)
        if (typeof v === 'number' && v >= 0) s = v
      }
    } catch { s = -1 }
    return s
  }

  /** Forget the kernel's window and centre it on the body. */
  private kGridRefresh (): void {
    const g = this.kGrid
    g.serial++
    const p = this.bot.entity?.position
    if (p === undefined) return
    g.x0 = Math.floor(p.x) - (KGRID_W >> 1)
    g.y0 = Math.floor(p.y) - (KGRID_H >> 1)
    g.z0 = Math.floor(p.z) - (KGRID_W >> 1)
  }

  /**
   * The movement-speed attribute without sprinting, exactly as prismarine
   * reads it (index.js moveEntityWithHeading: server attributes, the sprint
   * modifier removed; the kernel adds sprint back the same way).
   */
  private kSpeedBase (): number {
    const reg = this.bot.registry as unknown as { attributesByName?: Record<string, { resource?: string }> }
    const keyName = reg.attributesByName?.movementSpeed?.resource
    const attrs = (this.bot.entity as unknown as { attributes?: Record<string, { value: number, modifiers: Array<{ uuid: string, amount: number, operation: number }> }> }).attributes
    const attr = keyName !== undefined ? attrs?.[keyName] : undefined
    if (attr === undefined) return 0.1
    const mods = attr.modifiers.filter(m => m.uuid !== SPRINTING_UUID)
    let x = attr.value
    for (const m of mods) if (m.operation === 0) x += m.amount
    let y = x
    for (const m of mods) if (m.operation === 1) y += x * m.amount
    for (const m of mods) if (m.operation === 2) y += y * m.amount
    return y
  }

  /** The movement-speed attribute without sprinting (prismarine's reading), for a kernel elsewhere (the worker). */
  walkSpeed (): number {
    return this.kSpeedBase()
  }

  /** The kernel, current with the bot's hitbox and speed; null without shape tables. */
  private getKernel (): PlayerSim | null {
    if (this.kTables === null) return null
    const ph = this.bot.physics as unknown as { playerHalfWidth?: number, playerHeight?: number }
    if (this.kernel === null) {
      this.kernel = new PlayerSim({ ...this.kTables, stateAt: (x, y, z) => this.kStateAt(x, y, z), grid: this.kGrid })
    }
    // (out of a tick — tests, callers between ticks — nothing may be kept)
    if (this.tickSerial === 0) this.kGridRefresh()
    this.kernel.halfWidth = ph.playerHalfWidth ?? 0.3
    this.kernel.height = ph.playerHeight ?? 1.8
    this.kernel.speedBase = this.kSpeedBase()
    return this.kernel
  }

  /** The body now, as a kernel body. */
  private kBodyNow (): SimBody {
    const e = this.bot.entity as unknown as { position: Vec3, velocity: Vec3, onGround?: boolean, isCollidedHorizontally?: boolean }
    const b = newBody(e.position.x, e.position.y, e.position.z)
    b.vx = e.velocity.x
    b.vy = e.velocity.y
    b.vz = e.velocity.z
    b.onGround = e.onGround === true
    b.collidedH = e.isCollidedHorizontally === true
    b.jumpTicks = (this.bot as unknown as { jumpTicks?: number }).jumpTicks ?? 0
    return b
  }

  /** Open a tick: forget the last tick's blocks and state template. */
  beginTick (): void {
    this.tickSerial++
    this.blockCache.clear()
    this.kGridRefresh()
    this.seed = null
    const p = this.bot.entity?.position
    if (p !== undefined) {
      this.cacheX0 = Math.floor(p.x) - CACHE_REACH
      this.cacheY0 = Math.floor(p.y) - CACHE_REACH
      this.cacheZ0 = Math.floor(p.z) - CACHE_REACH
    }
  }

  private blockAt (pos: Vec3): unknown {
    if (this.tickSerial === 0) return this.bot.blockAt(pos, false)
    const x = Math.floor(pos.x) - this.cacheX0
    const y = Math.floor(pos.y) - this.cacheY0
    const z = Math.floor(pos.z) - this.cacheZ0
    if (x < 0 || x >= 2 * CACHE_REACH || y < 0 || y >= 2 * CACHE_REACH || z < 0 || z >= 2 * CACHE_REACH) {
      return this.bot.blockAt(pos, false)
    }
    const key = (x << 14) | (y << 7) | z
    let b = this.blockCache.get(key)
    if (b === undefined) {
      b = this.bot.blockAt(pos, false)
      this.blockCache.set(key, b)
    }
    return b
  }

  /**
   * A fresh PlayerState for a rollout starting from the body's state now.
   * Inside a tick the expensive, tick-invariant parts (effects, attributes,
   * enchantments) come from one template; position, velocity, look and the
   * controls are always taken fresh.
   */
  private newState (control: SimControl): PlayerState {
    if (this.tickSerial === 0) return new PlayerState(this.bot, control)
    if (this.seed === null) this.seed = new PlayerState(this.bot, control)
    const seed = this.seed
    const s = Object.create(Object.getPrototypeOf(seed) as object) as PlayerState
    Object.assign(s, seed)
    s.pos = this.bot.entity.position.clone()
    s.vel = this.bot.entity.velocity.clone()
    s.yaw = this.bot.entity.yaw
    ;(s as unknown as { pitch: number }).pitch = this.bot.entity.pitch
    s.onGround = this.bot.entity.onGround
    s.control = control
    return s
  }

  simulateUntil (
    goal: (state: PlayerState) => boolean,
    controller: Controller = () => {},
    ticks = 1,
    state: PlayerState | null = null,
    refuse: Refusal | null = null
  ): PlayerState {
    if (!state) {
      const simulationControl = {
        forward: this.bot.controlState.forward,
        back: this.bot.controlState.back,
        left: this.bot.controlState.left,
        right: this.bot.controlState.right,
        jump: this.bot.controlState.jump,
        sprint: this.bot.controlState.sprint,
        sneak: this.bot.controlState.sneak
      }
      state = this.newState(simulationControl)
    }

    this.refused = false
    const simulatePlayer = (this.bot.physics as unknown as { simulatePlayer: (state: PlayerState, world: unknown) => void }).simulatePlayer
    for (let i = 0; i < ticks; i++) {
      controller(state, i)
      simulatePlayer.call(this.bot.physics, state, this.world)
      if (state.isInLava) return state
      if (refuse !== null && refuse(state)) {
        this.refused = true
        return state
      }
      if (goal(state)) return state
    }

    return state
  }

  /**
   * A tick that is going nowhere: the body is pressed into a block face and
   * makes no headway along the line to the node.
   *
   * This is only ever an EARLY-OUT and a symptom test now, never a verdict.
   * It used to veto jump rollouts too, because a body at exact contact with a
   * face was a position the 1.21.x server refused outright — but that was the
   * hitbox-precision bug (see plugin.ts hitboxPrecisionFix), and with the
   * dimensions nudged off their boundaries the same jump goes from 19
   * corrections and no movement to zero corrections and a clean climb. Vetoing
   * on contact after that fix would refuse jumps the server is perfectly
   * happy with.
   */
  private grindGuard (target: { x: number, y: number, z: number }, from: { x: number, z: number }): Refusal {
    let prev = Math.hypot(target.x - from.x, target.z - from.z)
    return (state: PlayerState) => {
      const d = Math.hypot(target.x - state.pos.x, target.z - state.pos.z)
      const stuck = state.isCollidedHorizontally === true && prev - d <= GRIND_PROGRESS
      prev = d
      return stuck
    }
  }

  simulateUntilNextTick (): PlayerState {
    return this.simulateUntil(() => false, () => {}, 1)
  }

  simulateUntilOnGround (ticks = 5): PlayerState {
    return this.simulateUntil(state => state.onGround, () => {}, ticks)
  }

  canStraightLine (path: Array<{ x: number, y: number, z: number }>, sprint = false): boolean {
    const reached = this.getReached(path)
    // The grind guard is a pure early-out here, not a change of verdict: a
    // walk that is pressed into a face and making no headway is not going to
    // reach the node in the remaining 190-odd ticks either. It runs 200 ticks
    // of full physics per call, twice per executor tick, and a wedged node
    // paid all of it — this is the single biggest slice of the executor's
    // per-tick cost.
    //
    // A straight line stays on the path's level (STRAIGHT_MAX_DIP): the
    // rollout has 200 ticks, and a body that walks off a ledge has them to
    // find its way round to the node on the ground below. Traced on the
    // arena's spiral3-c: three up from a node five blocks out, the gate said
    // "walk" on the strength of a slope from the floor under the ledge up to
    // that node, the jump it stood in front of was never asked, and the body
    // walked off — a level of the spiral to climb again, 20 s.
    const grind = this.grindGuard(path[0], this.bot.entity.position)
    const low = Math.min(this.bot.entity.position.y, path[0].y) - STRAIGHT_MAX_DIP
    const state = this.simulateUntil(
      reached, this.getController(path[0], false, sprint), 200, null,
      s => grind(s) || (s.pos.y < low && s.isInWater !== true)
    )
    if (reached(state)) return true

    if (sprint) {
      if (this.canSprintJump(path, 0)) return false
    } else {
      if (this.canWalkJump(path, 0)) return false
    }

    for (let i = 1; i < 7; i++) {
      if (sprint) {
        if (this.canSprintJump(path, i)) return true
      } else {
        if (this.canWalkJump(path, i)) return true
      }
    }
    return false
  }

  canStraightLineBetween (n1: Vec3, n2: Vec3): boolean {
    const reached = (state: PlayerState): boolean => {
      const delta = n2.minus(state.pos)
      const r2 = 0.15 * 0.15
      return (delta.x * delta.x + delta.z * delta.z) <= r2 && Math.abs(delta.y) < 0.001 && ((state.onGround as boolean) || (state.isInWater as boolean))
    }
    const simulationControl = {
      forward: this.bot.controlState.forward,
      back: this.bot.controlState.back,
      left: this.bot.controlState.left,
      right: this.bot.controlState.right,
      jump: this.bot.controlState.jump,
      sprint: this.bot.controlState.sprint,
      sneak: this.bot.controlState.sneak
    }
    const state = this.newState(simulationControl)
    state.pos.update(n1)
    this.simulateUntil(reached, this.getController(n2, false, true), Math.floor(5 * n1.distanceTo(n2)), state)
    return reached(state)
  }

  // 45 ticks, not upstream's 20: extended-parkour drop landings spend up to
  // ~18 ticks airborne and then run in to the node center — at 20 the sim
  // budget expired mid-flight and legal deep drops were never attempted.
  canSprintJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    if (!this.takeoffReady(path[0])) return false
    const reached = this.getReached(path)
    if (this.jumpLands(path, reached, true, jumpAfter, 0)) {
      // A flight that touches a wall on the way is only as good as its
      // alignment: it has to land with the heading a hair either side too.
      if (!this.flightGrazed || (path[0] as { parkour?: boolean }).parkour !== true) return true
      return this.jumpLands(path, reached, true, jumpAfter, GRAZE_YAW) && this.jumpLands(path, reached, true, jumpAfter, -GRAZE_YAW)
    }
    return this.canJumpControlled(path[0], true, jumpAfter)
  }

  /**
   * One jump rollout: does it reach the node and land there? Sets
   * `flightGrazed` when the body was pressed against a face while airborne —
   * a flight whose outcome turns on centimetres (arena mcc-8-1: a (−4,−2)
   * jump grazed the pillar beside its landing, the rollout got onto the
   * block with 0.02 to spare, the live flight — a rounding error away —
   * slid down its side).
   */
  private jumpLands (
    path: Array<{ x: number, y: number, z: number, parkour?: boolean }>,
    reached: (state: PlayerState) => boolean,
    sprint: boolean,
    jumpAfter: number,
    headingOffset: number
  ): boolean {
    let grazed = false
    const state = this.simulateUntil(s => {
      if (s.isCollidedHorizontally === true && s.onGround !== true) grazed = true
      return reached(s)
    }, this.getController(path[0], true, sprint, jumpAfter, headingOffset), 45)
    this.flightGrazed = grazed
    return reached(state) && this.landsThere(path[0], state, sprint)
  }

  private flightGrazed = false

  canWalkJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    if (!this.takeoffReady(path[0])) return false
    const reached = this.getReached(path)
    if (this.jumpLands(path, reached, false, jumpAfter, 0)) {
      if (!this.flightGrazed || (path[0] as { parkour?: boolean }).parkour !== true) return true
      return this.jumpLands(path, reached, false, jumpAfter, GRAZE_YAW) && this.jumpLands(path, reached, false, jumpAfter, -GRAZE_YAW)
    }
    return this.canJumpControlled(path[0], false, jumpAfter)
  }

  /**
   * Does the body, flown with `opt` held from now to touchdown, come down ON
   * the node — and if not, by how much does it pass it? `err` is the signed
   * along-track miss where the feet come down through the node's height:
   * positive = long. Shared by the take-off gate and the in-flight controller.
   */
  private flyOut (
    node: { x: number, y: number, z: number },
    controller: Controller,
    state: PlayerState | null,
    ticks: number
  ): { lands: boolean, err: number, dist: number } {
    const p0 = state !== null ? state.pos : this.bot.entity.position
    let ux = node.x - p0.x
    let uz = node.z - p0.z
    const ul = Math.hypot(ux, uz)
    if (ul > 1e-6) { ux /= ul; uz /= ul }
    let err = NaN
    let prevY = p0.y
    const end = this.simulateUntil(s => {
      // the crossing: feet come down through the node's level
      if (Number.isNaN(err) && s.vel.y < 0 && prevY >= node.y && s.pos.y <= node.y + 0.001) {
        err = (s.pos.x - node.x) * ux + (s.pos.z - node.z) * uz
      }
      prevY = s.pos.y
      return this.caught(s) || s.pos.y < node.y - 1.5
    }, controller, ticks, state)
    const dist = Math.hypot(node.x - end.pos.x, node.z - end.pos.z)
    const lands = this.caught(end) && dist <= 1 && Math.abs(node.y - end.pos.y) < 1
    if (Number.isNaN(err)) err = (end.pos.x - node.x) * ux + (end.pos.z - node.z) * uz
    return { lands, err, dist }
  }

  /** Controller holding one air option, aimed at the node every tick. */
  private airController (node: { x: number, z: number }, opt: AirControl): Controller {
    return (state: PlayerState) => {
      state.yaw = Math.atan2(-(node.x - state.pos.x), -(node.z - state.pos.z))
      state.control.forward = opt.forward
      state.control.back = opt.back
      state.control.left = false
      state.control.right = false
      state.control.sneak = false
      state.control.jump = false
      state.control.sprint = opt.sprint
    }
  }

  /**
   * The take-off gate with air control (improvement). The plain rollout
   * holds forward to touchdown; where that flies OVER a parkour node, a jump
   * that lets go of forward part-way (or pulls back) may still land it. Tried
   * only on that evidence — a flight that comes down past its node — so a
   * jump that is simply out of reach costs nothing extra.
   */
  private canJumpControlled (
    node: { x: number, y: number, z: number, parkour?: boolean, narrowLanding?: boolean | null },
    sprint: boolean,
    jumpAfter: number
  ): boolean {
    // Only for a jump taken now or within two ticks: the later take-offs of
    // the delayed-jump scan are the same flight from a little further on.
    // And only onto a NARROW landing: that is where the executor flies with air
    // control (plugin.ts airNarrow). Authorising a release-forward flight onto a
    // full block, which is then flown with forward held, is how mcc-8-1 clipped
    // the pillar in front of its landing and fell.
    if (!AIR_GATE || node.parkour !== true || node.narrowLanding !== true || jumpAfter > 2) return false
    const probe = this.flyOut(node, this.getController(node, true, sprint, jumpAfter), null, 45)
    if (probe.lands || !(probe.err > AIR_MISS_TOL)) return false
    for (const tail of [AIR_OPTS[2], AIR_OPTS[3]]) {
      for (const hold of AIR_RELEASE_TICKS) {
        let air = 0
        let fired = false
        const run = this.getController(node, true, sprint, jumpAfter)
        const ctl: Controller = (state, tick) => {
          run(state, tick)
          if (!fired) { if (state.vel.y > 0 && state.onGround !== true) fired = true } else air++
          if (fired && air > hold) {
            state.control.forward = tail.forward
            state.control.back = tail.back
            state.control.sprint = tail.sprint
            state.control.jump = false
          }
        }
        if (this.flyOut(node, ctl, null, 45).lands) return true
      }
    }
    return false
  }

  /**
   * The search behind a scripted jump, resumable: every call tries up to
   * `budget` candidate scripts from the body's state NOW (which must not
   * change between calls — the executor holds the body still) and returns
   * the best script once the space is exhausted, null if nothing lands, or
   * undefined while there are candidates left. A candidate lands when the
   * body comes down ON the node's level within a block of its aim and is
   * still there, at rest, after six ticks of letting go — a touchdown that
   * slides off the far side is not a landing.
   */
  /**
   * MEASURED AND REVERTED (2026-09-18): a wall-clock slice (35 ms) on this
   * search, because its budget is a CANDIDATE count and a candidate is a
   * physics rollout whose cost varies with the course — 400 of them cost
   * 16 ms on the arena's paradise3-l49 and 254 ms on mcc-8-1, five times the
   * 50 ms tick, so the handler blocks the bot's own physics loop. The slice
   * fixed that and cost a route: spreading the same search over 5x the ticks
   * left it short of a script inside the futility window, parkouradv1 then
   * got stuck where it never had, and its post-ban replan (30-46k visited
   * against 5k) blew the think timeout — 3 of 4 attempts against 0 of 2.
   * Worth redoing, but only with the search off the tick loop (a worker), not
   * by stretching it across more ticks.
   */
  solveJump (node: { x: number, y: number, z: number }, budget: number): JumpScript | null | undefined {
    const p = this.bot.entity.position
    if (this.solveNode !== node || this.solveFrom === null ||
        Math.abs(this.solveFrom.x - p.x) > 1e-6 || Math.abs(this.solveFrom.z - p.z) > 1e-6 || Math.abs(this.solveFrom.y - p.y) > 1e-6) {
      this.solveNode = node
      this.solveFrom = p.clone()
      this.solveCursor = 0
      this.solveBest = null
    }
    const bearing = Math.atan2(-(node.x - p.x), -(node.z - p.z))
    const nA = SCRIPT_YAW_A.length
    const nB = SCRIPT_YAW_B.length
    const nT = SCRIPT_TURN_TICKS.length
    const nJ = SCRIPT_JUMP_AFTER.length
    const total = nA * nB * nT * nJ * 2
    for (let n = 0; n < budget && this.solveCursor < total; n++, this.solveCursor++) {
      let c = this.solveCursor
      const sprint = c % 2 === 0; c = (c - c % 2) / 2
      const jumpAfter = SCRIPT_JUMP_AFTER[c % nJ]; c = (c - c % nJ) / nJ
      const turnTick = SCRIPT_TURN_TICKS[c % nT]; c = (c - c % nT) / nT
      const yawB = bearing + SCRIPT_YAW_B[c % nB]; c = (c - c % nB) / nB
      const yawA = bearing + SCRIPT_YAW_A[c % nA]
      const script = this.runScript(node, yawA, yawB, turnTick, sprint, jumpAfter)
      if (script !== null && (this.solveBest === null || script.miss < this.solveBest.miss)) this.solveBest = script
      // good enough: the candidates are ordered gentlest first
      if (this.solveBest !== null && this.solveBest.miss <= SCRIPT_GOOD_MISS) { this.solveCursor = total; break }
    }
    if (this.solveCursor < total) return undefined
    const best = this.solveBest
    this.solveNode = null
    return best
  }

  private solveNode: unknown = null
  private solveFrom: Vec3 | null = null
  private solveCursor = 0
  private solveBest: JumpScript | null = null

  /** Fly one candidate script in the sim; the script if it lands and stays, else null. */
  private runScript (
    node: { x: number, y: number, z: number },
    yawA: number, yawB: number, turnTick: number, sprint: boolean, jumpAfter: number
  ): JumpScript | null {
    let fired = false
    let air = 0
    const track: Array<{ x: number, y: number, z: number }> = []
    const ctl: Controller = (state, tick) => {
      if (!fired) { if (state.vel.y > 0 && state.onGround !== true) fired = true } else air++
      state.yaw = fired && air >= turnTick ? yawB : yawA
      state.control.forward = true
      state.control.back = false
      state.control.left = false
      state.control.right = false
      state.control.sneak = false
      state.control.sprint = sprint
      state.control.jump = !fired && tick >= jumpAfter
    }
    let left = false
    const end = this.simulateUntil(s => {
      track.push({ x: s.pos.x, y: s.pos.y, z: s.pos.z })
      if (s.onGround !== true) left = true
      return (left && this.caught(s)) || s.pos.y < node.y - 1.5
    }, ctl, 40)
    if (!left || !this.caught(end) || Math.abs(end.pos.y - node.y) > 0.3 ||
        Math.hypot(end.pos.x - node.x, end.pos.z - node.z) > 1) return null
    const ticks = track.length
    // let go and see that it stays: sneaking, no keys
    const rest = this.simulateUntil(() => false, (s: PlayerState) => {
      s.control.forward = false
      s.control.back = false
      s.control.left = false
      s.control.right = false
      s.control.jump = false
      s.control.sprint = false
      s.control.sneak = true
    }, 6, end)
    if (!this.caught(rest) || Math.abs(rest.pos.y - node.y) > 0.3) return null
    return { yawA, yawB, turnTick, sprint, jumpAfter, ticks, track, miss: Math.hypot(rest.pos.x - node.x, rest.pos.z - node.z) }
  }

  /**
   * In-flight landing control (improvement): what to hold THIS tick so the
   * flight comes down on its node. The executor asks only for a narrow
   * landing support (a post, a pane, a skull).
   *
   * It ENGAGES only when the default — forward + sprint held to touchdown —
   * is predicted to miss: every jump the default lands flies exactly as it
   * always did (arena spiral3-b lands an open trapdoor's panel against a
   * wall with a few centimetres in hand, and "centring" that flight took a
   * tick of sprint off it and dropped it 0.03 short). Once engaged for a
   * flight it stays engaged and steers the predicted touchdown to the
   * node's aim — on the along-track miss where the feet come down through
   * the node's level, not on hit-or-miss, which at the edge of a 2/16 bar
   * differ by less than the rollout is good for: the option that brings the
   * miss closest to zero, re-chosen every tick, so the flight alternates
   * between the two options that bracket the node and settles on it.
   */
  airControl (node: { x: number, y: number, z: number }, engaged: boolean): { control: AirControl | null, engaged: boolean } {
    const first = this.flyOut(node, this.airController(node, AIR_OPTS[0]), null, 40)
    if (!engaged && first.lands) return { control: null, engaged: false }
    if (!(first.err > AIR_CENTRE_TOL)) return { control: null, engaged: true } // short or centred: forward is all there is
    let best = 0
    let bestAbs = Math.abs(first.err)
    for (let i = 1; i < AIR_OPTS.length; i++) {
      const r = this.flyOut(node, this.airController(node, AIR_OPTS[i]), null, 40)
      const a = Math.abs(r.err)
      if (a < bestAbs) { best = i; bestAbs = a }
      if (r.err < 0) break // the rest only fall shorter
    }
    return { control: best === 0 ? null : AIR_OPTS[best], engaged: true }
  }

  /**
   * A parkour take-off is committed to from the GROUND, never mid-air.
   *
   * Asked while the bot is still falling out of the previous jump, the
   * rollout has to guess the speed it will land with — and a marginal jump is
   * decided entirely by that number. On the arena's basic1 the plan ends with
   * a 5-block drop-jump immediately followed by a 4-block flat one across a
   * chasm: authorised in the air, the second take-off happened with whatever
   * speed the landing happened to leave, made it about one run in three, and
   * fell 40 blocks the rest of the time — on both gaits, so it was the timing
   * and not the hop.
   *
   * Nothing is lost by waiting: a jump can only start from the ground anyway,
   * and the executor re-decides every tick, so the same rollout runs again on
   * the landing tick with the speed it actually has. Meanwhile the in-flight
   * branch keeps forward held, so the current jump is still flown out.
   */
  private takeoffReady (node: { x: number, y: number, z: number, parkour?: boolean }): boolean {
    if (node.parkour !== true) return true
    return (this.bot.entity as { onGround?: boolean }).onGround === true
  }

  /**
   * Did the jump ARRIVE, or merely pass through?
   *
   * `getReached` is upstream's box: |dx|,|dz| <= 0.35 and |dy| < 1, with no
   * requirement to be standing on anything. That is fine for a walk, where
   * satisfying the box means being there — but a jump can satisfy it in
   * mid-air on the way past, and over a gap "on the way past" means on the
   * way down. On the arena's basic1 the plan ends with a 5-block drop-jump
   * followed immediately by a 4-block flat jump across a chasm; the take-off
   * was authorised by a box the bot clipped while falling, and it fell 40
   * blocks to its death having "reached" its node.
   *
   * So a PARKOUR node has to be landed on: the rollout carries on from the
   * moment it was satisfied, controls as the executor would hold them, and
   * the bot has to be on the ground near the node within a jump's worth of
   * ticks. Walking nodes keep upstream's box exactly — nothing about a walk
   * is transient.
   */
  private landsThere (
    node: { x: number, y: number, z: number, parkour?: boolean },
    state: PlayerState,
    sprint: boolean
  ): boolean {
    if (node.parkour !== true) return true
    if (this.caught(state)) return true
    const settled = this.simulateUntil(s => this.caught(s), (s: PlayerState) => {
      const dx = node.x - s.pos.x
      const dz = node.z - s.pos.z
      s.yaw = Math.atan2(-dx, -dz)
      s.control.forward = true
      s.control.jump = false
      s.control.sprint = sprint
    }, 12, state)
    return this.caught(settled) &&
      Math.hypot(node.x - settled.pos.x, node.z - settled.pos.z) <= 1 &&
      Math.abs(node.y - settled.pos.y) < 1
  }

  /**
   * Has the body ARRIVED somewhere, by any of the means the planner counts as
   * a landing? Ground, yes — but `moveGen.findExtLanding` also accepts water
   * and climbables, and the extended repertoire emits gap-jumps that catch a
   * pool or a ladder on purpose.
   *
   * Answering that question with `onGround` alone made every one of those
   * moves unauthorisable, and unauthorisable is worse than merely unused: the
   * planner keeps emitting them, every gate refuses, the bot creeps to the
   * lip and stands there, the wedge recovery shoves it backwards, the
   * futility timer replans, and the same plan comes back — a livelock at the
   * take-off, forever, on any route the search wants to send through a pond
   * or a ladder shaft. Neither case can ever set onGround: prismarine-physics
   * derives it from a downward vertical collision, and a ladder clamps
   * vel.y to -0.15 with no collision at all while water deeper than a block
   * outlasts the settle budget. Catching EARLY also keeps the height check
   * honest — a body left to sink six blocks into a pool is nowhere near its
   * node by the time it finally touches the bottom.
   */
  private caught (state: PlayerState): boolean {
    if (state.isInWater === true) return true
    // (ground that carries it: PlayerSim.carried — a touchdown that ends
    // the tick past the edge it came down on is a fall a tick later. Arena
    // tenways-slime-way: a jump "landed" 0.03 past a slime top, 5 down.)
    if (state.onGround === true) return this.carriedAt(state.pos)
    const feet = this.bot.blockAt(
      new Vec3(Math.floor(state.pos.x), Math.floor(state.pos.y), Math.floor(state.pos.z)), false
    ) as { name?: string } | null
    return feet !== null && CAUGHT_FEET.has(feet.name ?? '')
  }

  private readonly carryProbe = newBody(0, 0, 0)

  /** Is a body standing at `pos` carried by a block (PlayerSim.carried)? True without a kernel to ask. */
  private carriedAt (pos: { x: number, y: number, z: number }): boolean {
    const k = this.getKernel()
    if (k === null) return true
    const b = this.carryProbe
    b.x = pos.x
    b.y = pos.y
    b.z = pos.z
    return k.carried(b)
  }

  /**
   * Is the bot pressed against a face it is trying to walk through, with no
   * way forward? The gates above have all said no and the executor is about
   * to stand still — this is the difference between "waiting for a moment" and
   * "wedged", and it is what arms the back-off.
   */
  isGrinding (path: Array<{ x: number, y: number, z: number }>, ticks = 3): boolean {
    const from = this.bot.entity.position
    const before = Math.hypot(path[0].x - from.x, path[0].z - from.z)
    const state = this.simulateUntil(() => false, this.getController(path[0], false, false), ticks)
    const after = Math.hypot(path[0].x - state.pos.x, path[0].z - state.pos.z)
    return state.isCollidedHorizontally === true && before - after <= GRIND_PROGRESS * ticks
  }

  /**
   * Is holding jump while sprinting actually faster HERE, and safe?
   *
   * Sprint-hopping is the fastest way a player crosses open ground — the
   * jump preserves the sprint boost that ground friction eats, and the arena
   * measured it at 6.97 blocks/s against 5.56 sprinting, a 25% gain, with
   * zero server corrections over 40 ticks. It is also the single easiest way
   * to leave the ground somewhere there is nothing to land on, so it is not
   * taken on faith: both gaits are driven down the SAME path for the same
   * horizon and the faster one wins, and only if it did not lose height.
   *
   * Called only while the bot is on the ground (the one tick where the
   * decision can be acted on), so the cost is one pair of short rollouts per
   * take-off, not per tick.
   *
   * With `kernel` the verdict is the exact kernel's (gaitVerdict), and the
   * guards below — which exist because these rollouts are two fixed policies
   * a fixed horizon out — are its search's to answer.
   */
  sprintHopBetter (
    path: Array<{ x: number, y: number, z: number }>,
    lowCeilingHop = false,
    horizon = 16,
    kernel = false
  ): boolean {
    this.hopFlight = null
    const k = kernel ? this.getKernel() : null
    if (k !== null) return this.gaitVerdict(k, path)
    // The whole horizon has to be backed by real path. A hop covers ~5.6
    // blocks before it lands, so with less than that ahead the rollout scores
    // a flight off the end of the plan — and near the goal that is an
    // overshoot of the goal itself. On the arena's basic1 the last few nodes
    // sit on the far side of a chasm; there is nothing to hop toward there.
    const look = Math.min(HOP_LOOK, path.length)
    this.hopRefusal = ''
    if (path.length < HOP_LOOK) { this.hopRefusal = 'short'; return false }

    // Ground that is level or falling away, and only where the planner is
    // walking rather than jumping. A rise is the planner's business — the
    // jump gates own step-ups, and a hop into one lands on the riser's face.
    // Descents are allowed: a hop downhill covers more ground per tick than a
    // sprint and lands lower, which is where it was going anyway.
    const y0 = this.bot.entity.position.y
    let floor = y0
    for (let i = 0; i < look; i++) {
      const n = path[i] as { x: number, y: number, z: number, parkour?: boolean }
      if (n.parkour === true) { this.hopRefusal = 'jump-ahead'; return false }
      if (n.y > y0 + 0.1) { this.hopRefusal = 'rise-ahead'; return false }
      floor = Math.min(floor, n.y)
      // Never along an EDGE PANEL (an open trapdoor's 3/16 ledge, a
      // ladder's top — shapes.ts carryCode 1-4). A hop lands where the arc
      // puts it, a few centimetres off the line either way, and a panel has
      // no centimetres to give: the rollout lands the hop and the body
      // drifts off live (the arena's mcc-2-2 ledge). Walking keeps the sneak
      // guard and the wall-slide, which is what a player does there. Centred
      // narrow supports (posts, heads, fence lines) keep hopping as before.
      const sup = this.bot.blockAt(new Vec3(Math.floor(n.x), Math.floor(n.y - 0.5), Math.floor(n.z)), false) as { shapes?: number[][] } | null
      const carry = sup !== null && sup.shapes !== undefined && sup.shapes.length > 0 ? carryCode(sup.shapes) : 0
      if (carry >= CARRY_W_NONE && carry <= CARRY_S_NONE) {
        this.hopRefusal = 'narrow-ahead'
        return false
      }
    }

    // Nothing to swim into or burn in overhead.
    //
    // A hop lifts the feet 1.25, so the head sweeps up to about three blocks
    // above the node level — airspace the planner never looks at, because
    // walking never goes there. A floating water block in it turns a sprint
    // into a swim and starts the breath clock; lava in it is simply death.
    // The gait must not be the thing that finds them.
    //
    // A solid ceiling up there is NOT a reason to stay down. The bonked arc
    // is shorter, and with the press-on-landing cadence a low roof is the
    // FASTEST ground there is: measured on flat stone, under a 2-block roof,
    // 9.68 blocks/s against 5.59 sprinting and 7.05 hopping under open sky.
    // What jams is the roof DROPPING mid-arc — the body is already up at 1.25
    // when the low section arrives and stops dead against its side instead of
    // bonking cleanly off its underside — so a take-off is only taken when
    // nothing lower than the roof it is already under lies within the arc.
    //
    // Capped at 4, because that is as high as the hop can reach: the peak
    // puts the head 3.05 above the take-off, so any roof at 4 or above does
    // not constrain it at all and a "drop" from 5 to 4 is not a drop. Without
    // the cap, open sky here plus any overhang ahead refuses the gait for the
    // rest of the route.
    const roofHere = Math.min(4, this.ceilingAbove(
      Math.floor(this.bot.entity.position.x), Math.floor(y0 + 0.001), Math.floor(this.bot.entity.position.z)))
    // How far the BODY is above bonking height, and therefore how far ahead a
    // lower roof still matters, is set by the roof this take-off is under: a
    // free arc is airborne ~12 ticks and covers ~4 blocks, a 2-high bonk
    // lands in ~5 and covers under one. Scanning the free-arc distance from
    // under a low roof is what makes a bot refuse to enter a tunnel at all —
    // in a mostly-2-high passage every 3-high pocket has a 2-high section
    // within six nodes, so the gait switches off for the whole passage. The
    // scan is the arc's own footprint, so the bot sprints the last step into
    // a low section and hops the moment it is under it.
    const reach = lowCeilingHop ? (roofHere >= 3 ? 4 : 1) : look
    for (let i = 0; i < look; i++) {
      const n = path[i]
      const nx = Math.floor(n.x)
      const ny = Math.floor(n.y + 0.001)
      const nz = Math.floor(n.z)
      for (let dy = 2; dy <= 3; dy++) {
        const b = this.bot.blockAt(new Vec3(nx, ny + dy, nz), false) as { name?: string } | null
        if (b === null) { this.hopRefusal = 'unloaded'; return false } // unloaded: assume the worst
        if (OVERHEAD_HAZARDS.has(b.name ?? '')) { this.hopRefusal = 'hazard'; return false }
      }
      if (i < reach && this.ceilingAbove(nx, ny, nz) < roofHere) { this.hopRefusal = 'roof-drop'; return false }
    }

    const run = this.followPath(path, false, horizon)
    const hop = this.followPath(path, true, horizon)
    if (hop === null || run === null) { this.hopRefusal = 'lava'; return false }
    // Absolute, and measured against the PATH's own floor rather than the two
    // gaits' relative heights. Comparing the gaits was not enough: on the
    // arena's climb1 tower both of them fell off a 1-wide pillar, so the hop
    // was "no worse", and the bot flew off the side of a 46-block climb it
    // had been walking correctly (91.8 blocks travelled, y 141 down to 102,
    // 15 damage). A hop may follow the path down; it may not leave it.
    if (hop.minY < floor - HOP_MAX_DIP) { this.hopRefusal = 'dip'; return false }
    if (!hop.endedOnGround) { this.hopRefusal = 'airborne-end'; return false }
    if (!(hop.score > run.score + HOP_MARGIN)) { this.hopRefusal = 'no-gain'; return false }
    return true
  }

  // ── gait (gaitSearch.ts) ──────────────────────────────────────────────
  //
  // Hop now, or keep the feet down? Asked of the kernel, over the schedules
  // of hops the body could fly from here (gaitSearch.ts), to a goal that is
  // the ground's own: the landing of the first jump ahead — the rollouts'
  // guards refuse the gait whenever a jump or a rise is within their
  // look-ahead, which left every approach on a course sprinted, 5.6 b/s
  // against 7.07 hopping — or a node further on, or the path's end.
  /** Why the last gait search decided as it did (trace diagnostics). */
  hopJumpNote = ''

  /** The hop the last gait search chose, as it flew it: a yaw a tick, take-off to touchdown (GaitResult.flight); null when it chose none. */
  hopFlight: number[] | null = null

  /** The node the executor retired last, and the server's fall rule (GaitGoal.from, .wholeFall): the executor's to set. */
  gaitFrom: { x: number, z: number } | null = null
  wholeFall = true
  /** The gait with the 45° strafe (Movements.allowStrafe): the executor's to set; and whether the hop chosen flies with it. */
  gaitStrafe = false
  hopStrafe = false

  /** Is a schedule that hops NOW the soonest to the goal (gaitSearch)? */
  private gaitHop (k: PlayerSim, path: ReadonlyArray<GaitNode>, goal: GaitGoal): boolean {
    let floor = this.bot.entity.position.y
    for (let i = 0; i < Math.min(path.length, goal.jump ? goal.index : goal.index + 1); i++) floor = Math.min(floor, path[i].y)
    goal.from = this.gaitFrom
    goal.wholeFall = this.wholeFall
    goal.strafe = this.gaitStrafe
    const r = gaitSearch(k, this.kBodyNow(), path, goal, floor)
    const take = r.hop !== Infinity && r.run === Infinity && !r.spent
    this.hopFlight = take ? r.flight : null
    this.hopStrafe = take && r.strafe
    const f = (v: number): string => v === Infinity ? '-' : Number.isInteger(v) ? String(v) : v.toFixed(2)
    this.hopJumpNote = `${take ? 'hop' : 'run'} h${f(r.hop)} r${f(r.run)} w${r.work}${r.spent ? ' spent' : ''}`
    return take
  }

  /**
   * The gait's verdict from the kernel, for a body on the ground: is a
   * schedule that hops NOW the soonest to the goal? The goal is the landing
   * of the first jump ahead (jumpAhead); over ground that takes none, a node
   * GAIT_LOOK on, or the path's last where it ends sooner. What is flown is
   * exact — a roof that drops mid-arc, a landing short of the plan's end, a
   * pool overhead are the search's to find — so the guards left are the two
   * the kernel cannot answer: a world not loaded yet, and the edge panels
   * (an open trapdoor's 3/16 ledge, a ladder's top: shapes.ts carryCode 1-4)
   * a landing has no centimetres on.
   */
  private gaitVerdict (k: PlayerSim, path: ReadonlyArray<GaitNode>): boolean {
    this.hopRefusal = ''
    this.hopJumpNote = ''
    if (path.length === 0 || (this.bot.entity as { onGround?: boolean }).onGround !== true) { this.hopRefusal = 'airborne'; return false }
    const jump = this.jumpAhead(k, path)
    const end = jump >= 0 ? jump : Math.min(path.length, GAIT_LOOK) - 1
    for (let i = 0; i <= end; i++) {
      const n = path[i]
      const nx = Math.floor(n.x)
      const ny = Math.floor(n.y + 0.001)
      const nz = Math.floor(n.z)
      for (let dy = 2; dy <= 3; dy++) {
        if (this.bot.blockAt(new Vec3(nx, ny + dy, nz), false) === null) { this.hopRefusal = 'unloaded'; return false }
      }
      if (i === jump) break
      const sup = this.bot.blockAt(new Vec3(nx, Math.floor(n.y - 0.5), nz), false) as { shapes?: number[][] } | null
      const carry = sup !== null && sup.shapes !== undefined && sup.shapes.length > 0 ? carryCode(sup.shapes) : 0
      if (carry >= CARRY_W_NONE && carry <= CARRY_S_NONE) { this.hopRefusal = 'narrow-ahead'; return false }
    }
    const goal: GaitGoal = jump >= 0
      ? { index: jump, jump: true, last: false }
      : { index: end, jump: false, last: end === path.length - 1 }
    if (this.gaitHop(k, path, goal)) return true
    this.hopRefusal = jump >= 0 ? 'jump-ahead' : 'no-gain'
    return false
  }

  /**
   * The first node within HOP_JUMP_LOOK that takes a jump: a parkour node
   * that is one (not ground the plan jumps: walkedJump), or a rise too high
   * to step. -1 when there is none.
   */
  private jumpAhead (k: PlayerSim, path: ReadonlyArray<GaitNode>): number {
    let y = this.bot.entity.position.y
    for (let i = 0; i < Math.min(path.length, HOP_JUMP_LOOK); i++) {
      const n = path[i]
      if (n.y > y + SIM.stepHeight) return i
      if (n.parkour === true && !this.walkedJump(k, path, i)) return i
      y = n.y
    }
    return -1
  }

  /** Parkour nodes the kernel has walked to from the node before (kWalkable), by node. */
  private readonly walked = new WeakMap<object, boolean>()

  /**
   * Is the parkour node path[i] ground the plan happens to jump? The planner
   * prices a sprint jump over flat ground below the steps it replaces, so a
   * plan across a floor is strewn with them; the executor runs those (its
   * walking gate comes first), and a gait that took each for a take-off ran
   * every approach to one on its feet.
   */
  private walkedJump (k: PlayerSim, path: ReadonlyArray<GaitNode>, i: number): boolean {
    const n = path[i]
    if (i === 0) return kWalkable(k, this.bot.entity.position, n)
    const hit = this.walked.get(n)
    if (hit !== undefined) return hit
    const yes = kWalkable(k, path[i - 1], n)
    this.walked.set(n, yes)
    return yes
  }

  // ── control programs over the live world (HopOracle) ───────────────────
  //
  // The executor's take-off search: where no gate will fly a parkour node (a
  // neo, a wedged take-off), the exact kernel searches control programs from
  // the body as it stands — the same search the planner's hop pipeline runs,
  // over the live world, read through the kernel's window.

  /** Why the last hopFrom / hopLinedUp came out as it did (trace diagnostics). */
  hopNote = ''

  /**
   * A HopOracle over the live world, read through the kernel's window
   * (kGrid): one consistent world for the whole search — nothing changes
   * inside a tick — and no copy to build first. Null without shape tables.
   */
  private liveOracle (): HopOracle | null {
    const lut = this.kTablesLut
    const k = this.getKernel()
    if (this.kTables === null || lut === null || k === null) return null
    const simWorld: SimWorld = { ...this.kTables, stateAt: (x, y, z) => this.kStateAt(x, y, z), grid: this.kGrid }
    return new HopOracle(simWorld, (x, y, z) => { const st = k.stateAt(x, y, z); return st < 0 ? 0 : lut.flags[st] },
      k.halfWidth, k.height, k.speedBase)
  }

  /**
   * The planner node a body at `p` stands on: the cell under its centre, or
   * — at a lip, where the centre overhangs the gap by design — the nearest
   * cell its box rests on that has a stand. (Arena mcc-4-3: the centre's cell
   * was the gap, the lined-up search had no stand to start from, and a jump
   * the kernel flies from the block's centre was given up at its lip.)
   */
  private standCell (o: HopOracle, p: { x: number, y: number, z: number }): [number, number, number] {
    const cx = Math.floor(p.x)
    const cz = Math.floor(p.z)
    const centre: [number, number, number] = [cx, o.nodeY(cx, p.y, cz), cz]
    if (o.stands(centre[0], centre[1], centre[2]).some(s => !s.catch)) return centre
    const hw = (this.bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3
    let best = centre
    let bestD = Infinity
    for (let x = Math.floor(p.x - hw); x <= Math.floor(p.x + hw); x++) {
      for (let z = Math.floor(p.z - hw); z <= Math.floor(p.z + hw); z++) {
        if (x === cx && z === cz) continue
        const y = o.nodeY(x, p.y, z)
        const d = Math.hypot(x + 0.5 - p.x, z + 0.5 - p.z)
        if (d < bestD && o.stands(x, y, z).some(s => !s.catch)) {
          best = [x, y, z]
          bestD = d
        }
      }
    }
    return best
  }

  /**
   * A LIVE program to planner node `cell` from the body exactly as it is
   * now (HopOracle.hopFrom: every family, walking jumps too, the quickest
   * robust one), or null. Grounded bodies only.
   */
  hopFrom (cell: readonly [number, number, number], budgetMs: number): HopProgram | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    const found = o.hopFrom(this.kBodyNow(), [cell[0], cell[1], cell[2]], t0 + budgetMs, this.bot.entity.yaw)
    this.hopNote = `from ${found !== null ? `${found.family} ${found.ticks}t` : 'none'} ${(performance.now() - t0).toFixed(0)}ms`
    return found
  }

  /**
   * Planned program `prog` to planner node `cell`, re-anchored at the body as
   * it is (HopOracle.anchored): live, if it still lands robustly from here.
   */
  hopAnchored (cell: readonly [number, number, number], prog: HopProgram): HopProgram | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    const found = o.anchored(prog, this.kBodyNow(), this.bot.entity.yaw)
    this.hopNote = `anchored ${found !== null ? `${found.ticks}t` : 'no'} ${(performance.now() - t0).toFixed(0)}ms`
    return found
  }

  /**
   * A program to planner node `cell` from rest at a stand point of the
   * node the body is on (HopOracle.hop, every family) — the executor lines
   * the body up on it first — or null.
   */
  hopLinedUp (cell: readonly [number, number, number], budgetMs: number): HopProgram | null {
    const t0 = performance.now()
    const p = this.bot.entity.position
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    // simplest first: the simple families cost a fifth of the yaw family,
    // which put first spent the whole budget on 6-2's third neo and never
    // let the air-turn that flies it (18 ms) be tried
    const from = this.standCell(o, p)
    const to: [number, number, number] = [cell[0], cell[1], cell[2]]
    let found = o.hop(from, to, t0 + budgetMs, 'simple', true)
    let ran = !o.noVerdict
    if (found === null) { found = o.hop(from, to, t0 + budgetMs, 'yaw', true); ran = ran && !o.noVerdict }
    // every family run to the end and nothing lands: the executor's verdict
    // on a table jump (the beam, for what no family flies, may run out of time)
    this.hopExhausted = found === null && ran
    if (found === null) found = o.hop(from, to, t0 + budgetMs, 'beam')
    this.hopNote = `lined ${found !== null ? `${found.family} ${found.ticks}t` : this.hopExhausted ? 'none-exhausted' : 'none'} ${(performance.now() - t0).toFixed(0)}ms`
    return found
  }

  /** Did the last hopLinedUp fly every program of every family from the take-off cell and land none? */
  hopExhausted = false

  /**
   * Blocks of headroom over a standing body at (x, y, z): the offset of the
   * first non-empty cell at or above the head, capped. Cells 0 and 1 are the
   * body itself and are clear by construction on any node worth walking to.
   * An unloaded read is reported as low, not high — the hop guard reads this
   * to decide whether the roof drops ahead, and guessing high is the answer
   * that jams.
   */
  private ceilingAbove (x: number, y: number, z: number, cap = 4): number {
    for (let dy = 2; dy <= cap; dy++) {
      const b = this.bot.blockAt(new Vec3(x, y + dy, z), false) as { boundingBox?: string } | null
      if (b === null || b.boundingBox !== 'empty') return dy
    }
    return cap + 1
  }

  /**
   * Drive `path` for `horizon` ticks with one gait and score how far along it
   * got. Nodes are consumed the way the executor consumes them, with the
   * apex tolerance a hop needs (see HOP_ARRIVE_DY in plugin.ts).
   *
   * The hop is driven with the executor's cadence — jump PRESSED on grounded
   * ticks only, never held — because the two are not the same gait under a
   * low ceiling: a held key cannot re-fire for 10 ticks and a bonked arc
   * lands in 5. Scoring a held hop and then running a pressed one would price
   * the wrong thing by half.
   */
  private followPath (
    path: Array<{ x: number, y: number, z: number }>,
    jump: boolean,
    horizon: number
  ): { score: number, minY: number, endedOnGround: boolean } | null {
    const state = this.newState({
      forward: true, back: false, left: false, right: false, jump, sprint: true, sneak: false
    })
    const simulatePlayer = (this.bot.physics as unknown as { simulatePlayer: (s: PlayerState, w: unknown) => void }).simulatePlayer
    let i = 0
    let minY = state.pos.y
    for (let t = 0; t < horizon; t++) {
      const target = path[Math.min(i, path.length - 1)]
      state.yaw = Math.atan2(-(target.x - state.pos.x), -(target.z - state.pos.z))
      state.control.forward = true
      state.control.jump = jump && state.onGround === true
      state.control.sprint = true
      simulatePlayer.call(this.bot.physics, state, this.world)
      if (state.isInLava) return null
      minY = Math.min(minY, state.pos.y)
      while (i < path.length) {
        const n = path[i]
        if (Math.abs(n.x - state.pos.x) <= 0.35 && Math.abs(n.z - state.pos.z) <= 0.35 &&
            Math.abs(n.y - state.pos.y) < 1.5) i++
        else break
      }
      if (i >= path.length) break
    }
    const ahead = path[Math.min(i, path.length - 1)]
    const left = Math.hypot(ahead.x - state.pos.x, ahead.z - state.pos.z)
    // "Ended on the ground" has to tolerate the hop's own airtime, so it is
    // answered by letting the rollout fall for a few more ticks with the
    // controls released: a bot over solid ground lands, a bot over a hole
    // keeps going.
    const settle = this.simulateUntil(s => s.onGround === true, (s: PlayerState) => {
      s.control.forward = false
      s.control.jump = false
      s.control.sprint = false
    }, 8, state)
    return {
      score: i - Math.min(left, 4) / 4,
      minY: Math.min(minY, settle.pos.y),
      endedOnGround: settle.onGround === true
    }
  }

  /**
   * Find a take-off HEADING that lands the jump, instead of always aiming
   * dead at the node.
   *
   * A player lining up an awkward hop does not stare at the block they want —
   * they turn a few degrees to clear the corner in the way, and the jump that
   * was impossible head-on goes first try. (Leg0shii's ParkourCalculatorMod
   * is a whole TAS planner built around this: its "angle solver" searches yaw
   * inputs for the ones that land a given jump. This is the same idea at one
   * hundredth the scope — a handful of offsets, tried only when the straight
   * line has already failed.)
   *
   * Offsets are ordered by size, so a heading that works is only ever
   * traded for a straighter one, never the reverse, and 0 is tried first:
   * where aiming at the node works, nothing changes. Returns the offset in
   * radians, or null if no angle lands it.
   */
  bestHeading (
    path: Array<{ x: number, y: number, z: number }>,
    jump: boolean,
    sprint: boolean
  ): number | null {
    const reached = this.getReached(path)
    for (const offset of HEADING_OFFSETS) {
      const state = this.simulateUntil(
        reached, this.getController(path[0], jump, sprint, 0, offset), 45
      )
      if (reached(state)) return offset
    }
    return null
  }

  /**
   * The walking counterpart of `bestHeading`: a heading a few degrees off
   * the node that WALKS to it where the straight line grinds on a corner.
   *
   * A body a hair off its line clips a block beside the step — 0.03 of
   * overlap is enough — the per-axis collision stops it dead on that axis,
   * and the straight-line gate refuses. Left there, the cascade hands a flat
   * one-block walk to the JUMP gates: a 12-tick arc for a 5-tick step that
   * then overshoots the node and walks back for it (arena basic3, 22 ticks
   * for one block). A player just angles round the corner. Offsets are
   * tried smallest first; the first that reaches wins. Short horizon: a walk
   * step reaches in a few ticks or not at all, and the grind guard ends a
   * hopeless one early.
   */
  bestWalkHeading (path: Array<{ x: number, y: number, z: number }>, sprint: boolean): number | null {
    const reached = this.getReached(path)
    for (const offset of HEADING_OFFSETS) {
      if (offset === 0) continue
      const state = this.simulateUntil(
        reached, this.getController(path[0], false, sprint, 0, offset), 40, null,
        this.grindGuard(path[0], this.bot.entity.position)
      )
      if (reached(state)) return offset
    }
    return null
  }

  /**
   * Would holding `control` for a few ticks actually get the body somewhere,
   * and leave it standing where it can carry on? This is the question the
   * wedge recovery asks of each escape it is considering — a step back, a
   * step sideways — and it has to be asked of the physics rather than the
   * geometry, because "there is air behind me" and "I can stand there" are
   * different claims. A nudge that does not move is no escape, and a nudge
   * that loses height is a fall.
   */
  canNudge (control: Partial<SimControl>, ticks = 5): boolean {
    const p0 = this.bot.entity.position.clone()
    const state = this.simulateUntil(() => false, (s: PlayerState) => {
      s.control.forward = control.forward === true
      s.control.back = control.back === true
      s.control.left = control.left === true
      s.control.right = control.right === true
      s.control.jump = control.jump === true
      s.control.sprint = false
      s.control.sneak = control.sneak === true
    }, ticks)
    const moved = Math.hypot(state.pos.x - p0.x, state.pos.z - p0.z)
    return state.onGround === true && state.pos.y >= p0.y - 0.1 && moved >= 0.15
  }

  /** Can the bot step BACK without walking off what it is standing on? */
  canBackOff (ticks = 5): boolean {
    return this.canNudge({ back: true }, ticks)
  }

  private getReached (path: Array<{ x: number, y: number, z: number, parkour?: boolean }>): (state: PlayerState) => boolean {
    // Upstream's box is |dy| < 1 for everything. For a WALKING node that
    // lets a rollout "reach" it from on top of the block beside it — on the
    // arena's climb2 a same-level diagonal was answered with a sprint-jump
    // ONTO the adjacent +1 block, followed by walking off it: 18 ticks and
    // 3.5 blocks for a 1.4-block step. A walk node is arrived at on its own
    // level (postProcessPath puts it on the support's top); a parkour node
    // keeps the full box, `landsThere` settles it.
    const dyTol = path[0].parkour === true ? 1 : WALK_REACH_DY
    return (state: PlayerState) => {
      const delta = {
        x: path[0].x - state.pos.x,
        y: path[0].y - state.pos.y,
        z: path[0].z - state.pos.z
      }
      return Math.abs(delta.x) <= 0.35 && Math.abs(delta.z) <= 0.35 && Math.abs(delta.y) < dyTol
    }
  }

  /**
   * ONE jump per rollout, then walk it in.
   *
   * Upstream holds jump for the whole rollout, which means a jump that lands
   * short does not fail the test — the sim simply jumps again from wherever it
   * came down and satisfies the node many ticks later, from a cell the planner
   * never routed through. The executor then commits to a take-off whose real
   * landing is somewhere else entirely: on 2b2t spawn the residual cross-axis
   * velocity of a 4-block drop-jump carried the next take-off two cells short
   * into a 1-block pit, and the rollout still said yes because it reached the
   * node 9 ticks after landing in that pit — where the bot then sat for the
   * rest of the run. Upstream's 20-tick budget hid this by accident; ours is
   * 45 so deep drops fit, so the rule has to be explicit.
   *
   * Releasing jump on touchdown is also just what the executor does: it
   * re-decides every tick, and after a landing nothing is holding jump unless
   * a fresh decision asks for one. So the rollout now answers the question
   * actually being asked — does ONE jump from here, plus the run-in, reach
   * the node? — and a landing short of the node fails it unless the bot can
   * walk the rest, which is a real landing rather than a bounce.
   */
  private getController (nextPoint: { x: number, y: number, z: number }, jump: boolean, sprint: boolean, jumpAfter = 0, headingOffset = 0): Controller {
    let fired = false
    let landed = false
    let pressed = false
    const cooldown = (this.bot.physics as unknown as { autojumpCooldown?: number }).autojumpCooldown ?? 10
    return (state: PlayerState, tick: number) => {
      const dx = nextPoint.x - state.pos.x
      const dz = nextPoint.z - state.pos.z
      state.yaw = Math.atan2(-dx, -dz) + headingOffset

      // Latch on the state left by the previous SIMULATED tick. `fired`
      // latches on the sim ACTUALLY leaving the ground upward rather than on
      // the seeded onGround flag, and that distinction is the whole point:
      // mineflayer sets `entity.onGround = false` on every server position
      // correction (lib/plugins/physics.js), so a bot standing on solid
      // ground reads as airborne for a tick. The old latch took that at face
      // value, called the very first simulated tick a LANDING, and released
      // the jump it was being asked about — so every jump gate answered "no"
      // for as long as the corrections kept coming, which is exactly when the
      // bot most needs one. On the arena's staircases that was hundreds of
      // corrections a run, each one blinding the tick after it.
      //
      // And on the jump having FIRED, whatever became of it (the cooldown it
      // arms says so): under a low ceiling the head bonks within the tick and
      // no upward velocity is left to see, so the latch never set, the
      // rollout held jump through the landing and jumped again from further
      // on — and approved a take-off whose one jump goes nowhere. On the
      // arena's climb2, whose stairs run under the flight above, every other
      // step took a bonk, a fall back and a second jump: 12-14 ticks for 7.
      if (!fired) {
        if (state.vel.y > 0 || (pressed && (state as unknown as { jumpTicks?: number }).jumpTicks === cooldown)) fired = true
      } else if (state.onGround === true) {
        landed = true
      }

      // The whole control vector, not just the three this cares about:
      // PlayerState seeds `control` from bot.controlState, so a rollout taken
      // during a back-off or a sneak-creep would otherwise simulate those
      // held down and answer a question nobody asked.
      state.control.forward = true
      state.control.back = false
      state.control.left = false
      state.control.right = false
      state.control.sneak = false
      state.control.jump = jump && tick >= jumpAfter && !landed
      state.control.sprint = sprint
      pressed = state.control.jump
    }
  }
}
