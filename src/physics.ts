// Port of mineflayer-pathfinder/lib/physics.js (MIT): the executor's
// sprint/jump decisions come from short prismarine-physics simulations
// against the LIVE world — same engine the bot moves with, so path following
// inherits none of the solver's approximations.
import { PlayerState } from 'prismarine-physics'
import type { SimControl } from 'prismarine-physics'
import type { Bot } from 'mineflayer'
import { Vec3 } from 'vec3'
import { carryCode, CARRY_W_NONE, CARRY_S_NONE } from './shapes.js'
import { PlayerSim, SIM, newBody, simTablesFromLut, setVanillaJump } from './playerSim.js'
import type { SimBody, SimGrid, SimWorld } from './playerSim.js'
import { gaitSearch, kWalkable, GRAZE_YAW, HOP_MAX_DIP, EASE_NEVER, GAIT_CATCH, HURT_TICKS } from './gaitSearch.js'
import type { GaitNode, GaitPick } from './gaitSearch.js'
import type { GaitDrive } from './types.js'
import type { BlockLut } from './lut.js'
import { HopOracle, ONE_FLIGHT_REACH, LANE_KICK_TICKS, LIVE_PREFIX, LIVE_PREFIX_TICKS } from './hopOracle.js'
import type { HopProgram } from './hopOracle.js'
import { FALL_VETO } from './fallDamage.js'
import { isVineClimbable, physicsClimbsVines } from './movements.js'

/** prismarine-physics' sprinting modifier id (index.js physics.sprintingUUID). */
const SPRINTING_UUID = '662a6b8d-da3e-4c1c-8813-96ea6097278d'

type Controller = (state: PlayerState, tick: number) => void

/** Ticks holdsOn coasts a body to rest: 0.546^20 of a speed is left on ordinary ground, a tenth of it on ice. */
const HOLD_COAST_TICKS = 20
type Refusal = (state: PlayerState) => boolean

/** Minimum horizontal progress per tick before wall contact counts as grinding. */
const GRIND_PROGRESS = 0.01

/** Extra path progress a hop needs to justify take-off without flickering between gaits. */
const HOP_MARGIN = 0.5


/** Heading offsets in radians, smallest first; wider turns leave the planned route. */
const HEADING_OFFSETS = [0, 0.13, -0.13, 0.26, -0.26, 0.39, -0.39, 0.52, -0.52]

/** A hop raises the head into airspace walking never checks, where liquids can interrupt the flight. */
const OVERHEAD_HAZARDS = new Set([
  'water', 'flowing_water', 'bubble_column', 'lava', 'flowing_lava'
])

/** In-air controls, strongest forward first; releasing or reversing avoids overshooting narrow supports. */
export interface AirControl { forward: boolean, back: boolean, sprint: boolean }
const AIR_OPTS: readonly AirControl[] = [
  { forward: true, back: false, sprint: true },
  { forward: true, back: false, sprint: false },
  { forward: false, back: false, sprint: false },
  { forward: false, back: true, sprint: false }
]
/** Air ticks of held forward before the tail option, tried by the take-off gate. */
const AIR_RELEASE_TICKS = [8, 6, 4, 2, 0]
/** Enables controlled take-off checks. */
const AIR_GATE = process.env.PF_AIR_GATE !== '0'
const AIR_MISS_TOL = 0.05
/** A predicted touchdown within this of the node's aim is left alone by the in-flight controller. */
const AIR_CENTRE_TOL = 0.1

/**
 * Controls for one simulated jump, replayed tick for tick. Absolute yaw changes
 * from `yawA` to `yawB` at `turnTick` air ticks; `track` gives each predicted position.
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
// Jumping around a pillar can require running along its face to the last
// supported tick, then turning back toward the landing in mid-air.
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

/** Height tolerance for reaching a walking node in a rollout. */
const WALK_REACH_DY = 0.5
/** Maximum straight-line dip below the lower endpoint, limited to a recoverable step. */
const STRAIGHT_MAX_DIP = 0.6

/** Clear path nodes required ahead of a sprint-hop, without rises or jumps. */
const HOP_LOOK = Math.max(2, Number(process.env.PF_HOP_LOOK ?? 6) || 6)

/** Maximum path distance in blocks, and node count, searched by the gait. */
const GAIT_REACH = 10
const GAIT_NODES = 12
/** Retry delay in ticks after the kernel finds no flight from a take-off. */
const GAIT_REST = 3
/** Height above a catch node included in its climb window; higher catches shorten the climb. */
const CATCH_CLIMB = 2


/**
 * Budget live searches by kernel work so host load does not change the verdict.
 * Wall time is only a safety cap, using a conservative kernel tick rate.
 */
const LIVE_FLOOR_TICKS_PER_MS = 300
/** Separate kernel-work budget for the beam at a stuck take-off. */
const LINED_BEAM_TICKS = process.env.PF_LINED_GIVEUP === '0' ? 0 : 150_000
/** Separate kernel-work budget for the planned kick or pivot family in a live search. */
const SEED_FAMILY_TICKS = 150_000
/** Enables seed families in live and skip-ahead searches. */
export const SEED_FAMILY = process.env.PF_SEED_FAMILY !== '0'
/** Seed families for kick or pivot plans, with the planned family first; undefined for other plans. */
export function seedFamilies (p: HopProgram): Array<'kick' | 'pivot'> | undefined {
  if (p.seq !== undefined) return undefined
  if (p.family === 'kick') return ['kick', 'pivot']
  if (p.family === 'pivot') return ['pivot', 'kick']
  return undefined
}
/** Enables program-tick pricing for the planned skip-ahead leg. */
const SKIP_PLAN_LEG = process.env.PF_SKIP_PLAN_LEG !== '0'
/**
 * Skip-ahead limits and minimum savings over the planned leg.
 * The leg floor allows free flight reach before charging for horizontal distance.
 */
const SKIP_NODES = 2
const SKIP_RISE = 1.25
const SKIP_DROP = 6
const SKIP_TICKS = 100_000
const SKIP_MAX_TICKS = 30
const SKIP_MARGIN = 1
const SKIP_REACH_FREE = 1.8
const SKIP_SPEED_MAX = 0.5
/** Beam work budgets for skipping a jump around an obstacle, on arrival and from rest. */
const SKIP_NEO_TICKS = 500_000
const SKIP_NEO_REST_TICKS = 800_000
/** Skip walk nodes only when their detour leaves room to improve on the gait's own hops. */
const SKIP_WALK_NODES = 3
const SKIP_WALK_DETOUR = 0.5
const f2 = (v: number): string => Number.isInteger(v) ? String(v) : v.toFixed(2)

/**
 * Lower bound on flight ticks from `from` to `to`, accounting for roofs and vertical travel.
 * Skip-ahead must beat the planned leg even at this optimistic cost.
 */
export function legFloor (k: PlayerSim, from: { x: number, y: number, z: number }, to: { x: number, y: number, z: number }): number {
  const d = Math.hypot(to.x - from.x, to.z - from.z)
  const horizontal = Math.max(0, d - SKIP_REACH_FREE) / SKIP_SPEED_MAX
  const dy = to.y - from.y
  // Sample roofs along the flight too: a low roof over the gap can cut the
  // arc even when the take-off has open sky.
  const c = { forward: false, back: false, left: false, right: false, jump: true, sprint: false, sneak: false, hx: 0, hz: -1 }
  // A riser intersecting the body at take-off height is not a roof.
  const hw = k.halfWidth
  const fits = (x: number, z: number): boolean => {
    const n = k.boxesIn(x - hw, from.y, z - hw, x + hw, from.y + k.height, z + hw)
    const buf = k.scratchBoxes
    for (let j = 0; j < n; j++) {
      const o = j * 6
      if (buf[o] < x + hw - 1e-7 && buf[o + 3] > x - hw + 1e-7 && buf[o + 2] < z + hw - 1e-7 && buf[o + 5] > z - hw + 1e-7 &&
          buf[o + 1] < from.y + k.height - 1e-7 && buf[o + 4] > from.y + 1e-7) return false
    }
    return true
  }
  let apex = Infinity
  let up = 0
  const steps = Math.max(1, Math.ceil(d / 0.5))
  for (let s = 0; s <= steps; s++) {
    const f = s / steps
    const bx = from.x + (to.x - from.x) * f
    const bz = from.z + (to.z - from.z) * f
    if (s > 0 && !fits(bx, bz)) continue
    const b = newBody(bx, from.y, bz)
    b.onGround = true
    c.jump = true
    let top = from.y
    let at = 0
    for (let t = 1; t <= 20; t++) {
      k.step(b, c)
      c.jump = false
      if (b.inLiquid) break
      if (b.y > top + 1e-9) { top = b.y; at = t }
      if (b.vy <= 0) break
    }
    if (top < apex - 1e-9 || (Math.abs(top - apex) <= 1e-9 && at < up)) { apex = top; up = at }
  }
  // A target above the apex is unreachable.
  const fallTicks = (drop: number): number => {
    if (drop < 0) return Infinity
    let y = 0
    let vy = 0
    for (let t = 1; t <= 80; t++) {
      vy = (vy - SIM.gravity) * SIM.airdrag
      y += vy
      if (y <= -drop) return t
    }
    return Infinity
  }
  const jump = up + fallTicks(apex - to.y)
  // A lower target can also be reached by walking off, without rising first.
  const fall = dy < -1e-9 ? fallTicks(-dy) : Infinity
  const vertical = Math.min(jump, fall)
  if (!Number.isFinite(vertical)) return Infinity
  return Math.ceil(Math.max(vertical, horizontal))
}

const NO_DRIVE: GaitDrive = { kind: 'none', flight: null, strafe: false, ease: EASE_NEVER, yaw: 0, sprint: false }

export class PhysicsSim {
  private readonly bot: Bot
  private readonly world: { getBlock: (pos: Vec3) => unknown }
  /** Set by the last simulateUntil that hit its refusal predicate. */
  private refused = false
  /**
   * The world, effects and equipment stay fixed within a tick, so rollouts share
   * cached blocks and a state template. Without beginTick, each lookup is fresh.
   */
  private tickSerial = 0
  private cacheX0 = 0
  private cacheY0 = 0
  private cacheZ0 = 0
  private readonly blockCache = new Map<number, unknown>()
  private seed: PlayerState | null = null
  /** Refusal reason from the last sprintHopBetter call; empty on success. */
  hopRefusal = ''

  constructor (bot: Bot) {
    this.bot = bot
    this.world = { getBlock: (pos: Vec3) => this.blockAt(pos) }
  }

  // Build the allocation-free kernel lazily from shape tables.
  // Callers fall back to prismarine-physics until tables are available.
  private kernel: PlayerSim | null = null
  private kTables: Omit<SimWorld, 'stateAt'> | null = null
  private kTablesLut: BlockLut | null = null
  /** Lazy live-world grid; each cell is read once per tick. */
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

  /** Set per-state shape, slipperiness and liquid tables; repeated calls with the same LUT are cheap. */
  setShapeTables (lut: BlockLut): void {
    if (this.kTablesLut === lut) return
    this.kTablesLut = lut
    this.kTables = simTablesFromLut(lut)
    this.kernel = null
  }

  /** Live-world state id at a cell, or -1 if unloaded. */
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

  /** Invalidate the live-world grid and centre it on the body. */
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
   * Read server movement speed with the sprint modifier removed.
   * The kernel applies sprinting separately, as prismarine-physics does.
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

  /** Movement-speed attribute without sprinting, for simulations outside the executor. */
  walkSpeed (): number {
    return this.kSpeedBase()
  }

  /** The kernel, current with the bot's hitbox and speed; null without shape tables. */
  private getKernel (): PlayerSim | null {
    if (this.kTables === null) return null
    const ph = this.bot.physics as unknown as { playerHalfWidth?: number, playerHeight?: number }
    if (this.kernel === null) {
      this.kernel = new PlayerSim({ ...this.kTables, stateAt: (x, y, z) => this.kStateAt(x, y, z), grid: this.kGrid })
      this.probeVanillaJump()
    }
    // Callers outside a tick cannot reuse cached world reads.
    if (this.tickSerial === 0) this.kGridRefresh()
    this.kernel.halfWidth = ph.playerHalfWidth ?? 0.3
    this.kernel.height = ph.playerHeight ?? 1.8
    this.kernel.speedBase = this.kSpeedBase()
    return this.kernel
  }

  /** Current position, velocity and jump state as a kernel body. */
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

  private vanillaProbed: boolean | null = null

  /**
   * Probe whether the bot preserves a faster upward velocity when jumping, and set
   * the kernel to the same rule. Cache the result once the bot is ready.
   */
  probeVanillaJump (): boolean {
    if (this.vanillaProbed !== null) return this.vanillaProbed
    let on: boolean
    try {
      const st = new PlayerState(this.bot, { forward: false, back: false, left: false, right: false, jump: true, sprint: false, sneak: false })
      st.pos = new Vec3(0.5, 1000.5, 0.5)
      st.vel = new Vec3(0, 1, 0)
      st.onGround = true
      // Effects and movement media can mask the jump rule:
      // jump boost can mimic preserved velocity, while cobwebs can hide it.
      Object.assign(st, { jumpTicks: 0, jumpBoost: 0, levitation: 0, slowFalling: 0, dolphinsGrace: 0, isInWeb: false, isInWater: false, isInLava: false, elytraFlying: false, fireworkRocketDuration: 0 })
      const simulatePlayer = (this.bot.physics as unknown as { simulatePlayer: (s: PlayerState, w: unknown) => void }).simulatePlayer
      simulatePlayer.call(this.bot.physics, st, { getBlock: () => null })
      on = st.vel.y > 0.6
    } catch {
      return false // (not memoized: asked again once the bot is ready)
    }
    this.vanillaProbed = on
    setVanillaJump(on)
    return on
  }

  /**
   * Reuse tick-invariant effects, attributes and enchantments; always refresh
   * position, velocity, look and controls for each rollout.
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
   * Detect wall contact without progress for walking early-outs and stuck recovery.
   * Contact alone does not invalidate a jump.
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
    // Stop a walk that grinds against a face without progress. Limit dips so a
    // rollout cannot approve walking off a ledge and circling back from below.
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

  // Deep drops need enough rollout time for both the flight and the run-in.
  canSprintJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    if (!this.takeoffReady(path[0])) return false
    const reached = this.getReached(path)
    if (this.jumpLands(path, reached, true, jumpAfter, 0)) {
      // Wall-grazing flights must also land with small heading errors on either side.
      if (!this.flightGrazed || (path[0] as { parkour?: boolean }).parkour !== true) return true
      return this.jumpLands(path, reached, true, jumpAfter, GRAZE_YAW) && this.jumpLands(path, reached, true, jumpAfter, -GRAZE_YAW)
    }
    return this.canJumpControlled(path[0], true, jumpAfter)
  }

  /**
   * Check that one jump reaches the node and lands there. Wall contact sets
   * `flightGrazed`, since a marginal landing can fail with tiny alignment errors.
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
   * Predict touchdown with the supplied controller. `err` is the along-track miss
   * as the feet descend through the node's height; positive means overshoot.
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
   * Try releasing forward or pulling back only when the default flight overshoots.
   * Air control cannot rescue a jump that is out of reach.
   */
  private canJumpControlled (
    node: { x: number, y: number, z: number, parkour?: boolean, narrowLanding?: boolean | null },
    sprint: boolean,
    jumpAfter: number
  ): boolean {
    // Only narrow landings use air control during execution, so other landings
    // must pass with forward held. Later take-offs repeat the delayed-jump scan.
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
   * Try up to `budget` scripts per call while the body stays still. Return a script
   * or null when complete, undefined while work remains; landings must survive coasting.
   * Spreading this over more ticks can run out the futility window before it finishes,
   * so bounding its blocking time means moving it off the thread, not slicing it.
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
      // Gentler candidates come first, so a close landing ends the search.
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
    // A touchdown that slides off during coasting is not a landing.
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
   * On narrow landings, engage only when the default flight misses, then minimise
   * predicted along-track miss each tick. Centring a valid flight can remove needed speed.
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
   * Commit parkour take-offs only from the ground, when the landing speed is known.
   * The executor checks again on landing while continuing the current flight.
   */
  private takeoffReady (node: { x: number, y: number, z: number, parkour?: boolean }): boolean {
    if (node.parkour !== true) return true
    return (this.bot.entity as { onGround?: boolean }).onGround === true
  }

  /**
   * A parkour rollout must land near the node; crossing its reach box in mid-air
   * can merely mean falling past it. Continue the flight to check the landing.
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
   * Accept water and climbable catches as well as ground: they can stop a fall
   * without setting onGround. Check water before the body sinks away.
   */
  private caught (state: PlayerState): boolean {
    if (state.isInWater === true) return true
    // A landing can end past the support's edge with onGround still set.
    // Require support under the final position to avoid accepting a fall.
    if (state.onGround === true) return this.carriedAt(state.pos)
    const feet = this.bot.blockAt(
      new Vec3(Math.floor(state.pos.x), Math.floor(state.pos.y), Math.floor(state.pos.z)), false
    ) as { name?: string } | null
    // Climbables catch a falling body without setting onGround.
    return feet !== null && this.climbableName(feet.name ?? '')
  }

  /** Whether the bot's own physics climbs this block. */
  private climbableName (name: string): boolean {
    return name === 'ladder' || name === 'vine' || (isVineClimbable(name) && physicsClimbsVines(this.bot))
  }

  private readonly carryProbe = newBody(0, 0, 0)

  /**
   * Detect onGround with no support after vertical motion hits an edge and
   * horizontal motion leaves it. Return null without a kernel.
   */
  latchedNow (): boolean | null {
    const e = this.bot.entity as { onGround?: boolean, position: { x: number, y: number, z: number } }
    if (this.getKernel() === null) return null
    return e.onGround === true && !this.carriedAt(e.position)
  }

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

  /** Detect wall contact without forward progress, which triggers back-off recovery. */
  isGrinding (path: Array<{ x: number, y: number, z: number }>, ticks = 3): boolean {
    const from = this.bot.entity.position
    const before = Math.hypot(path[0].x - from.x, path[0].z - from.z)
    const state = this.simulateUntil(() => false, this.getController(path[0], false, false), ticks)
    const after = Math.hypot(path[0].x - state.pos.x, path[0].z - state.pos.z)
    return state.isCollidedHorizontally === true && before - after <= GRIND_PROGRESS * ticks
  }

  /**
   * Compare sprint-hopping with sprinting over the same path horizon and floor.
   * Hops preserve the sprint boost against ground friction but still need safe landings.
   */
  sprintHopBetter (
    path: Array<{ x: number, y: number, z: number }>,
    lowCeilingHop = false,
    horizon = 16
  ): boolean {
    // Back the whole horizon with real path so a hop cannot score progress
    // beyond the plan's end or overshoot the goal.
    const look = Math.min(HOP_LOOK, path.length)
    this.hopRefusal = ''
    if (path.length < HOP_LOOK) { this.hopRefusal = 'short'; return false }

    // Rises belong to the jump gates; a hop into one can hit the riser's face.
    // Descents are safe only when they follow the planned path.
    const y0 = this.bot.entity.position.y
    let floor = y0
    for (let i = 0; i < look; i++) {
      const n = path[i] as { x: number, y: number, z: number, parkour?: boolean }
      if (n.parkour === true) { this.hopRefusal = 'jump-ahead'; return false }
      if (n.y > y0 + 0.1) { this.hopRefusal = 'rise-ahead'; return false }
      floor = Math.min(floor, n.y)
      // Edge panels leave no room for landing drift. Walking retains the
      // sneak guard and wall slide; centred narrow supports can still use hops.
      const sup = this.bot.blockAt(new Vec3(Math.floor(n.x), Math.floor(n.y - 0.5), Math.floor(n.z)), false) as { shapes?: number[][] } | null
      const carry = sup !== null && sup.shapes !== undefined && sup.shapes.length > 0 ? carryCode(sup.shapes) : 0
      if (carry >= CARRY_W_NONE && carry <= CARRY_S_NONE) {
        this.hopRefusal = 'narrow-ahead'
        return false
      }
    }

    // A hop reaches airspace walking never checks; reject liquids and lower roofs in its arc.
    // A level roof only shortens the arc. Roofs at four blocks or higher cannot constrain it.
    const roofHere = Math.min(4, this.ceilingAbove(
      Math.floor(this.bot.entity.position.x), Math.floor(y0 + 0.001), Math.floor(this.bot.entity.position.z)))
    // A low roof shortens the arc, so scan only its reachable footprint.
    // A full open-sky scan would reject harmless lower sections farther ahead.
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
    // Compare against the path's floor: both gaits can fall off the same support,
    // so matching each other's height does not establish safety.
    if (hop.minY < floor - HOP_MAX_DIP) { this.hopRefusal = 'dip'; return false }
    if (!hop.endedOnGround) { this.hopRefusal = 'airborne-end'; return false }
    if (!(hop.score > run.score + HOP_MARGIN)) { this.hopRefusal = 'no-gain'; return false }
    return true
  }

  /** Diagnostic reason for the last gait decision. */
  hopJumpNote = ''

  /** Executor-supplied retirement, fall-damage, strafe and landing policies for gait goals. */
  gaitFrom: { x: number, z: number } | null = null
  wholeFall = true
  gaitStrafe = false
  gaitLandRetire = true

  /** Last selected ground policy, retained per bot between gait searches. */
  private readonly gaitPick: GaitPick = { round: false }
  /** Last unsuccessful take-off and its retry delay in ticks. */
  private gaitDead: object | null = null
  private gaitRest = 0

  /**
   * Choose a jump with its flight, a ground tick, or no schedule so the gates decide.
   * Grounded bodies only; stop the horizon before planned programs or obstacle jumps.
   */
  gaitDrive (path: ReadonlyArray<GaitNode>): GaitDrive {
    this.hopRefusal = ''
    this.hopJumpNote = ''
    const k = this.getKernel()
    if (k === null || path.length === 0 || (this.bot.entity as { onGround?: boolean }).onGround !== true) return NO_DRIVE
    if (this.gaitDead === path[0] && this.gaitRest > 0) {
      this.gaitRest--
      this.hopRefusal = 'rest'
      return NO_DRIVE
    }
    const nodes: GaitNode[] = []
    let px = this.bot.entity.position.x
    let pz = this.bot.entity.position.z
    let reach = 0
    for (let i = 0; i < Math.min(path.length, GAIT_NODES); i++) {
      const n = path[i] as GaitNode & { program?: unknown, chain?: boolean, via?: unknown, wOff?: number }
      if ((n.program !== null && n.program !== undefined) || n.chain === true || (n.via !== null && n.via !== undefined)) break
      // Obstacle jumps need their own take-off search from the preceding node;
      // the gait's straight flight cannot substitute for it.
      if (i > 0 && n.wOff !== undefined && n.wOff !== 0) break
      const nx = Math.floor(n.x)
      const nz = Math.floor(n.z)
      if (this.bot.blockAt(new Vec3(nx, Math.floor(n.y - 0.01), nz), false) === null || this.bot.blockAt(new Vec3(nx, Math.floor(n.y + 0.001) + 2, nz), false) === null) break
      nodes.push(n.parkour === true && this.walkedJump(k, path, i) ? { x: n.x, y: n.y, z: n.z, parkour: true, walked: true } : n)
      reach += Math.hypot(n.x - px, n.z - pz)
      px = n.x
      pz = n.z
      if (reach >= GAIT_REACH) break
    }
    if (nodes.length === 0) { this.hopRefusal = 'hard-ahead'; return NO_DRIVE }
    // Programs start from rest on the preceding node; flying past it can hand
    // control to line-up mid-air and overshoot its support. The path end also needs a stop.
    const stoodAt = (m: number): boolean => {
      const next = path[m] as (GaitNode & { program?: unknown, chain?: boolean, via?: unknown, wOff?: number }) | undefined
      return next !== undefined && ((next.program !== null && next.program !== undefined) || next.chain === true || (next.via !== null && next.via !== undefined) ||
        (next.wOff !== undefined && next.wOff !== 0))
    }
    const round = this.gaitPick.round
    let r = gaitSearch(k, this.kBodyNow(), nodes, {
      index: nodes.length - 1,
      last: nodes.length === path.length || stoodAt(nodes.length),
      from: this.gaitFrom,
      wholeFall: this.wholeFall,
      strafe: this.gaitStrafe,
      landRetire: this.gaitLandRetire
    }, this.gaitPick)
    // If the plain window has no schedule, try catching higher on the same
    // ladder or vine to shorten the climb. Keep the gate's catch as a fallback.
    let caught = false
    if (GAIT_CATCH && r.hop === Infinity && r.run === Infinity) {
      const c = this.catchWindow(nodes)
      if (c !== null) {
        caught = true
        const w0 = r.work
        const cut = nodes.slice(0, c.cut)
        this.gaitPick.round = round
        r = gaitSearch(k, this.kBodyNow(), cut, {
          index: cut.length - 1,
          catchAt: c.catchAt,
          // Climbing ends the schedule without requiring a grounded stop.
          last: stoodAt(cut.length),
          from: this.gaitFrom,
          wholeFall: this.wholeFall,
          strafe: this.gaitStrafe,
          landRetire: this.gaitLandRetire
        }, this.gaitPick)
        // Landing on the catch node itself is also valid, even without a higher catch.
        r = { ...r, work: r.work + w0 }
      }
    }
    const hop = r.hop !== Infinity && r.run === Infinity
    const step = r.run !== Infinity ? r.step : null
    const run = step !== null
    const facts = { ticks: Math.min(r.hop, r.run), cut: !caught && nodes.length < path.length && stoodAt(nodes.length) ? nodes.length : -1, goal: r.goal, goalLast: r.goalLast, work: r.work }
    const f = (v: number): string => v === Infinity ? '-' : Number.isInteger(v) ? String(v) : v.toFixed(2)
    this.hopJumpNote = `${hop ? 'hop' : run ? 'run' : 'none'} h${f(r.hop)} r${f(r.run)} n${nodes.length}${stoodAt(nodes.length) ? 's' : ''} g${r.goal}${r.goalLast ? 'L' : ''} w${r.work}${r.spent ? ' spent' : ''}`
    if (hop) return { kind: 'hop', flight: r.flight, strafe: r.strafe, ease: r.ease, yaw: r.flight[0], sprint: true, ...facts }
    if (step !== null) return { kind: 'run', flight: null, strafe: step.strafe, ease: EASE_NEVER, yaw: step.yaw, sprint: step.sprint, ...facts }
    // Delay failed searches briefly; oracle-only jumps can otherwise retry every tick.
    this.gaitDead = path[0]
    this.gaitRest = GAIT_REST
    return NO_DRIVE
  }

  /**
   * Extend the window up the first ladder or vine catch's column by CATCH_CLIMB.
   * Scaffolding is excluded because a sneaking body can slide down it.
   */
  private catchWindow (nodes: GaitNode[]): { catchAt: number, cut: number } | null {
    let py = this.bot.entity.position.y
    for (let j = 0; j < nodes.length; j++) {
      const n = nodes[j]
      const jump = (n.parkour === true && n.walked !== true) || n.y > py + SIM.stepHeight
      py = n.y
      if (!jump || (!this.ladderOrVine(n.x, n.y + 0.001, n.z) && !this.ladderOrVine(n.x, n.y - 0.999, n.z))) continue
      let m = j + 1
      while (m < nodes.length && Math.floor(nodes[m].x) === Math.floor(n.x) && Math.floor(nodes[m].z) === Math.floor(n.z) &&
        nodes[m].y > nodes[m - 1].y && nodes[m].y <= n.y + CATCH_CLIMB) m++
      return { catchAt: j, cut: m }
    }
    return null
  }

  private ladderOrVine (x: number, y: number, z: number): boolean {
    const b = this.bot.blockAt(new Vec3(Math.floor(x), Math.floor(y), Math.floor(z)), false) as { name?: string } | null
    return b !== null && this.climbableName(b.name ?? '')
  }

  /** Check level-ground access to `node` after a landing overshoots its aim. */
  walksTo (node: { x: number, y: number, z: number }): boolean {
    const k = this.getKernel()
    return k !== null && kWalkable(k, this.bot.entity.position, node)
  }

  /** Cached walkability of parkour nodes from their preceding nodes. */
  private readonly walked = new WeakMap<object, boolean>()

  /**
   * Detect parkour nodes reachable on foot from the preceding node. The planner
   * can price flat-ground sprint jumps below walks, so a jump flag alone is insufficient.
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


  /** Diagnostic result of the last live take-off search. */
  hopNote = ''
  /** Touchdown damage live hop searches refuse; the executor lowers it under a health floor. */
  fallVeto = FALL_VETO

  /**
   * Oracle over the live grid, consistent for one tick without copying the world.
   * Return null without shape tables.
   */
  private liveOracle (): HopOracle | null {
    const lut = this.kTablesLut
    const k = this.getKernel()
    if (this.kTables === null || lut === null || k === null) return null
    const simWorld: SimWorld = { ...this.kTables, stateAt: (x, y, z) => this.kStateAt(x, y, z), grid: this.kGrid }
    const oracle = new HopOracle(simWorld, (x, y, z) => { const st = k.stateAt(x, y, z); return st < 0 ? 0 : lut.flags[st] },
      k.halfWidth, k.height, k.speedBase)
    oracle.fallVeto = this.fallVeto
    return oracle
  }

  /**
   * Find a stand under the body. At a lip, the centre can overhang a gap, so use
   * the nearest supporting cell when the centre's cell has no stand.
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
   * Find the quickest robust live program to `cell` from the current grounded body,
   * including walking jumps; return null if none is found.
   */
  hopFrom (cell: readonly [number, number, number], budgetTicks: number, land?: { x: number, y: number, z: number }, next?: readonly [number, number, number], beam = true,
    seed?: ReadonlyArray<'kick' | 'pivot'>, seedFirst = true, hint?: HopProgram): HopProgram | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    o.tickLimit = budgetTicks
    // Seed families need their own work allowance and a matching wall-time cap.
    const seedTicks = seed !== undefined && seed.length > 0 ? SEED_FAMILY_TICKS : 0
    const laneTicks = hint?.family === 'kick' ? LANE_KICK_TICKS : 0
    // Preserve the region landing target and choose a landing for the next leg when supplied.
    const found = o.hopFrom(this.kBodyNow(), [cell[0], cell[1], cell[2]], t0 + (budgetTicks + seedTicks + laneTicks) / LIVE_FLOOR_TICKS_PER_MS, this.bot.entity.yaw, land,
      next !== undefined ? [next[0], next[1], next[2]] : undefined, beam, seed, seedTicks, seedFirst, hint)
    this.hopNote = `from ${found !== null ? `${found.family} ${found.ticks}t` : 'none'}${seedTicks > 0 ? ` +${seed!.join('+')}` : ''} ${(performance.now() - t0).toFixed(0)}ms ${Math.round(o.ticksFlown / 1000)}k`
    return found
  }

  /** A body resting flush against a wall needs room before jumping; share one allowance across the maneuver. */
  hopPrefix (cell: readonly [number, number, number], land?: { x: number, y: number, z: number }, hint?: HopProgram): HopProgram | null {
    if (!LIVE_PREFIX) return null
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    o.tickLimit = LIVE_PREFIX_TICKS
    const found = o.hopPrefix(this.kBodyNow(), [cell[0], cell[1], cell[2]], t0 + LIVE_PREFIX_TICKS / LIVE_FLOOR_TICKS_PER_MS,
      this.bot.entity.yaw, land, hint)
    this.hopNote = `prefix ${found !== null ? `${found.family} ${found.ticks}t` : 'none'} ${(performance.now() - t0).toFixed(0)}ms ${Math.round(o.ticksFlown / 1000)}k`
    return found
  }

  /**
   * Find a live program onto `cell` that minimises total ticks through `next`, or
   * null if no landing has a next leg. Grounded only; `nextGo` continues past `next`.
   */
  hopChain (cell: readonly [number, number, number], next: readonly [number, number, number], budgetTicks: number, nextGo = false): HopProgram | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    o.tickLimit = budgetTicks
    const found = o.hopChain(this.kBodyNow(), [cell[0], cell[1], cell[2]], [next[0], next[1], next[2]], t0 + budgetTicks / LIVE_FLOOR_TICKS_PER_MS, this.bot.entity.yaw, nextGo)
    this.hopNote = `chain ${found !== null ? `${found.family} ${found.ticks}t` : 'none'} ${(performance.now() - t0).toFixed(0)}ms ${Math.round(o.ticksFlown / 1000)}k`
    return found
  }

  /** Whether the last skipAhead search consumes one of the node's attempts. */
  skipSearched = false

  /**
   * Try a direct flight past the gait's stopping node to the planned program's target.
   * Accept only a robust landing faster than the gait plus the planned leg's lower bound.
   */
  skipAhead (path: ReadonlyArray<GaitNode>, drive: GaitDrive): { program: HopProgram, cut: number } | null {
    this.skipSearched = false
    const c = drive.cut ?? -1
    if (c < 1 || c > SKIP_NODES || c >= path.length || drive.goal !== c - 1 || drive.goalLast !== true) return null
    const t0 = drive.ticks ?? Infinity
    if (!Number.isFinite(t0)) return null
    return this.skipTo(path, c, t0, path[c - 1], SKIP_TICKS, false)
  }

  /**
   * Try flying straight to the node after an obstacle jump when it saves time.
   * Include the beam's detour search so one flight can clear both obstacles.
   */
  skipNeo (path: ReadonlyArray<GaitNode>, t0: number, rest: boolean): { program: HopProgram, cut: number } | null {
    this.skipSearched = false
    if (path.length < 2 || !Number.isFinite(t0)) return null
    const s = path[0] as GaitNode & { cell?: [number, number, number] }
    const from = s.cell !== undefined ? { x: s.cell[0] + 0.5, y: s.y, z: s.cell[2] + 0.5 } : s
    return this.skipTo(path, 1, t0, from, rest ? SKIP_NEO_REST_TICKS : SKIP_NEO_TICKS, true)
  }

  /**
   * Compare the gait's walk detour to the first jump with a direct robust flight.
   * `allowed` indicates whether the target node has search attempts remaining.
   */
  skipWalk (path: ReadonlyArray<GaitNode>, allowed: (m: GaitNode) => boolean): { program: HopProgram, cut: number } | null {
    this.skipSearched = false
    const k = this.getKernel()
    if (k === null || path.length < 2) return null
    type Node = GaitNode & { program?: HopProgram | null, chain?: boolean, via?: unknown, wOff?: number, toBreak?: unknown[], toPlace?: unknown[], cell?: [number, number, number] }
    const b = this.bot.entity.position
    let c = -1
    let walk = 0
    let px = b.x
    let pz = b.z
    for (let i = 0; i < Math.min(path.length, SKIP_WALK_NODES + 1); i++) {
      const n = path[i] as Node
      if ((n.program !== null && n.program !== undefined) || n.chain === true || (n.via !== null && n.via !== undefined) || (n.wOff !== undefined && n.wOff !== 0)) return null
      if ((n.toBreak?.length ?? 0) > 0 || (n.toPlace?.length ?? 0) > 0) return null
      walk += Math.hypot(n.x - px, n.z - pz)
      px = n.x
      pz = n.z
      if (n.parkour === true && !this.walkedJump(k, path, i)) { c = i; break }
    }
    if (c < 1) return null
    const m = path[c] as Node
    if ((path[c + 1] as Node | undefined)?.chain === true || this.climbableNode(m) || !allowed(m)) return null
    const direct = Math.hypot(m.x - b.x, m.z - b.z)
    if (walk - direct < SKIP_WALK_DETOUR || direct > ONE_FLIGHT_REACH || m.y - b.y > SKIP_RISE || b.y - m.y > SKIP_DROP) return null
    const nodes: GaitNode[] = []
    for (let i = 0; i <= c; i++) {
      const n = path[i]
      nodes.push(i < c && n.parkour === true ? { x: n.x, y: n.y, z: n.z, parkour: true, walked: true } : n)
    }
    const round = this.gaitPick.round
    const r = gaitSearch(k, this.kBodyNow(), nodes, { index: c, last: true, best: true, from: this.gaitFrom, wholeFall: this.wholeFall, strafe: this.gaitStrafe, landRetire: this.gaitLandRetire }, this.gaitPick)
    this.gaitPick.round = round
    const t0 = Math.min(r.hop, r.run)
    // Skip the search when even a minimum-cost flight cannot win.
    if (!Number.isFinite(t0) || legFloor(k, b, m) + SKIP_MARGIN > t0) return null
    this.skipSearched = true
    const cell: [number, number, number] = m.cell ?? [Math.floor(m.x), Math.floor(m.y + 0.001), Math.floor(m.z)]
    const after = path[c + 1] as Node | undefined
    const next = after !== undefined && after.parkour === true ? (after.cell ?? [Math.floor(after.x), Math.floor(after.y + 0.001), Math.floor(after.z)]) : undefined
    const p = this.hopFrom(cell, SKIP_TICKS, undefined, next, false)
    const note = this.hopNote
    const o = p !== null ? this.liveOracle() : null
    const hurt = o !== null && p !== null && o.landingHurts(p, this.wholeFall) ? HURT_TICKS : 0
    if (p === null || o === null || p.deep === true || p.climb === true || p.precise === true || p.ticks > SKIP_MAX_TICKS || p.ticks + hurt + SKIP_MARGIN > t0) {
      this.hopNote = `skip walk ${p !== null ? `${p.ticks}t${hurt > 0 ? ' hurt' : ''}` : 'none'} vs ${f2(t0)} (${note})`
      return null
    }
    if (!o.singleFlight(p)) {
      this.hopNote = `skip walk ${p.ticks}t not one flight (${note})`
      return null
    }
    this.hopNote = `skip walk ${p.ticks}t vs ${f2(t0)} (${note})`
    return { program: p, cut: c }
  }

  /** Whether a ladder or vine occupies the node's feet cell or the cell below it. */
  private climbableNode (n: { x: number, y: number, z: number }): boolean {
    return this.ladderOrVine(n.x, n.y + 0.001, n.z) || this.ladderOrVine(n.x, n.y - 0.999, n.z)
  }

  /**
   * Search past path[0..c-1] to path[c] from the current body with `ticks` of work.
   * Compare against `t0` plus the planned leg's floor; optionally include the beam.
   */
  private skipTo (path: ReadonlyArray<GaitNode>, c: number, t0: number, s: { x: number, y: number, z: number }, ticks: number, beam: boolean): { program: HopProgram, cut: number } | null {
    type Node = GaitNode & { program?: HopProgram | null, chain?: boolean, via?: unknown, toBreak?: unknown[], toPlace?: unknown[], cell?: [number, number, number] }
    for (let i = 0; i <= c; i++) {
      const n = path[i] as Node
      if (n.chain === true || (n.via !== null && n.via !== undefined)) return null
      if ((n.toBreak?.length ?? 0) > 0 || (n.toPlace?.length ?? 0) > 0) return null
    }
    const m = path[c] as Node
    if (m.parkour !== true) return null
    // Catch-window handling owns ladder arrivals; retiring a skipped catch mid-air can lose it.
    if (this.climbableNode(m)) return null
    // A momentum chain needs its planned approach to re-jump on the landing tick.
    if ((path[c + 1] as Node | undefined)?.chain === true) return null
    const planned = m.program ?? null
    // Search kick and pivot plans in their own families; a skip can avoid their exact line-up.
    const seed = SEED_FAMILY && planned !== null ? seedFamilies(planned) : undefined
    if (planned !== null && (planned.deep === true || planned.climb === true || (planned.precise === true && seed === undefined) || planned.seq !== undefined || planned.start !== undefined)) return null
    const b = this.bot.entity.position
    if (Math.hypot(m.x - b.x, m.z - b.z) > ONE_FLIGHT_REACH || m.y - b.y > SKIP_RISE || b.y - m.y > SKIP_DROP) return null
    // At the planned start already, execution belongs to the program branch.
    if (Math.floor(b.x) === Math.floor(s.x) && Math.floor(b.z) === Math.floor(s.z) && Math.abs(b.y - s.y) < 0.5) return null
    const k = this.getKernel()
    if (k === null) return null
    // Kicks and pivots start from rest, so price their program rather than a generic flight.
    // Other plans may be reached with momentum that permits a quicker live leg.
    const leg = SKIP_PLAN_LEG && planned !== null && seed !== undefined ? Math.max(legFloor(k, s, m), planned.ticks) : legFloor(k, s, m)
    // Without a finite planned-leg floor, the skip cannot be compared.
    if (!Number.isFinite(leg)) return null
    // Skip the search when even a minimum-cost flight cannot win.
    if (legFloor(k, b, m) + SKIP_MARGIN > t0 + leg) return null
    this.skipSearched = true
    const cell: [number, number, number] = m.cell ?? [Math.floor(m.x), Math.floor(m.y + 0.001), Math.floor(m.z)]
    const after = path[c + 1] as Node | undefined
    const next = after !== undefined && after.parkour === true ? (after.cell ?? [Math.floor(after.x), Math.floor(after.y + 0.001), Math.floor(after.z)]) : undefined
    const p = this.hopFrom(cell, ticks, planned?.land, next, beam, seed)
    const note = this.hopNote
    const o = p !== null ? this.liveOracle() : null
    // Charge damaging landings the same delay as the gait.
    const hurt = o !== null && p !== null && o.landingHurts(p, this.wholeFall) ? HURT_TICKS : 0
    // A live pivot replays from the exact state searched, without another line-up.
    if (p === null || o === null || p.deep === true || p.climb === true || (p.precise === true && !SEED_FAMILY) || p.ticks > SKIP_MAX_TICKS || p.ticks + hurt + SKIP_MARGIN > t0 + leg) {
      this.hopNote = `skip ${p !== null ? `${p.ticks}t${hurt > 0 ? ' hurt' : ''}` : 'none'} vs ${f2(t0)}+${leg} (${note})`
      return null
    }
    if (!o.singleFlight(p)) {
      this.hopNote = `skip ${p.ticks}t not one flight (${note})`
      return null
    }
    this.hopNote = `skip ${p.ticks}t vs ${f2(t0)}+${leg} (${note})`
    return { program: p, cut: c }
  }


  /** Re-anchor `prog` to the current body if it still lands robustly at `cell`. */
  hopAnchored (cell: readonly [number, number, number], prog: HopProgram): HopProgram | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    const found = o.anchored(prog, this.kBodyNow(), this.bot.entity.yaw)
    this.hopNote = `anchored ${found !== null ? `${found.ticks}t` : 'no'} ${(performance.now() - t0).toFixed(0)}ms`
    return found
  }

  /**
   * Check the remaining deep-bounce replay from the current body and counters,
   * including whether its basin abort still lands in the pool. Null without a kernel.
   */
  deepVerdict (prog: HopProgram, t: number, fired: boolean, air: number, rebounds: number): { arrives: boolean, abortable: boolean } | null {
    const o = this.liveOracle()
    if (o === null) return null
    return o.deepVerdict(prog, this.kBodyNow(), t, fired, air, rebounds, this.bot.entity.yaw)
  }

  // A moving line-up retains velocity; `tried` distinguishes an unsafe or
  // distant glide from the approach's one correction search.
  hopLineUp (prog: HopProgram, budgetTicks: number, moving?: { reach: number, tried: boolean }): { yaws: number[], delay?: number, prog: HopProgram } | null {
    const t0 = performance.now()
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; return null }
    o.tickLimit = budgetTicks
    const found = o.lineUp(prog, this.kBodyNow(), this.bot.entity.yaw, t0 + budgetTicks / LIVE_FLOOR_TICKS_PER_MS, moving)
    this.hopNote = `lineup ${found !== null ? `${found.yaws.length} press` : 'none'} ${(performance.now() - t0).toFixed(0)}ms ${Math.round(o.ticksFlown / 1000)}k`
    return found
  }

  /**
   * Find a program to `cell` from rest at a stand on the body's current node.
   * The executor lines up at that stand first; return null if none is found.
   */
  hopLinedUp (cell: readonly [number, number, number], budgetTicks: number): HopProgram | null {
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; this.hopExhausted = false; this.hopNoStand = true; return null }
    return this.linedVerdict(o, this.standCell(o, this.bot.entity.position), cell, budgetTicks, LINED_BEAM_TICKS)
  }

  /**
   * Search from `from`'s stands to `to` before the body arrives. Set hopExhausted
   * when all simple and yaw candidates fail; return null without a kernel.
   */
  /** `beam` grants a separate beam budget for a stuck take-off. */
  hopFromCell (from: readonly [number, number, number], to: readonly [number, number, number], budgetTicks: number, beam = false): HopProgram | null {
    const o = this.liveOracle()
    if (o === null) { this.hopNote = 'n/a'; this.hopExhausted = false; this.hopNoStand = true; return null }
    return this.linedVerdict(o, [from[0], from[1], from[2]], to, budgetTicks, beam ? LINED_BEAM_TICKS : 0)
  }

  /**
   * `beamTicks` reserves work after the families finish so they cannot starve the
   * last-resort beam search; zero lets the beam use only the shared budget's remainder.
   */
  private linedVerdict (o: HopOracle, from: [number, number, number], cell: readonly [number, number, number], budgetTicks: number, beamTicks = 0): HopProgram | null {
    const t0 = performance.now()
    // Try cheaper families first so yaw search cannot consume their whole budget.
    const to: [number, number, number] = [cell[0], cell[1], cell[2]]
    this.linedFrom = from
    let deadline = t0 + budgetTicks / LIVE_FLOOR_TICKS_PER_MS
    o.tickLimit = budgetTicks
    let found = o.hop(from, to, deadline, 'simple', true)
    let ran = !o.noVerdict
    this.hopNoStand = found === null && o.noStand
    if (found === null && !this.hopNoStand) { found = o.hop(from, to, deadline, 'yaw', true); ran = ran && !o.noVerdict }
    // Exhausted families establish failure even if the fallback beam runs out of time.
    this.hopExhausted = found === null && ran
    if (found === null && !this.hopNoStand) {
      if (beamTicks > 0) {
        o.tickLimit = o.ticksFlown + beamTicks
        deadline = performance.now() + beamTicks / LIVE_FLOOR_TICKS_PER_MS
      }
      found = o.hop(from, to, deadline, 'beam')
    }
    this.hopNote = `lined ${found !== null ? `${found.family} ${found.ticks}t` : this.hopNoStand ? 'no-stand' : this.hopExhausted ? 'none-exhausted' : 'none'} ${(performance.now() - t0).toFixed(0)}ms ${Math.round(o.ticksFlown / 1000)}k`
    return found
  }

  /** True when the simple and yaw families finish without a landing. */
  hopExhausted = false
  /** True when the last lined search has no take-off or landing stand. */
  hopNoStand = false
  /** Supporting stand cell used by the last lined search, including lip supports. */
  linedFrom: readonly [number, number, number] | null = null

  /**
   * Capped height of the first non-empty cell above a standing body.
   * Treat unloaded cells as low roofs so the hop guard cannot assume clearance.
   */
  private ceilingAbove (x: number, y: number, z: number, cap = 4): number {
    for (let dy = 2; dy <= cap; dy++) {
      const b = this.bot.blockAt(new Vec3(x, y + dy, z), false) as { boundingBox?: string } | null
      if (b === null || b.boundingBox !== 'empty') return dy
    }
    return cap + 1
  }

  /**
   * Score path progress over `horizon` ticks using the executor's node tolerances.
   * Press jump only on grounded ticks; a held key's 10-tick cooldown delays low-roof hops.
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
    // Allow the hop's remaining airtime to settle with controls released;
    // a body over a hole keeps falling instead of earning a safe landing.
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
   * Find a take-off heading that clears nearby corners and lands the jump.
   * Try offsets smallest first; return radians, or null when no heading lands.
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
   * Find a small heading offset that walks around a corner instead of jumping it.
   * Try nonzero offsets, smallest first; return null on failure.
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
   * Check that an escape nudge moves the body and leaves it standing without a fall.
   * Empty space behind the body does not guarantee a safe place to stand.
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

  /**
   * Hold `keys` for `keyTicks`, then coast; null uses current controls for the first tick.
   * Allow at most `maxDrop` descent. Sneak persists unless overridden but can miss thin panels.
   */
  holdsOn (keys: { forward?: boolean, back?: boolean, sneak?: boolean } | null, ticks = HOLD_COAST_TICKS, keyTicks = 1, maxDrop = 0.6): boolean {
    const y0 = this.bot.entity.position.y
    const state = this.simulateUntil(() => false, (s: PlayerState, i: number) => {
      if (keys === null && i === 0) return
      s.control.forward = i < keyTicks && keys?.forward === true
      s.control.back = i < keyTicks && keys?.back === true
      s.control.left = false
      s.control.right = false
      s.control.jump = false
      s.control.sprint = false
      if (i === 0 && keys?.sneak !== undefined) s.control.sneak = keys.sneak
    }, ticks)
    return state.onGround === true && state.pos.y >= y0 - maxDrop
  }

  canBackOff (ticks = 5): boolean {
    return this.canNudge({ back: true }, ticks)
  }

  private getReached (path: Array<{ x: number, y: number, z: number, parkour?: boolean }>): (state: PlayerState) => boolean {
    // Walking nodes require their own level, not the top of an adjacent block.
    // Parkour nodes allow more height tolerance and require a settled landing.
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
   * Simulate one jump followed by walking. Holding jump after touchdown could
   * approve a second take-off from a cell the planner never routed through.
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

      // Server corrections clear onGround even on solid support, so the seeded
      // flag cannot identify take-off. Use simulated upward motion or jump cooldown.
      // A ceiling can erase upward velocity within the jump tick; the cooldown
      // still records that it fired and prevents another jump after landing.
      if (!fired) {
        if (state.vel.y > 0 || (pressed && (state as unknown as { jumpTicks?: number }).jumpTicks === cooldown)) fired = true
      } else if (state.onGround === true) {
        landed = true
      }

      // Reset all controls so a live back-off or sneak does not leak into
      // the rollout and change the movement being evaluated.
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
