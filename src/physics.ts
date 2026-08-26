// Port of mineflayer-pathfinder/lib/physics.js (MIT): the executor's
// sprint/jump decisions come from short prismarine-physics simulations
// against the LIVE world — same engine the bot moves with, so path following
// inherits none of the solver's approximations.
import { PlayerState } from 'prismarine-physics'
import type { SimControl } from 'prismarine-physics'
import type { Bot } from 'mineflayer'
import { Vec3 } from 'vec3'

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
 * How far below the take-off level a hop rollout may dip before it counts as
 * having left the ground it was crossing. A hop that lands lower than it
 * started is a fall the sprint would not have taken.
 */
const HOP_MAX_DIP = 0.5

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

/** Half-extent of the per-tick block cache around the body, in blocks. */
const CACHE_REACH = 64

/** Height tolerance for reaching a WALKING node in a rollout (see getReached). */
const WALK_REACH_DY = 0.5

/**
 * Nodes ahead the sprint-hop gait needs backed by path and free of rises
 * and jumps (see sprintHopBetter). PF_HOP_LOOK overrides it for A/B runs.
 */
const HOP_LOOK = Math.max(2, Number(process.env.PF_HOP_LOOK ?? 6) || 6)

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

  /** Open a tick: forget the last tick's blocks and state template. */
  beginTick (): void {
    this.tickSerial++
    this.blockCache.clear()
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
    const state = this.simulateUntil(
      reached, this.getController(path[0], false, sprint), 200, null,
      this.grindGuard(path[0], this.bot.entity.position)
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
    const state = this.simulateUntil(reached, this.getController(path[0], true, true, jumpAfter), 45)
    return reached(state) && this.landsThere(path[0], state, true)
  }

  canWalkJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    if (!this.takeoffReady(path[0])) return false
    const reached = this.getReached(path)
    const state = this.simulateUntil(reached, this.getController(path[0], true, false, jumpAfter), 45)
    return reached(state) && this.landsThere(path[0], state, false)
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
    if (state.onGround === true || state.isInWater === true) return true
    const feet = this.bot.blockAt(
      new Vec3(Math.floor(state.pos.x), Math.floor(state.pos.y), Math.floor(state.pos.z)), false
    ) as { name?: string } | null
    return feet !== null && CAUGHT_FEET.has(feet.name ?? '')
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
   */
  sprintHopBetter (
    path: Array<{ x: number, y: number, z: number }>,
    lowCeilingHop = false,
    horizon = 16
  ): boolean {
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
      const n = path[i] as { y: number, parkour?: boolean }
      if (n.parkour === true) { this.hopRefusal = 'jump-ahead'; return false }
      if (n.y > y0 + 0.1) { this.hopRefusal = 'rise-ahead'; return false }
      floor = Math.min(floor, n.y)
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
      if (!fired) {
        if (state.vel.y > 0) fired = true
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
    }
  }
}
