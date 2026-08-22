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

export class PhysicsSim {
  private readonly bot: Bot
  private readonly world: { getBlock: (pos: Vec3) => unknown }
  /** Set by the last simulateUntil that hit its refusal predicate. */
  private refused = false

  constructor (bot: Bot) {
    this.bot = bot
    this.world = { getBlock: (pos: Vec3) => bot.blockAt(pos, false) }
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
      state = new PlayerState(this.bot, simulationControl)
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
    const state = new PlayerState(this.bot, simulationControl)
    state.pos.update(n1)
    this.simulateUntil(reached, this.getController(n2, false, true), Math.floor(5 * n1.distanceTo(n2)), state)
    return reached(state)
  }

  // 45 ticks, not upstream's 20: extended-parkour drop landings spend up to
  // ~18 ticks airborne and then run in to the node center — at 20 the sim
  // budget expired mid-flight and legal deep drops were never attempted.
  canSprintJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    const reached = this.getReached(path)
    const state = this.simulateUntil(reached, this.getController(path[0], true, true, jumpAfter), 45)
    return reached(state)
  }

  canWalkJump (path: Array<{ x: number, y: number, z: number }>, jumpAfter = 0): boolean {
    const reached = this.getReached(path)
    const state = this.simulateUntil(reached, this.getController(path[0], true, false, jumpAfter), 45)
    return reached(state)
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
  sprintHopBetter (path: Array<{ x: number, y: number, z: number }>, horizon = 16): boolean {
    // Level ground only, and only where the planner is walking rather than
    // jumping. This is a cheap precondition, but it is also the honest scope
    // of the gain: the 25% is what a sprint-hop buys on a plain. On anything
    // stepped, the arc is doing the planner's job for it — and it is exactly
    // there that a hop can leave a ledge that a sprint would have stopped at.
    const y0 = this.bot.entity.position.y
    const look = Math.min(3, path.length)
    for (let i = 0; i < look; i++) {
      const n = path[i] as { y: number, parkour?: boolean }
      if (n.parkour === true || Math.abs(n.y - y0) > 0.1) return false
    }

    const run = this.followPath(path, false, horizon)
    const hop = this.followPath(path, true, horizon)
    if (hop === null || run === null) return false
    // Absolute, not relative. Comparing the two gaits' heights was not enough:
    // on the arena's climb1 tower BOTH of them fell off a 1-wide pillar, the
    // hop was still "no worse", and the bot flew off the side of a 46-block
    // climb it had been walking correctly (91.8 blocks travelled, y 141 to
    // 102, 15 damage). A hop is only allowed to keep its feet at the level it
    // started on and put them back down there.
    if (hop.minY < y0 - HOP_MAX_DIP || !hop.endedOnGround) return false
    return hop.score > run.score + HOP_MARGIN
  }

  /**
   * Drive `path` for `horizon` ticks with one gait and score how far along it
   * got. Nodes are consumed the way the executor consumes them, with the
   * apex tolerance a hop needs (see HOP_ARRIVE_DY in plugin.ts).
   */
  private followPath (
    path: Array<{ x: number, y: number, z: number }>,
    jump: boolean,
    horizon: number
  ): { score: number, minY: number, endedOnGround: boolean } | null {
    const state = new PlayerState(this.bot, {
      forward: true, back: false, left: false, right: false, jump, sprint: true, sneak: false
    })
    const simulatePlayer = (this.bot.physics as unknown as { simulatePlayer: (s: PlayerState, w: unknown) => void }).simulatePlayer
    let i = 0
    let minY = state.pos.y
    for (let t = 0; t < horizon; t++) {
      const target = path[Math.min(i, path.length - 1)]
      state.yaw = Math.atan2(-(target.x - state.pos.x), -(target.z - state.pos.z))
      state.control.forward = true
      state.control.jump = jump
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

  private getReached (path: Array<{ x: number, y: number, z: number }>): (state: PlayerState) => boolean {
    return (state: PlayerState) => {
      const delta = {
        x: path[0].x - state.pos.x,
        y: path[0].y - state.pos.y,
        z: path[0].z - state.pos.z
      }
      return Math.abs(delta.x) <= 0.35 && Math.abs(delta.z) <= 0.35 && Math.abs(delta.y) < 1
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
