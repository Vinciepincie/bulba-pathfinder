// The executor's contract with the SERVER, not just with the physics engine.
//
// prismarine-physics happily produces positions a real server refuses, and a
// refused position is a teleport back — every tick, for as long as the bot
// keeps producing it. Measured on 2b2t spawn (bench/arena, route simple2):
// two unrelated spots on one route where the bot sat motionless for the rest
// of the run while upstream walked past both, and one where the take-off the
// rollout approved landed the bot in a 1-block pit two cells short.
//
// Each test here reproduces one of those against the pre-fix code.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createRequire } from 'node:module'
import { PhysicsSim } from '../src/physics.js'
import { nearWall, playerCollides } from '../src/geometry.js'
import { createPathfinder } from '../src/plugin.js'
import { GoalBlock } from '../src/goals.js'
import { Movements } from '../src/movements.js'
import {
  VoxelWorld, STONE, AIR, WATER, LAVA, mcData, Block, TEST_VERSION, makeFakeBot, makeOurMovements
} from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

const require = createRequire(import.meta.url)
/* eslint-disable @typescript-eslint/no-var-requires */
const { Physics } = require('prismarine-physics')
/* eslint-enable @typescript-eslint/no-var-requires */

/**
 * A bot with REAL prismarine-physics behind `bot.physics`, which is what
 * PhysicsSim rolls out against — the fake bot's stub would decide nothing.
 */
function physicsBot (world: VoxelWorld, pos: Vec3): unknown {
  return {
    version: TEST_VERSION,
    registry: mcData,
    physics: Physics(mcData, null),
    controlState: { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false },
    entity: {
      position: pos,
      velocity: new Vec3(0, 0, 0),
      onGround: true,
      isInWater: false,
      isInLava: false,
      isInWeb: false,
      isCollidedHorizontally: false,
      isCollidedVertically: false,
      elytraFlying: false,
      attributes: {},
      effects: {},
      yaw: 0,
      pitch: 0
    },
    jumpTicks: 0,
    jumpQueued: false,
    fireworkRocketDuration: 0,
    inventory: { slots: [] },
    blockAt (p: Vec3): unknown {
      const x = Math.floor(p.x); const y = Math.floor(p.y); const z = Math.floor(p.z)
      const state = world.stateAt(x, y, z)
      if (state === null) return null
      const b = Block.fromStateId(state, 0) as { position: Vec3 }
      b.position = new Vec3(x, y, z)
      return b
    }
  }
}

describe('executor: take-off honesty', () => {
  /**
   * Take-off at x=0, a gap, a LEDGE at x=3 the jump actually lands on, and
   * the node one higher at x=4. The rollout holds jump for its whole budget,
   * so from the ledge it simply jumps again and satisfies the node many ticks
   * later — from a cell the planner never routed through. Committing to that
   * take-off is how the bot ended up in a pit on 2b2t spawn.
   */
  function ledgeWorld (): VoxelWorld {
    const w = new VoxelWorld({ x0: -6, y0: -8, z0: -4, x1: 10, y1: 8, z1: 4 })
    w.set(0, 0, 0, STONE) // take-off, stand y = 1
    w.set(-1, 0, 0, STONE) // run-up
    w.set(3, 0, 0, STONE) // the short landing
    w.fill(4, 0, 0, 4, 1, 0, STONE) // the node's block, one higher
    return w
  }

  it('refuses a jump whose rollout touches down short of the node', () => {
    const world = ledgeWorld()
    const sim = new PhysicsSim(physicsBot(world, new Vec3(0.5, 1, 0.5)) as never)
    const node = { x: 4.5, y: 2, z: 0.5 }

    // The old rule — "did any tick in the budget satisfy the node" — says yes,
    // because the bot bounces on from the ledge it lands on.
    expect(reachesEventually(world, new Vec3(0.5, 1, 0.5), node)).to.equal(true)
    // The take-off rule says no: this jump does not go where it claims.
    expect(sim.canSprintJump([node])).to.equal(false)
    expect(sim.canWalkJump([node])).to.equal(false)
  })

  it('still accepts a jump that flies straight to its node', () => {
    const w = new VoxelWorld({ x0: -6, y0: -8, z0: -4, x1: 10, y1: 8, z1: 4 })
    w.set(0, 0, 0, STONE)
    w.set(-1, 0, 0, STONE)
    w.set(3, 0, 0, STONE) // same 3-block gap, but the node IS the landing
    const sim = new PhysicsSim(physicsBot(w, new Vec3(0.5, 1, 0.5)) as never)
    expect(sim.canSprintJump([{ x: 3.5, y: 1, z: 0.5 }])).to.equal(true)
  })

  it('landing on the node block off-centre and running in still counts', () => {
    // A wide plateau: the jump lands on it a little short of the node centre
    // and walks the rest, which is a real landing, not a bounce.
    const w = new VoxelWorld({ x0: -6, y0: -8, z0: -4, x1: 12, y1: 8, z1: 4 })
    w.set(0, 0, 0, STONE)
    w.set(-1, 0, 0, STONE)
    w.fill(3, 0, 0, 8, 0, 0, STONE)
    const sim = new PhysicsSim(physicsBot(w, new Vec3(0.5, 1, 0.5)) as never)
    expect(sim.canSprintJump([{ x: 4.5, y: 1, z: 0.5 }])).to.equal(true)
  })

  /** The pre-fix predicate: any tick in the budget satisfying the node. */
  function reachesEventually (world: VoxelWorld, from: Vec3, node: { x: number, y: number, z: number }): boolean {
    const bot = physicsBot(world, from.clone()) as {
      physics: { simulatePlayer: (s: unknown, w: unknown) => void }
    }
    const { PlayerState } = require('prismarine-physics')
    const state = new PlayerState(bot, { forward: true, back: false, left: false, right: false, jump: true, sprint: true, sneak: false })
    const adapter = { getBlock: (p: Vec3) => (bot as unknown as { blockAt: (v: Vec3) => unknown }).blockAt(p) }
    for (let t = 0; t < 45; t++) {
      const dx = node.x - state.pos.x
      const dz = node.z - state.pos.z
      state.yaw = Math.atan2(-dx, -dz)
      state.control.forward = true
      state.control.jump = true
      state.control.sprint = true
      bot.physics.simulatePlayer(state, adapter)
      if (Math.abs(node.x - state.pos.x) <= 0.35 &&
          Math.abs(node.z - state.pos.z) <= 0.35 &&
          Math.abs(node.y - state.pos.y) < 1) return true
    }
    return false
  }
})

describe('executor: never sprint against a wall', () => {
  /**
   * The server refuses every position a SPRINTING client claims with its body
   * against a block, and teleports it back. Measured back to back on the same
   * block: walking along the wall covered 7.1 blocks with zero corrections,
   * sprinting covered 0.0 with 26 — and because the bot cannot move, it never
   * stops touching the wall.
   */
  function corridor (): VoxelWorld {
    const w = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 8, y1: 6, z1: 8 })
    w.fill(-8, 0, -8, 8, 0, 8, STONE) // floor, stand y = 1
    w.fill(-8, 1, -1, 8, 2, -1, STONE) // wall along z = -1
    return w
  }

  it('nearWall counts exact contact, which is where a collision leaves us', () => {
    const world = corridor()
    // Body half-width 0.3: at z = 0.3 the box edge is exactly on the wall face.
    const touching = makeFakeBot(world, new Vec3(0.5, 1, 0.3))
    const clear = makeFakeBot(world, new Vec3(0.5, 1, 0.6))
    expect(nearWall(touching as never)).to.equal(true)
    expect(nearWall(clear as never)).to.equal(false)
  })

  function drive (world: VoxelWorld, at: Vec3, goal: GoalBlock): DriveableBot {
    const bot = makeDriveableBot(world, at)
    const plugin = createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: { setGoal: (g: unknown) => void, setMovements: (m: Movements) => void } }).pathfinder
    pf.setMovements(makeOurMovements(bot))
    pf.setGoal(goal)
    return bot
  }

  it('is never both sprinting and against the wall', () => {
    // Started flush against the wall, so the very first decisions are made in
    // the state the server refuses. The invariant is what matters, not one
    // tick of it: the bot walks away from the wall as it goes, and the gate
    // has to hold for every tick in between.
    const world = corridor()
    const bot = drive(world, new Vec3(0.5, 1, 0.3), new GoalBlock(-6, 1, 0))
    let violations = 0
    let sprinted = 0
    for (let i = 0; i < 120; i++) {
      bot.tick()
      if (bot.controlState.sprint === true) {
        sprinted++
        if (nearWall(bot as never)) violations++
      }
    }
    expect(violations, 'sprinted with the body on a block').to.equal(0)
    expect(sprinted, 'and it does still sprint once it is clear').to.be.greaterThan(0)
  })

  it('sprints normally with room to spare', () => {
    const world = corridor()
    const bot = drive(world, new Vec3(0.5, 1, 3.5), new GoalBlock(-6, 1, 3))
    for (let i = 0; i < 40 && bot.controlState.forward !== true; i++) bot.tick()
    bot.tick()
    expect(bot.controlState.sprint).to.equal(true)
  })
})

describe('executor: wall-slide steering', () => {
  /**
   * Aiming straight at a node that sits past a corner presses the body into
   * the corner block. The physics clamps that move onto the block face, and a
   * clamped position that also slid along the wall is refused: on the arena
   * corner it produced 25 corrections and 0.01 blocks of travel, while
   * steering along the wall produced none and 7.1 blocks.
   */
  it('steers along a blocking corner instead of into it', () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 8, y1: 6, z1: 8 })
    world.fill(-8, 0, -8, 8, 0, 8, STONE)
    world.fill(0, 1, -1, 0, 2, -1, STONE) // the corner block, due north

    const bot = makeDriveableBot(world, new Vec3(0.5, 1, 0.3))
    const plugin = createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: { setGoal: (g: unknown) => void, setMovements: (m: Movements) => void } }).pathfinder
    pf.setMovements(makeOurMovements(bot))
    pf.setGoal(new GoalBlock(-1, 1, -1)) // diagonally past the corner

    for (let i = 0; i < 40 && bot.controlState.forward !== true; i++) bot.tick()
    bot.tick()
    // yaw = atan2(-dx, -dz): heading -z is what walks into the corner block.
    const heading = { x: -Math.sin(bot.entity.yaw), z: -Math.cos(bot.entity.yaw) }
    expect(heading.x, 'still heads for the node in x').to.be.lessThan(-0.5)
    expect(heading.z, 'no longer presses into the corner').to.be.greaterThan(-0.01)
  })

  it('leaves a step-up alone — that block is to be climbed, not avoided', () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 8, y1: 6, z1: 8 })
    world.fill(-8, 0, -8, 8, 0, 8, STONE)
    world.set(0, 1, -1, STONE) // a one-block step, and the node is on top of it

    const bot = makeDriveableBot(world, new Vec3(0.5, 1, 0.3))
    const plugin = createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: { setGoal: (g: unknown) => void, setMovements: (m: Movements) => void } }).pathfinder
    pf.setMovements(makeOurMovements(bot))
    pf.setGoal(new GoalBlock(0, 2, -1))

    for (let i = 0; i < 40 && bot.controlState.forward !== true; i++) bot.tick()
    bot.tick()
    const heading = { x: -Math.sin(bot.entity.yaw), z: -Math.cos(bot.entity.yaw) }
    expect(heading.z, 'still presses into the step it has to climb').to.be.lessThan(-0.9)
  })
})

describe('executor: the 1.21.x hitbox-precision fix', () => {
  /**
   * A body built from 0.3 / 1.8 comes to rest exactly on block boundaries
   * after every collision, and on 1.21.x the server's own sweep then computes
   * exactly 1.0, calls the move blocked and teleports the client back. It
   * presents as a pathfinder that cannot climb a one-block step: measured on
   * the arena's climb1 riser, a walking jump from 0.01 clearance produced 19
   * corrections and no movement with the stock dimensions, and zero
   * corrections and a clean climb with them nudged.
   */
  function physicsHolder (): { physics: { playerHalfWidth: number, playerHeight: number } } {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 4, y1: 6, z1: 4 })
    world.fill(-4, 0, -4, 4, 0, 4, STONE)
    const bot = makeDriveableBot(world, new Vec3(0.5, 1, 0.5))
    return bot as unknown as { physics: { playerHalfWidth: number, playerHeight: number } }
  }

  it('nudges the player dimensions off their exact boundaries on inject', () => {
    const bot = physicsHolder()
    bot.physics.playerHalfWidth = 0.3
    bot.physics.playerHeight = 1.8
    ;(createPathfinder({ useWorkerThreads: false }) as unknown as (b: unknown) => void)(bot)
    expect(bot.physics.playerHalfWidth).to.equal(0.30001)
    expect(bot.physics.playerHeight).to.equal(1.80001)
  })

  it('can be turned off, and never doubles up on an already-nudged bot', () => {
    const off = physicsHolder()
    off.physics.playerHalfWidth = 0.3
    ;(createPathfinder({ useWorkerThreads: false, hitboxPrecisionFix: false }) as unknown as (b: unknown) => void)(off)
    expect(off.physics.playerHalfWidth).to.equal(0.3)

    // An application that applies its own nudge (bulbastore does) must not be
    // nudged a second time — the guard is on the exact stock values.
    const already = physicsHolder()
    already.physics.playerHalfWidth = 0.30001
    already.physics.playerHeight = 1.80001
    ;(createPathfinder({ useWorkerThreads: false }) as unknown as (b: unknown) => void)(already)
    expect(already.physics.playerHalfWidth).to.equal(0.30001)
    expect(already.physics.playerHeight).to.equal(1.80001)
  })

  it('body probes measure the dimensions the physics actually uses', () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 8, y1: 6, z1: 8 })
    world.fill(-8, 0, -8, 8, 0, 8, STONE)
    world.fill(-8, 1, -1, 8, 2, -1, STONE) // wall along z = -1, face at z = 0

    // Exactly one half-width off the face. With the stock 0.3 the body only
    // touches; nudged, it overlaps — and playerCollides has to agree with
    // whichever one the engine is going to simulate.
    const bot = makeDriveableBot(world, new Vec3(0.5, 1, 0.3)) as unknown as {
      physics: { playerHalfWidth: number, playerHeight: number }
    }
    // playerCollides insets by 0.02, so it reports an overlap below
    // z = half - 0.02 — 0.28 stock, 0.28001 nudged. A probe between the two
    // answers differently depending on which body it measured, and it has to
    // be the one the engine will simulate.
    const between = 0.280005
    bot.physics.playerHalfWidth = 0.3
    expect(playerCollides(bot as never, 0.5, 1, between)).to.equal(false)
    bot.physics.playerHalfWidth = 0.30001
    expect(playerCollides(bot as never, 0.5, 1, between)).to.equal(true)
  })
})

describe('executor: rollout honesty after a lagback', () => {
  /**
   * mineflayer sets `entity.onGround = false` on every server position
   * correction, so a bot standing on solid ground reads as airborne for a
   * tick. The one-jump latch used to take that at face value: it latched
   * "airborne" before the sim had run a step, called the first simulated tick
   * a LANDING, and released the jump it was being asked about — so every jump
   * gate answered "no" for as long as the corrections kept coming, which is
   * exactly when the bot most needs one.
   */
  it('still approves a jump when onGround is falsely false', () => {
    const w = new VoxelWorld({ x0: -6, y0: -8, z0: -4, x1: 10, y1: 8, z1: 4 })
    w.set(0, 0, 0, STONE)
    w.set(-1, 0, 0, STONE)
    w.set(3, 0, 0, STONE)
    const node = { x: 3.5, y: 1, z: 0.5 }

    const honest = physicsBot(w, new Vec3(0.5, 1, 0.5)) as { entity: { onGround: boolean } }
    expect(new PhysicsSim(honest as never).canSprintJump([node])).to.equal(true)

    const lagged = physicsBot(w, new Vec3(0.5, 1, 0.5)) as { entity: { onGround: boolean } }
    lagged.entity.onGround = false // what a correction leaves behind
    expect(new PhysicsSim(lagged as never).canSprintJump([node])).to.equal(true)
  })

  it('still refuses a second jump once the rollout has actually landed', () => {
    // The one-jump rule is the reason the latch exists: a take-off that lands
    // short must not "reach" the node by bouncing on from wherever it came
    // down. Nudging onGround must not cost that.
    const world = new VoxelWorld({ x0: -6, y0: -8, z0: -4, x1: 10, y1: 8, z1: 4 })
    world.set(0, 0, 0, STONE)
    world.set(-1, 0, 0, STONE)
    world.set(3, 0, 0, STONE)
    world.fill(4, 0, 0, 4, 1, 0, STONE)
    const sim = new PhysicsSim(physicsBot(world, new Vec3(0.5, 1, 0.5)) as never)
    expect(sim.canSprintJump([{ x: 4.5, y: 2, z: 0.5 }])).to.equal(false)
  })
})

describe('executor: the sprint-hop gait', () => {
  /**
   * Holding jump while sprinting is the fastest way across open ground —
   * measured on the arena's own server at 6.97 blocks/s against 5.56 — but it
   * is also the easiest way to leave the ground somewhere there is nothing to
   * land on. The gait is therefore never taken on faith: both gaits are driven
   * down the SAME path for the same horizon and the faster one wins, provided
   * it keeps its feet.
   */
  function plain (mut?: (w: VoxelWorld) => void, moving = true): { sim: PhysicsSim, path: Array<{ x: number, y: number, z: number }> } {
    const w = new VoxelWorld({ x0: -6, y0: -4, z0: -6, x1: 6, y1: 12, z1: 40 })
    w.fill(-6, 0, -6, 6, 0, 40, STONE)
    mut?.(w)
    const bot = physicsBot(w, new Vec3(0.5, 1, 0.5)) as { entity: { velocity: Vec3 } }
    // At sprint speed, which is when the executor actually asks: the decision
    // is taken on a grounded tick mid-run, not from a standstill.
    if (moving) bot.entity.velocity = new Vec3(0, 0, 0.28)
    const sim = new PhysicsSim(bot as never)
    const path = []
    for (let z = 1; z <= 12; z++) path.push({ x: 0.5, y: 1, z: z + 0.5 })
    return { sim, path }
  }

  it('takes the hop across open flat ground', () => {
    const { sim, path } = plain()
    expect(sim.sprintHopBetter(path)).to.equal(true)
  })

  it('declines from a standstill — there is no boost to keep yet', () => {
    // Jumping before you have speed is slower than running up to it, which is
    // why players sprint first and start hopping once they are moving. The
    // comparison finds that on its own; nothing hard-codes it.
    const { sim, path } = plain(undefined, false)
    expect(sim.sprintHopBetter(path)).to.equal(false)
  })

  it('takes it under a 2-high roof too — the bonked arc is still faster', () => {
    // Bonking cuts the airtime but keeps the take-off boost: 6.47 blocks/s
    // against 5.56 sprinting, measured. A ceiling is not a reason to walk.
    const { sim, path } = plain(w => w.fill(-6, 3, -6, 6, 3, 40, STONE))
    expect(sim.sprintHopBetter(path)).to.equal(true)
  })

  it('refuses when the roof DROPS ahead — that is the one that jams', () => {
    // 3-high here (ceiling at y = 4), 2-high from z = 4 on (ceiling at y = 3).
    // Taking off under the high part puts the body at 1.25 exactly as the low
    // part arrives, and it stops dead in the air against it instead of
    // bonking cleanly off it.
    const { sim, path } = plain(w => {
      w.fill(-6, 4, -6, 6, 4, 3, STONE) // 3-high up to z = 3
      w.fill(-6, 3, 4, 6, 3, 40, STONE) // 2-high from z = 4 on
    })
    expect(sim.sprintHopBetter(path)).to.equal(false)
  })

  it('refuses over water or lava overhead — the planner never looks up there', () => {
    for (const hazard of [WATER, LAVA]) {
      const { sim, path } = plain(w => { w.set(0, 3, 5, hazard) })
      expect(sim.sprintHopBetter(path)).to.equal(false)
    }
  })

  it('refuses when the hop would leave the ground the path stays on', () => {
    // A one-block-wide causeway with void either side of z = 6: sprinting
    // stops at the edge, hopping sails off it.
    const { sim, path } = plain(w => {
      for (let z = 6; z <= 40; z++) for (let x = -6; x <= 6; x++) w.set(x, 0, z, AIR)
    })
    expect(sim.sprintHopBetter(path)).to.equal(false)
  })

  it('refuses on rising ground and on the last stretch', () => {
    const rising = plain()
    for (const [i, n] of rising.path.entries()) n.y = 1 + i * 0.5
    expect(rising.sim.sprintHopBetter(rising.path)).to.equal(false)

    const { sim, path } = plain()
    expect(sim.sprintHopBetter(path.slice(0, 2))).to.equal(false)
  })

  /**
   * Nothing in the executor knows what ice is, or what a potion is. Every
   * gate rolls the LIVE PlayerState forward, so block slipperiness and the
   * server's movementSpeed attribute are already in the answer — and the two
   * point opposite ways, which is why this is worth pinning:
   *
   *   surface       walk   sprint   sprint-hop
   *   stone         4.30   5.60     7.07
   *   ice           4.10   5.33     9.11   ← hop 29% up, sprint DOWN
   *   blue ice      4.31   5.60     9.19
   *   slime         3.20   4.16     7.89
   *   speed II      6.03   7.83     7.58   ← sprint now beats the hop
   */
  function surfaced (block: string, mut?: (bot: Record<string, unknown>) => void): {
    sim: PhysicsSim, path: Array<{ x: number, y: number, z: number }>
  } {
    const w = new VoxelWorld({ x0: -6, y0: -4, z0: -6, x1: 6, y1: 12, z1: 40 })
    w.fill(-6, 0, -6, 6, 0, 40, mcData.blocksByName[block].minStateId as number)
    const bot = physicsBot(w, new Vec3(0.5, 1, 0.5)) as { entity: { velocity: Vec3 } }
    bot.entity.velocity = new Vec3(0, 0, 0.28)
    mut?.(bot as unknown as Record<string, unknown>)
    const path = []
    for (let z = 1; z <= 12; z++) path.push({ x: 0.5, y: 1, z: z + 0.5 })
    return { sim: new PhysicsSim(bot as never), path }
  }

  it('takes the hop on ice, where it is worth far more than on stone', () => {
    for (const ice of ['ice', 'packed_ice', 'blue_ice']) {
      expect(surfaced(ice).sim.sprintHopBetter(surfaced(ice).path), ice).to.equal(true)
    }
  })

  it('declines the hop under Speed II, where plain sprinting is faster', () => {
    // Vanilla delivers a speed potion as a movementSpeed attribute modifier,
    // which prismarine-physics reads — so the comparison simply comes out the
    // other way and the bot keeps its feet. Nothing tests for a potion.
    const { sim, path } = surfaced('stone', bot => {
      const phys = (bot as { physics: { movementSpeedAttribute: string } }).physics
      ;(bot as { entity: { attributes: Record<string, unknown> } }).entity.attributes = {
        [phys.movementSpeedAttribute]: {
          value: 0.1,
          modifiers: [{ uuid: '00000000-0000-0000-0000-0000000000ff', amount: 0.4, operation: 2 }]
        }
      }
    })
    expect(sim.sprintHopBetter(path)).to.equal(false)
  })

  it('scores the hop with the cadence the executor actually drives', () => {
    // A held jump key cannot re-fire for 10 ticks; a pressed one re-fires the
    // tick the body lands. Under a 2-high roof the arc lands in 5, so the two
    // are different gaits — 6.50 blocks/s against 9.68 — and scoring the held
    // one while running the pressed one would price the wrong thing by half.
    // The check is that the rollout REALLY re-jumps: over 16 ticks under a
    // roof, a held gait fits one take-off and a pressed gait fits three.
    const { sim, path } = plain(w => w.fill(-6, 3, -6, 6, 3, 40, STONE))
    const follow = (sim as unknown as {
      followPath: (p: typeof path, jump: boolean, horizon: number) => { score: number } | null
    }).followPath.bind(sim)
    const hop = follow(path, true, 16)
    const run = follow(path, false, 16)
    expect(hop).to.not.equal(null)
    expect(run).to.not.equal(null)
    // Three nodes of lead over the same 16 ticks is the bonk cadence working;
    // a held key manages about one.
    expect((hop as { score: number }).score).to.be.greaterThan((run as { score: number }).score + 2)
  })
})

describe('executor: the low-ceiling gait (allowLowCeilingHop)', () => {
  /**
   * Off, a ceiling that drops anywhere in the comparison horizon vetoes the
   * take-off — safe, and also enough to switch the gait off for a whole
   * passage, because in a mostly-2-high tunnel every 3-high pocket has a low
   * section within six nodes. On, the veto spans the arc's own footprint
   * instead: high ground next to a low roof still refuses (that is the one
   * that jams), but standing UNDER the low roof hops.
   */
  function tunnel (
    mut: (w: VoxelWorld) => void,
    at = new Vec3(0.5, 1, 0.5)
  ): { sim: PhysicsSim, path: Array<{ x: number, y: number, z: number }> } {
    const w = new VoxelWorld({ x0: -6, y0: -4, z0: -6, x1: 6, y1: 12, z1: 40 })
    w.fill(-6, 0, -6, 6, 0, 40, STONE)
    mut(w)
    const bot = physicsBot(w, at) as { entity: { velocity: Vec3 } }
    bot.entity.velocity = new Vec3(0, 0, 0.28)
    const sim = new PhysicsSim(bot as never)
    const path = []
    for (let z = Math.floor(at.z) + 1; z <= Math.floor(at.z) + 12; z++) path.push({ x: 0.5, y: 1, z: z + 0.5 })
    return { sim, path }
  }

  /** 2-high everywhere except a 3-high pocket over z = 0..3. */
  const pocket = (w: VoxelWorld): void => {
    w.fill(-6, 3, -6, 6, 3, 40, STONE)
    for (let z = 0; z <= 3; z++) for (let x = -6; x <= 6; x++) w.set(x, 3, z, AIR)
    for (let z = 0; z <= 3; z++) for (let x = -6; x <= 6; x++) w.set(x, 4, z, STONE)
  }

  it('still refuses to take off from high ground into a low roof', () => {
    // Standing IN the pocket with the 2-high section one node away: the arc
    // would be at 1.25 exactly as the low part arrives. This is the case the
    // user called "slamming its head", and it is refused either way.
    const { sim, path } = tunnel(pocket, new Vec3(0.5, 1, 2.5))
    expect(sim.sprintHopBetter(path, true)).to.equal(false)
    expect(sim.sprintHopBetter(path, false)).to.equal(false)
  })

  it('hops once it is under the low roof, where the plain gait gives up', () => {
    // One node further on, under the 2-high section: the arc bonks at 0.2 and
    // meets nothing lower than what it is already under. The 6-node veto is
    // still looking at the pocket behind and the tunnel ahead and refusing.
    const { sim, path } = tunnel(pocket, new Vec3(0.5, 1, 5.5))
    expect(sim.sprintHopBetter(path, true)).to.equal(true)
  })

  it('leaves open ground exactly as it was', () => {
    const w = new VoxelWorld({ x0: -6, y0: -4, z0: -6, x1: 6, y1: 12, z1: 40 })
    w.fill(-6, 0, -6, 6, 0, 40, STONE)
    const bot = physicsBot(w, new Vec3(0.5, 1, 0.5)) as { entity: { velocity: Vec3 } }
    bot.entity.velocity = new Vec3(0, 0, 0.28)
    const sim = new PhysicsSim(bot as never)
    const path = []
    for (let z = 1; z <= 12; z++) path.push({ x: 0.5, y: 1, z: z + 0.5 })
    expect(sim.sprintHopBetter(path, true)).to.equal(sim.sprintHopBetter(path, false))
  })

  it('never overrides the overhead hazard scan', () => {
    for (const hazard of [WATER, LAVA]) {
      const { sim, path } = tunnel(w => { w.set(0, 3, 5, hazard) })
      expect(sim.sprintHopBetter(path, true)).to.equal(false)
    }
  })
})
