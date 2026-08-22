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
import { nearWall } from '../src/geometry.js'
import { createPathfinder } from '../src/plugin.js'
import { GoalBlock } from '../src/goals.js'
import { Movements } from '../src/movements.js'
import {
  VoxelWorld, STONE, mcData, Block, TEST_VERSION, makeFakeBot, makeOurMovements
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
