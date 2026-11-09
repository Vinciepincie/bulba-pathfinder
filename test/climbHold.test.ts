import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { CLIMB_HOLD_ENABLED } from '../src/gaitSearch.js'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import type { Move } from '../src/move.js'
import { GoalBlock } from '../src/goals.js'
import { Movements } from '../src/movements.js'
import { VoxelWorld, STONE, LADDER_DRY, VINE, mcData, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

function withClimb (state: number, wall: boolean, targetY: number, check: (bot: DriveableBot) => void, program?: Move['program']): void {
  const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 16, y1: 12, z1: 16 })
  world.fill(-8, -1, -8, 16, -1, 16, STONE)
  world.fill(0, 0, 0, 0, 3, 0, state)
  if (wall) world.fill(0, 0, 1, 0, 6, 1, STONE)
  const bot = makeDriveableBot(world, new Vec3(0.5, 1.2, 0.4))
  bot.frozen = true
  bot.entity.onGround = false
  bot.entity.velocity.y = -0.225
  const physics = {
    ...makeFakePhysics(world, bot),
    canStraightLine: () => true,
    canSprintJump: () => false,
    canWalkJump: () => false
  }
  bot.loadPlugin(createPathfinder({
    useWorkerThreads: false,
    physicsFactory: () => physics
  }) as unknown as (b: unknown) => void)
  const pf = bot.pathfinder as unknown as {
    setGoal: (g: GoalBlock | null) => void
    setMovements: (m: Movements) => void
  }
  pf.setMovements(applyProfile(new Movements(bot as never)))
  bot.on('path_update', (r: { path: Move[] }) => {
    Object.assign(r.path[0], { x: 0.5, y: targetY, z: 0.7, program })
  })
  const compute = Solver.prototype.compute
  Solver.prototype.compute = (): RawSolveResult => ({
    status: 'success',
    cancelled: false, cost: 8, time: 0, visitedNodes: 1, generatedNodes: 2,
    touchedChunks: [], boundaryLimited: false,
    path: [
      { x: 0, y: 4, z: 0, cost: 1, parkour: false, useOne: null },
      { x: 8, y: 4, z: 0, cost: 7, parkour: true, useOne: null }
    ]
  } as unknown as RawSolveResult)
  try {
    pf.setGoal(new GoalBlock(8, 4, 0))
    bot.tick()
    check(bot)
  } finally {
    pf.setGoal(null)
    Solver.prototype.compute = compute
  }
}

describe('executor climb hold', () => {
  const fixtures: Array<[string, number, boolean]> = [
    ['ladder', LADDER_DRY, true],
    ['wall vine', VINE, true],
    ['free vine', VINE, false],
    ['weeping vine', mcData.blocksByName.weeping_vines_plant.minStateId as number, false]
  ]
  for (const [name, state, wall] of fixtures) {
    it(`holds the first and later upward ticks on a ${name}`, () => {
      withClimb(state, wall, 4, bot => {
        for (const [y, vy] of [[1.2, -0.225], [2.2, 0.1176]]) {
          bot.entity.position.y = y
          bot.entity.velocity.y = vy
          for (let t = 0; t < 3; t++) {
            bot.tick()
            expect(bot.controlState.jump).to.equal(CLIMB_HOLD_ENABLED)
            expect(bot.controlState.sneak).to.equal(CLIMB_HOLD_ENABLED ? vy < 0 : !wall)
            expect(bot.controlState.forward).to.equal(true)
            expect(bot.controlState).to.include({ back: false, left: false, right: false })
            if (CLIMB_HOLD_ENABLED) expect(bot.controlState.sprint).to.equal(false)
          }
        }
      })
    })
  }

  it('lets go of the climb keys when a correction moves the body off the ladder and off the path', function () {
    if (!CLIMB_HOLD_ENABLED) this.skip()
    withClimb(LADDER_DRY, true, 4, bot => {
      bot.tick()
      expect(bot.controlState.jump, 'climbing').to.equal(true)
      bot.entity.position.x = 14.5
      bot.entity.position.y = 0
      bot.entity.position.z = 14.5
      bot.entity.onGround = true
      bot.entity.velocity.y = 0
      bot.emit('forcedMove')
      expect(bot.controlState).to.include({ jump: false, forward: false, sneak: false })
    })
  })

  it('releases jump on leaving the ladder top', () => {
    withClimb(LADDER_DRY, true, 4, bot => {
      bot.entity.position.y = 4.04
      bot.entity.velocity.y = 0.037
      bot.tick()
      expect(bot.controlState.jump).to.equal(false)
    })
  })

  it('lets a descending body slide', () => {
    withClimb(LADDER_DRY, true, 0, bot => {
      expect(bot.controlState).to.include({ forward: false, jump: false, sneak: false })
    })
  })

  it('excludes scaffolding', () => {
    withClimb(mcData.blocksByName.scaffolding.minStateId as number, false, 4, bot => {
      expect(bot.controlState).to.include({ jump: false, sneak: false })
    })
  })

  for (const sy of [3, 1.2]) {
    it(sy === 3 ? 'keeps prog-climb in charge' : 'keeps prog-hold in charge', () => {
      const h = { fixed: true, x: 0, z: 1 }
      const program = { sx: 0.5, sy, sz: 0.4, tx: 8, ty: 4, tz: 0, run: h, air1: h, air2: h, jumpAt: 0, turnAir: Infinity, sprint: false, ticks: 12, family: 'straight', climb: true }
      withClimb(LADDER_DRY, true, 4, bot => {
        expect(bot.controlState).to.include({ forward: sy === 3, jump: sy === 3 && CLIMB_HOLD_ENABLED, sprint: false, sneak: sy !== 3 || CLIMB_HOLD_ENABLED })
      }, program as never)
    })
  }
})
