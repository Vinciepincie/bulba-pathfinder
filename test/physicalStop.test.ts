import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { GoalBlock } from '../src/goals.js'
import { VoxelWorld, STONE, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'

describe('physical stops', () => {
  for (const allowStopSnap of [false, true]) {
    it(allowStopSnap ? 'preserves the default stop behavior' : 'releases controls without rewriting position or momentum', () => {
      const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 8, y1: 8, z1: 8 })
      world.fill(-8, -1, -8, 8, -1, 8, STONE)
      const bot = makeDriveableBot(world, new Vec3(0.95, 0, 0.95))
      bot.loadPlugin(createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) }) as never)
      const pf = bot.pathfinder as any
      const moves = applyProfile(new Movements(bot as never) as never) as unknown as Movements
      if (!allowStopSnap) (moves as any).allowStopSnap = false
      pf.setMovements(moves)
      pf.setGoal(new GoalBlock(5, 0, 5))
      bot.entity.position.set(0.95, 0, 0.95)
      bot.entity.velocity.set(0.2, 0, 0.1)
      bot.setControlState('forward', true)
      bot.setControlState('jump', true)
      bot.setControlState('sprint', true)
      pf.stop()
      // Execute the stop without advancing fake physics: detect direct state writes.
      bot.emit('physicsTick')
      expect(Object.values(bot.controlState).some(Boolean)).to.equal(false)
      expect(bot.entity.position.toArray()).to.deep.equal(allowStopSnap ? [0.5, 0, 0.5] : [0.95, 0, 0.95])
      expect(bot.entity.velocity.toArray()).to.deep.equal(allowStopSnap ? [0, 0, 0] : [0.2, 0, 0.1])
      expect(pf.isMoving()).to.equal(false)
    })
  }
})
