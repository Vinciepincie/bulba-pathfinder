import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import type { Move } from '../src/move.js'
import { GoalBlock } from '../src/goals.js'
import { Movements } from '../src/movements.js'
import { VoxelWorld, STONE, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'

describe('live prefix from the program branch', () => {
  it('tries the prefix at a planned program\'s rest without a TDZ throw', function () {
    if (process.env.PF_LIVE_PREFIX === '0' || process.env.PF_TAKEOFF_KERNEL === '0' ||
        process.env.PF_JUMP_SOLVER === '0') this.skip()
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 24, y1: 8, z1: 24 })
    world.fill(-8, -1, -8, 24, -1, 24, STONE)
    const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
    bot.frozen = true
    let attempts = 0
    const physics = {
      ...makeFakePhysics(world, bot),
      canStraightLine: () => false,
      canSprintJump: () => false,
      canWalkJump: () => false,
      hopFrom: () => null,
      hopAnchored: () => null,
      hopPrefix: () => { attempts++; return null },
      hopLinedUp: () => null,
      hopLineUp: () => null,
      hopExhausted: false,
      solveJump: () => undefined
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
    const h = { fixed: true, x: 1, z: 0 }
    bot.on('path_update', (r: { path: Move[] }) => {
      // (a planned program, no live start: the program branch's rest search, then the prefix)
      r.path[0].program = { sx: 0.5, sy: 0, sz: 0.5, tx: 4, ty: 0, tz: 0, run: h, air1: h, air2: h, jumpAt: 0, turnAir: Infinity, sprint: true, ticks: 12, family: 'straight' } as never
    })
    const compute = Solver.prototype.compute
    Solver.prototype.compute = (): RawSolveResult => ({
      status: 'success',
      cancelled: false, cost: 8, time: 0, visitedNodes: 1, generatedNodes: 2,
      touchedChunks: [], boundaryLimited: false,
      path: [
        { x: 4, y: 0, z: 0, cost: 4, parkour: true, useOne: null },
        { x: 8, y: 0, z: 0, cost: 4, parkour: false, useOne: null }
      ]
    } as unknown as RawSolveResult)
    try {
      pf.setGoal(new GoalBlock(8, 0, 0))
      for (let i = 0; i < 4; i++) expect(() => bot.tick()).to.not.throw()
      expect(attempts, 'one prefix attempt from the program branch').to.equal(1)
    } finally {
      pf.setGoal(null)
      Solver.prototype.compute = compute
    }
  })
})

describe('a live prefix not yet taken off', () => {
  it('drops corrected prefixes and bounds retries across streamed paths', function () {
    if (process.env.PF_LIVE_PREFIX === '0' || process.env.PF_TAKEOFF_KERNEL === '0' ||
        process.env.PF_JUMP_SOLVER === '0') this.skip()
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 24, y1: 8, z1: 24 })
    world.fill(-8, -1, -8, 24, -1, 24, STONE)
    const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
    bot.frozen = true
    const h = { fixed: true, x: 1, z: 0 }
    let attempts = 0
    const physics = {
      ...makeFakePhysics(world, bot),
      canStraightLine: () => false,
      canSprintJump: () => false,
      canWalkJump: () => false,
      hopFrom: () => null,
      hopLinedUp: () => null,
      hopExhausted: false,
      solveJump: () => undefined,
      hopPrefix: () => {
        attempts++
        return { sx: 0.5, sy: 0, sz: 0.5, start: {}, yaw0: 0, tx: 1, ty: 0, tz: 0, run: h, air1: h, air2: h, jumpAt: 3, turnAir: Infinity, sprint: true, ticks: 12, family: 'straight', groundPrefix: [{ yaw: 0, sprint: false, sneak: false }, { yaw: 0, sprint: false, sneak: false }, { yaw: 0, sprint: false, sneak: false }] }
      }
    }
    bot.loadPlugin(createPathfinder({
      useWorkerThreads: false,
      physicsFactory: () => physics as never
    }) as unknown as (b: unknown) => void)
    const pf = bot.pathfinder as unknown as {
      setGoal: (g: GoalBlock | null) => void
      setMovements: (m: Movements) => void
    }
    pf.setMovements(applyProfile(new Movements(bot as never)))
    let head: Move | null = null
    bot.on('path_update', (r: { path: Move[] }) => {
      r.path[0].wOff = 1
      head = r.path[0]
    })
    const compute = Solver.prototype.compute
    Solver.prototype.compute = (): RawSolveResult => ({
      status: 'partial',
      cancelled: false, cost: 8, time: 0, visitedNodes: 1, generatedNodes: 2,
      touchedChunks: [], boundaryLimited: false,
      path: [
        // (close enough that a correction keeps the path: an off-path one resets to a fresh solve, which may search again)
        { x: 1, y: 0, z: 0, cost: 4, parkour: true, useOne: null },
        { x: 8, y: 0, z: 0, cost: 4, parkour: false, useOne: null }
      ]
    } as unknown as RawSolveResult)
    try {
      pf.setGoal(new GoalBlock(8, 0, 0))
      bot.tick()
      expect(attempts, 'the prefix found').to.equal(1)
      const n = head as unknown as Move
      expect(n.program?.groundPrefix, 'and installed').to.not.equal(undefined)
      bot.emit('forcedMove')
      expect(n.program ?? null, 'dropped by the correction').to.equal(null)
      for (let i = 0; i < 3; i++) {
        bot.tick()
        expect(head, 'a replacement partial').to.not.equal(n)
        expect(attempts, 'one correction retry per node').to.equal(
          process.env.PF_PREFIX_RETRY_CAP === '0' ? i + 2 : 2)
        const current = head as unknown as Move
        bot.emit('forcedMove')
        expect(current.program ?? null, 'no stale prefix retained').to.equal(null)
      }
    } finally {
      pf.setGoal(null)
      Solver.prototype.compute = compute
    }
  })
})

describe('live prefix attempts across streamed paths', () => {
  it('keeps a failed attempt without a carried program, then retries on a new solve', function () {
    if (process.env.PF_LIVE_PREFIX === '0' || process.env.PF_TAKEOFF_KERNEL === '0' ||
        process.env.PF_JUMP_SOLVER === '0') this.skip()
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 24, y1: 8, z1: 24 })
    world.fill(-8, -1, -8, 24, -1, 24, STONE)
    const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
    bot.frozen = true
    let attempts = 0
    let final = false
    const heads: Move[] = []
    const physics = {
      ...makeFakePhysics(world, bot),
      canStraightLine: () => false,
      canSprintJump: () => false,
      canWalkJump: () => false,
      hopFrom: () => null,
      hopPrefix: () => { attempts++; return null },
      hopLinedUp: () => null,
      hopExhausted: false,
      solveJump: () => undefined
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
      // mcc-6-2: a neo with no live flight keeps its failed retreat across partials.
      r.path[0].wOff = 1
      heads.push(r.path[0])
    })
    const compute = Solver.prototype.compute
    Solver.prototype.compute = (): RawSolveResult => ({
      status: final ? 'success' : 'partial',
      cancelled: false, cost: 8, time: 0, visitedNodes: 1, generatedNodes: 2,
      touchedChunks: [], boundaryLimited: false,
      path: [
        { x: 4, y: 0, z: 0, cost: 4, parkour: true, useOne: null },
        { x: 8, y: 0, z: 0, cost: 4, parkour: false, useOne: null }
      ]
    } as unknown as RawSolveResult)
    try {
      pf.setGoal(new GoalBlock(8, 0, 0))
      bot.tick()
      expect(attempts, 'first failed attempt').to.equal(1)
      const first = heads[heads.length - 1]
      bot.tick()
      expect(heads[heads.length - 1]).to.not.equal(first)
      expect(attempts, 'replacement partial').to.equal(1)
      final = true
      bot.tick()
      expect(attempts, 'replacement final').to.equal(1)
      final = false
      pf.setGoal(new GoalBlock(8, 0, 0))
      bot.tick()
      expect(attempts, 'new solve may try again').to.equal(2)
    } finally {
      pf.setGoal(null)
      Solver.prototype.compute = compute
    }
  })
})
