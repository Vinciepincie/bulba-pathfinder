// canDig parity + executor tests. The dig cost model must match upstream
// mineflayer-pathfinder EXACTLY (same digTime source, same
// dontCreateFlow / dontMineUnderFallingBlock refusals), and the executor
// must actually break the blocks and walk through.
import { expect } from 'chai'
import { createRequire } from 'node:module'
import { Vec3 } from 'vec3'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { GoalBlock } from '../src/goals.js'
import type { Goal } from '../src/goals.js'
import { Movements } from '../src/movements.js'
import { createPathfinder } from '../src/plugin.js'
import { computeDigData } from '../src/digData.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld, applyProfile,
  AIR, STONE, DIRT, WATER, OAK_LEAVES, mcData
} from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'

const require2 = createRequire(import.meta.url)
/* eslint-disable @typescript-eslint/no-var-requires */
const UpstreamAStar = require2('mineflayer-pathfinder/lib/astar')
const UpstreamMovements = require2('mineflayer-pathfinder/lib/movements')
const UpstreamMove = require2('mineflayer-pathfinder/lib/move')
const upstreamGoals = require2('mineflayer-pathfinder/lib/goals')
/* eslint-enable @typescript-eslint/no-var-requires */

const SAND = mcData.blocksByName.sand.minStateId as number

interface UpResult {
  status: string
  cost: number
  path: Array<{ x: number, y: number, z: number, toBreak: Array<{ x: number, y: number, z: number }> }>
}

function solveUpstreamDig (world: VoxelWorld, start: { x: number, y: number, z: number }, goal: unknown): UpResult {
  const bot = makeFakeBot(world) as unknown as Record<string, unknown>
  bot.pathfinder = { bestHarvestTool: () => null }
  const movements = applyProfile(new UpstreamMovements(bot))
  movements.canDig = true
  const startMove = new UpstreamMove(start.x, start.y, start.z, movements.countScaffoldingItems(), 0)
  const astar = new UpstreamAStar(startMove, movements, goal, 4000, 1e9, -1)
  let result = astar.compute()
  while (result.status === 'partial') result = astar.compute()
  return result
}

function solveOursDig (world: VoxelWorld, start: { x: number, y: number, z: number }, goal: Goal): RawSolveResult {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot, { canDig: true })
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut, true)
  const dig = {
    data: computeDigData(bot as never, movements, lut),
    states: snap.allocStates(),
    breakExclusion: null
  }
  const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), start, {
    timeout: 4000,
    searchRadius: -1
  }, null, dig)
  let result = solver.compute(1e9)
  while (result.status === 'partial') result = solver.compute(1e9)
  return result
}

function differential (world: VoxelWorld, start: { x: number, y: number, z: number }, gx: number, gy: number, gz: number): { up: UpResult, ours: RawSolveResult } {
  const up = solveUpstreamDig(world, start, new upstreamGoals.GoalBlock(gx, gy, gz))
  const ours = solveOursDig(world, start, new GoalBlock(gx, gy, gz))
  expect(ours.status === 'success').to.equal(up.status === 'success',
    `reachability mismatch: upstream=${up.status} ours=${ours.status}`)
  if (up.status === 'success' && ours.status === 'success') {
    expect(Math.abs(ours.cost - up.cost)).to.be.at.most(1e-6,
      `dig cost mismatch: upstream=${up.cost} ours=${ours.cost}`)
  }
  return { up, ours }
}

/**
 * Every edge of our path must exist in upstream's getNeighbors (canDig
 * profile) with a cost ≤ ours + eps; the summed upstream costs must not beat
 * our total (which would mean we OVERPRICE). Proves our cheaper totals are
 * search-quality wins, not a divergent edge model.
 */
function assertPathRealizableUpstream (world: VoxelWorld, start: { x: number, y: number, z: number }, ours: RawSolveResult, seed: number): void {
  const bot = makeFakeBot(world) as unknown as Record<string, unknown>
  bot.pathfinder = { bestHarvestTool: () => null }
  const movements = applyProfile(new UpstreamMovements(bot))
  movements.canDig = true

  let prev = start
  let upstreamSum = 0
  for (const node of ours.path) {
    const neighbors = movements.getNeighbors(new UpstreamMove(prev.x, prev.y, prev.z, 0, 0)) as Array<{ x: number, y: number, z: number, cost: number }>
    const matches = neighbors.filter(n => n.x === node.x && n.y === node.y && n.z === node.z)
    expect(matches.length, `seed ${seed}: our edge (${prev.x},${prev.y},${prev.z})→(${node.x},${node.y},${node.z}) does not exist upstream`).to.be.greaterThan(0)
    const bestUp = Math.min(...matches.map(m => m.cost))
    expect(bestUp).to.be.at.most(node.cost + 1e-6,
      `seed ${seed}: our edge to (${node.x},${node.y},${node.z}) is cheaper than upstream's best (${node.cost} < ${bestUp})`)
    upstreamSum += bestUp
    prev = node
  }
  expect(upstreamSum).to.be.at.most(ours.cost + 1e-6,
    `seed ${seed}: replaying our path upstream costs ${upstreamSum} > our total ${ours.cost}`)
}

describe('canDig', function () {
  this.timeout(60000)

  describe('solver dig scenes (exact upstream cost parity)', () => {
    it('tunnels through a 2-high stone wall, breaking exactly the body cells', () => {
      const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 12, y1: 8, z1: 2 })
      world.fill(-2, -1, -2, 12, -1, 2, STONE)
      world.fill(5, 0, -2, 5, 3, 2, STONE) // wall across z
      const { ours } = differential(world, { x: 0, y: 0, z: 0 }, 10, 0, 0)
      expect(ours.status).to.equal('success')
      const breaks = ours.path.flatMap(n => n.toBreak ?? [])
      expect(breaks.length).to.be.greaterThan(0)
      for (const b of breaks) expect(b.x).to.equal(5) // only wall cells dug
    })

    it('digs straight down through dirt', () => {
      const world = new VoxelWorld({ x0: -3, y0: -8, z0: -3, x1: 3, y1: 6, z1: 3 })
      world.fill(-3, -6, -3, 3, -1, 3, DIRT) // 6 deep dirt slab
      // goal: stand 3 below the surface, straight down
      world.set(0, -4, 0, DIRT) // ensure support below goal cell after dig
      const { ours } = differential(world, { x: 0, y: 0, z: 0 }, 0, -3, 0)
      expect(ours.status).to.equal('success')
      const breaks = ours.path.flatMap(n => n.toBreak ?? [])
      expect(breaks.length).to.be.greaterThan(0)
    })

    it('dontCreateFlow: refuses to dig a wall cell adjacent to water (parity)', () => {
      const world = new VoxelWorld({ x0: -2, y0: -2, z0: -4, x1: 10, y1: 8, z1: 4 })
      world.fill(-2, -1, -4, 10, -1, 4, STONE)
      world.fill(5, 0, -4, 5, 3, 4, STONE) // wall
      // Water hugging the wall's feet cell on the far side at every z — the
      // z-row the path would tunnel through has water next to (5,0,z).
      for (let z = -4; z <= 4; z++) {
        if (world.stateAt(6, 0, z) === AIR) world.set(6, 0, z, WATER)
      }
      differential(world, { x: 0, y: 0, z: 0 }, 9, 0, 0)
    })

    it('dontMineUnderFallingBlock: refuses to dig under sand (parity)', () => {
      const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 10, y1: 8, z1: 2 })
      world.fill(-2, -1, -2, 10, -1, 2, STONE)
      world.fill(5, 0, -2, 5, 1, 2, STONE) // 2-high wall
      world.fill(5, 2, -2, 5, 2, 2, SAND) // sand resting on the wall
      differential(world, { x: 0, y: 0, z: 0 }, 9, 0, 0)
    })

    it('prefers walking around a short wall over digging when cheaper (parity)', () => {
      const world = new VoxelWorld({ x0: -6, y0: -2, z0: -6, x1: 12, y1: 8, z1: 6 })
      world.fill(-6, -1, -6, 12, -1, 6, STONE)
      world.fill(5, 0, -1, 5, 2, 1, STONE) // short wall, easy to walk around
      const { ours } = differential(world, { x: 0, y: 0, z: 0 }, 9, 0, 0)
      const breaks = ours.path.flatMap(n => n.toBreak ?? [])
      expect(breaks.length).to.equal(0) // walking around beats stone dig time
    })

    it('walk-only profile in the same worlds stays noPath where digging succeeds', () => {
      const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 12, y1: 8, z1: 2 })
      world.fill(-2, -1, -2, 12, -1, 2, STONE)
      world.fill(5, 0, -2, 5, 3, 2, STONE)
      const bot = makeFakeBot(world)
      const movements = makeOurMovements(bot) // canDig false
      const lut = lutFor(bot, movements)
      const snap = snapshotFromWorld(world, lut)
      const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(new GoalBlock(10, 0, 0)), { x: 0, y: 0, z: 0 }, { timeout: 4000, searchRadius: -1 })
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      expect(r.status).to.equal('noPath')
    })
  })

  describe('randomized dig differential vs upstream', () => {
    function mulberry32 (seed: number): () => number {
      let a = seed >>> 0
      return function () {
        a |= 0
        a = (a + 0x6D2B79F5) | 0
        let t = Math.imul(a ^ (a >>> 15), 1 | a)
        t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296
      }
    }

    const CASES = Number(process.env.PF_DIG_FUZZ_CASES ?? 25)
    for (let i = 0; i < CASES; i++) {
      const seed = 9000 + i
      it(`dig fuzz case ${i} (seed ${seed})`, () => {
        const rand = mulberry32(seed)
        const world = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 18, y1: 10, z1: 18 })
        world.fill(0, 0, 0, 18, 0, 18, STONE)
        // Dense terrain: walls + columns + leaf blobs + a water puddle.
        const nWalls = 1 + Math.floor(rand() * 2)
        for (let w = 0; w < nWalls; w++) {
          const alongX = rand() < 0.5
          const fixed = 3 + Math.floor(rand() * 13)
          const h = 2 + Math.floor(rand() * 2)
          for (let a = 0; a <= 18; a++) {
            if (alongX) world.fill(a, 1, fixed, a, h, fixed, STONE)
            else world.fill(fixed, 1, a, fixed, h, a, STONE)
          }
        }
        for (let x = 0; x <= 18; x++) {
          for (let z = 0; z <= 18; z++) {
            const r = rand()
            if (r < 0.08) world.fill(x, 1, z, x, 1 + Math.floor(rand() * 2), z, rand() < 0.5 ? STONE : DIRT)
            else if (r < 0.12) world.set(x, 1, z, OAK_LEAVES)
            else if (r < 0.14 && world.stateAt(x, 1, z) === AIR) world.set(x, 1, z, WATER)
          }
        }
        // Clear start pocket + standable goal.
        world.fill(0, 1, 0, 2, 3, 2, AIR)
        const gx = 5 + Math.floor(rand() * 13)
        const gz = 5 + Math.floor(rand() * 13)
        world.set(gx, 0, gz, STONE)
        world.fill(gx, 1, gz, gx, 2, gz, AIR)

        const up = solveUpstreamDig(world, { x: 1, y: 1, z: 1 }, new upstreamGoals.GoalBlock(gx, 1, gz))
        const ours = solveOursDig(world, { x: 1, y: 1, z: 1 }, new GoalBlock(gx, 1, gz))
        if (up.status === 'timeout' || ours.status === 'timeout') return // rare; not comparable
        expect(ours.status === 'success').to.equal(up.status === 'success',
          `reachability mismatch (seed ${seed}): upstream=${up.status} ours=${ours.status}`)
        if (up.status === 'success' && ours.status === 'success') {
          // Both engines share the first-goal-pop approximation; our
          // reopening usually keeps us at-or-under. Within the mutual 15%
          // band → accept; outside it, ARBITRATE: our (cheaper) path must be
          // realizable edge-by-edge in UPSTREAM's own move generator with
          // identical costs — that pins any real edge-model divergence.
          const inBand = ours.cost <= up.cost * 1.15 + 1e-6 && up.cost <= ours.cost * 1.15 + 1e-6
          if (!inBand) {
            expect(ours.cost).to.be.at.most(up.cost + 1e-6,
              `ours much COSTLIER (seed ${seed}): upstream=${up.cost} ours=${ours.cost}`)
            assertPathRealizableUpstream(world, { x: 1, y: 1, z: 1 }, ours, seed)
          }
        }
        // Determinism.
        const again = solveOursDig(world, { x: 1, y: 1, z: 1 }, new GoalBlock(gx, 1, gz))
        expect(again.cost).to.equal(ours.cost)
        expect(again.visitedNodes).to.equal(ours.visitedNodes)
      })
    }
  })

  describe('executor', () => {
    it('digs through a wall and reaches the goal (isMining true while breaking)', async () => {
      const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 20, y1: 8, z1: 8 })
      world.fill(-8, -1, -8, 20, -1, 8, STONE)
      world.fill(6, 0, -8, 6, 4, 8, STONE) // full wall — no way around

      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      const plugin = createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) })
      bot.loadPlugin(plugin as unknown as (b: unknown) => void)
      const pf = (bot as unknown as { pathfinder: Record<string, any> }).pathfinder
      const movements = applyProfile(new Movements(bot as never) as never) as unknown as Movements
      movements.canDig = true
      pf.setMovements(movements)

      let sawMining = false
      let settled: string | null = null
      pf.goto(new GoalBlock(12, 0, 0)).then(() => { settled = 'resolved' }, (e: Error) => { settled = `rejected:${e.name}` })
      for (let i = 0; i < 4000 && !settled; i++) {
        bot.tick()
        if (pf.isMining()) sawMining = true
        await new Promise(resolve => setImmediate(resolve))
        await new Promise(resolve => setTimeout(resolve, 1))
      }

      expect(settled).to.equal('resolved')
      expect(bot.digs.length).to.be.greaterThan(0)
      expect(sawMining).to.equal(true)
      expect(bot.entity.position.distanceTo(new Vec3(12.5, 0, 0.5))).to.be.lessThan(1.5)
      // The dug cells really opened.
      for (const d of bot.digs) expect(world.stateAt(d.x, d.y, d.z)).to.equal(AIR)
    })
  })
})
