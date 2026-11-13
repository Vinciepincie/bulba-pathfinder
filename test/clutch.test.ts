// Planned clutch drops (movements.maxClutchDropDown): a drop past the fall budget onto a plain full top,
// landed with water placed just before touchdown. Both cores must agree edge for edge.
import { expect } from 'chai'
import { MoveGen, META_CLUTCH, META_DEEP } from '../src/moveGen.js'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { WasmSolver } from '../src/wasm/wasmSolver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { serializeGoal } from '../src/goalSerde.js'
import { GoalBlock } from '../src/goals.js'
import type { Goal } from '../src/goals.js'
import { Move } from '../src/move.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld,
  STONE, SLIME, STONE_SLAB_BOTTOM, LAVA, WATER, LADDER_DRY
} from './helpers/voxelWorld.js'

/** A ledge `d` stand-cells above a stone floor: start stands at y = d + 2, the floor's stand cell is y = 2. */
function cliff (d: number, floor = STONE): VoxelWorld {
  const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 4, y1: d + 6, z1: 2 })
  world.fill(1, 1, -2, 4, 1, 2, floor)
  world.fill(-2, 1 + d, -2, 0, 1 + d, 2, STONE)
  return world
}

function gen (world: VoxelWorld, overrides: Record<string, unknown> = {}): { gen: MoveGen, snap: ReturnType<typeof snapshotFromWorld> } {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot, { allowParkourTight: false, ...overrides })
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  return { gen: new MoveGen(snap, movements.toConfig(), null), snap }
}

function movesTo (ctx: ReturnType<typeof gen>, from: [number, number, number], to: [number, number, number]): Array<{ cost: number, meta: number }> {
  const { gen: g, snap } = ctx
  g.generate(from[0], from[1], from[2])
  const out: Array<{ cost: number, meta: number }> = []
  const target = snap.index(to[0], to[1], to[2])
  for (let i = 0; i < g.outCount; i++) if (g.outIdx[i] === target) out.push({ cost: g.outCost[i], meta: g.outMeta[i] })
  return out
}

describe('clutch drops: move generation', () => {
  it('plans no drop past maxDropDown by default', () => {
    expect(movesTo(gen(cliff(12)), [0, 14, 0], [1, 2, 0])).to.have.length(0)
  })

  it('plans a 12-block drop with maxClutchDropDown: flagged, one-way, and charged its height', () => {
    const drop = movesTo(gen(cliff(12), { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])
    expect(drop).to.have.length(1)
    expect(drop[0].meta).to.equal(META_CLUTCH | META_DEEP)
    // 1 for the step, 12 for the fall, 3 for the stop to take the water back.
    expect(drop[0].cost).to.equal(16)
  })

  it('leaves drops the budget allows alone, and does not spend a bucket on one that would not hurt', () => {
    const d3 = movesTo(gen(cliff(3), { maxClutchDropDown: 30 }), [0, 5, 0], [1, 2, 0])
    expect(d3).to.have.length(1)
    expect(d3[0].meta).to.equal(0)
    // A low-health budget of 2: a 3-block drop (support 4 down) still hurts nothing, so no clutch either.
    expect(movesTo(gen(cliff(3), { maxClutchDropDown: 30, maxDropDown: 2 }), [0, 5, 0], [1, 2, 0])).to.have.length(0)
  })

  it('stops at maxClutchDropDown, counted to the landing cell like a drop', () => {
    expect(movesTo(gen(cliff(12), { maxClutchDropDown: 12 }), [0, 14, 0], [1, 2, 0])).to.have.length(1)
    expect(movesTo(gen(cliff(12), { maxClutchDropDown: 11 }), [0, 14, 0], [1, 2, 0])).to.have.length(0)
  })

  it('needs a full top (no slab) and no lava beside the landing; water over slime still catches', () => {
    expect(movesTo(gen(cliff(12, STONE_SLAB_BOTTOM), { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])).to.have.length(0)
    expect(movesTo(gen(cliff(12, SLIME), { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])).to.have.length(1)
    const lava = cliff(12)
    lava.set(2, 2, 0, LAVA)
    expect(movesTo(gen(lava, { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])).to.have.length(0)
  })

  it('starts only from solid footing, not from a ladder or water', () => {
    const w = cliff(12)
    w.set(0, 13, 0, STONE) // a pillar the bot stands on at y 14...
    w.set(0, 14, 0, LADDER_DRY) // ...but in a ladder cell
    expect(movesTo(gen(w, { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])).to.have.length(0)
    const wet = cliff(12)
    wet.set(0, 14, 0, WATER)
    expect(movesTo(gen(wet, { maxClutchDropDown: 30 }), [0, 14, 0], [1, 2, 0])).to.have.length(0)
  })

  it('carries the flag to the executor (Move.clutch)', () => {
    const m = Move.fromRaw({ x: 1, y: 2, z: 0, cost: 16, parkour: false, useOne: null, clutch: true })
    expect(m.clutch).to.equal(true)
    expect(Move.fromRaw({ x: 1, y: 2, z: 0, cost: 1, parkour: false, useOne: null }).clutch).to.equal(false)
  })
})

function solveJs (world: VoxelWorld, goal: Goal, start: { x: number, y: number, z: number }, overrides: Record<string, unknown>, timeout = 30000): RawSolveResult {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot, overrides)
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  const descriptor = serializeGoal(goal)
  const evaluator = fastEvaluator(descriptor!) ?? new GoalAdapter(goal)
  const solver = new Solver(snap, movements.toConfig(), evaluator, start, { timeout, searchRadius: -1 }, null, null)
  let r = solver.compute(1e9)
  while (r.status === 'partial') r = solver.compute(1e9)
  return r
}

describe('clutch drops: search', () => {
  it('takes the drop when it is the only way down, and only with the bucket allowed', () => {
    const w = cliff(12)
    const goal = new GoalBlock(3, 2, 0)
    expect(solveJs(w, goal, { x: 0, y: 14, z: 0 }, {}).status).to.equal('noPath')
    const r = solveJs(w, goal, { x: 0, y: 14, z: 0 }, { maxClutchDropDown: 30 })
    expect(r.status).to.equal('success')
    expect(r.path.filter(n => n.clutch === true)).to.have.length(1)
  })

  it('prefers a staircase down when one exists', () => {
    // A ledge 8 up, and a one-wide staircase down along z = 2 that lands right by the goal.
    const w = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 12, y1: 14, z1: 2 })
    w.fill(-2, 9, -2, 0, 9, 2, STONE)
    w.fill(1, 1, -2, 12, 1, 2, STONE)
    for (let i = 1; i <= 7; i++) w.fill(i, 2, 2, i, 9 - i, 2, STONE)
    const r = solveJs(w, new GoalBlock(8, 2, 1), { x: 0, y: 10, z: 0 }, { maxClutchDropDown: 30 })
    expect(r.status).to.equal('success')
    expect(r.path.some(n => n.clutch === true)).to.equal(false)
  })
})

describe('clutch drops: wasm ↔ JS', function () {
  this.timeout(120000)
  let wasm: WasmSolver | null = null
  before(async function () {
    wasm = await WasmSolver.create()
    if (wasm === null) this.skip()
  })

  function both (world: VoxelWorld, goal: Goal, start: { x: number, y: number, z: number }, overrides: Record<string, unknown>): { js: RawSolveResult, wasm: RawSolveResult } {
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, overrides)
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const descriptor = serializeGoal(goal)!
    const evaluator = fastEvaluator(descriptor) ?? new GoalAdapter(goal)
    const solver = new Solver(snap, movements.toConfig(), evaluator, start, { timeout: 30000, searchRadius: -1 }, null, null)
    let js = solver.compute(1e9)
    while (js.status === 'partial') js = solver.compute(1e9)
    const w = wasm!.solve(
      { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: snap.states, special: snap.special, thin: snap.thin, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight, bannedEdges: null },
      movements.toConfig(), descriptor, start, null,
      { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
    )
    return { js, wasm: w }
  }

  const key = (r: RawSolveResult): string => r.path.map(n => `${n.x},${n.y},${n.z}${n.clutch === true ? 'K' : ''}${n.parkour ? 'P' : ''}`).join('|')

  function same (p: { js: RawSolveResult, wasm: RawSolveResult }, label: string): void {
    expect(p.wasm.status).to.equal(p.js.status, `${label}: status`)
    expect(Math.abs(p.wasm.cost - p.js.cost)).to.be.at.most(1e-9, `${label}: cost js=${p.js.cost} wasm=${p.wasm.cost}`)
    expect(key(p.wasm)).to.equal(key(p.js), `${label}: path`)
    expect(p.wasm.visitedNodes).to.equal(p.js.visitedNodes, `${label}: visited`)
  }

  it('agrees on the cliff course', () => {
    const p = both(cliff(12), new GoalBlock(3, 2, 0), { x: 0, y: 14, z: 0 }, { maxClutchDropDown: 30 })
    same(p, 'cliff')
    expect(key(p.js)).to.include('K')
  })

  function mulberry32 (seed: number): () => number {
    let a = seed >>> 0
    return () => {
      a = (a + 0x6D2B79F5) | 0
      let t = Math.imul(a ^ (a >>> 15), 1 | a)
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296
    }
  }

  /** Random pillars up to 20 high, with slabs, slime, water and lava sprinkled on the floor. */
  function pillars (seed: number): VoxelWorld {
    const r = mulberry32(seed)
    const w = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 15, y1: 26, z1: 15 })
    w.fill(0, 0, 0, 15, 0, 15, STONE)
    for (let x = 0; x <= 15; x++) {
      for (let z = 0; z <= 15; z++) {
        const h = r() < 0.35 ? Math.floor(r() * 20) : 0
        if (h > 0) w.fill(x, 1, z, x, h, z, STONE)
        const top = h + 1
        const k = r()
        if (k < 0.04) w.set(x, top, z, STONE_SLAB_BOTTOM)
        else if (k < 0.07) w.set(x, top - 1 < 1 ? 1 : top - 1, z, SLIME)
        else if (k < 0.09 && h === 0) w.set(x, 1, z, WATER)
        else if (k < 0.1 && h === 0) w.set(x, 1, z, LAVA)
      }
    }
    w.fill(0, 1, 0, 0, 22, 0, STONE) // a tall start pillar in the corner
    return w
  }

  const CASES = Number(process.env.PF_WASM_FUZZ_CASES ?? 20)
  for (let i = 0; i < CASES; i++) {
    const seed = 7100 + i
    it(`pillar world ${i} (seed ${seed}): clutch on and off identical`, () => {
      const w = pillars(seed)
      const goal = new GoalBlock(15, 1, 15)
      for (const overrides of [{}, { maxClutchDropDown: 30 }, { maxClutchDropDown: 30, allowParkourExtended: true }]) {
        const p = both(w, goal, { x: 0, y: 23, z: 0 }, overrides)
        same(p, `seed ${seed} ${JSON.stringify(overrides)}`)
      }
    })
  }

  it('a partial or timed-out path never ends past a clutch drop', () => {
    const r = solveJs(cliff(12), new GoalBlock(4, 30, 0), { x: 0, y: 14, z: 0 }, { maxClutchDropDown: 30 }, 1)
    expect(r.path.some(n => n.clutch === true)).to.equal(false)
  })
})
