// wasm core ↔ JS solver differential: the Rust port must produce IDENTICAL
// statuses, costs, paths, toBreak lists and visited counts on identical
// inputs — the JS solver is the reference implementation. Runs whenever the
// embedded wasm payload instantiates (skips otherwise).
import { expect } from 'chai'
import { WasmSolver } from '../src/wasm/wasmSolver.js'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { serializeGoal } from '../src/goalSerde.js'
import { GoalBlock, GoalNear, GoalXZ, GoalY, GoalGetToBlock, GoalCompositeAny } from '../src/goals.js'
import type { Goal } from '../src/goals.js'
import { computeDigData } from '../src/digData.js'
import { applySnapshotBlockUpdate } from '../src/snapshot.js'
import type { DigData } from '../src/types.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld,
  AIR, STONE, DIRT, WATER, OAK_LEAVES, LADDER
} from './helpers/voxelWorld.js'

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

function genWorld (seed: number): VoxelWorld {
  const rand = mulberry32(seed)
  const world = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 22, y1: 12, z1: 22 })
  world.fill(0, 0, 0, 22, 0, 22, STONE)
  for (let x = 0; x <= 22; x++) {
    for (let z = 0; z <= 22; z++) {
      const r = rand()
      if (r < 0.14) world.fill(x, 1, z, x, 1 + Math.floor(rand() * 3), z, rand() < 0.6 ? STONE : DIRT)
      else if (r < 0.18) world.set(x, 1, z, OAK_LEAVES)
      else if (r < 0.2 && world.stateAt(x, 1, z) === AIR) world.set(x, 1, z, WATER)
      else if (r < 0.21) world.set(x, 1, z, LADDER)
    }
  }
  world.fill(0, 1, 0, 2, 3, 2, AIR)
  return world
}

interface Pair {
  js: RawSolveResult
  wasm: RawSolveResult
}

function pathKey (r: RawSolveResult): string {
  return r.path.map(n =>
    `${n.x},${n.y},${n.z},${n.parkour ? 1 : 0},${n.useOne ? 1 : 0},[${(n.toBreak ?? []).map(b => `${b.x},${b.y},${b.z}`).join(';')}]`
  ).join('|')
}

describe('wasm ↔ JS solver differential', function () {
  this.timeout(120000)
  let wasm: WasmSolver | null = null

  before(async function () {
    wasm = await WasmSolver.create()
    if (!wasm) {
      console.warn('[wasm.test] SKIPPING — wasm payload missing or failed to instantiate')
      this.skip()
    }
  })

  function solveBoth (world: VoxelWorld, goal: Goal, opts: { canDig?: boolean, doors?: boolean, searchRadius?: number, parkourExtended?: boolean } = {}): Pair {
    const bot = makeFakeBot(world)
    const overrides: Record<string, unknown> = {}
    if (opts.canDig) overrides.canDig = true
    if (opts.parkourExtended) overrides.allowParkourExtended = true
    if (opts.doors) {
      overrides.canOpenDoors = true
      overrides.canOpenRealDoors = true
    }
    const movements = makeOurMovements(bot, overrides)
    if (opts.doors) (movements as { canOpenRealDoors: boolean }).canOpenRealDoors = true
    const lut = lutFor(bot, movements)
    const needStates = Boolean(opts.canDig)
    const snap = snapshotFromWorld(world, lut, needStates)
    const digData = opts.canDig ? computeDigData(bot as never, movements, lut) : null
    const digCtx = opts.canDig
      ? { data: digData!, states: snap.allocStates(), breakExclusion: null }
      : null
    const searchRadius = opts.searchRadius ?? -1

    const descriptor = serializeGoal(goal)
    expect(descriptor).to.not.equal(null)
    const evaluator = fastEvaluator(descriptor!) ?? new GoalAdapter(goal)
    const solver = new Solver(snap, movements.toConfig(), evaluator, { x: 1, y: 1, z: 1 }, {
      timeout: 30000,
      searchRadius
    }, null, digCtx)
    let js = solver.compute(1e9)
    while (js.status === 'partial') js = solver.compute(1e9)

    const wasmResult = wasm!.solve(
      {
        meta: snap.meta,
        flags: snap.flags,
        heights: snap.heights,
        states: snap.states,
        entityIdx: snap.entityIdx,
        entityWeight: snap.entityWeight
      },
      movements.toConfig(),
      descriptor!,
      { x: 1, y: 1, z: 1 },
      digData,
      { timeout: 30000, searchRadius, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
    )
    return { js, wasm: wasmResult }
  }

  function assertIdentical (p: Pair, label: string): void {
    expect(p.wasm.status).to.equal(p.js.status, `${label}: status`)
    expect(Math.abs(p.wasm.cost - p.js.cost)).to.be.at.most(1e-9, `${label}: cost js=${p.js.cost} wasm=${p.wasm.cost}`)
    expect(pathKey(p.wasm)).to.equal(pathKey(p.js), `${label}: path`)
    expect(p.wasm.visitedNodes).to.equal(p.js.visitedNodes, `${label}: visitedNodes`)
    expect(p.wasm.boundaryLimited).to.equal(p.js.boundaryLimited, `${label}: boundaryLimited`)
    const chunkSet = (r: RawSolveResult): string => r.touchedChunks.map(c => c.join(',')).sort().join(';')
    expect(chunkSet(p.wasm)).to.equal(chunkSet(p.js), `${label}: touchedChunks`)
  }

  const CASES = Number(process.env.PF_WASM_FUZZ_CASES ?? 40)
  for (let i = 0; i < CASES; i++) {
    const seed = 5000 + i
    it(`walk world ${i} (seed ${seed}) — GoalBlock identical`, () => {
      const world = genWorld(seed)
      const rand = mulberry32(seed * 7 + 1)
      const gx = 4 + Math.floor(rand() * 18)
      const gz = 4 + Math.floor(rand() * 18)
      world.set(gx, 0, gz, STONE)
      world.fill(gx, 1, gz, gx, 2, gz, AIR)
      assertIdentical(solveBoth(world, new GoalBlock(gx, 1, gz)), `seed ${seed}`)
    })
  }

  for (let i = 0; i < Math.max(10, CASES / 2); i++) {
    const seed = 6000 + i
    it(`dig world ${i} (seed ${seed}) — canDig identical incl. toBreak`, () => {
      const world = genWorld(seed)
      const rand = mulberry32(seed * 13 + 5)
      const gx = 4 + Math.floor(rand() * 18)
      const gz = 4 + Math.floor(rand() * 18)
      world.set(gx, 0, gz, STONE)
      world.fill(gx, 1, gz, gx, 2, gz, AIR)
      assertIdentical(solveBoth(world, new GoalBlock(gx, 1, gz), { canDig: true }), `seed ${seed}`)
    })
  }

  it('extended parkour course identical (diag jumps, up, drop, ladder catch)', () => {
    // Void-gapped course: knight (2,1), full (2,2), up-jump (+1), drop-jump
    // (-1), then a cardinal gap-jump to a wall ladder. y0 sits below the
    // pillars (the landing scan cannot land on the world-box bottom layer).
    const world = new VoxelWorld({ x0: 0, y0: -2, z0: 0, x1: 12, y1: 8, z1: 12 })
    world.set(1, 0, 1, STONE) // stand y=1
    world.set(3, 0, 2, STONE) // knight (2,1)
    world.set(5, 0, 4, STONE) // full (2,2)
    world.set(7, 1, 5, STONE) // up-jump (2,1): flight-level block, stand y=2
    world.set(9, 0, 6, STONE) // drop-jump (2,1) back down to y=1
    world.set(9, 1, 8, LADDER) // cardinal (0,2) jump into the ladder...
    world.set(9, 2, 8, LADDER)
    world.fill(9, 1, 9, 9, 2, 9, STONE) // ...mounted on this wall
    const pair = solveBoth(world, new GoalBlock(9, 2, 8), { parkourExtended: true })
    expect(pair.js.status).to.equal('success')
    expect(pair.js.path.filter(n => n.parkour).length).to.be.at.least(4)
    assertIdentical(pair, 'ext course')
    // Flag off: both must agree it is unreachable.
    assertIdentical(solveBoth(world, new GoalBlock(9, 2, 8)), 'ext course flag off')
  })

  const EXT_CASES = Math.max(10, CASES / 2)
  for (let i = 0; i < EXT_CASES; i++) {
    const seed = 8000 + i
    it(`walk world ${i} (seed ${seed}) — allowParkourExtended identical`, () => {
      const world = genWorld(seed)
      const rand = mulberry32(seed * 11 + 3)
      const gx = 4 + Math.floor(rand() * 18)
      const gz = 4 + Math.floor(rand() * 18)
      world.set(gx, 0, gz, STONE)
      world.fill(gx, 1, gz, gx, 2, gz, AIR)
      assertIdentical(solveBoth(world, new GoalBlock(gx, 1, gz), { parkourExtended: true }), `seed ${seed}`)
    })
  }

  it('goal-type coverage: Near / XZ / Y / GetToBlock identical', () => {
    const world = genWorld(4242)
    world.set(18, 0, 18, STONE)
    world.fill(18, 1, 18, 18, 2, 18, AIR)
    assertIdentical(solveBoth(world, new GoalNear(18, 1, 18, 2)), 'near')
    assertIdentical(solveBoth(world, new GoalXZ(18, 18)), 'xz')
    assertIdentical(solveBoth(world, new GoalY(2)), 'y')
    assertIdentical(solveBoth(world, new GoalGetToBlock(18, 1, 18)), 'getToBlock')
  })

  it('searchRadius slack pruning identical', () => {
    const world = genWorld(999)
    world.set(20, 0, 20, STONE)
    world.fill(20, 1, 20, 20, 2, 20, AIR)
    assertIdentical(solveBoth(world, new GoalBlock(20, 1, 20), { searchRadius: 10 }), 'slack10')
    assertIdentical(solveBoth(world, new GoalBlock(20, 1, 20), { searchRadius: 64 }), 'slack64')
  })

  it('GoalCompositeAny of coordinate goals identical (incl. nested)', () => {
    const world = genWorld(7777)
    world.set(20, 0, 20, STONE)
    world.fill(20, 1, 20, 20, 2, 20, AIR)
    world.set(4, 0, 20, STONE)
    world.fill(4, 1, 20, 4, 2, 20, AIR)
    const flat = new GoalCompositeAny([new GoalBlock(20, 1, 20), new GoalNear(4, 1, 20, 2)])
    assertIdentical(solveBoth(world, flat), 'compositeAny flat')
    const nested = new GoalCompositeAny([
      new GoalCompositeAny([new GoalBlock(20, 1, 20), new GoalXZ(4, 20)]),
      new GoalGetToBlock(4, 1, 20)
    ])
    assertIdentical(solveBoth(world, nested), 'compositeAny nested')
  })

  it('snapshot residency: repeat solve reuses upload; patches invalidate it', () => {
    const world = genWorld(1234)
    world.set(20, 0, 20, STONE)
    world.fill(20, 1, 20, 20, 2, 20, AIR)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot)
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const descriptor = serializeGoal(new GoalBlock(20, 1, 20))!
    const opts = { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
    const solveWasm = (): RawSolveResult => wasm!.solve(
      { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: null, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
      movements.toConfig(), descriptor, { x: 1, y: 1, z: 1 }, null, opts
    )
    const solveJs = (): RawSolveResult => {
      const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, { x: 1, y: 1, z: 1 }, { timeout: 30000, searchRadius: -1 }, null, null)
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      return r
    }

    const r1 = solveWasm()
    assertIdentical({ js: solveJs(), wasm: r1 }, 'residency solve 1')
    const r2 = solveWasm() // same (generation, patchCount) → resident grids reused
    expect(pathKey(r2)).to.equal(pathKey(r1))
    expect(r2.visitedNodes).to.equal(r1.visitedNodes)
    expect(r2.cost).to.equal(r1.cost)

    // Patch a wall in-place across z=10 (one gap at x=2): patchCount bumps,
    // so the resident copy must be refreshed — a stale reuse would keep
    // walking straight through where the wall now stands.
    for (let x = 0; x <= 22; x++) {
      if (x === 2) continue
      for (let y = 1; y <= 4; y++) applySnapshotBlockUpdate(snap, lut, x, y, 10, STONE)
    }
    // Keep the gap and its approach walkable whatever the seed grew there.
    for (let z = 9; z <= 11; z++) {
      applySnapshotBlockUpdate(snap, lut, 2, 0, z, STONE)
      for (let y = 1; y <= 4; y++) applySnapshotBlockUpdate(snap, lut, 2, y, z, AIR)
    }
    const r3 = solveWasm()
    assertIdentical({ js: solveJs(), wasm: r3 }, 'residency after patches')
    expect(pathKey(r3)).to.not.equal(pathKey(r1))
    expect(r3.path.some(n => n.x === 2 && n.z === 10)).to.equal(true, 'patched path must use the x=2 gap')
  })

  it('snapshot residency: alternating snapshots re-upload correctly', () => {
    const seeds = [3100, 3200]
    const scenes = seeds.map(seed => {
      const world = genWorld(seed)
      world.set(18, 0, 18, STONE)
      world.fill(18, 1, 18, 18, 2, 18, AIR)
      const bot = makeFakeBot(world)
      const movements = makeOurMovements(bot)
      const lut = lutFor(bot, movements)
      const snap = snapshotFromWorld(world, lut)
      return { movements, snap }
    })
    const descriptor = serializeGoal(new GoalBlock(18, 1, 18))!
    const opts = { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
    for (const pick of [0, 1, 0, 1]) {
      const { movements, snap } = scenes[pick]
      const wasmResult = wasm!.solve(
        { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: null, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
        movements.toConfig(), descriptor, { x: 1, y: 1, z: 1 }, null, opts
      )
      const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, { x: 1, y: 1, z: 1 }, { timeout: 30000, searchRadius: -1 }, null, null)
      let js = solver.compute(1e9)
      while (js.status === 'partial') js = solver.compute(1e9)
      assertIdentical({ js, wasm: wasmResult }, `alternate scene ${pick}`)
    }
  })

  it('dig residency: fingerprint switch swaps tables, revert restores them', () => {
    // Flat floor with a full-height wall: the only way through is digging.
    const world = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 16, y1: 8, z1: 16 })
    world.fill(0, 0, 0, 16, 0, 16, STONE)
    world.fill(8, 1, 0, 8, 6, 16, STONE)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, { canDig: true })
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut, true)
    const digData = computeDigData(bot as never, movements, lut)
    const descriptor = serializeGoal(new GoalBlock(14, 1, 8))!
    const opts = { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }

    const solveWasm = (dig: DigData): RawSolveResult => wasm!.solve(
      { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: snap.states, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
      movements.toConfig(), descriptor, { x: 1, y: 1, z: 1 }, dig, opts
    )
    const solveJs = (dig: DigData): RawSolveResult => {
      const digCtx = { data: dig, states: snap.states!, breakExclusion: null }
      const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, { x: 1, y: 1, z: 1 }, { timeout: 30000, searchRadius: -1 }, null, digCtx)
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      return r
    }

    const r1 = solveWasm(digData)
    assertIdentical({ js: solveJs(digData), wasm: r1 }, 'dig normal')
    expect(r1.status).to.equal('success')
    expect(r1.path.some(n => (n.toBreak ?? []).length > 0)).to.equal(true, 'must dig through the wall')

    // Same shape, 2× labor, DIFFERENT fingerprint: the wasm side must swap
    // tables, not reuse the resident cheap ones. (2×, not more: an edge
    // whose dig cost pushes it past upstream's 100-cost move cap becomes
    // ILLEGAL, not expensive — both engines would agree on noPath.)
    const expensive: DigData = {
      fingerprint: digData.fingerprint + ':expensive',
      labor: digData.labor.map(v => v * 2),
      flags: digData.flags
    }
    const r2 = solveWasm(expensive)
    assertIdentical({ js: solveJs(expensive), wasm: r2 }, 'dig expensive')
    expect(r2.status).to.equal('success')
    expect(r2.cost).to.be.greaterThan(r1.cost)

    // Revert to the original fingerprint: results must return to r1 exactly.
    const r3 = solveWasm(digData)
    expect(pathKey(r3)).to.equal(pathKey(r1))
    expect(r3.cost).to.equal(r1.cost)
    expect(r3.visitedNodes).to.equal(r1.visitedNodes)
  })

  it('walled goal noPath identical (boundary + visited parity)', () => {
    const world = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 24, y1: 10, z1: 24 })
    world.fill(0, 0, 0, 24, 0, 24, STONE)
    world.fill(14, 1, 14, 22, 4, 22, STONE)
    world.fill(17, 1, 17, 19, 3, 19, AIR)
    world.fill(0, 1, 0, 2, 3, 2, AIR)
    assertIdentical(solveBoth(world, new GoalBlock(18, 1, 18)), 'walled')
  })
})
