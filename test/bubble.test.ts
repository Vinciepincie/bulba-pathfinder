// Bubble-column elevators (opt-in improvement: useBubbleColumns). Covers the
// LUT special grid, planner semantics (ride up/down, water-like landing,
// float support, parkour guard), JS ↔ wasm bit-identical parity, the worker
// specialBuf plumbing, and the executor's ride behavior (including the
// down-column jump-suppression regression).
import { expect } from 'chai'
import { existsSync } from 'node:fs'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import { Vec3 } from 'vec3'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { MoveGen, META_PARKOUR } from '../src/moveGen.js'
import { GoalBlock } from '../src/goals.js'
import { serializeGoal } from '../src/goalSerde.js'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { WasmSolver } from '../src/wasm/wasmSolver.js'
import { SolverWorkerHost } from '../src/worker/host.js'
import { applySnapshotBlockUpdate, bakeEntityIndex } from '../src/snapshot.js'
import { LutSpecial } from '../src/types.js'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld, applyProfile,
  AIR, STONE, SOUL_SAND, MAGMA, BUBBLE_UP_STATE, BUBBLE_DOWN_STATE
} from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

const pkgRoot = join(dirname(fileURLToPath(import.meta.url)), '..')
const workerEntry = join(pkgRoot, 'dist', 'esm', 'worker', 'entry.js')

/**
 * Up-elevator scene: ground at y=0 (stand y=1), a fully cased 1×1 shaft at
 * (10, z=0) — soul sand base, bubble-up cells y1..7 — with a bottom entry
 * from the west and a top exit rim to the east. The ride is the ONLY route.
 */
function upElevatorWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -2, y0: -1, z0: -3, x1: 16, y1: 12, z1: 3 })
  world.fill(-2, 0, -3, 16, 0, 3, STONE)
  world.fill(9, 1, -1, 11, 8, 1, STONE) // casing
  world.set(10, 0, 0, SOUL_SAND)
  for (let y = 1; y <= 7; y++) world.set(10, y, 0, BUBBLE_UP_STATE)
  world.set(10, 8, 0, AIR) // open shaft top (bobbing headroom)
  world.fill(9, 1, 0, 9, 2, 0, AIR) // bottom entry
  world.fill(11, 7, 0, 11, 8, 0, AIR) // top exit; rim floor = casing at (11,6,0)
  return world
}

/**
 * Down-elevator scene: an elevated deck (floor y=7, stand y=8) feeding a
 * cased magma shaft at (10, z=0) — bubble-down cells y1..8 — with a bottom
 * exit east to the ground. Any plain drop is > maxDropDown, so the ride is
 * the only way down.
 */
function downElevatorWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -2, y0: -1, z0: -3, x1: 16, y1: 12, z1: 3 })
  world.fill(-2, 0, -3, 16, 0, 3, STONE)
  world.fill(-2, 7, -3, 9, 7, 3, STONE) // deck
  world.fill(9, 1, -1, 11, 9, 1, STONE) // casing
  world.set(10, 0, 0, MAGMA)
  for (let y = 1; y <= 8; y++) world.set(10, y, 0, BUBBLE_DOWN_STATE)
  world.set(10, 9, 0, AIR)
  world.fill(9, 8, 0, 9, 9, 0, AIR) // top entry from the deck
  world.fill(11, 1, 0, 11, 2, 0, AIR) // bottom exit
  return world
}

interface SolveOut {
  result: RawSolveResult
  wasm: RawSolveResult | null
}

function pathKey (r: RawSolveResult): string {
  return r.path.map(n => `${n.x},${n.y},${n.z},${n.parkour ? 1 : 0}`).join('|')
}

describe('bubble-column elevators', function () {
  this.timeout(60000)
  let wasm: WasmSolver | null = null

  before(async function () {
    wasm = await WasmSolver.create()
  })

  function solveScene (
    world: VoxelWorld,
    start: { x: number, y: number, z: number },
    goal: GoalBlock,
    enabled: boolean
  ): SolveOut {
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, enabled ? { useBubbleColumns: true } : {})
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const descriptor = serializeGoal(goal)!
    const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, start, {
      timeout: 30000,
      searchRadius: -1
    }, null, null)
    let result = solver.compute(1e9)
    while (result.status === 'partial') result = solver.compute(1e9)

    let wasmResult: RawSolveResult | null = null
    if (wasm) {
      wasmResult = wasm.solve(
        { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: null, special: snap.special, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
        movements.toConfig(), descriptor, start, null,
        { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
      )
    }
    return { result, wasm: wasmResult }
  }

  function assertWasmIdentical (out: SolveOut, label: string): void {
    if (!out.wasm) return
    expect(out.wasm.status).to.equal(out.result.status, `${label}: status`)
    expect(Math.abs(out.wasm.cost - out.result.cost)).to.be.at.most(1e-9, `${label}: cost`)
    expect(pathKey(out.wasm)).to.equal(pathKey(out.result), `${label}: path`)
    expect(out.wasm.visitedNodes).to.equal(out.result.visitedNodes, `${label}: visited`)
    expect(out.wasm.boundaryLimited).to.equal(out.result.boundaryLimited, `${label}: boundary`)
  }

  it('LUT: bubble_column states get the special grid only when opted in', () => {
    const world = upElevatorWorld()
    const bot = makeFakeBot(world)
    const on = lutFor(bot, makeOurMovements(bot, { useBubbleColumns: true }))
    expect(on.special).to.not.equal(null)
    expect(on.special![BUBBLE_UP_STATE]).to.equal(LutSpecial.BUBBLE_UP)
    expect(on.special![BUBBLE_DOWN_STATE]).to.equal(LutSpecial.BUBBLE_DOWN)
    const off = lutFor(bot, makeOurMovements(bot))
    expect(off.special).to.equal(null)
  })

  it('rides an up-column to the top exit (and wasm agrees bit-for-bit)', () => {
    const world = upElevatorWorld()
    const out = solveScene(world, { x: 0, y: 1, z: 0 }, new GoalBlock(11, 7, 0), true)
    expect(out.result.status).to.equal('success')
    const last = out.result.path[out.result.path.length - 1]
    expect([last.x, last.y, last.z]).to.deep.equal([11, 7, 0])
    // The ride: consecutive ascending nodes inside the column.
    const columnNodes = out.result.path.filter(n => n.x === 10 && n.z === 0)
    expect(columnNodes.length).to.be.greaterThan(4)
    for (let i = 1; i < columnNodes.length; i++) {
      expect(columnNodes[i].y).to.equal(columnNodes[i - 1].y + 1)
    }
    assertWasmIdentical(out, 'up elevator')
  })

  it('up-column is unreachable with the feature disabled (parity default)', () => {
    const world = upElevatorWorld()
    const out = solveScene(world, { x: 0, y: 1, z: 0 }, new GoalBlock(11, 7, 0), false)
    expect(out.result.status).to.equal('noPath')
    assertWasmIdentical(out, 'up elevator disabled')
  })

  it('rides a down-column past magma to the bottom exit (wasm identical)', () => {
    const world = downElevatorWorld()
    const out = solveScene(world, { x: 0, y: 8, z: 0 }, new GoalBlock(12, 1, 0), true)
    expect(out.result.status).to.equal('success')
    // Descent mixes 1-block rides with 2-block sink moves (moveDown's
    // landing scan stops at the next bubble cell) — strictly decreasing.
    const columnNodes = out.result.path.filter(n => n.x === 10 && n.z === 0)
    expect(columnNodes.length).to.be.greaterThan(2)
    for (let i = 1; i < columnNodes.length; i++) {
      expect(columnNodes[i].y).to.be.lessThan(columnNodes[i - 1].y)
    }
    // The bottom node stands on magma — walkable, exactly like upstream.
    expect(columnNodes[columnNodes.length - 1].y).to.equal(1)
    assertWasmIdentical(out, 'down elevator')
  })

  it('down-column is unreachable with the feature disabled', () => {
    const world = downElevatorWorld()
    const out = solveScene(world, { x: 0, y: 8, z: 0 }, new GoalBlock(12, 1, 0), false)
    expect(out.result.status).to.equal('noPath')
    assertWasmIdentical(out, 'down elevator disabled')
  })

  it('a fall lands IN the column water-style, beyond maxDropDown (wasm identical)', () => {
    // Ledge (floor y=7, stand y=8) over void; a short bubble column at x=3
    // whose top cell is y=3 → drop of 5 > maxDropDown 4, legal only because
    // a column catches like liquid (infiniteLiquidDropdownDistance).
    const world = new VoxelWorld({ x0: -1, y0: -1, z0: -2, x1: 6, y1: 10, z1: 2 })
    world.fill(-1, 7, -1, 2, 7, 1, STONE)
    world.set(3, 0, 0, SOUL_SAND)
    for (let y = 1; y <= 3; y++) world.set(3, y, 0, BUBBLE_UP_STATE)
    const out = solveScene(world, { x: 0, y: 8, z: 0 }, new GoalBlock(3, 3, 0), true)
    expect(out.result.status).to.equal('success')
    const last = out.result.path[out.result.path.length - 1]
    expect([last.x, last.y, last.z]).to.deep.equal([3, 3, 0])
    assertWasmIdentical(out, 'fall into column')

    const off = solveScene(world, { x: 0, y: 8, z: 0 }, new GoalBlock(3, 3, 0), false)
    expect(off.result.status).to.equal('noPath')
    assertWasmIdentical(off, 'fall into column disabled')
  })

  it('sealing the column via in-place patch invalidates the route (both engines)', () => {
    const world = upElevatorWorld()
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, { useBubbleColumns: true })
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const descriptor = serializeGoal(new GoalBlock(11, 7, 0))!
    const solveJs = (): RawSolveResult => {
      const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, { x: 0, y: 1, z: 0 }, { timeout: 30000, searchRadius: -1 }, null, null)
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      return r
    }
    const solveWasm = (): RawSolveResult | null => wasm
      ? wasm.solve(
        { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: null, special: snap.special, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
        movements.toConfig(), descriptor, { x: 0, y: 1, z: 0 }, null,
        { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} })
      : null

    expect(solveJs().status).to.equal('success')
    const w1 = solveWasm()
    if (w1) expect(w1.status).to.equal('success')

    // Seal the shaft mid-column: the special grid must be re-uploaded too.
    applySnapshotBlockUpdate(snap, lut, 10, 4, 0, STONE)
    const j2 = solveJs()
    expect(j2.status).to.equal('noPath')
    const w2 = solveWasm()
    if (w2) {
      expect(w2.status).to.equal('noPath')
      expect(w2.visitedNodes).to.equal(j2.visitedNodes)
    }
  })

  it('no parkour edges while floating in a column', () => {
    // A floating column node with a parkour-able ledge 2 blocks east: with a
    // solid stand-in block the jump generates; from the bubble cell it must not.
    const world = new VoxelWorld({ x0: -1, y0: -1, z0: -2, x1: 8, y1: 8, z1: 2 })
    world.set(3, 0, 0, SOUL_SAND)
    for (let y = 1; y <= 4; y++) world.set(3, y, 0, BUBBLE_UP_STATE)
    world.fill(5, 2, 0, 6, 2, 0, STONE) // ledge floor — parkour target at y=3
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, { useBubbleColumns: true })
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const gen = new MoveGen(snap, movements.toConfig())
    gen.generate(3, 3, 0) // floating in the column at ledge height
    for (let i = 0; i < gen.outCount; i++) {
      expect(gen.outMeta[i] & META_PARKOUR).to.equal(0, `parkour edge to ${gen.outX[i]},${gen.outY[i]},${gen.outZ[i]}`)
    }

    // Control: same geometry with a solid pillar instead of the column DOES
    // parkour from (3,3,0) — proving the guard (not geometry) suppressed it.
    const world2 = new VoxelWorld({ x0: -1, y0: -1, z0: -2, x1: 8, y1: 8, z1: 2 })
    world2.fill(3, 0, 0, 3, 2, 0, STONE)
    world2.fill(5, 2, 0, 6, 2, 0, STONE)
    const bot2 = makeFakeBot(world2)
    const movements2 = makeOurMovements(bot2, { useBubbleColumns: true })
    const lut2 = lutFor(bot2, movements2)
    const snap2 = snapshotFromWorld(world2, lut2)
    const gen2 = new MoveGen(snap2, movements2.toConfig())
    gen2.generate(3, 3, 0)
    let parkourEdges = 0
    for (let i = 0; i < gen2.outCount; i++) {
      if ((gen2.outMeta[i] & META_PARKOUR) !== 0) parkourEdges++
    }
    expect(parkourEdges).to.be.greaterThan(0)
  })

  it('worker round-trip carries the special grid (specialBuf plumbing)', async function () {
    if (!existsSync(workerEntry)) {
      console.warn('[bubble.test] SKIPPING worker round-trip — dist not built')
      this.skip()
    }
    const world = upElevatorWorld()
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, { useBubbleColumns: true })
    const lut = lutFor(bot, movements)
    const snapshot = snapshotFromWorld(world, lut)
    bakeEntityIndex(snapshot, movements)
    const host = new SolverWorkerHost()
    host.setEntryPath(workerEntry)
    try {
      const handle = await host.solve({
        snapshot,
        lut,
        cfg: movements.toConfig(),
        goal: serializeGoal(new GoalBlock(11, 7, 0))!,
        start: { x: 0, y: 1, z: 0 },
        timeout: 10000,
        searchRadius: -1,
        sliceMs: 40,
        onPartial: () => {}
      })
      expect(handle).to.not.equal(null)
      const r = await handle!.promise
      expect(r.status).to.equal('success')
      expect(r.path.some(n => n.x === 10 && n.z === 0 && n.y >= 5)).to.equal(true, 'must ride the column')
    } finally {
      await host.terminate()
    }
  })

  // ── executor rides (fake voxel physics with bubble drag) ────────────────

  interface ExecPF { goto: (goal: unknown) => Promise<void>, setMovements: (m: Movements) => void }

  function setupExec (world: VoxelWorld, start: Vec3): { bot: DriveableBot, pf: ExecPF } {
    const bot = makeDriveableBot(world, start)
    const plugin = createPathfinder({
      useWorkerThreads: false,
      physicsFactory: () => makeFakePhysics(world, bot)
    })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: ExecPF }).pathfinder
    const movements = applyProfile(new Movements(bot as never) as never) as unknown as Movements
    ;(movements as unknown as { useBubbleColumns: boolean }).useBubbleColumns = true
    pf.setMovements(movements)
    return { bot, pf }
  }

  async function driveUntilSettled (bot: DriveableBot, promise: Promise<void>, maxTicks = 3000): Promise<'resolved' | 'rejected'> {
    let settled: 'resolved' | 'rejected' | null = null
    promise.then(() => { settled = 'resolved' }, () => { settled = 'rejected' })
    for (let i = 0; i < maxTicks && !settled; i++) {
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }
    if (!settled) throw new Error(`goto did not settle within ${maxTicks} ticks (bot at ${bot.entity.position})`)
    return settled
  }

  it('executor rides an up-column to the top', async () => {
    const world = upElevatorWorld()
    const { bot, pf } = setupExec(world, new Vec3(0.5, 1, 0.5))
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(11, 7, 0)))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.y).to.be.closeTo(7, 1.2)
    expect(bot.entity.position.distanceTo(new Vec3(11.5, 7, 0.5))).to.be.lessThan(1.5)
  })

  it('executor recenters while rising: off-center corner start in a 2×2 column', async () => {
    // A 2×2 shaft: the bot starts at the shared corner (11.2, 1, 1.2) —
    // ~0.42 from its cell center, beyond the 0.35 node-arrival radius. The
    // drive loop must steer toward the node center WHILE the column lifts
    // it, or no ride node ever arrives and the goto stalls out.
    const world = new VoxelWorld({ x0: -2, y0: -1, z0: -3, x1: 16, y1: 12, z1: 4 })
    world.fill(-2, 0, -3, 16, 0, 4, STONE)
    world.fill(9, 1, -1, 12, 8, 2, STONE) // casing around the 2×2 shaft
    for (let x = 10; x <= 11; x++) {
      for (let z = 0; z <= 1; z++) {
        world.set(x, 0, z, SOUL_SAND)
        for (let y = 1; y <= 7; y++) world.set(x, y, z, BUBBLE_UP_STATE)
        world.set(x, 8, z, AIR)
      }
    }
    world.fill(12, 7, 0, 12, 8, 1, AIR) // top exit; rim floor = casing (12,6,*)
    const { bot, pf } = setupExec(world, new Vec3(11.2, 1, 1.2))
    let topEntry: Vec3 | null = null
    const origTick = bot.tick
    bot.tick = () => {
      origTick()
      if (topEntry === null && bot.entity.position.y > 5) topEntry = bot.entity.position.clone()
    }
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(12, 7, 0)))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(12.5, 7, 0.5))).to.be.lessThan(1.6)
    // By the upper half of the ride the bot must have converged onto a cell
    // center line (the corner start is ~0.76 from every center).
    expect(topEntry).to.not.equal(null)
    const centers = [[10.5, 0.5], [10.5, 1.5], [11.5, 0.5], [11.5, 1.5]]
    const offset = Math.min(...centers.map(([cx, cz]) => Math.hypot(topEntry!.x - cx, topEntry!.z - cz)))
    expect(offset).to.be.lessThan(0.4)
  })

  it('executor rides a down-column without fighting the drag (jump suppressed)', async () => {
    const world = downElevatorWorld()
    const { bot, pf } = setupExec(world, new Vec3(0.5, 8, 0.5))
    let jumpHeldWhileSinking = 0
    const origTick = bot.tick
    bot.tick = () => {
      const feet = world.stateAt(Math.floor(bot.entity.position.x), Math.floor(bot.entity.position.y + 0.001), Math.floor(bot.entity.position.z))
      if (feet === BUBBLE_DOWN_STATE && bot.entity.position.y > 2 && bot.controlState.jump) {
        jumpHeldWhileSinking++
      }
      origTick()
    }
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(12, 1, 0)))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(12.5, 1, 0.5))).to.be.lessThan(1.5)
    // The executor must not hold jump against the down-drag mid-ride.
    expect(jumpHeldWhileSinking).to.equal(0)
  })
})
