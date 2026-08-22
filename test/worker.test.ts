// Worker round-trip: solves run in a real worker_thread against the BUILT
// dist tree (the worker entry is a compiled artifact). Requires `npm run
// build` first — the suite skips itself with a clear message otherwise.
import { expect } from 'chai'
import { existsSync } from 'node:fs'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import { SolverWorkerHost } from '../src/worker/host.js'
import { GoalBlock } from '../src/goals.js'
import { serializeGoal } from '../src/goalSerde.js'
import { VoxelWorld, STONE, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld } from './helpers/voxelWorld.js'
import { bakeEntityIndex } from '../src/snapshot.js'
import { computeDigData } from '../src/digData.js'
import type { DigData } from '../src/types.js'
import type { RawSolveResult } from '../src/solver.js'

const pkgRoot = join(dirname(fileURLToPath(import.meta.url)), '..')
const workerEntry = join(pkgRoot, 'dist', 'esm', 'worker', 'entry.js')

describe('worker round-trip', function () {
  this.timeout(30000)

  before(function () {
    if (!existsSync(workerEntry)) {
      console.warn(`[worker.test] SKIPPING — dist not built (${workerEntry} missing). Run: npm run build`)
      this.skip()
    }
  })

  function makeScene (): { host: SolverWorkerHost, req: (goal: GoalBlock, onPartial?: (r: RawSolveResult) => void) => Promise<{ handle: NonNullable<Awaited<ReturnType<SolverWorkerHost['solve']>>>, result: Promise<RawSolveResult> }> } {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 40, y1: 8, z1: 40 })
    world.fill(-4, -1, -4, 40, -1, 40, STONE)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot)
    const lut = lutFor(bot, movements)
    const snapshot = snapshotFromWorld(world, lut)
    bakeEntityIndex(snapshot, movements)
    const host = new SolverWorkerHost()
    host.setEntryPath(workerEntry)

    const req = async (goal: GoalBlock, onPartial: (r: RawSolveResult) => void = () => {}) => {
      const descriptor = serializeGoal(goal)
      if (!descriptor) throw new Error('goal not serializable')
      const handle = await host.solve({
        snapshot,
        lut,
        cfg: movements.toConfig(),
        goal: descriptor,
        start: { x: 0, y: 0, z: 0 },
        timeout: 5000,
        searchRadius: -1,
        sliceMs: 40,
        onPartial
      })
      if (!handle) throw new Error('worker unavailable')
      return { handle, result: handle.promise }
    }
    return { host, req }
  }

  it('solves a goal in the worker and returns an upstream-shaped result', async () => {
    const { host, req } = makeScene()
    try {
      const { result } = await req(new GoalBlock(30, 0, 30))
      const r = await result
      expect(r.status).to.equal('success')
      expect(r.path.length).to.be.greaterThan(0)
      const last = r.path[r.path.length - 1]
      expect([last.x, last.y, last.z]).to.deep.equal([30, 0, 30])
      expect(r.visitedNodes).to.be.a('number')
      expect(r.time).to.be.a('number')
      expect(r.touchedChunks.length).to.be.greaterThan(0)
    } finally {
      await host.terminate()
    }
  })

  it('two sequential solves reuse the worker; results stay independent', async () => {
    const { host, req } = makeScene()
    try {
      const r1 = await (await req(new GoalBlock(10, 0, 0))).result
      const r2 = await (await req(new GoalBlock(0, 0, 12))).result
      expect(r1.status).to.equal('success')
      expect(r2.status).to.equal('success')
      const l1 = r1.path[r1.path.length - 1]
      const l2 = r2.path[r2.path.length - 1]
      expect([l1.x, l1.z]).to.deep.equal([10, 0])
      expect([l2.x, l2.z]).to.deep.equal([0, 12])
    } finally {
      await host.terminate()
    }
  })

  it('canDig round-trip: dig tables sent once per fingerprint, solves stay correct', async () => {
    // Flat floor with a full-height wall — the only route is through it.
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 20, y1: 8, z1: 20 })
    world.fill(-4, -1, -4, 20, -1, 20, STONE)
    world.fill(8, 0, -4, 8, 6, 20, STONE)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot, { canDig: true })
    const lut = lutFor(bot, movements)
    const snapshot = snapshotFromWorld(world, lut, true)
    bakeEntityIndex(snapshot, movements)
    const digData = computeDigData(bot as never, movements, lut)
    const host = new SolverWorkerHost()
    host.setEntryPath(workerEntry)
    try {
      const descriptor = serializeGoal(new GoalBlock(14, 0, 0))!
      const solveOnce = async (dig: DigData): Promise<RawSolveResult> => {
        const handle = await host.solve({
          snapshot,
          lut,
          cfg: movements.toConfig(),
          goal: descriptor,
          start: { x: 0, y: 0, z: 0 },
          timeout: 10000,
          searchRadius: -1,
          sliceMs: 40,
          dig,
          onPartial: () => {}
        })
        if (!handle) throw new Error('worker unavailable')
        return await handle.promise
      }
      // Same fingerprint twice: the second solve references the resident
      // tables (no resend) and must produce the identical answer.
      const r1 = await solveOnce(digData)
      const r2 = await solveOnce(digData)
      for (const r of [r1, r2]) {
        expect(r.status).to.equal('success')
        expect(r.path.some(n => (n.toBreak ?? []).length > 0)).to.equal(true, 'must dig through the wall')
      }
      expect(r2.path.map(n => `${n.x},${n.y},${n.z}`)).to.deep.equal(r1.path.map(n => `${n.x},${n.y},${n.z}`))
      expect(r2.cost).to.be.closeTo(r1.cost, 1e-9)

      // A new fingerprint with pricier labor must actually replace the
      // resident tables — the cost must rise accordingly. (2×, not more:
      // past upstream's 100-cost move cap a dig becomes illegal, not
      // expensive, and the solve flips to noPath.)
      const expensive: DigData = {
        fingerprint: digData.fingerprint + ':expensive',
        labor: digData.labor.map(v => v * 2),
        flags: digData.flags
      }
      const r3 = await solveOnce(expensive)
      expect(r3.status).to.equal('success')
      expect(r3.cost).to.be.greaterThan(r1.cost)
    } finally {
      await host.terminate()
    }
  })

  it('cancellation aborts a long solve promptly', async () => {
    const { host } = makeScene()
    try {
      // A big empty world with an unreachable goal makes the solver grind
      // through the whole reachable set — plenty of time to cancel.
      const world = new VoxelWorld({ x0: -60, y0: -2, z0: -60, x1: 60, y1: 6, z1: 60 })
      world.fill(-60, -1, -60, 60, -1, 60, STONE)
      const bot = makeFakeBot(world)
      const movements = makeOurMovements(bot)
      const lut = lutFor(bot, movements)
      const snapshot = snapshotFromWorld(world, lut)
      bakeEntityIndex(snapshot, movements)
      const descriptor = serializeGoal(new GoalBlock(0, 40, 0)) // floating: unreachable
      const handle = await host.solve({
        snapshot,
        lut,
        cfg: movements.toConfig(),
        goal: descriptor!,
        start: { x: 0, y: 0, z: 0 },
        timeout: 60000,
        searchRadius: -1,
        sliceMs: 20,
        onPartial: () => {}
      })
      expect(handle).to.not.equal(null)
      const started = Date.now()
      setTimeout(() => handle!.cancel(), 50)
      const r = await handle!.promise
      expect(r.cancelled).to.equal(true)
      expect(Date.now() - started).to.be.lessThan(5000)
    } finally {
      await host.terminate()
    }
  })

  it('streams partial results for a budgeted long solve', async () => {
    const { host } = makeScene()
    try {
      const world = new VoxelWorld({ x0: -60, y0: -2, z0: -60, x1: 60, y1: 6, z1: 60 })
      world.fill(-60, -1, -60, 60, -1, 60, STONE)
      const bot = makeFakeBot(world)
      const movements = makeOurMovements(bot)
      const lut = lutFor(bot, movements)
      const snapshot = snapshotFromWorld(world, lut)
      bakeEntityIndex(snapshot, movements)
      const descriptor = serializeGoal(new GoalBlock(0, 40, 0))
      let partials = 0
      const handle = await host.solve({
        snapshot,
        lut,
        cfg: movements.toConfig(),
        goal: descriptor!,
        start: { x: 0, y: 0, z: 0 },
        timeout: 3000,
        searchRadius: -1,
        sliceMs: 5, // tiny slices to force partial emission
        onPartial: () => { partials++ }
      })
      const r = await handle!.promise
      expect(r.status).to.be.oneOf(['noPath', 'timeout'])
      expect(partials).to.be.greaterThan(0)
    } finally {
      await host.terminate()
    }
  })
})
