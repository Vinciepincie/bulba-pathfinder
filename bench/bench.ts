// Benchmark: @bulba/pathfinder vs upstream mineflayer-pathfinder on
// identical worlds and identical movement profiles. Run with:
//   npm run bench            (compact table)
//   PF_BENCH_N=100 npm run bench
//
// "ours (warm)" = snapshot cached (the steady-state recompute case the
// plugin actually hits); "ours (cold)" = snapshot rebuilt every solve;
// "worker e2e" = full round-trip through the worker thread incl. messaging.
import { createRequire } from 'node:module'
import { performance } from 'node:perf_hooks'
import { existsSync } from 'node:fs'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
import { Solver } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { GoalBlock } from '../src/goals.js'
import { serializeGoal } from '../src/goalSerde.js'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { computeDigData } from '../src/digData.js'
import { SolverWorkerHost } from '../src/worker/host.js'
import { bakeEntityIndex } from '../src/snapshot.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld, applyProfile,
  AIR, STONE, DIRT, WATER, LAVA, OAK_LEAVES, COBWEB
} from '../test/helpers/voxelWorld.js'

const require2 = createRequire(import.meta.url)
const UpstreamAStar = require2('mineflayer-pathfinder/lib/astar')
const UpstreamMovements = require2('mineflayer-pathfinder/lib/movements')
const UpstreamMove = require2('mineflayer-pathfinder/lib/move')
const upstreamGoals = require2('mineflayer-pathfinder/lib/goals')

const N = Number(process.env.PF_BENCH_N ?? 40)
// Unsampled warmup iterations per cell: the bench measures STEADY STATE
// (what a long-lived bot actually pays per solve), not one-time costs like
// worker spawn, V8 JIT warmup or wasm instantiation that would otherwise be
// amortized into whichever scenario happens to run first. PF_BENCH_WARMUP=0
// restores cold-inclusive numbers.
const WARMUP = Number(process.env.PF_BENCH_WARMUP ?? 3)

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

interface Scenario {
  name: string
  world: VoxelWorld
  start: { x: number, y: number, z: number }
  goal: { x: number, y: number, z: number }
  searchRadius: number
  canDig?: boolean
}

function scatter (world: VoxelWorld, seed: number, density: number, x0: number, z0: number, x1: number, z1: number): void {
  const rand = mulberry32(seed)
  for (let x = x0; x <= x1; x++) {
    for (let z = z0; z <= z1; z++) {
      const r = rand()
      if (r < density) world.fill(x, 1, z, x, 1 + Math.floor(rand() * 2), z, rand() < 0.7 ? STONE : DIRT)
      else if (r < density + 0.03) world.set(x, 1, z, OAK_LEAVES)
      else if (r < density + 0.045 && world.stateAt(x, 1, z) === AIR) world.set(x, 1, z, WATER)
    }
  }
}

function makeScenarios (): Scenario[] {
  const scenarios: Scenario[] = []

  {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 20, y1: 8, z1: 20 })
    world.fill(-4, 0, -4, 20, 0, 20, STONE)
    world.fill(0, 1, 0, 2, 3, 2, AIR)
    scenarios.push({ name: 'short walk (8 blocks, flat)', world, start: { x: 1, y: 1, z: 1 }, goal: { x: 9, y: 1, z: 1 }, searchRadius: 64 })
  }
  {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 60, y1: 10, z1: 60 })
    world.fill(-4, 0, -4, 60, 0, 60, STONE)
    scatter(world, 11, 0.13, -4, -4, 60, 60)
    world.fill(0, 1, 0, 2, 3, 2, AIR)
    world.set(48, 0, 48, STONE)
    world.fill(48, 1, 48, 48, 2, 48, AIR)
    scenarios.push({ name: 'long walk (48 blocks, obstacles)', world, start: { x: 1, y: 1, z: 1 }, goal: { x: 48, y: 1, z: 48 }, searchRadius: 64 })
  }
  {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 48, y1: 10, z1: 48 })
    world.fill(-4, 0, -4, 48, 0, 48, STONE)
    scatter(world, 22, 0.12, -4, -4, 48, 48)
    world.fill(0, 1, 0, 2, 3, 2, AIR)
    // Walled goal — the historic worst case (NoPath proof).
    world.fill(38, 1, 38, 46, 4, 46, STONE)
    world.fill(41, 1, 41, 43, 3, 43, AIR)
    world.set(42, 0, 42, STONE)
    scenarios.push({ name: 'walled goal → NoPath (radius 64)', world, start: { x: 1, y: 1, z: 1 }, goal: { x: 42, y: 1, z: 42 }, searchRadius: 64 })
  }
  {
    const world = new VoxelWorld({ x0: -80, y0: -2, z0: -80, x1: 80, y1: 8, z1: 80 })
    world.fill(-80, 0, -80, 80, 0, 80, STONE)
    world.fill(-1, 1, -1, 1, 3, 1, AIR)
    world.fill(30, 1, -80, 34, 4, 80, STONE) // wall bisecting the world
    world.set(50, 0, 0, STONE)
    world.fill(50, 1, 0, 50, 2, 0, AIR)
    scenarios.push({ name: 'walled goal → NoPath (UNBOUNDED radius, 160×160)', world, start: { x: 0, y: 1, z: 0 }, goal: { x: 50, y: 1, z: 0 }, searchRadius: -1 })
  }
  {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 40, y1: 10, z1: 40 })
    world.fill(-4, 0, -4, 40, 0, 40, STONE)
    // Serpentine maze walls.
    for (let i = 4; i <= 36; i += 8) {
      world.fill(i, 1, -4, i, 3, 30, STONE)
      world.fill(i + 4, 1, 6, i + 4, 3, 40, STONE)
    }
    world.fill(0, 1, 0, 2, 3, 2, AIR)
    world.set(38, 0, 38, STONE)
    world.fill(38, 1, 38, 38, 2, 38, AIR)
    scenarios.push({ name: 'serpentine maze (long detours)', world, start: { x: 1, y: 1, z: 1 }, goal: { x: 38, y: 1, z: 38 }, searchRadius: -1 })
  }
  {
    // Parkour gauntlet: single-block pillars over a void deeper than any
    // legal drop — 2-gap sprint jumps are the ONLY route to the goal.
    const world = new VoxelWorld({ x0: -4, y0: -8, z0: -4, x1: 38, y1: 8, z1: 8 })
    world.fill(-1, 0, -1, 1, 0, 1, STONE) // start platform
    for (let x = 4; x <= 31; x += 3) world.set(x, 0, 0, STONE)
    world.fill(33, 0, -1, 35, 0, 1, STONE) // goal platform
    scenarios.push({ name: 'parkour gauntlet (11× 2-gap jumps over void)', world, start: { x: 0, y: 1, z: 0 }, goal: { x: 34, y: 1, z: 0 }, searchRadius: 64 })
  }
  {
    // "Anarchy spawn": a 97×97 wasteland in the 2b2t style — lavacast
    // cones (45° slopes, climbable), cratered ground with lava pits, junk
    // pillars / leaves / cobwebs / puddles everywhere, and a deep canyon
    // bisecting the map that can only be crossed by parkour stepping-stone
    // lines (1-gap, 2-gap and 3-gap sprint variants) or a 1-wide bridge.
    // Many viable routes, none trivial.
    const world = new VoxelWorld({ x0: -8, y0: -10, z0: -8, x1: 88, y1: 26, z1: 88 })
    const rand = mulberry32(20226)
    world.fill(-8, 0, -8, 88, 0, 88, STONE)

    // Cratered ground — some craters with lava floors.
    for (let i = 0; i < 60; i++) {
      const cx = -4 + Math.floor(rand() * 88)
      const cz = -4 + Math.floor(rand() * 88)
      const r = 1 + Math.floor(rand() * 3)
      const depth = 2 + Math.floor(rand() * 6)
      const lavaFloor = rand() < 0.35
      for (let x = cx - r; x <= cx + r; x++) {
        for (let z = cz - r; z <= cz + r; z++) {
          if (!world.inBounds(x, 0, z) || !world.inBounds(x, -depth, z)) continue
          world.fill(x, 1 - depth, z, x, 0, z, AIR)
          world.set(x, -depth, z, lavaFloor ? LAVA : STONE)
        }
      }
    }

    // Lavacast cones: radius shrinks one block per level → 45° step-up
    // slopes the pathfinder can climb (extra pathways over the junk).
    for (let i = 0; i < 9; i++) {
      const cx = 6 + Math.floor(rand() * 74)
      const cz = 6 + Math.floor(rand() * 74)
      const h = 7 + Math.floor(rand() * 9)
      for (let dy = 0; dy < h; dy++) {
        const r = h - dy - 1
        for (let dx = -r; dx <= r; dx++) {
          for (let dz = -r; dz <= r; dz++) {
            if (Math.abs(dx) + Math.abs(dz) > r) continue // diamond cone
            const x = cx + dx
            const z = cz + dz
            if (world.inBounds(x, 1 + dy, z)) world.set(x, 1 + dy, z, STONE)
          }
        }
      }
    }

    // Scattered junk: 1×1 pillars, leaves blobs, puddles, cobwebs.
    for (let i = 0; i < 220; i++) {
      const x = -6 + Math.floor(rand() * 92)
      const z = -6 + Math.floor(rand() * 92)
      if (!world.inBounds(x, 1, z)) continue
      const r = rand()
      if (r < 0.35) {
        const ph = 2 + Math.floor(rand() * 6)
        if (world.inBounds(x, ph, z)) world.fill(x, 1, z, x, ph, z, STONE)
      } else if (r < 0.6) {
        const s = rand() < 0.5 ? 0 : 1
        if (world.inBounds(x + s, 2 + s, z + s)) world.fill(x, 1, z, x + s, 2 + s, z + s, OAK_LEAVES)
      } else if (r < 0.75) {
        if (world.stateAt(x, 1, z) === AIR) world.set(x, 1, z, WATER)
      } else if (r < 0.85) {
        if (world.stateAt(x, 1, z) === AIR) world.set(x, 1, z, COBWEB)
      } else {
        if (world.inBounds(x, 4, z)) world.set(x, 4, z, STONE) // floating junk
      }
    }

    // The canyon: z 40..45, full width, 8 deep (beyond maxDropDown, walls
    // sheer) with a part-lava floor — slices straight through the casts.
    for (let x = -8; x <= 88; x++) {
      for (let z = 40; z <= 45; z++) {
        world.fill(x, -7, z, x, 26, z, AIR)
        world.set(x, -8, z, rand() < 0.4 ? LAVA : STONE)
      }
    }
    // Crossings — three parkour stepping-stone lines of varying difficulty
    // plus one 1-wide bridge, each with a cleared 3-wide approach slot.
    const clearApproach = (x: number): void => {
      world.fill(x - 1, 0, 36, x + 1, 0, 39, STONE)
      world.fill(x - 1, 1, 36, x + 1, 8, 39, AIR)
      world.fill(x - 1, 0, 46, x + 1, 0, 49, STONE)
      world.fill(x - 1, 1, 46, x + 1, 8, 49, AIR)
    }
    clearApproach(12) // 1-gap hops: stones every other block
    world.set(12, 0, 41, STONE); world.set(12, 0, 43, STONE); world.set(12, 0, 45, STONE)
    clearApproach(44) // 2-gap jumps
    world.set(44, 0, 42, STONE); world.set(44, 0, 45, STONE)
    clearApproach(76) // 3-gap sprint jump onto a single stone
    world.set(76, 0, 43, STONE)
    world.fill(60, 0, 40, 60, 0, 45, STONE) // the bridge
    world.fill(59, 1, 39, 61, 8, 46, AIR)
    world.fill(59, 0, 36, 61, 0, 39, STONE)
    world.fill(59, 0, 46, 61, 0, 49, STONE)

    // Start and goal pads, cleared last so nothing buries them.
    world.fill(-2, 0, -2, 2, 0, 2, STONE)
    world.fill(-2, 1, -2, 2, 5, 2, AIR)
    world.fill(78, 0, 78, 82, 0, 82, STONE)
    world.fill(78, 1, 78, 82, 5, 82, AIR)
    scenarios.push({ name: 'anarchy spawn (lavacasts, junk, parkour canyon crossings)', world, start: { x: 0, y: 1, z: 0 }, goal: { x: 80, y: 1, z: 80 }, searchRadius: -1 })
  }
  {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 24, y1: 10, z1: 8 })
    world.fill(-4, 0, -4, 24, 0, 8, STONE)
    world.fill(8, 1, -4, 10, 5, 8, STONE) // thick wall — digging is the only way
    // Unbounded radius: bare-hand stone digs cost ~48/step, which a 64
    // cost-slack correctly prunes — the dig route needs the slack off.
    scenarios.push({ name: 'dig tunnel through 3-thick wall (canDig)', world, start: { x: 1, y: 1, z: 1 }, goal: { x: 20, y: 1, z: 1 }, searchRadius: -1, canDig: true })
  }

  return scenarios
}

interface Timing {
  mean: number
  p95: number
}

function stats (samples: number[]): Timing {
  const sorted = [...samples].sort((a, b) => a - b)
  return {
    mean: samples.reduce((s, v) => s + v, 0) / samples.length,
    p95: sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * 0.95))]
  }
}

function fmt (t: Timing): string {
  return `${t.mean.toFixed(2)}ms (p95 ${t.p95.toFixed(2)})`
}

async function main (): Promise<void> {
  const rows: string[][] = []
  const pkgRoot = join(dirname(fileURLToPath(import.meta.url)), '..')
  const workerEntry = join(pkgRoot, 'dist', 'esm', 'worker', 'entry.js')
  const haveWorker = existsSync(workerEntry)
  const host = new SolverWorkerHost()
  if (haveWorker) host.setEntryPath(workerEntry)

  for (const sc of makeScenarios()) {
    const bot = makeFakeBot(sc.world) as unknown as Record<string, unknown>
    bot.pathfinder = { bestHarvestTool: () => null }
    const upMovements = applyProfile(new UpstreamMovements(bot))
    if (sc.canDig) upMovements.canDig = true

    const ourBot = makeFakeBot(sc.world)
    const movements = makeOurMovements(ourBot, sc.canDig ? { canDig: true } : {})
    const lut = lutFor(ourBot, movements)
    const goal = new GoalBlock(sc.goal.x, sc.goal.y, sc.goal.z)
    const descriptor = serializeGoal(goal)
    const evaluator = fastEvaluator(descriptor!) ?? new GoalAdapter(goal)
    const digData = sc.canDig ? computeDigData(ourBot as never, movements, lut) : null

    const mkDig = (snap: ReturnType<typeof snapshotFromWorld>) => sc.canDig
      ? { data: digData!, states: snap.allocStates(), breakExclusion: null }
      : null

    // upstream
    let upStatus = ''
    const upSamples: number[] = []
    for (let i = -WARMUP; i < N; i++) {
      const t0 = performance.now()
      const astar = new UpstreamAStar(
        new UpstreamMove(sc.start.x, sc.start.y, sc.start.z, 0, 0),
        upMovements,
        new upstreamGoals.GoalBlock(sc.goal.x, sc.goal.y, sc.goal.z),
        60000, 1e9, sc.searchRadius
      )
      let r = astar.compute()
      while (r.status === 'partial') r = astar.compute()
      if (i >= 0) upSamples.push(performance.now() - t0)
      upStatus = r.status
    }

    // ours (cold: snapshot rebuilt per solve)
    let ourStatus = ''
    const coldSamples: number[] = []
    for (let i = -WARMUP; i < N; i++) {
      const t0 = performance.now()
      const snap = snapshotFromWorld(sc.world, lut, Boolean(sc.canDig))
      bakeEntityIndex(snap, movements)
      const solver = new Solver(snap, movements.toConfig(), evaluator, sc.start, { timeout: 60000, searchRadius: sc.searchRadius }, null, mkDig(snap))
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      if (i >= 0) coldSamples.push(performance.now() - t0)
      ourStatus = r.status
    }

    // ours (warm: snapshot reused — the plugin's steady state)
    const warmSnap = snapshotFromWorld(sc.world, lut, Boolean(sc.canDig))
    bakeEntityIndex(warmSnap, movements)
    const warmDig = mkDig(warmSnap)
    const warmSamples: number[] = []
    for (let i = -WARMUP; i < N; i++) {
      const t0 = performance.now()
      const solver = new Solver(warmSnap, movements.toConfig(), evaluator, sc.start, { timeout: 60000, searchRadius: sc.searchRadius }, null, warmDig)
      let r = solver.compute(1e9)
      while (r.status === 'partial') r = solver.compute(1e9)
      if (i >= 0) warmSamples.push(performance.now() - t0)
    }

    // worker end-to-end (warm snapshot, includes messaging + scheduling)
    let workerCell = 'n/a (dist not built)'
    if (haveWorker && descriptor) {
      const e2eSamples: number[] = []
      for (let i = -WARMUP; i < N; i++) {
        const t0 = performance.now()
        const handle = await host.solve({
          snapshot: warmSnap,
          lut,
          cfg: movements.toConfig(),
          goal: descriptor,
          start: sc.start,
          timeout: 60000,
          searchRadius: sc.searchRadius,
          sliceMs: 1e9,
          dig: digData,
          onPartial: () => {}
        })
        if (!handle) break
        await handle.promise
        if (i >= 0) e2eSamples.push(performance.now() - t0)
      }
      if (e2eSamples.length > 0) workerCell = fmt(stats(e2eSamples))
    }

    const up = stats(upSamples)
    const cold = stats(coldSamples)
    const warm = stats(warmSamples)
    rows.push([
      sc.name,
      `${fmt(up)} [${upStatus}]`,
      `${fmt(cold)}`,
      `${fmt(warm)} [${ourStatus}]`,
      workerCell,
      `${(up.mean / warm.mean).toFixed(1)}x / ${(up.mean / cold.mean).toFixed(1)}x`
    ])
  }

  await host.terminate()

  const headers = ['scenario', 'upstream', 'ours cold', 'ours warm', 'worker e2e', 'speedup warm/cold']
  const widths = headers.map((h, i) => Math.max(h.length, ...rows.map(r => r[i].length)))
  const line = (cells: string[]): string => cells.map((c, i) => c.padEnd(widths[i])).join(' | ')
  console.log(`\n@bulba/pathfinder vs mineflayer-pathfinder — N=${N} solves per cell\n`)
  console.log(line(headers))
  console.log(widths.map(w => '-'.repeat(w)).join('-|-'))
  for (const r of rows) console.log(line(r))
  console.log()
}

main().then(() => process.exit(0), (e) => { console.error(e); process.exit(1) })
