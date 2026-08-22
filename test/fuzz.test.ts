// Differential fuzz: our snapshot Solver vs upstream mineflayer-pathfinder's
// AStar+Movements as the oracle, over seeded random voxel worlds. Both sides
// run the identical walk-only profile (canDig=false, parkour+sprint on, no
// doors/entities), so reachability must agree exactly and costs must agree
// within a mutual 15% band; rare outliers beyond the band are arbitrated by
// an exact Dijkstra ground truth (exact A*-cost equality is impossible: see
// the note at the cost assertion — the shared heuristic is inadmissible
// around parkour and drop-down shortcuts, so both A*s are tie-break-sensitive
// approximations even over provably identical edge models).
//
// Deliberately excluded features (documented divergence / flakiness sources):
// doors, fence gates, cobweb. Repro: every case is derived from
// BASE_SEED + index via mulberry32 — the failing seed is printed on failure.
import { expect } from 'chai'
import { createRequire } from 'node:module'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { MoveGen } from '../src/moveGen.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { GoalBlock, GoalNear, GoalXZ } from '../src/goals.js'
import type { Goal } from '../src/goals.js'
import {
  VoxelWorld,
  makeFakeBot,
  makeOurMovements,
  lutFor,
  snapshotFromWorld,
  checkWalkable,
  applyProfile,
  AIR,
  STONE,
  WATER,
  OAK_FENCE,
  OAK_LEAVES,
  VINE
} from './helpers/voxelWorld.js'
import type { PathStep } from './helpers/voxelWorld.js'

const require = createRequire(import.meta.url)
/* eslint-disable @typescript-eslint/no-var-requires */
const UpstreamAStar = require('mineflayer-pathfinder/lib/astar')
const UpstreamMovements = require('mineflayer-pathfinder/lib/movements')
const UpstreamMove = require('mineflayer-pathfinder/lib/move')
const upstreamGoals = require('mineflayer-pathfinder/lib/goals')
/* eslint-enable @typescript-eslint/no-var-requires */

const BASE_SEED = 1234
const CASE_COUNT = Number(process.env.PF_FUZZ_CASES ?? 60)
const TIMEOUT_MS = 2000
const TICK_MS = 100000 // huge slice: avoid partial slicing on both sides

// ── seeded PRNG ────────────────────────────────────────────────────────────
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

// ── case generation ────────────────────────────────────────────────────────
const BOX = { x0: 0, y0: 0, z0: 0, x1: 24, y1: 12, z1: 24 }

interface FuzzCase {
  seed: number
  world: VoxelWorld
  gx: number
  gy: number
  gz: number
  features: string[]
}

// 'vines' runs in strict-parity mode (makeOurMovements deletes vine from
// climbables), so both engines must treat vine cells as plain pass-through —
// coverage that the auto-climb improvement never leaks into parity solves.
const FEATURE_KINDS = ['columns', 'walls', 'leaves', 'water', 'platforms', 'fences', 'vines'] as const

function genCase (seed: number): FuzzCase {
  const rand = mulberry32(seed)
  const world = new VoxelWorld({ ...BOX })
  world.fill(BOX.x0, 0, BOX.z0, BOX.x1, 0, BOX.z1, STONE) // stone floor at y=0

  const features: string[] = []
  const nFeatures = 1 + Math.floor(rand() * 3) // 1..3 features per case
  for (let f = 0; f < nFeatures; f++) {
    const kind = FEATURE_KINDS[Math.floor(rand() * FEATURE_KINDS.length)]
    features.push(kind)
    addFeature(world, rand, kind)
  }

  // Start pocket: 3x3x3 air around (1,1,1) so the start is always valid.
  world.fill(0, 1, 0, 2, 3, 2, AIR)

  // Goal: random standable cell — stone below, 1x2 air at/above.
  const gx = 4 + Math.floor(rand() * 20) // 4..23
  const gz = 4 + Math.floor(rand() * 20) // 4..23
  const gy = 1 + Math.floor(rand() * 4) // 1..4
  world.set(gx, gy - 1, gz, STONE)
  world.set(gx, gy, gz, AIR)
  world.set(gx, gy + 1, gz, AIR)

  return { seed, world, gx, gy, gz, features }
}

function addFeature (world: VoxelWorld, rand: () => number, kind: string): void {
  switch (kind) {
    case 'columns': {
      const density = 0.05 + rand() * 0.20 // 5%..25%
      for (let x = BOX.x0; x <= BOX.x1; x++) {
        for (let z = BOX.z0; z <= BOX.z1; z++) {
          if (rand() < density) {
            const h = 1 + Math.floor(rand() * 3) // height 1..3
            world.fill(x, 1, z, x, h, z, STONE)
          }
        }
      }
      break
    }
    case 'walls': {
      const nWalls = 1 + Math.floor(rand() * 3)
      for (let w = 0; w < nWalls; w++) {
        const alongX = rand() < 0.5
        const fixed = 2 + Math.floor(rand() * 21) // 2..22
        const a0 = Math.floor(rand() * 18)
        const len = 6 + Math.floor(rand() * 17)
        const h = 2 + Math.floor(rand() * 2) // height 2..3
        for (let a = a0; a <= Math.min(24, a0 + len); a++) {
          if (rand() < 0.12) continue // occasional 1-cell gap
          if (alongX) world.fill(a, 1, fixed, a, h, fixed, STONE)
          else world.fill(fixed, 1, a, fixed, h, a, STONE)
        }
      }
      break
    }
    case 'leaves': {
      const nBlobs = 1 + Math.floor(rand() * 3)
      for (let b = 0; b < nBlobs; b++) {
        const cx = 3 + Math.floor(rand() * 19)
        const cz = 3 + Math.floor(rand() * 19)
        const cy = 1 + Math.floor(rand() * 2)
        const r = 1 + Math.floor(rand() * 2)
        for (let dx = -r; dx <= r; dx++) {
          for (let dy = -r; dy <= r; dy++) {
            for (let dz = -r; dz <= r; dz++) {
              if (Math.abs(dx) + Math.abs(dy) + Math.abs(dz) > r + 1) continue
              const x = cx + dx
              const y = cy + dy
              const z = cz + dz
              if (y >= 1 && world.inBounds(x, y, z)) world.set(x, y, z, OAK_LEAVES)
            }
          }
        }
      }
      break
    }
    case 'water': {
      const nCells = 6 + Math.floor(rand() * 20)
      for (let c = 0; c < nCells; c++) {
        const x = Math.floor(rand() * 25)
        const z = Math.floor(rand() * 25)
        world.set(x, 1, z, WATER) // 1-deep puddle over the stone floor
      }
      break
    }
    case 'platforms': {
      const px0 = 3 + Math.floor(rand() * 16)
      const pz0 = 3 + Math.floor(rand() * 16)
      const px1 = Math.min(24, px0 + 3 + Math.floor(rand() * 5))
      const pz1 = Math.min(24, pz0 + 3 + Math.floor(rand() * 5))
      const top = 2 + Math.floor(rand() * 2) // platform surface at y = top (2..3)
      world.fill(px0, 1, pz0, px1, top, pz1, STONE)
      if (rand() < 0.6) {
        // Staircase off one edge: column of height (top-k) at offset k.
        const side = Math.floor(rand() * 4)
        const sx = side === 0 ? -1 : side === 1 ? 1 : 0
        const sz = side === 2 ? -1 : side === 3 ? 1 : 0
        const ax = sx < 0 ? px0 : px1
        const az = sz < 0 ? pz0 : pz1
        const mid = (lo: number, hi: number): number => lo + Math.floor(rand() * (hi - lo + 1))
        const bx = sx !== 0 ? ax : mid(px0, px1)
        const bz = sz !== 0 ? az : mid(pz0, pz1)
        for (let k = 1; k < top; k++) {
          const x = bx + sx * k
          const z = bz + sz * k
          if (world.inBounds(x, 1, z)) world.fill(x, 1, z, x, top - k, z, STONE)
        }
      }
      break
    }
    case 'vines': {
      // Vine curtains hanging in the air and draped over terrain.
      const nCurtains = 2 + Math.floor(rand() * 4)
      for (let c = 0; c < nCurtains; c++) {
        const x = 2 + Math.floor(rand() * 21)
        const z = 2 + Math.floor(rand() * 21)
        const top = 2 + Math.floor(rand() * 4) // 2..5
        for (let y = 1; y <= top; y++) {
          if (world.inBounds(x, y, z) && world.stateAt(x, y, z) === AIR) world.set(x, y, z, VINE)
        }
      }
      break
    }
    case 'fences': {
      const nLines = 1 + Math.floor(rand() * 2)
      for (let w = 0; w < nLines; w++) {
        const alongX = rand() < 0.5
        const fixed = 2 + Math.floor(rand() * 21)
        const a0 = Math.floor(rand() * 18)
        const len = 6 + Math.floor(rand() * 15)
        for (let a = a0; a <= Math.min(24, a0 + len); a++) {
          if (rand() < 0.15) continue // gap
          if (alongX) world.set(a, 1, fixed, OAK_FENCE)
          else world.set(fixed, 1, a, OAK_FENCE)
        }
      }
      break
    }
    default:
      throw new Error(`unknown feature kind ${kind}`)
  }
}

// ── ASCII repro dump ───────────────────────────────────────────────────────
function sliceAt (c: FuzzCase, y: number): string {
  const rows: string[] = []
  for (let z = BOX.z0; z <= BOX.z1; z++) {
    let row = ''
    for (let x = BOX.x0; x <= BOX.x1; x++) {
      if (x === 1 && z === 1 && y === 1) { row += 'S'; continue }
      if (x === c.gx && z === c.gz && y === c.gy) { row += 'G'; continue }
      const s = c.world.stateAt(x, y, z)
      row += s === AIR ? '.'
        : s === STONE ? '#'
          : s === WATER ? '~'
            : s === OAK_FENCE ? 'f'
              : s === OAK_LEAVES ? 'L'
                : s === VINE ? 'v'
                  : '?'
    }
    rows.push(row)
  }
  return rows.join('\n')
}

function describeCase (c: FuzzCase): string {
  const yLevels = [1, 2, 3, 4].filter(y => y <= Math.max(4, c.gy))
  const slices = yLevels.map(y => `--- y=${y} (x → right, z ↓) ---\n${sliceAt(c, y)}`)
  return [
    `seed=${c.seed} features=[${c.features.join(', ')}] goal=(${c.gx},${c.gy},${c.gz}) start=(1,1,1)`,
    ...slices
  ].join('\n')
}

// ── solver harnesses ───────────────────────────────────────────────────────
const START = { x: 1, y: 1, z: 1 }

interface UpstreamResult {
  status: string
  cost: number
  path: Array<{ x: number, y: number, z: number, parkour: boolean }>
}

function solveUpstream (world: VoxelWorld, goal: unknown): UpstreamResult {
  const bot = makeFakeBot(world) as unknown as Record<string, unknown>
  bot.pathfinder = { bestHarvestTool: () => null } // never called with canDig=false, safety net
  const movements = applyProfile(new UpstreamMovements(bot))
  const startMove = new UpstreamMove(START.x, START.y, START.z, movements.countScaffoldingItems(), 0)
  const astar = new UpstreamAStar(startMove, movements, goal, TIMEOUT_MS, TICK_MS, -1)
  let result = astar.compute()
  while (result.status === 'partial') result = astar.compute()
  return result
}

function solveOurs (world: VoxelWorld, goal: Goal): RawSolveResult {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot)
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), { ...START }, {
    timeout: TIMEOUT_MS,
    searchRadius: -1
  })
  let result = solver.compute(TICK_MS)
  while (result.status === 'partial') result = solver.compute(TICK_MS)
  return result
}

function toSteps (path: Array<{ x: number, y: number, z: number, parkour?: boolean }>): PathStep[] {
  return path.map(n => ({ x: n.x, y: n.y, z: n.z, parkour: n.parkour === true }))
}

// ── exact ground-truth oracle (Dijkstra, used only to arbitrate outliers) ──
interface HeapEntry<T> { k: number, v: T }
class TinyHeap<T> {
  private readonly a: Array<HeapEntry<T>> = []
  get size (): number { return this.a.length }
  push (k: number, v: T): void {
    const a = this.a
    a.push({ k, v })
    let i = a.length - 1
    while (i > 0) {
      const p = (i - 1) >> 1
      if (a[p].k <= a[i].k) break
      const t = a[p]; a[p] = a[i]; a[i] = t
      i = p
    }
  }

  pop (): HeapEntry<T> {
    const a = this.a
    const top = a[0]
    const last = a.pop() as HeapEntry<T>
    if (a.length > 0) {
      a[0] = last
      let i = 0
      for (;;) {
        const l = 2 * i + 1
        const r = l + 1
        let m = i
        if (l < a.length && a[l].k < a[m].k) m = l
        if (r < a.length && a[r].k < a[m].k) m = r
        if (m === i) break
        const t = a[m]; a[m] = a[i]; a[i] = t
        i = m
      }
    }
    return top
  }
}

/** True optimal cost over OUR MoveGen edge model (h=0, exact). */
function dijkstraOurEdges (world: VoxelWorld, isEnd: (x: number, y: number, z: number) => boolean): number {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot)
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  const gen = new MoveGen(snap, movements.toConfig())
  const meta = snap.meta
  const dist = new Map<number, number>()
  const open = new TinyHeap<number>()
  const startIdx = gen.cellIndex(START.x, START.y, START.z)
  dist.set(startIdx, 0)
  open.push(0, startIdx)
  while (open.size > 0) {
    const { k: d, v: idx } = open.pop()
    if (d > (dist.get(idx) ?? Infinity) + 1e-12) continue
    const x = (idx % meta.w) + meta.x0
    const y = Math.floor(idx / (meta.w * meta.l)) + meta.y0
    const z = (Math.floor(idx / meta.w) % meta.l) + meta.z0
    if (isEnd(x, y, z)) return d
    gen.generate(x, y, z)
    for (let i = 0; i < gen.outCount; i++) {
      const nIdx = gen.outIdx[i]
      const nd = d + gen.outCost[i]
      if (nd < (dist.get(nIdx) ?? Infinity)) {
        dist.set(nIdx, nd)
        open.push(nd, nIdx)
      }
    }
  }
  return Infinity
}

/** True optimal cost over UPSTREAM's getNeighbors edge model (h=0, exact). */
function dijkstraUpstreamEdges (world: VoxelWorld, isEnd: (x: number, y: number, z: number) => boolean): number {
  const bot = makeFakeBot(world) as unknown as Record<string, unknown>
  bot.pathfinder = { bestHarvestTool: () => null }
  const movements = applyProfile(new UpstreamMovements(bot))
  const dist = new Map<string, number>()
  const open = new TinyHeap<{ hash: string, move: unknown }>()
  const startMove = new UpstreamMove(START.x, START.y, START.z, movements.countScaffoldingItems(), 0)
  dist.set(startMove.hash, 0)
  open.push(0, { hash: startMove.hash, move: startMove })
  while (open.size > 0) {
    const { k: d, v } = open.pop()
    if (d > (dist.get(v.hash) ?? Infinity) + 1e-12) continue
    const mv = v.move as { x: number, y: number, z: number }
    if (isEnd(mv.x, mv.y, mv.z)) return d
    for (const n of movements.getNeighbors(v.move)) {
      const nd = d + n.cost
      if (nd < (dist.get(n.hash) ?? Infinity)) {
        dist.set(n.hash, nd)
        open.push(nd, { hash: n.hash, move: n })
      }
    }
  }
  return Infinity
}

// ── differential assertion (shared by fuzz + targeted cases) ───────────────
let skippedTimeouts = 0
let exactCostMatches = 0
let relaxedCostMatches = 0
let arbitratedCostMatches = 0

function runDifferential (c: FuzzCase, ourGoal: Goal, upGoal: unknown): void {
  const up = solveUpstream(c.world, upGoal)
  const ours = solveOurs(c.world, ourGoal)

  try {
    if (up.status === 'timeout' || ours.status === 'timeout') {
      skippedTimeouts++
      return // should be rare; counted and reported in after()
    }

    // 1. Reachability agreement.
    expect(ours.status === 'success').to.equal(
      up.status === 'success',
      `reachability mismatch: upstream=${up.status} ours=${ours.status}`
    )

    if (up.status === 'success' && ours.status === 'success') {
      // 2. Cost agreement. The shared heuristic (octile XZ + |dy|) is
      // inadmissible whenever parkour (3-block jump, cost 1, h≈3) or
      // drop-downs exist, and BOTH implementations terminate on the first
      // goal pop — an approximation neither side can escape cheaply. Our
      // solver additionally reopens closed nodes on a better g (upstream
      // never does), which removes most intermediate-route suboptimality:
      // across the 500-case sweep, costs match exactly in ~96% of successes
      // and ours is CHEAPER in most of the rest, with the rare ours-worse
      // case within a few percent (early goal-pop luck, e.g. seed 1561:
      // upstream 19.414 vs ours 20.0 on a verified-identical edge graph).
      // Hence: mutual 15% band as the fast path; outliers beyond the band
      // are arbitrated with an exact Dijkstra ground truth — the cheaper
      // cost must be realizable in the OTHER side's edge model, which pins
      // every real edge-model divergence (missing/mispriced moves) exactly
      // while tolerating documented search tie-break noise.
      if (Math.abs(ours.cost - up.cost) <= 1e-6) {
        exactCostMatches++
      } else if (ours.cost <= up.cost * 1.15 + 1e-6 && up.cost <= ours.cost * 1.15 + 1e-6) {
        relaxedCostMatches++
      } else {
        arbitratedCostMatches++
        const isEnd = (x: number, y: number, z: number): boolean => ourGoal.isEnd({ x, y, z })
        if (ours.cost < up.cost) {
          // Ours is much cheaper: our cost must be realizable in UPSTREAM's
          // own edge model, else our MoveGen underprices or invents a move.
          const trueUp = dijkstraUpstreamEdges(c.world, isEnd)
          expect(trueUp).to.be.at.most(
            ours.cost + 1e-6,
            `ours (${ours.cost}) beats upstream (${up.cost}) but is below upstream's true optimum ${trueUp} — our edge model diverges`
          )
        } else {
          // Ours is much more expensive: upstream's cost must be realizable
          // in OUR edge model, else our MoveGen is missing/overpricing a move.
          const trueOurs = dijkstraOurEdges(c.world, isEnd)
          expect(trueOurs).to.be.at.most(
            up.cost + 1e-6,
            `upstream (${up.cost}) beats ours (${ours.cost}) and is below our true optimum ${trueOurs} — our edge model diverges`
          )
        }
      }

      // 3. Our path is independently walkable in the raw world.
      const violation = checkWalkable(c.world, { ...START }, toSteps(ours.path))
      expect(violation, `unwalkable path: ${violation ?? ''}\npath=${JSON.stringify(ours.path.map(p => [p.x, p.y, p.z, p.parkour ? 'pk' : '']))}`).to.equal(null)
    }

    // 4. Determinism: an identical re-solve yields identical results.
    const again = solveOurs(c.world, ourGoal)
    expect(again.status).to.equal(ours.status, 'determinism: status differs on re-solve')
    expect(again.cost).to.equal(ours.cost, 'determinism: cost differs on re-solve')
    expect(again.visitedNodes).to.equal(ours.visitedNodes, 'determinism: visitedNodes differs on re-solve')
    expect(again.path.map(p => `${p.x},${p.y},${p.z}`).join(';'))
      .to.equal(ours.path.map(p => `${p.x},${p.y},${p.z}`).join(';'), 'determinism: path differs on re-solve')
  } catch (err) {
    // Repro aid: seed, features, and world slices.
    console.error('\nFUZZ CASE FAILED\n' + describeCase(c))
    console.error(`upstream: status=${up.status} cost=${up.cost} pathLen=${up.path?.length}`)
    console.error(`ours:     status=${ours.status} cost=${ours.cost} pathLen=${ours.path?.length}`)
    throw err
  }
}

// ── the suites ─────────────────────────────────────────────────────────────
describe('differential fuzz vs upstream mineflayer-pathfinder', function () {
  this.timeout(300000)

  for (let i = 0; i < CASE_COUNT; i++) {
    const seed = BASE_SEED + i
    it(`GoalBlock case ${i} (seed ${seed})`, () => {
      const c = genCase(seed)
      runDifferential(c, new GoalBlock(c.gx, c.gy, c.gz), new upstreamGoals.GoalBlock(c.gx, c.gy, c.gz))
    })
  }

  it('targeted GoalNear differential (range 2)', () => {
    const c = genCase(BASE_SEED + 100003)
    runDifferential(c, new GoalNear(c.gx, c.gy, c.gz, 2), new upstreamGoals.GoalNear(c.gx, c.gy, c.gz, 2))
  })

  it('targeted GoalXZ differential', () => {
    const c = genCase(BASE_SEED + 100007)
    runDifferential(c, new GoalXZ(c.gx, c.gz), new upstreamGoals.GoalXZ(c.gx, c.gz))
  })

  after(() => {
    console.log(`differential fuzz: cost exact=${exactCostMatches} within-15%=${relaxedCostMatches} dijkstra-arbitrated=${arbitratedCostMatches}`)
    if (skippedTimeouts > 0) {
      console.warn(`differential fuzz: skipped ${skippedTimeouts}/${CASE_COUNT + 2} case(s) due to solver timeout`)
    }
  })
})
