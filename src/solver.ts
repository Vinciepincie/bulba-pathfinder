// A* core over the snapshot: node ids are snapshot cell indices, g/parent/
// state live in reusable epoch-stamped typed arrays (no clearing between
// solves), the open set is a lazy-delete binary heap. Semantics are an exact
// port of mineflayer-pathfinder/lib/astar.js: same slicing statuses, same
// searchRadius cost-slack pruning, same best-node (lowest heuristic)
// partial-path selection, same tie-break-relevant relaxation rule.
import { performance } from 'node:perf_hooks'
import { MinHeap } from './heap.js'
import { MoveGen, META_PARKOUR, META_USEONE } from './moveGen.js'
import type { SnapshotView, StepExclusionFn, DigContext } from './moveGen.js'
import type { MovementsConfig, RawPathNode, SolveStatus } from './types.js'

export interface GoalEvaluator {
  heuristic (x: number, y: number, z: number): number
  isEnd (x: number, y: number, z: number): boolean
}

export interface SolveOptions {
  /** Total think budget in wall-clock ms since solver construction. */
  timeout: number
  /** searchRadius cost slack; -1 = unbounded. */
  searchRadius: number
  /** Optional shared cancel flag (worker mode); non-zero aborts the solve. */
  cancelFlag?: Int32Array | null
}

export interface RawSolveResult {
  status: SolveStatus
  cancelled: boolean
  cost: number
  time: number
  visitedNodes: number
  generatedNodes: number
  path: RawPathNode[]
  touchedChunks: Array<[number, number]>
  boundaryLimited: boolean
  /** Which engine produced this result ('js' when absent). */
  engine?: 'js' | 'wasm'
}

// ── reusable arenas (grow-only, epoch-stamped so no per-solve clearing) ────
// A pool rather than a single global: upstream's API allows several live
// astarContexts at once (an abandoned getPathFromTo generator + the drive
// loop's own solve), and they must not stomp each other's g/parent tables.
class Arena {
  size = 0
  g = new Float64Array(0)
  parent = new Int32Array(0)
  meta = new Uint8Array(0)
  stamp = new Int32Array(0)
  closed = new Uint8Array(0)
  epoch = 0
  inUse = false

  ensure (n: number): void {
    if (n > this.size) {
      const cap = Math.ceil(n * 1.2)
      this.g = new Float64Array(cap)
      this.parent = new Int32Array(cap)
      this.meta = new Uint8Array(cap)
      this.stamp = new Int32Array(cap)
      this.closed = new Uint8Array(cap)
      this.size = cap
      // Epoch stays MONOTONIC across growth (fresh stamp arrays are zeroed,
      // so old epochs can't collide) — resetting it would let a finished
      // solver's stamp guard accept a later solve's tables as its own.
    }
    this.epoch++
    if (this.epoch === 0x7fffffff) {
      this.stamp.fill(0)
      this.epoch = 1
    }
  }
}

const arenaPool: Arena[] = []
const MAX_POOLED_ARENAS = 4

function acquireArena (n: number): Arena {
  let arena = arenaPool.find(a => !a.inUse && a.size >= n) ?? arenaPool.find(a => !a.inUse)
  if (!arena) {
    arena = new Arena()
    if (arenaPool.length < MAX_POOLED_ARENAS) arenaPool.push(arena)
  }
  arena.inUse = true
  arena.ensure(n)
  return arena
}

// Abandoned (never-finished) solvers release their arena on GC.
const arenaFinalizer = new FinalizationRegistry<Arena>((arena) => {
  arena.inUse = false
})

export class Solver {
  private readonly snap: SnapshotView
  private readonly moveGen: MoveGen
  private readonly goal: GoalEvaluator
  private readonly heap = new MinHeap(4096)
  private readonly startTime: number
  private readonly timeout: number
  private readonly maxCost: number
  private readonly cancelFlag: Int32Array | null
  private readonly myEpoch: number

  private readonly w: number
  private readonly l: number
  private readonly x0: number
  private readonly y0: number
  private readonly z0: number

  private readonly arena: Arena
  private bestIdx: number
  private bestH: number
  private readonly startIdx: number
  private visited = 0
  private openCount = 0
  private readonly chunkSet = new Set<number>()
  private readonly chunkList: Array<[number, number]> = []
  private done = false
  /** Terminal result snapshot — compute() after done must never re-read the
   * (released, possibly re-acquired) arena. */
  private finalResult: RawSolveResult | null = null
  /** Per-node toBreak (cell indices) for canDig solves; lazily created. */
  private breaks: Map<number, number[]> | null = null

  constructor (
    snap: SnapshotView,
    cfg: MovementsConfig,
    goal: GoalEvaluator,
    start: { x: number, y: number, z: number },
    opts: SolveOptions,
    stepExclusion: StepExclusionFn = null,
    dig: DigContext | null = null
  ) {
    this.snap = snap
    this.goal = goal
    this.moveGen = new MoveGen(snap, cfg, stepExclusion, dig)
    this.startTime = performance.now()
    this.timeout = opts.timeout
    this.cancelFlag = opts.cancelFlag ?? null

    const m = snap.meta
    this.w = m.w
    this.l = m.l
    this.x0 = m.x0
    this.y0 = m.y0
    this.z0 = m.z0

    const n = m.w * m.h * m.l
    const arena = acquireArena(n)
    this.arena = arena
    this.myEpoch = arena.epoch
    arenaFinalizer.register(this, arena, this)

    const sIdx = this.moveGen.cellIndex(start.x, start.y, start.z)
    const h0 = goal.heuristic(start.x, start.y, start.z)
    this.maxCost = opts.searchRadius < 0 ? -1 : h0 + opts.searchRadius
    this.bestH = h0
    this.startIdx = sIdx
    this.bestIdx = sIdx

    if (sIdx >= 0) {
      arena.stamp[sIdx] = this.myEpoch
      arena.closed[sIdx] = 0
      arena.g[sIdx] = 0
      arena.parent[sIdx] = -1
      arena.meta[sIdx] = 0
      this.heap.push(sIdx, h0)
      this.openCount = 1
    }
    // Start outside the snapshot: shouldn't happen (the box is built around
    // the start), degrade to an immediate boundary-limited noPath.
  }

  private finish (result: RawSolveResult): RawSolveResult {
    if (!this.done) {
      this.done = true
      this.finalResult = result
      this.arena.inUse = false
      arenaFinalizer.unregister(this)
    }
    return result
  }

  /** Upstream-parity visitedChunks (strings), for astarContext consumers. */
  get visitedChunks (): Set<string> {
    const out = new Set<string>()
    for (const [cx, cz] of this.chunkList) out.add(`${cx},${cz}`)
    return out
  }

  private decodeX (idx: number): number {
    return (idx % this.w) + this.x0
  }

  private decodeZ (idx: number): number {
    return (Math.floor(idx / this.w) % this.l) + this.z0
  }

  private decodeY (idx: number): number {
    return Math.floor(idx / (this.w * this.l)) + this.y0
  }

  private makeResult (status: SolveStatus, node: number, cancelled = false): RawSolveResult {
    const arena = this.arena
    const path: RawPathNode[] = []
    if (node >= 0) {
      let cur = node
      while (cur >= 0 && arena.stamp[cur] === this.myEpoch && arena.parent[cur] >= 0) {
        const parent = arena.parent[cur]
        const meta = arena.meta[cur]
        const x = this.decodeX(cur)
        const y = this.decodeY(cur)
        const z = this.decodeZ(cur)
        const node: RawPathNode = {
          x,
          y,
          z,
          cost: arena.g[cur] - arena.g[parent],
          parkour: (meta & META_PARKOUR) !== 0,
          useOne: (meta & META_USEONE) !== 0 ? { x, y, z } : null
        }
        const breakCells = this.breaks?.get(cur)
        if (breakCells !== undefined) {
          node.toBreak = breakCells.map(idx => ({
            x: this.decodeX(idx),
            y: this.decodeY(idx),
            z: this.decodeZ(idx)
          }))
        }
        path.push(node)
        cur = parent
      }
      path.reverse()
    }
    return {
      status,
      cancelled,
      cost: node >= 0 && arena.stamp[node] === this.myEpoch ? arena.g[node] : 0,
      time: performance.now() - this.startTime,
      visitedNodes: this.visited,
      generatedNodes: this.visited + this.openCount,
      path,
      touchedChunks: this.chunkList.slice(),
      boundaryLimited: this.moveGen.boundaryTouched
    }
  }

  /**
   * Run the search for up to `sliceMs` (upstream tickTimeout semantics).
   * Returns 'partial' when the slice ran out, 'timeout' when the total
   * budget ran out, 'success'/'noPath' when finished. Resumable until a
   * terminal status is returned.
   */
  compute (sliceMs: number): RawSolveResult {
    if (this.done) {
      // Never re-read the arena after release — it may belong to another
      // solve by now. Return the snapshotted terminal result.
      return this.finalResult ?? this.makeEmptyResult('noPath')
    }
    if (this.startIdx < 0) {
      return this.finish(this.makeEmptyResult('noPath'))
    }
    const computeStart = performance.now()
    const heap = this.heap
    const goal = this.goal
    const moveGen = this.moveGen
    const arena = this.arena
    const gAll = arena.g
    const parentAll = arena.parent
    const metaAll = arena.meta
    const stampAll = arena.stamp
    const closedAll = arena.closed
    const myEpoch = this.myEpoch
    const maxCost = this.maxCost
    let sinceCheck = 0

    while (!heap.isEmpty()) {
      if (++sinceCheck >= 32) {
        sinceCheck = 0
        const now = performance.now()
        if (this.cancelFlag !== null && Atomics.load(this.cancelFlag, 0) !== 0) {
          return this.finish(this.makeResult('timeout', this.bestIdx, true))
        }
        if (now - computeStart > sliceMs) {
          return this.makeResult('partial', this.bestIdx)
        }
        if (now - this.startTime > this.timeout) {
          return this.finish(this.makeResult('timeout', this.bestIdx))
        }
      }

      const idx = heap.pop()
      if (stampAll[idx] !== myEpoch || closedAll[idx] !== 0) continue // stale duplicate

      const x = this.decodeX(idx)
      const y = this.decodeY(idx)
      const z = this.decodeZ(idx)

      // Upstream checks isEnd on pop, before closing.
      if (goal.isEnd(x, y, z)) {
        return this.finish(this.makeResult('success', idx))
      }

      closedAll[idx] = 1
      this.visited++
      this.openCount--

      const chunkKey = (x >> 4) * 4194304 + (z >> 4)
      if (!this.chunkSet.has(chunkKey)) {
        this.chunkSet.add(chunkKey)
        this.chunkList.push([x >> 4, z >> 4])
      }

      moveGen.generate(x, y, z)
      const count = moveGen.outCount
      const outIdx = moveGen.outIdx
      const outX = moveGen.outX
      const outY = moveGen.outY
      const outZ = moveGen.outZ
      const outCost = moveGen.outCost
      const outMeta = moveGen.outMeta
      const outBreaks = moveGen.outBreaks
      const g = gAll[idx]

      for (let i = 0; i < count; i++) {
        const nIdx = outIdx[i]
        const touched = stampAll[nIdx] === myEpoch

        const g2 = g + outCost[i]
        const h = goal.heuristic(outX[i], outY[i], outZ[i])
        if (maxCost > 0 && g2 + h > maxCost) continue

        if (touched) {
          if (closedAll[nIdx] !== 0) {
            // Improvement over upstream: REOPEN a closed node on a strictly
            // better g. Parkour edges (up to 4 blocks for cost 1) make the
            // octile heuristic inadmissible, so first-closed routes aren't
            // always optimal — upstream locks them in and can return a
            // costlier path; reopening makes ours graph-optimal (never
            // worse than upstream, sometimes cheaper).
            if (gAll[nIdx] <= g2) continue
            closedAll[nIdx] = 0
            this.visited-- // keep visitedNodes ≈ unique closed (upstream parity)
            this.openCount++
          } else {
            // Upstream skips when neighborNode.g < gFromThisNode (strict),
            // so an equal-g route REPLACES the parent — mirror for tie parity.
            if (gAll[nIdx] < g2) continue
          }
        } else {
          stampAll[nIdx] = myEpoch
          closedAll[nIdx] = 0
        }

        gAll[nIdx] = g2
        parentAll[nIdx] = idx
        metaAll[nIdx] = outMeta[i]
        const br = outBreaks[i]
        if (br !== null) {
          (this.breaks ??= new Map()).set(nIdx, br)
        } else if (this.breaks !== null) {
          this.breaks.delete(nIdx)
        }
        if (h < this.bestH) {
          this.bestH = h
          this.bestIdx = nIdx
        }
        if (!touched) this.openCount++
        heap.push(nIdx, g2 + h)
      }
    }

    return this.finish(this.makeResult('noPath', this.bestIdx))
  }

  private makeEmptyResult (status: SolveStatus): RawSolveResult {
    return {
      status,
      cancelled: false,
      cost: 0,
      time: performance.now() - this.startTime,
      visitedNodes: this.visited,
      generatedNodes: this.visited + this.openCount,
      path: [],
      touchedChunks: this.chunkList.slice(),
      boundaryLimited: this.moveGen.boundaryTouched
    }
  }
}
