// The solve pipeline around physics-verified hops (allowParkourPhysics).
//
// A search that FAILS (no path, or out of think time) is handed to the hop
// oracle (hopOracle.ts), which looks for exact-physics hops out of what the
// search reached, from the tail of its best partial path. Each candidate hop
// is judged by what it OPENS: a follow-up search from its landing. A hop that
// reaches the goal wins outright; otherwise the one whose follow-up gets
// strictly closer to the goal than anything reached so far — so a hop into a
// dead end, or onto ground the search already had, is never taken, and the
// goal heuristic is only a tie-break (courses wind; the hop that opens the
// way can lead away from the goal first). The winner is stitched in —
// prefix, hop node carrying its control program, follow-up — and the round
// repeats from its landing until the goal is reached or the budget runs out.
//
// The result is a whole path to the goal or nothing: a stitched path that
// stops short is not returned, so a route the pipeline cannot finish fails
// exactly as it did before.

import type { HopOracle } from './hopOracle.js'
import type { RawPathNode } from './types.js'
import type { RawSolveResult } from './solver.js'

export interface HopPipelineContext {
  oracle: HopOracle
  heuristic: (x: number, y: number, z: number) => number
  /** A full search from `start` within `timeoutMs`; afterwards `reached` answers for it. */
  solve: (start: { x: number, y: number, z: number }, timeoutMs: number) => RawSolveResult
  /** A search from `start` to the one cell `cell` (the way to a vantage take-off); afterwards `reached` answers for it. */
  solveTo?: (start: { x: number, y: number, z: number }, cell: { x: number, y: number, z: number }, timeoutMs: number) => RawSolveResult
  /** Did the LAST solve() reach this cell? */
  reached: (x: number, y: number, z: number) => boolean
  /** The goal's cell, when it has one (a block or near goal): where vantage take-offs are looked for. */
  goal?: { x: number, y: number, z: number } | null
  /** performance.now() deadline for the whole pipeline. */
  deadline: number
}

/** Rounds (hops) one pipeline may stitch. */
const MAX_ROUNDS = 8
/** Nodes at the end of the best partial path the oracle takes off from. */
const SOURCES = 8
/** Candidate hops per round, each judged by a follow-up search. */
const EDGES = 6
/** Share of the remaining budget one discovery may spend. */
const DISCOVER_SHARE = 0.5
/** Least think time a follow-up search is given. */
const FOLLOW_UP_MIN_MS = 20
/**
 * Vantage take-offs (vantages()): reached cells within VANTAGE_REACH of the
 * goal horizontally (a hop's reach, a rebound's, and one more), at least
 * VANTAGE_ABOVE over the partial path's end (above anything a hop from there
 * rises to) and at most VANTAGE_OVER over the goal; the highest of each
 * column, the VANTAGES best.
 */
const VANTAGE_REACH = 16
const VANTAGE_ABOVE = 3
const VANTAGE_OVER = 10
const VANTAGES = 4

type XYZ = { x: number, y: number, z: number }

/**
 * Higher ground the failed search reached near the goal. The partial path
 * ends wherever the heuristic bottomed out — under a goal on a ledge, on the
 * floor right beneath it, where no hop rises far enough — while the way up
 * starts somewhere the search reached but did not end: a ladder's top edge to
 * jump off, onto slime that throws the body up there (the arena's Ten Ways
 * slime room). The highest reached cell of each column round the goal, the
 * nearest and highest first (a block of height is worth a block of reach).
 */
export function vantages (reached: (x: number, y: number, z: number) => boolean, goal: XYZ, end: XYZ, exclude: readonly XYZ[]): XYZ[] {
  const lo = end.y + VANTAGE_ABOVE
  const hi = goal.y + VANTAGE_OVER
  if (lo > hi) return []
  const out: Array<[XYZ, number]> = []
  for (let dx = -VANTAGE_REACH; dx <= VANTAGE_REACH; dx++) {
    for (let dz = -VANTAGE_REACH; dz <= VANTAGE_REACH; dz++) {
      const d = Math.hypot(dx, dz)
      if (d > VANTAGE_REACH) continue
      const x = goal.x + dx
      const z = goal.z + dz
      for (let y = hi; y >= lo; y--) {
        if (!reached(x, y, z)) continue
        if (!exclude.some(e => e.x === x && e.y === y && e.z === z)) out.push([{ x, y, z }, d - (y - goal.y)])
        break
      }
    }
  }
  out.sort((p, q) => p[1] - q[1])
  return out.slice(0, VANTAGES).map(v => v[0])
}

/** PF_HOP_DEBUG=1: one line per pipeline round on stderr (diagnostic). */
const DEBUG = process.env.PF_HOP_DEBUG === '1'

export function stitchHops (first: RawSolveResult, start: XYZ, ctx: HopPipelineContext): RawSolveResult | null {
  const t0 = performance.now()
  const h = (n: XYZ): number => ctx.heuristic(n.x, n.y, n.z)
  const last = (r: RawSolveResult, s: XYZ): XYZ => r.path.length > 0 ? r.path[r.path.length - 1] : s
  const sameCell = (a: XYZ, b: XYZ): boolean => a.x === b.x && a.y === b.y && a.z === b.z
  const stitched: RawPathNode[] = []
  let cur = first
  let curStart = start
  /** The start whose search `reached` currently answers for. */
  let reachedFor: XYZ = start
  let visited = first.visitedNodes
  let generated = first.generatedNodes
  const chunks = new Map<string, [number, number]>()
  const addChunks = (r: RawSolveResult): void => { for (const c of r.touchedChunks) chunks.set(`${c[0]},${c[1]}`, c) }
  addChunks(first)

  for (let round = 0; round < MAX_ROUNDS; round++) {
    if (cur.status === 'success') {
      const path = [...stitched, ...cur.path]
      let cost = 0
      for (const n of path) cost += n.cost
      return {
        status: 'success',
        cancelled: false,
        cost,
        time: first.time + (performance.now() - t0),
        visitedNodes: visited,
        generatedNodes: generated,
        path,
        touchedChunks: [...chunks.values()],
        boundaryLimited: false
      }
    }
    if (performance.now() >= ctx.deadline) return null
    if (!sameCell(reachedFor, curStart)) {
      // the follow-ups after the winner overwrote what `reached` answers for
      cur = ctx.solve(curStart, Math.max(1, ctx.deadline - performance.now()))
      reachedFor = curStart
    }
    const bestH = h(last(cur, curStart))
    const sources: XYZ[] = [curStart, ...cur.path.map(n => ({ x: n.x, y: n.y, z: n.z }))].slice(-SOURCES)
    const budget = Math.max(1, (ctx.deadline - performance.now()) * DISCOVER_SHARE)
    const tD = performance.now()
    let edges = ctx.oracle.discover(sources, ctx.heuristic, budget, ctx.reached, EDGES)
    // and from higher ground the search reached near the goal (vantages)
    const high = ctx.goal != null && ctx.solveTo !== undefined ? vantages(ctx.reached, ctx.goal, last(cur, curStart), sources) : []
    if (high.length > 0) edges = edges.concat(ctx.oracle.discover(high, ctx.heuristic, Math.max(1, (ctx.deadline - performance.now()) * DISCOVER_SHARE), ctx.reached, EDGES))
    // nothing simple opens the way: the yaw family for the most promising few
    if (edges.length === 0) edges = ctx.oracle.discover(sources, ctx.heuristic, Math.max(1, (ctx.deadline - performance.now()) * DISCOVER_SHARE), ctx.reached, EDGES, 'yaw')
    // nor the yaw family: the beam, for the most promising very few
    if (edges.length === 0) edges = ctx.oracle.discover(sources, ctx.heuristic, Math.max(1, (ctx.deadline - performance.now()) * DISCOVER_SHARE), ctx.reached, EDGES, 'beam')
    if (DEBUG) console.warn(`[hops] round ${round} from ${curStart.x},${curStart.y},${curStart.z}: ${cur.status} path ${cur.path.length} best h ${bestH.toFixed(1)}; discover ${edges.length} edges in ${(performance.now() - tD).toFixed(0)} ms (${ctx.oracle.ticksFlown} ticks), budget ${budget.toFixed(0)} ms${high.length > 0 ? `; vantages ${high.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}`)
    let pick: typeof edges[number] | null = null
    let pickRes: RawSolveResult | null = null
    let pickScore = bestH
    for (let ei = 0; ei < edges.length; ei++) {
      const e = edges[ei]
      const left = ctx.deadline - performance.now()
      if (left <= 0) break
      const target = { x: e.program.tx, y: e.program.ty, z: e.program.tz }
      // an even share of what is left, one more kept for the rounds after: a
      // follow-up that floods its box must not starve the candidates after it
      const f = ctx.solve(target, Math.max(FOLLOW_UP_MIN_MS, left / (edges.length - ei + 1)))
      reachedFor = target
      visited += f.visitedNodes
      generated += f.generatedNodes
      const score = f.status === 'success' ? -Infinity : h(last(f, target))
      if (DEBUG) console.warn(`[hops]   cand ${e.fromX},${e.fromY},${e.fromZ} -> ${target.x},${target.y},${target.z} ${e.program.family}: follow-up ${f.status} visited ${f.visitedNodes} best h ${score.toFixed(1)}`)
      if (score < pickScore) {
        pickScore = score
        pick = e
        pickRes = f
      }
    }
    if (pick === null || pickRes === null) return null
    addChunks(pickRes)
    const from = { x: pick.fromX, y: pick.fromY, z: pick.fromZ }
    if (!sameCell(from, curStart)) {
      const at = cur.path.findIndex(n => sameCell(n, from))
      if (at >= 0) {
        stitched.push(...cur.path.slice(0, at + 1))
      } else {
        // a vantage take-off: the way up to it (reached, so there is one)
        if (ctx.solveTo === undefined) return null
        const up = ctx.solveTo(curStart, from, Math.max(FOLLOW_UP_MIN_MS, ctx.deadline - performance.now()))
        reachedFor = curStart
        visited += up.visitedNodes
        generated += up.generatedNodes
        if (up.status !== 'success') return null
        addChunks(up)
        stitched.push(...up.path)
      }
    }
    const p = pick.program
    stitched.push({
      x: p.tx,
      y: p.ty,
      z: p.tz,
      // priced like the table prices a jump: blocks of distance plus the pad
      cost: Math.hypot(p.tx - from.x, p.tz - from.z) + 0.5 + Math.max(0, p.ty - from.y),
      parkour: true,
      useOne: null,
      program: p
    })
    cur = pickRes
    curStart = { x: p.tx, y: p.ty, z: p.tz }
  }
  return null
}
