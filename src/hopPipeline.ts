// Judge simulated hops by follow-up search progress, since a useful hop can initially lead away from the goal.
// Keep alternative branches and return a stitched path only when it reaches the goal.

import type { HopOracle } from './hopOracle.js'
import { newBody, SIM } from './playerSim.js'
import type { RawPathNode } from './types.js'
import type { RawSolveResult } from './solver.js'

export interface HopPipelineContext {
  oracle: HopOracle
  heuristic: (x: number, y: number, z: number) => number
  /** Full search from `start` within `timeoutMs`; `reached` then describes this search. */
  solve: (start: { x: number, y: number, z: number }, timeoutMs: number) => RawSolveResult
  /** Search from `start` to `cell` within `timeoutMs`; `reached` then describes this search. */
  solveTo?: (start: { x: number, y: number, z: number }, cell: { x: number, y: number, z: number }, timeoutMs: number) => RawSolveResult
  /** Whether the most recent search reaches this cell. */
  reached: (x: number, y: number, z: number) => boolean
  /** Goal cell, when available, used to find elevated take-offs. */
  goal?: { x: number, y: number, z: number } | null
  /** performance.now() deadline for the whole pipeline. */
  deadline: number
  /** Planner edges as flat x,y,z triples without momentum; with nodeLike, enables backward goal-side search. */
  edgesFrom?: (x: number, y: number, z: number, out: number[]) => void
  nodeLike?: (x: number, y: number, z: number) => boolean
  /** Exact starting feet position; uses region stitching when the body rests somewhere its planner node cannot represent. */
  startRest?: { x: number, y: number, z: number } | null
  /** Ban only the directed move from `from` to `to` in subsequent searches.
   * When absent, planner jumps are not verified. */
  banEdge?: (from: { x: number, y: number, z: number }, to: { x: number, y: number, z: number }) => void
  /** Receives the verified first search when verification changes it, for use if stitching fails. */
  onVerifiedFirst?: (r: RawSolveResult) => void
  /** Receives a verified prefix to a new stable landing after STREAM_AFTER_MS while stitching continues. */
  onStitched?: (path: RawPathNode[]) => void
  /** Whether the host cancels the solve; stops the pipeline immediately. */
  cancelled?: () => boolean
}

/** Verification budgets and ban limits for planner jumps with narrow take-off or landing footing.
 * A narrow strip may lack room for the planner's assumed run-up or touchdown. */
const VERIFY_TICKS = 200_000
/** Beam budget after family searches from stand points. */
const VERIFY_BEAM_TICKS = 300_000
const VERIFY_NARROW = 0.35
const VERIFY_BANS = 8
const VERIFY_BANS_TOTAL = 16

/** Backward goal-side search bounds: hop reach, vertical range, generated cells and budget share. */
const GOAL_SIDE_REACH_SQ = 6.5 * 6.5
const GOAL_SIDE_UP = 8
const GOAL_SIDE_DOWN = 2
const GOAL_SIDE_GEN = 4000
const GOAL_SIDE_SHARE = 0.15

/** Cells from which planner edges reach the goal, found by backward closure excluding the failed search's reach.
 * Goal-side hops bypass heuristic retreat limits because their follow-up can complete the route.
 * Returns null without a goal cell or edge generator. */
export type GoalSide = ((x: number, y: number, z: number) => boolean) & { cells: XYZ[] }
export function goalSide (ctx: HopPipelineContext, deadline: number): GoalSide | null {
  const g = ctx.goal
  if (g == null || ctx.edgesFrom === undefined || ctx.nodeLike === undefined) return null
  const key = (x: number, y: number, z: number): number => ((x & 0x3ff) << 20) | ((y & 0x3ff) << 10) | (z & 0x3ff)
  const side = new Set<number>([key(g.x, g.y, g.z)])
  const queue: XYZ[] = [g]
  const edges = new Map<number, number[]>()
  const out: number[] = []
  const has = (x: number, y: number, z: number): boolean => side.has(key(x, y, z))
  let gen = 0
  for (let qi = 0; qi < queue.length; qi++) {
    const t = queue[qi]
    for (let dy = -GOAL_SIDE_DOWN; dy <= GOAL_SIDE_UP; dy++) {
      for (let dx = -6; dx <= 6; dx++) {
        for (let dz = -6; dz <= 6; dz++) {
          if (dx * dx + dz * dz > GOAL_SIDE_REACH_SQ) continue
          const x = t.x + dx
          const y = t.y + dy
          const z = t.z + dz
          const k = key(x, y, z)
          if (side.has(k)) continue
          let e = edges.get(k)
          if (e === undefined) {
            if (ctx.reached(x, y, z) || !ctx.nodeLike(x, y, z)) { edges.set(k, []); continue }
            if (gen >= GOAL_SIDE_GEN || performance.now() >= deadline) return Object.assign(has, { cells: queue })
            out.length = 0
            ctx.edgesFrom(x, y, z, out)
            gen++
            e = out.slice()
            edges.set(k, e)
          }
          for (let i = 0; i < e.length; i += 3) {
            if (e[i] === t.x && e[i + 1] === t.y && e[i + 2] === t.z) {
              side.add(k)
              queue.push({ x, y, z })
              break
            }
          }
        }
      }
    }
  }
  if (DEBUG) console.warn(`[hops] goal side ${side.size} cells, ${gen} generated`)
  return Object.assign(has, { cells: queue })
}

/** Rounds (hops) one pipeline may stitch. */
const MAX_ROUNDS = 8
/** New best progress refreshes the remaining time to PROGRESS_STEP_MS, capped at PROGRESS_MAX_MS beyond the initial deadline.
 * Only a new best heuristic improvement of at least PROGRESS_MIN earns more time. */
const PROGRESS_MIN = 0.5
const PROGRESS_STEP_MS = 2500
const PROGRESS_MAX_MS = 6000
/** Budget share for ice carry chains; a stationary body on an ice pad may have no viable run-up. */
const CHAIN_SHARE = 0.85
/** Kernel-work limit for a chain so a failing search cannot consume the remaining deadline. */
const CHAIN_TICKS = 1_000_000
/** Maximum region-chain rounds; each hop can move between places that share one planner node. */
const MAX_REGION_ROUNDS = 20
/** How much further from the goal (heuristic) a region hop may land and still be kept: a pane crossed inside a hoop gets no closer. */
const REGION_SLACK = 1
/** Nodes at the end of the best partial path the oracle takes off from. */
const SOURCES = 8
/** Streaming delay in milliseconds; a quicker pipeline returns the whole path before a prefix is useful. */
const STREAM_AFTER_MS = 150
/** Candidate hops per round, each judged by a follow-up search. */
const EDGES = 6
/** Candidate limit per region round; best-first expansion follows the best while alternatives consume discovery time. */
const REGION_EDGES = 2
/** Share of the remaining budget one discovery may spend. */
const DISCOVER_SHARE = 0.5
/** Least think time a follow-up search is given. */
const FOLLOW_UP_MIN_MS = 20
/** Vantage bounds around the goal and above the partial-path end, in blocks.
 * Keep the highest reached cell per column, capped at VANTAGES. */
const VANTAGE_REACH = 16
const VANTAGE_ABOVE = 3
const VANTAGE_OVER = 10
const VANTAGES = 4

type XYZ = { x: number, y: number, z: number }

/** Elevated take-offs near the goal can enable hops that the partial path's lower endpoint cannot reach.
 * Returns the highest reached cell per column, ranked by proximity and height. */
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

/** Limit drop-edge take-offs and their discovery budget share; try bounce searches until the first edge. */
const DROP_TAKEOFFS = 4
const DROP_SHARE = 0.4
/** The slime ring a drop take-off is looked for in (hopOracle.ts BOUNCE_REACH). */
const DROP_RING = 7
/** How high above the highest source a drop take-off may be (hopOracle.ts HOP_RISE). */
const DROP_RISE = 2

/** Reached ledges above nearby slime, ranked nearest the slime first.
 * The best partial path can pass a useful drop edge without including it among its take-off sources. */
export function dropTakeoffs (oracle: HopOracle, reached: (x: number, y: number, z: number) => boolean, sources: readonly XYZ[]): XYZ[] {
  const tops = new Map<string, [number, number, number]>()
  for (const src of sources) for (const t of oracle.slimeTops(src.x, src.y, src.z)) tops.set(`${t[0]},${t[1]},${t[2]}`, t)
  if (tops.size === 0) return []
  const T = [...tops.values()]
  const lo = Math.min(...T.map(t => t[1])) + 3
  const hi = Math.max(...sources.map(src => src.y)) + DROP_RISE
  const out: Array<[XYZ, number]> = []
  const seenCol = new Set<string>()
  for (const t of T) {
    for (let dx = -DROP_RING; dx <= DROP_RING; dx++) {
      for (let dz = -DROP_RING; dz <= DROP_RING; dz++) {
        const x = t[0] + dx
        const z = t[2] + dz
        const col = `${x},${z}`
        if (seenCol.has(col)) continue
        seenCol.add(col)
        for (let y = hi; y >= lo; y--) {
          if (!reached(x, y, z)) continue
          if (sources.some(src => src.x === x && src.y === y && src.z === z)) break
          if (!oracle.openDrop(x, y, z)) break
          let near = Infinity
          for (const u of T) near = Math.min(near, Math.hypot(u[0] - x, u[2] - z))
          if (near <= DROP_RING) out.push([{ x, y, z }, near])
          break
        }
      }
    }
  }
  out.sort((a, b) => a[1] - b[1])
  return out.slice(0, DROP_TAKEOFFS).map(o => o[0])
}

/** Frontier bounds around the partial-path tail; test the lowest-heuristic standable cells for openings. */
const FRONTIERS = 4
/** Cap untested goal-side take-offs so they cannot crowd out sources with verified openings. */
const SIDE_FRONTIERS = process.env.PF_SIDE_FRONTIERS !== undefined ? Number(process.env.PF_SIDE_FRONTIERS) : 2
const FRONTIER_SCAN = 8
const FRONTIER_DOWN = 6
const FRONTIER_UP = 4
const FRONTIER_TESTS = 48

/** Reached take-offs near the tail that overlook unreached landing ground, ranked nearest the goal first.
 * Separate sources by a column or two levels to cover ledges and detours the partial path misses. */
export function frontierSources (oracle: HopOracle, reached: (x: number, y: number, z: number) => boolean, heuristic: (x: number, y: number, z: number) => number, sources: readonly XYZ[], side: GoalSide | null = null): XYZ[] {
  if (sources.length === 0) return []
  let x0 = Infinity; let x1 = -Infinity; let y0 = Infinity; let y1 = -Infinity; let z0 = Infinity; let z1 = -Infinity
  for (const s of sources) {
    x0 = Math.min(x0, s.x); x1 = Math.max(x1, s.x)
    y0 = Math.min(y0, s.y); y1 = Math.max(y1, s.y)
    z0 = Math.min(z0, s.z); z1 = Math.max(z1, s.z)
  }
  const cells: Array<[number, number, number, number]> = []
  for (let x = x0 - FRONTIER_SCAN; x <= x1 + FRONTIER_SCAN; x++) {
    for (let z = z0 - FRONTIER_SCAN; z <= z1 + FRONTIER_SCAN; z++) {
      for (let y = y0 - FRONTIER_DOWN; y <= y1 + FRONTIER_UP; y++) {
        if (reached(x, y, z)) cells.push([x, y, z, heuristic(x, y, z)])
      }
    }
  }
  cells.sort((p, q) => p[3] - q[3])
  const picks: XYZ[] = []
  const apart = (x: number, y: number, z: number, p: XYZ): boolean => Math.max(Math.abs(p.x - x), Math.abs(p.z - z)) > 1 || Math.abs(p.y - y) >= 2
  let tests = 0
  // Prioritize sources within hop range of a goal-side cell, since such a hop can complete the route.
  if (side != null && side.cells.length > 1) {
    for (const [x, y, z] of cells) {
      if (picks.length >= SIDE_FRONTIERS || tests >= FRONTIER_TESTS) break
      if (!sources.every(p => apart(x, y, z, p)) || !picks.every(p => apart(x, y, z, p))) continue
      if (!side.cells.some(c => (c.x - x) ** 2 + (c.z - z) ** 2 <= GOAL_SIDE_REACH_SQ && c.y - y <= 2 && c.y - y >= -6 && !reached(c.x, c.y, c.z))) continue
      if (oracle.stands(x, y, z).length === 0) continue
      tests++
      picks.push({ x, y, z })
    }
  }
  for (const [x, y, z] of cells) {
    if (picks.length >= FRONTIERS || tests >= FRONTIER_TESTS) break
    if (picks.some(p => p.x === x && p.y === y && p.z === z)) continue
    if (!sources.every(p => apart(x, y, z, p)) || !picks.every(p => apart(x, y, z, p))) continue
    if (oracle.stands(x, y, z).length === 0) continue
    tests++
    if (oracle.opens(x, y, z, heuristic, reached, side)) picks.push({ x, y, z })
  }
  return picks
}

/** Walkable prefix ending before the earliest candidate off-path take-off, or the whole partial path if none.
 * The host still cuts at one-way drops and never emits an empty prefix. */
export function streamablePrefix (first: RawSolveResult, start: XYZ, oracle: HopOracle, reached: (x: number, y: number, z: number) => boolean, goal: XYZ | null): RawPathNode[] {
  const path = first.path
  if (path.length === 0) return []
  const sources: XYZ[] = [start, ...path.map(n => ({ x: n.x, y: n.y, z: n.z }))].slice(-SOURCES)
  const end = path[path.length - 1]
  const high = goal !== null ? vantages(reached, goal, end, sources) : []
  const cands = [...high, ...vantages(reached, end, end, [...sources, ...high]), ...dropTakeoffs(oracle, reached, sources)]
  if (cands.length === 0) return path.slice()
  // Cut at the earliest of each candidate's nearest path point, including the start.
  // Choosing only the single nearest pair can walk the bot past another usable take-off.
  let cut = path.length
  for (const c of cands) {
    let at = 0
    let best = Math.hypot(start.x - c.x, start.y - c.y, start.z - c.z)
    for (let i = 0; i < path.length; i++) {
      const d = Math.hypot(path[i].x - c.x, path[i].y - c.y, path[i].z - c.z)
      if (d < best) { best = d; at = i }
    }
    if (at < cut) cut = at
  }
  return path.slice(0, cut)
}

/** Enables per-round hop-pipeline diagnostics on stderr. */
const DEBUG = process.env.PF_HOP_DEBUG === '1'

/** Stitched prefix, follow-up and next discovery level: 0 elevated/drop sources, 1 deferred tail,
 * 2 frontier and goal-side sources, 3 yaw/kick then pivot searches, 4 beam.
 * Cheaper levels run first so expensive searches do not exhaust every alternative. */
interface StitchState {
  stitched: RawPathNode[]
  cur: RawSolveResult
  curStart: XYZ
  score: number
  depth: number
  level: number
  tailSkipped: boolean
  /** Exact rest for region stitching, where the planner node can represent a different footing.
   * Expanded from this rest; null selects ordinary stitching. */
  rest: { x: number, y: number, z: number } | null
  /** The rest is in an under region: no planner follow-up from it. */
  under: boolean
  /** Fallback tier, tried after ordinary states when the starting rest differs from the start cell's footing. */
  tier: number
  /** Level 1 tries elevated and drop sources before the tail when level 0 uses an ice carry chain. */
  deferL0?: boolean
  /** Deferred drop take-offs at the parent's level and score; try them after closer level-0 states.
   * This preserves time for progress through elevated hops. */
  dropsOnly?: boolean
  /** A state whose follow-up reached the goal: the whole path's time (stitchTime), what the successes of one round are ranked by. */
  time?: number
}

/** Ticks a unit of planner cost stands for (a block at a sprint): stitchTime. */
const TICKS_PER_COST = 3.6

/** Estimated stitched duration in planner-cost units, combining program ticks and ordinary path costs.
 * Ranks successful candidates so discovery order does not choose a slower route. */
function stitchTime (path: ReadonlyArray<RawPathNode>): number {
  let t = 0
  for (const n of path) t += n.program !== null && n.program !== undefined ? n.program.ticks / TICKS_PER_COST : n.cost
  return t
}

/** Best-first hop stitching keeps progressing alternatives available if the closest branch dead-ends.
 * Deferred discovery levels try cheap hops across states before expensive searches. */
export function stitchHops (first: RawSolveResult, start: XYZ, ctx: HopPipelineContext): RawSolveResult | null {
  const t0 = performance.now()
  // Verified re-solves count inside pipeline time; only the original search time is added separately.
  const firstTime = first.time
  const h = (n: XYZ): number => ctx.heuristic(n.x, n.y, n.z)
  const last = (r: RawSolveResult, at: XYZ): XYZ => r.path.length > 0 ? r.path[r.path.length - 1] : at
  const hardDeadline = ctx.deadline + PROGRESS_MAX_MS
  let bestSeen = h(last(first, start))
  /** The quickest stitched path that reached the goal so far (stitchTime): a slower success is not pushed. */
  let bestDone = Infinity
  const sameCell = (a: XYZ, b: XYZ): boolean => a.x === b.x && a.y === b.y && a.z === b.z
  /** The start whose whole search `reached` answers for (null: a search to one cell ran last). */
  let reachedFor: XYZ | null = start
  let visited = first.visitedNodes
  let generated = first.generatedNodes
  const chunks = new Map<string, [number, number]>()
  const addChunks = (r: RawSolveResult): void => { for (const c of r.touchedChunks) chunks.set(`${c[0]},${c[1]}`, c) }
  addChunks(first)
  /** Whether take-off or landing stands span less than VERIFY_NARROW along the jump.
   * Returns false if either has no stands, since the oracle lacks a footing model. */
  const narrowJump = (from: XYZ, n: XYZ): boolean => {
    const along = Math.abs(n.x - from.x) >= Math.abs(n.z - from.z)
    const span = (c: XYZ): number => {
      let lo = Infinity
      let hi = -Infinity
      for (const s of ctx.oracle.stands(c.x, c.y, c.z)) {
        if (s.catch) continue
        lo = Math.min(lo, along ? s.x : s.z)
        hi = Math.max(hi, along ? s.x : s.z)
      }
      return hi - lo
    }
    const a = span(from)
    const b = span(n)
    return a >= 0 && b >= 0 && (a < VERIFY_NARROW || b < VERIFY_NARROW)
  }
  /** Planner jumps verifiedPath has flown (they land) or banned, by from:to cells. */
  const verifiedJumps = new Set<string>()
  let bans = 0
  /** Verify planner jumps on narrow footing, ban proven failures and re-solve the path.
   * Try the known rest first, then stand points; budget exhaustion alone is no verdict.
   * Skip momentum chains and slime bounces because their starts depend on the preceding flight. */
  const verifiedPath = (from: XYZ, rest: XYZ | null, r: RawSolveResult, again: () => RawSolveResult): RawSolveResult => {
    if (ctx.banEdge === undefined) return r
    const o = ctx.oracle
    // Track both a landed flight and an exhausted search; a timeout alone cannot disprove the jump.
    const attempt = (ticks: number, fly: () => unknown): { found: boolean, complete: boolean } => {
      o.tickLimit = o.ticksFlown + ticks
      o.noVerdict = false
      try {
        return fly() !== null ? { found: true, complete: true } : { found: false, complete: !o.noVerdict }
      } finally {
        o.tickLimit = 0
      }
    }
    scan: for (let k = 0; k <= VERIFY_BANS; k++) {
      let prev = from
      for (let i = 0; i < r.path.length; i++) {
        const n = r.path[i]
        const p = prev
        prev = n
        if (n.parkour !== true || (n.program !== null && n.program !== undefined)) continue
        if (n.chain === true || (n.via !== null && n.via !== undefined)) continue
        const key = `${p.x},${p.y},${p.z}:${n.x},${n.y},${n.z}`
        if (verifiedJumps.has(key) || !narrowJump(p, n)) continue
        if (k === VERIFY_BANS || bans >= VERIFY_BANS_TOTAL || performance.now() >= ctx.deadline) return r
        const tV = performance.now()
        const fromRest = i === 0 && rest !== null
        let res = { found: false, complete: false }
        let beamed = false
        if (fromRest) {
          const at = rest as XYZ
          const b = newBody(at.x, at.y, at.z)
          b.onGround = true
          b.vy = -SIM.gravity * SIM.airdrag
          res = attempt(VERIFY_TICKS, () => o.hopFrom(b, [n.x, n.y, n.z], ctx.deadline))
        }
        if (!res.found) {
          // Completed family searches give a verdict; try the beam only when those searches exhaust their budget.
          res = attempt(VERIFY_TICKS, () => o.hop([p.x, p.y, p.z], [n.x, n.y, n.z], ctx.deadline, 'all', true))
          if (!res.found && !res.complete) {
            beamed = true
            res = attempt(VERIFY_BEAM_TICKS, () => o.hop([p.x, p.y, p.z], [n.x, n.y, n.z], ctx.deadline, 'beam'))
          }
        }
        verifiedJumps.add(key)
        const verdict = res.found ? 'flies' : !res.complete ? 'no verdict' : 'none, banned'
        if (DEBUG) {
          const via = fromRest ? ` from ${(rest as XYZ).x.toFixed(2)},${(rest as XYZ).y.toFixed(2)},${(rest as XYZ).z.toFixed(2)}` : ''
          console.warn(`[hops]   verify ${p.x},${p.y},${p.z} -> ${n.x},${n.y},${n.z}${n.run === true ? ' (run)' : ''}${via}: ${verdict}${beamed ? ' (beam)' : ''} (${(performance.now() - tV).toFixed(0)} ms)`)
        }
        if (res.found || !res.complete) continue
        ctx.banEdge(p, n)
        bans++
        r = again()
        visited += r.visitedNodes
        generated += r.generatedNodes
        continue scan
      }
      return r
    }
    return r
  }
  {
    const r0 = first
    first = verifiedPath(start, ctx.startRest ?? null, first, () => ctx.solve(start, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE)))
    if (first !== r0) {
      addChunks(first)
      bestSeen = h(last(first, start))
      ctx.onVerifiedFirst?.(first)
    }
  }
  // Goal-side closure must use the first search's reach, regardless of later follow-ups.
  let side: GoalSide | null | undefined
  const sideOf = (): GoalSide | null => {
    if (side === undefined) {
      const tS = performance.now()
      // Re-run the first search if a follow-up overwrites its reach.
      if (reachedFor === null || !sameCell(reachedFor, start)) {
        const r0 = ctx.solve(start, Math.max(1, ctx.deadline - performance.now()))
        reachedFor = start
        visited += r0.visitedNodes
        generated += r0.generatedNodes
      }
      side = goalSide(ctx, tS + (ctx.deadline - tS) * GOAL_SIDE_SHARE)
    }
    return side
  }
  const frontier: StitchState[] = [{ stitched: [], cur: first, curStart: start, score: h(last(first, start)), depth: 0, level: 0, tailSkipped: false, rest: null, under: false, tier: 0 }]
  /** Region places a region chain has landed in (cell and rest region): each is expanded once. */
  const regionSeen = new Set<string>()
  const placeKey = (c: XYZ, rest: XYZ): string => {
    const own = ctx.oracle.regionOf([c.x, c.y, c.z], rest)
    const i = ctx.oracle.regions(c.x, c.y, c.z).findIndex(r => r.stands.some(q => own.includes(q)))
    return `${c.x},${c.y},${c.z}#${i >= 0 ? i : `${Math.round(rest.x * 16)},${Math.round(rest.y * 16)},${Math.round(rest.z * 16)}`}`
  }
  const noFollowUp = (r: RawSolveResult): RawSolveResult => ({ ...r, status: 'noPath', path: [], visitedNodes: 0, generatedNodes: 0, touchedChunks: [], cost: 0 })
  // Region stitching starts at the actual rest; a planner path beside a rail can refer to different footing.
  if (ctx.startRest != null) {
    const r = ctx.startRest
    const hw = ctx.oracle.halfWidth
    // Prefer the start column for an overhanging body before considering other support columns.
    const cols: Array<[number, number]> = []
    for (let cx = Math.floor(r.x - hw); cx <= Math.floor(r.x + hw); cx++) for (let cz = Math.floor(r.z - hw); cz <= Math.floor(r.z + hw); cz++) cols.push([cx, cz])
    const rank = (c: [number, number]): number => (c[0] === start.x && c[1] === start.z ? -1 : Math.hypot(c[0] + 0.5 - r.x, c[1] + 0.5 - r.z))
    cols.sort((a, b) => rank(a) - rank(b))
    for (const [cx, cz] of cols) {
      const node = { x: cx, y: ctx.oracle.nodeY(cx, r.y, cz), z: cz }
      const regs = ctx.oracle.regions(node.x, node.y, node.z)
      const at = regs.find(g => Math.abs(g.y - r.y) < 1e-3 && !g.stands[0].catch && g.stands.some(q => ctx.oracle.walkLink(r, q.x, q.z)))
      if (at === undefined) continue
      const rest = { x: r.x, y: r.y, z: r.z }
      if (at.under) {
        // The planner path starts at different footing from this under-region rest.
        frontier[0] = { stitched: [], cur: noFollowUp(first), curStart: node, score: h(node), depth: 0, level: 0, tailSkipped: false, rest, under: true, tier: 0 }
        reachedFor = null
        regionSeen.add(placeKey(node, r))
      } else if (ctx.oracle.underStands(node.x, node.y, node.z).length > 0) {
        // Keep both planner and region continuations when a rail's sides provide separate footing.
        frontier.push({ ...frontier[0], curStart: node, score: h(node), rest, tier: 0 })
        regionSeen.add(placeKey(node, r))
      } else if (regs.length > 1 || !sameCell(node, start)) {
        // Use the rest's region as a fallback when the start cell represents different footing.
        frontier.push({ ...frontier[0], curStart: node, score: h(node), rest, tier: 1 })
      }
      break
    }
  }
  const tried = new Set<string>()
  let expansions = 0
  /** Best heuristic reached by a streamed prefix and the hops already streamed. */
  let bestStreamed = Infinity
  let streamedHops: RawPathNode[] = []
  while (frontier.length > 0) {
    frontier.sort((a, b) => a.tier - b.tier || a.level - b.level || a.score - b.score || (a.time ?? 0) - (b.time ?? 0))
    const st = frontier.shift() as StitchState
    let cur = st.cur
    const curStart = st.curStart
    if (cur.status === 'success') {
      const path = [...st.stitched, ...cur.path]
      let cost = 0
      for (const n of path) cost += n.cost
      return {
        status: 'success',
        cancelled: false,
        cost,
        time: firstTime + (performance.now() - t0),
        visitedNodes: visited,
        generatedNodes: generated,
        path,
        touchedChunks: [...chunks.values()],
        boundaryLimited: false
      }
    }
    if (performance.now() >= ctx.deadline || ctx.cancelled?.() === true) return null
    // Region states use only simple families from their exact rest.
    if (st.depth >= (st.rest !== null ? MAX_REGION_ROUNDS : MAX_ROUNDS) || st.level > (st.rest !== null ? 0 : 4)) continue
    if (st.level === 1 && !st.tailSkipped) {
      frontier.push({ ...st, level: 2 })
      continue
    }
    const tkey = `${st.rest !== null ? placeKey(curStart, st.rest) : `${curStart.x},${curStart.y},${curStart.z}`}:${st.level}${st.dropsOnly === true ? 'drops' : ''}`
    if (tried.has(tkey)) continue
    tried.add(tkey)
    expansions++
    // Compute goal-side candidates before a re-solve can overwrite the first search's reach.
    const sideNow = st.rest === null && (st.level === 2 || st.level === 3) ? sideOf() : null
    if (!st.under && (reachedFor === null || !sameCell(reachedFor, curStart))) {
      // Restore reach after follow-ups, using only a budget share so a flooding search cannot consume all remaining time.
      const resolve = (): RawSolveResult => ctx.solve(curStart, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE))
      cur = resolve()
      reachedFor = curStart
      visited += cur.visitedNodes
      generated += cur.generatedNodes
      cur = verifiedPath(curStart, st.rest, cur, resolve)
    }
    const bestH = h(last(cur, curStart))
    const sources: XYZ[] = [curStart, ...cur.path.map(n => ({ x: n.x, y: n.y, z: n.z }))].slice(-SOURCES)
    const share = (): number => Math.max(1, (ctx.deadline - performance.now()) * DISCOVER_SHARE)
    const tD = performance.now()
    let edges: ReturnType<HopOracle['discover']> = []
    let high: XYZ[] = []
    let drops: XYZ[] = []
    let front: XYZ[] = []
    let tailSkipped = st.tailSkipped
    let deferNext = false
    const vantageEdges = (): ReturnType<HopOracle['discover']> => {
      if (ctx.solveTo !== undefined) {
        const end = last(cur, curStart)
        if (ctx.goal != null) high = vantages(ctx.reached, ctx.goal, end, sources)
        high = high.concat(vantages(ctx.reached, end, end, [...sources, ...high]))
      }
      return high.length > 0 ? ctx.oracle.discover(high, ctx.heuristic, share(), ctx.reached, EDGES) : []
    }
    const dropEdges = (): ReturnType<HopOracle['discover']> => {
      let found: ReturnType<HopOracle['discover']> = []
      if (ctx.solveTo !== undefined) drops = dropTakeoffs(ctx.oracle, ctx.reached, sources)
      if (drops.length > 0) {
        const until = performance.now() + share() * DROP_SHARE
        for (let di = 0; di < drops.length && performance.now() < until; di++) {
          found = found.concat(ctx.oracle.discover([drops[di]], ctx.heuristic, Math.max(1, until - performance.now()), ctx.reached, 1, 'bounce'))
        }
      }
      return found
    }
    const offPath = (): ReturnType<HopOracle['discover']> => vantageEdges().concat(dropEdges())
    if (st.dropsOnly === true) {
      edges = dropEdges()
    } else if (st.rest !== null) {
      // Only use planner reach from this start when skipping already reached region landings.
      const skipTop = !st.under && reachedFor !== null && sameCell(reachedFor, curStart) ? ctx.reached : null
      edges = ctx.oracle.discoverFromRest(curStart, st.rest, ctx.heuristic, share(), skipTop, REGION_EDGES, 'simple')
    } else if (st.level === 0) {
      // Ice gaps can require momentum from the last firm node; a hop from rest on ice loses that run-up.
      // Require a parkour leg so an ordinary ice-floor endpoint does not trigger a carry chain.
      const all = [curStart, ...cur.path.map(n => ({ x: n.x, y: n.y, z: n.z }))]
      const ci = ctx.oracle.chainStart(all)
      if (ci >= 0 && cur.path.slice(ci).some(n => n.parkour === true)) {
        ctx.oracle.tickLimit = CHAIN_TICKS
        try {
          edges = ctx.oracle.chain(all[ci], all.slice(ci + 1), ctx.heuristic, Math.max(1, (ctx.deadline - performance.now()) * CHAIN_SHARE), ctx.reached, 1)
        } finally {
          ctx.oracle.tickLimit = 0
        }
      }
      if (edges.length > 0) {
        tailSkipped = true
        deferNext = true
      } else {
        // Defer drop searches while elevated hops continue to make progress.
        edges = vantageEdges()
        if (edges.length > 0) {
          if (ctx.solveTo !== undefined && dropTakeoffs(ctx.oracle, ctx.reached, sources).length > 0) {
            frontier.push({ ...st, cur, level: 0, dropsOnly: true, deferL0: undefined })
          }
        } else {
          edges = dropEdges()
        }
        tailSkipped = edges.length > 0
        if (!tailSkipped) edges = ctx.oracle.discover(sources, ctx.heuristic, share(), ctx.reached, EDGES)
      }
    } else if (st.level === 1) {
      if (st.deferL0 === true) edges = offPath()
      if (tailSkipped) edges = edges.concat(ctx.oracle.discover(sources, ctx.heuristic, share(), ctx.reached, EDGES))
    } else if (st.level === 2 || st.level === 3) {
      // Try simple frontier hops before yaw/kick escalation; goal-side landings bypass retreat limits.
      if (ctx.solveTo !== undefined) front = frontierSources(ctx.oracle, ctx.reached, ctx.heuristic, sources, sideNow)
      if (st.level === 2) {
        if (front.length > 0) edges = ctx.oracle.discover(front, ctx.heuristic, share(), ctx.reached, EDGES, 'simple', sideNow)
      } else {
        edges = ctx.oracle.discover([...front, ...sources], ctx.heuristic, share(), ctx.reached, EDGES, 'yawKick', sideNow)
        // Precision pivots handle narrow landings when other families find no hop.
        if (edges.length === 0) edges = ctx.oracle.discover(sources.slice(-2), ctx.heuristic, share(), ctx.reached, EDGES, 'pivot')
      }
    } else {
      edges = ctx.oracle.discover(sources, ctx.heuristic, share(), ctx.reached, EDGES, 'beam')
    }
    if (st.rest === null && st.level < 4 && st.dropsOnly !== true) frontier.push({ ...st, cur, level: st.level + 1, tailSkipped, deferL0: deferNext, dropsOnly: undefined })
    if (DEBUG) console.warn(`[hops] L${st.level} #${expansions} depth ${st.depth} from ${curStart.x},${curStart.y},${curStart.z}: ${cur.status} path ${cur.path.length} best h ${bestH.toFixed(1)}; ${edges.length} edges in ${(performance.now() - tD).toFixed(0)} ms (${ctx.oracle.ticksFlown} ticks)${high.length > 0 ? `; vantages ${high.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}${drops.length > 0 ? `; drops ${drops.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}${front.length > 0 ? `; frontier ${front.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}`)
    const seenLand = new Set<string>()
    /** The round's best kept state (ctx.onStitched). */
    let roundBest: StitchState | null = null
    for (let ei = 0; ei < edges.length; ei++) {
      const e = edges[ei]
      const left = ctx.deadline - performance.now()
      if (left <= 0 || ctx.cancelled?.() === true) break
      const target = { x: e.program.tx, y: e.program.ty, z: e.program.tz }
      // Reject take-offs on ice because the required stationary line-up slides.
      if (ctx.oracle.onIce(e.fromX, e.fromY, e.fromZ)) continue
      // A rest beside a thin block below the node has no planner continuation from its actual footing.
      // Hold replay to that region and continue through region hops.
      const rest = e.program.catchTarget ? null : ctx.oracle.restOf(e.program)
      const under = rest !== null && ctx.oracle.besideRest([target.x, target.y, target.z], rest)
      let prog = e.program
      if (under && prog.land === undefined) {
        const held = { ...prog, land: { x: rest.x, y: rest.y, z: rest.z } }
        const ticks = ctx.oracle.verify(held)
        if (ticks < 0) continue
        prog = { ...held, ticks }
      }
      const region = st.rest !== null || under
      // Reserve an extra share for later states so one flooding follow-up cannot starve the remaining candidates.
      let f = under ? noFollowUp(cur) : ctx.solve(target, Math.max(FOLLOW_UP_MIN_MS, left / (edges.length - ei + 1)))
      if (!under) reachedFor = target
      visited += f.visitedNodes
      generated += f.generatedNodes
      // Verify only keepable follow-ups; banning a move cannot improve their heuristic progress.
      if (!under && (f.status === 'success' || h(last(f, target)) < bestH || region)) f = verifiedPath(target, rest, f, () => ctx.solve(target, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) / (edges.length - ei + 1))))
      const score = f.status === 'success' ? -Infinity : h(last(f, target))
      const pk = region && rest !== null ? placeKey(target, rest) : ''
      if (DEBUG) console.warn(`[hops]   cand ${e.fromX},${e.fromY},${e.fromZ} -> ${target.x},${target.y},${target.z} ${prog.family} ${prog.ticks}t${rest !== null ? ` rest ${rest.x.toFixed(2)},${rest.y.toFixed(2)},${rest.z.toFixed(2)}${under ? ' UNDER' : ''}` : ''}: follow-up ${f.status} visited ${f.visitedNodes} best h ${score.toFixed(1)}`)
      // Region chains may trade limited heuristic retreat for a previously unseen place within a node.
      if (!(score < bestH) && !(region && st.rest !== null && score <= bestH + REGION_SLACK && pk !== '' && !regionSeen.has(pk))) continue
      if (pk !== '') regionSeen.add(pk)
      // Follow-ups ending at the same cell open the same ground; retain faster successful paths too.
      const end = last(f, target)
      // An empty planner continuation can still open a new region.
      const lk = pk !== '' && f.path.length === 0 ? pk : `${end.x},${end.y},${end.z}`
      const succeeded = f.status === 'success'
      if (seenLand.has(lk) && !succeeded) continue
      seenLand.add(lk)
      const from = { x: e.fromX, y: e.fromY, z: e.fromZ }
      let prefix: RawPathNode[]
      if (sameCell(from, curStart)) {
        prefix = []
      } else {
        const at = cur.path.findIndex(n => sameCell(n, from))
        if (at >= 0) {
          prefix = cur.path.slice(0, at + 1)
        } else {
          // Off-path take-offs need a verified path from the current start.
          if (ctx.solveTo === undefined) continue
          const solveTo = ctx.solveTo
          const way = (): RawSolveResult => solveTo(curStart, from, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE))
          let up = way()
          reachedFor = null
          visited += up.visitedNodes
          generated += up.generatedNodes
          up = verifiedPath(curStart, st.rest, up, way)
          if (up.status !== 'success') continue
          addChunks(up)
          prefix = up.path
        }
      }
      addChunks(f)
      const pr = prog
      const hopNode: RawPathNode = {
        x: pr.tx,
        y: pr.ty,
        z: pr.tz,
        // Use the planner's jump cost so stitched and ordinary paths remain comparable.
        cost: Math.hypot(pr.tx - from.x, pr.tz - from.z) + 0.5 + Math.max(0, pr.ty - from.y),
        parkour: true,
        useOne: null,
        program: pr
      }
      const stitched = [...st.stitched, ...prefix, hopNode]
      const time = succeeded ? stitchTime(stitched) + stitchTime(f.path) : undefined
      if (time !== undefined) {
        if (time >= bestDone) continue
        bestDone = time
      }
      const pushed: StitchState = { stitched, cur: f, curStart: target, score, depth: st.depth + 1, level: 0, tailSkipped: false, rest: region && rest !== null ? { x: rest.x, y: rest.y, z: rest.z } : null, under, tier: st.tier, time }
      frontier.push(pushed)
      // The host cannot continue an ordinary planner path from a region-only rest.
      if (!succeeded && !region && (roundBest === null || score < roundBest.score)) roundBest = pushed
      if (score < bestSeen - PROGRESS_MIN) {
        bestSeen = score
        ctx.deadline = Math.max(ctx.deadline, Math.min(hardDeadline, performance.now() + PROGRESS_STEP_MS))
      }
    }
    // Stream only the next state to expand, along the branch already streamed, before any full solution exists.
    // The bot may already be following those legs, so another branch must wait for the whole path.
    if (ctx.onStitched !== undefined && roundBest !== null && bestDone === Infinity && roundBest.score < bestStreamed &&
        performance.now() - t0 >= STREAM_AFTER_MS) {
      const order = (a: StitchState, b: StitchState): number => a.tier - b.tier || a.level - b.level || a.score - b.score || (a.time ?? 0) - (b.time ?? 0)
      const nextUp = frontier.reduce((m, s) => order(s, m) < 0 ? s : m, frontier[0])
      const hops = roundBest.stitched.filter(n => n.program !== null && n.program !== undefined)
      if (nextUp === roundBest && hops.length > streamedHops.length && streamedHops.every((q, k) => hops[k] === q)) {
        bestStreamed = roundBest.score
        streamedHops = hops
        // Stream only through the last hop; later states can branch before the follow-up tail ends.
        ctx.onStitched([...roundBest.stitched])
      }
    }
  }
  return null
}
