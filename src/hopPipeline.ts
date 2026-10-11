// The solve pipeline around physics-verified hops (allowParkourPhysics).
//
// A search that FAILS (no path, or out of think time) is handed to the hop
// oracle (hopOracle.ts), which looks for exact-physics hops out of what the
// search reached, from the tail of its best partial path. Each candidate hop
// is judged by what it OPENS: a follow-up search from its landing. A hop that
// reaches the goal wins outright (of several, the quickest whole path:
// stitchTime); otherwise the one whose follow-up gets
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
import { newBody, SIM } from './playerSim.js'
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
  /**
   * The cell planner's edges out of (x, y, z) as flat x,y,z triples (no
   * momentum), and whether a cell is one a body could stand in: with both the
   * pipeline computes the goal side (goalSide). Absent: no goal side.
   */
  edgesFrom?: (x: number, y: number, z: number, out: number[]) => void
  nodeLike?: (x: number, y: number, z: number) => boolean
  /**
   * The body's exact feet position when the solve started (the worker's
   * SolveMessage.startPos): a body beside a pane rail is in a place its start
   * node does not mean, and the stitching starts from that rest (region mode).
   */
  startRest?: { x: number, y: number, z: number } | null
  /**
   * Ban landing cell (x, y, z) for every solve after this one (an entity
   * weight of BAN_WEIGHT: impassable to both cores, as the executor bans a
   * take-off it could not fly). Absent: no planner jump is verified
   * (stitchHops verifiedFollowUp).
   */
  ban?: (x: number, y: number, z: number) => void
}

/**
 * verifiedFollowUp: the kernel ticks a planner jump off a known rest is flown
 * for (the executor's own search at a rest, plugin.ts TAKEOFF_REST_TICKS), the
 * take-off it is asked for (its stands span less than VERIFY_NARROW along the
 * jump: a strip, where the planner prices a run-up the body has no room for),
 * and the bans one follow-up may take before it stands as it is.
 */
const VERIFY_TICKS = 200_000
const VERIFY_NARROW = 0.35
const VERIFY_BANS = 3

/**
 * Goal-side closure bounds: predecessors are looked for within a hop's reach
 * of a cell (GOAL_SIDE_REACH_SQ, GOAL_SIDE_UP above to GOAL_SIDE_DOWN below),
 * at most GOAL_SIDE_GEN cells generated, in GOAL_SIDE_SHARE of the budget.
 */
const GOAL_SIDE_REACH_SQ = 6.5 * 6.5
const GOAL_SIDE_UP = 8
const GOAL_SIDE_DOWN = 2
const GOAL_SIDE_GEN = 4000
const GOAL_SIDE_SHARE = 0.15

/**
 * The GOAL SIDE of a failed search: the cells the cell planner reaches the
 * goal from — a backward closure over its own edges from the goal cell,
 * generated lazily, only round what is already on the goal side and never on
 * the cells the failed search reached (none of them reaches the goal: it
 * failed). A hop that lands on the goal side completes the route; the
 * oracle's candidates are otherwise ranked by the heuristic and capped at
 * MAX_REGRESS, so a hop that leads away from the goal to get there (arena
 * opk2-1: the slot under the corridor wall, 4.8 further than its take-off)
 * was never tried, and a cave below was tried first. Null when there is no
 * goal cell or no edge generator.
 */
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
/**
 * A pipeline still getting closer gets more time: each hop that lands nearer
 * the goal than any before (by PROGRESS_MIN) pushes the deadline to at least
 * PROGRESS_STEP_MS from then, PROGRESS_MAX_MS past the first at most. One that
 * is not stops on time (an impossible route). Arena mcc-6-1, under the race's
 * load: twelve rounds of steady progress ran out two short of the goal.
 */
const PROGRESS_MIN = 0.5
const PROGRESS_STEP_MS = 2500
const PROGRESS_MAX_MS = 6000
/**
 * The share of what is left a carry chain over ice gets (HopOracle.chain):
 * mcc-7-2's 107-tick chain is ~0.67M kernel ticks, 2-3 s, and it is the only
 * way across - nothing lands from rest on the last pads.
 */
const CHAIN_SHARE = 0.85
/** ...and the kernel work a chain may fly before it is given up (mcc-7-2's takes 0.67M): a failing chain must not take the clock. */
const CHAIN_TICKS = 1_000_000
/**
 * ...and a REGION chain (StitchState.rest): hops between the places of nodes
 * the planner files as one (beside a pane rail, on it, beyond it), each a
 * round - a line of hoops is a dozen of them (arena mcc-6-1: 12).
 */
const MAX_REGION_ROUNDS = 20
/** How much further from the goal (heuristic) a region hop may land and still be kept: a pane crossed inside a hoop gets no closer. */
const REGION_SLACK = 1
/** Nodes at the end of the best partial path the oracle takes off from. */
const SOURCES = 8
/** Candidate hops per round, each judged by a follow-up search. */
const EDGES = 6
/**
 * ...and in a region chain's round (discoverFromRest): best-first follows
 * the best of them, and the rest cost a round its time - arena mcc-6-1 under
 * load, 6 a round 5.2-6.1 s, 2 a round 2.8-3.0 s, the same 12 programs.
 */
const REGION_EDGES = 2
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

/**
 * Drop-edge take-offs (dropTakeoffs): at most DROP_TAKEOFFS of them, given
 * DROP_SHARE of a discovery's budget, each flown by the bounce family alone
 * until its first edge.
 */
const DROP_TAKEOFFS = 4
const DROP_SHARE = 0.4
/** The slime ring a drop take-off is looked for in (hopOracle.ts BOUNCE_REACH). */
const DROP_RING = 7
/** How high above the highest source a drop take-off may be (hopOracle.ts HOP_RISE). */
const DROP_RISE = 2

/**
 * Ledges over slime the failed search reached but its partial path walked
 * past: the highest reached cell of each column round the slime tops the
 * sources see, with a drop beside it (HopOracle.openDrop) — the way down to
 * the slime. The partial path ends wherever the heuristic bottomed out; on
 * arena mcc-3-1 that is behind a pane window, and the ledge the 9-block drop
 * onto the slime leaves from (the only take-off the physics lands from) is
 * three columns along, reached and never a source. Nearest the slime first.
 */
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

/**
 * Frontier take-offs (frontierSources): at most FRONTIERS of them, looked
 * for among the reached cells within FRONTIER_SCAN of the tail horizontally
 * (FRONTIER_DOWN below its lowest node to FRONTIER_UP above its highest),
 * the FRONTIER_TESTS lowest-heuristic standable ones tested for an opening.
 */
const FRONTIERS = 4
/**
 * ...of them the goal-side picks at most (untested for an opening: they could crowd out the take-offs that
 * open - arena mcc-8-2, four cells near a goal-side cell and none the kick leaves from). Capped, mcc-8-2
 * stitches (offline: null -> 17 nodes; the other 16 pipeline routes identical) and, with the executor's
 * slip keys, arrives on all three engines (2026-10-10; uncapped its run had fallen from P3's plate band).
 * PF_SIDE_FRONTIERS overrides it (A/B).
 */
const SIDE_FRONTIERS = process.env.PF_SIDE_FRONTIERS !== undefined ? Number(process.env.PF_SIDE_FRONTIERS) : 2
const FRONTIER_SCAN = 8
const FRONTIER_DOWN = 6
const FRONTIER_UP = 4
const FRONTIER_TESTS = 48

/**
 * Take-offs the failed search reached off its partial path: reached,
 * standable cells round the tail that look out on unreached ground a hop
 * could land on (HopOracle.opens), nearest the goal first, a cell column or
 * two levels apart from the tail and from each other. The partial path
 * follows the heuristic to where it bottomed out; the way on may leave from
 * a ledge beside it (opk2-5: the grate pillar 810,233,-395 two blocks over
 * the chain the partial ends on, its jump under the shelf refused by the
 * cell planner) or from a detour that winds away first (opk2-1: the moss
 * stone 969,242,-396 whose slot drop under the corridor wall the planner
 * has no class for).
 */
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
  // (first the cells a goal-side cell is within hop range of: the hop off them completes the route)
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

/**
 * The part of a failed search's partial path the bot may walk while the
 * pipeline runs: up to (not including) the node nearest any level-0 off-path
 * take-off — a vantage, a drop edge — that the pipeline may stitch from, the
 * whole partial when there is none. The bot otherwise stood for the whole
 * pipeline (arena mcc-3-2: 14 ticks; mcc-4-2: 75), where a search that
 * happened to flood its box streamed partials and the bot walked from the
 * first tick, onto the same prefix the stitched path then kept. The host's
 * partial rules still hold: cut at a one-way drop, never emitted empty.
 */
export function streamablePrefix (first: RawSolveResult, start: XYZ, oracle: HopOracle, reached: (x: number, y: number, z: number) => boolean, goal: XYZ | null): RawPathNode[] {
  const path = first.path
  if (path.length === 0) return []
  const sources: XYZ[] = [start, ...path.map(n => ({ x: n.x, y: n.y, z: n.z }))].slice(-SOURCES)
  const end = path[path.length - 1]
  const high = goal !== null ? vantages(reached, goal, end, sources) : []
  const cands = [...high, ...vantages(reached, end, end, [...sources, ...high]), ...dropTakeoffs(oracle, reached, sources)]
  if (cands.length === 0) return path.slice()
  // (each candidate's nearest point of the walk - the start itself too - and
  // the earliest of those: never past a take-off the pipeline may choose.
  // The single nearest pair cut arena mcc-3-1 at a drop by the partial's far
  // end, the bot walked 14 ticks west and back to the drop behind its start)
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

/** PF_HOP_DEBUG=1: one line per pipeline round on stderr (diagnostic). */
const DEBUG = process.env.PF_HOP_DEBUG === '1'

/**
 * One state of the stitching search: a stitched prefix ending at a hop's
 * landing (`curStart`), the follow-up search from there (`cur`), how close
 * it got (`score`, its best node's heuristic), how many hops deep, and the
 * discovery level its next expansion runs:
 * 0 — vantages (goal and end) and drop-edge take-offs, the partial's tail
 *     only when those find nothing;
 * 1 — the partial's tail, when level 0 skipped it;
 * 2 — frontier take-offs off the tail (frontierSources), and hops onto the
 *     goal side (goalSide);
 * 3 — the yaw family (then the kicks, HopOracle 'kick') from the tail, then
 *     from the frontier;
 * 4 — the beam from the tail.
 */
interface StitchState {
  stitched: RawPathNode[]
  cur: RawSolveResult
  curStart: XYZ
  score: number
  depth: number
  level: number
  tailSkipped: boolean
  /**
   * The exact rest the last hop came to, when the chain is in REGION mode:
   * the hop landed beside a thin block (an under region: HopOracle.regions),
   * where the planner's node means a place the body is not, or it was flown
   * from such a rest. Expanded by HopOracle.discoverFromRest from this rest
   * alone; null: the ordinary pipeline.
   */
  rest: { x: number, y: number, z: number } | null
  /** The rest is in an under region: no planner follow-up from it. */
  under: boolean
  /** 1: a fallback tried only once every ordinary state is spent (a start rest the start cell does not mean). */
  tier: number
  /** Level 0 flew the carry chain over ice instead of the vantages and drop take-offs: level 1 flies them, before the tail. */
  deferL0?: boolean
  /**
   * The drop take-offs of a level 0 whose vantages found edges, deferred
   * (stitchHops): sorted at the parent's level and score, they are flown
   * once no level-0 state that got closer is left. Arena mcc-4-2's rounds
   * spent 35-50% of the pipeline on drops while its vantage hops got
   * closer, and no drop state is ever on its final path.
   */
  dropsOnly?: boolean
  /** A state whose follow-up reached the goal: the whole path's time (stitchTime), what the successes of one round are ranked by. */
  time?: number
}

/** Ticks a unit of planner cost stands for (a block at a sprint): stitchTime. */
const TICKS_PER_COST = 3.6

/**
 * The time a stitched path takes, in units of planner cost: its programs'
 * ticks (TICKS_PER_COST) and the planner's cost for the rest. The successes
 * one round finds are ranked by it - discovery order had arena opk2-2 fly
 * a 119-tick catch with 111-tick ones beside it.
 */
function stitchTime (path: ReadonlyArray<RawPathNode>): number {
  let t = 0
  for (const n of path) t += n.program !== null && n.program !== undefined ? n.program.ticks / TICKS_PER_COST : n.cost
  return t
}

/**
 * Best-first over the hops (improvement). The old loop committed to the
 * round's best hop and failed outright when its follow-up dead-ended: on
 * arena mcc-4-2 the hop onto a slime balloon (follow-up best h 22.4) beat
 * the hop onto the next tower's ladder (26.2), opened nothing, and the
 * pipeline returned null. Every hop whose follow-up gets strictly closer is
 * kept; the closest is expanded first (which is the old pick), the others
 * stay in reserve, and the costlier discoveries (the tail when vantages
 * found something, the yaw family, the beam) are deferred levels of the same
 * state, so every state's cheap hops are tried before any one's expensive.
 */
export function stitchHops (first: RawSolveResult, start: XYZ, ctx: HopPipelineContext): RawSolveResult | null {
  const t0 = performance.now()
  const h = (n: XYZ): number => ctx.heuristic(n.x, n.y, n.z)
  const last = (r: RawSolveResult, at: XYZ): XYZ => r.path.length > 0 ? r.path[r.path.length - 1] : at
  // (progress extends the deadline: PROGRESS_*)
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
  /** Does take-off cell `from`'s stands span less than VERIFY_NARROW along the jump to `n` (a strip)? */
  const narrowTakeoff = (from: XYZ, n: XYZ): boolean => {
    const along = Math.abs(n.x - from.x) >= Math.abs(n.z - from.z)
    let lo = Infinity
    let hi = -Infinity
    for (const s of ctx.oracle.stands(from.x, from.y, from.z)) {
      if (s.catch) continue
      lo = Math.min(lo, along ? s.x : s.z)
      hi = Math.max(hi, along ? s.x : s.z)
    }
    return hi - lo < VERIFY_NARROW
  }
  /**
   * Follow-up `f` from `from`, its first move a planner jump off a narrow
   * take-off, flown first from `rest` - the body's own at a replan, a
   * program's landing - as the executor will search it there; none landing,
   * its landing cell is banned (ctx.ban) and the follow-up searched again
   * (`again`). Arena mcc-8-2: the planner's jump off P3's 3/16 panel onto
   * the strip beside the pipe flies from no rest on P3, and the executor gave
   * up and re-planned three times, 3 s each, banning a strip cell a time,
   * before a plan took the pipe.
   */
  const verifiedFollowUp = (from: XYZ, rest: XYZ, f: RawSolveResult, again: () => RawSolveResult): RawSolveResult => {
    if (ctx.ban === undefined) return f
    for (let k = 0; k < VERIFY_BANS && f.path.length > 0; k++) {
      const n = f.path[0]
      if (n.parkour !== true || (n.program !== null && n.program !== undefined) || !narrowTakeoff(from, n)) return f
      const b = newBody(rest.x, rest.y, rest.z)
      b.onGround = true
      b.vy = -SIM.gravity * SIM.airdrag
      ctx.oracle.tickLimit = VERIFY_TICKS
      let flies = false
      try {
        flies = ctx.oracle.hopFrom(b, [n.x, n.y, n.z], ctx.deadline) !== null
      } finally {
        ctx.oracle.tickLimit = 0
      }
      if (DEBUG) console.warn(`[hops]   verify ${from.x},${from.y},${from.z} -> ${n.x},${n.y},${n.z} from ${rest.x.toFixed(2)},${rest.y.toFixed(2)},${rest.z.toFixed(2)}: ${flies ? 'flies' : 'none, banned'}`)
      if (flies || performance.now() >= ctx.deadline) return f
      ctx.ban(n.x, n.y, n.z)
      f = again()
      visited += f.visitedNodes
      generated += f.generatedNodes
    }
    return f
  }
  // (a replan from the body's own rest: the first search's first jump, flown from there)
  if (ctx.startRest != null) {
    const r0 = first
    first = verifiedFollowUp(start, ctx.startRest, first, () => ctx.solve(start, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE)))
    if (first !== r0) {
      addChunks(first)
      bestSeen = h(last(first, start))
    }
  }
  // (the goal side: computed once, against the FIRST search's reach, at the first escalation that uses it)
  let side: GoalSide | null | undefined
  const sideOf = (): GoalSide | null => {
    if (side === undefined) {
      const tS = performance.now()
      // `reached` must answer for the first search: re-run it if a follow-up overwrote it
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
  // a body resting on a node of several places (beside a pane rail, on it):
  // the stitching starts from the rest, in region mode - and beside the rail
  // (an under region) the first search's partial path is from a place the
  // body is not
  if (ctx.startRest != null) {
    const r = ctx.startRest
    const hw = ctx.oracle.halfWidth
    // the node whose place the body is in: of the columns under its box, the
    // start's first (a body over a lip is filed under the column it stands on)
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
        // beside the block: the first search's partial path is from a place the body is not
        frontier[0] = { stitched: [], cur: noFollowUp(first), curStart: node, score: h(node), depth: 0, level: 0, tailSkipped: false, rest, under: true, tier: 0 }
        reachedFor = null
        regionSeen.add(placeKey(node, r))
      } else if (ctx.oracle.underStands(node.x, node.y, node.z).length > 0) {
        // on top of a rail whose sides are places too: the ordinary pipeline AND the rest's region chain
        frontier.push({ ...frontier[0], curStart: node, score: h(node), rest, tier: 0 })
        regionSeen.add(placeKey(node, r))
      } else if (regs.length > 1 || !sameCell(node, start)) {
        // a place the start cell does not mean: the region chain from the rest, once the ordinary pipeline is spent
        frontier.push({ ...frontier[0], curStart: node, score: h(node), rest, tier: 1 })
      }
      break
    }
  }
  const tried = new Set<string>()
  let expansions = 0
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
        time: first.time + (performance.now() - t0),
        visitedNodes: visited,
        generatedNodes: generated,
        path,
        touchedChunks: [...chunks.values()],
        boundaryLimited: false
      }
    }
    if (performance.now() >= ctx.deadline) return null
    // (a region state has one level: simple families from its rest)
    if (st.depth >= (st.rest !== null ? MAX_REGION_ROUNDS : MAX_ROUNDS) || st.level > (st.rest !== null ? 0 : 4)) continue
    // (level 1 is the tail when level 0 skipped it; otherwise straight on to the frontier, no re-solve)
    if (st.level === 1 && !st.tailSkipped) {
      frontier.push({ ...st, level: 2 })
      continue
    }
    const tkey = `${st.rest !== null ? placeKey(curStart, st.rest) : `${curStart.x},${curStart.y},${curStart.z}`}:${st.level}${st.dropsOnly === true ? 'drops' : ''}`
    if (tried.has(tkey)) continue
    tried.add(tkey)
    expansions++
    // (the goal side first: it needs the first search's reach, and may re-run it)
    const sideNow = st.rest === null && (st.level === 2 || st.level === 3) ? sideOf() : null
    if (!st.under && (reachedFor === null || !sameCell(reachedFor, curStart))) {
      // the follow-ups (or a search to one cell) overwrote what `reached` answers for
      // (a share of what is left, as a follow-up gets: one search that floods
      // its box must not take the whole think budget)
      cur = ctx.solve(curStart, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE))
      reachedFor = curStart
      visited += cur.visitedNodes
      generated += cur.generatedNodes
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
    // higher ground the search reached near the goal, and near where it ended;
    // then the ledges over the slime the tail sees, each to its first bounce
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
      // (a level 0's deferred drop take-offs: nothing else, no level after)
      edges = dropEdges()
    } else if (st.rest !== null) {
      // a region chain: from the rest alone, to every place in hop range
      // (the planner's reach only where it ran from here)
      const skipTop = !st.under && reachedFor !== null && sameCell(reachedFor, curStart) ? ctx.reached : null
      edges = ctx.oracle.discoverFromRest(curStart, st.rest, ctx.heuristic, share(), skipTop, REGION_EDGES, 'simple')
    } else if (st.level === 0) {
      // a partial path that ends on ice: the carry chain from the last firm
      // node, FIRST (HopOracle.chainStart) - the tail's own hops leave from
      // ice, where nothing stops for a run-up
      // (only across gaps: a parkour leg in the icy run - a partial that
      // merely ends on an ice floor is no chain)
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
        // (the vantages and drop take-offs go to level 1, before the tail)
        tailSkipped = true
        deferNext = true
      } else {
        // the vantages first; their drops deferred while those get closer (StitchState.dropsOnly)
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
      // reached take-offs beside the tail: simple families, then the yaw
      // family after the tail's (discover flies its sources last first)
      // (and goal-side landings, worth any regress: hopPipeline.ts goalSide)
      if (ctx.solveTo !== undefined) front = frontierSources(ctx.oracle, ctx.reached, ctx.heuristic, sources, sideNow)
      if (st.level === 2) {
        if (front.length > 0) edges = ctx.oracle.discover(front, ctx.heuristic, share(), ctx.reached, EDGES, 'simple', sideNow)
      } else {
        edges = ctx.oracle.discover([...front, ...sources], ctx.heuristic, share(), ctx.reached, EDGES, 'yawKick', sideNow)
        // narrow landings off the tail's end: the pivot family, where nothing else lands (HopProgram.precise)
        if (edges.length === 0) edges = ctx.oracle.discover(sources.slice(-2), ctx.heuristic, share(), ctx.reached, EDGES, 'pivot')
      }
    } else {
      edges = ctx.oracle.discover(sources, ctx.heuristic, share(), ctx.reached, EDGES, 'beam')
    }
    if (st.rest === null && st.level < 4 && st.dropsOnly !== true) frontier.push({ ...st, cur, level: st.level + 1, tailSkipped, deferL0: deferNext, dropsOnly: undefined })
    if (DEBUG) console.warn(`[hops] L${st.level} #${expansions} depth ${st.depth} from ${curStart.x},${curStart.y},${curStart.z}: ${cur.status} path ${cur.path.length} best h ${bestH.toFixed(1)}; ${edges.length} edges in ${(performance.now() - tD).toFixed(0)} ms (${ctx.oracle.ticksFlown} ticks)${high.length > 0 ? `; vantages ${high.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}${drops.length > 0 ? `; drops ${drops.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}${front.length > 0 ? `; frontier ${front.map(v => `${v.x},${v.y},${v.z}`).join(' ')}` : ''}`)
    const seenLand = new Set<string>()
    for (let ei = 0; ei < edges.length; ei++) {
      const e = edges[ei]
      const left = ctx.deadline - performance.now()
      if (left <= 0) break
      const target = { x: e.program.tx, y: e.program.ty, z: e.program.tz }
      // (never a program off ice: its line-up slides, and the ones it opens
      // are over voids - mcc-7-2's pads; the precision line-up is not there yet)
      if (ctx.oracle.onIce(e.fromX, e.fromY, e.fromZ)) continue
      // where the hop's body comes to rest: beside a thin block in the cell
      // below the node (an under region), the planner's node is a place the
      // body is not - no planner follow-up from there, and the hop is held
      // to that place (HopProgram.land) wherever it is flown again
      const rest = e.program.catchTarget ? null : ctx.oracle.restOf(e.program)
      // (beside the block in the cell under the node, not on a low block's top: HopOracle.besideRest)
      const under = rest !== null && ctx.oracle.besideRest([target.x, target.y, target.z], rest)
      let prog = e.program
      if (under && prog.land === undefined) {
        const held = { ...prog, land: { x: rest.x, y: rest.y, z: rest.z } }
        const ticks = ctx.oracle.verify(held)
        if (ticks < 0) continue
        prog = { ...held, ticks }
      }
      const region = st.rest !== null || under
      // an even share of what is left, one more kept for the states after: a
      // follow-up that floods its box must not starve the candidates after it
      let f = under ? noFollowUp(cur) : ctx.solve(target, Math.max(FOLLOW_UP_MIN_MS, left / (edges.length - ei + 1)))
      if (!under) reachedFor = target
      visited += f.visitedNodes
      generated += f.generatedNodes
      // (its first planner jump, flown from the hop's own rest)
      if (!under && rest !== null) f = verifiedFollowUp(target, rest, f, () => ctx.solve(target, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) / (edges.length - ei + 1))))
      const score = f.status === 'success' ? -Infinity : h(last(f, target))
      const pk = region && rest !== null ? placeKey(target, rest) : ''
      if (DEBUG) console.warn(`[hops]   cand ${e.fromX},${e.fromY},${e.fromZ} -> ${target.x},${target.y},${target.z} ${prog.family} ${prog.ticks}t${rest !== null ? ` rest ${rest.x.toFixed(2)},${rest.y.toFixed(2)},${rest.z.toFixed(2)}${under ? ' UNDER' : ''}` : ''}: follow-up ${f.status} visited ${f.visitedNodes} best h ${score.toFixed(1)}`)
      // a hop is kept only where it gets strictly closer than its state did -
      // or, in a region chain, onto a place not landed on before, no more
      // than REGION_SLACK further (across the pane inside a hoop)
      if (!(score < bestH) && !(region && st.rest !== null && score <= bestH + REGION_SLACK && pk !== '' && !regionSeen.has(pk))) continue
      if (pk !== '') regionSeen.add(pk)
      // follow-ups that end at the same cell open the same ground: the first (discovery order) stands -
      // but of those that reach the goal, a quicker one (stitchTime) is pushed too, the sort taking it first
      const end = last(f, target)
      // (a region hop that opens nothing for the planner: its place is what it opens)
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
          // an off-path take-off (a vantage, a drop edge): the way to it, reached so there is one
          if (ctx.solveTo === undefined) continue
          const up = ctx.solveTo(curStart, from, Math.max(FOLLOW_UP_MIN_MS, (ctx.deadline - performance.now()) * DISCOVER_SHARE))
          reachedFor = null
          visited += up.visitedNodes
          generated += up.generatedNodes
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
        // priced like the table prices a jump: blocks of distance plus the pad
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
      frontier.push({ stitched, cur: f, curStart: target, score, depth: st.depth + 1, level: 0, tailSkipped: false, rest: region && rest !== null ? { x: rest.x, y: rest.y, z: rest.z } : null, under, tier: st.tier, time })
      if (score < bestSeen - PROGRESS_MIN) {
        bestSeen = score
        ctx.deadline = Math.max(ctx.deadline, Math.min(hardDeadline, performance.now() + PROGRESS_STEP_MS))
      }
    }
  }
  return null
}
