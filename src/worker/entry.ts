// Shared snapshot views keep solving off the bot thread without copying the world.
// Slices stream partial paths and check cancellation between batches.
import { parentPort } from 'node:worker_threads'
import { Solver } from '../solver.js'
import { GoalAdapter } from '../goalAdapter.js'
import { SnapshotRaycastWorld } from '../raycast.js'
import { instantiateGoal } from '../goalSerde.js'
import { fastEvaluator } from '../fastEvaluator.js'
import { WasmSolver, wasmSupportsGoal } from '../wasm/wasmSolver.js'
import { MAX_DIG_TABLES } from './digProtocol.js'
import { HopOracle } from '../hopOracle.js'
import { MoveGen, MOM_NONE } from '../moveGen.js'
import { LutFlags } from '../types.js'
import { stitchHops, streamablePrefix } from '../hopPipeline.js'
import { simTablesFromLut, setVanillaJump } from '../playerSim.js'
import type { SimWorld } from '../playerSim.js'
import type { RawSolveResult } from '../solver.js'
import type { GoalDescriptor, MovementsConfig, SnapshotMeta } from '../types.js'
import type { RaycastLut } from '../raycast.js'

interface LutMessage {
  t: 'lut'
  fingerprint: string
  maxStateId: number
  shapeStarts: Int32Array
  shapeCounts: Uint8Array
  shapeData: Float32Array
  /** Optional physics tables for per-state slipperiness and block kind. */
  simSlip?: Float64Array
  simKind?: Uint8Array
}

/** Per-state dig tables, sent once per fingerprint (inventory/effects hash). */
interface DigMessage {
  t: 'dig'
  fingerprint: string
  labor: Float32Array
  flags: Uint8Array
}

interface SolveMessage {
  t: 'solve'
  id: number
  meta: SnapshotMeta
  flagsBuf: SharedArrayBuffer
  heightsBuf: SharedArrayBuffer
  statesBuf: SharedArrayBuffer | null
  specialBuf: SharedArrayBuffer | null
  thinBuf?: SharedArrayBuffer | null
  entityIdx: Int32Array
  entityWeight: Int32Array
  cfg: MovementsConfig
  goal: GoalDescriptor
  start: { x: number, y: number, z: number }
  timeout: number
  searchRadius: number
  sliceMs: number
  cancelBuf: SharedArrayBuffer
  /** Fingerprint of dig tables sent via DigMessage (canDig only). */
  digFingerprint: string | null
  /** The bot's body for the physics kernel (allowParkourPhysics): hitbox and walking speed. */
  body?: { halfWidth: number, height: number, speed: number }
  /** Allow hops in the first boundary-limited box; grown boxes make follow-up floods expensive. */
  hopsOnBoundary?: boolean
  /** Solve on the JavaScript reference core instead of the wasm core (PathfinderOptions.core). */
  core?: 'wasm' | 'js'
  /** The body's exact feet position when on the ground (hopPipeline.ts startRest). */
  startPos?: { x: number, y: number, z: number }
  /** Whether the physics kernel keeps vanilla's faster jump rise, matching the bot. */
  vanillaJump?: boolean
}

/** Prewarm: size the wasm arena for a box of this many cells now. */
interface ReserveMessage {
  t: 'reserve'
  cells: number
}

type InMessage = LutMessage | DigMessage | SolveMessage | ReserveMessage

if (!parentPort) {
  throw new Error('@bulba/pathfinder worker entry must run inside a worker_thread')
}
const port = parentPort

let lut: RaycastLut | null = null
let lutFingerprint = ''
/** Kernel tables of the current LUT (allowParkourPhysics). */
let simLut: { simSlip: Float64Array, simKind: Uint8Array } | null = null

// Mirror the host's insertion-order eviction bound so dig-table residency stays synchronized.
const digTables = new Map<string, { labor: Float32Array, flags: Uint8Array }>()

// Solves use JavaScript while wasm loads or when wasm is unavailable.
let wasmSolver: WasmSolver | null = null
/** A reserve request that arrived before the core was instantiated. */
let pendingReserve = 0
if (process.env.PF_NO_WASM !== '1') {
  WasmSolver.create().then(ws => {
    wasmSolver = ws
    if (ws && pendingReserve > 0) ws.reserve(pendingReserve)
  }, () => {})
}

/** Banned move pairs `a` with (from, to) appended (SnapshotView.bannedEdges: a pipeline ban). */
function withEdge (a: Int32Array | null | undefined, from: number, to: number): Int32Array {
  const n = a != null ? a.length : 0
  const out = new Int32Array(n + 2)
  if (a != null) out.set(a)
  out[n] = from
  out[n + 1] = to
  return out
}

/** Kernel tables built from the current LUT, cached per fingerprint. */
let simTablesCache: { fingerprint: string, tables: Omit<SimWorld, 'stateAt'> } | null = null

/** The exact-physics hop oracle over this solve's snapshot, or null when the solve lacks what it needs. */
function makeOracle (msg: SolveMessage): HopOracle | null {
  if (msg.cfg.allowParkourPhysics !== true || !msg.cfg.allowParkourExtended) return null
  if (lut === null || simLut === null || msg.statesBuf === null) return null
  if (simTablesCache === null || simTablesCache.fingerprint !== lutFingerprint) {
    simTablesCache = { fingerprint: lutFingerprint, tables: simTablesFromLut({ ...lut, ...simLut }) }
  }
  const m = msg.meta
  const states = new Uint16Array(msg.statesBuf)
  const flags = new Uint8Array(msg.flagsBuf)
  const idx = (x: number, y: number, z: number): number => {
    const lx = x - m.x0
    const ly = y - m.y0
    const lz = z - m.z0
    if (lx < 0 || lx >= m.w || ly < 0 || ly >= m.h || lz < 0 || lz >= m.l) return -1
    return (ly * m.l + lz) * m.w + lx
  }
  const body = msg.body ?? { halfWidth: 0.3, height: 1.8, speed: 0.1 }
  const oracle = new HopOracle(
    {
      ...simTablesCache.tables,
      stateAt: (x, y, z) => { const i = idx(x, y, z); return i < 0 ? -1 : states[i] },
      grid: { states, x0: m.x0, y0: m.y0, z0: m.z0, w: m.w, h: m.h, l: m.l }
    },
    (x, y, z) => { const i = idx(x, y, z); return i < 0 ? 0 : flags[i] },
    body.halfWidth, body.height, body.speed)
  if (msg.cfg.fallVeto !== undefined) oracle.fallVeto = msg.cfg.fallVeto
  return oracle
}

/** Whether the partial path ends within one cell of a snapshot boundary.
 * Boundary-stopped paths need box growth before hop discovery. */
function tailOnBoundary (path: ReadonlyArray<{ x: number, y: number, z: number }>, m: { x0: number, y0: number, z0: number, w: number, h: number, l: number }): boolean {
  if (path.length === 0) return false
  const t = path[path.length - 1]
  return t.x <= m.x0 + 1 || t.x >= m.x0 + m.w - 2 || t.z <= m.z0 + 1 || t.z >= m.z0 + m.l - 2 || t.y <= m.y0 + 1 || t.y >= m.y0 + m.h - 2
}

function hopPipeline (
  msg: SolveMessage,
  first: RawSolveResult,
  cancelFlag: Int32Array,
  solveFrom: (start: { x: number, y: number, z: number }, timeoutMs: number, goal?: GoalDescriptor) => RawSolveResult,
  reached: (x: number, y: number, z: number) => boolean,
  postPartial: (partial: RawSolveResult) => void,
  snap: ConstructorParameters<typeof MoveGen>[0],
  banEdge: (from: number, to: number) => void
): RawSolveResult | null {
  const r = hopPipelineRun(msg, first, cancelFlag, solveFrom, reached, postPartial, snap, banEdge)
  // Keep verification bans even when no stitched path is found.
  return r.stitched ?? r.verifiedFirst
}

function hopPipelineRun (
  msg: SolveMessage,
  first: RawSolveResult,
  cancelFlag: Int32Array,
  solveFrom: (start: { x: number, y: number, z: number }, timeoutMs: number, goal?: GoalDescriptor) => RawSolveResult,
  reached: (x: number, y: number, z: number) => boolean,
  postPartial: (partial: RawSolveResult) => void,
  snap: ConstructorParameters<typeof MoveGen>[0],
  banEdge: (from: number, to: number) => void
): { stitched: RawSolveResult | null, verifiedFirst: RawSolveResult | null } {
  let verifiedFirst: RawSolveResult | null = null
  const none = { stitched: null, verifiedFirst: null }
  if (first.status !== 'noPath' && first.status !== 'timeout') return none
  const oracle = makeOracle(msg)
  if (oracle === null) return none
  // Cancellation stops hop work so the next solve is not held behind discarded results.
  const abortOn = process.env.PF_PIPE_ABORT !== '0'
  if (abortOn) oracle.abort = () => Atomics.load(cancelFlag, 0) !== 0
  // Grow a boundary-stopped search before spending hop-discovery budget in a box that is too small.
  const skip = first.cancelled === true ? 'cancelled'
    : first.boundaryLimited && msg.hopsOnBoundary !== true ? 'boundary-limited'
      : first.boundaryLimited && tailOnBoundary(first.path, msg.meta) ? 'tail-on-boundary'
        : ''
  if (process.env.PF_HOP_DEBUG === '1') {
    console.warn(`[hops] ${first.status} from ${msg.start.x},${msg.start.y},${msg.start.z} visited ${first.visitedNodes}: ${skip === '' ? 'pipeline' : 'skipped (' + skip + ')'}`)
  }
  if (skip !== '') return none
  const evaluator = fastEvaluator(msg.goal)
  if (evaluator === null) return none
  const g = msg.goal as { x?: unknown, y?: unknown, z?: unknown }
  const goalCell = typeof g.x === 'number' && typeof g.y === 'number' && typeof g.z === 'number'
    ? { x: Math.floor(g.x), y: Math.floor(g.y), z: Math.floor(g.z) }
    : null
  const prefix = streamablePrefix(first, msg.start, oracle, reached, goalCell)
  if (prefix.length > 0) postPartial({ ...first, status: 'partial', path: prefix })
  let moveGen: MoveGen | null = null
  const m = msg.meta
  const flagAt = (x: number, y: number, z: number): number => {
    const lx = x - m.x0
    const ly = y - m.y0
    const lz = z - m.z0
    if (lx < 0 || lx >= m.w || ly < 0 || ly >= m.h || lz < 0 || lz >= m.l) return 0
    return snap.flags[(ly * m.l + lz) * m.w + lx]
  }
  const stitched = stitchHops(first, msg.start, {
    oracle,
    heuristic: (x, y, z) => evaluator.heuristic(x, y, z),
    solve: (start, timeoutMs) => {
      if (Atomics.load(cancelFlag, 0) !== 0) return { ...first, path: [], status: 'noPath' }
      return solveFrom(start, timeoutMs)
    },
    solveTo: (start, cell, timeoutMs) => {
      if (Atomics.load(cancelFlag, 0) !== 0) return { ...first, path: [], status: 'noPath' }
      return solveFrom(start, timeoutMs, { type: 'block', x: cell.x, y: cell.y, z: cell.z })
    },
    reached,
    goal: goalCell,
    deadline: performance.now() + msg.timeout,
    onVerifiedFirst: (r) => { verifiedFirst = r },
    ...(abortOn ? { cancelled: () => Atomics.load(cancelFlag, 0) !== 0 } : {}),
    // Verified legs can be followed while the remaining hops are searched.
    ...(process.env.PF_STREAM_STITCH !== '0'
      ? { onStitched: (path: typeof first.path) => { if (Atomics.load(cancelFlag, 0) === 0) postPartial({ ...first, status: 'partial', path }) } }
      : {}),
    startRest: msg.startPos ?? null,
    edgesFrom: (x, y, z, out) => {
      const mg = moveGen ?? (moveGen = new MoveGen(snap, msg.cfg))
      mg.generate(x, y, z, MOM_NONE)
      for (let i = 0; i < mg.outCount; i++) out.push(mg.outX[i], mg.outY[i], mg.outZ[i])
    },
    // Keep unlandable planner edges banned in subsequent searches.
    banEdge: (a, b) => {
      const at = (c: { x: number, y: number, z: number }): number => {
        const lx = c.x - m.x0
        const ly = c.y - m.y0
        const lz = c.z - m.z0
        return lx >= 0 && lx < m.w && ly >= 0 && ly < m.h && lz >= 0 && lz < m.l ? (ly * m.l + lz) * m.w + lx : -1
      }
      const ia = at(a)
      const ib = at(b)
      if (ia >= 0 && ib >= 0) banEdge(ia, ib)
    },
    nodeLike: (x, y, z) => {
      const f0 = flagAt(x, y, z)
      const fd = flagAt(x, y - 1, z)
      return (f0 & LutFlags.SAFE) !== 0 && (flagAt(x, y + 1, z) & LutFlags.SAFE) !== 0 &&
        ((fd & LutFlags.SAFE) === 0 || (fd & (LutFlags.PHYSICAL | LutFlags.CLIMBABLE)) !== 0 || (f0 & (LutFlags.CLIMBABLE | LutFlags.LIQUID)) !== 0)
    }
  })
  if (Atomics.load(cancelFlag, 0) !== 0) return none
  return { stitched, verifiedFirst }
}

port.on('message', (msg: InMessage) => {
  try {
    if (msg.t === 'reserve') {
      if (wasmSolver) wasmSolver.reserve(msg.cells)
      else pendingReserve = Math.max(pendingReserve, msg.cells)
      return
    }

    if (msg.t === 'lut') {
      lut = {
        maxStateId: msg.maxStateId,
        shapeStarts: msg.shapeStarts,
        shapeCounts: msg.shapeCounts,
        shapeData: msg.shapeData
      }
      simLut = msg.simSlip !== undefined && msg.simKind !== undefined ? { simSlip: msg.simSlip, simKind: msg.simKind } : null
      lutFingerprint = msg.fingerprint
      port.postMessage({ t: 'lutAck', fingerprint: lutFingerprint })
      return
    }

    if (msg.t === 'dig') {
      digTables.delete(msg.fingerprint) // refresh insertion order
      digTables.set(msg.fingerprint, { labor: msg.labor, flags: msg.flags })
      // Empty fingerprints are resent, so evicting a cacheable entry here would desynchronize the host.
      const cacheable = (): number => digTables.size - (digTables.has('') ? 1 : 0)
      if (cacheable() > MAX_DIG_TABLES) {
        for (const key of digTables.keys()) {
          if (key === '') continue
          digTables.delete(key)
          if (cacheable() <= MAX_DIG_TABLES) break
        }
      }
      return
    }

    if (msg.t === 'solve') {
      setVanillaJump(msg.vanillaJump === true)
      const snap = {
        meta: msg.meta,
        flags: new Uint8Array(msg.flagsBuf),
        heights: new Uint8Array(msg.heightsBuf),
        special: msg.specialBuf ? new Uint8Array(msg.specialBuf) : null,
        thin: msg.thinBuf ? new Uint8Array(msg.thinBuf) : null,
        entityIdx: msg.entityIdx,
        entityWeight: msg.entityWeight,
        bannedEdges: null as Int32Array | null
      }
      const cancelFlag = new Int32Array(msg.cancelBuf)

      let world: SnapshotRaycastWorld | null = null
      if (msg.statesBuf) {
        if (!lut) throw new Error('raycast goal received before LUT tables')
        world = new SnapshotRaycastWorld(msg.meta, new Uint16Array(msg.statesBuf), lut)
      }

      let dig = null
      if (msg.cfg.canDig) {
        if (!msg.statesBuf || msg.digFingerprint == null) {
          throw new Error('canDig solve requires state grid + dig tables')
        }
        const tables = digTables.get(msg.digFingerprint)
        if (!tables) {
          throw new Error(`dig tables not resident for fingerprint ${msg.digFingerprint}`)
        }
        dig = {
          data: { fingerprint: msg.digFingerprint, labor: tables.labor, flags: tables.flags },
          states: new Uint16Array(msg.statesBuf),
          breakExclusion: null
        }
      }

      if (wasmSolver && msg.core !== 'js' && wasmSupportsGoal(msg.goal)) {
        try {
          const result = wasmSolver.solve(
            {
              meta: msg.meta,
              flags: snap.flags,
              heights: snap.heights,
              states: msg.statesBuf ? new Uint16Array(msg.statesBuf) : null,
              special: snap.special,
              thin: snap.thin,
              entityIdx: msg.entityIdx,
              entityWeight: msg.entityWeight
            },
            msg.cfg,
            msg.goal,
            msg.start,
            dig ? dig.data : null,
            {
              timeout: msg.timeout,
              searchRadius: msg.searchRadius,
              sliceMs: msg.sliceMs,
              cancelFlag,
              onPartial: (partial) => {
                partial.engine = 'wasm'
                port.postMessage({ t: 'partial', id: msg.id, result: partial })
              }
            }
          )
          result.engine = 'wasm'
          const ws = wasmSolver
          const wasmSnap = {
            meta: msg.meta, flags: snap.flags, heights: snap.heights,
            states: msg.statesBuf ? new Uint16Array(msg.statesBuf) : null,
            special: snap.special, thin: snap.thin, entityIdx: msg.entityIdx, entityWeight: msg.entityWeight,
            bannedEdges: null as Int32Array | null
          }
          const hops = hopPipeline(msg, result, cancelFlag,
            (start, timeoutMs, goal) => ws.solve(wasmSnap, msg.cfg, goal ?? msg.goal, start, dig ? dig.data : null, {
              timeout: timeoutMs, searchRadius: msg.searchRadius, sliceMs: 1e9, cancelFlag, onPartial: () => {}
            }),
            (x, y, z) => ws.reached(x, y, z),
            (partial) => port.postMessage({ t: 'partial', id: msg.id, result: partial }), snap,
            (a, b) => { wasmSnap.bannedEdges = snap.bannedEdges = withEdge(wasmSnap.bannedEdges, a, b) })
          if (hops !== null) { hops.engine = 'wasm'; hops.hops = hops.path.filter(n => n.program !== undefined).length }
          port.postMessage({ t: 'done', id: msg.id, result: hops ?? result })
          return
        } catch (error) {
          console.warn('[bulba-pathfinder] wasm solve failed, falling back to JS:', (error as Error).message)
        }
      }

      const evaluator = fastEvaluator(msg.goal) ?? new GoalAdapter(instantiateGoal(msg.goal, world))
      const solver = new Solver(
        snap,
        msg.cfg,
        evaluator,
        msg.start,
        { timeout: msg.timeout, searchRadius: msg.searchRadius, cancelFlag },
        null,
        dig
      )

      // Bound cancellation delay while streaming partial paths between slices.
      for (;;) {
        const result = solver.compute(msg.sliceMs)
        result.engine = 'js'
        if (result.status === 'partial') {
          port.postMessage({ t: 'partial', id: msg.id, result })
          continue
        }
        let lastSolver = solver
        const hops = hopPipeline(msg, result, cancelFlag,
          (start, timeoutMs, goal) => {
            const ev = goal !== undefined ? fastEvaluator(goal) ?? evaluator : evaluator
            const s2 = new Solver(snap, msg.cfg, ev, start, { timeout: timeoutMs, searchRadius: msg.searchRadius, cancelFlag }, null, dig)
            let r = s2.compute(1e9)
            while (r.status === 'partial') r = s2.compute(1e9)
            lastSolver = s2
            return r
          },
          (x, y, z) => lastSolver.reached(x, y, z),
          (partial) => port.postMessage({ t: 'partial', id: msg.id, result: partial }), snap,
          (a, b) => { snap.bannedEdges = withEdge(snap.bannedEdges, a, b) })
        if (hops !== null) { hops.engine = 'js'; hops.hops = hops.path.filter(n => n.program !== undefined).length }
        port.postMessage({ t: 'done', id: msg.id, result: hops ?? result })
        return
      }
    }
  } catch (error) {
    const err = error as Error
    port.postMessage({
      t: 'error',
      id: (msg as SolveMessage).id ?? -1,
      message: err?.message ?? String(error),
      stack: err?.stack ?? ''
    })
  }
})
