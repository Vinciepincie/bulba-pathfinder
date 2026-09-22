// Worker-thread solver host. Receives LUT tables once per profile change and
// per-solve snapshot views (SharedArrayBuffer-backed — zero copy), runs the
// solver in ~tick-sized slices posting interim 'partial' results (so the bot
// can walk a partial path while thinking, upstream-style), and honors a
// shared cancel flag between slices. The event loop of the BOT process is
// never blocked by a solve — that failure class is structurally gone here.
import { parentPort } from 'node:worker_threads'
import { Solver } from '../solver.js'
import { GoalAdapter } from '../goalAdapter.js'
import { SnapshotRaycastWorld } from '../raycast.js'
import { instantiateGoal } from '../goalSerde.js'
import { fastEvaluator } from '../fastEvaluator.js'
import { WasmSolver, wasmSupportsGoal } from '../wasm/wasmSolver.js'
import { MAX_DIG_TABLES } from './digProtocol.js'
import { HopOracle } from '../hopOracle.js'
import { stitchHops } from '../hopPipeline.js'
import { simTablesFromLut } from '../playerSim.js'
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
  /** The physics kernel's per-state tables (lut.ts simSlip / simKind); absent from older hosts. */
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
  /** References dig tables previously sent via DigMessage (canDig only). */
  digFingerprint: string | null
  /** The bot's body for the physics kernel (allowParkourPhysics): hitbox and walking speed. */
  body?: { halfWidth: number, height: number, speed: number }
  /** Run the hop pipeline on a boundary-limited failure too (WorkerSolveRequest.hopsOnBoundary). */
  hopsOnBoundary?: boolean
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

// Dig tables by fingerprint — sent once per (inventory × effects × lut)
// change instead of ~120KB per solve. Insertion-ordered; the host mirrors
// this eviction policy (MAX_DIG_TABLES in host.ts) so both sides always
// agree on residency.
const digTables = new Map<string, { labor: Float32Array, flags: Uint8Array }>()

// The wasm core loads in the background; solves that arrive before it is
// ready (or when it is unavailable) run on the JS solver — the reference
// implementation and permanent fallback. PF_NO_WASM=1 disables it.
let wasmSolver: WasmSolver | null = null
/** A reserve request that arrived before the core was instantiated. */
let pendingReserve = 0
if (process.env.PF_NO_WASM !== '1') {
  WasmSolver.create().then(ws => {
    wasmSolver = ws
    if (ws && pendingReserve > 0) ws.reserve(pendingReserve)
  }, () => {})
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
  return new HopOracle(
    {
      ...simTablesCache.tables,
      stateAt: (x, y, z) => { const i = idx(x, y, z); return i < 0 ? -1 : states[i] },
      grid: { states, x0: m.x0, y0: m.y0, z0: m.z0, w: m.w, h: m.h, l: m.l }
    },
    (x, y, z) => { const i = idx(x, y, z); return i < 0 ? 0 : flags[i] },
    body.halfWidth, body.height, body.speed)
}

/**
 * Physics-verified hops where the search FAILED (allowParkourPhysics,
 * hopPipeline.ts): a stitched whole path to the goal, or null — in which case
 * the original failure stands. Never for a cancelled solve; for a
 * boundary-limited one only in the first box of a solve (hopsOnBoundary) —
 * a failure the table could not bridge floods its box, and grown boxes only
 * give the follow-up searches more to flood; the host grows the box when the
 * pipeline finds nothing.
 */
/** Does the partial path end within a cell of the box's faces? */
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
  reached: (x: number, y: number, z: number) => boolean
): RawSolveResult | null {
  if (first.status !== 'noPath' && first.status !== 'timeout') return null
  const oracle = makeOracle(msg)
  if (oracle === null) return null
  // A partial path that ends against the box's own faces was stopped by the
  // box, not the terrain: growth first, or the discovery spends its share of
  // the budget from the edge of a box that is simply too small (climb1's
  // goal is 46 blocks up: 1.6 s standing still, where a grown box finds the
  // whole path in 200 ms).
  const skip = first.cancelled === true ? 'cancelled'
    : first.boundaryLimited && msg.hopsOnBoundary !== true ? 'boundary-limited'
      : first.boundaryLimited && tailOnBoundary(first.path, msg.meta) ? 'tail-on-boundary'
        : ''
  if (process.env.PF_HOP_DEBUG === '1') {
    console.warn(`[hops] ${first.status} from ${msg.start.x},${msg.start.y},${msg.start.z} visited ${first.visitedNodes}: ${skip === '' ? 'pipeline' : 'skipped (' + skip + ')'}`)
  }
  if (skip !== '') return null
  const evaluator = fastEvaluator(msg.goal)
  if (evaluator === null) return null
  const g = msg.goal as { x?: unknown, y?: unknown, z?: unknown }
  const goalCell = typeof g.x === 'number' && typeof g.y === 'number' && typeof g.z === 'number'
    ? { x: Math.floor(g.x), y: Math.floor(g.y), z: Math.floor(g.z) }
    : null
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
    deadline: performance.now() + msg.timeout
  })
  if (Atomics.load(cancelFlag, 0) !== 0) return null
  return stitched
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
      // The '' fingerprint (uncacheable, resent per solve) is exempt from
      // the bound — evicting a real entry for it would desync the host's
      // residency record.
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
      const snap = {
        meta: msg.meta,
        flags: new Uint8Array(msg.flagsBuf),
        heights: new Uint8Array(msg.heightsBuf),
        special: msg.specialBuf ? new Uint8Array(msg.specialBuf) : null,
        thin: msg.thinBuf ? new Uint8Array(msg.thinBuf) : null,
        entityIdx: msg.entityIdx,
        entityWeight: msg.entityWeight
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

      // ── wasm fast path: coordinate goals on the Rust core ──────────────
      if (wasmSolver && wasmSupportsGoal(msg.goal)) {
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
            special: snap.special, thin: snap.thin, entityIdx: msg.entityIdx, entityWeight: msg.entityWeight
          }
          const hops = hopPipeline(msg, result, cancelFlag,
            (start, timeoutMs, goal) => ws.solve(wasmSnap, msg.cfg, goal ?? msg.goal, start, dig ? dig.data : null, {
              timeout: timeoutMs, searchRadius: msg.searchRadius, sliceMs: 1e9, cancelFlag, onPartial: () => {}
            }),
            (x, y, z) => ws.reached(x, y, z))
          if (hops !== null) { hops.engine = 'wasm'; hops.hops = hops.path.filter(n => n.program !== undefined).length }
          port.postMessage({ t: 'done', id: msg.id, result: hops ?? result })
          return
        } catch (error) {
          // Fall through to the JS reference solver.
          console.warn('[bulba-pathfinder] wasm solve failed, falling back to JS:', (error as Error).message)
        }
      }

      // Monomorphic evaluator for coordinate goals; adapter for the rest.
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

      // Slice loop: never blocks this worker for more than sliceMs between
      // cancel checks; interim partial paths stream back to the host.
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
          (x, y, z) => lastSolver.reached(x, y, z))
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
