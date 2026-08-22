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
import type { GoalDescriptor, MovementsConfig, SnapshotMeta } from '../types.js'
import type { RaycastLut } from '../raycast.js'

interface LutMessage {
  t: 'lut'
  fingerprint: string
  maxStateId: number
  shapeStarts: Int32Array
  shapeCounts: Uint8Array
  shapeData: Float32Array
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
}

type InMessage = LutMessage | DigMessage | SolveMessage

if (!parentPort) {
  throw new Error('@bulba/pathfinder worker entry must run inside a worker_thread')
}
const port = parentPort

let lut: RaycastLut | null = null
let lutFingerprint = ''

// Dig tables by fingerprint — sent once per (inventory × effects × lut)
// change instead of ~120KB per solve. Insertion-ordered; the host mirrors
// this eviction policy (MAX_DIG_TABLES in host.ts) so both sides always
// agree on residency.
const digTables = new Map<string, { labor: Float32Array, flags: Uint8Array }>()

// The wasm core loads in the background; solves that arrive before it is
// ready (or when it is unavailable) run on the JS solver — the reference
// implementation and permanent fallback. PF_NO_WASM=1 disables it.
let wasmSolver: WasmSolver | null = null
if (process.env.PF_NO_WASM !== '1') {
  WasmSolver.create().then(ws => { wasmSolver = ws }, () => {})
}

port.on('message', (msg: InMessage) => {
  try {
    if (msg.t === 'lut') {
      lut = {
        maxStateId: msg.maxStateId,
        shapeStarts: msg.shapeStarts,
        shapeCounts: msg.shapeCounts,
        shapeData: msg.shapeData
      }
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
          port.postMessage({ t: 'done', id: msg.id, result })
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
        port.postMessage({ t: 'done', id: msg.id, result })
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
