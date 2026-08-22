// Host side of the worker solver: one worker per PROCESS (shared by every
// bot instance — reconnect loops must never leak threads), lazy spawn
// serialized through a single in-flight promise, crash-tolerant (pending
// solves reject → callers fall back to main-thread solving), and the worker
// is unref()ed so it never keeps the process alive.
//
// Failure accounting distinguishes SPAWN failures (the worker dies before
// its 'online' event — bad entry path, bundler mangling) from runtime
// crashes: repeated spawn failures latch `unavailable` so callers stop
// paying thread-churn and stay on the main-thread path permanently.
import { Worker } from 'node:worker_threads'
import { join } from 'node:path'
import { MAX_DIG_TABLES } from './digProtocol.js'
import type { BlockLut } from '../lut.js'
import type { GoalDescriptor, MovementsConfig, DigData } from '../types.js'
import type { RawSolveResult } from '../solver.js'
import type { Snapshot } from '../snapshot.js'

export interface WorkerSolveRequest {
  snapshot: Snapshot
  lut: BlockLut
  cfg: MovementsConfig
  goal: GoalDescriptor
  start: { x: number, y: number, z: number }
  timeout: number
  searchRadius: number
  sliceMs: number
  /** Per-state dig tables (required when cfg.canDig). */
  dig?: DigData | null
  onPartial: (result: RawSolveResult) => void
}

export interface WorkerSolveHandle {
  promise: Promise<RawSolveResult>
  cancel: () => void
}

interface Pending {
  resolve: (r: RawSolveResult) => void
  reject: (e: Error) => void
  onPartial: (r: RawSolveResult) => void
}

/** A spawned worker with ITS OWN pending map and LUT state — a dead
 * worker's late 'exit' event can only ever touch its own solves. */
interface ActiveWorker {
  worker: Worker
  pending: Map<number, Pending>
  sentLutFingerprint: string
  /** Dig fingerprints resident in the worker — insertion-ordered, evicted
   * at MAX_DIG_TABLES exactly like the worker's own cache. */
  sentDigFingerprints: Set<string>
  online: boolean
  gone: boolean
}

const MAX_SPAWN_FAILURES = 3

export class SolverWorkerHost {
  private active: ActiveWorker | null = null
  private spawning: Promise<ActiveWorker | null> | null = null
  private nextId = 1
  private spawnFailures = 0
  private entryPath: string | null = null

  /** Permanently unavailable (spawn kept failing) — callers use main-thread. */
  get unavailable (): boolean {
    return this.spawnFailures >= MAX_SPAWN_FAILURES
  }

  setEntryPath (path: string): void {
    this.entryPath = path
  }

  private async resolveEntryPath (): Promise<string> {
    if (this.entryPath) return this.entryPath
    // The compiled tree carries a format-specific runtime shim next to this
    // file; in dev (running TS directly) it doesn't exist and the import
    // throws → callers fall back to main-thread solving.
    const { moduleDir } = await import('../runtime/moduleDir.js')
    return join(moduleDir(), 'worker', 'entry.js')
  }

  private retire (aw: ActiveWorker, reason: string, info?: unknown): void {
    if (aw.gone) return
    aw.gone = true
    if (!aw.online) this.spawnFailures++ // died before 'online' = spawn failure
    if (this.unavailable) {
      console.warn('[bulba-pathfinder] worker thread unavailable, staying on main-thread solving')
    }
    if (this.active === aw) this.active = null
    const err = new Error(`pathfinder worker ${reason}${info !== undefined ? `: ${String(info)}` : ''}`)
    for (const p of aw.pending.values()) p.reject(err)
    aw.pending.clear()
  }

  private async spawnWorker (): Promise<ActiveWorker | null> {
    try {
      const entry = await this.resolveEntryPath()
      const worker = new Worker(entry)
      worker.unref()
      const aw: ActiveWorker = {
        worker,
        pending: new Map(),
        sentLutFingerprint: '',
        sentDigFingerprints: new Set(),
        online: false,
        gone: false
      }
      worker.on('online', () => {
        aw.online = true
        this.spawnFailures = 0
      })
      worker.on('message', (msg: { t: string, id?: number, result?: RawSolveResult, message?: string }) => {
        if (msg.t === 'lutAck') return
        const p = aw.pending.get(msg.id ?? -1)
        if (!p) return
        if (msg.t === 'partial' && msg.result) {
          p.onPartial(msg.result)
          return
        }
        aw.pending.delete(msg.id ?? -1)
        if (msg.t === 'done' && msg.result) {
          p.resolve(msg.result)
        } else {
          p.reject(new Error(`worker solve failed: ${msg.message ?? 'unknown'}`))
        }
      })
      worker.on('error', (err) => this.retire(aw, 'errored', err?.message ?? err))
      worker.on('exit', (code) => this.retire(aw, 'exited', code !== 0 ? code : undefined))
      this.active = aw
      return aw
    } catch (error) {
      this.spawnFailures++
      if (this.unavailable) {
        console.warn('[bulba-pathfinder] worker thread unavailable, staying on main-thread solving:', (error as Error).message)
      }
      return null
    }
  }

  private async ensureWorker (): Promise<ActiveWorker | null> {
    if (this.active && !this.active.gone) return this.active
    if (this.unavailable) return null
    // Serialize spawning: concurrent first solves share one Worker.
    if (!this.spawning) {
      this.spawning = this.spawnWorker().finally(() => {
        this.spawning = null
      })
    }
    return this.spawning
  }

  /**
   * Spawn the worker now rather than on the first solve.
   *
   * Thread start, module load and wasm instantiation are a one-time ~150 ms
   * that otherwise lands entirely inside the first goal — the arena measures
   * it directly as `1st move`, where the worker engine was 100-250 ms behind
   * the main-thread one on every route while being faster at everything
   * afterwards. A bot is idle when it connects and busy when it is asked to
   * walk somewhere, so the cost belongs at load time. Fire and forget: a
   * spawn failure is already handled (callers fall back to the main thread),
   * and the worker is unref()ed so an idle process still exits.
   */
  prewarm (): void {
    if (this.unavailable) return
    void this.ensureWorker().catch(() => {})
  }

  /** Null when the worker can't be used — caller solves on the main thread. */
  async solve (req: WorkerSolveRequest): Promise<WorkerSolveHandle | null> {
    const aw = await this.ensureWorker()
    if (!aw || aw.gone) return null

    if (aw.sentLutFingerprint !== req.lut.fingerprint) {
      aw.worker.postMessage({
        t: 'lut',
        fingerprint: req.lut.fingerprint,
        maxStateId: req.lut.maxStateId,
        shapeStarts: req.lut.shapeStarts,
        shapeCounts: req.lut.shapeCounts,
        shapeData: req.lut.shapeData
      })
      aw.sentLutFingerprint = req.lut.fingerprint
    }

    // Dig tables travel once per fingerprint, not per solve (~120KB saved
    // per repeat solve). An empty fingerprint is uncacheable — resend it.
    const digFp = req.dig ? req.dig.fingerprint : null
    if (req.dig && digFp !== null && (digFp === '' || !aw.sentDigFingerprints.has(digFp))) {
      aw.worker.postMessage({
        t: 'dig',
        fingerprint: digFp,
        labor: req.dig.labor,
        flags: req.dig.flags
      })
      if (digFp !== '') {
        aw.sentDigFingerprints.delete(digFp) // refresh insertion order
        aw.sentDigFingerprints.add(digFp)
        while (aw.sentDigFingerprints.size > MAX_DIG_TABLES) {
          const oldest = aw.sentDigFingerprints.values().next().value as string
          aw.sentDigFingerprints.delete(oldest)
        }
      }
    }

    const id = this.nextId++
    const cancelBuf = new SharedArrayBuffer(4)
    const cancelFlag = new Int32Array(cancelBuf)

    const promise = new Promise<RawSolveResult>((resolve, reject) => {
      aw.pending.set(id, { resolve, reject, onPartial: req.onPartial })
    })

    const snap = req.snapshot
    aw.worker.postMessage({
      t: 'solve',
      id,
      meta: snap.meta,
      flagsBuf: snap.flags.buffer,
      heightsBuf: snap.heights.buffer,
      statesBuf: snap.states ? snap.states.buffer : null,
      specialBuf: snap.special ? snap.special.buffer : null,
      entityIdx: snap.entityIdx,
      entityWeight: snap.entityWeight,
      cfg: req.cfg,
      goal: req.goal,
      start: req.start,
      timeout: req.timeout,
      searchRadius: req.searchRadius,
      sliceMs: req.sliceMs,
      cancelBuf,
      digFingerprint: req.dig ? req.dig.fingerprint : null
    })

    return {
      promise,
      cancel: () => {
        Atomics.store(cancelFlag, 0, 1)
      }
    }
  }

  async terminate (): Promise<void> {
    const aw = this.active
    this.active = null
    if (aw) {
      this.retire(aw, 'terminated')
      await aw.worker.terminate()
    }
  }
}

let sharedHost: SolverWorkerHost | null = null

/** One worker per process, shared across bot instances and reconnects. */
export function getSharedWorkerHost (): SolverWorkerHost {
  if (!sharedHost) sharedHost = new SolverWorkerHost()
  return sharedHost
}
