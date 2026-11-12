// Share one lazy worker across bots; failed solves fall back to the main thread.
// Repeated startup failures disable spawning, and unref lets idle processes exit.
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
  /** The bot's body for the physics kernel (cfg.allowParkourPhysics): hitbox and walking speed. */
  body?: { halfWidth: number, height: number, speed: number }
  /** Allow hops in the first boundary-limited box; grown boxes make follow-up floods expensive. */
  hopsOnBoundary?: boolean
  /** The core to solve on (PathfinderOptions.core); the wasm core when absent. */
  core?: 'wasm' | 'js'
  /** The body's exact feet position, when on the ground (hopPipeline.ts startRest). */
  startPos?: { x: number, y: number, z: number }
  /** Whether the worker kernel keeps vanilla's faster jump rise, matching the bot's physics. */
  vanillaJump?: boolean
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

/** Worker-local pending solves and LUT state keep late exit events isolated from replacement workers. */
interface ActiveWorker {
  worker: Worker
  pending: Map<number, Pending>
  sentLutFingerprint: string
  /** Resident dig fingerprints, evicted in insertion order at the worker's MAX_DIG_TABLES bound. */
  sentDigFingerprints: Set<string>
  online: boolean
  gone: boolean
}

const MAX_SPAWN_FAILURES = 3

/** Prewarm reserve size avoids allocating search arrays on the first solve. */
const DEFAULT_RESERVE_CELLS = 1_200_000

export class SolverWorkerHost {
  /** Where the host's warnings go. */
  warn: (message: string) => void = message => console.warn(message)
  private active: ActiveWorker | null = null
  private spawning: Promise<ActiveWorker | null> | null = null
  private nextId = 1
  private spawnFailures = 0
  private entryPath: string | null = null
  /** Arena size every spawned worker is asked to reserve (0 = none). */
  private reserveCells = 0

  /** True after repeated spawn failures; callers solve on the main thread. */
  get unavailable (): boolean {
    return this.spawnFailures >= MAX_SPAWN_FAILURES
  }

  setEntryPath (path: string): void {
    this.entryPath = path
  }

  private async resolveEntryPath (): Promise<string> {
    if (this.entryPath) return this.entryPath
    // The format-specific shim is absent during direct TS execution; failure uses the main thread.
    const { moduleDir } = await import('../runtime/moduleDir.js')
    return join(moduleDir(), 'worker', 'entry.js')
  }

  private retire (aw: ActiveWorker, reason: string, info?: unknown): void {
    if (aw.gone) return
    aw.gone = true
    if (!aw.online) this.spawnFailures++ // died before 'online' = spawn failure
    if (this.unavailable) {
      this.warn('[bulba-pathfinder] worker thread unavailable, staying on main-thread solving')
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
      // After the listeners: adding a message listener refs the worker again, and it would hold the process open.
      worker.unref()
      if (this.reserveCells > 0) worker.postMessage({ t: 'reserve', cells: this.reserveCells })
      this.active = aw
      return aw
    } catch (error) {
      this.spawnFailures++
      if (this.unavailable) {
        this.warn(`[bulba-pathfinder] worker thread unavailable, staying on main-thread solving: ${(error as Error).message}`)
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

  /** Prewarm thread startup and wasm loading so the first goal avoids their cost.
   * Spawn failures still use main-thread fallback; unref lets idle processes exit. */
  prewarm (cells = DEFAULT_RESERVE_CELLS): void {
    if (this.unavailable) return
    // Reserve search arrays now and for every replacement worker.
    if (cells > this.reserveCells) {
      this.reserveCells = cells
      if (this.active && !this.active.gone) this.active.worker.postMessage({ t: 'reserve', cells })
    }
    void this.ensureWorker().catch(() => {})
  }

  /** Null when the worker is unavailable; the caller solves on the main thread. */
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
        shapeData: req.lut.shapeData,
        simSlip: req.lut.simSlip,
        simKind: req.lut.simKind
      })
      aw.sentLutFingerprint = req.lut.fingerprint
    }

    // Send dig tables once per fingerprint; an empty fingerprint is uncacheable.
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
    // A cancelled solve has nobody waiting on it, and a worker dying then must not crash the process.
    promise.catch(() => {})

    const snap = req.snapshot
    aw.worker.postMessage({
      t: 'solve',
      id,
      meta: snap.meta,
      flagsBuf: snap.flags.buffer,
      heightsBuf: snap.heights.buffer,
      statesBuf: snap.states ? snap.states.buffer : null,
      specialBuf: snap.special ? snap.special.buffer : null,
      thinBuf: snap.thin ? snap.thin.buffer : null,
      entityIdx: snap.entityIdx,
      entityWeight: snap.entityWeight,
      cfg: req.cfg,
      goal: req.goal,
      start: req.start,
      timeout: req.timeout,
      searchRadius: req.searchRadius,
      sliceMs: req.sliceMs,
      cancelBuf,
      digFingerprint: req.dig ? req.dig.fingerprint : null,
      body: req.body,
      hopsOnBoundary: req.hopsOnBoundary,
      core: req.core,
      startPos: req.startPos,
      vanillaJump: req.vanillaJump
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
