// Loader + runner for the Rust wasm solver core (rust/src/lib.rs). The JS
// solver stays the REFERENCE implementation; the wasm core is an accelerator
// for coordinate goals (the fastEvaluator set, plus GoalCompositeAny over
// them) with no exclusion callbacks. Any load/instantiation failure degrades
// silently to the JS path.
//
// The core keeps persistent state between solves: the snapshot grids and dig
// tables stay RESIDENT in wasm memory keyed by (generation, patchCount) /
// dig fingerprint, and the A* arena is epoch-stamped so repeat solves skip
// both the input copies and the arena clear entirely.
import type { GoalDescriptor, MovementsConfig, RawPathNode, DigData, SnapshotMeta } from '../types.js'
import type { RawSolveResult } from '../solver.js'
import { getParkourExtTable, serializeParkourTable } from '../parkourTable.js'
import { J_RUN, J_LOW_RUN, ENVELOPE_SAFETY_MARGIN } from '../parkourEnvelope.js'

interface WasmExports {
  memory: WebAssembly.Memory
  wasm_alloc (size: number): number
  wasm_free (ptr: number, size: number): void
  snap_begin (n: number, statesLen: number, specialLen: number): void
  snap_flags_ptr (): number
  snap_heights_ptr (): number
  snap_states_ptr (): number
  snap_special_ptr (): number
  snap_set_meta (x0: number, y0: number, z0: number, w: number, h: number, l: number, worldMinY: number): void
  dig_begin (len: number): void
  dig_labor_ptr (): number
  dig_flags_ptr (): number
  solve_init (params: number): number
  solve_run (maxExpansions: number): number
  finalize (status: number): number
  result_ptr (): number
}

const GOAL_KINDS: Record<string, number> = {
  block: 0,
  near: 1,
  follow: 1,
  xz: 2,
  nearxz: 3,
  y: 4,
  getToBlock: 5
}

interface GoalSpec {
  kind: number
  gx: number
  gy: number
  gz: number
  rangeSq: number
}

const MAX_GOAL_SPECS = 4096 // matches the solve_init cap in lib.rs

/**
 * Flatten a descriptor into wasm goal specs. Coordinate goals map 1:1;
 * compositeAny of (recursively) coordinate goals flattens exactly — min is
 * associative, so nested min-folds equal the flat one bit-for-bit. Null when
 * any leaf needs the JS path.
 */
export function flattenGoalSpecs (d: GoalDescriptor): GoalSpec[] | null {
  const kind = GOAL_KINDS[d.type]
  if (kind !== undefined) {
    return [{
      kind,
      gx: (d.x as number) ?? 0,
      gy: (d.y as number) ?? 0,
      gz: (d.z as number) ?? 0,
      rangeSq: (d.rangeSq as number) ?? 0
    }]
  }
  if (d.type === 'compositeAny') {
    const children = d.goals as GoalDescriptor[]
    if (!Array.isArray(children) || children.length === 0) return null
    const specs: GoalSpec[] = []
    for (const child of children) {
      const s = flattenGoalSpecs(child)
      if (!s) return null
      specs.push(...s)
      if (specs.length > MAX_GOAL_SPECS) return null
    }
    return specs
  }
  return null
}

export function wasmSupportsGoal (d: GoalDescriptor): boolean {
  return flattenGoalSpecs(d) !== null
}

export interface WasmSnapshotInput {
  meta: SnapshotMeta
  flags: Uint8Array
  heights: Uint8Array
  states: Uint16Array | null
  special?: Uint8Array | null
  entityIdx: Int32Array
  entityWeight: Int32Array
}

export interface WasmSolveOptions {
  timeout: number
  searchRadius: number
  sliceMs: number
  cancelFlag: Int32Array | null
  onPartial: (result: RawSolveResult) => void
}

export class WasmSolver {
  private readonly exports: WasmExports
  /** Content key of the snapshot currently resident in wasm memory. */
  private lastSnapKey: string | null = null
  /** Fingerprint of the dig tables currently resident in wasm memory. */
  private lastDigKey: string | null = null

  private constructor (exports: WasmExports) {
    this.exports = exports
  }

  /** Null when no wasm payload is embedded or instantiation fails. */
  static async create (): Promise<WasmSolver | null> {
    try {
      const { wasmBase64 } = await import('./solverWasmData.js')
      if (!wasmBase64) return null
      const bytes = Buffer.from(wasmBase64, 'base64')
      const { instance } = await WebAssembly.instantiate(bytes, {})
      return new WasmSolver(instance.exports as unknown as WasmExports)
    } catch (error) {
      console.warn('[bulba-pathfinder] wasm solver unavailable, using JS solver:', (error as Error).message)
      return null
    }
  }

  /**
   * Full solve loop with host-side slicing: between expansion batches the
   * host checks its cancel flag + think budget and emits interim partials
   * (same cadence contract as the JS solver path).
   */
  solve (
    snap: WasmSnapshotInput,
    cfg: MovementsConfig,
    goal: GoalDescriptor,
    start: { x: number, y: number, z: number },
    dig: DigData | null,
    opts: WasmSolveOptions
  ): RawSolveResult {
    try {
      return this.solveInner(snap, cfg, goal, start, dig, opts)
    } catch (error) {
      // Residency state may be inconsistent after a mid-upload/trap failure
      // — force full re-upload on the next attempt.
      this.lastSnapKey = null
      this.lastDigKey = null
      throw error
    }
  }

  private solveInner (
    snap: WasmSnapshotInput,
    cfg: MovementsConfig,
    goal: GoalDescriptor,
    start: { x: number, y: number, z: number },
    dig: DigData | null,
    opts: WasmSolveOptions
  ): RawSolveResult {
    const ex = this.exports
    const startTime = performance.now()
    const m = snap.meta
    const n = m.w * m.h * m.l

    const specs = flattenGoalSpecs(goal)
    if (!specs) throw new Error(`wasm solver does not support goal type ${goal.type}`)

    // ── snapshot residency: upload only when content changed ─────────────
    const statesLen = cfg.canDig && snap.states ? snap.states.length : 0
    // Uploaded whenever present (bubble columns and/or climbable vines mark it).
    const specialLen = snap.special ? snap.special.length : 0
    const snapKey = `${m.generation}:${m.patchCount}:${m.x0},${m.y0},${m.z0}:${m.w}x${m.h}x${m.l}:${statesLen}:${specialLen}`
    if (this.lastSnapKey !== snapKey) {
      this.lastSnapKey = null // a failure below must not leave a stale claim
      ex.snap_begin(n, statesLen, specialLen)
      const flagsPtr = ex.snap_flags_ptr()
      const heightsPtr = ex.snap_heights_ptr()
      const statesPtr = statesLen > 0 ? ex.snap_states_ptr() : 0
      const specialPtr = specialLen > 0 ? ex.snap_special_ptr() : 0
      // snap_begin may have grown wasm memory — build the view after it.
      const mem = new Uint8Array(ex.memory.buffer)
      mem.set(snap.flags, flagsPtr)
      mem.set(snap.heights, heightsPtr)
      if (statesLen > 0) {
        mem.set(new Uint8Array(snap.states!.buffer, snap.states!.byteOffset, statesLen * 2), statesPtr)
      }
      if (specialLen > 0) {
        mem.set(snap.special!, specialPtr)
      }
      ex.snap_set_meta(m.x0, m.y0, m.z0, m.w, m.h, m.l, m.worldMinY)
      this.lastSnapKey = snapKey
    }

    // ── dig residency: upload once per fingerprint ────────────────────────
    if (cfg.canDig && dig) {
      const digKey = dig.fingerprint ? `${dig.fingerprint}:${dig.labor.length}` : null
      if (digKey === null || this.lastDigKey !== digKey) {
        this.lastDigKey = null
        ex.dig_begin(dig.labor.length)
        const laborPtr = ex.dig_labor_ptr()
        const digFlagsPtr = ex.dig_flags_ptr()
        const mem = new Uint8Array(ex.memory.buffer)
        mem.set(new Uint8Array(dig.labor.buffer, dig.labor.byteOffset, dig.labor.length * 4), laborPtr)
        mem.set(dig.flags, digFlagsPtr)
        this.lastDigKey = digKey
      }
    }

    // ── per-solve blobs: params + entity weights + parkour table ─────────
    const entityCount = snap.entityIdx.length
    const entitySize = entityCount * 8
    const paramsSize = 4 * 4 + specs.length * 36 + 4 * 2 + 8 * 6 + 4 * 2 + 4 * 2
    // The extended-parkour table travels with the solve (~2.5KB) so the core
    // consumes the exact table the JS reference builds — one geometry source.
    const extBlob = cfg.allowParkourExtended && cfg.allowParkour && cfg.allowSprinting
      ? serializeParkourTable(getParkourExtTable(), J_RUN, J_LOW_RUN)
      : null
    // Allocs before any view — each may grow (and detach views of) wasm
    // memory; resident snapshot/dig data stays valid (growth extends).
    const entityPtr = entityCount > 0 ? ex.wasm_alloc(entitySize) : 0
    const extPtr = extBlob !== null ? ex.wasm_alloc(extBlob.byteLength) : 0
    const paramsPtr = ex.wasm_alloc(paramsSize)
    if (extBlob !== null) {
      new Uint8Array(ex.memory.buffer).set(new Uint8Array(extBlob), extPtr)
    }

    if (entityCount > 0) {
      const interleaved = new Int32Array(entityCount * 2)
      for (let i = 0; i < entityCount; i++) {
        interleaved[i * 2] = snap.entityIdx[i]
        interleaved[i * 2 + 1] = snap.entityWeight[i]
      }
      new Uint8Array(ex.memory.buffer).set(new Uint8Array(interleaved.buffer), entityPtr)
    }

    const view = new DataView(ex.memory.buffer)
    let off = paramsPtr
    const i32 = (v: number): void => { view.setInt32(off, v, true); off += 4 }
    const f64 = (v: number): void => { view.setFloat64(off, v, true); off += 8 }
    const u32 = (v: number): void => { view.setUint32(off, v, true); off += 4 }

    i32(start.x); i32(start.y); i32(start.z)
    i32(specs.length)
    for (const s of specs) {
      i32(s.kind); f64(s.gx); f64(s.gy); f64(s.gz); f64(s.rangeSq)
    }
    let cfgBits = 0
    if (cfg.allowSprinting) cfgBits |= 1
    if (cfg.allowParkour) cfgBits |= 2
    if (cfg.canOpenDoors) cfgBits |= 4
    if (cfg.canOpenDoors && cfg.canOpenRealDoors) cfgBits |= 8
    if (cfg.infiniteLiquidDropdownDistance) cfgBits |= 16
    if (cfg.canDig) cfgBits |= 32
    if (cfg.dontCreateFlow) cfgBits |= 64
    if (cfg.dontMineUnderFallingBlock) cfgBits |= 128
    if (cfg.useBubbleColumns) cfgBits |= 256
    if (cfg.allowParkourExtended) cfgBits |= 512
    i32(cfgBits)
    i32(cfg.maxDropDown)
    f64(cfg.liquidCost); f64(cfg.entityCost); f64(cfg.digCost)
    f64(cfg.bubbleCost)
    f64(ENVELOPE_SAFETY_MARGIN - (cfg.parkourSafetyMargin ?? ENVELOPE_SAFETY_MARGIN)) // margin credit
    f64(opts.searchRadius)
    u32(entityPtr); u32(entityCount)
    u32(extPtr); u32(extBlob !== null ? extBlob.byteLength : 0)

    try {
      const rc = ex.solve_init(paramsPtr)
      if (rc !== 0) {
        throw new Error(`wasm solve_init rejected the parameters (code ${rc})`)
      }

      // ── slice loop: cancel + budget between expansion batches ──────────
      const BATCH = 16384
      let lastPartial = performance.now()
      for (;;) {
        const runRc = ex.solve_run(BATCH)
        if (runRc === 1) return this.readResult(0, startTime)
        if (runRc === 2) return this.readResult(3, startTime)

        const now = performance.now()
        if (opts.cancelFlag !== null && Atomics.load(opts.cancelFlag, 0) !== 0) {
          const result = this.readResult(2, startTime)
          result.cancelled = true
          return result
        }
        if (now - startTime > opts.timeout) {
          return this.readResult(2, startTime)
        }
        if (now - lastPartial >= opts.sliceMs) {
          lastPartial = now
          opts.onPartial(this.readResult(1, startTime))
        }
      }
    } finally {
      ex.wasm_free(paramsPtr, paramsSize)
      if (entityPtr) ex.wasm_free(entityPtr, entitySize)
      if (extPtr) ex.wasm_free(extPtr, (extBlob as ArrayBuffer).byteLength)
    }
  }

  private readResult (status: number, startTime: number): RawSolveResult {
    const ex = this.exports
    const len = ex.finalize(status)
    const ptr = ex.result_ptr()
    const view = new DataView(ex.memory.buffer, ptr, len)
    let off = 0
    const statusByte = view.getUint8(off); off += 1
    const boundaryLimited = view.getUint8(off) !== 0; off += 1
    const cost = view.getFloat64(off, true); off += 8
    const visitedNodes = view.getUint32(off, true); off += 4
    const generatedNodes = view.getUint32(off, true); off += 4

    const chunkCount = view.getUint32(off, true); off += 4
    const touchedChunks: Array<[number, number]> = []
    for (let i = 0; i < chunkCount; i++) {
      const cx = view.getInt32(off, true); off += 4
      const cz = view.getInt32(off, true); off += 4
      touchedChunks.push([cx, cz])
    }

    const pathLen = view.getUint32(off, true); off += 4
    const path: RawPathNode[] = []
    for (let i = 0; i < pathLen; i++) {
      const x = view.getInt32(off, true); off += 4
      const y = view.getInt32(off, true); off += 4
      const z = view.getInt32(off, true); off += 4
      const edgeCost = view.getFloat64(off, true); off += 8
      const meta = view.getUint8(off); off += 1
      const node: RawPathNode = {
        x,
        y,
        z,
        cost: edgeCost,
        parkour: (meta & 1) !== 0,
        useOne: (meta & 2) !== 0 ? { x, y, z } : null
      }
      if ((meta & (4 | 8)) !== 0) {
        // META_BOUNCE / META_CHAIN: the via cell follows the meta byte
        // (lib.rs serialize_result), before the toBreak list.
        const vx = view.getInt32(off, true); off += 4
        const vy = view.getInt32(off, true); off += 4
        const vz = view.getInt32(off, true); off += 4
        node.via = { x: vx, y: vy, z: vz }
        if ((meta & 8) !== 0) node.chain = true
      }
      const breakCount = view.getUint16(off, true); off += 2
      if (breakCount > 0) {
        node.toBreak = []
        for (let b = 0; b < breakCount; b++) {
          const bx = view.getInt32(off, true); off += 4
          const by = view.getInt32(off, true); off += 4
          const bz = view.getInt32(off, true); off += 4
          node.toBreak.push({ x: bx, y: by, z: bz })
        }
      }
      path.push(node)
    }

    const statusName = (['success', 'partial', 'timeout', 'noPath'] as const)[statusByte] ?? 'noPath'
    return {
      status: statusName,
      cancelled: false,
      cost,
      time: performance.now() - startTime,
      visitedNodes,
      generatedNodes,
      path,
      touchedChunks,
      boundaryLimited
    }
  }
}
