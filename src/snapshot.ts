// World snapshot: the classification of every block in an AABB around the
// search, resolved through the LUT into flat SharedArrayBuffer-backed arrays.
// Building it is one linear pass over chunk columns; after that the solver
// (in-process or in the worker) probes cells with pure typed-array reads and
// zero allocation, and block updates patch single cells in place instead of
// forcing a rebuild.
import type { Bot } from 'mineflayer'
import type { BlockLut } from './lut.js'
import type { Movements } from './movements.js'
import type { SnapshotMeta } from './types.js'

export interface Box {
  x0: number
  y0: number
  z0: number
  x1: number // inclusive
  y1: number
  z1: number
}

export class Snapshot {
  readonly meta: SnapshotMeta
  readonly flags: Uint8Array
  readonly heights: Uint8Array
  /** Present only when the goal needs raycasting (GoalLookAtBlock / GoalPlaceBlock). */
  states: Uint16Array | null = null
  /** LutSpecial byte per cell (bubble columns). Present only when the
   * profile's LUT carries a special table (useBubbleColumns). */
  special: Uint8Array | null = null
  entityIdx: Int32Array = EMPTY_I32
  entityWeight: Int32Array = EMPTY_I32
  /** LUT this snapshot was resolved through (for in-place cell patches). */
  lutFingerprint: string

  constructor (meta: SnapshotMeta, lutFingerprint: string) {
    this.meta = meta
    const n = meta.w * meta.h * meta.l
    this.flags = new Uint8Array(new SharedArrayBuffer(n))
    this.heights = new Uint8Array(new SharedArrayBuffer(n))
    this.lutFingerprint = lutFingerprint
  }

  get cellCount (): number {
    return this.meta.w * this.meta.h * this.meta.l
  }

  contains (x: number, y: number, z: number): boolean {
    const m = this.meta
    return x >= m.x0 && x < m.x0 + m.w && y >= m.y0 && y < m.y0 + m.h && z >= m.z0 && z < m.z0 + m.l
  }

  /** Linear cell index; caller must ensure containment. */
  index (x: number, y: number, z: number): number {
    const m = this.meta
    return ((y - m.y0) * m.l + (z - m.z0)) * m.w + (x - m.x0)
  }

  allocStates (): Uint16Array {
    if (!this.states) {
      this.states = new Uint16Array(new SharedArrayBuffer(this.cellCount * 2))
    }
    return this.states
  }

  allocSpecial (): Uint8Array {
    if (!this.special) {
      this.special = new Uint8Array(new SharedArrayBuffer(this.cellCount))
    }
    return this.special
  }
}

const EMPTY_I32 = new Int32Array(0)

// Process-global so two bots (or tests) can never mint colliding generations
// — downstream caches key on (generation, patchCount).
let globalGeneration = 0

export function nextSnapshotGeneration (): number {
  return ++globalGeneration
}

/**
 * Compute the snapshot box for a solve. `targets` are the goal's known
 * coordinates (empty for goals without any); `slack` is the searchRadius
 * cost-slack (-1 = unbounded → generous default, bounded by growth+cap).
 */
export function computeBox (
  bot: Bot,
  start: { x: number, y: number, z: number },
  targets: Array<{ x: number, y?: number, z: number }>,
  slack: number,
  growFactor: number,
  maxCells: number
): Box {
  let minX = start.x
  let maxX = start.x
  let minY = start.y
  let maxY = start.y
  let minZ = start.z
  let maxZ = start.z
  let maxDist = 0
  for (const t of targets) {
    minX = Math.min(minX, t.x); maxX = Math.max(maxX, t.x)
    minZ = Math.min(minZ, t.z); maxZ = Math.max(maxZ, t.z)
    if (t.y !== undefined) { minY = Math.min(minY, t.y); maxY = Math.max(maxY, t.y) }
    const d = Math.hypot(t.x - start.x, t.z - start.z)
    if (d > maxDist) maxDist = d
  }

  // Horizontal margin beyond the start/goal hull. With a cost-slack bound a
  // detour of d off the straight line costs ≥ ~2d (out and back), so any
  // on-path node stays within ~slack/2 of the hull; parkour compresses
  // distance-per-cost, which the boundary-growth retry absorbs. Unbounded
  // searches start moderate and grow on boundary contact — a small first box
  // keeps the snapshot build in the low-millisecond range.
  let margin = Math.ceil((slack >= 0 ? slack / 2 + 16 : maxDist * 0.5 + 32) * growFactor)
  // A close goal doesn't need a slack-sized box — the boundary-growth retry
  // covers the rare long detour, so first builds stay small and fast.
  if (targets.length > 0) margin = Math.min(margin, Math.ceil((maxDist + 24) * growFactor))
  const vMargin = Math.ceil(16 * growFactor)

  const worldMinY = (bot.game as { minY?: number }).minY ?? 0
  const worldHeight = (bot.game as { height?: number }).height ?? 256

  const box: Box = {
    x0: Math.floor(minX - margin),
    y0: Math.max(worldMinY, Math.floor(minY - vMargin)),
    z0: Math.floor(minZ - margin),
    x1: Math.ceil(maxX + margin),
    y1: Math.min(worldMinY + worldHeight - 1, Math.ceil(maxY + vMargin)),
    z1: Math.ceil(maxZ + margin)
  }

  // Respect the memory cap by shrinking the horizontal margin if needed.
  const cells = (): number => (box.x1 - box.x0 + 1) * (box.y1 - box.y0 + 1) * (box.z1 - box.z0 + 1)
  let guard = 0
  while (cells() > maxCells && guard++ < 64) {
    const shrink = Math.max(1, Math.floor((box.x1 - box.x0) * 0.05))
    box.x0 += shrink; box.x1 -= shrink
    box.z0 += shrink; box.z1 -= shrink
    if (box.x1 - box.x0 < 16 || box.z1 - box.z0 < 16) break
  }
  return box
}

interface ColumnLike {
  getBlockStateId (pos: { x: number, y: number, z: number }): number
  /** prismarine-chunk 1.18+ internals (fast path; guarded by feature checks). */
  minY?: number
  sections?: Array<{
    data?: {
      get?: (idx: number) => number
      value?: number
    }
  } | null>
}

interface WorldLike {
  getColumn (cx: number, cz: number): ColumnLike | null | undefined
}

/** Build a snapshot of the box, resolving every cell through the LUT. */
export function buildSnapshot (bot: Bot, lut: BlockLut, box: Box, needStates: boolean, generation: number = nextSnapshotGeneration()): Snapshot {
  const meta: SnapshotMeta = {
    x0: box.x0,
    y0: box.y0,
    z0: box.z0,
    w: box.x1 - box.x0 + 1,
    h: box.y1 - box.y0 + 1,
    l: box.z1 - box.z0 + 1,
    worldMinY: (bot.game as { minY?: number }).minY ?? 0,
    generation,
    patchCount: 0
  }
  const snap = new Snapshot(meta, lut.fingerprint)
  const states = needStates ? snap.allocStates() : null
  const special = lut.special ? snap.allocSpecial() : null

  const world = bot.world as unknown as WorldLike
  const flags = snap.flags
  const heights = snap.heights
  const lutFlags = lut.flags
  const lutHeights = lut.heights
  const lutSpecial = lut.special
  const maxStateId = lut.maxStateId

  const cx0 = box.x0 >> 4
  const cx1 = box.x1 >> 4
  const cz0 = box.z0 >> 4
  const cz1 = box.z1 >> 4
  const probe = { x: 0, y: 0, z: 0 }

  for (let cx = cx0; cx <= cx1; cx++) {
    const bx0 = Math.max(box.x0, cx << 4)
    const bx1 = Math.min(box.x1, (cx << 4) + 15)
    for (let cz = cz0; cz <= cz1; cz++) {
      const bz0 = Math.max(box.z0, cz << 4)
      const bz1 = Math.min(box.z1, (cz << 4) + 15)
      let column: ColumnLike | null | undefined
      try {
        column = world.getColumn(cx, cz)
      } catch {
        column = null
      }
      if (!column) continue // unloaded → cells stay 0 (upstream null-block: unsafe, not physical)

      // Fast path: read section containers directly (prismarine-chunk 1.18+).
      // A SingleValueContainer section (all air, all stone) resolves without
      // per-cell reads; paletted sections use container.get (bit-read +
      // palette index — no hash lookup, no Vec3). Falls back to the generic
      // per-cell API if the internals don't look as expected.
      const sections = column.sections
      const colMinY = column.minY
      const useSections = Array.isArray(sections) && typeof colMinY === 'number' && (colMinY & 15) === 0

      for (let y = box.y0; y <= box.y1; y++) {
        probe.y = y
        const rowBase = (y - meta.y0) * meta.l

        let uniformState = -1
        let getCell: ((idx: number) => number) | null = null
        let sectionOk = false
        if (useSections) {
          const sec = sections[(y - (colMinY as number)) >> 4]
          if (!sec || !sec.data) {
            // Missing section in a loaded column = all air.
            uniformState = 0
            sectionOk = true
          } else if (typeof sec.data.value === 'number') {
            uniformState = sec.data.value
            sectionOk = true
          } else if (typeof sec.data.get === 'function') {
            getCell = sec.data.get.bind(sec.data)
            sectionOk = true
          }
        }
        const secY = ((y - (colMinY as number || 0)) & 15) << 8

        for (let z = bz0; z <= bz1; z++) {
          probe.z = z & 15
          const base = (rowBase + (z - meta.z0)) * meta.w - meta.x0
          if (sectionOk && uniformState >= 0) {
            if (uniformState <= maxStateId) {
              const f = lutFlags[uniformState]
              const h = lutHeights[uniformState]
              const sp = lutSpecial ? lutSpecial[uniformState] : 0
              for (let x = bx0; x <= bx1; x++) {
                const idx = base + x
                flags[idx] = f
                heights[idx] = h
                if (states) states[idx] = uniformState
                if (special) special[idx] = sp
              }
            }
            continue
          }
          if (sectionOk && getCell) {
            const zPart = secY | ((z & 15) << 4)
            try {
              for (let x = bx0; x <= bx1; x++) {
                const stateId = getCell(zPart | (x & 15))
                if (stateId >= 0 && stateId <= maxStateId) {
                  const idx = base + x
                  flags[idx] = lutFlags[stateId]
                  heights[idx] = lutHeights[stateId]
                  if (states) states[idx] = stateId
                  if (special) special[idx] = lutSpecial![stateId]
                }
              }
              continue
            } catch {
              // fall through to the generic path for this row
            }
          }
          for (let x = bx0; x <= bx1; x++) {
            probe.x = x & 15
            let stateId: number
            try {
              stateId = column.getBlockStateId(probe)
            } catch {
              stateId = -1
            }
            if (stateId >= 0 && stateId <= maxStateId) {
              const idx = base + x
              flags[idx] = lutFlags[stateId]
              heights[idx] = lutHeights[stateId]
              if (states) states[idx] = stateId
              if (special) special[idx] = lutSpecial![stateId]
            }
          }
        }
      }
    }
  }

  return snap
}

/** Patch one cell in place after a blockUpdate. Returns false if outside the box. */
export function applySnapshotBlockUpdate (
  snap: Snapshot,
  lut: BlockLut,
  x: number,
  y: number,
  z: number,
  stateId: number
): boolean {
  if (!snap.contains(x, y, z)) return false
  snap.meta.patchCount++
  const idx = snap.index(x, y, z)
  if (stateId >= 0 && stateId <= lut.maxStateId) {
    snap.flags[idx] = lut.flags[stateId]
    snap.heights[idx] = lut.heights[stateId]
    if (snap.states) snap.states[idx] = stateId
    if (snap.special) snap.special[idx] = lut.special ? lut.special[stateId] : 0
  } else {
    snap.flags[idx] = 0
    snap.heights[idx] = 0
    if (snap.states) snap.states[idx] = 0
    if (snap.special) snap.special[idx] = 0
  }
  return true
}

/**
 * Bake the movements' entity-intersection index (already computed via
 * updateCollisionIndex, upstream-identical) into sparse per-cell weights.
 */
export function bakeEntityIndex (snap: Snapshot, movements: Movements): void {
  const entries = Object.entries(movements.entityIntersections)
  if (entries.length === 0) {
    snap.entityIdx = EMPTY_I32
    snap.entityWeight = EMPTY_I32
    return
  }
  const idxs: number[] = []
  const weights: number[] = []
  for (const [key, weight] of entries) {
    const [xs, ys, zs] = key.split(',')
    const x = Number(xs); const y = Number(ys); const z = Number(zs)
    if (!snap.contains(x, y, z)) continue
    idxs.push(snap.index(x, y, z))
    weights.push(weight)
  }
  snap.entityIdx = Int32Array.from(idxs)
  snap.entityWeight = Int32Array.from(weights)
}
