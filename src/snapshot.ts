// Resolve a search box through the LUT into shared typed arrays for allocation-free probes.
// Block updates patch cells in place so the snapshot can stay resident.
import type { Bot } from 'mineflayer'
import type { BlockLut } from './lut.js'
import type { Movements } from './movements.js'
import type { SnapshotMeta } from './types.js'

/** Weight above both solvers' impassability threshold, used for banned landings and takeoffs. */
export const BAN_WEIGHT = 200

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
  /** Optional special grid for features enabled by the LUT, including bubble columns. */
  special: Uint8Array | null = null
  /** Optional thin-footprint grid for extended parkour. */
  thin: Uint8Array | null = null
  /** Chunk columns inside the box that were not loaded when it was built (their cells read as unsafe), as cx,cz keys. */
  missingColumns: Set<string> = new Set()
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

  allocThin (): Uint8Array {
    if (!this.thin) {
      this.thin = new Uint8Array(new SharedArrayBuffer(this.cellCount))
    }
    return this.thin
  }
}

const EMPTY_I32 = new Int32Array(0)

// Process-wide generations keep snapshot cache keys unique across bots.
let globalGeneration = 0

export function nextSnapshotGeneration (): number {
  return ++globalGeneration
}

/** Size a solve box around start and known targets; slack is cost allowance (-1 = unbounded).
 * `floorY` extends the box downward, for example to include a deep slime landing. */
export function computeBox (
  bot: Bot,
  start: { x: number, y: number, z: number },
  targets: Array<{ x: number, y?: number, z: number }>,
  slack: number,
  growFactor: number,
  maxCells: number,
  floorY?: number
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

  // A detour costs roughly twice its distance from the start/goal hull.
  // Keep the initial margin modest; boundary retries grow it for longer detours and parkour.
  let margin = Math.ceil((slack >= 0 ? slack / 2 + 16 : maxDist * 0.3 + 20) * growFactor)
  // Close goals start with a small box; boundary retries cover long detours.
  if (targets.length > 0) margin = Math.min(margin, Math.ceil((maxDist + 24) * growFactor))
  const vMargin = Math.ceil(16 * growFactor)

  const worldMinY = (bot.game as { minY?: number }).minY ?? 0
  const worldHeight = (bot.game as { height?: number }).height ?? 256

  const box: Box = {
    x0: Math.floor(minX - margin),
    y0: Math.max(worldMinY, Math.min(Math.floor(minY - vMargin), floorY ?? Infinity)),
    z0: Math.floor(minZ - margin),
    x1: Math.ceil(maxX + margin),
    y1: Math.min(worldMinY + worldHeight - 1, Math.ceil(maxY + vMargin)),
    z1: Math.ceil(maxZ + margin)
  }

  // Meet the memory cap by shrinking margins, then windowing the hull around the start.
  // Shrink each axis by its own extent to keep long narrow boxes valid.
  const h = box.y1 - box.y0 + 1
  const fits = (m: number): boolean =>
    (Math.ceil(maxX + m) - Math.floor(minX - m) + 1) * h * (Math.ceil(maxZ + m) - Math.floor(minZ - m) + 1) <= maxCells
  if (!fits(margin)) {
    let lo = 0
    let hi = margin
    while (lo < hi) {
      const mid = Math.ceil((lo + hi) / 2)
      if (fits(mid)) lo = mid
      else hi = mid - 1
    }
    box.x0 = Math.floor(minX - lo); box.x1 = Math.ceil(maxX + lo)
    box.z0 = Math.floor(minZ - lo); box.z1 = Math.ceil(maxZ + lo)
    if (!fits(lo)) {
      const w = box.x1 - box.x0 + 1
      const l = box.z1 - box.z0 + 1
      const area = Math.floor(maxCells / h)
      const f = Math.sqrt(area / (w * l))
      let nw = Math.max(16, Math.floor(w * f))
      let nl = Math.max(16, Math.floor(l * f))
      // An axis held at the 16-cell floor leaves the rest of the area to the other.
      if (nl === 16) nw = Math.max(16, Math.min(w, Math.floor(area / 16)))
      else if (nw === 16) nl = Math.max(16, Math.min(l, Math.floor(area / 16)))
      box.x0 = windowStart(box.x0, box.x1, Math.floor(start.x), nw); box.x1 = box.x0 + nw - 1
      box.z0 = windowStart(box.z0, box.z1, Math.floor(start.z), nl); box.z1 = box.z0 + nl - 1
    }
  }
  return box
}

/** First cell of a `size`-wide window of [lo, hi] centred on `s` as far as the range allows. */
function windowStart (lo: number, hi: number, s: number, size: number): number {
  return Math.max(lo, Math.min(hi - size + 1, s - Math.floor(size / 2)))
}

interface ColumnLike {
  getBlockStateId (pos: { x: number, y: number, z: number }): number
  /** prismarine-chunk 1.18+ internals (fast path; guarded by feature checks). */
  minY?: number
  sections?: Array<{
    data?: {
      get?: (idx: number) => number
      value?: number
      palette?: unknown
    }
  } | null>
}

/** Find slime within `depth` below the box with a clear drop from `openTo`.
 * Extend snapshots for deep bounce landings; skip sections whose palettes contain no slime. */
export function slimeUnderBox (bot: Bot, lut: BlockLut, box: Box, depth: number, openTo: number): Array<[number, number, number]> {
  const slime = (bot.registry as unknown as { blocksByName?: Record<string, { minStateId?: number }> }).blocksByName?.slime_block?.minStateId
  if (slime === undefined) return []
  const worldMinY = (bot.game as { minY?: number }).minY ?? 0
  const lo = Math.max(worldMinY, box.y0 - depth)
  const hi = box.y0 - 1
  if (hi < lo) return []
  const world = bot.world as unknown as WorldLike
  const probe = { x: 0, y: 0, z: 0 }
  const collides = (st: number): boolean => st > 0 && st <= lut.maxStateId && lut.shapeStarts[st] >= 0 && lut.shapeCounts[st] > 0
  const out: Array<[number, number, number]> = []
  for (let cx = box.x0 >> 4; cx <= box.x1 >> 4; cx++) {
    for (let cz = box.z0 >> 4; cz <= box.z1 >> 4; cz++) {
      let column: ColumnLike | null | undefined
      try { column = world.getColumn(cx, cz) } catch { column = null }
      if (!column) continue
      const sections = column.sections
      const colMinY = column.minY
      if (Array.isArray(sections) && typeof colMinY === 'number' && (colMinY & 15) === 0) {
        let any = false
        for (let sy = (lo - colMinY) >> 4; sy <= (hi - colMinY) >> 4 && !any; sy++) {
          const d = sections[sy]?.data
          if (d === undefined) continue
          if (typeof d.value === 'number') any = d.value === slime
          else if (Array.isArray(d.palette)) any = d.palette.includes(slime)
          else any = true // A direct container has no palette to filter.
        }
        if (!any) continue
      }
      for (let x = Math.max(box.x0, cx << 4); x <= Math.min(box.x1, (cx << 4) + 15); x++) {
        for (let z = Math.max(box.z0, cz << 4); z <= Math.min(box.z1, (cz << 4) + 15); z++) {
          probe.x = x & 15
          probe.z = z & 15
          let top = -Infinity
          for (let y = hi; y >= lo; y--) {
            probe.y = y
            const st = column.getBlockStateId(probe)
            if (!collides(st)) continue
            if (st === slime) top = y
            break
          }
          if (top === -Infinity) continue
          // A landing needs an unobstructed drop through the box.
          let open = true
          for (let y = box.y0; y <= openTo && open; y++) {
            probe.y = y
            if (collides(column.getBlockStateId(probe))) open = false
          }
          if (open) out.push([x, top, z])
        }
      }
    }
  }
  return out
}

interface WorldLike {
  getColumn (cx: number, cz: number): ColumnLike | null | undefined
}

/** Packed section cells and LUT-resolved palette values.
 * Direct containers store state IDs and use the LUT itself as the palette. */
interface NoSpanBits {
  words: Uint32Array
  bitsPerValue: number
  valuesPerLong: number
  valueMask: number
  flags: Uint8Array
  heights: Uint8Array
  special: Uint8Array
  thin: Uint8Array
  states: Uint16Array
}

const NO_SPECIAL = new Uint8Array(0)

/** Resolve a BitArrayNoSpan palette once per section during a snapshot build.
 * Return null for unsupported layouts so callers can use the generic accessor. */
function noSpanBits (container: unknown, lut: BlockLut, cache: Map<unknown, NoSpanBits | null>): NoSpanBits | null {
  const hit = cache.get(container)
  if (hit !== undefined) return hit
  let out: NoSpanBits | null = null
  const c = container as { data?: unknown, palette?: unknown }
  const ba = c.data as { data?: unknown, bitsPerValue?: unknown, valuesPerLong?: unknown, valueMask?: unknown, capacity?: unknown } | undefined
  if (ba && ba.data instanceof Uint32Array &&
      typeof ba.bitsPerValue === 'number' && typeof ba.valuesPerLong === 'number' &&
      typeof ba.valueMask === 'number' && ba.capacity === 4096 &&
      ba.valuesPerLong === Math.floor(64 / ba.bitsPerValue) &&
      ba.valueMask === (1 << ba.bitsPerValue) - 1 &&
      ba.data.length >= Math.ceil(4096 / ba.valuesPerLong) * 2) {
    const palette = c.palette
    if (Array.isArray(palette)) {
      const n = palette.length
      const flags = new Uint8Array(n)
      const heights = new Uint8Array(n)
      const special = new Uint8Array(n)
      const thin = new Uint8Array(n)
      const states = new Uint16Array(n)
      let ok = true
      for (let i = 0; i < n; i++) {
        const s = palette[i]
        if (typeof s !== 'number' || s < 0 || s > lut.maxStateId) { ok = false; break }
        flags[i] = lut.flags[s]
        heights[i] = lut.heights[s]
        special[i] = lut.special ? lut.special[s] : 0
        thin[i] = lut.thin ? lut.thin[s] : 0
        states[i] = s
      }
      if (ok) {
        out = {
          words: ba.data, bitsPerValue: ba.bitsPerValue, valuesPerLong: ba.valuesPerLong, valueMask: ba.valueMask,
          flags, heights, special, thin, states
        }
      }
    } else if (palette === undefined) {
      // Direct containers store state IDs rather than palette indices.
      const n = lut.maxStateId + 1
      if (directStates === null || directStates.length !== n) {
        directStates = new Uint16Array(n)
        for (let i = 0; i < n; i++) directStates[i] = i
      }
      // Without a special grid this empty array is never indexed.
      out = {
        words: ba.data, bitsPerValue: ba.bitsPerValue, valuesPerLong: ba.valuesPerLong, valueMask: ba.valueMask,
        flags: lut.flags, heights: lut.heights, special: lut.special ?? NO_SPECIAL, thin: lut.thin ?? NO_SPECIAL, states: directStates
      }
    }
  }
  cache.set(container, out)
  return out
}

let directStates: Uint16Array | null = null

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
  const thin = lut.thin ? snap.allocThin() : null

  const world = bot.world as unknown as WorldLike
  const flags = snap.flags
  const heights = snap.heights
  const lutFlags = lut.flags
  const lutHeights = lut.heights
  const lutSpecial = lut.special
  const lutThin = lut.thin
  const maxStateId = lut.maxStateId

  const cx0 = box.x0 >> 4
  const cx1 = box.x1 >> 4
  const cz0 = box.z0 >> 4
  const cz1 = box.z1 >> 4
  const probe = { x: 0, y: 0, z: 0 }
  const bitsCache = new Map<unknown, NoSpanBits | null>()
  const counting = process.env.PF_SOLVE_TIMING === '1'
  let rowsUniform = 0
  let rowsFast = 0
  let rowsGet = 0
  let rowsGeneric = 0
  let colsMissing = 0

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
      if (!column) { colsMissing++; snap.missingColumns.add(`${cx},${cz}`); continue } // Unloaded cells remain unsafe and non-physical.

      // Read known section layouts directly; unfamiliar layouts use generic cell access.
      const sections = column.sections
      const colMinY = column.minY
      const useSections = Array.isArray(sections) && typeof colMinY === 'number' && (colMinY & 15) === 0

      for (let y = box.y0; y <= box.y1; y++) {
        probe.y = y
        const rowBase = (y - meta.y0) * meta.l

        let uniformState = -1
        let getCell: ((idx: number) => number) | null = null
        let bits: NoSpanBits | null = null
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
          } else {
            // Resolve the palette once, then unpack no-span values without per-cell method calls.
            bits = noSpanBits(sec.data, lut, bitsCache)
            if (bits !== null) {
              sectionOk = true
            } else if (typeof sec.data.get === 'function') {
              getCell = sec.data.get.bind(sec.data)
              sectionOk = true
            }
          }
        }
        const secY = ((y - (colMinY as number || 0)) & 15) << 8

        for (let z = bz0; z <= bz1; z++) {
          probe.z = z & 15
          const base = (rowBase + (z - meta.z0)) * meta.w - meta.x0
          if (sectionOk && uniformState >= 0) {
            rowsUniform++
            if (uniformState <= maxStateId) {
              const f = lutFlags[uniformState]
              const h = lutHeights[uniformState]
              const sp = lutSpecial ? lutSpecial[uniformState] : 0
              const th = lutThin ? lutThin[uniformState] : 0
              for (let x = bx0; x <= bx1; x++) {
                const idx = base + x
                flags[idx] = f
                heights[idx] = h
                if (states) states[idx] = uniformState
                if (special) special[idx] = sp
                if (thin) thin[idx] = th
              }
            }
            continue
          }
          if (sectionOk && bits !== null) {
            rowsFast++
            const zPart = secY | ((z & 15) << 4)
            const words = bits.words
            const bpv = bits.bitsPerValue
            const vpl = bits.valuesPerLong
            const mask = bits.valueMask
            const pf = bits.flags
            const ph = bits.heights
            const ps = bits.special
            const pt = bits.thin
            const pst = bits.states
            for (let x = bx0; x <= bx1; x++) {
              const ci = zPart | (x & 15)
              // Mirror of BitArrayNoSpan.get: values never straddle a long.
              const li = (ci / vpl) | 0
              const inLong = (ci - li * vpl) * bpv
              let v: number
              if (inLong >= 32) {
                v = (words[li * 2 + 1] >>> (inLong - 32)) & mask
              } else {
                v = words[li * 2] >>> inLong
                if (inLong + bpv > 32) v |= words[li * 2 + 1] << (32 - inLong)
                v &= mask
              }
              if (v < pf.length) {
                const idx = base + x
                flags[idx] = pf[v]
                heights[idx] = ph[v]
                if (states) states[idx] = pst[v]
                if (special) special[idx] = ps[v]
                if (thin) thin[idx] = pt[v]
              }
            }
            continue
          }
          if (sectionOk && getCell) {
            rowsGet++
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
                  if (thin) thin[idx] = lutThin![stateId]
                }
              }
              continue
            } catch {
            }
          }
          rowsGeneric++
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
              if (thin) thin[idx] = lutThin![stateId]
            }
          }
        }
      }
    }
  }

  if (counting) {
    console.log(`[pf-timing] snapshot rows: fast=${rowsFast} uniform=${rowsUniform} get=${rowsGet} generic=${rowsGeneric} missingColumns=${colsMissing}`)
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
    if (snap.thin) snap.thin[idx] = lut.thin ? lut.thin[stateId] : 0
  } else {
    snap.flags[idx] = 0
    snap.heights[idx] = 0
    if (snap.states) snap.states[idx] = 0
    if (snap.special) snap.special[idx] = 0
    if (snap.thin) snap.thin[idx] = 0
  }
  return true
}

/** Bake the movements' entity-intersection index into sparse per-cell weights. */
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
