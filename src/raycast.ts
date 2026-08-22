// Voxel raycast over a snapshot, using the real per-state collision shapes
// from the LUT — the same AABBs prismarine-world's RaycastIterator tests, so
// GoalLookAtBlock's "clean face ray" guarantee holds identically when the
// solve runs inside the worker. Face encoding matches prismarine-world:
// 0:-y 1:+y 2:-z 3:+z 4:-x 5:+x.
import { Vec3 } from 'vec3'
import type { BlockLut } from './lut.js'
import type { SnapshotMeta } from './types.js'
import type { RaycastWorld } from './goals.js'

export interface RaycastLut {
  maxStateId: number
  shapeStarts: Int32Array
  shapeCounts: Uint8Array
  shapeData: Float32Array
}

export class SnapshotRaycastWorld implements RaycastWorld {
  private readonly meta: SnapshotMeta
  private readonly states: Uint16Array
  private readonly lut: RaycastLut

  constructor (meta: SnapshotMeta, states: Uint16Array, lut: RaycastLut | BlockLut) {
    this.meta = meta
    this.states = states
    this.lut = lut
  }

  private stateAt (x: number, y: number, z: number): number {
    const m = this.meta
    const lx = x - m.x0
    const ly = y - m.y0
    const lz = z - m.z0
    if (lx < 0 || lx >= m.w || ly < 0 || ly >= m.h || lz < 0 || lz >= m.l) return -1
    return this.states[(ly * m.l + lz) * m.w + lx]
  }

  getBlock (pos: Vec3): { shapes: number[][] } | null {
    const state = this.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
    if (state < 0) return null
    const start = this.lut.shapeStarts[state]
    const count = this.lut.shapeCounts[state]
    const shapes: number[][] = []
    if (start >= 0) {
      for (let i = 0; i < count; i++) {
        const o = (start + i) * 6
        const d = this.lut.shapeData
        shapes.push([d[o], d[o + 1], d[o + 2], d[o + 3], d[o + 4], d[o + 5]])
      }
    }
    return { shapes }
  }

  raycast (from: Vec3, direction: Vec3, range: number): { position: Vec3, face: number } | null {
    const dx = direction.x
    const dy = direction.y
    const dz = direction.z

    let cx = Math.floor(from.x)
    let cy = Math.floor(from.y)
    let cz = Math.floor(from.z)

    const stepX = dx > 0 ? 1 : dx < 0 ? -1 : 0
    const stepY = dy > 0 ? 1 : dy < 0 ? -1 : 0
    const stepZ = dz > 0 ? 1 : dz < 0 ? -1 : 0

    const tDeltaX = dx !== 0 ? Math.abs(1 / dx) : Infinity
    const tDeltaY = dy !== 0 ? Math.abs(1 / dy) : Infinity
    const tDeltaZ = dz !== 0 ? Math.abs(1 / dz) : Infinity

    let tMaxX = dx !== 0 ? Math.abs((cx + (stepX > 0 ? 1 : 0) - from.x) / dx) : Infinity
    let tMaxY = dy !== 0 ? Math.abs((cy + (stepY > 0 ? 1 : 0) - from.y) / dy) : Infinity
    let tMaxZ = dz !== 0 ? Math.abs((cz + (stepZ > 0 ? 1 : 0) - from.z) / dz) : Infinity

    let t = 0
    const shapeStarts = this.lut.shapeStarts
    const shapeCounts = this.lut.shapeCounts
    const shapeData = this.lut.shapeData

    while (t <= range) {
      const state = this.stateAt(cx, cy, cz)
      if (state >= 0) {
        const start = shapeStarts[state]
        if (start >= 0) {
          const count = shapeCounts[state]
          let bestT = Infinity
          let bestFace = -1
          for (let i = 0; i < count; i++) {
            const o = (start + i) * 6
            const hit = intersectBox(
              from.x - cx, from.y - cy, from.z - cz, dx, dy, dz,
              shapeData[o], shapeData[o + 1], shapeData[o + 2],
              shapeData[o + 3], shapeData[o + 4], shapeData[o + 5]
            )
            if (hit !== null && hit.t < bestT) {
              bestT = hit.t
              bestFace = hit.face
            }
          }
          if (bestFace >= 0 && bestT <= range) {
            return { position: new Vec3(cx, cy, cz), face: bestFace }
          }
        }
      }

      // Advance to the next cell.
      if (tMaxX < tMaxY) {
        if (tMaxX < tMaxZ) {
          t = tMaxX
          tMaxX += tDeltaX
          cx += stepX
        } else {
          t = tMaxZ
          tMaxZ += tDeltaZ
          cz += stepZ
        }
      } else {
        if (tMaxY < tMaxZ) {
          t = tMaxY
          tMaxY += tDeltaY
          cy += stepY
        } else {
          t = tMaxZ
          tMaxZ += tDeltaZ
          cz += stepZ
        }
      }
      if (stepX === 0 && stepY === 0 && stepZ === 0) break
    }
    return null
  }
}

/** Slab-method ray/AABB in cell-local coordinates. Returns entry t + face. */
function intersectBox (
  ox: number, oy: number, oz: number,
  dx: number, dy: number, dz: number,
  x0: number, y0: number, z0: number,
  x1: number, y1: number, z1: number
): { t: number, face: number } | null {
  let tMin = -Infinity
  let tMax = Infinity
  let face = -1

  if (dx !== 0) {
    const inv = 1 / dx
    let t0 = (x0 - ox) * inv
    let t1 = (x1 - ox) * inv
    let f = dx > 0 ? 4 : 5 // entering through -x face (4) or +x face (5)
    if (t0 > t1) { const tmp = t0; t0 = t1; t1 = tmp }
    if (t0 > tMin) { tMin = t0; face = f }
    if (t1 < tMax) tMax = t1
  } else if (ox < x0 || ox > x1) {
    return null
  }

  if (dy !== 0) {
    const inv = 1 / dy
    let t0 = (y0 - oy) * inv
    let t1 = (y1 - oy) * inv
    const f = dy > 0 ? 0 : 1
    if (t0 > t1) { const tmp = t0; t0 = t1; t1 = tmp }
    if (t0 > tMin) { tMin = t0; face = f }
    if (t1 < tMax) tMax = t1
  } else if (oy < y0 || oy > y1) {
    return null
  }

  if (dz !== 0) {
    const inv = 1 / dz
    let t0 = (z0 - oz) * inv
    let t1 = (z1 - oz) * inv
    const f = dz > 0 ? 2 : 3
    if (t0 > t1) { const tmp = t0; t0 = t1; t1 = tmp }
    if (t0 > tMin) { tMin = t0; face = f }
    if (t1 < tMax) tMax = t1
  } else if (oz < z0 || oz > z1) {
    return null
  }

  if (tMax < tMin || tMax < 0) return null
  return { t: Math.max(0, tMin), face }
}
