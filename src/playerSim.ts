// Allocation-free normal-movement physics matching prismarine-physics over state-id shape tables.
// Look direction is (-sin yaw, -cos yaw), keeping rollouts free of trigonometry.
// Liquids, fire and cobwebs stop the model via inLiquid; elytra, levitation and slow falling are unsupported.
// As in prismarine-physics 1.21, soul sand and honey do not slow movement, and trapdoors do not extend ladders.

/** Flat state-id grid for direct cell lookups; unstamped grids return -1 outside their bounds. */
export interface SimGrid {
  states: Uint16Array | Int32Array
  x0: number
  y0: number
  z0: number
  w: number
  h: number
  l: number
  /**
   * A stamped cell is current only when its stamp equals `serial`; other cells
   * read through stateAt. Incrementing serial invalidates the whole window.
   */
  stamps?: Int32Array
  serial?: number
}

/** World state and collision tables read by the simulation. */
export interface SimWorld {
  /** State id at a cell, or -1 outside the loaded world (no collision, no support). */
  stateAt: (x: number, y: number, z: number) => number
  /** Optional flat grid of the same world; stateAt must agree with its cells. */
  grid?: SimGrid
  /** Box offset (in boxes) into `boxes`, per state; -1 for no collision. */
  shapeStart: Int32Array
  /** Number of boxes per state. */
  shapeCount: Uint8Array
  /** [x0, y0, z0, x1, y1, z1] per box, block-relative. */
  boxes: Float64Array
  /** Per state: slipperiness (0 = default 0.6). */
  slip: Float64Array
  /** Per state: SIM_* flags. */
  kind: Uint8Array
}

export const SIM_CLIMBABLE = 1
export const SIM_LIQUID = 2
export const SIM_SLIME = 4
/** Hay and honey retain one fifth of fall damage. */
export const SIM_CUSHION = 8
export const SIM_BED = 16
/**
 * Fire triggers the executor's escape controls, while cobweb drag is unmodelled.
 * Both stop a rollout via inLiquid when hazard checks are enabled.
 */
export const SIM_HAZARD = 32
/** Enables stopping rollouts at hazard cells. */
const KERNEL_HAZARD = typeof process === 'undefined' || process.env.PF_KERNEL_HAZARD !== '0'
/**
 * Honey scales jump velocity by 0.4, sampled below the floored feet.
 * On honey's 15/16 top, that sample is below the honey block itself.
 */
export const SIM_HONEY = 64
/** Honey jump multiplier used by prismarine-physics. */
const HONEY_JUMP = 0.4
/** Enables logging each encountered hazard cell once. */
const HAZARD_DEBUG = typeof process !== 'undefined' && process.env.PF_HAZARD_DEBUG === '1'
const hazardSeen = new Set<string>()

/** Mutable body state; one per simulated player. */
export interface SimBody {
  x: number
  y: number
  z: number
  vx: number
  vy: number
  vz: number
  onGround: boolean
  collidedH: boolean
  collidedV: boolean
  jumpTicks: number
  /** Set on liquid or enabled hazard contact; the model stops at that position. */
  inLiquid: boolean
}

export interface SimControl {
  forward: boolean
  back: boolean
  left: boolean
  right: boolean
  jump: boolean
  sprint: boolean
  sneak: boolean
  /** Unit look direction on the ground plane: (-sin yaw, -cos yaw). */
  hx: number
  hz: number
}

/**
 * Match the bot's jump rule per thread: vanilla 1.21 preserves a faster upward
 * velocity, such as a slime rebound; stock prismarine-physics replaces it.
 */
let vanillaJump = false
export function setVanillaJump (on: boolean): void { vanillaJump = on }
export function isVanillaJump (): boolean { return vanillaJump }

export const SIM = {
  gravity: 0.08,
  airdrag: Math.fround(1 - 0.02),
  jumpVelocity: Math.fround(0.42),
  sprintJumpBoost: 0.2,
  playerSpeed: 0.1,
  sprintSpeed: 0.3,
  sneakSpeed: 0.3,
  stepHeight: 0.6,
  negligibleVelocity: 0.003,
  ladderMaxSpeed: 0.15,
  ladderClimbSpeed: 0.2,
  airborneInertia: 0.91,
  airborneAcceleration: 0.02,
  defaultSlipperiness: 0.6,
  autojumpCooldown: 10
} as const

export function newBody (x: number, y: number, z: number): SimBody {
  return { x, y, z, vx: 0, vy: 0, vz: 0, onGround: false, collidedH: false, collidedV: false, jumpTicks: 0, inLiquid: false }
}

export function copyBody (dst: SimBody, src: SimBody): SimBody {
  dst.x = src.x; dst.y = src.y; dst.z = src.z
  dst.vx = src.vx; dst.vy = src.vy; dst.vz = src.vz
  dst.onGround = src.onGround; dst.collidedH = src.collidedH; dst.collidedV = src.collidedV
  dst.jumpTicks = src.jumpTicks; dst.inLiquid = src.inLiquid
  return dst
}

/** Invalid range serial; valid grid serials are nonnegative. */
const NO_RANGE = -1
/** Brick side as a shift: 2 means four cells per side. */
const BRICK = 2
const BRICK_MASK = (1 << BRICK) - 1
const BRICK_SEEN = 1
/** A cell of the brick has a collision box. */
const BRICK_SOLID = 2
/** A cell of the brick is a liquid. */
const BRICK_WET = 4
/** A cell of the brick is a fire or a cobweb (SIM_HAZARD). */
const BRICK_HOT = 8

/** Player simulator with shared scratch space; use one instance per thread without re-entry. */
export class PlayerSim {
  readonly world: SimWorld
  /** Body half-width and height; the executor slightly reduces the stock 0.3 and 1.8. */
  halfWidth: number
  height: number
  /** Movement-speed attribute base (0.1 unless the server says otherwise). */
  speedBase: number
  /** Scratch collision boxes in absolute coordinates, six numbers per box. */
  private buf: Float64Array = new Float64Array(6 * 512)
  private nBoxes = 0
  /**
   * Cache two cell ranges so step-up queries do not evict the preceding move.
   * Stamped grids cache in-window ranges by serial; unstamped grids must stay fixed.
   */
  private readonly slotBuf: Float64Array[] = [this.buf, new Float64Array(6 * 512)]
  private readonly slotN = [0, 0]
  /** Per slot: x0, y0, z0, x1, y1, z1, serial; NO_RANGE marks an empty slot. */
  private readonly slotKey = new Float64Array([0, 0, 0, 0, 0, 0, NO_RANGE, 0, 0, 0, 0, 0, 0, NO_RANGE])
  private slot = 0
  /** Liquid-contact cache key by cell range and grid serial. */
  private readonly liqKey = new Float64Array([0, 0, 0, 0, 0, 0, NO_RANGE])
  private liqLast = false
  /** Hazard-contact cache key by cell range and grid serial. */
  private readonly hazKey = new Float64Array([0, 0, 0, 0, 0, 0, NO_RANGE])
  private hazLast = false
  /**
   * Lazy occupancy bits for 4x4x4-cell bricks in unstamped grids only.
   * Skip empty layers and dry or hazard-free regions without reading every cell.
   */
  private bricks: Uint8Array | null = null
  private bw = 0
  private bl = 0
  private readonly grid: SimGrid | null

  constructor (world: SimWorld, halfWidth = 0.3, height = 1.8, speedBase: number = SIM.playerSpeed) {
    this.world = world
    this.grid = world.grid ?? null
    this.halfWidth = halfWidth
    this.height = height
    this.speedBase = speedBase
  }

  /** State id at a cell, using the grid when available. */
  stateAt (x: number, y: number, z: number): number {
    return this.sid(x, y, z)
  }

  /** State lookup through the grid or the world's stateAt callback. */
  private sid (x: number, y: number, z: number): number {
    const g = this.grid
    if (g === null) return this.world.stateAt(x, y, z)
    const lx = x - g.x0
    const ly = y - g.y0
    const lz = z - g.z0
    const st = g.stamps
    if (lx < 0 || lx >= g.w || ly < 0 || ly >= g.h || lz < 0 || lz >= g.l) return st === undefined ? -1 : this.world.stateAt(x, y, z)
    const i = (ly * g.l + lz) * g.w + lx
    if (st !== undefined && st[i] !== g.serial) {
      g.states[i] = this.world.stateAt(x, y, z)
      st[i] = g.serial as number
    }
    return g.states[i]
  }

  /**
   * Grid serial for caching a cell range; zero for fixed grids, NO_RANGE without
   * a grid or outside a stamped window, where reads cannot safely be cached.
   */
  private rangeSerial (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): number {
    const g = this.grid
    if (g === null) return NO_RANGE
    if (g.stamps === undefined) return 0
    if (x0 < g.x0 || y0 < g.y0 || z0 < g.z0 || x1 >= g.x0 + g.w || y1 >= g.y0 + g.h || z1 >= g.z0 + g.l) return NO_RANGE
    return g.serial as number
  }

  /** Collect collision boxes using prismarine-physics' surrounding-cell range. */
  private gather (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number {
    const y0 = Math.floor(minY) - 1
    const y1 = Math.floor(maxY)
    const z0 = Math.floor(minZ)
    const z1 = Math.floor(maxZ)
    const x0 = Math.floor(minX)
    const x1 = Math.floor(maxX)
    const serial = this.rangeSerial(x0, y0, z0, x1, y1, z1)
    const key = this.slotKey
    if (serial !== NO_RANGE) {
      for (let j = 0; j < 2; j++) {
        const sl = (this.slot + j) & 1
        const o = sl * 7
        if (key[o + 6] === serial && key[o] === x0 && key[o + 1] === y0 && key[o + 2] === z0 && key[o + 3] === x1 && key[o + 4] === y1 && key[o + 5] === z1) {
          this.slot = sl
          this.buf = this.slotBuf[sl]
          this.nBoxes = this.slotN[sl]
          return this.nBoxes
        }
      }
    }
    // Keep the previous query in the other slot in case the next move reuses it.
    const sl = this.slot ^ 1
    this.slot = sl
    this.buf = this.slotBuf[sl]
    const n = this.collect(x0, y0, z0, x1, y1, z1)
    this.slotBuf[sl] = this.buf
    this.slotN[sl] = n
    const o = sl * 7
    key[o] = x0; key[o + 1] = y0; key[o + 2] = z0; key[o + 3] = x1; key[o + 4] = y1; key[o + 5] = z1; key[o + 6] = serial
    this.nBoxes = n
    return n
  }

  /** Fill the scratch buffer with collision boxes in cell traversal order; return the count. */
  private collect (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): number {
    const w = this.world
    const g = this.grid
    let n = 0
    // An in-grid range needs no per-cell bounds checks.
    const inside = g !== null && x0 >= g.x0 && y0 >= g.y0 && z0 >= g.z0 && x1 < g.x0 + g.w && y1 < g.y0 + g.h && z1 < g.z0 + g.l
    const states = g !== null ? g.states : null
    const stamps = g !== null ? g.stamps : undefined
    const serial = g !== null ? g.serial : undefined
    const sparse = inside && stamps === undefined
    for (let y = y0; y <= y1; y++) {
      if (sparse) {
        const gg = g as SimGrid
        const ly = y - gg.y0
        if ((y === y0 || (ly & BRICK_MASK) === 0) && this.bricksLack(gg, BRICK_SOLID, x0 - gg.x0, ly, z0 - gg.z0, x1 - gg.x0, ly, z1 - gg.z0)) {
          y = gg.y0 + (((ly >> BRICK) + 1) << BRICK) - 1
          continue
        }
      }
      for (let z = z0; z <= z1; z++) {
        let i = inside ? ((y - (g as SimGrid).y0) * (g as SimGrid).l + z - (g as SimGrid).z0) * (g as SimGrid).w + x0 - (g as SimGrid).x0 : -1
        for (let x = x0; x <= x1; x++, i++) {
          let s: number
          if (!inside) {
            s = this.sid(x, y, z)
          } else if (stamps !== undefined && stamps[i] !== serial) {
            s = w.stateAt(x, y, z)
            ;(states as SimGrid['states'])[i] = s
            stamps[i] = serial as number
          } else {
            s = (states as SimGrid['states'])[i]
          }
          if (s < 0) continue
          const start = w.shapeStart[s]
          if (start < 0) continue
          const count = w.shapeCount[s]
          if ((n + count) * 6 > this.buf.length) {
            const bigger = new Float64Array(this.buf.length * 2)
            bigger.set(this.buf)
            this.buf = bigger
          }
          for (let k = 0; k < count; k++) {
            const o = (start + k) * 6
            const d = n * 6
            this.buf[d] = w.boxes[o] + x
            this.buf[d + 1] = w.boxes[o + 1] + y
            this.buf[d + 2] = w.boxes[o + 2] + z
            this.buf[d + 3] = w.boxes[o + 3] + x
            this.buf[d + 4] = w.boxes[o + 4] + y
            this.buf[d + 5] = w.boxes[o + 5] + z
            n++
          }
        }
      }
    }
    return n
  }

  /** Test for any surrounding collision boxes using the sneak edge guard's cell range. */
  private anyBoxes (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): boolean {
    const w = this.world
    const y0 = Math.floor(minY) - 1
    const y1 = Math.floor(maxY)
    const z0 = Math.floor(minZ)
    const z1 = Math.floor(maxZ)
    const x0 = Math.floor(minX)
    const x1 = Math.floor(maxX)
    for (let y = y0; y <= y1; y++) {
      for (let z = z0; z <= z1; z++) {
        for (let x = x0; x <= x1; x++) {
          const s = this.sid(x, y, z)
          if (s >= 0 && w.shapeStart[s] >= 0 && w.shapeCount[s] > 0) return true
        }
      }
    }
    return false
  }

  // Offset calculations use the block box as the obstacle and the player box as the moving box.
  private offY (bMinX: number, bMinY: number, bMinZ: number, bMaxX: number, bMaxY: number, bMaxZ: number, dy: number): number {
    const b = this.buf
    for (let i = 0; i < this.nBoxes; i++) {
      const o = i * 6
      if (bMaxX > b[o] && bMinX < b[o + 3] && bMaxZ > b[o + 2] && bMinZ < b[o + 5]) {
        if (dy > 0.0 && bMaxY <= b[o + 1]) {
          dy = Math.min(b[o + 1] - bMaxY, dy)
        } else if (dy < 0.0 && bMinY >= b[o + 4]) {
          dy = Math.max(b[o + 4] - bMinY, dy)
        }
      }
    }
    return dy
  }

  private offX (bMinX: number, bMinY: number, bMinZ: number, bMaxX: number, bMaxY: number, bMaxZ: number, dx: number): number {
    const b = this.buf
    for (let i = 0; i < this.nBoxes; i++) {
      const o = i * 6
      if (bMaxY > b[o + 1] && bMinY < b[o + 4] && bMaxZ > b[o + 2] && bMinZ < b[o + 5]) {
        if (dx > 0.0 && bMaxX <= b[o]) {
          dx = Math.min(b[o] - bMaxX, dx)
        } else if (dx < 0.0 && bMinX >= b[o + 3]) {
          dx = Math.max(b[o + 3] - bMinX, dx)
        }
      }
    }
    return dx
  }

  private offZ (bMinX: number, bMinY: number, bMinZ: number, bMaxX: number, bMaxY: number, bMaxZ: number, dz: number): number {
    const b = this.buf
    for (let i = 0; i < this.nBoxes; i++) {
      const o = i * 6
      if (bMaxX > b[o] && bMinX < b[o + 3] && bMaxY > b[o + 1] && bMinY < b[o + 4]) {
        if (dz > 0.0 && bMaxZ <= b[o + 2]) {
          dz = Math.min(b[o + 2] - bMaxZ, dz)
        } else if (dz < 0.0 && bMinZ >= b[o + 5]) {
          dz = Math.max(b[o + 5] - bMinZ, dz)
        }
      }
    }
    return dz
  }

  /** Slipperiness one block below the feet; zero means the default 0.6. */
  slipUnder (x: number, y: number, z: number): number {
    const s = this.sid(Math.floor(x), Math.floor(y - 1), Math.floor(z))
    return s < 0 ? 0 : this.world.slip[s]
  }

  kindAt (x: number, y: number, z: number): number {
    const s = this.sid(Math.floor(x), Math.floor(y), Math.floor(z))
    return s < 0 ? 0 : this.world.kind[s]
  }

  /** Climbable contact uses the feet cell, without a trapdoor extension rule. */
  private onLadder (b: SimBody): boolean {
    return (this.kindAt(b.x, b.y, b.z) & SIM_CLIMBABLE) !== 0
  }

  /** Whether the body's feet occupy a climbable cell. */
  onClimbable (b: SimBody): boolean {
    return this.onLadder(b)
  }

  /** Whether a step stops at liquid or an enabled hazard, without advancing the body. */
  touchesWet (b: SimBody): boolean {
    return this.touchesLiquid(b) || (KERNEL_HAZARD && this.touchesHazard(b))
  }

  /** Liquid contact uses block cells rather than collision shapes. */
  private touchesLiquid (b: SimBody): boolean {
    const hw = this.halfWidth
    const minX = b.x - hw + 0.001; const maxX = b.x + hw - 0.001
    const minY = b.y + 0.401; const maxY = b.y + this.height - 0.401
    const minZ = b.z - hw + 0.001; const maxZ = b.z + hw - 0.001
    const x0 = Math.floor(minX); const x1 = Math.floor(maxX)
    const y0 = Math.floor(minY); const y1 = Math.floor(maxY)
    const z0 = Math.floor(minZ); const z1 = Math.floor(maxZ)
    const g = this.grid
    if (g !== null && g.stamps === undefined && x0 >= g.x0 && y0 >= g.y0 && z0 >= g.z0 && x1 < g.x0 + g.w && y1 < g.y0 + g.h && z1 < g.z0 + g.l &&
      this.bricksLack(g, BRICK_WET, x0 - g.x0, y0 - g.y0, z0 - g.z0, x1 - g.x0, y1 - g.y0, z1 - g.z0)) return false
    // Reuse the answer while the queried cell range and grid serial stay unchanged.
    const serial = this.rangeSerial(x0, y0, z0, x1, y1, z1)
    const k = this.liqKey
    if (serial !== NO_RANGE && k[6] === serial && k[0] === x0 && k[1] === y0 && k[2] === z0 && k[3] === x1 && k[4] === y1 && k[5] === z1) return this.liqLast
    let hit = false
    cells: for (let y = y0; y <= y1; y++) {
      for (let z = z0; z <= z1; z++) {
        for (let x = x0; x <= x1; x++) {
          const s = this.sid(x, y, z)
          if (s >= 0 && (this.world.kind[s] & SIM_LIQUID) !== 0) { hit = true; break cells }
        }
      }
    }
    k[0] = x0; k[1] = y0; k[2] = z0; k[3] = x1; k[4] = y1; k[5] = z1; k[6] = serial
    this.liqLast = hit
    return hit
  }

  /** Hazard contact trims the body box by 0.001 on each side, matching the executor. */
  private touchesHazard (b: SimBody): boolean {
    const hw = this.halfWidth
    const x0 = Math.floor(b.x - hw + 0.001); const x1 = Math.floor(b.x + hw - 0.001)
    const y0 = Math.floor(b.y + 0.001); const y1 = Math.floor(b.y + this.height - 0.001)
    const z0 = Math.floor(b.z - hw + 0.001); const z1 = Math.floor(b.z + hw - 0.001)
    const g = this.grid
    if (g !== null && g.stamps === undefined && x0 >= g.x0 && y0 >= g.y0 && z0 >= g.z0 && x1 < g.x0 + g.w && y1 < g.y0 + g.h && z1 < g.z0 + g.l &&
      this.bricksLack(g, BRICK_HOT, x0 - g.x0, y0 - g.y0, z0 - g.z0, x1 - g.x0, y1 - g.y0, z1 - g.z0)) return false
    const serial = this.rangeSerial(x0, y0, z0, x1, y1, z1)
    const k = this.hazKey
    if (serial !== NO_RANGE && k[6] === serial && k[0] === x0 && k[1] === y0 && k[2] === z0 && k[3] === x1 && k[4] === y1 && k[5] === z1) return this.hazLast
    let hit = false
    cells: for (let y = y0; y <= y1; y++) {
      for (let z = z0; z <= z1; z++) {
        for (let x = x0; x <= x1; x++) {
          const st = this.sid(x, y, z)
          if (st >= 0 && (this.world.kind[st] & SIM_HAZARD) !== 0) {
            hit = true
            if (HAZARD_DEBUG && !hazardSeen.has(`${x},${y},${z},${st}`)) { hazardSeen.add(`${x},${y},${z},${st}`); console.warn(`[hazard] ${x},${y},${z} state ${st}`) }
            break cells
          }
        }
      }
    }
    k[0] = x0; k[1] = y0; k[2] = z0; k[3] = x1; k[4] = y1; k[5] = z1; k[6] = serial
    this.hazLast = hit
    return hit
  }

  /** Whether every brick in an unstamped grid's cell range lacks `bit`. */
  private bricksLack (g: SimGrid, bit: number, lx0: number, ly0: number, lz0: number, lx1: number, ly1: number, lz1: number): boolean {
    for (let by = ly0 >> BRICK; by <= ly1 >> BRICK; by++) {
      for (let bz = lz0 >> BRICK; bz <= lz1 >> BRICK; bz++) {
        for (let bx = lx0 >> BRICK; bx <= lx1 >> BRICK; bx++) {
          if ((this.brick(g, bx, by, bz) & bit) !== 0) return false
        }
      }
    }
    return true
  }

  /** Lazily read a brick's occupancy bits. */
  private brick (g: SimGrid, bx: number, by: number, bz: number): number {
    let bricks = this.bricks
    if (bricks === null) {
      this.bw = ((g.w - 1) >> BRICK) + 1
      this.bl = ((g.l - 1) >> BRICK) + 1
      bricks = this.bricks = new Uint8Array(this.bw * (((g.h - 1) >> BRICK) + 1) * this.bl)
    }
    const bi = (by * this.bl + bz) * this.bw + bx
    let v = bricks[bi]
    if (v !== 0) return v
    v = BRICK_SEEN
    const w = this.world
    const ye = Math.min(g.h, (by + 1) << BRICK)
    const ze = Math.min(g.l, (bz + 1) << BRICK)
    const xe = Math.min(g.w, (bx + 1) << BRICK)
    for (let y = by << BRICK; y < ye; y++) {
      for (let z = bz << BRICK; z < ze; z++) {
        for (let x = bx << BRICK, i = (y * g.l + z) * g.w + x; x < xe; x++, i++) {
          const st = g.states[i]
          if (st < 0) continue
          if (w.shapeStart[st] >= 0 && w.shapeCount[st] > 0) v |= BRICK_SOLID
          if ((w.kind[st] & SIM_LIQUID) !== 0) v |= BRICK_WET
          if ((w.kind[st] & SIM_HAZARD) !== 0) v |= BRICK_HOT
        }
      }
    }
    bricks[bi] = v
    return v
  }

  /**
   * Test the body box at (x, y, z) against collision shapes, expanding its sides
   * by `grow` and trimming its top and bottom by `trim`. Negative grow shrinks it.
   */
  boxHits (x: number, y: number, z: number, grow: number, trim: number): boolean {
    const hw = this.halfWidth + grow
    const minX = x - hw; const maxX = x + hw
    const minY = y + trim; const maxY = y + this.height - trim
    const minZ = z - hw; const maxZ = z + hw
    const n = this.gather(minX, minY, minZ, maxX, maxY, maxZ)
    const d = this.buf
    for (let i = 0; i < n; i++) {
      const o = i * 6
      if (minX < d[o + 3] && maxX > d[o] && minY < d[o + 4] && maxY > d[o + 1] && minZ < d[o + 5] && maxZ > d[o + 2]) return true
    }
    return false
  }

  /**
   * Gather boxes for probes within `reach` horizontally and `below` the feet.
   * Use probeHit until the next step or other probe overwrites the shared buffer.
   */
  probeBegin (x: number, y: number, z: number, reach: number, below: number): number {
    const r = this.halfWidth + reach
    return this.gather(x - r, y - below, z - r, x + r, y + this.height, z + r)
  }

  /**
   * Gather collision boxes and return their count. Each box in scratchBoxes is
   * (minX, minY, minZ, maxX, maxY, maxZ); valid until the next step or probe.
   */
  boxesIn (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number {
    return this.gather(minX, minY, minZ, maxX, maxY, maxZ)
  }

  /** Shared scratch buffer filled by boxesIn and the probes. */
  get scratchBoxes (): Float64Array {
    return this.buf
  }

  /** Test the body box against the boxes gathered by probeBegin. */
  probeHit (x: number, y: number, z: number, grow: number, trim: number): boolean {
    const hw = this.halfWidth + grow
    const minX = x - hw; const maxX = x + hw
    const minY = y + trim; const maxY = y + this.height - trim
    const minZ = z - hw; const maxZ = z + hw
    const d = this.buf
    for (let i = 0, n = this.nBoxes; i < n; i++) {
      const o = i * 6
      if (minX < d[o + 3] && maxX > d[o] && minY < d[o + 4] && maxY > d[o + 1] && minZ < d[o + 5] && maxZ > d[o + 2]) return true
    }
    return false
  }

  /** Whether the body overlaps a collision box, indicating an embedded position. */
  collides (b: SimBody): boolean {
    const hw = this.halfWidth
    const minX = b.x - hw; const maxX = b.x + hw
    const minY = b.y; const maxY = b.y + this.height
    const minZ = b.z - hw; const maxZ = b.z + hw
    const n = this.gather(minX, minY, minZ, maxX, maxY, maxZ)
    const d = this.buf
    for (let i = 0; i < n; i++) {
      const o = i * 6
      if (minX < d[o + 3] && maxX > d[o] && minY < d[o + 4] && maxY > d[o + 1] && minZ < d[o + 5] && maxZ > d[o + 2]) return true
    }
    return false
  }

  /**
   * Check for a supporting collision box at the feet; onGround can remain true
   * after horizontal motion leaves an edge, allowing one more jump before falling.
   */
  carried (b: SimBody): boolean {
    const hw = this.halfWidth
    const minX = b.x - hw; const maxX = b.x + hw
    const minZ = b.z - hw; const maxZ = b.z + hw
    const n = this.gather(minX, b.y - 0.01, minZ, maxX, b.y, maxZ)
    const d = this.buf
    for (let i = 0; i < n; i++) {
      const o = i * 6
      if (minX < d[o + 3] && maxX > d[o] && minZ < d[o + 5] && maxZ > d[o + 2] && Math.abs(d[o + 4] - b.y) <= 1e-6) return true
    }
    return false
  }

  /**
   * Check support from column (cx, cz) at the feet, distinguishing standing
   * on that column from overhanging it while supported by a neighbouring column.
   */
  supportedIn (b: SimBody, cx: number, cz: number): boolean {
    const hw = this.halfWidth
    const w = this.world
    for (let yy = Math.floor(b.y - 1e-7) - 1; yy <= Math.floor(b.y - 1e-7); yy++) {
      const s = this.sid(cx, yy, cz)
      if (s < 0) continue
      const start = w.shapeStart[s]
      if (start < 0) continue
      for (let k = 0; k < w.shapeCount[s]; k++) {
        const o = (start + k) * 6
        if (Math.abs(yy + w.boxes[o + 4] - b.y) > 1e-6) continue
        if (b.x - hw < cx + w.boxes[o + 3] && b.x + hw > cx + w.boxes[o] &&
            b.z - hw < cz + w.boxes[o + 5] && b.z + hw > cz + w.boxes[o + 2]) return true
      }
    }
    return false
  }

  /** Resolve normal-movement collisions using prismarine-physics' axis order. */
  private moveEntity (b: SimBody, sneak: boolean, dx: number, dy: number, dz: number): void {
    const hw = this.halfWidth
    const h = this.height
    let oldVelX = dx
    const oldVelY = dy
    let oldVelZ = dz

    if (sneak && b.onGround) {
      const step = 0.05
      for (; dx !== 0 && !this.anyBoxes(b.x - hw + dx, b.y, b.z - hw, b.x + hw + dx, b.y + h, b.z + hw); oldVelX = dx) {
        if (dx < step && dx >= -step) dx = 0
        else if (dx > 0) dx -= step
        else dx += step
      }
      for (; dz !== 0 && !this.anyBoxes(b.x - hw, b.y, b.z - hw + dz, b.x + hw, b.y + h, b.z + hw + dz); oldVelZ = dz) {
        if (dz < step && dz >= -step) dz = 0
        else if (dz > 0) dz -= step
        else dz += step
      }
      while (dx !== 0 && dz !== 0 && !this.anyBoxes(b.x - hw + dx, b.y, b.z - hw + dz, b.x + hw + dx, b.y + h, b.z + hw + dz)) {
        if (dx < step && dx >= -step) dx = 0
        else if (dx > 0) dx -= step
        else dx += step
        if (dz < step && dz >= -step) dz = 0
        else if (dz > 0) dz -= step
        else dz += step
        oldVelX = dx
        oldVelZ = dz
      }
    }

    let pMinX = b.x - hw; let pMinY = b.y; let pMinZ = b.z - hw
    let pMaxX = b.x + hw; let pMaxY = b.y + h; let pMaxZ = b.z + hw
    this.gather(
      dx < 0 ? pMinX + dx : pMinX, dy < 0 ? pMinY + dy : pMinY, dz < 0 ? pMinZ + dz : pMinZ,
      dx < 0 ? pMaxX : pMaxX + dx, dy < 0 ? pMaxY : pMaxY + dy, dz < 0 ? pMaxZ : pMaxZ + dz)
    const oMinX = pMinX; const oMinY = pMinY; const oMinZ = pMinZ
    const oMaxX = pMaxX; const oMaxY = pMaxY; const oMaxZ = pMaxZ

    dy = this.offY(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ, dy)
    pMinY += dy; pMaxY += dy
    dx = this.offX(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ, dx)
    pMinX += dx; pMaxX += dx
    dz = this.offZ(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ, dz)
    pMinZ += dz; pMaxZ += dz

    if (SIM.stepHeight > 0 && (b.onGround || (dy !== oldVelY && oldVelY < 0)) && (dx !== oldVelX || dz !== oldVelZ)) {
      const oldVelXCol = dx
      const oldVelYCol = dy
      const oldVelZCol = dz
      const cMinX = pMinX; const cMinY = pMinY; const cMinZ = pMinZ
      const cMaxX = pMaxX; const cMaxY = pMaxY; const cMaxZ = pMaxZ

      let sdy: number = SIM.stepHeight
      this.gather(
        oldVelX < 0 ? oMinX + oldVelX : oMinX, oMinY, oldVelZ < 0 ? oMinZ + oldVelZ : oMinZ,
        oldVelX < 0 ? oMaxX : oMaxX + oldVelX, oMaxY + sdy, oldVelZ < 0 ? oMaxZ : oMaxZ + oldVelZ)

      // Compare step clearance over the swept footprint and the original footprint.
      let b1MinX = oMinX; let b1MinY = oMinY; let b1MinZ = oMinZ; let b1MaxX = oMaxX; let b1MaxY = oMaxY; let b1MaxZ = oMaxZ
      let b2MinX = oMinX; let b2MinY = oMinY; let b2MinZ = oMinZ; let b2MaxX = oMaxX; let b2MaxY = oMaxY; let b2MaxZ = oMaxZ
      const xzMinX = dx < 0 ? oMinX + dx : oMinX
      const xzMaxX = dx < 0 ? oMaxX : oMaxX + dx
      const xzMinZ = dz < 0 ? oMinZ + dz : oMinZ
      const xzMaxZ = dz < 0 ? oMaxZ : oMaxZ + dz

      let dy1 = sdy
      let dy2 = sdy
      dy1 = this.offY(xzMinX, oMinY, xzMinZ, xzMaxX, oMaxY, xzMaxZ, dy1)
      dy2 = this.offY(b2MinX, b2MinY, b2MinZ, b2MaxX, b2MaxY, b2MaxZ, dy2)
      b1MinY += dy1; b1MaxY += dy1
      b2MinY += dy2; b2MaxY += dy2

      let dx1 = oldVelX
      let dx2 = oldVelX
      dx1 = this.offX(b1MinX, b1MinY, b1MinZ, b1MaxX, b1MaxY, b1MaxZ, dx1)
      dx2 = this.offX(b2MinX, b2MinY, b2MinZ, b2MaxX, b2MaxY, b2MaxZ, dx2)
      b1MinX += dx1; b1MaxX += dx1
      b2MinX += dx2; b2MaxX += dx2

      let dz1 = oldVelZ
      let dz2 = oldVelZ
      dz1 = this.offZ(b1MinX, b1MinY, b1MinZ, b1MaxX, b1MaxY, b1MaxZ, dz1)
      dz2 = this.offZ(b2MinX, b2MinY, b2MinZ, b2MaxX, b2MaxY, b2MaxZ, dz2)
      b1MinZ += dz1; b1MaxZ += dz1
      b2MinZ += dz2; b2MaxZ += dz2

      const norm1 = dx1 * dx1 + dz1 * dz1
      const norm2 = dx2 * dx2 + dz2 * dz2
      if (norm1 > norm2) {
        dx = dx1; sdy = -dy1; dz = dz1
        pMinX = b1MinX; pMinY = b1MinY; pMinZ = b1MinZ; pMaxX = b1MaxX; pMaxY = b1MaxY; pMaxZ = b1MaxZ
      } else {
        dx = dx2; sdy = -dy2; dz = dz2
        pMinX = b2MinX; pMinY = b2MinY; pMinZ = b2MinZ; pMaxX = b2MaxX; pMaxY = b2MaxY; pMaxZ = b2MaxZ
      }
      sdy = this.offY(pMinX, pMinY, pMinZ, pMaxX, pMaxY, pMaxZ, sdy)
      pMinY += sdy; pMaxY += sdy
      dy = sdy

      if (oldVelXCol * oldVelXCol + oldVelZCol * oldVelZCol >= dx * dx + dz * dz) {
        dx = oldVelXCol
        dy = oldVelYCol
        dz = oldVelZCol
        pMinX = cMinX; pMinY = cMinY; pMinZ = cMinZ; pMaxX = cMaxX; pMaxY = cMaxY; pMaxZ = cMaxZ
      }
    }

    b.x = pMinX + hw
    b.y = pMinY
    b.z = pMinZ + hw
    b.collidedH = dx !== oldVelX || dz !== oldVelZ
    b.collidedV = dy !== oldVelY
    b.onGround = b.collidedV && oldVelY < 0

    if (dx !== oldVelX) b.vx = 0
    if (dz !== oldVelZ) b.vz = 0
    if (dy !== oldVelY) {
      if (!sneak && (this.kindAt(b.x, b.y - 0.2, b.z) & SIM_SLIME) !== 0) b.vy = -b.vy
      else b.vy = 0
    }
  }

  /** Advance one normal-movement tick using prismarine-physics' update order. */
  step (b: SimBody, c: SimControl): void {
    if (this.touchesLiquid(b) || (KERNEL_HAZARD && this.touchesHazard(b))) { b.inLiquid = true; return }

    if (Math.abs(b.vx) < SIM.negligibleVelocity) b.vx = 0
    if (Math.abs(b.vy) < SIM.negligibleVelocity) b.vy = 0
    if (Math.abs(b.vz) < SIM.negligibleVelocity) b.vz = 0

    if (c.jump) {
      if (b.jumpTicks > 0) b.jumpTicks--
      if (b.onGround && b.jumpTicks === 0) {
        const power = (this.kindAt(b.x, Math.floor(b.y) - 0.5, b.z) & SIM_HONEY) !== 0 ? SIM.jumpVelocity * HONEY_JUMP : SIM.jumpVelocity
        b.vy = vanillaJump && b.vy > power ? b.vy : power
        if (c.sprint) {
          b.vx += c.hx * SIM.sprintJumpBoost
          b.vz += c.hz * SIM.sprintJumpBoost
        }
        b.jumpTicks = SIM.autojumpCooldown
      }
    } else {
      b.jumpTicks = 0
    }

    let strafe = ((c.right ? 1 : 0) - (c.left ? 1 : 0)) * 0.98
    let forward = ((c.forward ? 1 : 0) - (c.back ? 1 : 0)) * 0.98
    if (c.sneak) {
      strafe *= SIM.sneakSpeed
      forward *= SIM.sneakSpeed
    }

    let acceleration: number
    let inertia: number
    const under = this.sid(Math.floor(b.x), Math.floor(b.y - 1), Math.floor(b.z))
    if (b.onGround && under >= 0) {
      let speed = this.speedBase
      if (c.sprint) speed += speed * SIM.sprintSpeed
      const slip = this.world.slip[under]
      inertia = (slip > 0 ? slip : SIM.defaultSlipperiness) * 0.91
      acceleration = speed * (0.1627714 / (inertia * inertia * inertia))
      if (acceleration < 0) acceleration = 0
    } else {
      acceleration = SIM.airborneAcceleration
      inertia = SIM.airborneInertia
      if (c.sprint) acceleration += SIM.airborneAcceleration * 0.3
    }

    // The heading's right vector is (-hz, hx).
    let mag = Math.sqrt(strafe * strafe + forward * forward)
    if (mag >= 0.01) {
      mag = acceleration / Math.max(mag, 1)
      strafe *= mag
      forward *= mag
      b.vx -= strafe * c.hz - forward * c.hx
      b.vz += forward * c.hz + strafe * c.hx
    }

    if (this.onLadder(b)) {
      b.vx = Math.max(-SIM.ladderMaxSpeed, Math.min(b.vx, SIM.ladderMaxSpeed))
      b.vz = Math.max(-SIM.ladderMaxSpeed, Math.min(b.vz, SIM.ladderMaxSpeed))
      b.vy = Math.max(b.vy, c.sneak ? 0 : -SIM.ladderMaxSpeed)
    }

    this.moveEntity(b, c.sneak, b.vx, b.vy, b.vz)

    if (this.onLadder(b) && (b.collidedH || c.jump)) b.vy = SIM.ladderClimbSpeed

    b.vy -= SIM.gravity
    b.vy *= SIM.airdrag
    b.vx *= inertia
    b.vz *= inertia
  }
}

/** Slipperiness by block name; zero means the default 0.6. */
const SLIP_BY_NAME: Record<string, number> = { slime_block: 0.8, ice: 0.98, packed_ice: 0.98, frosted_ice: 0.98, blue_ice: 0.989 }

/** Slipperiness for a block type; zero means the default 0.6. */
export function simSlipOf (name: string): number {
  return SLIP_BY_NAME[name] ?? 0
}

/**
 * SIM_* flags for climbables, liquids, slime, fall cushions, beds, honey and hazards.
 * Waterlogged states count as liquid even when their block name does not.
 */
export function simKindOf (name: string, waterlogged: boolean): number {
  return (name === 'ladder' || name === 'vine' || name === 'scaffolding' || name.endsWith('_vines') || name.endsWith('_vines_plant') ? SIM_CLIMBABLE : 0) |
    (name === 'water' || name === 'lava' || name === 'bubble_column' || name === 'seagrass' ||
     name === 'tall_seagrass' || name === 'kelp' || name === 'kelp_plant' || waterlogged ? SIM_LIQUID : 0) |
    (name === 'slime_block' ? SIM_SLIME : 0) |
    (name === 'hay_block' || name === 'honey_block' ? SIM_CUSHION : 0) |
    (name === 'honey_block' ? SIM_HONEY : 0) |
    (name.endsWith('_bed') ? SIM_BED : 0) |
    (name === 'fire' || name === 'soul_fire' || name === 'cobweb' || name === 'web' ? SIM_HAZARD : 0)
}

/**
 * Build per-state shape, slipperiness and kind tables; the caller supplies stateAt.
 * `waterlogged(stateId)` marks states that prismarine-physics treats as water.
 */
export function simTables (
  lut: { shapeStarts: Int32Array, shapeCounts: Uint8Array, shapeData: Float32Array, maxStateId: number },
  blocksArray: ReadonlyArray<{ name: string, minStateId: number, maxStateId: number }>,
  waterlogged: (stateId: number) => boolean = () => false
): Omit<SimWorld, 'stateAt'> {
  const n = lut.maxStateId + 1
  const slip = new Float64Array(n)
  const kind = new Uint8Array(n)
  for (const b of blocksArray) {
    const s = simSlipOf(b.name)
    for (let id = b.minStateId; id <= b.maxStateId; id++) {
      slip[id] = s
      kind[id] = simKindOf(b.name, waterlogged(id))
    }
  }
  return {
    shapeStart: lut.shapeStarts,
    shapeCount: lut.shapeCounts,
    boxes: Float64Array.from(lut.shapeData),
    slip,
    kind
  }
}

/** Build per-state simulation tables from a LUT with slipperiness and kind data. */
export function simTablesFromLut (
  lut: { shapeStarts: Int32Array, shapeCounts: Uint8Array, shapeData: Float32Array, simSlip: Float64Array, simKind: Uint8Array }
): Omit<SimWorld, 'stateAt'> {
  return {
    shapeStart: lut.shapeStarts,
    shapeCount: lut.shapeCounts,
    boxes: Float64Array.from(lut.shapeData),
    slip: lut.simSlip,
    kind: lut.simKind
  }
}
