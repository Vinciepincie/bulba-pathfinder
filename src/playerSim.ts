// Allocation-free player physics: the normal-movement branch of
// prismarine-physics (index.js simulatePlayer / moveEntityWithHeading /
// moveEntity), op for op, over a flat state-id world with the LUT's shape
// table. Same engine the executor's rollouts use, at a fraction of the cost —
// no Vec3 or AABB per tick, no block objects — so a planner can afford to
// fly candidate jumps instead of approximating them.
//
// The one deliberate difference: the look direction is a unit HEADING vector
// (hx, hz) = (-sin yaw, -cos yaw), not an angle. prismarine derives the same
// vector from the yaw with Math.sin/Math.cos, so the two agree to rounding;
// taking the vector keeps every operation here +,-,*,/ and sqrt, which a Rust
// port reproduces bit for bit.
//
// Out of scope (the caller is told, never silently wrong): water and lava
// (`inLiquid` is raised and the step returns), cobwebs, bubble columns,
// elytra, levitation, slow falling. On 1.21 prismarine applies no soul-sand
// or honey slowdown and no ladder-trapdoor climbing, and neither does this.

/**
 * A flat state-id grid (a snapshot's): when a SimWorld carries one, the
 * kernel indexes it inline instead of calling stateAt per cell — the lookups
 * are most of a tick's cost. Cells outside it read as -1, as stateAt must.
 */
export interface SimGrid {
  states: Uint16Array | Int32Array
  x0: number
  y0: number
  z0: number
  w: number
  h: number
  l: number
  /**
   * A LAZY grid (a window on a live world): a cell is current while its
   * stamp equals `serial`; any other is read through stateAt and stored, and
   * cells outside the window are read through stateAt too. Bumping `serial`
   * forgets the whole window at no cost.
   */
  stamps?: Int32Array
  serial?: number
}

/** Blocks of the world the simulation reads. */
export interface SimWorld {
  /** State id at a cell, or -1 outside the loaded world (no collision, no support). */
  stateAt: (x: number, y: number, z: number) => number
  /** Optional: the same world as a flat grid (see SimGrid); stateAt must agree with it. */
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
/** Breaks a fall: keeps a fifth of its damage (hay, honey), or halves its height (a bed). See fallDamage.ts. */
export const SIM_CUSHION = 8
export const SIM_BED = 16

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
  /** Set when the body touches water or lava: this model stops there. */
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

/** prismarine-physics constants (index.js `physics`), as that engine holds them. */
/**
 * The jump rule of the prismarine-physics the bot runs (PhysicsSim.probeVanillaJump):
 * vanilla 1.21's jumpFromGround keeps the faster of the jump and the body's own
 * rise (max(jumpPower, vy): a jump held on a slime rebound keeps the rebound,
 * scripts/patchPhysics.mjs), stock prismarine sets the jump's. The kernel
 * follows whichever the bot's own physics does: one flag per thread.
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

/** A cell-range key's serial when it holds no range (PlayerSim.slotKey): a grid's serial is never negative. */
const NO_RANGE = -1
/** PlayerSim.bricks: a brick's side, as a shift (4 cells), and its bits. */
const BRICK = 2
const BRICK_MASK = (1 << BRICK) - 1
const BRICK_SEEN = 1
/** A cell of the brick has a collision box. */
const BRICK_SOLID = 2
/** A cell of the brick is a liquid. */
const BRICK_WET = 4

/**
 * The simulator: world + body dimensions + scratch space. Not re-entrant
 * (one scratch buffer); one instance per thread of use.
 */
export class PlayerSim {
  readonly world: SimWorld
  /** Half width / height of the body (the executor nudges both off the stock 0.3 / 1.8). */
  halfWidth: number
  height: number
  /** Movement-speed attribute base (0.1 unless the server says otherwise). */
  speedBase: number
  /** Scratch: collected boxes, 6 per box, absolute coordinates (the slot gather() last filled). */
  private buf: Float64Array = new Float64Array(6 * 512)
  private nBoxes = 0
  /**
   * The last two gathers, by cell range: a range asked again (the next tick's
   * move; a move's after the step-up query between two) takes its boxes as
   * they are. Only over a grid: a stamped grid's ranges inside its window,
   * keyed by its serial; an unstamped grid is fixed for the sim's life (a
   * solve's snapshot, which the oracle's stand caches assume too).
   */
  private readonly slotBuf: Float64Array[] = [this.buf, new Float64Array(6 * 512)]
  private readonly slotN = [0, 0]
  /** Per slot: x0, y0, z0, x1, y1, z1, serial; serial NO_RANGE: holds no range. */
  private readonly slotKey = new Float64Array([0, 0, 0, 0, 0, 0, NO_RANGE, 0, 0, 0, 0, 0, 0, NO_RANGE])
  private slot = 0
  /** touchesLiquid's last answer, by cell range (keyed as a slot). */
  private readonly liqKey = new Float64Array([0, 0, 0, 0, 0, 0, NO_RANGE])
  private liqLast = false
  /**
   * An unstamped grid by brick of 4³ cells: BRICK_SEEN once looked at, with
   * BRICK_SOLID when a cell has a collision box and BRICK_WET when one is a
   * liquid. A gather skips the layers of empty bricks its range crosses (a
   * fall's range is 4-6 cells tall, mostly air), and a body over dry bricks
   * touches no liquid without a cell read (touchesLiquid ran every tick; most
   * courses have none). Built as asked; null for other worlds.
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

  /** State id at a cell, as the simulation reads it (through the grid when there is one). */
  stateAt (x: number, y: number, z: number): number {
    return this.sid(x, y, z)
  }

  /** State id at a cell: the grid inline when there is one, else the world's stateAt. */
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
   * The key a cell range is kept under (PlayerSim.slotKey): the grid's
   * serial, 0 for an unstamped grid; NO_RANGE when the range cannot be kept
   * (no grid, or cells of a stamped grid's range outside its window, which
   * are read through every time).
   */
  private rangeSerial (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): number {
    const g = this.grid
    if (g === null) return NO_RANGE
    if (g.stamps === undefined) return 0
    if (x0 < g.x0 || y0 < g.y0 || z0 < g.z0 || x1 >= g.x0 + g.w || y1 >= g.y0 + g.h || z1 >= g.z0 + g.l) return NO_RANGE
    return g.serial as number
  }

  /** Collect every box of every cell prismarine's getSurroundingBBs would visit for this query. */
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
    // (into the other slot: the one just used may be asked again next)
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

  /** gather()'s cells, in its order, into this.buf: the box count. */
  private collect (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): number {
    const w = this.world
    const g = this.grid
    let n = 0
    // (a range inside the grid: its cells indexed straight, no bounds test a cell)
    const inside = g !== null && x0 >= g.x0 && y0 >= g.y0 && z0 >= g.z0 && x1 < g.x0 + g.w && y1 < g.y0 + g.h && z1 < g.z0 + g.l
    const states = g !== null ? g.states : null
    const stamps = g !== null ? g.stamps : undefined
    const serial = g !== null ? g.serial : undefined
    // (an unstamped grid: whole layers of empty bricks skipped, PlayerSim.bricks)
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

  /** Would getSurroundingBBs return anything at all for this query? (the sneak edge guard's test) */
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

  // computeOffset{X,Y,Z}: aabb.js, `this` = the block box, `other` = the player box.
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

  /** The SIM_* bits of the block at a point. */
  kindAt (x: number, y: number, z: number): number {
    const s = this.sid(Math.floor(x), Math.floor(y), Math.floor(z))
    return s < 0 ? 0 : this.world.kind[s]
  }

  /** index.js isOnLadder (1.21: ladders and vines; no trapdoor rule). */
  private onLadder (b: SimBody): boolean {
    return (this.kindAt(b.x, b.y, b.z) & SIM_CLIMBABLE) !== 0
  }

  /** Is the body on a climbable (index.js isOnLadder: the cell its feet are in)? */
  onClimbable (b: SimBody): boolean {
    return this.onLadder(b)
  }

  /** index.js getWaterInBB/isMaterialInBB, reduced to "does the body touch a liquid cell". */
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
    // (the cells of the last tick's question: its answer, kept as a gather's boxes are)
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

  /** Do the bricks over grid cells [lx0..lx1]×[ly0..ly1]×[lz0..lz1] of an unstamped grid all lack `bit` (PlayerSim.bricks)? */
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

  /** A brick's bits (PlayerSim.bricks), its cells read the first time it is asked. */
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
        }
      }
    }
    bricks[bi] = v
    return v
  }

  /**
   * Does the body's box with its feet at (x, y, z), its sides grown by `grow`
   * (negative: shrunk) and its top and bottom trimmed by `trim`, overlap a
   * collision box? The executor's probes (geometry.playerCollides, nearWall)
   * for a kernel that steers as it does.
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
   * Many probes round one place: gather once the boxes any of them could
   * touch — a body's box within `reach` of (x, y, z) sideways, from `below`
   * under its feet up — and ask probeHit of each. Valid until the next step
   * or probe of another kind (they share the scratch buffer).
   */
  probeBegin (x: number, y: number, z: number, reach: number, below: number): number {
    const r = this.halfWidth + reach
    return this.gather(x - r, y - below, z - r, x + r, y + this.height, z + r)
  }

  /**
   * The collision boxes in a region: their count, six numbers a box in
   * `scratchBoxes` (minX, minY, minZ, maxX, maxY, maxZ). Valid until the
   * next step or probe (one scratch buffer).
   */
  boxesIn (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number {
    return this.gather(minX, minY, minZ, maxX, maxY, maxZ)
  }

  /** The scratch buffer boxesIn() and the probes fill. */
  get scratchBoxes (): Float64Array {
    return this.buf
  }

  /** boxHits, against the boxes probeBegin gathered. */
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

  /** Does the body's box overlap any collision box (an embedded position no move can produce)? */
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
   * Is the body carried: a collision box under its hitbox, its top at the
   * feet? A body can be onGround without. The tick it lands it moves down
   * before it moves on, so it touches down on the edge it is leaving and ends
   * the tick past it, with the ground flag and nothing underneath: it can
   * still jump on the next tick, and falls on the one after if it does not.
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
   * Is the body carried by a box of the block column (cx, cz): a box whose top
   * is at the feet and whose footprint is under the hitbox? Tells a body
   * standing ON a column from one overhanging it from the column beside.
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

  /** index.js moveEntity, normal branch (no web). */
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

    // player box
    let pMinX = b.x - hw; let pMinY = b.y; let pMinZ = b.z - hw
    let pMaxX = b.x + hw; let pMaxY = b.y + h; let pMaxZ = b.z + hw
    // query = player box extended by the move
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

    // step up
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

      // BB1 = old box, BB2 = old box; BB_XZ = old box extended by (dx, 0, dz)
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

    // setPositionToBB
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

  /** One tick of index.js simulatePlayer for the normal-movement branch. */
  step (b: SimBody, c: SimControl): void {
    if (this.touchesLiquid(b)) { b.inLiquid = true; return }

    if (Math.abs(b.vx) < SIM.negligibleVelocity) b.vx = 0
    if (Math.abs(b.vy) < SIM.negligibleVelocity) b.vy = 0
    if (Math.abs(b.vz) < SIM.negligibleVelocity) b.vz = 0

    if (c.jump) {
      if (b.jumpTicks > 0) b.jumpTicks--
      if (b.onGround && b.jumpTicks === 0) {
        b.vy = vanillaJump && b.vy > SIM.jumpVelocity ? b.vy : SIM.jumpVelocity
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

    // moveEntityWithHeading, normal movement
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

    // applyHeading: vel += strafe * right + forward * look, right = (-hz, hx)
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

/** prismarine-physics' slipperiness by block name (index.js blockSlipperiness); 0 = the default 0.6. */
const SLIP_BY_NAME: Record<string, number> = { slime_block: 0.8, ice: 0.98, packed_ice: 0.98, frosted_ice: 0.98, blue_ice: 0.989 }

/** Slipperiness of a block type for SimWorld.slip (0 = the default). */
export function simSlipOf (name: string): number {
  return SLIP_BY_NAME[name] ?? 0
}

/**
 * SIM_* kind bits of a block state: climbable (index.js isOnLadder), liquid
 * (water, lava and prismarine's waterLike set, or any waterlogged state),
 * slime, and what breaks a fall.
 */
export function simKindOf (name: string, waterlogged: boolean): number {
  return (name === 'ladder' || name === 'vine' || name === 'scaffolding' || name.endsWith('_vines') || name.endsWith('_vines_plant') ? SIM_CLIMBABLE : 0) |
    (name === 'water' || name === 'lava' || name === 'bubble_column' || name === 'seagrass' ||
     name === 'tall_seagrass' || name === 'kelp' || name === 'kelp_plant' || waterlogged ? SIM_LIQUID : 0) |
    (name === 'slime_block' ? SIM_SLIME : 0) |
    (name === 'hay_block' || name === 'honey_block' ? SIM_CUSHION : 0) |
    (name.endsWith('_bed') ? SIM_BED : 0)
}

/**
 * A SimWorld's per-state tables from a LUT's shape table and a registry
 * (slipperiness and kinds by block name). `stateAt` is the caller's.
 * `waterlogged(stateId)` marks the states prismarine counts as water
 * (index.js getWaterInBB reads `block.isWaterlogged`).
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

/** A SimWorld's per-state tables straight from a LUT that carries them (lut.ts simSlip / simKind). */
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
