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
  /** Scratch: collected boxes, 6 per box, absolute coordinates. */
  private buf = new Float64Array(6 * 512)
  private nBoxes = 0
  private readonly grid: SimGrid | null

  constructor (world: SimWorld, halfWidth = 0.3, height = 1.8, speedBase: number = SIM.playerSpeed) {
    this.world = world
    this.grid = world.grid ?? null
    this.halfWidth = halfWidth
    this.height = height
    this.speedBase = speedBase
  }

  /** State id at a cell: the grid inline when there is one, else the world's stateAt. */
  private sid (x: number, y: number, z: number): number {
    const g = this.grid
    if (g === null) return this.world.stateAt(x, y, z)
    const lx = x - g.x0
    const ly = y - g.y0
    const lz = z - g.z0
    if (lx < 0 || lx >= g.w || ly < 0 || ly >= g.h || lz < 0 || lz >= g.l) return -1
    return g.states[(ly * g.l + lz) * g.w + lx]
  }

  /** Collect every box of every cell prismarine's getSurroundingBBs would visit for this query. */
  private gather (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): number {
    const w = this.world
    let n = 0
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
    this.nBoxes = n
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

  private kindAt (x: number, y: number, z: number): number {
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
    for (let y = Math.floor(minY); y <= Math.floor(maxY); y++) {
      for (let z = Math.floor(minZ); z <= Math.floor(maxZ); z++) {
        for (let x = Math.floor(minX); x <= Math.floor(maxX); x++) {
          const s = this.sid(x, y, z)
          if (s >= 0 && (this.world.kind[s] & SIM_LIQUID) !== 0) return true
        }
      }
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
        b.vy = SIM.jumpVelocity
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
 * (water, lava and prismarine's waterLike set, or any waterlogged state), slime.
 */
export function simKindOf (name: string, waterlogged: boolean): number {
  return (name === 'ladder' || name === 'vine' || name === 'scaffolding' ? SIM_CLIMBABLE : 0) |
    (name === 'water' || name === 'lava' || name === 'bubble_column' || name === 'seagrass' ||
     name === 'tall_seagrass' || name === 'kelp' || name === 'kelp_plant' || waterlogged ? SIM_LIQUID : 0) |
    (name === 'slime_block' ? SIM_SLIME : 0)
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
