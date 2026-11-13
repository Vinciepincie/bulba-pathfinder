// Landing prediction for a falling body on the PlayerSim kernel, from the live state and the live world.
// It has its own world view: the executor's kernel grid only reaches 24 blocks up and down.
import { PlayerSim, newBody, copyBody, SIM_SLIME, SIM_CUSHION, SIM_BED } from '../playerSim.js'
import type { SimBody, SimControl, SimWorld } from '../playerSim.js'
import { landingPoints, healthLoss, healthAfter } from './model.js'
import type { FallBody } from './model.js'

/** What a landing touches. Water, cobweb, powder snow and climbables end a fall with no damage. */
export type LandingMedium = 'ground' | 'water' | 'cobweb' | 'powder_snow' | 'climbable' | 'lava' | 'fire' | 'void' | 'unknown'

/** Air control held through a rollout: none (coast), or a heading with forward held. */
export interface AirProgram {
  /** Mineflayer yaw to face, or null to coast with no keys. */
  yaw: number | null
  sprint: boolean
  /** Let go of forward from this tick on (Infinity: hold to the end). */
  releaseAt: number
}

export const COAST: AirProgram = Object.freeze({ yaw: null, sprint: false, releaseAt: Infinity })

export interface Landing {
  /** Ticks from now to the landing packet (0: this tick's position already lands). */
  tick: number
  x: number
  y: number
  z: number
  medium: LandingMedium
  /** The fall the server will have counted on landing, in blocks. */
  fall: number
  /** The landing tick's share of the fall (the old rounding rule). */
  last: number
  /** SIM_* flags of the block landed on. */
  kind: number
  /** The block whose top face the body lands on, under the feet centre when it can be. */
  support: { x: number, y: number, z: number } | null
  points: number
  /** Health the landing costs after protection. Lava and fire add their own penalty here. */
  loss: number
  healthAfter: number
  /** Body state per tick, index 0 = now; the clutch timing reads the positions sent before landing. */
  track: SimBody[]
  program: AirProgram
}

/** Tick limit for one rollout (a fall from the build limit to the floor takes about 150). */
const MAX_TICKS = 220
/** Health counted for landing in lava or fire on top of the fall itself. */
const LAVA_PENALTY = 20
const FIRE_PENALTY = 4

interface BlockInfo {
  name: string
  waterloggable: boolean
  full: boolean
  /** A pointed dripstone tip facing up: landing on it hurts double. */
  spikeUp?: boolean
}

/** The live world as the kernel reads it, with a per-tick memo and an overlay for imagined placements. */
export class FallWorld {
  private readonly memo = new Map<number, number>()
  /** Cells this world pretends hold another state (a placed hay bale), keyed like the memo. */
  readonly overlay = new Map<number, number>()
  /** A read hit an unloaded cell inside the world's height since the last reset. */
  unknown = false
  minY = -64
  maxY = 320

  constructor (private readonly read: (x: number, y: number, z: number) => number) {}

  /** Keys are relative to an anchor near the body, so they stay exact below 2^53. */
  private ax = 0
  private az = 0

  private key (x: number, y: number, z: number): number {
    // 16 bits each of x and z around the anchor, 13 of y: 45 bits, exact in a double.
    return (((x - this.ax + 32768) & 0xffff) * 65536 + ((z - this.az + 32768) & 0xffff)) * 8192 + ((y + 4096) & 0x1fff)
  }

  /** Forget the memo; the world may have changed since the last tick. Anchor keys at the body. */
  beginTick (x = this.ax, z = this.az): void {
    this.memo.clear()
    this.overlay.clear()
    this.ax = Math.floor(x)
    this.az = Math.floor(z)
  }

  stateAt (x: number, y: number, z: number): number {
    const k = this.key(x, y, z)
    const o = this.overlay.get(k)
    if (o !== undefined) return o
    let s = this.memo.get(k)
    if (s === undefined) {
      s = this.read(x, y, z)
      this.memo.set(k, s)
    }
    if (s < 0 && y >= this.minY && y < this.maxY) this.unknown = true
    return s
  }

  setOverlay (x: number, y: number, z: number, state: number): void { this.overlay.set(this.key(x, y, z), state) }
  clearOverlay (): void { this.overlay.clear() }
}

/** Rollouts of a falling body, one kernel per guard. */
export class FallPredictor {
  readonly world: FallWorld
  private readonly sim: PlayerSim
  private readonly control: SimControl = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false, hx: 0, hz: -1 }
  /** Kernel ticks spent since the last budget reset; callers cap work per game tick with it. */
  ticksSpent = 0

  constructor (
    tables: Omit<SimWorld, 'stateAt'>,
    read: (x: number, y: number, z: number) => number,
    private readonly info: (state: number) => BlockInfo | null,
    halfWidth = 0.3,
    height = 1.8
  ) {
    this.world = new FallWorld(read)
    const w = this.world
    this.sim = new PlayerSim({ ...tables, stateAt: (x, y, z) => w.stateAt(x, y, z) }, halfWidth, height)
  }

  setBody (halfWidth: number, height: number, speedBase: number): void {
    this.sim.halfWidth = halfWidth
    this.sim.height = height
    this.sim.speedBase = speedBase
  }

  /**
   * Fly the body with a program until it lands. `fall0` is the fall counted so far,
   * `whole` the 1.21.5+ rounding rule, `sneaking` whether the landing is sneaked.
   */
  predict (body0: SimBody, fall0: number, program: AirProgram, fallBody: FallBody, whole: boolean, sprintOk: boolean): Landing {
    const k = this.sim
    const c = this.control
    const b = copyBody(newBody(0, 0, 0), body0)
    b.inLiquid = false
    const track: SimBody[] = [copyBody(newBody(0, 0, 0), b)]
    this.world.unknown = false
    let fall = fall0
    let last = 0
    let tick = 0
    let medium: LandingMedium = 'void'
    // Already touching something that ends the fall.
    const now = this.contact(b)
    if (now !== null && !b.onGround) return this.finish(now, b, 0, fall, 0, track, program, fallBody, whole)
    for (tick = 1; tick <= MAX_TICKS; tick++) {
      this.setControl(c, program, tick - 1, sprintOk)
      const y0 = b.y
      k.step(b, c)
      this.ticksSpent++
      if (b.inLiquid) {
        // The kernel refused to move this tick: the contact started at the previous position.
        tick--
        medium = this.contact(b) ?? 'water'
        return this.finish(medium, b, tick, fall, last, track, program, fallBody, whole)
      }
      const drop = y0 - b.y
      if (drop > 0) { fall += drop; last = drop } else last = 0
      track.push(copyBody(newBody(0, 0, 0), b))
      if (this.world.unknown) return this.finish('unknown', b, tick, fall, last, track, program, fallBody, whole)
      const touched = this.contact(b)
      if (touched !== null) return this.finish(touched, b, tick, fall, last, track, program, fallBody, whole)
      if (b.onGround) return this.finish('ground', b, tick, fall, last, track, program, fallBody, whole)
      if (b.y < this.world.minY - 2) return this.finish('void', b, tick, fall, last, track, program, fallBody, whole)
    }
    return this.finish('unknown', b, MAX_TICKS, fall, last, track, program, fallBody, whole)
  }

  private setControl (c: SimControl, p: AirProgram, i: number, sprintOk: boolean): void {
    const held = p.yaw !== null && i < p.releaseAt
    c.forward = held
    c.sprint = held && p.sprint && sprintOk
    c.jump = false
    c.sneak = false
    c.left = false
    c.right = false
    c.back = false
    if (p.yaw !== null) {
      c.hx = -Math.sin(p.yaw)
      c.hz = -Math.cos(p.yaw)
    }
  }

  /** The medium the body touches that ends a fall, or null for open air or plain ground. */
  private contact (b: SimBody): LandingMedium | null {
    const k = this.sim
    if (k.onClimbable(b)) return 'climbable'
    const hw = k.halfWidth
    const x0 = Math.floor(b.x - hw + 0.001); const x1 = Math.floor(b.x + hw - 0.001)
    const z0 = Math.floor(b.z - hw + 0.001); const z1 = Math.floor(b.z + hw - 0.001)
    const y0 = Math.floor(b.y + 0.001); const y1 = Math.floor(b.y + k.height - 0.001)
    let found: LandingMedium | null = null
    for (let y = y0; y <= y1; y++) {
      for (let z = z0; z <= z1; z++) {
        for (let x = x0; x <= x1; x++) {
          const s = this.world.stateAt(x, y, z)
          if (s < 0) continue
          const inf = this.info(s)
          if (inf === null) continue
          const n = inf.name
          if (n === 'lava') return 'lava'
          if (n === 'fire' || n === 'soul_fire') found = found ?? 'fire'
          else if (n === 'cobweb') found = 'cobweb'
          else if (n === 'powder_snow') found = 'powder_snow'
          else if (n === 'water' || n === 'bubble_column' || n === 'seagrass' || n === 'tall_seagrass' || n === 'kelp' || n === 'kelp_plant') found = 'water'
        }
      }
    }
    return found
  }

  private finish (medium: LandingMedium, b: SimBody, tick: number, fall: number, last: number, track: SimBody[], program: AirProgram, fb: FallBody, whole: boolean): Landing {
    // Fire carries nothing and ends no fall: touched in mid-air, the kernel cannot fly on, so no verdict.
    if (medium === 'fire' && !b.onGround && this.supportUnder(b) === null) medium = 'unknown'
    let kind = 0
    let support: Landing['support'] = null
    let points = 0
    let loss = 0
    if (medium === 'ground' || medium === 'fire') {
      support = this.supportUnder(b)
      // The block that carries the body decides (hay, slime, a bed), not whatever is under the feet centre.
      const sState = support !== null ? this.world.stateAt(support.x, support.y, support.z) : -1
      kind = support !== null ? this.kindOf(sState) : this.sim.kindAt(b.x, b.y - 0.2, b.z)
      points = landingPoints(fall, last, kind, false, whole, fb)
      // An upward dripstone tip doubles the fall and adds 2.5 blocks (vanilla PointedDripstoneBlock.fallOn).
      if (this.blockInfo(sState)?.spikeUp === true && !fb.slowFalling) {
        const raw = whole ? Math.floor((fall + 2.5 + 1e-6 - fb.safeFall) * 2 * fb.multiplier) : Math.ceil((fall - last + 2.5 - fb.safeFall) * 2 * fb.multiplier)
        points = Math.max(points, raw)
      }
      loss = healthLoss(points, fb)
      if (medium === 'fire') loss += FIRE_PENALTY
    } else if (medium === 'lava') {
      loss = LAVA_PENALTY
    } else if (medium === 'void' || medium === 'unknown') {
      // Unknown is not death: the caller treats it as no verdict.
      loss = medium === 'void' ? Infinity : NaN
    }
    return {
      tick, x: b.x, y: b.y, z: b.z, medium, fall, last, kind, support, points, loss,
      healthAfter: Number.isFinite(loss) ? healthAfter(loss, fb) : (medium === 'void' ? -Infinity : NaN),
      track, program
    }
  }

  /** The block under the feet that carries the body, preferring the feet centre's column. */
  supportUnder (b: SimBody): Landing['support'] {
    const k = this.sim
    const yTop = b.y
    const cy = Math.floor(yTop - 0.001)
    // Columns the box overlaps by more than a hair: a box flush with a cell edge is not carried by it.
    const hw = k.halfWidth - 1e-3
    const cols: Array<[number, number]> = [[Math.floor(b.x), Math.floor(b.z)]]
    for (const dx of [-hw, hw]) {
      for (const dz of [-hw, hw]) {
        const c: [number, number] = [Math.floor(b.x + dx), Math.floor(b.z + dz)]
        if (!cols.some(o => o[0] === c[0] && o[1] === c[1])) cols.push(c)
      }
    }
    for (const [x, z] of cols) {
      const s = this.world.stateAt(x, cy, z)
      if (s < 0) continue
      const top = this.topOf(s)
      if (top !== null && Math.abs(cy + top - yTop) < 1e-3) return { x, y: cy, z }
    }
    return null
  }

  /** Highest collision top of a state within its cell, or null with no collision. */
  topOf (state: number): number | null {
    const w = this.sim.world
    const start = w.shapeStart[state]
    const n = w.shapeCount[state]
    if (start === undefined || start < 0 || n === 0) return null
    let top = -Infinity
    for (let i = 0; i < n; i++) top = Math.max(top, w.boxes[(start + i) * 6 + 4])
    return top
  }

  /** Is the state a full cube (the only supports a clutch targets)? */
  fullCube (state: number): boolean {
    const w = this.sim.world
    const start = w.shapeStart[state]
    if (start === undefined || start < 0 || w.shapeCount[state] !== 1) return false
    const o = start * 6
    return w.boxes[o] === 0 && w.boxes[o + 1] === 0 && w.boxes[o + 2] === 0 && w.boxes[o + 3] === 1 && w.boxes[o + 4] === 1 && w.boxes[o + 5] === 1
  }

  /** SIM flags of a state, for scoring landings on slime, hay or beds. */
  kindOf (state: number): number {
    return state < 0 ? 0 : this.sim.world.kind[state]
  }

  blockInfo (state: number): BlockInfo | null { return state < 0 ? null : this.info(state) }
}

export { SIM_SLIME, SIM_CUSHION, SIM_BED }
