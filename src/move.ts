// Path results retain Vec3 inheritance for consumer compatibility.
import { Vec3 } from 'vec3'
import type { RawPathNode } from './types.js'
import type { HopProgram } from './hopOracle.js'
import { AIM_VARIANTS } from './parkourTable.js'

export interface ToPlaceEntry {
  x: number
  y: number
  z: number
  dx: number
  dy: number
  dz: number
  useOne?: boolean
  jump?: boolean
  returnPos?: Vec3
}

export class Move extends Vec3 {
  remainingBlocks: number
  cost: number
  toBreak: Vec3[]
  toPlace: ToPlaceEntry[]
  parkour: boolean
  hash: string
  /** Intermediate stand cell for a slime bounce or momentum chain.
   * A chain expands to two landings and re-jumps on the intermediate landing tick. */
  via: Vec3 | null
  chain: boolean
  /** A jump beyond standing reach; the executor builds sprint speed from the support's rear. */
  run = false
  /** A drop past the fall budget, landed by placing water just before touchdown (fall recovery). */
  clutch = false
  /** Whether the landing support is narrow; null until the executor checks its shape. */
  narrowLanding: boolean | null = null
  /** Signed takeoff/landing offsets in blocks along the flight's perpendicular (-dz, dx).
   * postProcessPath applies `qOff` to the landing and stores `pOff` beside the previous aim. */
  pOff = 0
  qOff = 0
  /** A curved jump around a pillar, flown with a searched PhysicsSim.solveJump script. */
  wOff = 0
  takeoff: { x: number, z: number } | null = null
  /** XZ aim offset in blocks; lands far across a chain's stepping stone to preserve re-jump reach. */
  aimDx = 0
  aimDz = 0
  /** Physics-verified controls for landing this node, or null. */
  program: HopProgram | null = null
  /** Original planner cell; postProcessPath changes x/y/z to the support aim point. */
  readonly cell: readonly [number, number, number]

  constructor (
    x: number,
    y: number,
    z: number,
    remainingBlocks: number,
    cost: number,
    toBreak: Vec3[] = [],
    toPlace: ToPlaceEntry[] = [],
    parkour = false,
    via: Vec3 | null = null,
    chain = false
  ) {
    super(Math.floor(x), Math.floor(y), Math.floor(z))
    this.cell = [this.x, this.y, this.z]
    this.remainingBlocks = remainingBlocks
    this.cost = cost
    this.toBreak = toBreak
    this.toPlace = toPlace
    this.parkour = parkour
    this.via = via
    this.chain = chain
    this.hash = this.x + ',' + this.y + ',' + this.z
  }

  static fromRaw (raw: RawPathNode): Move {
    const toPlace: ToPlaceEntry[] = raw.useOne
      ? [{ x: raw.useOne.x, y: raw.useOne.y, z: raw.useOne.z, dx: 0, dy: 0, dz: 0, useOne: true }]
      : []
    const toBreak: Vec3[] = raw.toBreak ? raw.toBreak.map(b => new Vec3(b.x, b.y, b.z)) : []
    const via = raw.via ? new Vec3(raw.via.x, raw.via.y, raw.via.z) : null
    const move = new Move(raw.x, raw.y, raw.z, 0, raw.cost, toBreak, toPlace, raw.parkour, via, raw.chain === true)
    move.run = raw.run === true
    if (raw.clutch === true) move.clutch = true
    if (raw.program !== undefined) move.program = raw.program
    if (raw.aim !== undefined && raw.aim !== 0) {
      const v = AIM_VARIANTS[(raw.aim & 0x7f) - 1]
      if (v !== undefined) {
        const side = (raw.aim & 0x80) !== 0 ? -1 : 1
        move.pOff = v[0] * side
        move.qOff = v[1] * side
        move.wOff = v[2] * side
      }
    }
    return move
  }

  /** Expand a compound chain into its stepping stone and destination; re-jump on landing. */
  static expandRaw (raw: RawPathNode): Move[] {
    const node = Move.fromRaw(raw)
    if (!node.chain || node.via === null) return [node]
    const stone = new Move(node.via.x, node.via.y, node.via.z, 0, 0, [], [], true)
    Move.aimStone(stone, node)
    return [stone, node]
  }

  /** Expand raw nodes, aiming an existing momentum stepping stone far-side without duplicating it.
   * A stepping stone stored only in `via` is inserted as a separate landing. */
  static expandRawPath (raws: RawPathNode[]): Move[] {
    const out: Move[] = []
    for (const raw of raws) {
      const node = Move.fromRaw(raw)
      if (node.chain && node.via !== null) {
        const prev = out.length > 0 ? out[out.length - 1] : null
        if (prev !== null && prev.x === node.via.x && prev.y === node.via.y && prev.z === node.via.z) {
          Move.aimStone(prev, node)
        } else {
          const stone = new Move(node.via.x, node.via.y, node.via.z, 0, 0, [], [], true)
          Move.aimStone(stone, node)
          out.push(stone)
        }
      }
      out.push(node)
    }
    return out
  }

  /** Re-jump unit direction; the executor scales it by the stepping stone's shape-dependent credit. */
  private static aimStone (stone: Move, node: Move): void {
    const dx = node.x - stone.x
    const dz = node.z - stone.z
    const len = Math.hypot(dx, dz)
    if (len > 0) {
      stone.aimDx = dx / len
      stone.aimDz = dz / len
    }
  }
}
