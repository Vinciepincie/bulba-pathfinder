// Path node — API-identical to mineflayer-pathfinder/lib/move.js. Consumers
// receive these in path_update results and rely on the Vec3 inheritance.
import { Vec3 } from 'vec3'
import type { RawPathNode } from './types.js'

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
  /**
   * Two-stage moves (extended parkour). Slime bounce: the slime STAND cell to
   * drop onto first; the rebound then carries the body up to this node.
   * Momentum chain (`chain`): the executor expands the node into the
   * stepping-stone landing followed by this node, and re-jumps on the
   * landing tick. Null / false on every other move. Additive over upstream.
   */
  via: Vec3 | null
  chain: boolean
  /**
   * Aim offset (XZ, blocks) applied after the aim point is set on the
   * support's top: a momentum-chain stepping stone is landed on its FAR side
   * so the re-jump starts with the creep credit the planner assumed.
   */
  aimDx = 0
  aimDz = 0

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
    return new Move(raw.x, raw.y, raw.z, 0, raw.cost, toBreak, toPlace, raw.parkour, via, raw.chain === true)
  }

  /**
   * Path form of a raw node: a momentum-chain node becomes its stepping
   * stone (a plain parkour landing) followed by the node itself, so the
   * follower lands the first hop like any other and the chain flag on the
   * second says "press jump on that landing tick".
   */
  static expandRaw (raw: RawPathNode): Move[] {
    const node = Move.fromRaw(raw)
    if (!node.chain || node.via === null) return [node]
    const stone = new Move(node.via.x, node.via.y, node.via.z, 0, 0, [], [], true)
    Move.aimStone(stone, node)
    return [stone, node]
  }

  /**
   * Whole-path form of expandRaw. With momentum in the search
   * (allowParkourMomentum) a chain node's stone IS the node before it, so
   * that one is aimed far-side rather than duplicated; a compound chain
   * (stone folded into `via` only) still expands into stone + node.
   */
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

  /** Unit direction of the re-jump off `stone` toward `node`; the executor
   * scales it by the stone's creep credit once it knows the support's shape. */
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
