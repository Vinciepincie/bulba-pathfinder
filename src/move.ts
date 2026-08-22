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

  constructor (
    x: number,
    y: number,
    z: number,
    remainingBlocks: number,
    cost: number,
    toBreak: Vec3[] = [],
    toPlace: ToPlaceEntry[] = [],
    parkour = false
  ) {
    super(Math.floor(x), Math.floor(y), Math.floor(z))
    this.remainingBlocks = remainingBlocks
    this.cost = cost
    this.toBreak = toBreak
    this.toPlace = toPlace
    this.parkour = parkour
    this.hash = this.x + ',' + this.y + ',' + this.z
  }

  static fromRaw (raw: RawPathNode): Move {
    const toPlace: ToPlaceEntry[] = raw.useOne
      ? [{ x: raw.useOne.x, y: raw.useOne.y, z: raw.useOne.z, dx: 0, dy: 0, dz: 0, useOne: true }]
      : []
    const toBreak: Vec3[] = raw.toBreak ? raw.toBreak.map(b => new Vec3(b.x, b.y, b.z)) : []
    return new Move(raw.x, raw.y, raw.z, 0, raw.cost, toBreak, toPlace, raw.parkour)
  }
}
