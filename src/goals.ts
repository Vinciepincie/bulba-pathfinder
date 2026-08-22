// Faithful port of mineflayer-pathfinder/lib/goals.js (MIT). Same class
// names, same constructor signatures, same heuristic/isEnd math — external
// callers must not be able to tell the difference. Formula changes are bugs.
import { Vec3 } from 'vec3'
import type { XYZ } from './types.js'

/** The subset of a world the goals need: prismarine-world compatible raycast. */
export interface RaycastWorld {
  raycast (from: Vec3, direction: Vec3, range: number, matcher?: unknown): { position: Vec3, face: number } | null
  getBlock? (pos: Vec3): { shapes: number[][] } | null
}

function distanceXZ (dx: number, dz: number): number {
  dx = Math.abs(dx)
  dz = Math.abs(dz)
  return Math.abs(dx - dz) + Math.min(dx, dz) * Math.SQRT2
}

export class Goal {
  heuristic (_node: XYZ): number {
    return 0
  }

  isEnd (_node: XYZ): boolean {
    return true
  }

  hasChanged (): boolean {
    return false
  }

  isValid (): boolean {
    return true
  }
}

/** One specific block that the player should stand inside at foot level. */
export class GoalBlock extends Goal {
  x: number
  y: number
  z: number

  constructor (x: number, y: number, z: number) {
    super()
    this.x = Math.floor(x)
    this.y = Math.floor(y)
    this.z = Math.floor(z)
  }

  heuristic (node: XYZ): number {
    const dx = this.x - node.x
    const dy = this.y - node.y
    const dz = this.z - node.z
    return distanceXZ(dx, dz) + Math.abs(dy)
  }

  isEnd (node: XYZ): boolean {
    return node.x === this.x && node.y === this.y && node.z === this.z
  }
}

/** A block position that the player should get within a certain radius of. */
export class GoalNear extends Goal {
  x: number
  y: number
  z: number
  rangeSq: number

  constructor (x: number, y: number, z: number, range: number) {
    super()
    this.x = Math.floor(x)
    this.y = Math.floor(y)
    this.z = Math.floor(z)
    this.rangeSq = range * range
  }

  heuristic (node: XYZ): number {
    const dx = this.x - node.x
    const dy = this.y - node.y
    const dz = this.z - node.z
    return distanceXZ(dx, dz) + Math.abs(dy)
  }

  isEnd (node: XYZ): boolean {
    const dx = this.x - node.x
    const dy = this.y - node.y
    const dz = this.z - node.z
    return (dx * dx + dy * dy + dz * dz) <= this.rangeSq
  }
}

/** Long-range goal without a specific Y level. */
export class GoalXZ extends Goal {
  x: number
  z: number

  constructor (x: number, z: number) {
    super()
    this.x = Math.floor(x)
    this.z = Math.floor(z)
  }

  heuristic (node: XYZ): number {
    return distanceXZ(this.x - node.x, this.z - node.z)
  }

  isEnd (node: XYZ): boolean {
    return node.x === this.x && node.z === this.z
  }
}

export class GoalNearXZ extends Goal {
  x: number
  z: number
  rangeSq: number

  constructor (x: number, z: number, range: number) {
    super()
    this.x = Math.floor(x)
    this.z = Math.floor(z)
    this.rangeSq = range * range
  }

  heuristic (node: XYZ): number {
    return distanceXZ(this.x - node.x, this.z - node.z)
  }

  isEnd (node: XYZ): boolean {
    const dx = this.x - node.x
    const dz = this.z - node.z
    return (dx * dx + dz * dz) <= this.rangeSq
  }
}

export class GoalY extends Goal {
  y: number

  constructor (y: number) {
    super()
    this.y = Math.floor(y)
  }

  heuristic (node: XYZ): number {
    return Math.abs(this.y - node.y)
  }

  isEnd (node: XYZ): boolean {
    return node.y === this.y
  }
}

/** Don't get into the block, but get directly adjacent to it. Useful for chests. */
export class GoalGetToBlock extends Goal {
  x: number
  y: number
  z: number

  constructor (x: number, y: number, z: number) {
    super()
    this.x = Math.floor(x)
    this.y = Math.floor(y)
    this.z = Math.floor(z)
  }

  heuristic (node: XYZ): number {
    const dx = node.x - this.x
    const dy = node.y - this.y
    const dz = node.z - this.z
    return distanceXZ(dx, dz) + Math.abs(dy < 0 ? dy + 1 : dy)
  }

  isEnd (node: XYZ): boolean {
    const dx = node.x - this.x
    const dy = node.y - this.y
    const dz = node.z - this.z
    return Math.abs(dx) + Math.abs(dy < 0 ? dy + 1 : dy) + Math.abs(dz) === 1
  }
}

export interface GoalLookAtBlockOptions {
  reach?: number
  entityHeight?: number
}

/** Path to a position from which a face of the block at `pos` is visible. */
export class GoalLookAtBlock extends Goal {
  pos: Vec3
  world: RaycastWorld
  reach: number
  entityHeight: number

  constructor (pos: Vec3, world: RaycastWorld, options: GoalLookAtBlockOptions = {}) {
    super()
    this.pos = pos
    this.world = world
    this.reach = options.reach || 4.5 // default survival: 4.5 creative: 5
    this.entityHeight = options.entityHeight || 1.6
  }

  heuristic (node: XYZ): number {
    const dx = node.x - this.pos.x
    const dy = node.y - this.pos.y
    const dz = node.z - this.pos.z
    return distanceXZ(dx, dz) + Math.abs(dy < 0 ? dy + 1 : dy)
  }

  isEnd (node: XYZ): boolean {
    // Upstream calls node.distanceTo (nodes are Vec3 subclasses); accept
    // plain XYZ nodes too.
    const eye = this.pos.offset(0, this.entityHeight, 0)
    const ex = node.x - eye.x
    const ey = node.y - eye.y
    const ez = node.z - eye.z
    if (Math.sqrt(ex * ex + ey * ey + ez * ez) > this.reach) return false
    const dx = node.x - (this.pos.x + 0.5)
    const dy = node.y + this.entityHeight - (this.pos.y + 0.5)
    const dz = node.z - (this.pos.z + 0.5)
    const visibleFaces = {
      y: Math.sign(Math.abs(dy) > 0.5 ? dy : 0),
      x: Math.sign(Math.abs(dx) > 0.5 ? dx : 0),
      z: Math.sign(Math.abs(dz) > 0.5 ? dz : 0)
    }
    for (const i of ['y', 'x', 'z'] as const) {
      if (!visibleFaces[i]) continue
      const targetPos = new Vec3(this.pos.x, this.pos.y, this.pos.z).offset(
        0.5 + (i === 'x' ? visibleFaces[i] * 0.5 : 0),
        0.5 + (i === 'y' ? visibleFaces[i] * 0.5 : 0),
        0.5 + (i === 'z' ? visibleFaces[i] * 0.5 : 0)
      )
      const startPos = new Vec3(node.x + 0.5, node.y + this.entityHeight, node.z + 0.5)
      const rayPos = this.world.raycast(startPos, targetPos.clone().subtract(startPos).normalize(), this.reach)?.position
      if (rayPos && rayPos.x === this.pos.x && rayPos.y === this.pos.y && rayPos.z === this.pos.z) {
        return true
      }
    }
    return false
  }
}

/**
 * Path to a position from which a face of the block is visible. You'll
 * manually need to break the block — THIS WON'T BREAK IT. (Ported verbatim,
 * including upstream's constructor signature.)
 */
export class GoalBreakBlock extends Goal {
  goal: GoalLookAtBlock

  constructor (x: number, y: number, z: number, bot: { world: RaycastWorld }, options: GoalLookAtBlockOptions = {}) {
    super()
    this.goal = new GoalLookAtBlock(new Vec3(x, y, z), bot.world, options)
  }

  heuristic (node: XYZ): number {
    return this.goal.heuristic(node)
  }

  isEnd (node: XYZ): boolean {
    return this.goal.isEnd(node)
  }
}

/** A composite of many goals, any one of which satisfies the composite. */
export class GoalCompositeAny<G extends Goal = Goal> extends Goal {
  goals: G[]

  constructor (goals: G[] = []) {
    super()
    this.goals = goals
  }

  push (goal: G): void {
    this.goals.push(goal)
  }

  heuristic (node: XYZ): number {
    let min = Number.MAX_VALUE
    for (const g of this.goals) min = Math.min(min, g.heuristic(node))
    return min
  }

  isEnd (node: XYZ): boolean {
    for (const g of this.goals) if (g.isEnd(node)) return true
    return false
  }

  hasChanged (): boolean {
    for (const g of this.goals) if (g.hasChanged()) return true
    return false
  }

  isValid (): boolean {
    return this.goals.reduce((pre, curr) => pre && curr.isValid(), true)
  }
}

/** A composite of many goals, all of which need to be satisfied. */
export class GoalCompositeAll<G extends Goal = Goal> extends Goal {
  goals: G[]

  constructor (goals: G[] = []) {
    super()
    this.goals = goals
  }

  push (goal: G): void {
    this.goals.push(goal)
  }

  heuristic (node: XYZ): number {
    let max = Number.MIN_VALUE
    for (const g of this.goals) max = Math.max(max, g.heuristic(node))
    return max
  }

  isEnd (node: XYZ): boolean {
    for (const g of this.goals) if (!g.isEnd(node)) return false
    return true
  }

  hasChanged (): boolean {
    for (const g of this.goals) if (g.hasChanged()) return true
    return false
  }

  isValid (): boolean {
    return this.goals.reduce((pre, curr) => pre && curr.isValid(), true)
  }
}

export class GoalInvert extends Goal {
  goal: Goal

  constructor (goal: Goal) {
    super()
    this.goal = goal
  }

  heuristic (node: XYZ): number {
    return -this.goal.heuristic(node)
  }

  isEnd (node: XYZ): boolean {
    return !this.goal.isEnd(node)
  }

  hasChanged (): boolean {
    return this.goal.hasChanged()
  }

  isValid (): boolean {
    return this.goal.isValid()
  }
}

export interface FollowEntity {
  position: Vec3
  isValid?: boolean
}

export class GoalFollow extends Goal {
  entity: FollowEntity
  x: number
  y: number
  z: number
  rangeSq: number

  constructor (entity: FollowEntity, range: number) {
    super()
    this.entity = entity
    this.x = Math.floor(entity.position.x)
    this.y = Math.floor(entity.position.y)
    this.z = Math.floor(entity.position.z)
    this.rangeSq = range * range
  }

  heuristic (node: XYZ): number {
    const dx = this.x - node.x
    const dy = this.y - node.y
    const dz = this.z - node.z
    return distanceXZ(dx, dz) + Math.abs(dy)
  }

  isEnd (node: XYZ): boolean {
    const dx = this.x - node.x
    const dy = this.y - node.y
    const dz = this.z - node.z
    return (dx * dx + dy * dy + dz * dz) <= this.rangeSq
  }

  hasChanged (): boolean {
    const p = this.entity.position.floored()
    const dx = this.x - p.x
    const dy = this.y - p.y
    const dz = this.z - p.z
    if ((dx * dx + dy * dy + dz * dz) > this.rangeSq) {
      this.x = p.x
      this.y = p.y
      this.z = p.z
      return true
    }
    return false
  }

  isValid (): boolean {
    return this.entity != null
  }
}

export interface GoalPlaceBlockOptions {
  range?: number
  faces?: Vec3[]
  facing?: string
  facing3D?: boolean
  half?: 'top' | 'bottom'
  LOS?: boolean
}

interface ResolvedPlaceOptions {
  range: number
  faces: Vec3[]
  facing: number
  facing3D?: boolean
  half?: 'top' | 'bottom'
  LOS: boolean
}

import { getShapeFaceCenters } from './shapes.js'

/** Path to a position from which the block at `pos` can be placed against. */
export class GoalPlaceBlock extends Goal {
  pos: Vec3
  world: RaycastWorld
  options: ResolvedPlaceOptions
  facesPos: Array<[Vec3, Vec3, Vec3]>

  constructor (pos: Vec3, world: RaycastWorld, options: GoalPlaceBlockOptions = {}) {
    super()
    this.pos = pos.floored()
    this.world = world
    const resolved: ResolvedPlaceOptions = {
      range: options.range ?? 5,
      faces: options.faces ?? [new Vec3(0, -1, 0), new Vec3(0, 1, 0), new Vec3(0, 0, -1), new Vec3(0, 0, 1), new Vec3(-1, 0, 0), new Vec3(1, 0, 0)],
      facing: ['north', 'east', 'south', 'west', 'up', 'down'].indexOf(options.facing as string),
      facing3D: options.facing3D,
      half: options.half,
      LOS: 'LOS' in options ? (options.LOS as boolean) : true
    }
    this.options = resolved
    this.facesPos = []
    for (const dir of this.options.faces) {
      const ref = this.pos.plus(dir)
      const refBlock = this.world.getBlock ? this.world.getBlock(ref) : null
      if (!refBlock) continue
      for (const center of getShapeFaceCenters(refBlock.shapes, dir.scaled(-1), this.options.half)) {
        this.facesPos.push([dir, center.add(ref), ref])
      }
    }
  }

  heuristic (node: XYZ): number {
    const dx = node.x - this.pos.x
    const dy = node.y - this.pos.y
    const dz = node.z - this.pos.z
    return distanceXZ(dx, dz) + Math.abs(dy < 0 ? dy + 1 : dy)
  }

  isEnd (node: XYZ): boolean {
    if (this.isStandingIn(node)) return false
    const headPos = new Vec3(node.x, node.y, node.z).offset(0.5, 1.6, 0.5)
    return this.getFaceAndRef(headPos) !== null
  }

  getFaceAndRef (headPos: Vec3): { face: Vec3, to: Vec3, ref: Vec3 } | null {
    for (const [face, to, ref] of this.facesPos) {
      const dir = to.minus(headPos)
      if (dir.norm() > this.options.range) continue
      if (!this.checkFacing(dir)) continue

      if (!this.options.LOS) {
        return { face, to, ref }
      }

      const block = this.world.raycast(headPos, dir.normalize(), this.options.range)
      if (block && block.position.equals(ref) && block.face === vectorToDirection(face.scaled(-1))) {
        return { face, to, ref }
      }
    }
    return null
  }

  checkFacing (dir: Vec3): boolean {
    if (this.options.facing < 0) return true

    if (this.options.facing3D) {
      const dH = Math.sqrt(dir.x * dir.x + dir.z * dir.z)
      const vAngle = Math.atan2(dir.y, dH) * 180 / Math.PI
      if (vAngle > 45) return this.options.facing === 4
      if (vAngle < -45) return this.options.facing === 5
    }
    const angle = Math.atan2(dir.x, -dir.z) * 180 / Math.PI + 180
    const facing = Math.floor(angle / 90 + 0.5) & 0x3

    return this.options.facing === facing
  }

  isStandingIn (node: XYZ): boolean {
    const dx = node.x - this.pos.x
    const dy = node.y - this.pos.y
    const dz = node.z - this.pos.z
    return (Math.abs(dx) + Math.abs(dy < 0 ? dy + 1 : dy) + Math.abs(dz)) < 1
  }
}

function vectorToDirection (v: Vec3): number {
  if (v.y < 0) return 0
  else if (v.y > 0) return 1
  else if (v.z < 0) return 2
  else if (v.z > 0) return 3
  else if (v.x < 0) return 4
  else if (v.x > 0) return 5
  return -1
}
