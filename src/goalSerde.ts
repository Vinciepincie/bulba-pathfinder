// Goal (de)serialization for the worker boundary. Only EXACT instances of
// this package's goal classes are serializable — a user subclass (overridden
// heuristic/isEnd) silently falls back to main-thread solving so custom
// logic always runs, exactly like upstream.
import { Vec3 } from 'vec3'
import {
  Goal, GoalBlock, GoalNear, GoalXZ, GoalNearXZ, GoalY, GoalGetToBlock,
  GoalLookAtBlock, GoalBreakBlock, GoalCompositeAny, GoalCompositeAll,
  GoalInvert, GoalFollow, GoalPlaceBlock
} from './goals.js'
import type { RaycastWorld } from './goals.js'
import type { GoalDescriptor } from './types.js'

export function serializeGoal (goal: Goal): GoalDescriptor | null {
  const ctor = (goal as { constructor: unknown }).constructor
  if (ctor === GoalBlock) {
    const g = goal as GoalBlock
    return { type: 'block', x: g.x, y: g.y, z: g.z }
  }
  if (ctor === GoalNear) {
    const g = goal as GoalNear
    return { type: 'near', x: g.x, y: g.y, z: g.z, rangeSq: g.rangeSq }
  }
  if (ctor === GoalXZ) {
    const g = goal as GoalXZ
    return { type: 'xz', x: g.x, z: g.z }
  }
  if (ctor === GoalNearXZ) {
    const g = goal as GoalNearXZ
    return { type: 'nearxz', x: g.x, z: g.z, rangeSq: g.rangeSq }
  }
  if (ctor === GoalY) {
    const g = goal as GoalY
    return { type: 'y', y: g.y }
  }
  if (ctor === GoalGetToBlock) {
    const g = goal as GoalGetToBlock
    return { type: 'getToBlock', x: g.x, y: g.y, z: g.z }
  }
  if (ctor === GoalLookAtBlock) {
    const g = goal as GoalLookAtBlock
    return {
      type: 'lookAt',
      x: g.pos.x,
      y: g.pos.y,
      z: g.pos.z,
      reach: g.reach,
      entityHeight: g.entityHeight
    }
  }
  if (ctor === GoalBreakBlock) {
    const g = (goal as GoalBreakBlock).goal
    return {
      type: 'lookAt',
      x: g.pos.x,
      y: g.pos.y,
      z: g.pos.z,
      reach: g.reach,
      entityHeight: g.entityHeight
    }
  }
  if (ctor === GoalFollow) {
    const g = goal as GoalFollow
    // Frozen coordinates: hasChanged() runs on the live goal on the main
    // thread and triggers a re-solve with fresh coordinates (upstream
    // mutates mid-search; freezing per solve is strictly cleaner).
    return { type: 'follow', x: g.x, y: g.y, z: g.z, rangeSq: g.rangeSq }
  }
  if (ctor === GoalInvert) {
    const child = serializeGoal((goal as GoalInvert).goal)
    if (!child) return null
    return { type: 'invert', goal: child }
  }
  if (ctor === GoalCompositeAny || ctor === GoalCompositeAll) {
    const children: GoalDescriptor[] = []
    for (const child of (goal as GoalCompositeAny).goals) {
      const c = serializeGoal(child)
      if (!c) return null
      children.push(c)
    }
    return { type: ctor === GoalCompositeAny ? 'compositeAny' : 'compositeAll', goals: children }
  }
  if (ctor === GoalPlaceBlock) {
    const g = goal as GoalPlaceBlock
    return {
      type: 'placeBlock',
      x: g.pos.x,
      y: g.pos.y,
      z: g.pos.z,
      range: g.options.range,
      facing: g.options.facing,
      facing3D: g.options.facing3D === true,
      LOS: g.options.LOS,
      // facesPos precomputed on the main thread (needs live world.getBlock).
      facesPos: g.facesPos.map(([face, to, ref]) => ({
        face: { x: face.x, y: face.y, z: face.z },
        to: { x: to.x, y: to.y, z: to.z },
        ref: { x: ref.x, y: ref.y, z: ref.z }
      }))
    }
  }
  return null
}

/** True when solving this goal requires the snapshot to carry raw state ids. */
export function goalNeedsRaycast (descriptor: GoalDescriptor): boolean {
  switch (descriptor.type) {
    case 'lookAt':
    case 'placeBlock':
      return true
    case 'invert':
      return goalNeedsRaycast(descriptor.goal as GoalDescriptor)
    case 'compositeAny':
    case 'compositeAll':
      return (descriptor.goals as GoalDescriptor[]).some(goalNeedsRaycast)
    default:
      return false
  }
}

/** Known XZ(+Y) targets for snapshot box sizing. */
export function descriptorTargets (descriptor: GoalDescriptor): Array<{ x: number, y?: number, z: number }> {
  switch (descriptor.type) {
    case 'block':
    case 'near':
    case 'getToBlock':
    case 'lookAt':
    case 'follow':
    case 'placeBlock':
      return [{ x: descriptor.x as number, y: descriptor.y as number, z: descriptor.z as number }]
    case 'xz':
    case 'nearxz':
      return [{ x: descriptor.x as number, z: descriptor.z as number }]
    case 'compositeAny':
    case 'compositeAll':
      return (descriptor.goals as GoalDescriptor[]).flatMap(descriptorTargets)
    default:
      return []
  }
}

/** Worker-side reconstruction of a GoalPlaceBlock from its serialized faces. */
class RehydratedPlaceGoal extends Goal {
  private readonly pos: Vec3
  private readonly world: RaycastWorld
  private readonly range: number
  private readonly facing: number
  private readonly facing3D: boolean
  private readonly LOS: boolean
  private readonly facesPos: Array<[Vec3, Vec3, Vec3]>

  constructor (d: GoalDescriptor, world: RaycastWorld) {
    super()
    this.pos = new Vec3(d.x as number, d.y as number, d.z as number)
    this.world = world
    this.range = d.range as number
    this.facing = d.facing as number
    this.facing3D = d.facing3D as boolean
    this.LOS = d.LOS as boolean
    this.facesPos = (d.facesPos as Array<{ face: Vec3, to: Vec3, ref: Vec3 }>).map(f => [
      new Vec3(f.face.x, f.face.y, f.face.z),
      new Vec3(f.to.x, f.to.y, f.to.z),
      new Vec3(f.ref.x, f.ref.y, f.ref.z)
    ])
  }

  heuristic (node: { x: number, y: number, z: number }): number {
    const dx = node.x - this.pos.x
    const dy = node.y - this.pos.y
    const dz = node.z - this.pos.z
    const adx = Math.abs(dx)
    const adz = Math.abs(dz)
    return Math.abs(adx - adz) + Math.min(adx, adz) * Math.SQRT2 + Math.abs(dy < 0 ? dy + 1 : dy)
  }

  isEnd (node: { x: number, y: number, z: number }): boolean {
    const dx = node.x - this.pos.x
    const dy = node.y - this.pos.y
    const dz = node.z - this.pos.z
    if ((Math.abs(dx) + Math.abs(dy < 0 ? dy + 1 : dy) + Math.abs(dz)) < 1) return false
    const headPos = new Vec3(node.x, node.y, node.z).offset(0.5, 1.6, 0.5)
    for (const [face, to, ref] of this.facesPos) {
      const dir = to.minus(headPos)
      if (dir.norm() > this.range) continue
      if (!this.checkFacing(dir)) continue
      if (!this.LOS) return true
      const block = this.world.raycast(headPos, dir.normalize(), this.range)
      if (block && block.position.equals(ref) && block.face === vectorToDirection(face.scaled(-1))) {
        return true
      }
    }
    return false
  }

  private checkFacing (dir: Vec3): boolean {
    if (this.facing < 0) return true
    if (this.facing3D) {
      const dH = Math.sqrt(dir.x * dir.x + dir.z * dir.z)
      const vAngle = Math.atan2(dir.y, dH) * 180 / Math.PI
      if (vAngle > 45) return this.facing === 4
      if (vAngle < -45) return this.facing === 5
    }
    const angle = Math.atan2(dir.x, -dir.z) * 180 / Math.PI + 180
    return this.facing === (Math.floor(angle / 90 + 0.5) & 0x3)
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

export function instantiateGoal (d: GoalDescriptor, world: RaycastWorld | null): Goal {
  switch (d.type) {
    case 'block':
      return new GoalBlock(d.x as number, d.y as number, d.z as number)
    case 'near': {
      const g = new GoalNear(d.x as number, d.y as number, d.z as number, 0)
      g.rangeSq = d.rangeSq as number
      return g
    }
    case 'xz':
      return new GoalXZ(d.x as number, d.z as number)
    case 'nearxz': {
      const g = new GoalNearXZ(d.x as number, d.z as number, 0)
      g.rangeSq = d.rangeSq as number
      return g
    }
    case 'y':
      return new GoalY(d.y as number)
    case 'getToBlock':
      return new GoalGetToBlock(d.x as number, d.y as number, d.z as number)
    case 'lookAt': {
      if (!world) throw new Error('lookAt goal needs a raycast world')
      return new GoalLookAtBlock(
        new Vec3(d.x as number, d.y as number, d.z as number),
        world,
        { reach: d.reach as number, entityHeight: d.entityHeight as number }
      )
    }
    case 'follow': {
      const g = new GoalNear(d.x as number, d.y as number, d.z as number, 0)
      g.rangeSq = d.rangeSq as number
      return g
    }
    case 'invert':
      return new GoalInvert(instantiateGoal(d.goal as GoalDescriptor, world))
    case 'compositeAny':
      return new GoalCompositeAny((d.goals as GoalDescriptor[]).map(c => instantiateGoal(c, world)))
    case 'compositeAll':
      return new GoalCompositeAll((d.goals as GoalDescriptor[]).map(c => instantiateGoal(c, world)))
    case 'placeBlock': {
      if (!world) throw new Error('placeBlock goal needs a raycast world')
      return new RehydratedPlaceGoal(d, world)
    }
    default:
      throw new Error(`Unknown goal descriptor type: ${d.type}`)
  }
}
