// Shared test infrastructure: a finite voxel world, a fake bot exposing the
// exact surface the package (and upstream mineflayer-pathfinder's Movements/
// AStar, used as the differential oracle) reads, snapshot construction over
// the same box, and an independent walkability checker for produced paths.
import { Vec3 } from 'vec3'
import minecraftData from 'minecraft-data'
import prismarineBlockLoader from 'prismarine-block'
import { Movements } from '../../src/movements.js'
import { buildLut } from '../../src/lut.js'
import type { BlockLut } from '../../src/lut.js'
import { Snapshot, nextSnapshotGeneration } from '../../src/snapshot.js'
import type { SnapshotMeta } from '../../src/types.js'

export const TEST_VERSION = '1.21.1'

export const mcData = minecraftData(TEST_VERSION)
export const Block = prismarineBlockLoader(TEST_VERSION)

export const AIR = mcData.blocksByName.air.minStateId as number
export const STONE = mcData.blocksByName.stone.minStateId as number
export const DIRT = mcData.blocksByName.dirt.minStateId as number
export const WATER = mcData.blocksByName.water.minStateId as number
export const LAVA = mcData.blocksByName.lava.minStateId as number
export const OAK_FENCE = mcData.blocksByName.oak_fence.minStateId as number
export const OAK_LEAVES = mcData.blocksByName.oak_leaves.minStateId as number
export const LADDER = mcData.blocksByName.ladder.minStateId as number
export const VINE = mcData.blocksByName.vine.minStateId as number
export const COBWEB = mcData.blocksByName.cobweb.minStateId as number
export const SOUL_SAND = mcData.blocksByName.soul_sand.minStateId as number
export const MAGMA = mcData.blocksByName.magma_block.minStateId as number
// bubble_column states: drag=true (min) pulls down, drag=false pushes up.
export const BUBBLE_DOWN_STATE = mcData.blocksByName.bubble_column.minStateId as number
export const BUBBLE_UP_STATE = (mcData.blocksByName.bubble_column.minStateId as number) + 1

export interface WorldBox {
  x0: number
  y0: number
  z0: number
  x1: number
  y1: number
  z1: number
}

/**
 * Finite voxel world: inside `box` unset cells are air; outside the box the
 * world is UNLOADED (blockAt → null), exactly matching the snapshot boundary
 * so upstream and our solver see identical geometry.
 */
export class VoxelWorld {
  readonly box: WorldBox
  private readonly cells = new Map<string, number>()

  constructor (box: WorldBox) {
    this.box = box
  }

  key (x: number, y: number, z: number): string {
    return `${x},${y},${z}`
  }

  inBounds (x: number, y: number, z: number): boolean {
    const b = this.box
    return x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1 && z >= b.z0 && z <= b.z1
  }

  set (x: number, y: number, z: number, stateId: number): void {
    if (!this.inBounds(x, y, z)) throw new Error(`set out of bounds: ${x},${y},${z}`)
    if (stateId === AIR) this.cells.delete(this.key(x, y, z))
    else this.cells.set(this.key(x, y, z), stateId)
  }

  fill (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, stateId: number): void {
    for (let x = x0; x <= x1; x++) {
      for (let y = y0; y <= y1; y++) {
        for (let z = z0; z <= z1; z++) this.set(x, y, z, stateId)
      }
    }
  }

  stateAt (x: number, y: number, z: number): number | null {
    if (!this.inBounds(x, y, z)) return null
    return this.cells.get(this.key(x, y, z)) ?? AIR
  }
}

interface FakeBot {
  registry: typeof mcData
  version: string
  game: { minY: number, height: number }
  world: { getColumn: (cx: number, cz: number) => unknown }
  entity: {
    position: Vec3
    velocity: Vec3
    onGround: boolean
    effects: Record<string, unknown>
    height: number
  }
  entities: Record<string, unknown>
  inventory: { items: () => unknown[] }
  blockAt: (pos: Vec3, extraInfos?: boolean) => unknown
  pathfinder?: unknown
}

/**
 * The read-only surface both sides consume. Upstream Movements calls
 * bot.blockAt / bot.registry / bot.game / bot.entities / bot.inventory;
 * our snapshot builder uses bot.world.getColumn.
 */
export function makeFakeBot (world: VoxelWorld, position = new Vec3(0.5, 0, 0.5)): FakeBot {
  const columns = new Map<string, unknown>()

  const bot: FakeBot = {
    registry: mcData,
    version: TEST_VERSION,
    game: { minY: world.box.y0, height: world.box.y1 - world.box.y0 + 1 },
    world: {
      getColumn (cx: number, cz: number) {
        const k = `${cx},${cz}`
        let col = columns.get(k)
        if (!col) {
          col = {
            getBlockStateId (pos: { x: number, y: number, z: number }): number {
              const state = world.stateAt((cx << 4) + pos.x, pos.y, (cz << 4) + pos.z)
              return state ?? AIR
            }
          }
          columns.set(k, col)
        }
        return col
      }
    },
    entity: {
      position,
      velocity: new Vec3(0, 0, 0),
      onGround: true,
      effects: {},
      height: 1.8
    },
    entities: {},
    inventory: { items: () => [] },
    blockAt (pos: Vec3): unknown {
      const x = Math.floor(pos.x)
      const y = Math.floor(pos.y)
      const z = Math.floor(pos.z)
      const state = world.stateAt(x, y, z)
      if (state === null) return null
      const block = Block.fromStateId(state, 0)
      ;(block as { position: Vec3 }).position = new Vec3(x, y, z)
      return block
    }
  }
  return bot
}

/** Walk-only Movements profile (both our class and upstream's accept it). */
export function applyProfile<T extends {
  canDig: boolean
  allowSprinting: boolean
  allowParkour: boolean
  allow1by1towers: boolean
  scafoldingBlocks: unknown[]
  canOpenDoors: boolean
  allowEntityDetection: boolean
  maxDropDown: number
}> (m: T, overrides: Partial<T> = {}): T {
  m.canDig = false
  m.allowSprinting = true
  m.allowParkour = true
  m.allow1by1towers = false
  m.scafoldingBlocks = []
  m.canOpenDoors = false
  m.allowEntityDetection = false
  m.maxDropDown = 4
  Object.assign(m, overrides)
  return m
}

export function makeOurMovements (bot: unknown, overrides: Record<string, unknown> = {}): Movements {
  const m = new Movements(bot as never)
  applyProfile(m as never, overrides as never)
  // Strict-parity defaults for differential tests: no door/gate improvement,
  // and no vine climbing (upstream never climbs vines — auto-enabled for
  // 1.16+ in the package, tested separately in vine.test.ts).
  ;(m as { canOpenRealDoors: boolean }).canOpenRealDoors = false
  m.climbables.delete(mcData.blocksByName.vine.id as number)
  Object.assign(m, overrides)
  return m
}

let cachedLut: { m: Movements, lut: BlockLut } | null = null

export function lutFor (bot: unknown, movements: Movements): BlockLut {
  if (cachedLut && cachedLut.m.lutFingerprint() === movements.lutFingerprint()) return cachedLut.lut
  const lut = buildLut(bot as never, movements)
  cachedLut = { m: movements, lut }
  return lut
}

/** Snapshot over exactly the world's box (parity with upstream's null-outside). */
export function snapshotFromWorld (world: VoxelWorld, lut: BlockLut, needStates = false): Snapshot {
  const b = world.box
  const meta: SnapshotMeta = {
    x0: b.x0,
    y0: b.y0,
    z0: b.z0,
    w: b.x1 - b.x0 + 1,
    h: b.y1 - b.y0 + 1,
    l: b.z1 - b.z0 + 1,
    worldMinY: b.y0,
    generation: nextSnapshotGeneration(),
    patchCount: 0
  }
  const snap = new Snapshot(meta, lut.fingerprint)
  const states = needStates ? snap.allocStates() : null
  const special = lut.special ? snap.allocSpecial() : null
  for (let y = b.y0; y <= b.y1; y++) {
    for (let z = b.z0; z <= b.z1; z++) {
      for (let x = b.x0; x <= b.x1; x++) {
        const state = world.stateAt(x, y, z) as number
        const idx = snap.index(x, y, z)
        snap.flags[idx] = lut.flags[state]
        snap.heights[idx] = lut.heights[state]
        if (states) states[idx] = state
        if (special) special[idx] = lut.special![state]
      }
    }
  }
  return snap
}

export interface PathStep {
  x: number
  y: number
  z: number
  parkour?: boolean
}

/**
 * Independent walkability checker (plan §5): validates a path against the
 * RAW world using only registry data — no LUT, no Movements — so a shared
 * misclassification can't hide a bad path. Conservative approximations:
 *   - a body cell must have no collision shape taller than 0.6 above its floor
 *   - each node needs support (solid top within 1 below, water, or ladder)
 *     unless it's a descent step
 *   - step height ≤ 1.25, horizontal reach ≤ √2 (4 for parkour edges)
 * Returns null when OK, else a description of the first violation.
 */
export function checkWalkable (world: VoxelWorld, start: PathStep, path: PathStep[]): string | null {
  const shapesAt = (x: number, y: number, z: number): number[][] => {
    const state = world.stateAt(x, y, z)
    if (state === null) return [[0, 0, 0, 1, 1, 1]] // unloaded = solid
    const block = Block.fromStateId(state, 0)
    return (block as { shapes: number[][] }).shapes ?? []
  }
  const nameAt = (x: number, y: number, z: number): string => {
    const state = world.stateAt(x, y, z)
    if (state === null) return 'unloaded'
    return (Block.fromStateId(state, 0) as { name: string }).name
  }
  const topAt = (x: number, y: number, z: number): number => {
    let top = 0
    for (const s of shapesAt(x, y, z)) top = Math.max(top, s[4])
    return top
  }
  const bodyBlocked = (x: number, y: number, z: number): boolean => {
    // Blocking = any collision above 0.6 within the feet or head cell,
    // except thin floors (carpet/bottom-slab handled via the floor rule).
    const feetTop = topAt(x, y, z)
    const headTop = topAt(x, y + 1, z)
    const name = nameAt(x, y, z)
    const headName = nameAt(x, y + 1, z)
    const passableName = (n: string): boolean =>
      n.includes('door') || n.includes('gate') || n === 'ladder' || n === 'vine' || n === 'water'
    if (feetTop > 0.6 && !passableName(name)) return true
    if (headTop > 0.11 && !passableName(headName)) return true
    return false
  }
  const supported = (x: number, y: number, z: number): boolean => {
    const name = nameAt(x, y, z)
    if (name === 'water' || name === 'ladder') return true
    const belowName = nameAt(x, y - 1, z)
    if (belowName === 'water') return true
    return topAt(x, y - 1, z) > 0
  }

  let prev = start
  for (let i = 0; i < path.length; i++) {
    const node = path[i]
    const label = `step ${i} (${prev.x},${prev.y},${prev.z})→(${node.x},${node.y},${node.z})`
    const dxz = Math.max(Math.abs(node.x - prev.x), Math.abs(node.z - prev.z))
    const dy = node.y - prev.y
    const maxReach = node.parkour ? 4 : Math.SQRT2 + 0.01
    if (dxz > maxReach) return `${label}: horizontal reach ${dxz} > ${maxReach}`
    if (dy > 1) return `${label}: climbs ${dy} in one step`
    if (bodyBlocked(node.x, node.y, node.z)) return `${label}: body cell blocked (${nameAt(node.x, node.y, node.z)}/${nameAt(node.x, node.y + 1, node.z)})`
    if (dy >= 0 && !supported(node.x, node.y, node.z)) {
      return `${label}: no support under non-descending step`
    }
    prev = node
  }
  return null
}
