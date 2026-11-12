// Unlike mineflayer-pathfinder, digging defaults off and block placement is unsupported.
import { Vec3 } from 'vec3'
import type { Bot } from 'mineflayer'
import prismarineBlockLoader from 'prismarine-block'
import type { Block } from 'prismarine-block'
import type { MovementsConfig } from './types.js'
import { interactableBlocks as interactableJson } from './interactableBlocks.js'
import { passableEntities as passableEntitiesJson } from './passableEntities.js'

/** Whether vines are climbable by default on this version (Java 1.16+).
 * Uses the registry comparator or parses the version; unknown versions default to false. */
export function vineClimbingDefault (registry: unknown, version: string | undefined): boolean {
  const reg = registry as { isNewerOrEqualTo?: (v: string) => boolean, version?: { minecraftVersion?: string } }
  if (typeof reg.isNewerOrEqualTo === 'function') {
    try {
      return reg.isNewerOrEqualTo('1.16')
    } catch {
    }
  }
  const source = version ?? reg.version?.minecraftVersion ?? ''
  const m = /^(\d+)\.(\d+)/.exec(source)
  if (!m) return false
  const major = Number(m[1])
  const minor = Number(m[2])
  return major > 1 || (major === 1 && minor >= 16)
}

/** Extra cost for a block, from a function of yours. It receives the live block from bot.blockAt. */
export type ExclusionArea = (block: Block) => number

/** Additional vines in the vanilla climbable tag: nether vines from 1.16 and cave vines from 1.17. */
export const VINE_CLIMBABLES = ['weeping_vines', 'weeping_vines_plant', 'twisting_vines', 'twisting_vines_plant', 'cave_vines', 'cave_vines_plant'] as const
const VINE_CLIMBABLE_NAMES: ReadonlySet<string> = new Set(VINE_CLIMBABLES)

/** Named sets of movement flags for `Movements.applyPreset`. */
export type MovementPreset = 'default' | 'parkour'

/** Is this one of the vines only a patched prismarine-physics climbs? */
export function isVineClimbable (name: string): boolean {
  return VINE_CLIMBABLE_NAMES.has(name)
}

const vineProbe = new WeakMap<object, boolean>()

/**
 * Does the bot's prismarine-physics climb nether and cave vines? Stock does not, and
 * scripts/patchPhysics.mjs adds them. One tick of falling through a vine cell tells:
 * a climbable caps the fall, stock falls on. True when it cannot be told (a stand-in physics).
 */
export function physicsClimbsVines (bot: unknown): boolean {
  const b = bot as { physics?: { simulatePlayer?: (state: unknown, world: unknown) => void }, registry?: { blocksByName?: Record<string, { id: number, name: string } | undefined> } }
  const physics = b.physics
  if (physics?.simulatePlayer === undefined) return true
  const known = vineProbe.get(physics)
  if (known !== undefined) return known
  const vine = b.registry?.blocksByName?.weeping_vines ?? b.registry?.blocksByName?.cave_vines
  if (vine === undefined) return true
  const cell = { type: vine.id, name: vine.name, shapes: [], boundingBox: 'empty', position: new Vec3(0, 1000, 0), getProperties: () => ({}) }
  const state = {
    pos: new Vec3(0.5, 1000, 0.5),
    vel: new Vec3(0, -0.5, 0),
    control: { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false },
    onGround: false,
    isInWater: false,
    isInLava: false,
    isInWeb: false,
    isCollidedHorizontally: false,
    isCollidedVertically: false,
    elytraFlying: false,
    jumpTicks: 0,
    jumpQueued: false,
    fireworkRocketDuration: 0,
    yaw: 0,
    pitch: 0,
    jumpBoost: 0,
    speed: 0,
    slowness: 0,
    dolphinsGrace: 0,
    slowFalling: 0,
    levitation: 0,
    depthStrider: 0
  }
  try {
    physics.simulatePlayer(state, {
      getBlock: (p: Vec3) => Math.floor(p.x) === 0 && Math.floor(p.y) === 1000 && Math.floor(p.z) === 0 ? cell : null
    })
  } catch {
    return true // Not memoized: the physics may not be ready yet.
  }
  const climbs = state.vel.y >= -0.5
  vineProbe.set(physics, climbs)
  return climbs
}

export class Movements {
  bot: Bot

  /** Enable digging with the mineflayer-pathfinder cost model. Default false, unlike mineflayer-pathfinder. */
  canDig: boolean
  digCost: number
  placeCost: number
  liquidCost: number
  entityCost: number
  /** Enable bubble-column elevators: soul sand pushes up, magma pulls down. Default false. */
  useBubbleColumns: boolean
  /** Cost per block of bubble-column travel. Default 1. */
  bubbleCost: number
  /** Blocks of jump reach reserved for timing uncertainty and server corrections; 0 allows frame-perfect jumps. */
  parkourSafetyMargin: number

  dontCreateFlow: boolean
  dontMineUnderFallingBlock: boolean
  allow1by1towers: boolean
  allowFreeMotion: boolean
  allowParkour: boolean
  /** Enable diagonal, long, raised and drop jumps, including ladder and liquid catches. Default false. */
  allowParkourExtended: boolean
  /** Carry landing direction into search state to chain jumps. Default false; requires allowParkourExtended. */
  allowParkourMomentum: boolean
  /** Offer frame-perfect jumps at a cost premium as a fallback. Default true; requires allowParkourExtended. */
  allowParkourTight: boolean
  /** Use physics-verified hops when table search fails. Default false; requires allowParkourExtended. */
  allowParkourPhysics: boolean
  allowSprinting: boolean
  /** Enable sprint-hopping when a rollout shows more progress without losing height.
   * Airborne travel preserves the sprint boost that ground friction reduces. Default false. */
  allowSprintHop: boolean
  /** When false, stopping releases controls without snapping position or zeroing momentum. */
  allowStopSnap: boolean
  /** Limit ceiling-drop checks to the jump's footprint so low tunnels remain eligible for hopping.
   * Prevents takeoff into the side of a lower ceiling. Default false; requires allowSprintHop. */
  allowLowCeilingHop: boolean
  /** Steer to the furthest node reachable by a swept hitbox to smooth the planner's zig-zags.
   * Skipped nodes stay within collecting range. Default true; disabling follows each node. */
  allowCornerCut: boolean
  /** Back off and sprint across the support for jumps that standing reach cannot clear.
   * Executor only; default false. */
  allowRunUp: boolean
  /** Retire a parkour landing when the body lands within a block beyond it along the flight line,
   * avoiding a turn back into the node's arrival box. Default false. */
  allowLandingRetire: boolean
  /** Let the physics kernel choose and replay the fastest run-and-jump schedule through nearby gaps
   * and rises. Default false; requires allowSprintHop. */
  allowHopIntoJump: boolean
  /** Run and hop with 45-degree strafe input: its magnitude is 1.0 versus 0.98 for forward alone.
   * Default false; requires allowHopIntoJump. */
  allowStrafe: boolean
  allowEntityDetection: boolean

  entitiesToAvoid: Set<string>
  passableEntities: Set<string>
  interactableBlocks: Set<string>

  blocksCantBreak: Set<number>
  blocksToAvoid: Set<number>
  liquids: Set<number>
  gravityBlocks: Set<number>
  climbables: Set<number>
  emptyBlocks: Set<number>
  replaceables: Set<number>
  scafoldingBlocks: number[]
  fences: Set<number>
  carpets: Set<number>
  openable: Set<number>

  canOpenDoors: boolean
  /** Also activate closed non-iron doors when canOpenDoors is enabled. Default true;
   * mineflayer-pathfinder handles only fence gates. */
  canOpenRealDoors: boolean
  /** Non-iron door block ids (traversed via activate when doors are enabled). */
  doors: Set<number>

  exclusionAreasStep: ExclusionArea[]
  exclusionAreasBreak: ExclusionArea[]
  exclusionAreasPlace: ExclusionArea[]

  maxDropDown: number
  infiniteLiquidDropdownDistance: boolean

  entityIntersections: Record<string, number>

  constructor (bot: Bot) {
    const registry = bot.registry
    this.bot = bot

    this.canDig = false 
    this.digCost = 1
    this.placeCost = 1
    this.liquidCost = 1
    this.entityCost = 1
    this.useBubbleColumns = false 
    this.bubbleCost = 1
    this.parkourSafetyMargin = 0.1

    this.dontCreateFlow = true
    this.dontMineUnderFallingBlock = true
    this.allow1by1towers = false 
    this.allowFreeMotion = false
    this.allowParkour = true
    this.allowParkourExtended = false 
    this.allowParkourMomentum = false 
    this.allowParkourTight = true 
    this.allowParkourPhysics = false 
    this.allowSprinting = true
    this.allowSprintHop = false 
    this.allowStopSnap = true 
    this.allowLowCeilingHop = false 
    this.allowCornerCut = true 
    this.allowRunUp = false 
    this.allowLandingRetire = false 
    this.allowHopIntoJump = false 
    this.allowStrafe = false 
    this.allowEntityDetection = true

    this.entitiesToAvoid = new Set()
    this.passableEntities = new Set(passableEntitiesJson)
    this.interactableBlocks = new Set(interactableJson)

    this.blocksCantBreak = new Set()
    this.blocksCantBreak.add(registry.blocksByName.chest.id)
    for (const block of registry.blocksArray) {
      if (block.diggable) continue
      this.blocksCantBreak.add(block.id)
    }

    this.blocksToAvoid = new Set()
    this.blocksToAvoid.add(registry.blocksByName.fire.id)
    if (registry.blocksByName.cobweb) this.blocksToAvoid.add(registry.blocksByName.cobweb.id)
    if ((registry.blocksByName as Record<string, { id: number } | undefined>).web) {
      this.blocksToAvoid.add((registry.blocksByName as Record<string, { id: number }>).web.id)
    }
    this.blocksToAvoid.add(registry.blocksByName.lava.id)

    this.liquids = new Set()
    this.liquids.add(registry.blocksByName.water.id)
    this.liquids.add(registry.blocksByName.lava.id)

    this.gravityBlocks = new Set()
    this.gravityBlocks.add(registry.blocksByName.sand.id)
    this.gravityBlocks.add(registry.blocksByName.gravel.id)

    this.climbables = new Set()
    this.climbables.add(registry.blocksByName.ladder.id)
    // Before Java 1.16, vines require a backing block that the planner cannot verify.
    // Nether and cave vines rely on the package's prismarine-physics patch for climbing.
    if (registry.blocksByName.vine && vineClimbingDefault(registry, (bot as { version?: string }).version)) {
      this.climbables.add(registry.blocksByName.vine.id)
      for (const name of physicsClimbsVines(bot) ? VINE_CLIMBABLES : []) {
        const b = registry.blocksByName[name]
        if (b !== undefined) this.climbables.add(b.id)
      }
    }

    this.emptyBlocks = new Set()

    this.replaceables = new Set()
    this.replaceables.add(registry.blocksByName.air.id)
    if (registry.blocksByName.cave_air) this.replaceables.add(registry.blocksByName.cave_air.id)
    if (registry.blocksByName.void_air) this.replaceables.add(registry.blocksByName.void_air.id)
    this.replaceables.add(registry.blocksByName.water.id)
    this.replaceables.add(registry.blocksByName.lava.id)

    // Scaffolding defaults empty because the planner generates no placement moves.
    this.scafoldingBlocks = []

    // Classification uses each block type's default state: fences have tops > 1, carpets < 0.1.
    const Block = prismarineBlockLoader(bot.registry)
    this.fences = new Set()
    this.carpets = new Set()
    this.openable = new Set()
    this.doors = new Set()
    for (const x of registry.blocksArray) {
      const block = Block.fromStateId(x.minStateId, 0)
      if (block.shapes.length > 0) {
        if (block.shapes[0][4] > 1) this.fences.add(block.type)
        if (block.shapes[0][4] < 0.1) this.carpets.add(block.type)
      } else if (block.shapes.length === 0) {
        this.emptyBlocks.add(block.type)
      }
    }
    for (const block of registry.blocksArray) {
      const name = block.name.toLowerCase()
      if (this.interactableBlocks.has(block.name) && name.includes('gate') && !name.includes('iron')) {
        this.openable.add(block.id)
      }
      if (this.interactableBlocks.has(block.name) && name.endsWith('_door') && !name.includes('iron')) {
        this.doors.add(block.id)
      }
    }

    this.canOpenDoors = false
    this.canOpenRealDoors = true

    this.exclusionAreasStep = []
    this.exclusionAreasBreak = []
    this.exclusionAreasPlace = []

    this.maxDropDown = 4
    this.infiniteLiquidDropdownDistance = true

    this.entityIntersections = {}
  }


  exclusionPlace (block: Block): number {
    if (this.exclusionAreasPlace.length === 0) return 0
    let weight = 0
    for (const a of this.exclusionAreasPlace) weight += a(block)
    return weight
  }

  exclusionStep (block: Block): number {
    if (this.exclusionAreasStep.length === 0) return 0
    let weight = 0
    for (const a of this.exclusionAreasStep) weight += a(block)
    return weight
  }

  exclusionBreak (block: Block): number {
    if (this.exclusionAreasBreak.length === 0) return 0
    let weight = 0
    for (const a of this.exclusionAreasBreak) weight += a(block)
    return weight
  }

  countScaffoldingItems (): number {
    let count = 0
    const items = this.bot.inventory.items()
    for (const id of this.scafoldingBlocks) {
      for (const item of items) {
        if (item.type === id) count += item.count
      }
    }
    return count
  }

  getScaffoldingItem (): unknown {
    const items = this.bot.inventory.items()
    for (const id of this.scafoldingBlocks) {
      for (const item of items) {
        if (item.type === id) return item
      }
    }
    return null
  }

  clearCollisionIndex (): void {
    this.entityIntersections = {}
  }

  /** Index blocks intersected by entity bounding boxes for the solver snapshot. */
  updateCollisionIndex (): void {
    for (const ent of Object.values(this.bot.entities)) {
      if (ent === this.bot.entity) continue

      const entName = (ent as { name?: string }).name ?? ''
      const avoidedEnt = this.entitiesToAvoid.has(entName)
      if (avoidedEnt || !this.passableEntities.has(entName)) {
        const entSquareRadius = ent.width / 2.0
        const minY = Math.floor(ent.position.y)
        const maxY = Math.ceil(ent.position.y + ent.height)
        const minX = Math.floor(ent.position.x - entSquareRadius)
        const maxX = Math.ceil(ent.position.x + entSquareRadius)
        const minZ = Math.floor(ent.position.z - entSquareRadius)
        const maxZ = Math.ceil(ent.position.z + entSquareRadius)

        const cost = avoidedEnt ? 100 : 1

        for (let y = minY; y < maxY; y++) {
          for (let x = minX; x < maxX; x++) {
            for (let z = minZ; z < maxZ; z++) {
              const key = `${x},${y},${z}`
              this.entityIntersections[key] = (this.entityIntersections[key] ?? 0) + cost
            }
          }
        }
      }
    }
  }

  getNumEntitiesAt (pos: { x: number, y: number, z: number } | null, dx: number, dy: number, dz: number): number {
    if (this.allowEntityDetection === false) return 0
    if (!pos) return 0
    return this.entityIntersections[`${pos.x + dx},${pos.y + dy},${pos.z + dz}`] ?? 0
  }

  /** Pseudo-block probe for external consumers; the internal solver reads the equivalent LUT classification. */
  getBlock (pos: { x: number, y: number, z: number } | null, dx: number, dy: number, dz: number): Record<string, unknown> {
    const b = pos
      ? (this.bot.blockAt(new Vec3(pos.x + dx, pos.y + dy, pos.z + dz), false) as Record<string, unknown> | null)
      : null
    if (!b) {
      return {
        replaceable: false,
        canFall: false,
        safe: false,
        physical: false,
        liquid: false,
        climbable: false,
        height: (pos ? pos.y : 0) + dy,
        openable: false
      }
    }
    const type = b.type as number
    b.climbable = this.climbables.has(type)
    b.safe = ((b.boundingBox === 'empty') || (b.climbable as boolean) || this.carpets.has(type)) && !this.blocksToAvoid.has(type)
    b.physical = b.boundingBox === 'block' && !this.fences.has(type)
    b.replaceable = this.replaceables.has(type) && !(b.physical as boolean)
    b.liquid = this.liquids.has(type)
    b.height = (pos as { y: number }).y + dy
    b.canFall = this.gravityBlocks.has(type)
    b.openable = this.openable.has(type)
    for (const shape of (b.shapes as number[][] | undefined) ?? []) {
      b.height = Math.max(b.height as number, (pos as { y: number }).y + dy + shape[4])
    }
    return b
  }

  /** Check whether mining is enabled and safe under liquid, falling-block and exclusion constraints. */
  safeToBreak (block: { position?: { x: number, y: number, z: number }, type?: number }): boolean {
    if (!this.canDig) return false
    // Mining beside liquid can create a flow into the path.
    if (this.dontCreateFlow) {
      if ((this.getBlock(block.position ?? null, 0, 1, 0) as { liquid: boolean }).liquid) return false
      if ((this.getBlock(block.position ?? null, -1, 0, 0) as { liquid: boolean }).liquid) return false
      if ((this.getBlock(block.position ?? null, 1, 0, 0) as { liquid: boolean }).liquid) return false
      if ((this.getBlock(block.position ?? null, 0, 0, -1) as { liquid: boolean }).liquid) return false
      if ((this.getBlock(block.position ?? null, 0, 0, 1) as { liquid: boolean }).liquid) return false
    }
    if (this.dontMineUnderFallingBlock) {
      if ((this.getBlock(block.position ?? null, 0, 1, 0) as { canFall: boolean }).canFall ||
          this.getNumEntitiesAt(block.position ?? null, 0, 1, 0) > 0) {
        return false
      }
    }
    return Boolean(block.type) && !this.blocksCantBreak.has(block.type as number) && this.exclusionBreak(block as unknown as Block) < 100
  }

  /** Return step and entity weights, or 100 for an unbreakable block; append required breaks to toBreak. */
  safeOrBreak (block: { safe?: boolean, position?: { x: number, y: number, z: number } }, toBreak: Array<{ x: number, y: number, z: number }> = []): number {
    let cost = 0
    cost += this.exclusionStep(block as unknown as Block)
    cost += this.getNumEntitiesAt(block.position ?? null, 0, 0, 0) * this.entityCost
    if (block.safe) return cost
    if (!this.safeToBreak(block as { position?: { x: number, y: number, z: number }, type?: number })) return 100
    if (block.position) toBreak.push(block.position)
    return cost 
  }


  /** Serializable solver config snapshot of this profile. */
  toConfig (): MovementsConfig {
    return {
      allowSprinting: this.allowSprinting,
      allowParkour: this.allowParkour,
      allowParkourExtended: this.allowParkourExtended,
      canOpenDoors: this.canOpenDoors,
      canOpenRealDoors: this.canOpenRealDoors,
      maxDropDown: this.maxDropDown,
      infiniteLiquidDropdownDistance: this.infiniteLiquidDropdownDistance,
      liquidCost: this.liquidCost,
      entityCost: this.entityCost,
      canDig: this.canDig,
      digCost: this.digCost,
      dontCreateFlow: this.dontCreateFlow,
      dontMineUnderFallingBlock: this.dontMineUnderFallingBlock,
      useBubbleColumns: this.useBubbleColumns,
      bubbleCost: this.bubbleCost,
      parkourSafetyMargin: this.parkourSafetyMargin,
      allowParkourMomentum: this.allowParkourMomentum,
      allowParkourTight: this.allowParkourTight,
      allowParkourPhysics: this.allowParkourPhysics
    }
  }

  /** Fingerprint of the block-classification settings; changes invalidate
   * the LUT and snapshots built from it. */
  lutFingerprint (): string {
    const setKey = (s: Set<number>): string => [...s].sort((a, b) => a - b).join(',')
    return [
      // State IDs vary by Minecraft version, so bots on different versions cannot share LUTs.
      String((this.bot as { version?: string }).version ?? ''),
      // The kernel's climbable table follows the bot's physics, not only this profile.
      physicsClimbsVines(this.bot) ? 'v1' : 'v0',
      setKey(this.blocksToAvoid),
      setKey(this.liquids),
      setKey(this.climbables),
      setKey(this.fences),
      setKey(this.carpets),
      setKey(this.openable),
      setKey(this.doors),
      this.canOpenRealDoors ? 'd1' : 'd0',
      // Optional special-grid contents are part of the cache identity.
      this.useBubbleColumns ? 'b1' : 'b0',
      this.allowParkourExtended ? 'x1' : 'x0'
    ].join('|')
  }

  /**
   * Apply a named set of movement flags, and return this profile.
   * `parkour` turns on every physics-verified move and the fastest gaits; `default` turns them back off.
   */
  applyPreset (preset: MovementPreset): this {
    if (preset !== 'default' && preset !== 'parkour') throw new Error(`Unknown movement preset: ${String(preset)}`)
    const on = preset === 'parkour'
    this.allowParkour = true
    this.allowSprinting = true
    this.allowCornerCut = true
    this.allowParkourExtended = on
    this.allowParkourMomentum = on
    this.allowParkourPhysics = on
    this.allowSprintHop = on
    this.allowLowCeilingHop = on
    this.allowRunUp = on
    this.allowLandingRetire = on
    this.allowHopIntoJump = on
    this.allowStrafe = on
    return this
  }

  /** Warn once when unsupported block-placement settings are enabled. */
  assertSupported (warn: (message: string) => void = console.warn): void {
    if ((this.allow1by1towers || this.scafoldingBlocks.length > 0) && !warnedScaffolding) {
      warnedScaffolding = true
      warn(
        'bulba-pathfinder: allow1by1towers and scafoldingBlocks are ignored, because this planner generates no block placement moves.'
      )
    }
  }
}

let warnedScaffolding = false
