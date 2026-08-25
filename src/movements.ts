// API-compatible port of mineflayer-pathfinder's Movements class. Every
// upstream field exists with the same meaning so existing code that tweaks a
// Movements profile keeps working unchanged.
//
// Deliberate divergences:
//   - canDig defaults to FALSE (upstream: true). Digging is fully supported
//     with the upstream cost model when enabled, but it is opt-in — a
//     pathfinder that silently digs is how bots get banned.
//   - scafoldingBlocks defaults to [] and allow1by1towers to false. Non-empty
//     scaffolding / towers are ignored with a one-time warning: this planner
//     generates no placement moves (digging yes, placing no).
import { Vec3 } from 'vec3'
import type { Bot } from 'mineflayer'
import prismarineBlockLoader from 'prismarine-block'
import type { MovementsConfig } from './types.js'
import { interactableBlocks as interactableJson } from './interactableBlocks.js'
import { passableEntities as passableEntitiesJson } from './passableEntities.js'

/**
 * Should vines default to climbable on this version? True from 1.16 (the
 * vanilla `climbable` block tag — vines climb unconditionally in Java from
 * there). Uses the registry's own comparator when present (minecraft-data
 * ships one on every version); otherwise parses "major.minor" from the
 * version string and answers conservatively (false) when it can't tell.
 */
export function vineClimbingDefault (registry: unknown, version: string | undefined): boolean {
  const reg = registry as { isNewerOrEqualTo?: (v: string) => boolean, version?: { minecraftVersion?: string } }
  if (typeof reg.isNewerOrEqualTo === 'function') {
    try {
      return reg.isNewerOrEqualTo('1.16')
    } catch {
      // fall through to string parsing
    }
  }
  const source = version ?? reg.version?.minecraftVersion ?? ''
  const m = /^(\d+)\.(\d+)/.exec(source)
  if (!m) return false
  const major = Number(m[1])
  const minor = Number(m[2])
  return major > 1 || (major === 1 && minor >= 16)
}

export type ExclusionArea = (block: unknown) => number

export class Movements {
  bot: Bot

  // ── upstream-compatible knobs ────────────────────────────────────────────
  /** Default false (opt-in). When true, dig moves use the upstream cost model. */
  canDig: boolean
  digCost: number
  placeCost: number
  liquidCost: number
  entityCost: number
  /** Improvement (opt-in): ride bubble-column elevators (soul sand up, magma down). */
  useBubbleColumns: boolean
  /** Cost per block of bubble-column ride (default 1; true ride is faster). */
  bubbleCost: number
  /**
   * Improvement (allowParkourExtended): how far short of the physics limit
   * a planned jump stays, in blocks (default 0.1). 0 plans frame-tight
   * jumps — a practised player's course — at the price of more refused
   * take-offs; the executor's rollout still gates every one.
   */
  parkourSafetyMargin: number

  dontCreateFlow: boolean
  dontMineUnderFallingBlock: boolean
  allow1by1towers: boolean
  allowFreeMotion: boolean
  allowParkour: boolean
  /** Improvement (opt-in): extended parkour — see MovementsConfig.allowParkourExtended. */
  allowParkourExtended: boolean
  /** Improvement (opt-in, needs allowParkourExtended): momentum-aware search — see MovementsConfig.allowParkourMomentum. */
  allowParkourMomentum: boolean
  allowSprinting: boolean
  /**
   * Improvement (opt-in): hold jump while sprinting across open ground.
   *
   * Sprint-hopping is how a player actually crosses a plain — the jump keeps
   * the sprint boost that ground friction eats. Measured on the arena's own
   * server: 6.97 blocks/s against 5.56 sprinting, a 25% gain, with zero
   * position corrections. It is only taken when a rollout of BOTH gaits down
   * the same path says the hop gets further without losing height, so a ledge
   * or a low ceiling simply declines it.
   *
   * Executor-only: the search never sees it, paths are unchanged and the wasm
   * core needs nothing. Off by default — upstream has no such gait, and the
   * package's promise is that the walking outcome matches until you ask for
   * more.
   */
  allowSprintHop: boolean
  /**
   * Improvement (opt-in, needs allowSprintHop): hop through LOW-headroom
   * ground instead of walking round or under it — a tunnel, an overhang, a
   * single block in the way overhead.
   *
   * A 2-block roof is the fastest ground in the game for a bot that re-presses
   * jump on each landing: the arc bonks off the ceiling and lands in ~5 ticks,
   * so the sprint boost is renewed four times as often as it is under open
   * sky. Measured on flat stone, same physics the executor rolls out against:
   * sprint 5.59 blocks/s, hold-jump 6.50, press-on-landing 9.68.
   *
   * What it changes is only how far ahead a DROP in the ceiling vetoes a
   * take-off. Off, that is the whole comparison horizon, which is safe and
   * also switches the gait off for an entire passage — in a mostly-2-high
   * tunnel every 3-high pocket has a low section within six nodes. On, the
   * veto spans the arc's own footprint, so the bot sprints the last step into
   * a low section and hops the moment it is under it, and never takes off
   * from high ground into a ceiling it would hit side-on.
   */
  allowLowCeilingHop: boolean
  /**
   * Improvement: steer at the furthest node reachable in a straight line the
   * body fits down, instead of visiting every cell centre in turn.
   *
   * The planner routes over eight directions, so a run a few degrees off a
   * cardinal comes back as an alternating zig-zag; a follower that aims at
   * each centre walks every zig and swings its heading at every one. Skipped
   * nodes retire by being gone by rather than by being stood on, and only
   * where a swept hitbox says the line is walkable and the skipped nodes stay
   * within collecting range of it.
   *
   * Executor-only: the plan is unchanged, and turning this off restores
   * node-by-node following exactly.
   */
  allowCornerCut: boolean
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
  /**
   * Improvement over upstream: when canOpenDoors is set, also traverse
   * closed non-iron doors by activating them (upstream only handles fence
   * gates). Set to false for strict upstream reachability parity.
   */
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

    this.canDig = false // divergence from upstream default (true): digging is opt-in
    this.digCost = 1
    this.placeCost = 1
    this.liquidCost = 1
    this.entityCost = 1
    this.useBubbleColumns = false // improvement, opt-in (upstream: columns are plain air)
    this.bubbleCost = 1
    this.parkourSafetyMargin = 0.1

    this.dontCreateFlow = true
    this.dontMineUnderFallingBlock = true
    this.allow1by1towers = false // divergence: no placement moves exist
    this.allowFreeMotion = false
    this.allowParkour = true
    this.allowParkourExtended = false // improvement, opt-in (upstream only jumps straight and flat)
    this.allowParkourMomentum = false // improvement, opt-in (landing momentum as search state)
    this.allowSprinting = true
    this.allowSprintHop = false // improvement, opt-in (executor gait only)
    this.allowLowCeilingHop = false // improvement, opt-in (executor gait only)
    this.allowCornerCut = true // improvement, on (executor steering only; plan unchanged)
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
    // Improvement over upstream (which ships `climbables.add(vine)` commented
    // out): vines are unconditionally climbable in vanilla Java since the
    // 1.16 `climbable` block tag, and prismarine-physics climbs them, so the
    // executor can perform every vine plan. Auto-enabled on 1.16+ only —
    // older versions gated vine climbing on a backing block, which the
    // planner can't verify, so there we keep upstream's behavior. Opt out
    // with `movements.climbables.delete(vineId)`. Deliberately ladder+vine
    // ONLY: nether/cave vines are in the vanilla climbable tag too, but
    // prismarine-physics does NOT climb them — planning them would produce
    // paths the bot physically cannot walk.
    if (registry.blocksByName.vine && vineClimbingDefault(registry, (bot as { version?: string }).version)) {
      this.climbables.add(registry.blocksByName.vine.id)
    }

    this.emptyBlocks = new Set()

    this.replaceables = new Set()
    this.replaceables.add(registry.blocksByName.air.id)
    if (registry.blocksByName.cave_air) this.replaceables.add(registry.blocksByName.cave_air.id)
    if (registry.blocksByName.void_air) this.replaceables.add(registry.blocksByName.void_air.id)
    this.replaceables.add(registry.blocksByName.water.id)
    this.replaceables.add(registry.blocksByName.lava.id)

    // Divergence: upstream defaults to [dirt, cobblestone]; this planner has
    // no placement moves, so scaffolding is inert and defaults empty.
    this.scafoldingBlocks = []

    // Same classification pass as upstream: per block TYPE from its default
    // state — fences are anything with collision top > 1, carpets < 0.1.
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

  // ── upstream-compatible helpers used by the plugin ───────────────────────

  exclusionPlace (block: unknown): number {
    if (this.exclusionAreasPlace.length === 0) return 0
    let weight = 0
    for (const a of this.exclusionAreasPlace) weight += a(block)
    return weight
  }

  exclusionStep (block: unknown): number {
    if (this.exclusionAreasStep.length === 0) return 0
    let weight = 0
    for (const a of this.exclusionAreasStep) weight += a(block)
    return weight
  }

  exclusionBreak (block: unknown): number {
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

  /**
   * Finds blocks intersected by entity bounding boxes, upstream-identical.
   * The result is baked into the solver snapshot per compute.
   */
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

  /**
   * Upstream-compatible pseudo-block probe (external consumers use this).
   * The internal solver does NOT go through here — it reads the LUT — but
   * both implement the identical classification.
   */
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

  /** Upstream-compatible; always false here — this planner never digs. */
  safeToBreak (block: { position?: { x: number, y: number, z: number }, type?: number }): boolean {
    if (!this.canDig) return false
    // Unreachable in this package (canDig is rejected at setMovements), but
    // keep the upstream logic for API completeness.
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
    return Boolean(block.type) && !this.blocksCantBreak.has(block.type as number) && this.exclusionBreak(block) < 100
  }

  /** Upstream-compatible cost probe: safe blocks cost their weights, unbreakable = 100. */
  safeOrBreak (block: { safe?: boolean, position?: { x: number, y: number, z: number } }, toBreak: Array<{ x: number, y: number, z: number }> = []): number {
    let cost = 0
    cost += this.exclusionStep(block)
    cost += this.getNumEntitiesAt(block.position ?? null, 0, 0, 0) * this.entityCost
    if (block.safe) return cost
    if (!this.safeToBreak(block as { position?: { x: number, y: number, z: number }, type?: number })) return 100
    if (block.position) toBreak.push(block.position)
    return cost // dig-time labor cost is unreachable with canDig=false
  }

  // ── package-internal ─────────────────────────────────────────────────────

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
      allowParkourMomentum: this.allowParkourMomentum
    }
  }

  /**
   * Fingerprint of everything the block-classification LUT depends on. The
   * LUT (and any snapshot built from it) is rebuilt when this changes.
   */
  lutFingerprint (): string {
    const setKey = (s: Set<number>): string => [...s].sort((a, b) => a - b).join(',')
    return [
      // Registry identity: state ids renumber between MC versions, so two
      // bots on different versions must never share LUT tables.
      String((this.bot as { version?: string }).version ?? ''),
      setKey(this.blocksToAvoid),
      setKey(this.liquids),
      setKey(this.climbables),
      setKey(this.fences),
      setKey(this.carpets),
      setKey(this.openable),
      setKey(this.doors),
      this.canOpenRealDoors ? 'd1' : 'd0',
      // The LUT's special grid (bubble columns) exists only when opted in.
      this.useBubbleColumns ? 'b1' : 'b0',
      // Extended parkour marks slime blocks into the special grid.
      this.allowParkourExtended ? 'x1' : 'x0'
    ].join('|')
  }

  /** Loud invariant check, called from setMovements / goto. */
  assertSupported (): void {
    if ((this.allow1by1towers || this.scafoldingBlocks.length > 0) && !warnedScaffolding) {
      warnedScaffolding = true
      console.warn(
        '@bulba/pathfinder: allow1by1towers/scafoldingBlocks are ignored — this planner generates no block-placement moves.'
      )
    }
  }
}

let warnedScaffolding = false
