// Cache block-state classification per registry and movement profile to avoid per-probe allocation.
import type { Bot } from 'mineflayer'
import prismarineBlockLoader from 'prismarine-block'
import { LutFlags, LutSpecial } from './types.js'
import { topCatchClass, carryCode, thinFootprint, lidByte, CARRY_SHIFT } from './shapes.js'
import { simSlipOf, simKindOf, SIM_CLIMBABLE } from './playerSim.js'
import { isVineClimbable, physicsClimbsVines } from './movements.js'
import type { Movements } from './movements.js'

export interface BlockLut {
  fingerprint: string
  maxStateId: number
  /** LutFlags byte per state. */
  flags: Uint8Array
  /** Bits 0-5: collision top in 1/32 blocks; bits 6-7: top-catch class. */
  heights: Uint8Array
  /** Optional LutSpecial bytes for bubble columns, vines and extended parkour. */
  special: Uint8Array | null
  /** Optional thin-footprint bytes for extended parkour. */
  thin: Uint8Array | null
  /** Box offsets into shapeData; each box occupies six floats. */
  shapeStarts: Int32Array
  shapeCounts: Uint8Array
  /** Flattened AABBs: [x0,y0,z0,x1,y1,z1] per box. Deduplicated across states. */
  shapeData: Float32Array
  /** 1 for waterlogged states, which physics treats as water. */
  waterlogged: Uint8Array
  /** Physics tables: slipperiness (0 = default) and SIM_* kind bits per state. */
  simSlip: Float64Array
  simKind: Uint8Array
}

const MAX_LUTS_PER_REGISTRY = 8
const lutCache = new WeakMap<object, Map<string, BlockLut>>()

/** Get the LUT for this registry and classification settings, building it the first time. */
export function getLut (bot: Bot, movements: Movements): BlockLut {
  const registry = bot.registry as unknown as {
    blocksArray: Array<{
      id: number
      name: string
      minStateId: number
      maxStateId: number
      boundingBox: string
    }>
  }
  const fingerprint = movements.lutFingerprint()
  let cached = lutCache.get(registry)
  if (cached === undefined) {
    cached = new Map()
    lutCache.set(registry, cached)
  }
  const hit = cached.get(fingerprint)
  if (hit !== undefined) return hit

  const lut = buildLut(bot, movements)
  cached.set(fingerprint, lut)
  // Oldest first: a few profiles in turn stay cached, and memory stays bounded.
  if (cached.size > MAX_LUTS_PER_REGISTRY) cached.delete(cached.keys().next().value as string)
  return lut
}

export function buildLut (bot: Bot, movements: Movements): BlockLut {
  const registry = bot.registry as unknown as {
    blocksArray: Array<{
      id: number
      name: string
      minStateId: number
      maxStateId: number
      boundingBox: string
    }>
  }
  const Block = prismarineBlockLoader(bot.registry)
  const climbsVines = physicsClimbsVines(bot)

  let maxStateId = 0
  for (const b of registry.blocksArray) {
    if (b.maxStateId > maxStateId) maxStateId = b.maxStateId
  }

  const flags = new Uint8Array(maxStateId + 1)
  const heights = new Uint8Array(maxStateId + 1)
  // Allocate special data only for features that need bubble, vine or slime rules.
  const vineBlock = registry.blocksArray.find(b => b.name === 'vine')
  const vineClimb = vineBlock !== undefined && movements.climbables.has(vineBlock.id)
  const slimeMark = movements.allowParkourExtended
  const special = movements.useBubbleColumns || vineClimb || slimeMark ? new Uint8Array(maxStateId + 1) : null
  const thin = slimeMark ? new Uint8Array(maxStateId + 1) : null
  const shapeStarts = new Int32Array(maxStateId + 1).fill(-1)
  const shapeCounts = new Uint8Array(maxStateId + 1)
  const waterlogged = new Uint8Array(maxStateId + 1)
  const simSlip = new Float64Array(maxStateId + 1)
  const simKind = new Uint8Array(maxStateId + 1)
  const shapeChunks: number[] = []
  const shapeDedup = new Map<string, { start: number, count: number }>()

  for (const blockType of registry.blocksArray) {
    // boundingBox is type-level in prismarine-block, even when state shapes differ.
    const id = blockType.id
    const bboxEmpty = blockType.boundingBox === 'empty'
    const climbable = movements.climbables.has(id)
    const carpet = movements.carpets.has(id)
    const avoid = movements.blocksToAvoid.has(id)
    const liquid = movements.liquids.has(id)
    const fence = movements.fences.has(id)
    const gate = movements.openable.has(id)
    const door = movements.doors.has(id)

    let typeFlags = 0
    if ((bboxEmpty || climbable || carpet) && !avoid) typeFlags |= LutFlags.SAFE
    if (!bboxEmpty && !fence) typeFlags |= LutFlags.PHYSICAL
    if (liquid) typeFlags |= LutFlags.LIQUID
    if (climbable) typeFlags |= LutFlags.CLIMBABLE

    for (let stateId = blockType.minStateId; stateId <= blockType.maxStateId; stateId++) {
      const block = Block.fromStateId(stateId, 0)
      const shapes: number[][] = block.shapes ?? []

      let stateFlags = typeFlags
      if (gate) {
        // Only closed gates have collision shapes and need activation.
        // GATE_OPEN lets the optional open-gate rule accept gates with no shapes.
        stateFlags |= shapes.length !== 0 ? LutFlags.OPENABLE_GATE : LutFlags.GATE_OPEN
      }
      if (door) {
        const props = block.getProperties() as Record<string, unknown>
        const open = props.open === true || props.open === 'true'
        stateFlags |= open ? LutFlags.DOOR_OPEN : LutFlags.DOOR_CLOSED
      }
      // Extended movement uses state shapes to reject supports that cannot carry a body.
      if (slimeMark) {
        if (blockType.name === 'big_dripleaf') {
          // Big dripleaves tilt under a standing body, so they cannot be stable supports.
          stateFlags &= ~LutFlags.PHYSICAL
        } else if (shapes.length === 0 && !gate && !door && !carpet) {
          // A state without collision boxes cannot support the body.
          stateFlags &= ~LutFlags.PHYSICAL
        }
        // Powder snow sinks and freezes the body.
        if (blockType.name === 'powder_snow') stateFlags &= ~LutFlags.SAFE
      }
      flags[stateId] = stateFlags
      const wl = (block as { isWaterlogged?: unknown }).isWaterlogged
      if (wl === true || wl === 'true') waterlogged[stateId] = 1
      simSlip[stateId] = simSlipOf(blockType.name)
      simKind[stateId] = simKindOf(blockType.name, waterlogged[stateId] === 1)
      // The kernel climbs what the bot's own physics climbs.
      if (!climbsVines && isVineClimbable(blockType.name)) simKind[stateId] &= ~SIM_CLIMBABLE

      if (special) {
        if (movements.useBubbleColumns && blockType.name === 'bubble_column') {
          // Magma pulls entities down; soul sand pushes them up.
          const props = block.getProperties() as Record<string, unknown>
          const drag = props.drag === true || props.drag === 'true'
          special[stateId] = drag ? LutSpecial.BUBBLE_DOWN : LutSpecial.BUBBLE_UP
        } else if (vineClimb && blockType.name === 'vine') {
          special[stateId] = LutSpecial.VINE
        } else if (slimeMark && blockType.name === 'slime_block') {
          special[stateId] = LutSpecial.SLIME
        } else if (slimeMark && blockType.name.endsWith('_stairs')) {
          // Bottom stairs have a full-footprint half slab that catches a landing.
          // Top-half stairs already have a full-height top.
          const lowSlab = shapes.some(s =>
            s[1] === 0 && s[4] === 0.5 && s[0] === 0 && s[2] === 0 && s[3] === 1 && s[5] === 1)
          const hasUpperStep = shapes.some(s => s[4] > 0.5)
          if (lowSlab && hasUpperStep) special[stateId] = LutSpecial.STAIR
        }
        // Carry codes occupy bits 5-7 only on narrow supports, never bubbles, vines or slime.
        if (slimeMark) special[stateId] |= carryCode(shapes) << CARRY_SHIFT
      }

      // Shape tops fit exactly in six bits at 1/32-block precision; the top two bits store catch class.
      let top = 0
      for (const s of shapes) {
        if (s[4] > top) top = s[4]
      }
      heights[stateId] = Math.min(63, Math.round(top * 32)) | (topCatchClass(shapes) << 6)
      if (thin !== null) thin[stateId] = thinFootprint(shapes) || lidByte(shapes)

      if (shapes.length > 0) {
        const key = JSON.stringify(shapes)
        let entry = shapeDedup.get(key)
        if (!entry) {
          entry = { start: shapeChunks.length / 6, count: shapes.length }
          for (const s of shapes) shapeChunks.push(s[0], s[1], s[2], s[3], s[4], s[5])
          shapeDedup.set(key, entry)
        }
        shapeStarts[stateId] = entry.start
        shapeCounts[stateId] = Math.min(255, entry.count)
      }
    }
  }

  return {
    fingerprint: movements.lutFingerprint(),
    maxStateId,
    flags,
    heights,
    special,
    thin,
    shapeStarts,
    shapeCounts,
    shapeData: Float32Array.from(shapeChunks),
    waterlogged,
    simSlip,
    simKind
  }
}
