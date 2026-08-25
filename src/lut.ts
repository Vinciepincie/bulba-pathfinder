// Block-classification LUT: per block-state → flags byte + collision-top
// height + collision shapes. Built once per (minecraft version × movements
// profile) from the bot's registry, then every solver probe is two typed
// array reads instead of a world hash lookup + pseudo-block allocation.
//
// The flag semantics reproduce mineflayer-pathfinder's Movements.getBlock()
// classification EXACTLY (including its type-level boundingBox quirks) so
// the solver's reachability matches upstream move for move.
import type { Bot } from 'mineflayer'
import prismarineBlockLoader from 'prismarine-block'
import { LutFlags, LutSpecial } from './types.js'
import { topCatchClass } from './shapes.js'
import type { Movements } from './movements.js'

export interface BlockLut {
  fingerprint: string
  maxStateId: number
  /** flags[stateId] — LutFlags byte. */
  flags: Uint8Array
  /** heights[stateId] — packed: bits 0–5 max collision-shape top in 1/32
   * blocks (0 when no shapes), bits 6–7 topCatchClass (see types.ts). */
  heights: Uint8Array
  /** special[stateId] — LutSpecial byte (bubble columns). Null unless the
   * profile opts into a feature that needs it (useBubbleColumns). */
  special: Uint8Array | null
  /** shapeStarts[stateId] — box offset into shapeData (in boxes, ×6 floats); shapeCounts boxes. */
  shapeStarts: Int32Array
  shapeCounts: Uint8Array
  /** Flattened AABBs: [x0,y0,z0,x1,y1,z1] per box. Deduplicated across states. */
  shapeData: Float32Array
}

interface LutCacheEntry {
  fingerprint: string
  lut: BlockLut
}

const lutCache = new WeakMap<object, LutCacheEntry>()

/**
 * Build (or fetch from cache) the LUT for this bot's registry + movements
 * profile. Cached per registry object; rebuilt when the movements profile's
 * classification-relevant sets change (fingerprint).
 */
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
  const cached = lutCache.get(registry)
  if (cached && cached.fingerprint === fingerprint) return cached.lut

  const lut = buildLut(bot, movements)
  lutCache.set(registry, { fingerprint, lut })
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

  let maxStateId = 0
  for (const b of registry.blocksArray) {
    if (b.maxStateId > maxStateId) maxStateId = b.maxStateId
  }

  const flags = new Uint8Array(maxStateId + 1)
  const heights = new Uint8Array(maxStateId + 1)
  // The special grid exists only when a feature needs it: bubble columns,
  // climbable vines (vine cells need the wall-adjacency rule), and slime
  // blocks when the extended-parkour repertoire (bounce moves) is on.
  const vineBlock = registry.blocksArray.find(b => b.name === 'vine')
  const vineClimb = vineBlock !== undefined && movements.climbables.has(vineBlock.id)
  const slimeMark = movements.allowParkourExtended
  const special = movements.useBubbleColumns || vineClimb || slimeMark ? new Uint8Array(maxStateId + 1) : null
  const shapeStarts = new Int32Array(maxStateId + 1).fill(-1)
  const shapeCounts = new Uint8Array(maxStateId + 1)
  const shapeChunks: number[] = []
  const shapeDedup = new Map<string, { start: number, count: number }>()

  for (const blockType of registry.blocksArray) {
    // Type-level classification — identical to upstream's per-probe logic:
    //   climbable  = climbables.has(type)
    //   safe       = (boundingBox === 'empty' || climbable || carpets.has(type)) && !blocksToAvoid.has(type)
    //   physical   = boundingBox === 'block' && !fences.has(type)
    //   liquid     = liquids.has(type)
    // boundingBox in prismarine-block is per TYPE (from minecraft-data), so
    // this is exact — not an approximation.
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
        // Upstream: `canOpenDoors && blockC.openable && blockC.shapes.length !== 0`
        // — only a closed gate (has collision) is activated; an open gate has
        // no shapes and (because bbox stays 'block') stays blocked upstream.
        // GATE_OPEN powers the opt-in open-gate-passable improvement.
        stateFlags |= shapes.length !== 0 ? LutFlags.OPENABLE_GATE : LutFlags.GATE_OPEN
      }
      if (door) {
        const props = block.getProperties() as Record<string, unknown>
        const open = props.open === true || props.open === 'true'
        stateFlags |= open ? LutFlags.DOOR_OPEN : LutFlags.DOOR_CLOSED
      }
      flags[stateId] = stateFlags

      if (special) {
        if (movements.useBubbleColumns && blockType.name === 'bubble_column') {
          // drag=true (magma below) pulls entities down; drag=false (soul
          // sand below) pushes them up — prismarine-physics mirrors vanilla.
          const props = block.getProperties() as Record<string, unknown>
          const drag = props.drag === true || props.drag === 'true'
          special[stateId] = drag ? LutSpecial.BUBBLE_DOWN : LutSpecial.BUBBLE_UP
        } else if (vineClimb && blockType.name === 'vine') {
          special[stateId] = LutSpecial.VINE
        } else if (slimeMark && blockType.name === 'slime_block') {
          special[stateId] = LutSpecial.SLIME
        }
      }

      // Collision-top height, quantized to 1/32 blocks (vanilla shape tops
      // are multiples of 1/16 — a few 1/32 — and max out at 1.5, so 6 bits
      // are exact), packed with the topCatchClass in bits 6–7 (types.ts).
      let top = 0
      for (const s of shapes) {
        if (s[4] > top) top = s[4]
      }
      heights[stateId] = Math.min(63, Math.round(top * 32)) | (topCatchClass(shapes) << 6)

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
    shapeStarts,
    shapeCounts,
    shapeData: Float32Array.from(shapeChunks)
  }
}
