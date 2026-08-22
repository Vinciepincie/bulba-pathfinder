// Per-state dig tables for canDig solves. Upstream re-runs bestHarvestTool +
// block.digTime for EVERY unsafe block probe (nbt parsing per neighbor!);
// the numbers only depend on (block type × inventory × effects), so we
// compute them once per solve and cache by an inventory fingerprint —
// identical costs, computed thousands of times less often.
import prismarineBlockLoader from 'prismarine-block'
import nbt from 'prismarine-nbt'
import type { Bot } from 'mineflayer'
import type { Movements } from './movements.js'
import type { BlockLut } from './lut.js'
import { DigFlags } from './types.js'
import type { DigData } from './types.js'

interface ToolLike {
  type: number
  nbt?: unknown
}

interface DigBlockLike {
  digTime (toolType: number | null, creative: boolean, inWater: boolean, notOnGround: boolean, enchants: unknown[], effects: unknown): number
}

/** Fingerprint of everything digTime can depend on. */
export function digFingerprint (bot: Bot, movements: Movements, lut: BlockLut): string {
  const items = bot.inventory.items() as ToolLike[]
  const toolKey = items
    .map(i => `${i.type}${i.nbt ? '+' : ''}`)
    .sort()
    .join(',')
  const effects = (bot.entity as unknown as { effects?: Record<string, unknown> }).effects ?? {}
  const effectKey = Object.keys(effects).sort().join(',')
  const cantBreakKey = [...movements.blocksCantBreak].sort((a, b) => a - b).join(',')
  const gravityKey = [...movements.gravityBlocks].sort((a, b) => a - b).join(',')
  return `${lut.fingerprint}|${toolKey}|${effectKey}|${cantBreakKey}|${gravityKey}`
}

export function computeDigData (bot: Bot, movements: Movements, lut: BlockLut): DigData {
  const registry = bot.registry as unknown as {
    blocksArray: Array<{ id: number, minStateId: number, maxStateId: number }>
  }
  const Block = prismarineBlockLoader(bot.registry)

  const labor = new Float32Array(lut.maxStateId + 1)
  const flags = new Uint8Array(lut.maxStateId + 1)

  // Pre-simplify tool enchantments once (upstream does this per probe).
  const items = bot.inventory.items() as ToolLike[]
  const tools = items.map(tool => ({
    type: tool.type,
    enchants: (tool && tool.nbt)
      ? ((nbt.simplify(tool.nbt as never) as { Enchantments?: unknown[] }).Enchantments ?? [])
      : []
  }))
  const effects = (bot.entity as { effects?: unknown }).effects

  for (const blockType of registry.blocksArray) {
    let typeFlags = 0
    if (movements.gravityBlocks.has(blockType.id)) typeFlags |= DigFlags.CAN_FALL
    if (movements.blocksCantBreak.has(blockType.id)) typeFlags |= DigFlags.CANT_BREAK

    let typeLabor = 0
    if ((typeFlags & DigFlags.CANT_BREAK) === 0) {
      const block = Block.fromStateId(blockType.minStateId, 0) as unknown as DigBlockLike
      // bestHarvestTool, upstream-identical: fastest digTime over inventory
      // (bare hand only when the inventory is empty — upstream quirk kept).
      let fastest = Number.MAX_VALUE
      try {
        for (const tool of tools) {
          const t = block.digTime(tool.type, false, false, false, tool.enchants, effects)
          if (t < fastest) fastest = t
        }
        if (tools.length === 0) fastest = block.digTime(null, false, false, false, [], effects)
      } catch {
        fastest = Number.MAX_VALUE
      }
      // Upstream: laborCost = (1 + 3 * digTime / 1000) * digCost. digCost is
      // applied at move-generation time; unbreakable-in-practice blocks
      // (digTime Infinity) get a huge labor so the 100 gate rejects them.
      typeLabor = Number.isFinite(fastest) ? 1 + (3 * fastest) / 1000 : 1e9
    }

    for (let s = blockType.minStateId; s <= blockType.maxStateId; s++) {
      labor[s] = typeLabor
      flags[s] = typeFlags
    }
  }

  return { fingerprint: digFingerprint(bot, movements, lut), labor, flags }
}
