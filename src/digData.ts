// Dig costs depend on block type, inventory and effects, so cache them per solve.
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

/** Cache key for inventory, effects, LUT and dig restrictions. */
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

  // Enchantment parsing is shared by all block probes.
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
      // Use the fastest inventory tool, or bare hands when the inventory is empty.
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
      // Move generation applies digCost; infinite dig times must exceed the labor gate.
      typeLabor = Number.isFinite(fastest) ? 1 + (3 * fastest) / 1000 : 1e9
    }

    for (let s = blockType.minStateId; s <= blockType.maxStateId; s++) {
      labor[s] = typeLabor
      flags[s] = typeFlags
    }
  }

  return { fingerprint: digFingerprint(bot, movements, lut), labor, flags }
}
