// Which inventory items can break a fall, and getting one into a hand within a tick.
import type { Item } from 'prismarine-item'

/**
 * How a breaker works, which decides the packet and the reach window:
 * - `water`: a bucket used on the support's top face (use_item, 4.5 reach); the landing cell fills with water.
 * - `cobweb`: placed on the support's top face (block_place); the body may already overlap the cell.
 * - `cushion`: a hay-like block placed on the support's top face; the body lands on it and keeps a fifth of the damage.
 */
export type FallItemKind = 'water' | 'cobweb' | 'cushion'

/** One rule: which items act as which kind of breaker, and what using one costs. */
export interface FallItemRule {
  kind: FallItemKind
  /** An item name, a list of names, an item id, or a test on the item (custom names, components, modded items). */
  match: string | readonly string[] | number | ((item: Item) => boolean)
  /**
   * What spending the item is worth, in health points (default 1). A breaker is only used when it
   * saves more health than this. Water buckets come back after the landing, fish buckets lose their fish.
   */
  cost?: number
}

/** The breakers known by default, cheapest first. Spread it to add your own rules. */
export const defaultFallItems: readonly FallItemRule[] = Object.freeze([
  { kind: 'water', match: 'water_bucket', cost: 0.5 },
  { kind: 'water', match: ['cod_bucket', 'salmon_bucket', 'pufferfish_bucket', 'tropical_fish_bucket', 'axolotl_bucket', 'tadpole_bucket'], cost: 4 },
  { kind: 'cobweb', match: 'cobweb', cost: 2 },
  { kind: 'cushion', match: 'hay_block', cost: 3 }
] as FallItemRule[])

/** An item a breaker can come from, and where it sits. */
export interface FallItemRef {
  item: Item
  kind: FallItemKind
  cost: number
  /** Inventory window slot: 36-44 hotbar, 45 offhand, 9-35 main inventory. */
  slot: number
}

/** Does the item match the rule? A throwing predicate counts as no. */
export function ruleMatches (rule: FallItemRule, item: Item): boolean {
  const m = rule.match
  try {
    if (typeof m === 'function') return m(item) === true
    if (typeof m === 'number') return item.type === m
    if (typeof m === 'string') return item.name === m
    return m.includes(item.name)
  } catch {
    return false
  }
}

interface BotLike {
  inventory?: { slots?: Array<Item | null> }
  currentWindow?: unknown
  quickBarSlot: number
}

/**
 * Usable breakers, best rule first per item. `inventory` lists the items the bot may spend
 * (default: everything from the main inventory, hotbar and offhand). Each must keep its `slot`.
 */
export function usableBreakers (
  bot: BotLike,
  rules: readonly FallItemRule[],
  inventory?: (bot: never) => Iterable<Item | null | undefined>
): FallItemRef[] {
  const out: FallItemRef[] = []
  const slots = bot.inventory?.slots
  if (slots === undefined) return out
  let items: Iterable<Item | null | undefined>
  try {
    items = inventory !== undefined ? inventory(bot as never) : defaultItems(bot)
  } catch {
    return out
  }
  const seen = new Set<number>()
  for (const item of items) {
    if (item === null || item === undefined || typeof item.slot !== 'number' || seen.has(item.slot)) continue
    // The item must still be where the inventory says it is.
    if (slots[item.slot] !== item && slots[item.slot]?.name !== item.name) continue
    // With a container open, only the hotbar and offhand can be reached without window clicks.
    if (bot.currentWindow !== null && bot.currentWindow !== undefined && !inHand(item.slot)) continue
    for (const rule of rules) {
      if (!ruleMatches(rule, item)) continue
      out.push({ item, kind: rule.kind, cost: rule.cost ?? 1, slot: item.slot })
      seen.add(item.slot)
      break
    }
  }
  return out
}

function defaultItems (bot: BotLike): Array<Item | null> {
  const out: Array<Item | null> = []
  const slots = bot.inventory?.slots ?? []
  for (let s = 9; s <= 45; s++) out.push(slots[s] ?? null)
  return out
}

/** Hotbar or offhand: usable with at most a held_item_slot packet. */
export function inHand (slot: number): boolean {
  return slot >= 36
}

/** The hand a staged item is used from, and the hotbar index to select (null for the offhand). */
export function handFor (slot: number): { hand: 0 | 1, hotbar: number | null } {
  if (slot === 45) return { hand: 1, hotbar: null }
  if (slot >= 36 && slot <= 44) return { hand: 0, hotbar: slot - 36 }
  return { hand: 0, hotbar: null }
}
