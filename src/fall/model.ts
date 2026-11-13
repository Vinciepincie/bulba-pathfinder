// What a landing costs in health on 1.21.x: raw points (fallDamage.ts), then Resistance, then
// Feather Falling / Protection, then absorption. Armor points do not reduce falls.
import { fallDamagePoints } from '../fallDamage.js'
import { SIM_SLIME, SIM_CUSHION, SIM_BED } from '../playerSim.js'

/** Everything about the body that changes what a fall costs, read once per episode tick. */
export interface FallBody {
  /** The safe_fall_distance attribute, Jump Boost included (default 3). */
  safeFall: number
  /** The fall_damage_multiplier attribute (default 1). */
  multiplier: number
  /** Resistance levels (amplifier + 1), 0 without the effect. */
  resistance: number
  /** Enchantment protection factor against falls, capped at 20. */
  epf: number
  absorption: number
  health: number
  /** Slow Falling resets the fall every tick. */
  slowFalling: boolean
  /** A totem in either hand turns a lethal landing into 1 HP. */
  totemInHand: boolean
}

export const DEFAULT_BODY: FallBody = {
  safeFall: 3, multiplier: 1, resistance: 0, epf: 0, absorption: 0, health: 20, slowFalling: false, totemInHand: false
}

/**
 * Raw damage points of a landing; `kind` holds the SIM_* flags of the block landed on. The same rule as
 * fallDamagePoints, with the attributes inside the one rounding (vanilla calculateFallDamage).
 */
export function landingPoints (fall: number, last: number, kind: number, sneaking: boolean, whole: boolean, body: FallBody): number {
  if (body.slowFalling) return 0
  if (body.safeFall === 3 && body.multiplier === 1) return fallDamagePoints(fall, last, kind, sneaking, whole)
  if ((kind & SIM_SLIME) !== 0 && !sneaking) return 0
  const reach = (kind & SIM_BED) !== 0 ? 0.5 : 1
  const kept = ((kind & SIM_CUSHION) !== 0 ? 0.2 : 1) * body.multiplier
  if (whole) return Math.max(0, Math.floor((fall * reach + 1e-6 - body.safeFall) * kept))
  return Math.max(0, Math.ceil(((fall - last) * reach - body.safeFall) * kept))
}

/** Health a landing of `points` raw damage removes, after Resistance, enchantments and absorption. */
export function healthLoss (points: number, body: FallBody): number {
  if (points <= 0) return 0
  let dmg = points * Math.max(0, 1 - 0.2 * body.resistance)
  dmg *= 1 - Math.min(20, Math.max(0, body.epf)) / 25
  return Math.max(0, dmg - body.absorption)
}

/** Health left after losing `loss`; a totem in hand keeps 1 when the loss would kill. */
export function healthAfter (loss: number, body: FallBody): number {
  const left = body.health - loss
  if (left <= 0 && body.totemInHand) return 1
  return left
}

interface AttrLike { value: number, modifiers?: Array<{ amount: number, operation: number }> }
interface ItemLike { name?: string, enchants?: Array<{ name: string | null, lvl: number }> }

/** Attribute value with its modifiers applied (add, then multiply base, then multiply total). */
export function attributeValue (attr: AttrLike): number {
  const mods = attr.modifiers ?? []
  let x = attr.value
  for (const m of mods) if (m.operation === 0) x += m.amount
  let y = x
  for (const m of mods) if (m.operation === 1) y += x * m.amount
  for (const m of mods) if (m.operation === 2) y *= 1 + m.amount
  return y
}

const norm = (s: string): string => s.replace(/^minecraft:/, '').replace(/_/g, '').toLowerCase()

/** Read the fall-relevant state of a mineflayer bot. Unknown data counts as no protection. */
export function readFallBody (bot: unknown): FallBody {
  const b = bot as {
    health?: number
    registry?: { effects?: Record<number, { name?: string }>, attributesByName?: Record<string, { resource?: string }> }
    entity?: { effects?: Record<number, { id: number, amplifier: number }>, attributes?: Record<string, AttrLike>, metadata?: unknown[] }
    inventory?: { slots?: Array<ItemLike | null> }
    heldItem?: ItemLike | null
    quickBarSlot?: number
  }
  const body: FallBody = { ...DEFAULT_BODY }
  if (typeof b.health === 'number') body.health = b.health
  const attrs = b.entity?.attributes ?? {}
  let safeSeen = false
  for (const [key, attr] of Object.entries(attrs)) {
    const k = norm(key)
    if (k === 'safefalldistance' || k === 'genericsafefalldistance') { body.safeFall = attributeValue(attr); safeSeen = true }
    if (k === 'falldamagemultiplier' || k === 'genericfalldamagemultiplier') body.multiplier = attributeValue(attr)
  }
  const effects = b.entity?.effects ?? {}
  for (const e of Object.values(effects)) {
    const name = norm(b.registry?.effects?.[e.id]?.name ?? '')
    if (name === 'resistance') body.resistance = Math.min(5, e.amplifier + 1)
    if (name === 'slowfalling') body.slowFalling = true
    // Jump Boost reaches the server's safe distance through the attribute; count it only without one.
    if (name === 'jumpboost' && !safeSeen) body.safeFall += e.amplifier + 1
  }
  const slots = b.inventory?.slots ?? []
  let epf = 0
  for (let i = 5; i <= 8; i++) {
    for (const ench of enchantsOf(slots[i], b.registry as { enchantments?: Record<number, { name?: string }> })) {
      if (ench.name === 'feather_falling') epf += 3 * ench.lvl
      if (ench.name === 'protection') epf += ench.lvl
    }
  }
  body.epf = Math.min(20, epf)
  const held = b.heldItem ?? (typeof b.quickBarSlot === 'number' ? slots[36 + b.quickBarSlot] : null)
  body.totemInHand = held?.name === 'totem_of_undying' || slots[45]?.name === 'totem_of_undying'
  return body
}

/**
 * Enchantments as names and levels. prismarine-item returns `[{ name, lvl }]` from NBT, but the raw
 * 1.20.5+ component `{ enchantments: [{ id, level }] }`, whose ids index the registry.
 */
export function enchantsOf (item: ItemLike | null | undefined, registry?: { enchantments?: Record<number, { name?: string }> }): Array<{ name: string | null, lvl: number }> {
  if (item === null || item === undefined) return []
  try {
    const raw = item.enchants as unknown
    const list: unknown[] = Array.isArray(raw)
      ? raw
      : (raw !== null && typeof raw === 'object' && Array.isArray((raw as { enchantments?: unknown }).enchantments) ? (raw as { enchantments: unknown[] }).enchantments : [])
    return list.map(e => {
      const o = e as { name?: string | null, lvl?: number, id?: number | string, level?: number }
      let name = o.name ?? null
      if (name === null && typeof o.id === 'string') name = o.id
      if (name === null && typeof o.id === 'number') name = registry?.enchantments?.[o.id]?.name ?? null
      return { name: name === null ? null : name.replace(/^minecraft:/, ''), lvl: o.lvl ?? o.level ?? 0 }
    })
  } catch {
    return []
  }
}
