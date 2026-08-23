// A tiny bot for action-layer tests. Not a physics simulator — just enough
// world, inventory and protocol surface for dig/place/open to run against,
// with switches for the failure modes those actions exist to survive.
import { EventEmitter } from 'node:events'
import { Vec3 } from 'vec3'

export interface FakeBlock {
  type: number
  name: string
  position: Vec3
  boundingBox: string
  shapes?: number[][]
  digTime?: (...args: unknown[]) => number
}

export interface FakeItem { type: number, name: string }

export interface ActionBot extends EventEmitter {
  entity: { position: Vec3, velocity: Vec3, onGround: boolean, height: number, yaw: number }
  world: { raycast: (from: Vec3, dir: Vec3, range: number) => { position: Vec3, face: number } | null }
  inventory: { items: () => FakeItem[], emptySlotCount: () => number }
  heldItem: FakeItem | null
  currentWindow: unknown
  controlState: Record<string, boolean>

  blockAt: (pos: Vec3, extraInfos?: boolean) => FakeBlock | null
  blockAtCursor: (max?: number) => (FakeBlock & { face: number }) | null
  setControlState: (name: string, value: boolean) => void
  clearControlStates: () => void
  look: (yaw: number, pitch: number, force?: boolean) => Promise<void>
  lookAt: (pos: Vec3, force?: boolean) => Promise<void>
  waitForTicks: (ticks: number) => Promise<void>
  dig: (block: unknown, forceLook?: unknown, face?: unknown) => Promise<void>
  stopDigging: () => void
  placeBlock: (ref: unknown, face: Vec3) => Promise<void>
  activateBlock: (block: unknown, dir?: Vec3, cursor?: Vec3) => Promise<void>
  openBlock: (block: unknown, dir?: Vec3, cursor?: Vec3) => Promise<unknown>
  closeWindow: (window: unknown) => void
  equip: (item: unknown, dest?: string) => Promise<void>
  swingArm: (hand?: string) => void

  // ── test controls ────────────────────────────────────────────────────
  set: (pos: Vec3, name: string, boundingBox?: string) => void
  /** Positions bot.dig was called with. */
  digs: Vec3[]
  /** Reference blocks + faces placeBlock was called with. */
  places: Array<{ ref: Vec3, face: Vec3, sneaking: boolean }>
  /** Sneak state observed at each placeBlock call. */
  activations: Vec3[]
  /** Make dig resolve without actually clearing the block. */
  digNoOp: boolean
  /** Make dig reject. */
  digThrows: Error | null
  /** ms bot.dig takes. */
  digDelay: number
  /** Make openBlock never resolve (a server that silently drops the click). */
  openSilent: boolean
  /** Make placeBlock reject even though the placement lands. */
  placeThrowsButWorks: boolean
  /** Make placeBlock do nothing at all. */
  placeNoOp: boolean
  /** Cursor override: what blockAtCursor reports (null = derive from lookAt). */
  cursorHit: { pos: Vec3, face: number } | null
}

const AIR: FakeBlock = {
  type: 0, name: 'air', position: new Vec3(0, 0, 0), boundingBox: 'empty'
}

let nextType = 1
const typeOf = new Map<string, number>()
function typeFor (name: string): number {
  let t = typeOf.get(name)
  if (t === undefined) { t = nextType++; typeOf.set(name, t) }
  return t
}

export function makeActionBot (start = new Vec3(0.5, 64, 0.5)): ActionBot {
  const bot = new EventEmitter() as ActionBot
  const world = new Map<string, string>()
  const key = (x: number, y: number, z: number): string => `${x},${y},${z}`

  bot.entity = { position: start.clone(), velocity: new Vec3(0, 0, 0), onGround: true, height: 1.8, yaw: 0 }
  bot.inventory = { items: () => items, emptySlotCount: () => 9 }
  bot.heldItem = null
  bot.currentWindow = null
  bot.controlState = {}
  bot.digs = []
  bot.places = []
  bot.activations = []
  bot.digNoOp = false
  bot.digThrows = null
  bot.digDelay = 5
  bot.openSilent = false
  bot.placeThrowsButWorks = false
  bot.placeNoOp = false
  bot.cursorHit = null

  let items: FakeItem[] = []
  ;(bot as unknown as { setItems: (i: FakeItem[]) => void }).setItems = (i: FakeItem[]) => { items = i }

  bot.set = (pos: Vec3, name: string, boundingBox?: string) => {
    if (name === 'air') world.delete(key(pos.x, pos.y, pos.z))
    else world.set(key(pos.x, pos.y, pos.z), `${name}|${boundingBox ?? 'block'}`)
  }

  bot.blockAt = (pos: Vec3) => {
    const p = pos.floored()
    const raw = world.get(key(p.x, p.y, p.z))
    if (!raw) return { ...AIR, position: p }
    const [name, boundingBox] = raw.split('|')
    return {
      type: typeFor(name),
      name,
      position: p,
      boundingBox,
      shapes: boundingBox === 'block' ? [[0, 0, 0, 1, 1, 1]] : [],
      digTime: () => 250
    }
  }

  // Reports whatever the test pinned, or the block the bot last looked at.
  let lastLook: Vec3 | null = null
  bot.blockAtCursor = () => {
    if (bot.cursorHit) {
      const b = bot.blockAt(bot.cursorHit.pos)
      return b ? { ...b, face: bot.cursorHit.face } : null
    }
    if (!lastLook) return null
    const b = bot.blockAt(lastLook)
    if (!b || b.name === 'air') return null
    return { ...b, face: faceFromEye(bot.entity.position.offset(0, 1.62, 0), lastLook) }
  }

  bot.world = {
    raycast: (from: Vec3, dir: Vec3, range: number) => {
      // March in small steps; report the first solid cell.
      for (let t = 0.05; t <= range; t += 0.05) {
        const p = from.plus(dir.scaled(t)).floored()
        const b = bot.blockAt(p)
        if (b && b.boundingBox === 'block') return { position: p, face: 1 }
      }
      return null
    }
  }

  bot.setControlState = (name, value) => { bot.controlState[name] = value }
  bot.clearControlStates = () => { bot.controlState = {} }
  bot.look = async (yaw) => { bot.entity.yaw = yaw }
  bot.lookAt = async (pos) => { lastLook = pos.floored() }
  bot.waitForTicks = async (ticks: number) => {
    await new Promise(resolve => setTimeout(resolve, Math.min(ticks, 2)))
  }
  bot.swingArm = () => {}

  bot.dig = async (block: unknown) => {
    const b = block as { position: Vec3 }
    bot.digs.push(b.position.clone())
    await new Promise(resolve => setTimeout(resolve, bot.digDelay))
    if (bot.digThrows) throw bot.digThrows
    if (bot.digNoOp) return
    bot.set(b.position, 'air')
    bot.emit('diggingCompleted', bot.blockAt(b.position))
  }
  bot.stopDigging = () => {}

  bot.placeBlock = async (ref: unknown, face: Vec3) => {
    const r = ref as { position: Vec3 }
    bot.places.push({
      ref: r.position.clone(), face: face.clone(), sneaking: bot.controlState.sneak === true
    })
    await new Promise(resolve => setTimeout(resolve, 2))
    if (!bot.placeNoOp) {
      const target = r.position.plus(face)
      bot.set(target, bot.heldItem?.name ?? 'stone')
    }
    if (bot.placeThrowsButWorks) throw new Error('Event blockUpdate did not fire within timeout')
  }

  bot.activateBlock = async (block: unknown) => {
    const b = block as { position: Vec3 }
    bot.activations.push(b.position.clone())
  }

  bot.openBlock = async (block: unknown) => {
    const b = block as { position: Vec3, name: string }
    bot.activations.push(b.position.clone())
    if (bot.openSilent) {
      // Exactly the shape of the bug: register the listener, never resolve.
      return await new Promise(() => { bot.once('windowOpen', () => {}) })
    }
    await new Promise(resolve => setTimeout(resolve, 2))
    const window = Object.assign(new EventEmitter(), {
      id: 1,
      type: windowTypeFor(b.name),
      slots: [],
      close: () => { bot.currentWindow = null }
    })
    bot.currentWindow = window
    bot.emit('windowOpen', window)
    return window
  }

  bot.closeWindow = () => { bot.currentWindow = null }
  bot.equip = async (item: unknown) => { bot.heldItem = item as FakeItem }

  return bot
}

export function setItems (bot: ActionBot, names: string[]): void {
  ;(bot as unknown as { setItems: (i: FakeItem[]) => void })
    .setItems(names.map(name => ({ type: typeFor(name), name })))
}

function windowTypeFor (name: string): string {
  if (name === 'crafting_table') return 'minecraft:crafting'
  if (name === 'enchanting_table') return 'minecraft:enchantment'
  if (name === 'furnace') return 'minecraft:furnace'
  return 'minecraft:generic_9x3'
}

/** Which face of `pos` an eye at `eye` is looking at, crudely. */
function faceFromEye (eye: Vec3, pos: Vec3): number {
  const c = new Vec3(pos.x + 0.5, pos.y + 0.5, pos.z + 0.5)
  const d = eye.minus(c)
  const ax = Math.abs(d.x)
  const ay = Math.abs(d.y)
  const az = Math.abs(d.z)
  if (ay >= ax && ay >= az) return d.y > 0 ? 1 : 0
  if (ax >= az) return d.x > 0 ? 5 : 4
  return d.z > 0 ? 3 : 2
}
