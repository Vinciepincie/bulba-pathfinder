// Fall recovery: the damage model, item choice, landing prediction, clutch windows, and the guard
// end to end on a kernel-driven bot with a small server model (fall distance per move packet, water
// resets it, damage on landing; buckets ray-cast 4.5 from the last position sent).
import { expect } from 'chai'
import { EventEmitter } from 'node:events'
import { Vec3 } from 'vec3'
import prismarineItem from 'prismarine-item'
import { landingPoints, healthLoss, healthAfter, readFallBody, attributeValue, enchantsOf, DEFAULT_BODY } from '../src/fall/model.js'
import { usableBreakers, defaultFallItems, handFor } from '../src/fall/items.js'
import type { FallItemRule } from '../src/fall/items.js'
import { FallGuard } from '../src/fall/guard.js'
import type { FallInfo, FallOutcome, FallRecoveryOptions, GuardHost } from '../src/fall/guard.js'
import { PlayerSim, newBody, simKindOf, simTablesFromLut } from '../src/playerSim.js'
import type { SimWorld } from '../src/playerSim.js'
import { VoxelWorld, makeFakeBot, makeOurMovements, lutFor, mcData, Block, AIR, STONE, WATER, COBWEB } from './helpers/voxelWorld.js'

const Item = prismarineItem(mcData.version.minecraftVersion as string) as unknown as new (type: number, count: number) => { name: string, type: number, count: number, slot: number, customName?: string | null }
const id = (name: string): number => (mcData.itemsByName as Record<string, { id: number }>)[name].id
const HAY = (mcData.blocksByName.hay_block as { minStateId: number }).minStateId + 1
/** Leaves that are not waterlogged (the helper's OAK_LEAVES is the waterlogged state). */
const DRY_LEAVES = ((): number => {
  const b = mcData.blocksByName.oak_leaves as { minStateId: number, maxStateId: number }
  for (let s = b.minStateId; s <= b.maxStateId; s++) {
    const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    if (p.waterlogged === false || p.waterlogged === 'false') return s
  }
  throw new Error('no dry leaves')
})()
const CHEST = (mcData.blocksByName.chest as { minStateId: number }).minStateId
const on = (name: string): number => simKindOf(name, false)

describe('fall model', () => {
  it('rounds the whole fall down and scales it by the landing block (1.21.5+)', () => {
    expect(landingPoints(23.5, 2, on('stone'), false, true, DEFAULT_BODY)).to.equal(20)
    expect(landingPoints(30, 2, on('hay_block'), false, true, DEFAULT_BODY)).to.equal(5)
    expect(landingPoints(30, 2, on('red_bed'), false, true, DEFAULT_BODY)).to.equal(12)
    expect(landingPoints(60, 3, on('slime_block'), false, true, DEFAULT_BODY)).to.equal(0)
    // Sneaking onto slime takes the whole fall.
    expect(landingPoints(20, 3, on('slime_block'), true, true, DEFAULT_BODY)).to.equal(17)
  })

  it('reads safe_fall_distance and fall_damage_multiplier, and Slow Falling cancels the fall', () => {
    expect(landingPoints(10, 1, 0, false, true, { ...DEFAULT_BODY, safeFall: 5 })).to.equal(5)
    expect(landingPoints(10, 1, 0, false, true, { ...DEFAULT_BODY, multiplier: 0.5 })).to.equal(3)
    // One rounding, after the multiplier: 3.8 blocks at x2 is floor(1.6) = 1, not floor(0.8) x 2 = 0.
    expect(landingPoints(3.8, 0.5, 0, false, true, { ...DEFAULT_BODY, multiplier: 2 })).to.equal(1)
    expect(landingPoints(40, 3, 0, false, true, { ...DEFAULT_BODY, slowFalling: true })).to.equal(0)
  })

  it('takes Resistance, then Feather Falling and Protection (capped at 80%), then absorption', () => {
    expect(healthLoss(10, { ...DEFAULT_BODY, resistance: 2 })).to.be.closeTo(6, 1e-9)
    expect(healthLoss(10, { ...DEFAULT_BODY, epf: 12 })).to.be.closeTo(5.2, 1e-9)
    expect(healthLoss(10, { ...DEFAULT_BODY, epf: 40 })).to.be.closeTo(2, 1e-9)
    expect(healthLoss(10, { ...DEFAULT_BODY, absorption: 4 })).to.be.closeTo(6, 1e-9)
    expect(healthAfter(25, { ...DEFAULT_BODY, totemInHand: true })).to.equal(1)
    expect(healthAfter(25, DEFAULT_BODY)).to.equal(-5)
  })

  it('applies attribute modifiers in vanilla order', () => {
    expect(attributeValue({ value: 3, modifiers: [{ amount: 2, operation: 0 }] })).to.equal(5)
    expect(attributeValue({ value: 2, modifiers: [{ amount: 1, operation: 0 }, { amount: 0.5, operation: 1 }, { amount: 1, operation: 2 }] })).to.equal(9)
  })

  it('reads enchantments from NBT lists and from 1.20.5+ components', () => {
    const reg = { enchantments: { 3: { name: 'feather_falling' } } }
    expect(enchantsOf({ enchants: [{ name: 'feather_falling', lvl: 4 }] } as never)).to.deep.equal([{ name: 'feather_falling', lvl: 4 }])
    expect(enchantsOf({ enchants: { enchantments: [{ id: 3, level: 2 }] } } as never, reg)).to.deep.equal([{ name: 'feather_falling', lvl: 2 }])
  })

  it('reads the body: attributes, effects, boots and a totem in hand', () => {
    const slots: unknown[] = new Array(46).fill(null)
    slots[8] = { name: 'diamond_boots', enchants: [{ name: 'feather_falling', lvl: 4 }, { name: 'protection', lvl: 4 }] }
    slots[45] = { name: 'totem_of_undying' }
    const b = readFallBody({
      health: 12,
      registry: { effects: { 11: { name: 'resistance' }, 8: { name: 'JumpBoost' } } },
      entity: { effects: { 11: { id: 11, amplifier: 0 }, 8: { id: 8, amplifier: 1 } }, attributes: {} },
      inventory: { slots }
    })
    expect(b.health).to.equal(12)
    expect(b.resistance).to.equal(1)
    expect(b.epf).to.equal(16)
    expect(b.safeFall).to.equal(5) // Jump Boost II without a synced attribute
    expect(b.totemInHand).to.equal(true)
  })
})

function invWith (items: Array<[number, string, number?]>): { inventory: { slots: unknown[] }, quickBarSlot: number, currentWindow: null } {
  const slots: unknown[] = new Array(46).fill(null)
  for (const [slot, name, count] of items) {
    const it = new Item(id(name), count ?? 1)
    it.slot = slot
    slots[slot] = it
  }
  return { inventory: { slots }, quickBarSlot: 0, currentWindow: null }
}

describe('fall world view', () => {
  it('keeps every cell apart, far from the origin and below zero', async () => {
    const { FallWorld } = await import('../src/fall/predict.js')
    const code = (x: number, y: number, z: number): number => ((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) & 0xfff
    const w = new FallWorld(code)
    for (const [ax, az] of [[-660, -664], [29_999_000, -29_999_000], [0, 0]]) {
      w.beginTick(ax, az)
      for (let dx = -40; dx <= 40; dx += 7) {
        for (let y = -64; y <= 319; y += 13) {
          for (let dz = -40; dz <= 40; dz += 9) expect(w.stateAt(ax + dx, y, az + dz)).to.equal(code(ax + dx, y, az + dz))
        }
      }
    }
  })
})

describe('fall items', () => {
  it('finds every default breaker and its kind', () => {
    const bot = invWith([[36, 'water_bucket'], [12, 'cod_bucket'], [40, 'cobweb', 4], [20, 'hay_block', 2], [30, 'stone', 64]])
    const refs = usableBreakers(bot as never, defaultFallItems)
    expect(refs.map(r => `${r.item.name}:${r.kind}`).sort()).to.deep.equal(['cobweb:cobweb', 'cod_bucket:water', 'hay_block:cushion', 'water_bucket:water'])
    expect(refs.find(r => r.item.name === 'water_bucket')?.cost).to.equal(0.5)
  })

  it('takes custom rules by predicate, and only the items an inventory function allows', () => {
    const bot = invWith([[36, 'water_bucket'], [37, 'water_bucket'], [38, 'stone']])
    ;(bot.inventory.slots[38] as { customName: string }).customName = 'clutch rock'
    const rules: FallItemRule[] = [...defaultFallItems, { kind: 'cushion', match: i => (i as { customName?: string }).customName === 'clutch rock', cost: 0 }]
    expect(usableBreakers(bot as never, rules).map(r => r.slot)).to.deep.equal([36, 37, 38])
    // A shop that owns only the second bucket.
    const owned = (b: never): unknown[] => [(b as { inventory: { slots: unknown[] } }).inventory.slots[37]]
    expect(usableBreakers(bot as never, rules, owned as never).map(r => r.slot)).to.deep.equal([37])
    // A throwing inventory function means nothing is usable, not a crash.
    expect(usableBreakers(bot as never, rules, (() => { throw new Error('x') }) as never)).to.deep.equal([])
  })

  it('with a container open, uses only the hotbar and offhand', () => {
    const bot = { ...invWith([[12, 'water_bucket'], [45, 'cobweb']]), currentWindow: {} }
    expect(usableBreakers(bot as never, defaultFallItems).map(r => r.slot)).to.deep.equal([45])
  })

  it('maps slots to hands', () => {
    expect(handFor(45)).to.deep.equal({ hand: 1, hotbar: null })
    expect(handFor(40)).to.deep.equal({ hand: 0, hotbar: 4 })
    expect(handFor(12)).to.deep.equal({ hand: 0, hotbar: null })
  })
})

// ---- a kernel-driven bot and a small server ----

interface Packet { tick: number, name: string, data: unknown }

interface Rig {
  bot: Record<string, any>
  guard: FallGuard
  options: FallRecoveryOptions
  packets: Packet[]
  events: Array<[string, unknown]>
  run: (ticks: number) => void
  untilSettled: (max?: number) => void
  /** Move the body and give it a knock, as the server would. */
  push: (pos: Vec3, vel: Vec3) => void
  server: { fd: number, health: number }
  world: VoxelWorld
  /** How often the guard took the controls (host.takeOver). */
  takeOvers: () => number
}

function rig (world: VoxelWorld, start: Vec3, opts: {
  items?: Array<[number, string, number?]>
  mode?: FallRecoveryOptions['mode']
  velocity?: Vec3
  knock?: boolean
  dimension?: string
  flight?: GuardHost['flight']
  options?: Partial<FallRecoveryOptions>
} = {}): Rig {
  const fake = makeFakeBot(world, start.clone()) as unknown as Record<string, any>
  const bot = Object.assign(new EventEmitter(), fake) as Record<string, any>
  const packets: Packet[] = []
  const events: Array<[string, unknown]> = []
  let tick = 0
  let takeOvers = 0
  const server = { fd: 0, health: 20 }
  let sent = start.clone()
  bot.entity = { position: start.clone(), velocity: (opts.velocity ?? new Vec3(0, 0, 0)).clone(), onGround: false, isInWater: false, yaw: 0, pitch: 0, effects: {}, height: 1.8, attributes: {} }
  bot.health = 20
  bot.food = 20
  bot.game = { ...bot.game, dimension: opts.dimension ?? 'overworld' }
  Object.assign(bot, invWith(opts.items ?? []))
  bot.controlState = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false }
  bot.setControlState = (k: string, v: boolean) => { bot.controlState[k] = v }
  bot.look = (yaw: number, pitch: number) => { bot.entity.yaw = yaw; bot.entity.pitch = pitch }
  bot.setQuickBarSlot = (i: number) => { bot.quickBarSlot = i; packets.push({ tick, name: 'held_item_slot', data: i }) }
  bot.clickWindow = async (slot: number, button: number, mode: number) => {
    packets.push({ tick, name: 'window_click', data: { slot, button, mode } })
    const s = bot.inventory.slots
    const a = s[slot]; const b = s[36 + button]
    s[slot] = b; s[36 + button] = a
    if (a !== null) a.slot = 36 + button
    if (b !== null) b.slot = slot
  }
  bot.physics = { playerHalfWidth: 0.3, playerHeight: 1.8 }
  bot.swingArm = () => {}
  bot._updateBlockState = (p: Vec3, s: number) => world.set(p.x, p.y, p.z, s)
  const held = (hand: number): any => hand === 1 ? bot.inventory.slots[45] : bot.inventory.slots[36 + bot.quickBarSlot]
  const solid = (x: number, y: number, z: number): boolean => {
    const s = world.stateAt(x, y, z)
    return s !== null && s !== AIR && s !== WATER && s !== COBWEB
  }
  bot.activateItem = (off = false) => {
    const hand = off ? 1 : 0
    const item = held(hand)
    packets.push({ tick, name: 'use_item', data: { hand, yaw: bot.entity.yaw, pitch: bot.entity.pitch, from: sent.clone(), item: item?.name } })
    if (item === null || item === undefined) return
    // The server's ray from the eye it last saw, along the packet rotation, 4.5 long.
    const eye = sent.offset(0, 1.62, 0)
    const yaw = bot.entity.yaw; const pitch = bot.entity.pitch
    const dir = new Vec3(-Math.sin(yaw) * Math.cos(pitch), Math.sin(pitch), -Math.cos(yaw) * Math.cos(pitch))
    let prev = eye.floored()
    for (let t = 0; t <= 4.5; t += 0.005) {
      const p = eye.plus(dir.scaled(t))
      const c = p.floored()
      if (item.name === 'bucket') {
        if (world.stateAt(c.x, c.y, c.z) === WATER) {
          world.set(c.x, c.y, c.z, AIR)
          item.name = 'water_bucket'; item.type = id('water_bucket')
          return
        }
        continue
      }
      if (solid(c.x, c.y, c.z)) {
        if (item.name.endsWith('_bucket') && !String(bot.game.dimension).includes('nether') && world.stateAt(prev.x, prev.y, prev.z) === AIR) {
          world.set(prev.x, prev.y, prev.z, WATER)
        }
        if (item.name.endsWith('_bucket')) { item.name = 'bucket'; item.type = id('bucket') }
        return
      }
      prev = c
    }
  }
  bot._client = {
    write: (name: string, data: any) => {
      packets.push({ tick, name, data })
      if (name !== 'block_place') return
      const item = held(data.hand)
      const loc = data.location
      const eye = sent.offset(0, 1.62, 0)
      const dx = Math.max(loc.x - eye.x, 0, eye.x - (loc.x + 1))
      const dy = Math.max(loc.y - eye.y, 0, eye.y - (loc.y + 1))
      const dz = Math.max(loc.z - eye.z, 0, eye.z - (loc.z + 1))
      if (Math.hypot(dx, dy, dz) >= 5.5 || item === null) return
      const cell = { x: loc.x, y: loc.y + 1, z: loc.z }
      const state = item.name === 'cobweb' ? COBWEB : HAY
      if (state === HAY && sent.y < cell.y + 1 && sent.y + 1.8 > cell.y) return
      world.set(cell.x, cell.y, cell.z, state)
      item.count--
      if (item.count === 0) bot.inventory.slots[item.slot] = null
    }
  }
  const options: FallRecoveryOptions = { mode: opts.mode ?? 'recover', items: defaultFallItems, pickUpWater: true, equipTotem: true, ...opts.options }
  const movements = makeOurMovements(fake, {})
  const tables = simTablesFromLut(lutFor(fake, movements) as never) as Omit<SimWorld, 'stateAt'>
  const guard = new FallGuard({
    bot: bot as never,
    options,
    tables: () => tables,
    wholeFall: () => true,
    minHealth: () => 0,
    flight: opts.flight ?? (() => ({ driving: false, floorY: null, clutch: null })),
    userHolds: () => false,
    takeOver: () => { takeOvers++ },
    released: () => {},
    warn: () => {}
  })
  for (const ev of ['pathfinder:fall_start', 'pathfinder:fall_lethal', 'pathfinder:fall_finish']) bot.on(ev, (p: unknown) => events.push([ev, p]))
  // The client: the kernel, and water or cobweb stop the body (no swimming model).
  const sim = new PlayerSim({ ...tables, stateAt: (x, y, z) => world.stateAt(x, y, z) ?? -1 }, 0.3, 1.8)
  const body = newBody(start.x, start.y, start.z)
  body.vx = bot.entity.velocity.x; body.vy = bot.entity.velocity.y; body.vz = bot.entity.velocity.z
  if (opts.knock === true) guard.onVelocity(body.vx, body.vy, body.vz)
  const wetOrWeb = (x: number, y: number, z: number): boolean => {
    for (let cy = Math.floor(y); cy <= Math.floor(y + 1.79); cy++) {
      for (const [ox, oz] of [[-0.29, -0.29], [0.29, -0.29], [-0.29, 0.29], [0.29, 0.29]]) {
        const s = world.stateAt(Math.floor(x + ox), cy, Math.floor(z + oz))
        if (s === WATER || s === COBWEB) return true
      }
    }
    return false
  }
  const stepOnce = (): void => {
    tick++
    const e = bot.entity
    if (!e.isInWater) {
      const c = bot.controlState
      sim.step(body, { ...c, hx: -Math.sin(e.yaw), hz: -Math.cos(e.yaw) })
      if (body.inLiquid) { body.vx = 0; body.vy = 0; body.vz = 0; e.isInWater = wetOrWeb(body.x, body.y, body.z); body.inLiquid = false }
      e.position.set(body.x, body.y, body.z)
      e.velocity.set(body.vx, body.vy, body.vz)
      e.onGround = body.onGround
    }
    if (wetOrWeb(body.x, body.y, body.z)) e.isInWater = true
    bot.emit('physicsTick')
    guard.tick()
    // The server handles this tick's movement packet.
    const prevY = sent.y
    const moved = e.position.distanceTo(sent)
    const ray = moved >= 1 && (() => { for (let t = 0; t <= 1; t += 0.05) { const p = sent.plus(e.position.minus(sent).scaled(t)); if (wetOrWeb(p.x, p.y, p.z)) return true } return false })()
    sent = e.position.clone()
    if (wetOrWeb(sent.x, sent.y, sent.z) || ray) server.fd = 0
    else if (sent.y < prevY) server.fd += prevY - sent.y
    if (e.onGround && server.fd > 0) {
      const under = world.stateAt(Math.floor(sent.x), Math.floor(sent.y - 0.2), Math.floor(sent.z)) ?? AIR
      const name = (mcData.blocksByStateId as Record<number, { name: string }>)[under]?.name ?? 'air'
      const dmg = landingPoints(server.fd, 0, simKindOf(name, false), false, true, DEFAULT_BODY)
      if (process.env.FALL_DEBUG === '1') console.log(`  [server] tick ${tick} landing fd ${server.fd.toFixed(2)} on ${name} at ${sent.toString()} dmg ${dmg}`)
      server.health -= dmg
      bot.health = server.health
      server.fd = 0
    }
    bot.emit('move')
    guard.onMove()
  }
  return {
    bot, guard, options, packets, events, server, world,
    takeOvers: () => takeOvers,
    run: (n) => { for (let i = 0; i < n; i++) stepOnce() },
    push: (pos, vel) => {
      body.x = pos.x; body.y = pos.y; body.z = pos.z
      body.vx = vel.x; body.vy = vel.y; body.vz = vel.z
      body.onGround = false; body.inLiquid = false
      bot.entity.position.set(pos.x, pos.y, pos.z)
      bot.entity.velocity.set(vel.x, vel.y, vel.z)
      bot.entity.onGround = false
      bot.entity.isInWater = false
      sent = pos.clone()
      guard.onVelocity(vel.x, vel.y, vel.z)
    },
    untilSettled: (max = 400) => {
      for (let i = 0; i < max; i++) {
        stepOnce()
        if (!guard.active() && (bot.entity.onGround === true || bot.entity.isInWater === true) && i > 5) return
      }
    }
  }
}

/** Open air above a stone floor at y = 0, from x -12..12. */
function plain (height: number): VoxelWorld {
  const w = new VoxelWorld({ x0: -12, y0: -2, z0: -12, x1: 12, y1: height + 8, z1: 12 })
  w.fill(-12, 0, -12, 12, 0, 12, STONE)
  return w
}

const names = (r: Rig): string[] => r.events.map(e => e[0])

describe('fall guard', function () {
  this.timeout(20000)

  it('observe mode: reports a damaging knock-off once, writes nothing, and the fall costs what it predicted', () => {
    const r = rig(plain(30), new Vec3(0.5, 21, 0.5), { mode: 'observe', velocity: new Vec3(0.3, 0.3, 0), knock: true, items: [[36, 'water_bucket']] })
    r.untilSettled()
    expect(names(r)).to.deep.equal(['pathfinder:fall_start', 'pathfinder:fall_finish'])
    const info = r.events[0][1] as FallInfo
    expect(info.cause).to.equal('knockback')
    expect(info.executed).to.equal(false)
    expect(info.plan.kind).to.equal('clutch') // what recover mode would do
    expect(r.packets).to.deep.equal([])
    expect(Object.values(r.bot.controlState).some(v => v === true)).to.equal(false)
    const out = r.events[1][1] as FallOutcome
    expect(out.damage).to.equal(info.unaided.loss)
    expect(r.server.health).to.equal(20 - info.unaided.loss)
  })

  it('recover mode: places water from the hotbar on a tick the server can reach, takes no damage, takes the water back', () => {
    const r = rig(plain(30), new Vec3(0.5, 25, 0.5), { velocity: new Vec3(0, 0, 0), knock: true, items: [[39, 'water_bucket']] })
    r.untilSettled()
    expect(r.server.health).to.equal(20)
    const uses = r.packets.filter(p => p.name === 'use_item')
    expect(uses).to.have.length(2) // place, then pick up
    const place = uses[0].data as { from: Vec3, pitch: number }
    expect(place.from.y + 1.62 - 1).to.be.at.most(4.5) // the eye the server judged from reaches the floor's top
    expect(place.pitch).to.be.below(-1.2) // looking down
    expect(r.world.stateAt(0, 1, 0)).to.equal(AIR) // the water came back
    expect(r.bot.inventory.slots[39].name).to.equal('water_bucket')
    const out = r.events.find(e => e[0] === 'pathfinder:fall_finish')?.[1] as FallOutcome
    expect(out.clutched).to.equal(true)
    expect(out.pickedUp).to.equal(true)
    expect(r.bot.quickBarSlot).to.equal(0) // the held slot is put back
  })

  it('stages a bucket from the main inventory with one number-key click during the fall', () => {
    const r = rig(plain(30), new Vec3(0.5, 25, 0.5), { knock: true, items: [[14, 'water_bucket']] })
    r.untilSettled()
    expect(r.server.health).to.equal(20)
    const click = r.packets.find(p => p.name === 'window_click')
    const use = r.packets.find(p => p.name === 'use_item')
    expect(click).to.not.equal(undefined)
    expect(click!.tick).to.be.below(use!.tick)
  })

  it('falls back on hay: lands on it and keeps a fifth of the damage', () => {
    const r = rig(plain(40), new Vec3(0.5, 31, 0.5), { knock: true, items: [[36, 'hay_block', 3]] })
    r.untilSettled()
    expect(r.packets.some(p => p.name === 'block_place')).to.equal(true)
    expect(r.world.stateAt(0, 1, 0)).to.equal(HAY)
    // 30 blocks, one less onto the bale: floor((29 - 3) * 0.2) = 5.
    expect(20 - r.server.health).to.be.within(4, 6)
  })

  it('uses cobweb in the Nether where water would boil away', () => {
    const r = rig(plain(30), new Vec3(0.5, 25, 0.5), { knock: true, dimension: 'minecraft:the_nether', items: [[36, 'water_bucket'], [37, 'cobweb', 2]] })
    r.untilSettled()
    expect(r.packets.some(p => p.name === 'use_item')).to.equal(false)
    expect(r.packets.some(p => p.name === 'block_place')).to.equal(true)
    expect(r.server.health).to.equal(20)
  })

  it('does not pour water on leaves (it would soak in), but will on a hay bale it placed there first', () => {
    const w = plain(30)
    w.fill(-12, 1, -12, 12, 1, 12, DRY_LEAVES)
    const only = rig(w, new Vec3(0.5, 26, 0.5), { knock: true, items: [[36, 'water_bucket']] })
    only.untilSettled()
    expect(only.packets.some(p => p.name === 'use_item')).to.equal(false)
    // With hay too: the bale goes on the leaves, then water on the bale saves the fifth that hay keeps.
    const w2 = plain(30)
    w2.fill(-12, 1, -12, 12, 1, 12, DRY_LEAVES)
    const both = rig(w2, new Vec3(0.5, 26, 0.5), { knock: true, items: [[36, 'water_bucket'], [37, 'hay_block', 2]] })
    both.untilSettled()
    const place = both.packets.findIndex(p => p.name === 'block_place')
    const use = both.packets.findIndex(p => p.name === 'use_item')
    expect(place).to.be.at.least(0)
    expect(use).to.be.above(place)
    expect(both.server.health).to.equal(20)
    // The pickup goes to the bucket's slot, not the hay's.
    expect(both.bot.inventory.slots[36].name).to.equal('water_bucket')
  })

  it('never clicks a chest to place against it', () => {
    const w = plain(30)
    w.fill(-12, 0, -12, 12, 0, 12, CHEST)
    const r = rig(w, new Vec3(0.5, 25, 0.5), { knock: true, items: [[36, 'hay_block', 2]] })
    r.untilSettled()
    expect(r.packets.some(p => p.name === 'block_place')).to.equal(false)
  })

  it('steers into a pool it can reach and lands with no damage and no item', () => {
    const w = plain(40)
    w.fill(4, 1, -1, 6, 1, 1, WATER)
    const r = rig(w, new Vec3(0.5, 36, 0.5), { knock: true })
    r.untilSettled()
    expect(r.server.health).to.equal(20)
    expect(r.bot.entity.isInWater).to.equal(true)
    const start = r.events.find(e => e[0] === 'pathfinder:fall_start')?.[1] as FallInfo
    expect(start.plan.kind).to.equal('steer')
    expect(start.plan.landing.medium).to.equal('water')
  })

  it('reports a lethal fall early, and holds a totem from the hotbar', () => {
    const r = rig(plain(60), new Vec3(0.5, 55, 0.5), { knock: true, items: [[42, 'totem_of_undying']] })
    r.run(3)
    const lethal = r.events.find(e => e[0] === 'pathfinder:fall_lethal')?.[1] as FallInfo | undefined
    expect(lethal).to.not.equal(undefined)
    expect(lethal!.ticksToImpact).to.be.above(20)
    expect(r.bot.quickBarSlot).to.equal(6)
  })

  it('stays quiet for a fall that does not hurt, and for flights the executor plans', () => {
    const r = rig(plain(10), new Vec3(0.5, 3.5, 0.5), { knock: true })
    r.untilSettled()
    expect(r.events).to.deep.equal([])
    // A planned 6-block drop (it hurts a little): the executor's own flight, nothing to report.
    const p = rig(plain(10), new Vec3(0.5, 7, 0.5), { flight: () => ({ driving: true, floorY: 1, clutch: null }) })
    p.untilSettled()
    expect(p.events).to.deep.equal([])
  })

  it('a planned flight that ends up far below its path is a fall again', () => {
    const p = rig(plain(30), new Vec3(0.5, 25, 0.5), { mode: 'observe', flight: () => ({ driving: true, floorY: 22, clutch: null }) })
    p.untilSettled()
    expect((p.events[0]?.[1] as FallInfo | undefined)?.cause).to.equal('missed_landing')
  })

  it('switching to observe mid-fall hands the controls back and uses nothing', () => {
    const r = rig(plain(40), new Vec3(0.5, 36, 0.5), { knock: true, items: [[36, 'water_bucket']] })
    r.run(3)
    expect(r.guard.owns()).to.equal(true)
    r.options.mode = 'observe'
    r.untilSettled()
    expect(r.packets.some(p => p.name === 'use_item')).to.equal(false)
    expect(Object.values(r.bot.controlState).some(v => v === true)).to.equal(false)
    expect(r.server.health).to.be.below(20)
  })

  it('a second knock while taking the water back is a new fall', () => {
    const w = plain(40)
    w.fill(-1, 1, -1, 1, 20, 1, STONE) // a 3x3 tower, top stand y 21
    const r = rig(w, new Vec3(0.5, 30, 0.5), { knock: true, items: [[36, 'water_bucket'], [37, 'water_bucket']] })
    for (let i = 0; i < 400 && !(r.bot.entity.onGround === true || r.bot.entity.isInWater === true); i++) r.run(1)
    // Pushed off the tower edge right after landing on it.
    r.push(new Vec3(2.4, 21, 0.5), new Vec3(0.3, 0.2, 0))
    r.untilSettled()
    const starts = r.events.filter(e => e[0] === 'pathfinder:fall_start')
    expect(starts.length).to.be.at.least(1)
    expect(r.server.health).to.equal(20)
  })

  it('swaps a totem from the main inventory into the hand when nothing else helps, and only one it may use', () => {
    const r = rig(plain(60), new Vec3(0.5, 55, 0.5), { knock: true, items: [[20, 'totem_of_undying']] })
    r.run(3)
    expect(r.packets.some(p => p.name === 'window_click')).to.equal(true)
    const shop = rig(plain(60), new Vec3(0.5, 55, 0.5), { knock: true, items: [[20, 'totem_of_undying']], options: { inventory: () => [] } })
    shop.run(3)
    expect(shop.packets.some(p => p.name === 'window_click')).to.equal(false)
  })

  it('a harmless knock never takes the controls', () => {
    const r = rig(plain(10), new Vec3(0.5, 4, 0.5), { knock: true, velocity: new Vec3(0.3, 0.3, 0), items: [[36, 'water_bucket']] })
    r.untilSettled()
    expect(r.takeOvers()).to.equal(0)
    expect(r.events).to.deep.equal([])
  })

  it('a server correction mid-fall keeps the fall it has counted, and does not make the landing look lethal', () => {
    const r = rig(plain(30), new Vec3(0.5, 21, 0.5), { mode: 'observe', knock: true })
    r.run(10)
    const before = r.guard.fallDistance
    r.push(r.bot.entity.position.offset(0, 0.5, 0), r.bot.entity.velocity.clone())
    r.guard.onForcedMove()
    r.run(1)
    expect(r.guard.fallDistance).to.be.at.least(before)
    r.untilSettled()
    expect(r.events.some(e => e[0] === 'pathfinder:fall_lethal')).to.equal(false)
  })

  it('flies a planned clutch drop itself: water under the real landing, the bucket back, and no fall events', () => {
    const r = rig(plain(30), new Vec3(0.5, 21, 0.5), {
      velocity: new Vec3(0.2, 0, 0),
      items: [[36, 'water_bucket']],
      flight: () => ({ driving: true, floorY: 1, clutch: { x: 1, y: 0, z: 0 } })
    })
    r.untilSettled()
    expect(r.server.health).to.equal(20)
    expect(r.events).to.deep.equal([])
    expect(r.bot.inventory.slots[36].name).to.equal('water_bucket')
  })

  it('a planned drop that cannot place its water any more is an unplanned fall, with every breaker back in play', () => {
    const r = rig(plain(30), new Vec3(0.5, 21, 0.5), {
      dimension: 'minecraft:the_nether',
      velocity: new Vec3(0.2, 0, 0),
      items: [[36, 'water_bucket'], [37, 'cobweb', 2]],
      flight: () => ({ driving: true, floorY: 1, clutch: { x: 1, y: 0, z: 0 } })
    })
    r.untilSettled()
    const start = r.events.find(e => e[0] === 'pathfinder:fall_start')?.[1] as FallInfo | undefined
    expect(start?.cause).to.equal('missed_landing')
    expect(r.packets.some(p => p.name === 'block_place')).to.equal(true)
    expect(r.server.health).to.equal(20)
  })

  it('a choose hook can refuse the item', () => {
    const r = rig(plain(30), new Vec3(0.5, 25, 0.5), { knock: true, items: [[36, 'water_bucket']], options: { choose: () => null } })
    r.untilSettled()
    expect(r.packets.some(p => p.name === 'use_item')).to.equal(false)
    expect(r.server.health).to.be.below(20)
  })

  it('an item is only spent when it saves more health than it costs', () => {
    // A 5-block fall costs 2; hay (cost 3) is not worth it, a water bucket (0.5) is.
    const hay = rig(plain(10), new Vec3(0.5, 6, 0.5), { knock: true, items: [[36, 'hay_block']] })
    hay.untilSettled()
    expect(hay.packets.some(p => p.name === 'block_place')).to.equal(false)
    const water = rig(plain(10), new Vec3(0.5, 6, 0.5), { knock: true, items: [[36, 'water_bucket']] })
    water.untilSettled()
    expect(water.server.health).to.equal(20)
  })
})

describe('fall recovery through the plugin', () => {
  // Imported lazily so the guard tests above stay independent of the plugin.
  async function setup (world: VoxelWorld, start: Vec3): Promise<{ bot: Record<string, any>, pf: Record<string, any> }> {
    const { createPathfinder } = await import('../src/plugin.js')
    const { Movements } = await import('../src/movements.js')
    const { makeDriveableBot, makeFakePhysics } = await import('./helpers/fakeBot.js')
    const { applyProfile } = await import('./helpers/voxelWorld.js')
    const bot = makeDriveableBot(world, start) as unknown as Record<string, any>
    bot.loadPlugin(createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot as never) }) as never)
    const pf = bot.pathfinder
    pf.setMovements(applyProfile(new Movements(bot as never) as never) as never)
    return { bot, pf }
  }

  /** A ledge 12 up (stand y 14) over a stone floor (stand y 2), the goal down on the floor. */
  function ledge (): VoxelWorld {
    const w = new VoxelWorld({ x0: -4, y0: 0, z0: -4, x1: 8, y1: 20, z1: 4 })
    w.fill(1, 1, -4, 8, 1, 4, STONE)
    w.fill(-4, 13, -4, 0, 13, 4, STONE)
    return w
  }

  it('starts in observe mode with the default items, and keeps its options object', async () => {
    const { pf } = await setup(ledge(), new Vec3(0.5, 14, 0.5))
    expect(pf.fallRecovery.mode).to.equal('observe')
    expect(pf.fallRecovery.items).to.equal(defaultFallItems)
    expect(pf.fallRecovery.pickUpWater).to.equal(true)
    const same = pf.fallRecovery
    pf.fallRecovery.mode = 'recover'
    expect(pf.fallRecovery).to.equal(same)
  })

  it('plans a clutch drop only in recover mode, with a usable bucket, where water stays', async () => {
    const { bot, pf } = await setup(ledge(), new Vec3(0.5, 14, 0.5))
    pf.movements.maxClutchDropDown = 30
    const goal = new (await import('../src/goals.js')).GoalBlock(4, 2, 0)
    const solve = async (): Promise<{ status: string, path: Array<{ clutch?: boolean }> }> => await pf.computePath(goal)
    expect((await solve()).status).to.equal('noPath') // observe mode
    pf.fallRecovery.mode = 'recover'
    expect((await solve()).status).to.equal('noPath') // no bucket
    Object.assign(bot, invWith([[38, 'water_bucket']]))
    const ok = await solve()
    expect(ok.status).to.equal('success')
    expect(ok.path.some(n => n.clutch === true)).to.equal(true)
    // A shop that does not own the bucket.
    pf.fallRecovery.inventory = () => []
    expect((await solve()).status).to.equal('noPath')
    pf.fallRecovery.inventory = undefined
    bot.game.dimension = 'minecraft:the_nether'
    expect((await solve()).status).to.equal('noPath')
  })
})
