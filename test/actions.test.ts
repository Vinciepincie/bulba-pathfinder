// The interaction layer: the guards that turn "the bot just stood there"
// into an error a caller can act on, and the retries that make the common
// transient failures invisible.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createActionTable } from '../src/actions/index.js'
import { ActionErrors, DEFAULT_ACTION_CONFIG } from '../src/actions/types.js'
import type { ActionConfig, ActionTable } from '../src/actions/types.js'
import type { ActionContext } from '../src/actions/context.js'
import { Pacer } from '../src/actions/pacing.js'
import { reachToCell, eyePos, faceId, FACE_VECTORS } from '../src/actions/reach.js'
import { makeActionBot, setItems } from './helpers/actionBot.js'
import type { ActionBot } from './helpers/actionBot.js'

interface Rig {
  bot: ActionBot
  actions: ActionTable
  config: ActionConfig
  approaches: Array<{ pos: Vec3, reach: number }>
  /** What `ctx.approach` returns, and whether it teleports the bot into range. */
  approachResult: boolean
  forcedMoveAgo: number
}

function rig (start = new Vec3(0.5, 64, 0.5)): Rig {
  const bot = makeActionBot(start)
  // Pacing is a wall-clock delay; it has its own test.
  const config: ActionConfig = { ...DEFAULT_ACTION_CONFIG, pacing: 'none', retries: 3 }
  const r: Partial<Rig> = { bot, config, approaches: [], approachResult: true, forcedMoveAgo: Infinity }

  const ctx: ActionContext = {
    bot: bot as never,
    config,
    pacer: new Pacer(),
    rawDig: (block, forceLook, face) => bot.dig(block, forceLook, face),
    msSinceForcedMove: () => r.forcedMoveAgo as number,
    serverTps: () => 20,
    bestHarvestTool: () => bot.inventory.items()[0] ?? null,
    approach: async (pos, reach) => {
      ;(r.approaches as Array<{ pos: Vec3, reach: number }>).push({ pos: pos.clone(), reach })
      if (r.approachResult === true) {
        // Stand next to the target so the reach gate passes on the retry.
        bot.entity.position = new Vec3(pos.x + 1.5, pos.y, pos.z + 0.5)
        return true
      }
      return false
    }
  }
  r.actions = createActionTable(ctx)
  return r as Rig
}

async function failure (p: Promise<unknown>): Promise<Error> {
  try {
    await p
  } catch (err) {
    return err as Error
  }
  throw new Error('expected the action to fail, but it resolved')
}

/**
 * Run `fn` as soon as `ready()` holds, and never before.
 *
 * The mid-action guards register their listeners part-way through an attempt,
 * after the aim has settled — so a fixed timer races the settle rather than
 * testing anything. Emitting at 40 ms against a ~45 ms settle is a coin flip,
 * and the losing side reports the *later* failure (the verify) instead of the
 * one under test. Each caller waits on the side effect that the listener is
 * provably registered before.
 */
function when (ready: () => boolean, fn: () => void): void {
  const poll = (): void => {
    if (ready()) fn()
    else setTimeout(poll, 1)
  }
  setTimeout(poll, 1)
}

describe('actions: vanilla reach', () => {
  it('measures eye to the closest point of the cell, not centre to centre', () => {
    const eye = new Vec3(0.5, 65.62, 0.5)
    // The cell at (5,64,5): closest corner is (5,65,5) once y is above it.
    const near = reachToCell(eye, new Vec3(5, 64, 5))
    const centre = eye.distanceTo(new Vec3(5.5, 64.5, 5.5))
    expect(near).to.be.lessThan(centre)
    // Standing inside the cell is distance zero, not 0.87.
    expect(reachToCell(new Vec3(3.5, 64.5, 3.5), new Vec3(3, 64, 3))).to.equal(0)
  })

  it('reports the eye at the right height', () => {
    const bot = makeActionBot(new Vec3(0.5, 64, 0.5))
    expect(eyePos(bot as never).y).to.be.closeTo(64 + 1.62, 0.03)
  })

  it('maps face vectors to protocol direction ids', () => {
    expect(faceId(new Vec3(0, -1, 0))).to.equal(0)
    expect(faceId(new Vec3(0, 1, 0))).to.equal(1)
    expect(faceId(new Vec3(1, 0, 0))).to.equal(5)
    expect(faceId(new Vec3(0, 0, 0))).to.equal(-1)
    expect(FACE_VECTORS).to.have.length(6)
  })
})

describe('actions: dig', () => {
  it('breaks a block that is in range', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    setItems(r.bot, ['diamond_pickaxe'])

    await r.actions.dig(r.bot.blockAt(target) as never)

    expect(r.bot.digs).to.have.length(1)
    expect(r.bot.blockAt(target)?.name).to.equal('air')
    expect(r.approaches).to.have.length(0) // already in range: no walking
  })

  it('equips the best tool before aiming, not after', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    setItems(r.bot, ['netherite_pickaxe'])

    await r.actions.dig(r.bot.blockAt(target) as never)
    expect(r.bot.heldItem?.name).to.equal('netherite_pickaxe')
  })

  it('refuses a block outside vanilla range instead of swinging into the void', async () => {
    const r = rig()
    const target = new Vec3(20, 64, 0)
    r.bot.set(target, 'stone')

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never, { approach: false, retries: 1 }))
    expect(err.name).to.equal(ActionErrors.OUT_OF_REACH)
    expect(r.bot.digs).to.have.length(0)
  })

  it('walks into range when the block is too far and approaching is allowed', async () => {
    const r = rig()
    const target = new Vec3(20, 64, 0)
    r.bot.set(target, 'stone')

    await r.actions.dig(r.bot.blockAt(target) as never)

    expect(r.approaches).to.have.length(1)
    expect(r.approaches[0].pos).to.deep.equal(target)
    expect(r.bot.blockAt(target)?.name).to.equal('air')
  })

  it('gives up with Unreachable when there is no way to get near it', async () => {
    const r = rig()
    r.approachResult = false
    const target = new Vec3(20, 64, 0)
    r.bot.set(target, 'stone')

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never))
    expect(err.name).to.equal(ActionErrors.UNREACHABLE)
  })

  it('treats an already-broken block as success, not as an error', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    const block = { ...r.bot.blockAt(target), name: 'stone', position: target } as never
    // Never placed: the cell is air.
    await r.actions.dig(block)
    expect(r.bot.digs).to.have.length(0)
  })

  it('refuses to break something that is not what the caller named', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')
    const stale = { ...r.bot.blockAt(target), name: 'stone' } as never

    const err = await failure(r.actions.dig(stale, { retries: 1 }))
    expect(err.name).to.equal(ActionErrors.NO_TARGET)
    expect(r.bot.digs).to.have.length(0)
  })

  // The dig promise resolving is not evidence the block broke: mineflayer
  // resolves on its own timer. The world is the only authority.
  it('fails when the dig resolves but the block is still standing', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'obsidian')
    r.bot.digNoOp = true

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never, { retries: 2 }))
    expect(err.name).to.equal(ActionErrors.DIG_FAILED)
    expect(err.message).to.contain('still there')
    expect(r.bot.digs).to.have.length(2) // retried, then gave up
  })

  it('retries a transient failure and succeeds on a later attempt', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    r.bot.digNoOp = true

    let attempts = 0
    const inner = r.bot.dig.bind(r.bot)
    r.bot.dig = async (block, forceLook, face) => {
      if (++attempts === 2) r.bot.digNoOp = false
      return await inner(block, forceLook, face)
    }

    await r.actions.dig(r.bot.blockAt(target) as never)
    expect(attempts).to.equal(2)
    expect(r.bot.blockAt(target)?.name).to.equal('air')
  })

  it('holds the swing inside the position-correction window', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    r.forcedMoveAgo = 100 // a lagback 100ms ago; grace is 800ms

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never, { retries: 1 }))
    expect(err.name).to.equal(ActionErrors.DIG_FAILED)
    expect(err.message).to.contain('corrected our position')
    expect(r.bot.digs).to.have.length(0)
  })

  it('refuses to swing while airborne', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    r.bot.entity.onGround = false

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never, { retries: 1 }))
    expect(err.name).to.equal(ActionErrors.DIG_FAILED)
    expect(r.bot.digs).to.have.length(0)
  })

  it('abandons an in-flight dig when the server lags the bot back', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    r.bot.digDelay = 400
    r.bot.digNoOp = true

    const digging = failure(r.actions.dig(r.bot.blockAt(target) as never, { retries: 1 }))
    // The swing has left: `bot.dig` pushes to `digs` synchronously, and
    // `swingWatched` attaches its forcedMove listener before calling it.
    when(() => r.bot.digs.length > 0, () => r.bot.emit('forcedMove'))

    const err = await digging
    expect(err.name).to.equal(ActionErrors.DIG_FAILED)
    expect(err.message).to.contain('lagback')
  })

  it('abandons an in-flight dig when the footing goes', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    r.bot.digDelay = 500
    r.bot.digNoOp = true

    const digging = failure(r.actions.dig(r.bot.blockAt(target) as never, { retries: 1 }))
    // Only once the swing is out: dropping onGround any earlier is caught by
    // the pre-swing `settleGrounded` gate, which is a different test.
    when(() => r.bot.digs.length > 0, () => { r.bot.entity.onGround = false })

    const err = await digging
    expect(err.message).to.contain('footing gone')
  })

  it('emits dig_start and dig_finish around a successful break', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')
    const seen: string[] = []
    r.bot.on('pathfinder:dig_start', () => seen.push('start'))
    r.bot.on('pathfinder:dig_finish', () => seen.push('finish'))

    await r.actions.dig(r.bot.blockAt(target) as never)
    expect(seen).to.deep.equal(['start', 'finish'])
  })
})

describe('actions: place', () => {
  function floorRig (): Rig {
    const r = rig()
    for (let x = -2; x <= 4; x++) {
      for (let z = -2; z <= 4; z++) r.bot.set(new Vec3(x, 63, z), 'stone')
    }
    setItems(r.bot, ['stone'])
    return r
  }

  it('places against the block below and verifies by looking at the world', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)

    await r.actions.place(target, { item: 'stone' })

    expect(r.bot.blockAt(target)?.name).to.equal('stone')
    expect(r.bot.places).to.have.length(1)
    expect(r.bot.places[0].ref).to.deep.equal(new Vec3(2, 63, 0))
    expect(r.bot.places[0].face).to.deep.equal(new Vec3(0, 1, 0))
  })

  // Right-clicking a chest with a block in hand opens it. Vanilla's answer is
  // to sneak, and so is ours — without it a shulker station on a hopper is
  // simply unplaceable.
  it('sneaks when the reference block is one that would open instead', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(new Vec3(2, 63, 0), 'hopper')

    await r.actions.place(target, { item: 'stone' })

    expect(r.bot.places[0].sneaking).to.equal(true)
    expect(r.bot.controlState.sneak).to.equal(false) // and released afterwards
  })

  it('releases sneak even when the click throws', async () => {
    const r = floorRig()
    r.bot.set(new Vec3(2, 63, 0), 'chest')
    r.bot.placeThrowsButWorks = true

    await r.actions.place(new Vec3(2, 64, 0), { item: 'stone' })
    expect(r.bot.controlState.sneak).to.equal(false)
  })

  // placeBlock's block-update wait rejects on a laggy server after the
  // placement landed. Believing the rejection means placing a second block.
  it('ignores a placeBlock rejection when the block actually appeared', async () => {
    const r = floorRig()
    r.bot.placeThrowsButWorks = true
    const target = new Vec3(2, 64, 0)

    await r.actions.place(target, { item: 'stone' })

    expect(r.bot.blockAt(target)?.name).to.equal('stone')
    expect(r.bot.places).to.have.length(1) // not placed twice
  })

  it('fails when nothing appeared, however the click went', async () => {
    const r = floorRig()
    r.bot.placeNoOp = true

    const err = await failure(r.actions.place(new Vec3(2, 64, 0), { item: 'stone', retries: 2 }))
    expect(err.name).to.equal(ActionErrors.PLACE_FAILED)
    expect(r.bot.places).to.have.length(2)
  })

  it('is a no-op when the cell already holds what was asked for', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'stone')

    await r.actions.place(target, { item: 'stone' })
    expect(r.bot.places).to.have.length(0)
  })

  it('refuses to replace a block that is already there', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')

    const err = await failure(r.actions.place(target, { item: 'stone' }))
    expect(err.name).to.equal(ActionErrors.PLACE_FAILED)
    expect(err.message).to.contain('refusing to replace')
  })

  it('says so when the item is not in the inventory', async () => {
    const r = floorRig()
    const err = await failure(r.actions.place(new Vec3(2, 64, 0), { item: 'red_shulker_box' }))
    expect(err.name).to.equal(ActionErrors.MISSING_ITEM)
  })

  it('will not place into the cell the bot is standing in', async () => {
    const r = floorRig()
    const err = await failure(r.actions.place(new Vec3(0, 64, 0), { item: 'stone', retries: 1 }))
    expect(err.name).to.equal(ActionErrors.PLACE_FAILED)
    expect(err.message).to.contain('Standing in')
  })

  // A shulker opens along the face it was placed against. Placed against a
  // face whose opposite side is walled in, it can never be opened again.
  it('skips a supporting face whose opening is obstructed', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    setItems(r.bot, ['red_shulker_box'])
    // Ceiling directly above the target: placing on the floor would seal it.
    r.bot.set(new Vec3(2, 65, 0), 'stone')
    // Leave one side open with a wall to place against.
    r.bot.set(new Vec3(2, 64, 1), 'stone')

    await r.actions.place(target, { item: 'red_shulker_box' })

    // Not the floor (sealed by the ceiling) — the +Z wall, opening toward -Z.
    expect(r.bot.places[0].ref).to.deep.equal(new Vec3(2, 64, 1))
    expect(r.bot.places[0].face).to.deep.equal(new Vec3(0, 0, -1))
  })

  it('gives up when every face would leave the block sealed', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    for (const f of FACE_VECTORS) r.bot.set(target.plus(f), 'stone')

    const err = await failure(r.actions.place(target, { item: 'stone', retries: 1 }))
    expect(err.name).to.equal(ActionErrors.PLACE_FAILED)
    expect(err.message).to.contain('No supporting face')
  })

  it('can be told not to care about the opening', async () => {
    const r = floorRig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(new Vec3(2, 65, 0), 'stone') // sealed from above

    await r.actions.place(target, { item: 'stone', requireOpenable: false })
    expect(r.bot.places[0].ref).to.deep.equal(new Vec3(2, 63, 0))
  })
})

describe('actions: open', () => {
  it('opens a chest and hands back the window', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')

    const window = await r.actions.open(r.bot.blockAt(target) as never)
    expect(window.type).to.contain('generic')
    expect(r.bot.activations).to.have.length(1)
  })

  // The whole point of not using bot.openContainer: it asserts the window
  // against a chest allowlist and throws on anything else.
  it('opens things that are not containers at all', async () => {
    const r = rig()
    for (const [name, type] of [['crafting_table', 'crafting'], ['enchanting_table', 'enchantment'], ['furnace', 'furnace']]) {
      const target = new Vec3(2, 64, 0)
      r.bot.set(target, name)
      const window = await r.actions.open(r.bot.blockAt(target) as never, { reuse: false })
      expect(window.type).to.contain(type)
    }
  })

  it('times out instead of hanging when the server drops the click', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')
    r.bot.openSilent = true

    const err = await failure(r.actions.open(r.bot.blockAt(target) as never, {
      windowTimeout: 30, retries: 2
    }))
    expect(err.name).to.equal(ActionErrors.OPEN_FAILED)
    expect(err.message).to.contain('never processed the click')
  })

  // The listener openBlock leaves behind would otherwise resolve its
  // abandoned promise with a window opened minutes later by something else.
  it('removes the listener the abandoned open left behind', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')
    r.bot.openSilent = true
    const before = r.bot.listenerCount('windowOpen')

    await failure(r.actions.open(r.bot.blockAt(target) as never, { windowTimeout: 30, retries: 2 }))

    expect(r.bot.listenerCount('windowOpen')).to.equal(before)
  })

  it('distinguishes a click the server refused from one it never saw', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')
    r.bot.openSilent = true
    // A block update on the target: the server processed the click. Sent once
    // the click is actually out — `openBlock` pushes to `activations`
    // synchronously, and `clickAndWait` subscribes to blockUpdate first.
    when(() => r.bot.activations.length > 0, () => {
      r.bot.emit('blockUpdate', null, { position: target })
    })

    const err = await failure(r.actions.open(r.bot.blockAt(target) as never, {
      windowTimeout: 40, retries: 1
    }))
    expect(err.message).to.contain('processed the click and refused it')
  })

  it('clears sneak first — a sneaking player places instead of opening', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')
    r.bot.setControlState('sneak', true)

    await r.actions.open(r.bot.blockAt(target) as never)
    expect(r.bot.controlState.sneak).to.equal(false)
  })

  it('closes a stale window before opening another', async () => {
    const r = rig()
    r.bot.set(new Vec3(2, 64, 0), 'chest')
    r.bot.set(new Vec3(2, 64, 1), 'barrel')

    await r.actions.open(r.bot.blockAt(new Vec3(2, 64, 0)) as never)
    const first = r.bot.currentWindow
    await r.actions.open(r.bot.blockAt(new Vec3(2, 64, 1)) as never)

    expect(r.bot.currentWindow).to.not.equal(first)
    expect(r.bot.activations).to.have.length(2)
  })

  it('hands back the window it already has for the same block', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'chest')

    const first = await r.actions.open(r.bot.blockAt(target) as never)
    const second = await r.actions.open(r.bot.blockAt(target) as never)

    expect(second).to.equal(first)
    expect(r.bot.activations).to.have.length(1) // not clicked twice
  })

  it('walks into range for a container it cannot reach', async () => {
    const r = rig()
    const target = new Vec3(20, 64, 0)
    r.bot.set(target, 'chest')

    await r.actions.open(r.bot.blockAt(target) as never)
    expect(r.approaches).to.have.length(1)
  })
})

describe('actions: activate', () => {
  it('right-clicks a block without waiting for a window', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'lever')

    await r.actions.activate(r.bot.blockAt(target) as never)
    expect(r.bot.activations).to.deep.equal([target])
  })

  it('skips the aim settle when the caller asks it to', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'oak_door')
    let looks = 0
    const inner = r.bot.lookAt.bind(r.bot)
    r.bot.lookAt = async (pos, force) => { looks++; return await inner(pos, force) }

    await r.actions.activate(r.bot.blockAt(target) as never, { settle: false })
    expect(looks).to.equal(0)
    expect(r.bot.activations).to.deep.equal([target])
  })

  it('refuses a block out of vanilla range', async () => {
    const r = rig()
    const target = new Vec3(20, 64, 0)
    r.bot.set(target, 'lever')

    const err = await failure(r.actions.activate(r.bot.blockAt(target) as never, { approach: false }))
    expect(err.name).to.equal(ActionErrors.OUT_OF_REACH)
  })
})

describe('actions: pacing', () => {
  it('spaces consecutive actions out, and shares one window across kinds', async () => {
    const pacer = new Pacer()
    const waits: number[] = []
    const record = async (ticks: number): Promise<void> => { waits.push(ticks) }

    for (let i = 0; i < 4; i++) await pacer.cooldown('default', record)

    expect(waits).to.have.length(4)
    for (const w of waits.slice(0, 3)) {
      expect(w).to.be.at.least(12).and.at.most(18)
    }
    // Every fourth action takes the longer breather.
    expect(waits[3]).to.be.at.least(12 + 30)
  })

  it('drill pacing stays above vanilla\'s five-tick floor', async () => {
    const pacer = new Pacer()
    const waits: number[] = []
    for (let i = 0; i < 9; i++) await pacer.cooldown('drill', async (t) => { waits.push(t) })
    for (const w of waits) expect(w).to.be.greaterThan(5)
  })

  it('none means none', async () => {
    const pacer = new Pacer()
    const waits: number[] = []
    for (let i = 0; i < 5; i++) await pacer.cooldown('none', async (t) => { waits.push(t) })
    expect(waits).to.have.length(0)
  })
})

describe('actions: the table is the seam', () => {
  it('a wrapped dig sees every break, and the wrapper can veto', async () => {
    const r = rig()
    const target = new Vec3(2, 64, 0)
    r.bot.set(target, 'bedrock')

    const inner = r.actions.dig
    const seen: string[] = []
    r.actions.dig = async (block, options) => {
      seen.push(block.name)
      if (block.name === 'bedrock') throw new Error('not on my watch')
      return await inner(block, options)
    }

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never))
    expect(err.message).to.equal('not on my watch')
    expect(seen).to.deep.equal(['bedrock'])
    expect(r.bot.digs).to.have.length(0)
  })

  it('config changes take effect on the next action', async () => {
    const r = rig()
    const target = new Vec3(6, 64, 0) // 5.5 away: outside 4.5, inside 6
    r.bot.set(target, 'stone')

    const err = await failure(r.actions.dig(r.bot.blockAt(target) as never, { approach: false, retries: 1 }))
    expect(err.name).to.equal(ActionErrors.OUT_OF_REACH)

    r.actions.config.reach = 8
    await r.actions.dig(r.bot.blockAt(target) as never, { approach: false })
    expect(r.bot.blockAt(target)?.name).to.equal('air')
  })
})
