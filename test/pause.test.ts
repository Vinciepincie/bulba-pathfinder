// The interrupt protocol driving a real path: what a borrower actually gets,
// and what the walk looks like on either side of it.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { GoalBlock } from '../src/goals.js'
import { VoxelWorld, STONE, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'
import { autoEatIntegration } from '../src/integrations/autoEat.js'
import type { InterruptHandle, InterruptOptions, MotionPhase } from '../src/interrupt.js'
import type { Move } from '../src/move.js'

interface PF {
  goto: (goal: unknown) => Promise<void>
  setMovements: (m: Movements) => void
  stop: () => void
  isMoving: () => boolean
  goal: unknown
  interrupt: (reason: string, options?: InterruptOptions) => Promise<InterruptHandle>
  withInterrupt: <T>(reason: string, fn: () => Promise<T>, options?: InterruptOptions) => Promise<T>
  motion: {
    phase: MotionPhase
    critical: boolean
    jumpPending: boolean
    paused: boolean
    holders: string[]
    node: Move | null
  }
  actions: { config: { pacing: string } }
}

function makeWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 32, y1: 8, z1: 24 })
  world.fill(-8, -1, -8, 32, -1, 24, STONE) // floor at y=-1, walkable at y=0
  return world
}

function setup (world: VoxelWorld, start = new Vec3(0.5, 0, 0.5)): { bot: DriveableBot, pf: PF } {
  const bot = makeDriveableBot(world, start)
  bot.loadPlugin(createPathfinder({
    useWorkerThreads: false,
    physicsFactory: () => makeFakePhysics(world, bot)
  }) as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: PF }).pathfinder
  pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
  return { bot, pf }
}

/** Pump ticks until `done()` or the budget runs out. */
async function pump (bot: DriveableBot, ticks: number, done?: () => boolean): Promise<number> {
  for (let i = 0; i < ticks; i++) {
    if (done?.() === true) return i
    bot.tick()
    await new Promise(resolve => setImmediate(resolve))
    await new Promise(resolve => setTimeout(resolve, 1))
  }
  return ticks
}

describe('pause: borrowing the bot mid-path', function () {
  this.timeout(60000)

  it('stops the bot without touching the path, and finishes the walk on release', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)

    let settled: string | null = null
    const walking = pf.goto(new GoalBlock(20, 0, 0)).then(
      () => { settled = 'resolved' },
      (err: Error) => { settled = `rejected:${err.name}` }
    )

    // Let it get moving.
    await pump(bot, 30, () => pf.isMoving())
    expect(pf.isMoving()).to.equal(true)

    const resets: string[] = []
    const stops: number[] = []
    bot.on('path_reset', (reason: string) => resets.push(reason))
    bot.on('path_stop', () => stops.push(1))

    // Borrow it.
    const grant = pf.interrupt('autoeat')
    await pump(bot, 20, () => pf.motion.paused)
    const handle = await grant

    expect(pf.motion.paused).to.equal(true)
    expect(pf.motion.phase).to.equal('paused')
    expect(pf.motion.holders).to.deep.equal(['autoeat'])
    // The path survives: no reset, no stop, the goal is still set and the
    // goto promise is still pending.
    expect(pf.isMoving()).to.equal(true)
    expect(pf.goal).to.not.equal(null)
    expect(settled).to.equal(null)
    expect(resets).to.deep.equal([])
    expect(stops).to.deep.equal([])
    expect(Object.values(bot.controlState).some(v => v)).to.equal(false)

    // And it really is standing still.
    const parked = bot.entity.position.clone()
    await pump(bot, 40)
    expect(bot.entity.position.distanceTo(parked)).to.be.lessThan(0.05)
    expect(settled).to.equal(null)

    handle.release()
    await pump(bot, 1500, () => settled !== null)
    await walking

    expect(settled).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(20.5, 0, 0.5))).to.be.lessThan(1.2)
  })

  it('a long pause is not mistaken for a stall', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    // Without the timer correction on resume, a pause longer than this would
    // trip the stuck detector the moment the bot started walking again.
    ;(pf as unknown as { stuckTimeout: number }).stuckTimeout = 400

    let settled: string | null = null
    const walking = pf.goto(new GoalBlock(20, 0, 0)).then(
      () => { settled = 'resolved' },
      (err: Error) => { settled = `rejected:${err.name}` }
    )

    await pump(bot, 30, () => pf.isMoving())
    const handle = await (async () => {
      const g = pf.interrupt('long-eat')
      await pump(bot, 20, () => pf.motion.paused)
      return await g
    })()

    // Hold it well past the stuck timeout.
    await pump(bot, 500)
    handle.release()

    await pump(bot, 1500, () => settled !== null)
    await walking
    expect(settled).to.equal('resolved')
  })

  it('stop() still takes effect while paused', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)

    let settled: string | null = null
    const walking = pf.goto(new GoalBlock(20, 0, 0)).then(
      () => { settled = 'resolved' },
      (err: Error) => { settled = `rejected:${err.name}` }
    )
    await pump(bot, 30, () => pf.isMoving())

    const grant = pf.interrupt('holder')
    await pump(bot, 20, () => pf.motion.paused)
    await grant

    pf.stop()
    await pump(bot, 50, () => settled !== null)
    await walking.catch(() => {})
    expect(settled).to.equal('rejected:PathStopped')
  })

  it('withInterrupt releases even when the body throws', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)

    let message: string | null = null
    const running = pf.withInterrupt('boom', async () => { throw new Error('nope') })
      .catch((e: Error) => { message = e.message })

    // The grant happens on a tick, so the body cannot run until we pump.
    await pump(bot, 40, () => message !== null)
    await running

    expect(message).to.equal('nope')
    expect(pf.motion.paused).to.equal(false)
  })

  it('reports the phase the executor is actually in', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    expect(pf.motion.phase).to.equal('idle')
    expect(pf.motion.node).to.equal(null)

    const seen = new Set<MotionPhase>()
    const walking = pf.goto(new GoalBlock(16, 0, 0)).catch(() => {})
    await pump(bot, 1200, () => {
      seen.add(pf.motion.phase)
      return !pf.isMoving() && pf.goal === null
    })
    await walking

    expect(seen.has('walking')).to.equal(true)
    expect(pf.motion.phase).to.equal('idle')
  })
})

describe('pause: auto-eat integration', function () {
  this.timeout(60000)

  interface FakeAutoEat {
    isEating: boolean
    enableAuto: () => void
    disableAuto: () => void
    eat: () => Promise<void>
    opts: Record<string, unknown>
    /** When each bite started, and where the bot was. */
    bites: Array<{ onGround: boolean, paused: boolean }>
    autoEnabled: boolean
    biteMs: number
  }

  function withAutoEat (bot: DriveableBot, pf: PF): FakeAutoEat {
    const ae: FakeAutoEat = {
      isEating: false,
      autoEnabled: true,
      biteMs: 30,
      bites: [],
      opts: { startAt: 16 },
      enableAuto: () => { ae.autoEnabled = true },
      disableAuto: () => { ae.autoEnabled = false },
      eat: async () => {
        ae.isEating = true
        ae.bites.push({ onGround: bot.entity.onGround, paused: pf.motion.paused })
        await new Promise(resolve => setTimeout(resolve, ae.biteMs))
        ae.isEating = false
        ;(bot as unknown as { food: number }).food = 20
      }
    }
    ;(bot as unknown as { autoEat: FakeAutoEat }).autoEat = ae
    return ae
  }

  it('takes the plugin\'s automatic mode over and hands it back on detach', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const ae = withAutoEat(bot, pf)
    ;(bot as unknown as { food: number }).food = 20

    bot.loadPlugin(autoEatIntegration({ checkEveryTicks: 1 }) as unknown as (b: unknown) => void)
    await pump(bot, 3)
    expect(ae.autoEnabled).to.equal(false)

    const control = (bot as unknown as { pathfinderAutoEat: { detach: () => void } }).pathfinderAutoEat
    control.detach()
    expect(ae.autoEnabled).to.equal(true)
  })

  it('does not eat until the bot is standing on the ground', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const ae = withAutoEat(bot, pf)
    ;(bot as unknown as { food: number }).food = 10 // hungry from the start

    bot.loadPlugin(autoEatIntegration({ checkEveryTicks: 1, waitForSafe: 0 }) as unknown as (b: unknown) => void)

    // Airborne and staying that way: the request must wait.
    bot.entity.onGround = false
    bot.frozen = true
    await pump(bot, 40)
    expect(ae.bites).to.have.length(0)

    bot.entity.onGround = true
    await pump(bot, 30, () => ae.bites.length > 0)
    expect(ae.bites).to.have.length(1)
    // And it ate with the executor stopped, not while it was driving.
    expect(ae.bites[0].onGround).to.equal(true)
    expect(ae.bites[0].paused).to.equal(true)
  })

  it('eats mid-walk without disturbing the goal, then the walk finishes', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const ae = withAutoEat(bot, pf)
    ;(bot as unknown as { food: number }).food = 20

    bot.loadPlugin(autoEatIntegration({ checkEveryTicks: 1 }) as unknown as (b: unknown) => void)

    const resets: string[] = []
    bot.on('path_reset', (reason: string) => resets.push(reason))

    let settled: string | null = null
    const walking = pf.goto(new GoalBlock(20, 0, 0)).then(
      () => { settled = 'resolved' },
      (err: Error) => { settled = `rejected:${err.name}` }
    )

    await pump(bot, 40, () => pf.isMoving())
    ;(bot as unknown as { food: number }).food = 10 // gets hungry mid-walk

    await pump(bot, 2000, () => settled !== null)
    await walking

    expect(ae.bites.length).to.be.greaterThan(0)
    expect(ae.bites[0].paused).to.equal(true)
    expect(settled).to.equal('resolved')
    expect(resets).to.deep.equal([]) // eating never replanned anything
  })

  it('suspend() holds the automatic checks off', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const ae = withAutoEat(bot, pf)
    ;(bot as unknown as { food: number }).food = 5

    bot.loadPlugin(autoEatIntegration({ checkEveryTicks: 1 }) as unknown as (b: unknown) => void)
    const control = (bot as unknown as {
      pathfinderAutoEat: { suspend: () => void, resume: () => void, suspended: boolean }
    }).pathfinderAutoEat

    control.suspend()
    await pump(bot, 40)
    expect(ae.bites).to.have.length(0)
    expect(control.suspended).to.equal(true)

    control.resume()
    await pump(bot, 40, () => ae.bites.length > 0)
    expect(ae.bites).to.have.length(1)
  })
})
