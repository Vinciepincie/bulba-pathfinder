// The contract other packages build on: cancellation, stop reasons, the health
// floor, lifecycle, composable actions and planning without moving.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createRequire } from 'node:module'
import { existsSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { createPathfinder } from '../src/plugin.js'
import { Movements, physicsClimbsVines } from '../src/movements.js'
import * as goals from '../src/goals.js'
import { GoalBlock } from '../src/goals.js'
import { dropForHealth, fallVetoForHealth, FALL_VETO } from '../src/fallDamage.js'
import { getLut } from '../src/lut.js'
import { PhysicsSim } from '../src/physics.js'
import { SIM_CLIMBABLE } from '../src/playerSim.js'
import { VoxelWorld, STONE, applyProfile, mcData } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'
import type { Pathfinder, PathStoppedError } from '../src/api.js'

const require = createRequire(import.meta.url)
const { Physics } = require('prismarine-physics')
const workerEntry = join(dirname(fileURLToPath(import.meta.url)), '..', 'dist', 'esm', 'worker', 'entry.js')

function makeWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -8, y0: -12, z0: -8, x1: 24, y1: 8, z1: 24 })
  world.fill(-8, -1, -8, 24, -1, 24, STONE)
  return world
}

function setup (world: VoxelWorld, options: Record<string, unknown> = {}): { bot: DriveableBot, pf: Pathfinder } {
  const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
  bot.loadPlugin(createPathfinder({
    useWorkerThreads: false,
    physicsFactory: () => makeFakePhysics(world, bot),
    ...options
  }) as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: Pathfinder }).pathfinder
  pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
  return { bot, pf }
}

/** A clear line of sight to `target` from anywhere, for the goals that raycast. */
function seeing (bot: DriveableBot, target: Vec3): Vec3 {
  ;(bot.world as { raycast?: unknown }).raycast = () => ({ position: target })
  return target
}

async function settle<T> (bot: DriveableBot, promise: Promise<T>, maxTicks = 1500): Promise<{ resolved: boolean, error?: Error }> {
  let out: { resolved: boolean, error?: Error } | null = null
  promise.then(() => { out = { resolved: true } }, (error: Error) => { out = { resolved: false, error } })
  for (let i = 0; i < maxTicks && out === null; i++) {
    bot.tick()
    await new Promise(resolve => setTimeout(resolve, 1))
  }
  if (out === null) throw new Error('did not settle')
  return out
}

describe('public API', function () {
  this.timeout(30000)

  describe('goto signal', () => {
    it('rejects with the signal reason and stops the bot', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      const going = pf.goto(new GoalBlock(20, 0, 0), { signal: controller.signal })
      const result = settle(bot, going)
      for (let i = 0; i < 20; i++) bot.tick()
      const reason = new Error('enough')
      controller.abort(reason)
      const outcome = await result
      expect(outcome.resolved).to.equal(false)
      expect(outcome.error).to.equal(reason)
      expect(pf.goal).to.equal(null)
      expect(pf.isMoving()).to.equal(false)
      expect(bot.controlState.forward).to.equal(false)
    })

    it('rejects with AbortError on a plain abort', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      const going = pf.goto(new GoalBlock(20, 0, 0), { signal: controller.signal })
      controller.abort()
      const outcome = await settle(bot, going)
      expect(outcome.error?.name).to.equal('AbortError')
    })

    it('does not touch the current goal when already aborted', async () => {
      const { bot, pf } = setup(makeWorld())
      const running = new GoalBlock(20, 0, 0)
      pf.setGoal(running)
      const outcome = await settle(bot, pf.goto(new GoalBlock(1, 0, 0), { signal: AbortSignal.abort() }))
      expect(outcome.resolved).to.equal(false)
      expect(pf.goal).to.equal(running)
    })

    it('leaves the next goal alone', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      const first = pf.goto(new GoalBlock(20, 0, 0), { signal: controller.signal })
      first.catch(() => {})
      for (let i = 0; i < 10; i++) bot.tick()
      controller.abort()
      expect((await settle(bot, pf.goto(new GoalBlock(4, 0, 0)))).resolved).to.equal(true)
    })

    it('removes its abort listener when the goto ends', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      await settle(bot, pf.goto(new GoalBlock(2, 0, 0), { signal: controller.signal }))
      // A late abort must not stop a goal that started afterwards.
      const next = new GoalBlock(20, 0, 0)
      pf.setGoal(next)
      controller.abort()
      expect(pf.goal).to.equal(next)
    })

    it('an abort fired during the replacement leaves the new goal alone', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      const next = new GoalBlock(9, 0, 0)
      // Registered before goto's own listener, so it runs first.
      bot.on('goal_updated', (g) => { if (g === next) controller.abort() })
      const first = pf.goto(new GoalBlock(20, 0, 0), { signal: controller.signal })
      const error = first.then(() => null, (e: Error) => e)
      pf.setGoal(next)
      expect(pf.goal).to.equal(next)
      expect((await error)?.name).to.equal('AbortError')
    })

    it('honours an abort fired by a stop being flushed', async () => {
      const { bot, pf } = setup(makeWorld())
      const old = pf.goto(new GoalBlock(20, 0, 0))
      old.catch(() => {})
      for (let i = 0; i < 5; i++) bot.tick()
      const controller = new AbortController()
      bot.once('path_stop', () => controller.abort())
      pf.stop()
      const error = await pf.goto(new GoalBlock(3, 0, 0), { signal: controller.signal }).then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('AbortError')
      expect(pf.goal).to.equal(null)
    })
  })

  describe('a failed goto', () => {
    /** Walls off the x axis at x = 5, so anything past it has no path. */
    function walled (): VoxelWorld {
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE)
      return world
    }

    it('rejects NoPath and ends the goal instead of walking the partial path', async () => {
      const { bot, pf } = setup(walled())
      const stops: unknown[] = []
      bot.on('path_stop', r => stops.push(r))
      const outcome = await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(outcome.error?.name).to.equal('NoPath')
      bot.tick()
      expect(pf.goal).to.equal(null)
      expect(pf.isMoving()).to.equal(false)
      expect(stops).to.deep.equal(['no_path'])
      const at = bot.entity.position.clone()
      for (let i = 0; i < 20; i++) bot.tick()
      expect(bot.entity.position.distanceTo(at)).to.be.lessThan(0.5)
    })

    it('rejects NoPath for a bot walled in, where the path is empty', async () => {
      const world = makeWorld()
      world.fill(-1, 0, -1, 1, 8, 1, STONE)
      world.fill(0, 0, 0, 0, 8, 0, 0)
      const { bot, pf } = setup(world)
      const outcome = await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(outcome.error?.name).to.equal('NoPath')
    })

    it('best effort walks as close as it can, resolves and ends the goal', async () => {
      const { bot, pf } = setup(walled())
      const outcome = await settle(bot, pf.goto(new GoalBlock(9, 0, 0), { bestEffort: true }))
      expect(outcome.resolved).to.equal(true)
      bot.tick()
      expect(pf.goal).to.equal(null)
      expect(pf.isMoving()).to.equal(false)
      expect(bot.entity.position.x).to.be.greaterThan(3)
    })

    it('rejects Timeout on an empty path, and a success on an empty path resolves', async () => {
      const { bot, pf } = setup(makeWorld())
      const timedOut = pf.goto(new GoalBlock(9, 0, 0))
      bot.emit('path_update', { status: 'timeout', path: [] })
      expect((await settle(bot, timedOut)).error?.name).to.equal('Timeout')
      const fine = pf.goto(new GoalBlock(9, 0, 0))
      bot.emit('path_update', { status: 'success', path: [] })
      expect((await settle(bot, fine)).resolved).to.equal(true)
    })

    it('a plain setGoal stays set after a failed search', async () => {
      const { bot, pf } = setup(walled())
      const goal = new GoalBlock(9, 0, 0)
      pf.setGoal(goal)
      for (let i = 0; i < 30; i++) bot.tick()
      expect(pf.goal).to.equal(goal)
    })
  })

  describe('stop reasons', () => {
    it('stop() rejects PathStopped with reason stop, and the event carries it', async () => {
      const { bot, pf } = setup(makeWorld())
      const seen: unknown[] = []
      bot.on('path_stop', r => seen.push(r))
      const going = pf.goto(new GoalBlock(20, 0, 0))
      const result = settle(bot, going)
      for (let i = 0; i < 10; i++) bot.tick()
      pf.stop()
      const outcome = await result
      expect(outcome.error?.name).to.equal('PathStopped')
      expect((outcome.error as PathStoppedError).reason).to.equal('stop')
      expect(seen).to.deep.equal(['stop'])
    })

    it('a goto right after stop() runs, and only the old one rejects', async () => {
      const { bot, pf } = setup(makeWorld())
      const old = pf.goto(new GoalBlock(20, 0, 0))
      const oldResult = settle(bot, old)
      for (let i = 0; i < 10; i++) bot.tick()
      pf.stop()
      const next = await settle(bot, pf.goto(new GoalBlock(3, 0, 0)))
      expect(next.resolved).to.equal(true)
      expect((await oldResult).error?.name).to.equal('PathStopped')
    })
  })

  describe('minHealth', () => {
    it('stays out of the way at 0, the default', async () => {
      const { bot, pf } = setup(makeWorld())
      expect(pf.minHealth).to.equal(0)
      ;(bot as unknown as { health: number }).health = 1
      expect((await settle(bot, pf.goto(new GoalBlock(3, 0, 0)))).resolved).to.equal(true)
    })

    it('stops the goal below the floor, with the reason and an event', async () => {
      const { bot, pf } = setup(makeWorld())
      const body = bot as unknown as { health: number }
      body.health = 20
      pf.minHealth = 8
      const low: unknown[] = []
      bot.on('pathfinder:low_health', (health, floor) => low.push([health, floor]))
      const result = settle(bot, pf.goto(new GoalBlock(20, 0, 0)))
      for (let i = 0; i < 10; i++) bot.tick()
      expect(pf.goal).to.not.equal(null)
      body.health = 7
      const outcome = await result
      expect((outcome.error as PathStoppedError).name).to.equal('PathStopped')
      expect((outcome.error as PathStoppedError).reason).to.equal('low_health')
      expect(low).to.deep.equal([[7, 8]])
    })

    it('waits for the landing rather than stopping in flight', async () => {
      const { bot, pf } = setup(makeWorld())
      const body = bot as unknown as { health: number }
      body.health = 20
      pf.minHealth = 8
      let stopped = 0
      bot.on('path_stop', () => { stopped++ })
      const going = pf.goto(new GoalBlock(20, 0, 0))
      going.catch(() => {})
      for (let i = 0; i < 5; i++) bot.tick()
      body.health = 2
      bot.entity.onGround = false
      bot.frozen = true
      bot.tick()
      expect(stopped).to.equal(0)
      bot.entity.onGround = true
      bot.tick()
      expect(stopped).to.equal(1)
    })

    it('rejects a new goal while still below the floor', async () => {
      const { bot, pf } = setup(makeWorld())
      ;(bot as unknown as { health: number }).health = 3
      pf.minHealth = 8
      const outcome = await settle(bot, pf.goto(new GoalBlock(5, 0, 0)))
      expect((outcome.error as PathStoppedError).reason).to.equal('low_health')
    })

    it('caps planned drops to what health can pay for', async () => {
      const world = new VoxelWorld({ x0: -4, y0: -10, z0: -4, x1: 12, y1: 6, z1: 4 })
      world.fill(-4, -1, -2, 1, -1, 2, STONE)
      world.fill(2, -8, -2, 12, -8, 2, STONE) // a 7-block drop to the far floor
      const { bot, pf } = setup(world)
      pf.movements.maxDropDown = 10
      const body = bot as unknown as { health: number }
      body.health = 20
      pf.minHealth = 19
      const goal = new GoalBlock(6, -7, 0)
      expect((await settle(bot, pf.goto(goal), 200)).error?.name).to.equal('NoPath')
      expect(pf.getPathTo(pf.movements, goal).status).to.equal('noPath')
      pf.minHealth = 1
      const cleared = await settle(bot, pf.goto(goal), 400)
      expect(cleared.resolved).to.equal(true)
    })

    it('keeps a chosen drop short of everything health has spare', () => {
      expect(dropForHealth(0)).to.equal(2)
      expect(dropForHealth(1)).to.equal(2)
      expect(dropForHealth(2)).to.equal(3)
      expect(dropForHealth(9.5)).to.equal(10)
      // A jump's rise adds 1.25 to the fall; the whole fall past 3 blocks is the damage.
      for (let spare = 0; spare < 20; spare++) {
        expect(Math.max(0, Math.floor(dropForHealth(spare) + 1.25 - 3))).to.be.at.most(spare)
      }
    })

    it('hop searches may not spend everything health has spare', () => {
      expect(fallVetoForHealth(-3)).to.equal(1)
      expect(fallVetoForHealth(0)).to.equal(1)
      expect(fallVetoForHealth(0.5)).to.equal(1)
      expect(fallVetoForHealth(3)).to.equal(3)
      expect(fallVetoForHealth(3.5)).to.equal(4)
      expect(fallVetoForHealth(19)).to.equal(FALL_VETO)
    })

    it('hands the hop limit to the worker search and to the live searches', async () => {
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE)
      const physics: { fallVeto?: number } = {}
      let cfg: { fallVeto?: number } | null = null
      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      bot.loadPlugin(createPathfinder({
        useWorkerThreads: false,
        physicsFactory: () => Object.assign(physics, makeFakePhysics(world, bot)) as never,
        onNoPath: (dump) => { cfg = dump.cfg }
      }) as unknown as (b: unknown) => void)
      const pf = (bot as unknown as { pathfinder: Pathfinder }).pathfinder
      pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
      const body = bot as unknown as { health: number }
      body.health = 20

      await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(cfg!.fallVeto).to.equal(undefined)
      expect(physics.fallVeto).to.equal(FALL_VETO)

      pf.minHealth = 18
      await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(cfg!.fallVeto).to.equal(2)
      bot.tick()
      expect(physics.fallVeto).to.equal(2)
    })
  })

  describe('lifecycle', () => {
    it('a disconnect settles goto without another tick, and touches no controls', async () => {
      const { bot, pf } = setup(makeWorld())
      const going = pf.goto(new GoalBlock(20, 0, 0))
      for (let i = 0; i < 5; i++) bot.tick()
      let writes = 0
      bot.setControlState = () => { writes++ }
      bot.clearControlStates = () => { writes++ }
      bot.emit('end', 'socketClosed')
      const error = await going.then(() => null, (e: PathStoppedError) => e)
      expect(error?.reason).to.equal('disconnected')
      expect(writes).to.equal(0)
    })

    it('a disconnect fails waiting interrupts and later requests', async () => {
      const { bot, pf } = setup(makeWorld())
      bot.entity.onGround = false
      bot.frozen = true
      const waiting = pf.interrupt('x', { timeout: 0 })
      bot.emit('end', 'socketClosed')
      const error = await waiting.then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('InterruptAborted')
      const later = await pf.interrupt('y').then(() => null, (e: Error) => e)
      expect(later?.name).to.equal('InterruptAborted')
    })

    it('death ends the goal by default', async () => {
      const { bot, pf } = setup(makeWorld())
      const going = pf.goto(new GoalBlock(20, 0, 0))
      for (let i = 0; i < 5; i++) bot.tick()
      bot.emit('death')
      const error = await going.then(() => null, (e: PathStoppedError) => e)
      expect(error?.reason).to.equal('death')
      expect(pf.goal).to.equal(null)
    })

    it('stopOnDeath off keeps the goal', () => {
      const { bot, pf } = setup(makeWorld())
      pf.stopOnDeath = false
      const goal = new GoalBlock(20, 0, 0)
      pf.setGoal(goal)
      bot.emit('death')
      expect(pf.goal).to.equal(goal)
    })

    it('a respawn drops the path and solves again from the new place', async () => {
      const { bot, pf } = setup(makeWorld())
      pf.stopOnDeath = false
      const goal = new GoalBlock(20, 0, 0)
      pf.setGoal(goal)
      for (let i = 0; i < 5; i++) bot.tick()
      expect(pf.isMoving()).to.equal(true)
      bot.emit('respawn')
      expect(pf.isMoving()).to.equal(false)
      expect(pf.goal).to.equal(goal)
      for (let i = 0; i < 5; i++) bot.tick()
      expect(pf.isMoving()).to.equal(true)
    })

    it('dispose removes every listener and the plugin', async () => {
      const world = makeWorld()
      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      const before = new Map(bot.eventNames().map(n => [n, bot.listenerCount(n)]))
      bot.loadPlugin(createPathfinder({
        useWorkerThreads: false,
        physicsFactory: () => makeFakePhysics(world, bot)
      }) as unknown as (b: unknown) => void)
      const pf = (bot as unknown as { pathfinder: Pathfinder }).pathfinder
      pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
      const going = pf.goto(new GoalBlock(20, 0, 0))
      for (let i = 0; i < 5; i++) bot.tick()
      pf.dispose()
      const error = await going.then(() => null, (e: PathStoppedError) => e)
      expect(error?.reason).to.equal('disposed')
      for (const name of bot.eventNames()) expect(bot.listenerCount(name), String(name)).to.equal(before.get(name) ?? 0)
      expect((bot as unknown as { pathfinder?: unknown }).pathfinder).to.equal(undefined)
      pf.dispose() // Idempotent.
    })

    it('stopping a dig leaves other plugins\' listeners alone', async () => {
      const world = makeWorld()
      const { bot, pf } = setup(world)
      const theirs = (): void => {}
      bot.on('diggingCompleted', theirs)
      bot.on('diggingAborted', theirs)
      world.set(2, 0, 0, STONE)
      world.set(2, 1, 0, STONE)
      world.fill(2, 0, -8, 2, 8, 24, STONE)
      pf.movements.canDig = true
      pf.setMovements(pf.movements)
      const going = pf.goto(new GoalBlock(4, 0, 0))
      going.catch(() => {})
      for (let i = 0; i < 400 && !pf.isMining(); i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(pf.isMining()).to.equal(true)
      pf.stop()
      bot.tick()
      bot.emit('diggingAborted')
      expect(bot.listeners('diggingCompleted')).to.include(theirs)
      expect(bot.listeners('diggingAborted')).to.include(theirs)
    })

    it('a goto after the end rejects at once, also from a path_stop listener', async () => {
      const { bot, pf } = setup(makeWorld())
      const going = pf.goto(new GoalBlock(20, 0, 0))
      going.catch(() => {})
      let fromListener: Promise<void> | null = null
      bot.once('path_stop', () => { fromListener = pf.goto(new GoalBlock(9, 0, 0)) })
      for (let i = 0; i < 5; i++) bot.tick()
      bot.emit('end', 'socketClosed')
      const error = await (fromListener as unknown as Promise<void>).then(() => null, (e: PathStoppedError) => e)
      expect(error?.reason).to.equal('disconnected')
      const later = await pf.goto(new GoalBlock(9, 0, 0)).then(() => null, (e: PathStoppedError) => e)
      expect(later?.reason).to.equal('disconnected')
    })

    it('a disconnect during an action\'s walk rejects it with ActionAborted, without another tick', async () => {
      const { bot, pf } = setup(makeWorld())
      const acting = pf.activate(seeing(bot, new Vec3(20, 0, 0)))
      for (let i = 0; i < 400 && !pf.isMoving(); i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(pf.isMoving()).to.equal(true)
      bot.emit('end', 'socketClosed')
      const error = await acting.then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('ActionAborted')
    })

    it('a disconnect settles a click mineflayer would never finish', async () => {
      const { bot, pf } = setup(makeWorld())
      let clicked = false
      bot.activateBlock = async () => {
        clicked = true
        await new Promise(() => {})
      }
      const acting = pf.activate(new Vec3(1, 0, 0))
      for (let i = 0; i < 400 && !clicked; i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(clicked).to.equal(true)
      bot.emit('end', 'socketClosed')
      const error = await acting.then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('ActionAborted')
    })

    it('a queued action fails too once the bot is gone', async () => {
      const { bot, pf } = setup(makeWorld())
      bot.emit('end', 'socketClosed')
      const error = await pf.activate(new Vec3(1, 0, 0)).then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('ActionAborted')
    })

    it('stop() during an action\'s walk fails it, and the bot stays stopped', async () => {
      const { bot, pf } = setup(makeWorld())
      const acting = pf.activate(seeing(bot, new Vec3(20, 0, 0)))
      for (let i = 0; i < 400 && !pf.isMoving(); i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      pf.stop()
      const outcome = await settle(bot, acting)
      expect(outcome.error?.name).to.equal('ActionAborted')
      expect(pf.goal).to.equal(null)
      expect(pf.isMoving()).to.equal(false)
    })

    it('a new goal during an action\'s walk fails it, and the new goal stays', async () => {
      const { bot, pf } = setup(makeWorld())
      const acting = pf.activate(seeing(bot, new Vec3(20, 0, 0)))
      for (let i = 0; i < 400 && !pf.isMoving(); i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      const next = new GoalBlock(-5, 0, 0)
      pf.setGoal(next)
      const outcome = await settle(bot, acting)
      expect(outcome.error?.name).to.equal('ActionAborted')
      expect(pf.goal).to.equal(next)
    })

    it('dispose ends an action in progress and leaves no dig listeners', async () => {
      const world = makeWorld()
      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      const before = new Map(bot.eventNames().map(n => [n, bot.listenerCount(n)]))
      bot.loadPlugin(createPathfinder({
        useWorkerThreads: false,
        physicsFactory: () => makeFakePhysics(world, bot)
      }) as unknown as (b: unknown) => void)
      const pf = (bot as unknown as { pathfinder: Pathfinder }).pathfinder
      pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
      world.set(1, 0, 0, STONE)
      const acting = pf.dig(new Vec3(1, 0, 0))
      const result = settle(bot, acting)
      for (let i = 0; i < 5; i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      pf.dispose()
      const outcome = await result
      expect(outcome.error?.name).to.equal('ActionAborted')
      for (const name of bot.eventNames()) expect(bot.listenerCount(name), String(name)).to.equal(before.get(name) ?? 0)
    })

    it('dispose settles a click that needs ticks, and everything queued behind it', async () => {
      const { bot, pf } = setup(makeWorld())
      let clicked = false
      bot.activateBlock = async () => {
        clicked = true
        await new Promise(() => {})
      }
      const first = pf.activate(new Vec3(1, 0, 0))
      const queued = pf.activate(new Vec3(1, 0, 1))
      for (let i = 0; i < 400 && !clicked; i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(clicked).to.equal(true)
      pf.dispose()
      expect((await first.then(() => null, (e: Error) => e))?.name).to.equal('ActionAborted')
      expect((await queued.then(() => null, (e: Error) => e))?.name).to.equal('ActionAborted')
    })

    it('an abort does not wait for its turn in the queue', async () => {
      const { bot, pf } = setup(makeWorld())
      let clicked = false
      let release = (): void => {}
      bot.activateBlock = async () => {
        clicked = true
        await new Promise<void>(resolve => { release = resolve })
      }
      const first = pf.activate(new Vec3(1, 0, 0))
      const controller = new AbortController()
      const queued = pf.activate(new Vec3(1, 0, 1), { signal: controller.signal })
      for (let i = 0; i < 400 && !clicked; i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(clicked).to.equal(true)
      controller.abort()
      expect((await queued.then(() => null, (e: Error) => e))?.name).to.equal('ActionAborted')
      release()
      expect((await settle(bot, first)).resolved).to.equal(true)
    })

    it('an abort during a sneak wait stops the click', async () => {
      const { bot, pf } = setup(makeWorld())
      const controller = new AbortController()
      let clicks = 0
      bot.activateBlock = async () => { clicks++ }
      const acting = pf.activate(new Vec3(1, 0, 0), { sneak: true, signal: controller.signal })
      const original = bot.setControlState
      bot.setControlState = (name, value) => {
        original(name, value)
        if (name === 'sneak' && value) controller.abort()
      }
      const outcome = await settle(bot, acting)
      expect(outcome.error?.name).to.equal('ActionAborted')
      expect(clicks).to.equal(0)
      expect(bot.controlState.sneak).to.equal(false)
    })

    it('a listener that replaces the goal during a failed result keeps its goal', async () => {
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE)
      const { bot, pf } = setup(world)
      const next = new GoalBlock(-5, 0, 0)
      // Registered before goto's own listener, so it runs first in the same emit.
      bot.once('path_update', () => pf.setGoal(next))
      const outcome = await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(outcome.error?.name).to.equal('GoalChanged')
      for (let i = 0; i < 5; i++) bot.tick()
      expect(pf.goal).to.equal(next)
    })

    it('a stop from a path_update listener drops the result instead of walking it', async () => {
      const { bot, pf } = setup(makeWorld())
      bot.once('path_update', () => pf.stop())
      const outcome = await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      expect(outcome.error?.name).to.equal('PathStopped')
      expect(pf.isMoving()).to.equal(false)
      expect(bot.entity.position.x).to.be.lessThan(1)
    })

    it('a stop while the executor is still aiming a dig never swings', async () => {
      const world = makeWorld()
      const { bot, pf } = setup(world)
      world.fill(2, 0, -8, 2, 8, 24, STONE)
      pf.movements.canDig = true
      pf.setMovements(pf.movements)
      const going = pf.goto(new GoalBlock(4, 0, 0))
      going.catch(() => {})
      for (let i = 0; i < 400 && !pf.isMining(); i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 1))
      }
      expect(pf.isMining()).to.equal(true)
      expect(bot.digs).to.have.length(0)
      pf.stop()
      bot.tick()
      for (let i = 0; i < 40; i++) {
        bot.tick()
        await new Promise(resolve => setTimeout(resolve, 20))
      }
      expect(bot.digs).to.have.length(0)
      expect(pf.isMining()).to.equal(false)
    })

    it('given a block, digs that block and not whatever replaced it', async () => {
      const world = makeWorld()
      const { pf } = setup(world)
      world.set(1, 0, 0, STONE)
      const asked = { name: 'dirt', position: new Vec3(1, 0, 0), type: 0 }
      const error = await pf.dig(asked as never).then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('NoTarget')
    })

    it('dispose from inside an interrupt request refuses the request', async () => {
      const { bot, pf } = setup(makeWorld())
      bot.once('pathfinder:interrupt_requested', () => pf.dispose())
      const error = await pf.interrupt('x', { timeout: 0 }).then(() => null, (e: Error) => e)
      expect(error?.name).to.equal('InterruptAborted')
    })
  })

  describe('who owns the controls', () => {
    /** Counts every control write from here on. */
    function countWrites (bot: DriveableBot): { writes: number } {
      const seen = { writes: 0 }
      const set = bot.setControlState
      const clear = bot.clearControlStates
      bot.setControlState = (name, value) => { seen.writes++; set(name, value) }
      bot.clearControlStates = () => { seen.writes++; clear() }
      return seen
    }

    it('an exclusive interrupt holder gets every control, in water and in the air too', async () => {
      const { bot, pf } = setup(makeWorld())
      const going = pf.goto(new GoalBlock(20, 0, 0))
      going.catch(() => {})
      for (let i = 0; i < 10; i++) bot.tick()
      const granted = pf.interrupt('pvp', { force: true })
      bot.tick()
      const handle = await granted
      const seen = countWrites(bot)
      bot.frozen = true
      bot.entity.isInWater = true
      for (let i = 0; i < 5; i++) bot.tick()
      bot.entity.isInWater = false
      bot.entity.onGround = false
      for (let i = 0; i < 5; i++) bot.tick()
      pf.stop()
      bot.tick()
      expect(seen.writes).to.equal(0)
      handle.release()
    })

    it('with no goal, someone else\'s controls are left alone', async () => {
      const { bot } = setup(makeWorld())
      bot.tick()
      const seen = countWrites(bot)
      bot.frozen = true
      bot.entity.isInWater = true
      for (let i = 0; i < 5; i++) bot.tick()
      bot.entity.isInWater = false
      bot.controlState.left = true
      bot.entity.velocity.set(0.2, 0, 0)
      for (let i = 0; i < 5; i++) bot.tick()
      expect(seen.writes).to.equal(0)
    })

    it('an action inside withInterrupt runs at once instead of waiting for its own caller', async () => {
      const world = makeWorld()
      world.set(1, 0, 0, STONE)
      const { bot, pf } = setup(world)
      const outcome = await settle(bot, pf.withInterrupt('build', async () => {
        await pf.activate(new Vec3(1, 0, 0))
      }))
      expect(outcome.resolved).to.equal(true)
      expect(bot.activations).to.have.length(1)
    })

    it('one that would have to walk there says OutOfReach instead of hanging', async () => {
      const { bot, pf } = setup(makeWorld())
      const outcome = await settle(bot, pf.withInterrupt('build', async () => {
        await pf.activate(seeing(bot, new Vec3(20, 0, 0)))
      }))
      expect(outcome.error?.name).to.equal('OutOfReach')
    })

    it('a goto that fails in mid-air ends its goal once the bot lands', async () => {
      const { bot, pf } = setup(makeWorld())
      const going = pf.goto(new GoalBlock(20, 0, 0))
      going.catch(() => {})
      for (let i = 0; i < 10; i++) bot.tick()
      const stops: unknown[] = []
      bot.on('path_stop', r => stops.push(r))
      bot.frozen = true
      bot.entity.onGround = false
      bot.emit('path_update', { status: 'noPath', path: [] })
      for (let i = 0; i < 3; i++) bot.tick()
      expect(stops).to.deep.equal([])
      expect(pf.goal).to.not.equal(null)
      bot.entity.onGround = true
      bot.tick()
      expect(stops).to.deep.equal(['no_path'])
      expect((await going.then(() => null, (e: Error) => e))?.name).to.equal('NoPath')
    })

    it('an interrupt deadline before login does not throw', async () => {
      const { bot, pf } = setup(makeWorld())
      const entity = bot.entity
      ;(bot as unknown as { entity?: unknown }).entity = undefined
      const handle = await pf.interrupt('early', { timeout: 5 })
      ;(bot as unknown as { entity?: unknown }).entity = entity
      handle.release()
    })

    it('an option passed as undefined keeps its default', async () => {
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE)
      const { bot, pf } = setup(world, { logger: undefined, onNoPath: () => { throw new Error('boom') } })
      const original = console.warn
      console.warn = () => {}
      try {
        expect((await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))).error?.name).to.equal('NoPath')
      } finally {
        console.warn = original
      }
    })

    it('a goal_reached listener can set the next goal', async () => {
      const { bot, pf } = setup(makeWorld())
      const next = new GoalBlock(6, 0, 0)
      bot.once('goal_reached', () => pf.setGoal(next))
      await settle(bot, pf.goto(new GoalBlock(3, 0, 0)))
      expect(pf.goal).to.equal(next)
      for (let i = 0; i < 300 && pf.goal !== null; i++) bot.tick()
      expect(Math.floor(bot.entity.position.x)).to.equal(6)
    })

    it('GoalFollow ends when the entity goes away', async () => {
      const { bot, pf } = setup(makeWorld())
      const entity = { position: new Vec3(15, 0, 0), isValid: true }
      pf.setGoal(new goals.GoalFollow(entity, 1), true)
      for (let i = 0; i < 5; i++) bot.tick()
      entity.isValid = false
      bot.tick()
      expect(pf.goal).to.equal(null)
    })

    it('GoalLookAtBlock accepts a stance below a block within reach', () => {
      const target = new Vec3(0, 4, 2)
      const world = { raycast: () => ({ position: target, face: 0 }) }
      const goal = new goals.GoalLookAtBlock(target, world as never, { reach: 4.5 })
      expect(goal.isEnd({ x: 0, y: 0, z: 0 })).to.equal(true)
      expect(goal.isEnd({ x: 0, y: -6, z: 0 })).to.equal(false)
    })
  })

  describe('computePath', () => {
    it('plans without moving, and leaves the goal alone', async () => {
      const { bot, pf } = setup(makeWorld())
      const goal = new GoalBlock(9, 0, 0)
      pf.setGoal(goal)
      const moved = bot.entity.position.clone()
      const result = await pf.computePath(new GoalBlock(4, 0, 3))
      expect(result.status).to.equal('success')
      expect(result.path.length).to.be.greaterThan(0)
      const last = result.path[result.path.length - 1]
      expect(Math.floor(last.x)).to.equal(4)
      expect(Math.floor(last.z)).to.equal(3)
      expect(pf.goal).to.equal(goal)
      expect(bot.entity.position.distanceTo(moved)).to.equal(0)
    })

    it('plans from elsewhere with another profile', async () => {
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE) // a wall only a digger gets through
      const { bot, pf } = setup(world)
      const from = new Vec3(3.5, 0, 0.5)
      const walled = await pf.computePath(new GoalBlock(7, 0, 0), { from })
      expect(walled.status).to.equal('noPath')
      const digger = applyProfile(new Movements(bot as never) as never) as unknown as Movements
      digger.canDig = true
      const dug = await pf.computePath(new GoalBlock(7, 0, 0), { from, movements: digger })
      expect(dug.status).to.equal('success')
      expect(Math.floor(dug.path[0].x)).to.be.greaterThan(2)
    })

    it('rejects an aborted search with the reason', async () => {
      const { pf } = setup(makeWorld())
      const reason = new Error('no longer needed')
      const error = await pf.computePath(new GoalBlock(4, 0, 0), { signal: AbortSignal.abort(reason) }).then(() => null, (e: Error) => e)
      expect(error).to.equal(reason)
    })

    it('runs on the worker', async function () {
      if (!existsSync(workerEntry)) this.skip()
      const world = makeWorld()
      const { pf } = setup(world, { useWorkerThreads: true, workerEntryPath: workerEntry })
      const result = await pf.computePath(new GoalBlock(9, 0, 4))
      expect(result.status).to.equal('success')
      expect(result.path.length).to.be.greaterThan(0)
    })

    it('honours an abort fired right after the call, on the worker', async function () {
      if (!existsSync(workerEntry)) this.skip()
      const { pf } = setup(makeWorld(), { useWorkerThreads: true, workerEntryPath: workerEntry })
      const controller = new AbortController()
      const reason = new Error('never mind')
      const searching = pf.computePath(new GoalBlock(9, 0, 4), { signal: controller.signal })
      controller.abort(reason)
      expect(await searching.then(() => null, (e: Error) => e)).to.equal(reason)
    })
  })

  describe('actions.wrap', () => {
    it('layers, passes through, and comes off in any order', async () => {
      const { pf } = setup(makeWorld())
      const calls: string[] = []
      const base = pf.actions.equip
      pf.actions.equip = async () => { calls.push('base') }
      const offA = pf.actions.wrap('equip', next => async (i, d) => { calls.push('a'); return await next(i, d) })
      const offB = pf.actions.wrap('equip', next => async (i, d) => { calls.push('b'); return await next(i, d) })
      await pf.actions.equip(null)
      expect(calls).to.deep.equal(['b', 'a', 'base'])
      calls.length = 0
      offA()
      await pf.actions.equip(null)
      expect(calls).to.deep.equal(['b', 'base'])
      calls.length = 0
      offB()
      await pf.actions.equip(null)
      expect(calls).to.deep.equal(['base'])
      pf.actions.equip = base
    })

    it('leaves nothing behind, whatever the order of removal', async () => {
      const { pf } = setup(makeWorld())
      const base = async (): Promise<void> => {}
      pf.actions.equip = base
      const offA = pf.actions.wrap('equip', next => next)
      const offB = pf.actions.wrap('equip', next => next)
      offA()
      offB()
      expect(pf.actions.equip).to.equal(base)
      offB() // Idempotent.
      expect(pf.actions.equip).to.equal(base)
    })

    it('does not grow when wrappers replace each other in turn', async () => {
      const { pf } = setup(makeWorld())
      let calls = 0
      pf.actions.equip = async () => { calls++ }
      let off = pf.actions.wrap('equip', next => next)
      for (let i = 0; i < 20000; i++) {
        const nextOff = pf.actions.wrap('equip', next => next)
        off()
        off = nextOff
      }
      // A chain of removed layers would overflow the stack here.
      await pf.actions.equip(null)
      expect(calls).to.equal(1)
    })

    it('a reference kept to a removed layer still reaches what is below it', async () => {
      const { pf } = setup(makeWorld())
      let calls = 0
      pf.actions.equip = async () => { calls++ }
      const off = pf.actions.wrap('equip', () => async () => { throw new Error('removed') })
      const kept = pf.actions.equip
      off()
      await kept(null)
      expect(calls).to.equal(1)
    })

    it('a wrapper can veto', async () => {
      const { pf } = setup(makeWorld())
      pf.actions.wrap('activate', () => async () => { throw new Error('not here') })
      const error = await pf.actions.activate({} as never).then(() => null, (e: Error) => e)
      expect(error?.message).to.equal('not here')
    })
  })

  describe('logger', () => {
    it('receives warnings instead of the console', async () => {
      const lines: string[] = []
      const world = makeWorld()
      world.fill(5, 0, -8, 5, 8, 24, STONE)
      const { bot, pf } = setup(world, {
        logger: { warn: (m: string) => lines.push(m) },
        onNoPath: () => { throw new Error('boom') }
      })
      const original = console.warn
      let consoled = 0
      console.warn = () => { consoled++ }
      try {
        await settle(bot, pf.goto(new GoalBlock(9, 0, 0)))
      } finally {
        console.warn = original
      }
      expect(consoled).to.equal(0)
      expect(lines.some(l => l.includes('boom'))).to.equal(true)
    })
  })

  describe('applyPreset', () => {
    it('parkour turns the engine on, default turns it off, and both return the profile', () => {
      const { pf } = setup(makeWorld())
      const m = pf.movements
      expect(m.applyPreset('parkour')).to.equal(m)
      for (const flag of ['allowParkourExtended', 'allowParkourMomentum', 'allowParkourPhysics', 'allowSprintHop', 'allowLowCeilingHop', 'allowRunUp', 'allowLandingRetire', 'allowHopIntoJump', 'allowStrafe'] as const) {
        expect(m[flag], flag).to.equal(true)
      }
      m.applyPreset('default')
      for (const flag of ['allowParkourExtended', 'allowParkourPhysics', 'allowSprintHop', 'allowStrafe'] as const) {
        expect(m[flag], flag).to.equal(false)
      }
      expect(m.canDig).to.equal(false)
    })

    it('refuses a name it does not know', () => {
      const { pf } = setup(makeWorld())
      expect(() => pf.movements.applyPreset('fast' as never)).to.throw('Unknown movement preset')
    })
  })

  describe('vine climbing follows the bot\'s physics', () => {
    const ids = ['weeping_vines', 'cave_vines'].map(n => mcData.blocksByName[n]?.id).filter(id => id !== undefined)

    /** A physics whose vines do what stock prismarine-physics does: nothing. */
    function stock (): { simulatePlayer: (s: { vel: Vec3 }) => void } {
      return { simulatePlayer: (s) => { s.vel.y = (s.vel.y - 0.08) * 0.98 } }
    }

    it('reads the real physics in this repository as patched', () => {
      expect(physicsClimbsVines({ physics: Physics(mcData, null), registry: mcData })).to.equal(ids.length > 0)
    })

    it('leaves nether and cave vines out of plans when the physics cannot climb them', function () {
      if (ids.length === 0) this.skip()
      const world = makeWorld()
      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      bot.physics = stock() as never
      const movements = new Movements(bot as never)
      for (const id of ids) expect(movements.climbables.has(id as number)).to.equal(false)
      expect(movements.climbables.has(mcData.blocksByName.vine.id)).to.equal(true)
    })

    it('keeps the kernel out of them too', function () {
      if (ids.length === 0) this.skip()
      const world = makeWorld()
      const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      bot.physics = stock() as never
      const lut = getLut(bot as never, new Movements(bot as never))
      expect(lut.simKind[mcData.blocksByName.weeping_vines.minStateId as number] & SIM_CLIMBABLE).to.equal(0)
      expect(lut.simKind[mcData.blocksByName.ladder.minStateId as number] & SIM_CLIMBABLE).to.equal(SIM_CLIMBABLE)
    })

    it('assumes yes for a stand-in physics that does nothing', () => {
      expect(physicsClimbsVines({ physics: { simulatePlayer: () => {} }, registry: mcData })).to.equal(true)
    })

    it('gives the two physics different block tables, even for identical profiles', function () {
      if (ids.length === 0) this.skip()
      const world = makeWorld()
      const stockBot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      stockBot.physics = stock() as never
      const realBot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      realBot.physics = Physics(mcData, null)
      const stockMoves = new Movements(stockBot as never)
      const realMoves = new Movements(realBot as never)
      realMoves.climbables = new Set(stockMoves.climbables)
      expect(realMoves.lutFingerprint()).to.not.equal(stockMoves.lutFingerprint())
    })

    it('the executor counts a catch only on what the physics climbs', function () {
      if (ids.length === 0) this.skip()
      const names = (bot: unknown): (name: string) => boolean => {
        const physics = new PhysicsSim(bot as never) as unknown as { climbableName: (name: string) => boolean }
        return (name) => physics.climbableName(name)
      }
      const world = makeWorld()
      const stockBot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      stockBot.physics = stock() as never
      const realBot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
      realBot.physics = Physics(mcData, null)
      expect(names(stockBot)('weeping_vines')).to.equal(false)
      expect(names(stockBot)('vine')).to.equal(true)
      expect(names(stockBot)('ladder')).to.equal(true)
      expect(names(realBot)('weeping_vines')).to.equal(true)
      expect(names(realBot)('stone')).to.equal(false)
    })
  })
})
