// Executor-level tests: the full plugin driving a fake bot through fake
// voxel physics. Covers the goto() promise contract (verbatim upstream error
// names — the bulbastore wrapper string-matches them), events, stop()
// semantics, replan-on-block-update, and the improvement knobs.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { GoalBlock, GoalNear } from '../src/goals.js'
import { VoxelWorld, STONE, AIR, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

interface PF {
  goto: (goal: unknown, options?: { bestEffort?: boolean, maxBestEffortLegs?: number }) => Promise<void>
  setGoal: (goal: unknown, dynamic?: boolean) => void
  setMovements: (m: Movements) => void
  stop: () => void
  isMoving: () => boolean
  isMining: () => boolean
  isBuilding: () => boolean
  stuckTimeout: number
  executionTimeout: number
  searchRadius: number
  thinkTimeout: number
  goal: unknown
  movements: Movements
}

function makeWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 24, y1: 8, z1: 24 })
  world.fill(-8, -1, -8, 24, -1, 24, STONE) // floor at y=-1, walkable at y=0
  return world
}

function setup (world: VoxelWorld, start = new Vec3(0.5, 0, 0.5)): { bot: DriveableBot, pf: PF } {
  const bot = makeDriveableBot(world, start)
  const plugin = createPathfinder({
    useWorkerThreads: false, // main-thread solving in executor tests
    physicsFactory: () => makeFakePhysics(world, bot)
  })
  bot.loadPlugin(plugin as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: PF }).pathfinder
  const movements = applyProfile(new Movements(bot as never) as never) as unknown as Movements
  pf.setMovements(movements)
  return { bot, pf }
}

/** Drive ticks until the promise settles or the tick budget runs out. */
async function driveUntilSettled<T> (
  bot: DriveableBot,
  promise: Promise<T>,
  maxTicks = 2000,
  onTick?: (tick: number) => void
): Promise<{ status: 'resolved' | 'rejected', value?: T, error?: Error }> {
  let settled: { status: 'resolved' | 'rejected', value?: T, error?: Error } | null = null
  promise.then(
    (value) => { settled = { status: 'resolved', value } },
    (error) => { settled = { status: 'rejected', error } }
  )
  for (let i = 0; i < maxTicks && !settled; i++) {
    onTick?.(i)
    bot.tick()
    await new Promise(resolve => setImmediate(resolve))
    await new Promise(resolve => setTimeout(resolve, 1))
  }
  if (!settled) throw new Error(`goto did not settle within ${maxTicks} ticks (bot at ${bot.entity.position})`)
  return settled
}

describe('plugin executor', function () {
  this.timeout(60000)

  it('goto resolves on a flat walk and releases all controls', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(8, 0, 0)))
    expect(outcome.status).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(8.5, 0, 0.5))).to.be.lessThan(1.2)
    expect(Object.values(bot.controlState).some(v => v)).to.equal(false)
    expect(pf.goal).to.equal(null)
    expect(pf.isMoving()).to.equal(false)
  })

  it('emits path_update (success) and goal_reached', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    // NOTE: results.path is the LIVE path array (upstream aliases it the
    // same way and drains it as the bot walks) — snapshot at emit time.
    const updates: Array<{ status: string, pathLen: number, time: number, visitedNodes: number }> = []
    let reached = 0
    bot.on('path_update', ((r: { status: string, path: unknown[], time: number, visitedNodes: number }) => {
      updates.push({ status: r.status, pathLen: r.path.length, time: r.time, visitedNodes: r.visitedNodes })
    }) as never)
    bot.on('goal_reached', () => reached++)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(6, 0, 3)))
    expect(outcome.status).to.equal('resolved')
    expect(reached).to.equal(1)
    const success = updates.filter(u => u.status === 'success')
    expect(success.length).to.be.greaterThan(0)
    expect(success[0].pathLen).to.be.greaterThan(0)
    expect(success[0].time).to.be.a('number')
    expect(success[0].visitedNodes).to.be.a('number')
  })

  it('rejects NoPath (verbatim name/message) for a walled goal', async () => {
    const world = makeWorld()
    // Box the goal in with a 3-high wall, hollow inside.
    world.fill(9, 0, -3, 15, 2, 3, STONE)
    world.fill(11, 0, -1, 13, 2, 1, AIR)
    const { bot, pf } = setup(world)
    pf.searchRadius = 64
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(12, 0, 0)))
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('NoPath')
    expect(outcome.error?.message).to.equal('No path to the goal!')
  })

  it('bestEffort resolves at the closest reachable cell instead of rejecting NoPath', async () => {
    const world = makeWorld()
    // Same walled goal as the NoPath test above: unreachable, but the solver
    // still hands back the path to its closest node.
    world.fill(9, 0, -3, 15, 2, 3, STONE)
    world.fill(11, 0, -1, 13, 2, 1, AIR)
    const { bot, pf } = setup(world)
    pf.searchRadius = 64
    const start = bot.entity.position.clone()
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(12, 0, 0), { bestEffort: true }))
    expect(outcome.status).to.equal('resolved')
    // It walked: away from the start, and up against the wall it cannot pass
    // (the wall's near face is x=9, so the last standable cell is x=8).
    const end = bot.entity.position
    expect(end.distanceTo(start)).to.be.greaterThan(4)
    expect(end.x).to.be.greaterThan(6)
    expect(end.x).to.be.lessThan(9)
    expect(Object.values(bot.controlState).some(v => v)).to.equal(false)
  })

  it('bestEffort still resolves normally when the goal IS reachable', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(8, 0, 0), { bestEffort: true }))
    expect(outcome.status).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(8.5, 0, 0.5))).to.be.lessThan(1.2)
    expect(pf.goal).to.equal(null)
  })

  it('bestEffort is off by default (NoPath contract is unchanged)', async () => {
    const world = makeWorld()
    world.fill(9, 0, -3, 15, 2, 3, STONE)
    world.fill(11, 0, -1, 13, 2, 1, AIR)
    const { bot, pf } = setup(world)
    pf.searchRadius = 64
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(12, 0, 0), {}))
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('NoPath')
  })

  it('stop() rejects PathStopped within a tick and clears controls', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const promise = pf.goto(new GoalBlock(20, 0, 20))
    // Stop after 15 ticks of walking — well before the ~70-tick arrival.
    let stopped = false
    const outcome = await driveUntilSettled(bot, promise, 2000, (tick) => {
      if (tick === 15) {
        pf.stop()
        stopped = true
      }
    })
    expect(stopped).to.equal(true)
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('PathStopped')
    expect(Object.values(bot.controlState).some(v => v)).to.equal(false)
  })

  it('stop() while idle does not poison the next goal (upstream quirk fixed)', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    pf.stop() // idle stop — must be a no-op
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(5, 0, 0)))
    expect(outcome.status).to.equal('resolved')
  })

  it('changing the goal rejects GoalChanged', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const promise = pf.goto(new GoalBlock(20, 0, 20))
    const outcomePromise = driveUntilSettled(bot, promise)
    setTimeout(() => pf.setGoal(new GoalBlock(1, 0, 1)), 100)
    const outcome = await outcomePromise
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('GoalChanged')
  })

  it('goto resolves immediately when already at the goal', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalNear(0, 0, 0, 2)))
    expect(outcome.status).to.equal('resolved')
  })

  it('stuckTimeout: a frozen bot rejects PathStopped instead of hanging', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    pf.stuckTimeout = 250
    bot.frozen = true // physics disabled — bot cannot move
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(10, 0, 0)))
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('PathStopped')
    expect(Object.values(bot.controlState).some(v => v)).to.equal(false)
  })

  it('re-solves when a path runs out short of the goal instead of freezing', async () => {
    // The 2b2t arena benchmark caught this: a bot that walks a delivered path
    // to exhaustion WITHOUT reaching the goal used to stand still forever.
    // Upstream gates the recompute on `!pathUpdated`, which latches true as
    // soon as any final path is delivered, and the futility timer is reset on
    // every empty-path tick so it can never fire either. Measured live: 83 s
    // of a 97 s run motionless, and runs abandoned 29 blocks short.
    //
    // Reproduced by moving the target out from under the bot mid-walk while
    // reporting hasChanged() === false, so nothing triggers a replan: the bot
    // consumes the last node of a delivered `success` path, finds isEnd()
    // false there, and is left with an empty path and an unmet goal. That is
    // exactly the state the arena runs ended in.
    class MovingTarget {
      constructor (public x: number, public y: number, public z: number) {}
      heuristic (node: { x: number, y: number, z: number }): number {
        return Math.hypot(this.x - node.x, this.z - node.z) + Math.abs(this.y - node.y)
      }

      isEnd (node: { x: number, y: number, z: number }): boolean {
        return node.x === this.x && node.y === this.y && node.z === this.z
      }

      isValid (): boolean { return true }
      hasChanged (): boolean { return false } // never asks for a replan
    }

    const world = makeWorld()
    const { bot, pf } = setup(world)
    const goal = new MovingTarget(8, 0, 0)
    const resets: string[] = []
    bot.on('path_reset', ((reason: string) => { resets.push(reason) }) as never)

    let moved = false
    const outcome = await driveUntilSettled(bot, pf.goto(goal), 4000, () => {
      if (!moved && bot.entity.position.x > 6) { moved = true; goal.x = 14 }
    })
    expect(moved, 'the target must actually move mid-walk').to.equal(true)
    expect(outcome.status).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(14.5, 0, 0.5))).to.be.lessThan(1.2)
    // Recovery must come from noticing the exhausted path, not from a reset.
    expect(resets, `unexpected replan: ${resets.join(',')}`).to.deep.equal([])
  })

  it('a wedged bot still trips futility while the server keeps correcting it', async function () {
    // The forcedMove handler refreshed the futility timer on every server
    // position correction ("a correction is not stuck"), which is right for a
    // lagback and catastrophic for a bot WEDGED in geometry: those get
    // corrected every tick, so the 3.5 s timer was reset before it could ever
    // expire. Measured on 2b2t spawn: 46 s motionless with 34 path nodes
    // left, no `stuck` reset, same spot every run, while upstream — which has
    // no such handler — walked the route fine.
    //
    // Real time, not ticks: the grace window and the futility timer are both
    // wall-clock.
    this.timeout(40000)
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const resets: string[] = []
    bot.on('path_reset', ((reason: string) => { resets.push(reason) }) as never)

    const promise = pf.goto(new GoalBlock(12, 0, 0))
    promise.catch(() => {}) // the goal never completes here; not what we assert

    const deadline = Date.now() + 20000
    let ticks = 0
    while (Date.now() < deadline && !resets.includes('stuck')) {
      if (ticks === 20) bot.frozen = true // wedged: physics can no longer move it
      if (ticks > 20) bot.emit('forcedMove' as never) // ... and the server nudges it
      bot.tick()
      ticks++
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }
    pf.stop()
    expect(resets, 'corrections must not postpone the verdict forever').to.include('stuck')
  })

  it('a genuinely unreachable goal does not spin on re-solves', async () => {
    // The recompute above is gated on the bot having MOVED since the last
    // solve, so a stationary bot facing an impossible goal must not re-solve
    // every tick.
    const world = makeWorld()
    world.fill(9, 0, -8, 9, 2, 24, STONE) // full-height wall, no way around
    const { bot, pf } = setup(world)
    let finals = 0
    bot.on('path_update', ((r: { status: string }) => { if (r.status !== 'partial') finals++ }) as never)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(14, 0, 0)))
    expect(outcome.status).to.equal('rejected')
    expect(outcome.error?.name).to.equal('NoPath')
    expect(finals, 'one noPath verdict, not a per-tick storm').to.be.lessThan(3)
  })

  it('replans around a wall dropped onto the path (path_reset + still arrives)', async () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const resets: string[] = []
    bot.on('path_reset', (reason: never) => resets.push(reason as unknown as string))

    const promise = pf.goto(new GoalBlock(14, 0, 0))

    // After ~10 ticks (bot around x≈2-3), drop a wall across the corridor
    // ahead of it and emit the corresponding blockUpdates.
    const outcome = await driveUntilSettled(bot, promise, 4000, (tick) => {
      if (tick !== 10) return
      for (let z = -4; z <= 4; z++) {
        for (let y = 0; y <= 2; y++) {
          const pos = new Vec3(9, y, z)
          const oldBlock = bot.blockAt(pos)
          world.set(9, y, z, STONE)
          const newBlock = bot.blockAt(pos)
          bot.emit('blockUpdate', oldBlock as never, newBlock as never)
        }
      }
    })
    expect(outcome.status).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(14.5, 0, 0.5))).to.be.lessThan(1.5)
    expect(resets.length).to.be.greaterThan(0)
  })

  it('canDig defaults to false and is accepted by setMovements when enabled', () => {
    const world = makeWorld()
    const { bot, pf } = setup(world)
    const fresh = new Movements(bot as never)
    expect(fresh.canDig).to.equal(false) // divergence from upstream default: opt-in
    fresh.canDig = true
    expect(() => pf.setMovements(fresh)).to.not.throw()
  })

  it('isMining is always false; isBuilding false when idle', () => {
    const world = makeWorld()
    const { pf } = setup(world)
    expect(pf.isMining()).to.equal(false)
    expect(pf.isBuilding()).to.equal(false)
  })
})
