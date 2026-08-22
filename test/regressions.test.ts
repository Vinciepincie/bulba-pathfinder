// Regression tests for defects found by the adversarial review — each one
// reproduced against the pre-fix code before being fixed.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { Goal, GoalBlock, GoalNear } from '../src/goals.js'
import { Solver } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import {
  VoxelWorld, STONE, AIR, mcData, applyProfile, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld
} from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

/** A goal whose weak heuristic forces a long multi-slice main-thread solve. */
class SlowGoal extends Goal {
  constructor (private readonly gx: number, private readonly gy: number, private readonly gz: number) { super() }
  heuristic (n: { x: number, y: number, z: number }): number {
    const dx = Math.abs(this.gx - n.x)
    const dz = Math.abs(this.gz - n.z)
    return 0.05 * (Math.abs(dx - dz) + Math.min(dx, dz) * Math.SQRT2 + Math.abs(this.gy - n.y))
  }

  isEnd (n: { x: number, y: number, z: number }): boolean {
    return n.x === this.gx && n.y === this.gy && n.z === this.gz
  }
}

interface PF extends Record<string, unknown> {
  setGoal: (g: Goal | null, dynamic?: boolean) => void
  setMovements: (m: Movements) => void
  goto: (g: Goal) => Promise<void>
  isMoving: () => boolean
  tickTimeout: number
}

function setup (world: VoxelWorld, start = new Vec3(0.5, 0, 0.5)): { bot: DriveableBot, pf: PF } {
  const bot = makeDriveableBot(world, start)
  const plugin = createPathfinder({ useWorkerThreads: false, physicsFactory: () => makeFakePhysics(world, bot) })
  bot.loadPlugin(plugin as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: PF }).pathfinder
  pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)
  return { bot, pf }
}

describe('review regressions', function () {
  this.timeout(60000)

  it('R1: a block update queued during a solve never overwrites a newer write (patch inversion)', async () => {
    const world = new VoxelWorld({ x0: -40, y0: -20, z0: -40, x1: 50, y1: 20, z1: 50 })
    world.fill(-40, -1, -40, 50, -1, 50, STONE)
    const { bot, pf } = setup(world)

    const updates: Array<{ status: string, path: Array<{ x: number, y: number, z: number }> }> = []
    bot.on('path_update', ((r: { status: string, path: Array<{ x: number, y: number, z: number }> }) => {
      updates.push({ status: r.status, path: r.path.map(n => ({ x: n.x, y: n.y, z: n.z })) })
    }) as never)

    const emitUpdate = (x: number, y: number, z: number, state: number): void => {
      const pos = new Vec3(x, y, z)
      const oldBlock = bot.blockAt(pos)
      world.set(x, y, z, state)
      const newBlock = bot.blockAt(pos)
      bot.emit('blockUpdate', oldBlock as never, newBlock as never)
    }
    const tick = async (): Promise<void> => {
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
    }

    // Slow multi-slice solve in flight…
    pf.tickTimeout = 0.001
    pf.setGoal(new SlowGoal(0, 0, 10))
    await tick()

    // …a wall appears (queued against the busy snapshot)…
    emitUpdate(7, 0, 0, STONE)
    emitUpdate(7, 1, 0, STONE)

    for (let i = 0; i < 500 && !updates.some(u => u.status === 'success'); i++) await tick()
    expect(updates.some(u => u.status === 'success'), 'slow solve should finish').to.equal(true)

    // …and disappears again after the solve completed (newer write).
    emitUpdate(7, 0, 0, AIR)
    emitUpdate(7, 1, 0, AIR)

    // A fresh goal must see the CURRENT world: straight through (7,0,0).
    bot.entity.position.set(0.5, 0, 0.5)
    bot.entity.velocity.set(0, 0, 0)
    bot.entity.onGround = true
    updates.length = 0
    pf.tickTimeout = 40
    pf.setGoal(new GoalBlock(14, 0, 0))
    for (let i = 0; i < 200 && !updates.some(u => u.status !== 'partial'); i++) await tick()
    const final = updates.find(u => u.status !== 'partial')
    pf.setGoal(null)

    expect(final, 'GoalBlock must produce a result').to.not.equal(undefined)
    expect(final!.status).to.equal('success')
    const through = final!.path.some(n => Math.floor(n.x) === 7 && Math.floor(n.y) === 0 && Math.floor(n.z) === 0)
    expect(through, `path must go straight through the (re-)opened cell, got ${JSON.stringify(final!.path.map(n => [n.x, n.y, n.z]))}`).to.equal(true)
  })

  it('R2: goal_reached cancels an in-flight soft-replan solve — no zombie path afterwards', async () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 40, y1: 8, z1: 40 })
    world.fill(-8, -1, -8, 40, -1, 40, STONE)
    const { bot, pf } = setup(world)

    const postGoalUpdates: string[] = []
    let reached = 0
    bot.on('goal_reached', () => reached++)
    bot.on('path_update', ((r: { status: string }) => {
      if (reached > 0) postGoalUpdates.push(r.status)
    }) as never)

    const tick = async (): Promise<void> => {
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }

    const promise = pf.goto(new GoalBlock(10, 0, 0))
    let settled = false
    promise.then(() => { settled = true }, () => { settled = true })

    // Let it compute + start walking.
    for (let i = 0; i < 10 && !settled; i++) await tick()

    // Trigger a soft replan that leaves a SLOW main-thread solve in flight
    // while the bot keeps walking the kept path: shrink the slice budget and
    // poke a path-adjacent (but not imminent) block.
    pf.tickTimeout = 0.001
    const pos = new Vec3(8, 2, 1) // near the path, not imminent, not blocking
    const oldBlock = bot.blockAt(pos)
    world.set(8, 2, 1, STONE)
    bot.emit('blockUpdate', oldBlock as never, bot.blockAt(pos) as never)

    for (let i = 0; i < 2000 && !settled; i++) await tick()
    expect(settled, 'goto should settle').to.equal(true)
    expect(reached).to.equal(1)

    // Keep ticking: the (cancelled) replan solve must never re-install a
    // path on a goal-less pathfinder.
    for (let i = 0; i < 80; i++) await tick()
    expect(pf.isMoving(), 'no zombie path after goal_reached').to.equal(false)
    expect(postGoalUpdates, 'no path_update after goal completion').to.deep.equal([])
  })

  it('R3: Solver.compute() after a terminal result returns the same snapshot, never a fabricated path', () => {
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 20, y1: 6, z1: 20 })
    world.fill(-2, -1, -2, 20, -1, 20, STONE)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot)
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)

    const s1 = new Solver(snap, movements.toConfig(), new GoalAdapter(new GoalBlock(5, 0, 5)), { x: 0, y: 0, z: 0 }, { timeout: 5000, searchRadius: -1 })
    let r1 = s1.compute(1e9)
    while (r1.status === 'partial') r1 = s1.compute(1e9)
    expect(r1.status).to.equal('success')

    // A second solver re-acquires (and possibly regrows) the arena.
    const bigWorld = new VoxelWorld({ x0: -30, y0: -4, z0: -30, x1: 60, y1: 12, z1: 60 })
    bigWorld.fill(-30, -1, -30, 60, -1, 60, STONE)
    const bigBot = makeFakeBot(bigWorld)
    const bigSnap = snapshotFromWorld(bigWorld, lutFor(bigBot, makeOurMovements(bigBot)))
    const s2 = new Solver(bigSnap, movements.toConfig(), new GoalAdapter(new GoalBlock(40, 0, 40)), { x: 0, y: 0, z: 0 }, { timeout: 5000, searchRadius: -1 })
    let r2 = s2.compute(1e9)
    while (r2.status === 'partial') r2 = s2.compute(1e9)
    expect(r2.status).to.equal('success')

    // compute() on the FINISHED first solver must return its own result —
    // not a path fabricated from the second solve's arena tables.
    const again = s1.compute(1e9)
    expect(again.status).to.equal(r1.status)
    expect(again.cost).to.equal(r1.cost)
    expect(again.path.map(p => `${p.x},${p.y},${p.z}`)).to.deep.equal(r1.path.map(p => `${p.x},${p.y},${p.z}`))
  })
})

describe('lagback (forcedMove) reaction', function () {
  this.timeout(60000)

  it('a small landing-correction lagback splices and keeps walking (no reset, no futility wait)', async () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 30, y1: 8, z1: 8 })
    world.fill(-8, -1, -8, 30, -1, 8, STONE)
    const { bot, pf } = setup(world)

    let resets = 0
    bot.on('path_reset', () => resets++)

    const promise = pf.goto(new GoalBlock(18, 0, 0))
    let settled: string | null = null
    promise.then(() => { settled = 'resolved' }, (e: Error) => { settled = `rejected:${e.name}` })

    let lagbacked = false
    const started = Date.now()
    for (let i = 0; i < 3000 && !settled; i++) {
      if (!lagbacked && bot.entity.position.x > 10) {
        // The common micro-lagback: a sub-block landing correction.
        bot.entity.position.set(bot.entity.position.x - 0.9, 0, bot.entity.position.z)
        bot.entity.velocity.set(0, 0, 0)
        bot.emit('forcedMove')
        lagbacked = true
      }
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }

    expect(lagbacked).to.equal(true)
    expect(settled).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(18.5, 0, 0.5))).to.be.lessThan(1.5)
    // On-path corrections splice and keep walking — no reset, and no 3.5s
    // futility wait (generous bound: well under the futility window).
    expect(resets).to.equal(0)
    expect(Date.now() - started).to.be.lessThan(3000)
  })

  it('a far sideways lagback replans immediately (path_reset forced_move) and still arrives', async () => {
    const world = new VoxelWorld({ x0: -8, y0: -2, z0: -20, x1: 30, y1: 8, z1: 20 })
    world.fill(-8, -1, -20, 30, -1, 20, STONE)
    const { bot, pf } = setup(world)

    const reasons: string[] = []
    bot.on('path_reset', ((r: string) => reasons.push(r)) as never)

    const promise = pf.goto(new GoalBlock(18, 0, 0))
    let settled: string | null = null
    promise.then(() => { settled = 'resolved' }, (e: Error) => { settled = `rejected:${e.name}` })

    let lagbacked = false
    for (let i = 0; i < 3000 && !settled; i++) {
      if (!lagbacked && bot.entity.position.x > 8) {
        // Snap far off the path (15 blocks sideways).
        bot.entity.position.set(bot.entity.position.x, 0, 15.5)
        bot.entity.velocity.set(0, 0, 0)
        bot.emit('forcedMove')
        lagbacked = true
      }
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }

    expect(lagbacked).to.equal(true)
    expect(settled).to.equal('resolved')
    expect(reasons).to.include('forced_move')
    expect(bot.entity.position.distanceTo(new Vec3(18.5, 0, 0.5))).to.be.lessThan(1.5)
  })
})

describe('onNoPath capture hook', function () {
  this.timeout(20000)

  it('fires once per terminal no-path with the exact solver inputs', async () => {
    const world = new VoxelWorld({ x0: -10, y0: -2, z0: -10, x1: 20, y1: 8, z1: 20 })
    world.fill(-10, -1, -10, 20, -1, 20, STONE)
    // Wall off the goal so every solve is a genuine no-path.
    world.fill(8, 0, -10, 8, 4, 20, STONE)
    const dumps: Array<{ start: { x: number, y: number, z: number }, visitedNodes: number, flagsLen: number }> = []
    const bot = makeDriveableBot(world, new Vec3(0.5, 0, 0.5))
    const plugin = createPathfinder({
      useWorkerThreads: false,
      physicsFactory: () => makeFakePhysics(world, bot),
      onNoPath: d => dumps.push({ start: d.start, visitedNodes: d.visitedNodes, flagsLen: d.flags.length })
    })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: PF }).pathfinder
    pf.setMovements(applyProfile(new Movements(bot as never) as never) as unknown as Movements)

    const promise = pf.goto(new GoalBlock(15, 0, 0))
    let settled = false
    promise.then(() => { settled = true }, () => { settled = true })
    for (let i = 0; i < 3000 && !settled; i++) {
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
    }
    expect(settled, 'goto must settle').to.equal(true)
    expect(dumps.length).to.be.at.least(1)
    expect(dumps[0].start).to.deep.equal({ x: 0, y: 0, z: 0 })
    expect(dumps[0].visitedNodes).to.be.greaterThan(0)
    expect(dumps[0].flagsLen).to.be.greaterThan(1000)
  })
})

// Thin-floor (carpet) landings and the flight-curve corridor — the prod
// storage-room "No path" (2026-08-20): a fully carpeted pedestal grid made
// every diagonal/drop/extended-parkour landing float one cell up in the air,
// and drop-jumps demanded full landing-depth clearance over same-level
// corners near takeoff.
describe('thin floors + flight-curve corridor', function () {
  this.timeout(20000)
  const CARPET = mcData.blocksByName.gray_carpet.minStateId as number

  interface XYZ { x: number, y: number, z: number }

  function rawSolve (world: VoxelWorld, start: XYZ, goal: Goal): { status: string, path: Array<{ x: number, y: number, z: number, parkour?: boolean }> } {
    const bot = makeFakeBot(world, new Vec3(start.x + 0.5, start.y, start.z + 0.5))
    const movements = makeOurMovements(bot, { allowParkourExtended: true })
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), start, { timeout: 5000, searchRadius: 64 })
    let res = solver.compute(1000)
    while (res.status === 'partial') res = solver.compute(1000)
    return res as never
  }

  /** Carpeted pedestal grid: 3x3 pedestals (walk 1) with 1-wide channels
   * (walk 0), carpet on EVERY walk cell — the prod storage-room floor. */
  function carpetedPedestalRoom (): VoxelWorld {
    const world = new VoxelWorld({ x0: -8, y0: -6, z0: -8, x1: 24, y1: 12, z1: 24 })
    world.fill(-8, -1, -8, 24, -1, 24, STONE)
    for (let px = 0; px < 5; px++) {
      for (let pz = 0; pz < 5; pz++) {
        world.fill(px * 4, 0, pz * 4, px * 4 + 2, 0, pz * 4 + 2, STONE)
      }
    }
    for (let x = -8; x <= 24; x++) {
      for (let z = -8; z <= 24; z++) {
        const onPed = x >= 0 && z >= 0 && x <= 18 && z <= 18 && x % 4 !== 3 && z % 4 !== 3
        world.set(x, onPed ? 1 : 0, z, CARPET)
      }
    }
    return world
  }

  it('carpeted pedestal grid solves with parkour at the true walk level (no floating nodes)', () => {
    const world = carpetedPedestalRoom()
    const res = rawSolve(world, { x: 1, y: 1, z: 1 }, new GoalNear(17, 1, 17, 2))
    expect(res.status).to.equal('success')
    expect(res.path.some(n => n.parkour === true), 'must cross pedestals by jumping').to.equal(true)
    for (const n of res.path) {
      // Real support: feet cell holds the carpet (thin floor) with a solid
      // block below it — a node one higher would be floating in the air.
      const feet = world.stateAt(n.x, n.y, n.z)
      const below = world.stateAt(n.x, n.y - 1, n.z)
      expect(feet, `feet cell at (${n.x},${n.y},${n.z})`).to.equal(CARPET)
      expect(below, `support below (${n.x},${n.y},${n.z})`).to.equal(STONE)
    }
  })

  it('diagonal walk on flat carpet stays same-level', () => {
    const world = new VoxelWorld({ x0: -4, y0: -6, z0: -4, x1: 8, y1: 8, z1: 8 })
    world.fill(-4, -1, -4, 8, -1, 8, STONE)
    world.fill(-4, 0, -4, 8, 0, 8, CARPET)
    const res = rawSolve(world, { x: 0, y: 0, z: 0 }, new GoalBlock(3, 0, 3))
    expect(res.status).to.equal('success')
    expect(res.path.length).to.equal(3) // pure diagonals, no detours, no hops
    for (const n of res.path) expect(n.y, 'diagonal on carpet must not climb').to.equal(0)
  })

  it('deep drop lands IN the carpet cell, not on top of it', () => {
    const world = new VoxelWorld({ x0: -4, y0: -8, z0: -4, x1: 8, y1: 8, z1: 8 })
    world.fill(-4, -5, -4, 8, -5, 8, STONE) // pit floor, walk -4
    world.fill(-4, -4, -4, 8, -4, 8, CARPET) // carpeted pit
    world.fill(0, -1, -1, 0, -1, 1, STONE) // takeoff ledge, walk 0
    const res = rawSolve(world, { x: 0, y: 0, z: 0 }, new GoalBlock(1, -4, 0))
    expect(res.status).to.equal('success')
    const last = res.path[res.path.length - 1]
    expect(last.y, 'landing node must be the carpet cell').to.equal(-4)
  })

  it('storage-room pillar course: standing diagonal 1x1 hops climb to the player (prod dump 2026-08-20)', () => {
    // Exact geometry from the PF_DUMP_NOPATH capture, rebased to origin:
    // floor top at walk 0; 1-tall pillar A, 2-tall pillar D, 3-tall goal
    // pillar; goal = GoalNear(player on the tall pillar, r=2). The climb is
    // A → D (offset (-1,-3), +1) → top (offset (-1,-3), +1): standing
    // corner-takeoff diagonal hops (per-axis credit model).
    const world = new VoxelWorld({ x0: -10, y0: -6, z0: -10, x1: 12, y1: 12, z1: 12 })
    world.fill(-10, -1, -10, 12, -1, 12, STONE) // floor, walk 0
    world.fill(2, 0, 6, 2, 0, 6, STONE) // A: 1-tall, walk 1  (161701,87667)
    world.fill(1, 0, 3, 1, 1, 3, STONE) // D: 2-tall, walk 2  (161700,87664)
    world.fill(0, 0, 0, 0, 2, 0, STONE) // goal pillar: 3-tall, walk 3 (161699,87661)
    const res = rawSolve(world, { x: 4, y: 0, z: 3 }, new GoalNear(0, 3, 0, 2))
    expect(res.status).to.equal('success')
    const tail = res.path.slice(-2)
    expect(tail[tail.length - 1].y, 'must top out on the goal pillar').to.equal(3)
    expect(res.path.some(n => n.parkour === true)).to.equal(true)
  })

  it('extended course: head-hitter drop from the pillar top to a ledge under the overhang (prod dump 2026-08-20 #2)', () => {
    // Vince extended the pillar course: from the 3-tall pillar top the next
    // leg is a (2,2) drop-1 onto a ledge that sits UNDER the raised
    // platform's overhang — solid blocks 2 above the takeoff. A player
    // jumps, bonks at +0.2, and lands; the planner must classify the lid as
    // head-hitter instead of vetoing.
    const world = new VoxelWorld({ x0: -10, y0: -6, z0: -10, x1: 12, y1: 12, z1: 12 })
    world.fill(-10, -1, -10, 12, -1, 12, STONE) // floor, walk 0
    world.fill(0, 0, 0, 0, 2, 0, STONE) // 3-tall takeoff pillar, walk 3
    world.fill(2, 1, -2, 4, 1, -2, STONE) // ledge, walk 2, one below takeoff
    world.fill(-1, 5, -4, 6, 5, 1, STONE) // the overhang: lid 2 above takeoff walk
    const res = rawSolve(world, { x: 0, y: 3, z: 0 }, new GoalNear(3, 2, -2, 1))
    expect(res.status).to.equal('success')
    const last = res.path[res.path.length - 1]
    expect(last.y, 'must land on the ledge under the lid').to.equal(2)
    expect(last.parkour, 'must arrive by the bonked drop-jump').to.equal(true)
  })

  it('corner jump past a pillar beside the landing (prod dump 2026-08-20 #3)', () => {
    // Rebased from the capture: takeoff on a 1-tall block, landing on a
    // block +1 up at offset (1,3), with a 3-tall pillar filling the cell
    // directly beside the landing. The flight only NICKS that pillar's
    // corner (<7cm) — vanilla slides past — but the old full-hitbox sweep
    // treated it as a wall and vetoed, stranding the bot one hop short.
    const world = new VoxelWorld({ x0: -8, y0: -6, z0: -8, x1: 10, y1: 10, z1: 12 })
    world.fill(-8, -1, -8, 10, -1, 10, STONE) // floor, walk 0
    world.set(0, 0, 3, STONE) // takeoff block, walk 1
    world.set(1, 0, 0, STONE) // landing block, walk 1... raised below
    world.set(1, 1, 0, STONE) // landing top at walk 2 (+1 from takeoff)
    world.fill(0, 0, 0, 0, 2, 0, STONE) // the pillar beside the landing
    const res = rawSolve(world, { x: 0, y: 1, z: 3 }, new GoalBlock(1, 2, 0))
    expect(res.status).to.equal('success')
    const last = res.path[res.path.length - 1]
    expect(last.parkour, 'must arrive by the corner jump').to.equal(true)
    expect(res.path.length, 'one hop, no detour').to.equal(1)
  })

  it('drop-jump flies over same-level corners near takeoff (flight-curve corridor)', () => {
    // (2,2) drop of 2: pre-fix the corridor demanded clearance down to the
    // landing depth over EVERY swept cell, so the same-level corner floors
    // beside the takeoff vetoed the jump.
    const world = new VoxelWorld({ x0: -4, y0: -8, z0: -4, x1: 10, y1: 8, z1: 10 })
    world.fill(0, -1, 0, 1, -1, 1, STONE) // takeoff platform 2x2, walk 0
    world.set(1, -1, 2, STONE) // same-level corner floor in the swept corridor
    world.set(2, -1, 1, STONE) // (the other corner)
    world.fill(3, -3, 3, 4, -3, 4, STONE) // landing platform, walk -2
    const res = rawSolve(world, { x: 1, y: 0, z: 1 }, new GoalBlock(3, -2, 3))
    expect(res.status).to.equal('success')
    const last = res.path[res.path.length - 1]
    expect(last.parkour, 'must arrive by the diagonal drop-jump').to.equal(true)
    expect(last.y).to.equal(-2)
  })
})
