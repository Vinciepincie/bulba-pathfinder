// The taut corner cut: the executor walks the straight line between the ends
// of a zig-zag, not the zig-zag. Eight-direction planning turns a run a few
// blocks across and many along into a 45° leg and a straight leg; the old cut
// could only skip nodes within half a block of its chord, so it walked both
// legs. Now skipped nodes retire as the body draws level with them and the
// chord runs end to end — and the swept-box check still refuses a chord the
// body does not fit down.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { GoalBlock } from '../src/goals.js'
import { VoxelWorld, STONE, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

interface PF {
  goto: (goal: unknown) => Promise<void>
  setMovements: (m: Movements) => void
  movements: Movements
}

function flatWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -8, y0: -2, z0: -8, x1: 24, y1: 8, z1: 24 })
  world.fill(-8, -1, -8, 24, -1, 24, STONE) // floor at y=-1, walkable at y=0
  return world
}

function setup (world: VoxelWorld, start: Vec3): { bot: DriveableBot, pf: PF } {
  const bot = makeDriveableBot(world, start)
  const plugin = createPathfinder({
    useWorkerThreads: false,
    physicsFactory: () => makeFakePhysics(world, bot)
  })
  bot.loadPlugin(plugin as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: PF }).pathfinder
  const movements = applyProfile(new Movements(bot as never) as never) as unknown as Movements
  pf.setMovements(movements)
  return { bot, pf }
}

/** Horizontal distance from q to the segment ab. */
function offLine (q: { x: number, z: number }, a: { x: number, z: number }, b: { x: number, z: number }): number {
  const abx = b.x - a.x
  const abz = b.z - a.z
  let t = ((q.x - a.x) * abx + (q.z - a.z) * abz) / (abx * abx + abz * abz)
  t = Math.max(0, Math.min(1, t))
  return Math.hypot(q.x - (a.x + abx * t), q.z - (a.z + abz * t))
}

async function drive (
  bot: DriveableBot,
  promise: Promise<void>,
  maxTicks: number,
  onTick: () => void
): Promise<'resolved' | 'rejected'> {
  let settled: 'resolved' | 'rejected' | null = null
  promise.then(() => { settled = 'resolved' }, () => { settled = 'rejected' })
  for (let i = 0; i < maxTicks && settled === null; i++) {
    onTick()
    bot.tick()
    await new Promise(resolve => setImmediate(resolve))
    await new Promise(resolve => setTimeout(resolve, 1))
  }
  if (settled === null) throw new Error(`goto did not settle within ${maxTicks} ticks (bot at ${bot.entity.position})`)
  return settled
}

describe('taut corner cut', function () {
  this.timeout(60000)

  it('walks the chord between the ends of a zig-zag and never doubles back', async () => {
    const world = flatWorld()
    const start = new Vec3(0.5, 0, 0.5)
    const goal = new Vec3(4.5, 0, 13.5)
    const { bot, pf } = setup(world, start)
    let plan: Array<{ x: number, z: number }> = []
    bot.on('path_update', ((r: { status: string, path: Array<{ x: number, z: number }> }) => {
      if (r.status === 'success' && plan.length === 0) plan = r.path.map(n => ({ x: n.x, z: n.z }))
    }) as never)
    const track: Vec3[] = []
    const outcome = await drive(bot, pf.goto(new GoalBlock(4, 0, 13)), 600, () => track.push(bot.entity.position.clone()))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(goal)).to.be.lessThan(1.2)

    // The PLAN is eight-connected and some of it sits well off the chord.
    const planOff = Math.max(...plan.map(n => offLine(n, start, goal)))
    expect(planOff, `plan ${JSON.stringify(plan)}`).to.be.greaterThan(1.0)
    // The BODY stayed on the chord: the whole zig-zag was skipped in one line.
    const walkedOff = Math.max(...track.map(q => offLine(q, start, goal)))
    expect(walkedOff, 'body left the chord').to.be.lessThan(0.6)
    // And every tick made headway along it — a stranded node turns the bot round.
    const ux = (goal.x - start.x) / start.distanceTo(goal)
    const uz = (goal.z - start.z) / start.distanceTo(goal)
    let worstBack = 0
    for (let i = 1; i < track.length; i++) {
      const step = (track[i].x - track[i - 1].x) * ux + (track[i].z - track[i - 1].z) * uz
      worstBack = Math.min(worstBack, step)
    }
    expect(worstBack).to.be.greaterThan(-0.05)
  })

  it('refuses a chord the body does not fit down and follows the plan round the wall', async () => {
    const world = flatWorld()
    // A wall across the straight line from start to goal, open at its far end.
    for (let z = 3; z <= 11; z++) {
      world.set(3, 0, z, STONE)
      world.set(3, 1, z, STONE)
    }
    const start = new Vec3(0.5, 0, 0.5)
    const goal = new Vec3(4.5, 0, 13.5)
    const { bot, pf } = setup(world, start)
    const track: Vec3[] = []
    const outcome = await drive(bot, pf.goto(new GoalBlock(4, 0, 13)), 800, () => track.push(bot.entity.position.clone()))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(goal)).to.be.lessThan(1.2)
    // Never inside the wall's cells (the fake physics would not stop it).
    for (const q of track) {
      const inWall = Math.floor(q.x) === 3 && Math.floor(q.z) >= 3 && Math.floor(q.z) <= 11
      expect(inWall, `walked through the wall at ${q}`).to.equal(false)
    }
  })
})
