// A take-off nothing will sign is given up and reported to the planner. The
// jump gates here refuse every parkour take-off (as the real rollouts did on
// the arena's basic3, where a 4x4 diagonal was planned from a hemmed-in
// cell); the executor must not sit at the lip until the futility timer
// brings the same plan back, but ban the landing cell and re-solve, and the
// re-solve must route round it — here over a bridge a few blocks along.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createPathfinder } from '../src/plugin.js'
import { Movements } from '../src/movements.js'
import { GoalBlock } from '../src/goals.js'
import type { PhysicsLike, XYZ } from '../src/types.js'
import { VoxelWorld, STONE, AIR, applyProfile } from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

interface PF {
  goto: (goal: unknown) => Promise<void>
  setMovements: (m: Movements) => void
}

/** Flat floor with a one-wide chasm at x=5 over z -2..2 and a bridge at z 3..8. */
function chasmWorld (): VoxelWorld {
  const world = new VoxelWorld({ x0: -4, y0: -3, z0: -2, x1: 16, y1: 8, z1: 9 })
  world.fill(-4, -1, -2, 16, -1, 8, STONE)
  for (let z = -2; z <= 2; z++) world.set(5, -1, z, AIR)
  return world
}

function setup (world: VoxelWorld, start: Vec3): { bot: DriveableBot, pf: PF } {
  const bot = makeDriveableBot(world, start)
  const plugin = createPathfinder({
    useWorkerThreads: false,
    physicsFactory: () => {
      const base = makeFakePhysics(world, bot)
      // Every parkour take-off is refused, as the real rollouts refuse one
      // the body cannot make from where it stands.
      const gated: PhysicsLike = {
        ...base,
        canStraightLine: (path: XYZ[]) => (path[0] as { parkour?: boolean }).parkour === true ? false : base.canStraightLine(path),
        canSprintJump: () => false,
        canWalkJump: () => false
      }
      return gated
    }
  })
  bot.loadPlugin(plugin as unknown as (b: unknown) => void)
  const pf = (bot as unknown as { pathfinder: PF }).pathfinder
  const movements = applyProfile(new Movements(bot as never) as never) as unknown as Movements
  pf.setMovements(movements)
  return { bot, pf }
}

async function drive (bot: DriveableBot, promise: Promise<void>, maxTicks: number, onTick: () => void): Promise<'resolved' | 'rejected'> {
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

describe('take-off give-up and landing ban', function () {
  this.timeout(120000)

  it('abandons a jump no gate signs, bans its landing and arrives over the bridge', async () => {
    const world = chasmWorld()
    const { bot, pf } = setup(world, new Vec3(0.5, 0, 0.5))
    const resets: string[] = []
    bot.on('path_reset', ((reason: string) => { resets.push(reason) }) as never)
    const track: Vec3[] = []
    const ctl: string[] = []
    const outcome = await drive(bot, pf.goto(new GoalBlock(10, 0, 0)), 2500, () => {
      const p = bot.entity.position
      const c = bot.controlState
      track.push(p.clone())
      ctl.push(`${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)} g${bot.entity.onGround ? 1 : 0} f${c.forward ? 1 : 0}b${c.back ? 1 : 0}s${c.sneak ? 1 : 0}j${c.jump ? 1 : 0}`)
    })
    if (process.env.PF_TEST_DEBUG) {
      console.log('DEBUGCTL\n' + ctl.map((l, i) => `${i} ${l}`).join('\n') + `\nresets ${JSON.stringify(resets)} outcome ${outcome}`)
    }
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(10.5, 0, 0.5))).to.be.lessThan(1.2)
    // It gave the jump up (at least once) rather than waiting out the futility timer.
    expect(resets.filter(r => r === 'stuck').length).to.be.greaterThan(0)
    // Never in the chasm, and the crossing of x=5 happened on the bridge.
    for (const q of track) {
      expect(q.y, `fell into the chasm at ${q}`).to.be.greaterThan(-0.5)
      if (q.x > 5.3 && q.x < 5.7) expect(q.z, `crossed the chasm at ${q}`).to.be.greaterThan(2.5)
    }
  })
})
