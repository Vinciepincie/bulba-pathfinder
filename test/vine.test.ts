// Vine climbing (auto-enabled improvement on 1.16+). Upstream ships
// `climbables.add(vine)` commented out; we enable it where vanilla makes
// vines unconditionally climbable (the 1.16 climbable tag) — but ONLY for
// vine cells with an adjacent solid block to press against, because both
// vanilla and prismarine-physics ascend climbables via horizontal collision:
// a free-hanging curtain is passable, not climbable. Covers version gating,
// LUT/fingerprint plumbing, planner scenes (wall-backed ascent, free-hanging
// refusal, partial-wall stall, head-bonk exit), JS ↔ wasm parity, and the
// executor's collision-climb with the look-at-wall steering aid.
import { expect } from 'chai'
import minecraftData from 'minecraft-data'
import { Vec3 } from 'vec3'
import { Solver } from '../src/solver.js'
import type { RawSolveResult } from '../src/solver.js'
import { GoalBlock } from '../src/goals.js'
import { serializeGoal } from '../src/goalSerde.js'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { WasmSolver } from '../src/wasm/wasmSolver.js'
import { Movements, vineClimbingDefault } from '../src/movements.js'
import { LutSpecial, LutFlags } from '../src/types.js'
import { buildLut } from '../src/lut.js'
import { createPathfinder } from '../src/plugin.js'
import {
  VoxelWorld, makeFakeBot, makeOurMovements, lutFor, snapshotFromWorld, applyProfile,
  STONE, VINE, mcData
} from './helpers/voxelWorld.js'
import { makeDriveableBot, makeFakePhysics } from './helpers/fakeBot.js'
import type { DriveableBot } from './helpers/fakeBot.js'

const VINE_ID = mcData.blocksByName.vine.id as number

/** Default-profile movements (vine auto-added on 1.21.1), strict doors off. */
function vineMovements (bot: unknown, climbVines = true): Movements {
  const m = applyProfile(new Movements(bot as never) as never) as unknown as Movements
  ;(m as { canOpenRealDoors: boolean }).canOpenRealDoors = false
  if (!climbVines) m.climbables.delete(VINE_ID)
  return m
}

/**
 * Wall-backed vine ladder: ground at y=0 (stand y=1), a solid wall column at
 * (6, 1..wallTop, 0) with vines hugging it at (5, 1..6, 0), and an isolated
 * shelf at (3..4, 5, 0) reachable only from the vine top. Goal on the shelf.
 */
function vineLadderWorld (wallTop: number): VoxelWorld {
  const world = new VoxelWorld({ x0: -2, y0: -1, z0: -2, x1: 10, y1: 10, z1: 2 })
  world.fill(-2, 0, -2, 10, 0, 2, STONE)
  if (wallTop >= 1) world.fill(6, 1, 0, 6, wallTop, 0, STONE)
  for (let y = 1; y <= 6; y++) world.set(5, y, 0, VINE)
  world.fill(3, 5, 0, 4, 5, 0, STONE) // shelf
  return world
}

function solveScene (
  world: VoxelWorld,
  movements: Movements,
  start: { x: number, y: number, z: number },
  goal: GoalBlock
): { result: RawSolveResult, snapKeyed: ReturnType<typeof snapshotFromWorld>, movements: Movements } {
  const lut = lutFor(makeFakeBot(world), movements)
  const snap = snapshotFromWorld(world, lut)
  const descriptor = serializeGoal(goal)!
  const solver = new Solver(snap, movements.toConfig(), fastEvaluator(descriptor)!, start, {
    timeout: 30000,
    searchRadius: -1
  }, null, null)
  let result = solver.compute(1e9)
  while (result.status === 'partial') result = solver.compute(1e9)
  return { result, snapKeyed: snap, movements }
}

describe('vine climbing (auto on 1.16+)', function () {
  this.timeout(60000)
  let wasm: WasmSolver | null = null

  before(async function () {
    wasm = await WasmSolver.create()
  })

  function assertWasmIdentical (
    out: { result: RawSolveResult, snapKeyed: ReturnType<typeof snapshotFromWorld>, movements: Movements },
    goal: GoalBlock,
    start: { x: number, y: number, z: number },
    label: string
  ): void {
    if (!wasm) return
    const snap = out.snapKeyed
    const descriptor = serializeGoal(goal)!
    const w = wasm.solve(
      { meta: snap.meta, flags: snap.flags, heights: snap.heights, states: null, special: snap.special, entityIdx: snap.entityIdx, entityWeight: snap.entityWeight },
      out.movements.toConfig(), descriptor, start, null,
      { timeout: 30000, searchRadius: -1, sliceMs: 1e9, cancelFlag: null, onPartial: () => {} }
    )
    expect(w.status).to.equal(out.result.status, `${label}: status`)
    expect(Math.abs(w.cost - out.result.cost)).to.be.at.most(1e-9, `${label}: cost`)
    expect(w.path.map(n => `${n.x},${n.y},${n.z}`).join('|'))
      .to.equal(out.result.path.map(n => `${n.x},${n.y},${n.z}`).join('|'), `${label}: path`)
    expect(w.visitedNodes).to.equal(out.result.visitedNodes, `${label}: visited`)
  }

  // ── version gating ───────────────────────────────────────────────────────

  it('vineClimbingDefault: registry comparator decides at the 1.16 line', () => {
    expect(vineClimbingDefault(minecraftData('1.8.9'), '1.8.9')).to.equal(false)
    expect(vineClimbingDefault(minecraftData('1.15.2'), '1.15.2')).to.equal(false)
    expect(vineClimbingDefault(minecraftData('1.16.5'), '1.16.5')).to.equal(true)
    expect(vineClimbingDefault(minecraftData('1.21.1'), '1.21.1')).to.equal(true)
  })

  it('vineClimbingDefault: string fallback parses major.minor, conservative otherwise', () => {
    expect(vineClimbingDefault({}, '1.15.2')).to.equal(false)
    expect(vineClimbingDefault({}, '1.16')).to.equal(true)
    expect(vineClimbingDefault({}, '1.20.4')).to.equal(true)
    expect(vineClimbingDefault({}, '2.0')).to.equal(true)
    expect(vineClimbingDefault({}, undefined)).to.equal(false)
    expect(vineClimbingDefault({}, 'weird')).to.equal(false)
    expect(vineClimbingDefault({ version: { minecraftVersion: '1.18.2' } }, undefined)).to.equal(true)
  })

  it('Movements auto-adds vine on 1.21.1 and leaves it out on 1.15.2', () => {
    const modern = new Movements(makeFakeBot(vineLadderWorld(6)) as never)
    expect(modern.climbables.has(VINE_ID)).to.equal(true)

    const legacyData = minecraftData('1.15.2')
    const legacyBot = { registry: legacyData, version: '1.15.2' }
    const legacy = new Movements(legacyBot as never)
    expect(legacy.climbables.has(legacyData.blocksByName.vine.id as number)).to.equal(false)
    // Ladder support is version-independent.
    expect(legacy.climbables.has(legacyData.blocksByName.ladder.id as number)).to.equal(true)
  })

  // ── LUT + fingerprint plumbing ───────────────────────────────────────────

  it('LUT: vine states are CLIMBABLE+SAFE and VINE-marked in the special grid', () => {
    const bot = makeFakeBot(vineLadderWorld(6))
    const on = buildLut(bot as never, vineMovements(bot))
    expect(on.flags[VINE] & LutFlags.CLIMBABLE).to.not.equal(0)
    expect(on.flags[VINE] & LutFlags.SAFE).to.not.equal(0)
    expect(on.special).to.not.equal(null)
    expect(on.special![VINE]).to.equal(LutSpecial.VINE)

    // Opted out (and no bubbles): parity classification, no special grid.
    const off = buildLut(bot as never, vineMovements(bot, false))
    expect(off.flags[VINE] & LutFlags.CLIMBABLE).to.equal(0)
    expect(off.flags[VINE] & LutFlags.SAFE).to.not.equal(0)
    expect(off.special).to.equal(null)
  })

  it('lutFingerprint changes when vine climbing is toggled (cache safety)', () => {
    const bot = makeFakeBot(vineLadderWorld(6))
    expect(vineMovements(bot).lutFingerprint()).to.not.equal(vineMovements(bot, false).lutFingerprint())
  })

  // ── planner scenes ───────────────────────────────────────────────────────

  it('climbs a wall-backed vine ladder to the shelf (wasm identical)', () => {
    const world = vineLadderWorld(6)
    const bot = makeFakeBot(world)
    const start = { x: 0, y: 1, z: 0 }
    const goal = new GoalBlock(3, 6, 0)
    const out = solveScene(world, vineMovements(bot), start, goal)
    expect(out.result.status).to.equal('success')
    const vineNodes = out.result.path.filter(n => n.x === 5 && n.z === 0)
    expect(vineNodes.length).to.be.greaterThan(4)
    for (let i = 1; i < vineNodes.length; i++) {
      expect(vineNodes[i].y).to.equal(vineNodes[i - 1].y + 1)
    }
    assertWasmIdentical(out, goal, start, 'wall-backed ascent')
  })

  it('refuses a FREE-HANGING vine curtain — nothing to press against (wasm identical)', () => {
    const world = vineLadderWorld(0) // same scene, no wall behind the vines
    const bot = makeFakeBot(world)
    const start = { x: 0, y: 1, z: 0 }
    const goal = new GoalBlock(3, 6, 0)
    const out = solveScene(world, vineMovements(bot), start, goal)
    expect(out.result.status).to.equal('noPath')
    assertWasmIdentical(out, goal, start, 'free-hanging curtain')
  })

  it('climb stalls where the backing wall ends (partial wall)', () => {
    const world = vineLadderWorld(3) // wall only y1..3 → climb tops out at y4
    const bot = makeFakeBot(world)
    const out = solveScene(world, vineMovements(bot), { x: 0, y: 1, z: 0 }, new GoalBlock(3, 6, 0))
    expect(out.result.status).to.equal('noPath')
    const maxVineY = Math.max(0, ...out.result.path.filter(n => n.x === 5 && n.z === 0).map(n => n.y))
    expect(maxVineY).to.be.at.most(4)
  })

  /**
   * The vine-SOURCE-block scenario: vines hang from the underside of a solid
   * block at (5,5,0); climbing to (5,4,0) would put the head inside it.
   * Backing wall at x=6, exit shelf at (3..4, 2, 0) below the overhang.
   */
  function vineSourceBlockWorld (): VoxelWorld {
    const world = new VoxelWorld({ x0: -2, y0: -1, z0: -2, x1: 10, y1: 10, z1: 2 })
    world.fill(-2, 0, -2, 10, 0, 2, STONE)
    world.fill(6, 1, 0, 6, 4, 0, STONE)
    for (let y = 1; y <= 4; y++) world.set(5, y, 0, VINE)
    world.set(5, 5, 0, STONE) // the source block the vines hang from
    world.fill(3, 2, 0, 4, 2, 0, STONE) // shelf below the overhang (stand y=3)
    return world
  }

  it('head against the vine source block: node under it is unreachable (wasm identical)', () => {
    const world = vineSourceBlockWorld()
    const bot = makeFakeBot(world)
    const start = { x: 0, y: 1, z: 0 }
    const goal = new GoalBlock(5, 4, 0)
    const out = solveScene(world, vineMovements(bot), start, goal)
    // Entering (5,4,0) would jam the head into the source block — refused.
    expect(out.result.status).to.equal('noPath')
    const maxVineY = Math.max(0, ...out.result.path.filter(n => n.x === 5 && n.z === 0).map(n => n.y))
    expect(maxVineY).to.be.at.most(3)
    assertWasmIdentical(out, goal, start, 'head-bonk unreachable')
  })

  it('head-bonk still USES the climbable portion: sideways exit below the source block', () => {
    const world = vineSourceBlockWorld()
    const bot = makeFakeBot(world)
    const start = { x: 0, y: 1, z: 0 }
    const goal = new GoalBlock(3, 3, 0)
    const out = solveScene(world, vineMovements(bot), start, goal)
    expect(out.result.status).to.equal('success')
    // Climbs to (5,3,0) — one below the bonk — and steps off west.
    expect(out.result.path.some(n => n.x === 5 && n.y === 3 && n.z === 0)).to.equal(true)
    expect(out.result.path.some(n => n.x === 5 && n.y === 4 && n.z === 0)).to.equal(false, 'must not enter the head-bonk cell')
    const last = out.result.path[out.result.path.length - 1]
    expect([last.x, last.y, last.z]).to.deep.equal([3, 3, 0])
    assertWasmIdentical(out, goal, start, 'head-bonk sideways exit')
  })

  it('parity mode (vine deleted) treats vines as pass-through air', () => {
    const world = vineLadderWorld(6)
    const bot = makeFakeBot(world)
    const start = { x: 0, y: 1, z: 0 }
    const goal = new GoalBlock(3, 6, 0)
    const out = solveScene(world, vineMovements(bot, false), start, goal)
    expect(out.result.status).to.equal('noPath')
    assertWasmIdentical(out, goal, start, 'parity mode')
    // makeOurMovements (the differential-fuzz profile) matches upstream too.
    const parity = solveScene(world, makeOurMovements(bot), start, goal)
    expect(parity.result.status).to.equal('noPath')
  })

  it('walking THROUGH a vine curtain on the ground stays possible either way', () => {
    // Vines across the walkway — passable in both modes (climbable is also safe).
    const world = new VoxelWorld({ x0: -2, y0: -1, z0: -2, x1: 10, y1: 6, z1: 2 })
    world.fill(-2, 0, -2, 10, 0, 2, STONE)
    for (let z = -2; z <= 2; z++) for (let y = 1; y <= 3; y++) world.set(4, y, z, VINE)
    const bot = makeFakeBot(world)
    for (const climb of [true, false]) {
      const out = solveScene(world, vineMovements(bot, climb), { x: 0, y: 1, z: 0 }, new GoalBlock(8, 1, 0))
      expect(out.result.status).to.equal('success', `climbVines=${climb}`)
      expect(out.result.path.some(n => n.x === 4)).to.equal(true, 'walks through the curtain')
    }
  })

  // ── executor ─────────────────────────────────────────────────────────────

  async function driveUntilSettled (bot: DriveableBot, promise: Promise<void>, maxTicks = 3000): Promise<'resolved' | 'rejected'> {
    let settled: 'resolved' | 'rejected' | null = null
    promise.then(() => { settled = 'resolved' }, () => { settled = 'rejected' })
    for (let i = 0; i < maxTicks && !settled; i++) {
      bot.tick()
      await new Promise(resolve => setImmediate(resolve))
      await new Promise(resolve => setTimeout(resolve, 1))
    }
    if (!settled) throw new Error(`goto did not settle within ${maxTicks} ticks (bot at ${bot.entity.position})`)
    return settled
  }

  function setupExec (world: VoxelWorld): { bot: DriveableBot, pf: { goto: (g: unknown) => Promise<void> } } {
    const bot = makeDriveableBot(world, new Vec3(0.5, 1, 0.5))
    const plugin = createPathfinder({
      useWorkerThreads: false,
      physicsFactory: () => makeFakePhysics(world, bot)
    })
    bot.loadPlugin(plugin as unknown as (b: unknown) => void)
    const pf = (bot as unknown as { pathfinder: { goto: (g: unknown) => Promise<void>, setMovements: (m: Movements) => void } }).pathfinder
    pf.setMovements(vineMovements(bot))
    return { bot, pf }
  }

  it('executor climbs the wall-backed vines via collision (look-at-wall aid)', async () => {
    const world = vineLadderWorld(6)
    const { bot, pf } = setupExec(world)
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(3, 6, 0)))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.y).to.be.closeTo(6, 1.2)
    expect(bot.entity.position.distanceTo(new Vec3(3.5, 6, 0.5))).to.be.lessThan(1.5)
  })

  it('executor: climbs under the vine source block and exits sideways (never bonks)', async () => {
    const world = vineSourceBlockWorld()
    const { bot, pf } = setupExec(world)
    let maxY = 0
    const origTick = bot.tick
    bot.tick = () => {
      origTick()
      if (bot.entity.position.y > maxY) maxY = bot.entity.position.y
    }
    const outcome = await driveUntilSettled(bot, pf.goto(new GoalBlock(3, 3, 0)))
    expect(outcome).to.equal('resolved')
    expect(bot.entity.position.distanceTo(new Vec3(3.5, 3, 0.5))).to.be.lessThan(1.5)
    // Head stays below the source block at (5,5,0): feet never above y4.
    expect(maxY).to.be.at.most(4)
  })
})
