// Hand-built voxel scenes over Solver + MoveGen: each test constructs a tiny
// world with a known optimal route (or a provably unreachable goal), asserts
// exact reachability/cost, and validates every returned path with the
// independent checkWalkable() verifier from the shared test helpers.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import {
  VoxelWorld,
  makeFakeBot,
  makeOurMovements,
  lutFor,
  snapshotFromWorld,
  checkWalkable,
  AIR,
  STONE,
  WATER,
  LADDER,
  OAK_FENCE_DRY,
  CREEPER_HEAD,
  mcData,
  Block
} from './helpers/voxelWorld.js'
import { Solver } from '../src/solver.js'
import { Move } from '../src/move.js'
import type { RawSolveResult } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { GoalBlock } from '../src/goals.js'
import type { Goal } from '../src/goals.js'

interface XYZ { x: number, y: number, z: number }

interface SolveOpts {
  searchRadius?: number
  overrides?: Record<string, unknown>
}

/** Build movements/lut/snapshot for the world and run the solver to a terminal status. */
function solve (world: VoxelWorld, start: XYZ, goal: Goal, opts: SolveOpts = {}): RawSolveResult {
  const bot = makeFakeBot(world, new Vec3(start.x + 0.5, start.y, start.z + 0.5))
  const movements = makeOurMovements(bot, opts.overrides ?? {})
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), start, {
    timeout: 5000,
    searchRadius: opts.searchRadius ?? -1
  })
  let res = solver.compute(1000)
  while (res.status === 'partial') res = solver.compute(1000)
  return res
}

function assertWalkable (world: VoxelWorld, start: XYZ, res: RawSolveResult): void {
  const err = checkWalkable(world, start, res.path)
  expect(err, 'independent path walkability check').to.equal(null)
}

function lastNode (res: RawSolveResult): XYZ {
  expect(res.path.length).to.be.greaterThan(0)
  const n = res.path[res.path.length - 1]
  return { x: n.x, y: n.y, z: n.z }
}

/** Find door states of the given block for the corridor scenes. */
function findDoorStates (name: string, open: boolean): { lower: number, upper: number } {
  const door = mcData.blocksByName[name]
  let lower = -1
  let upper = -1
  for (let s = door.minStateId as number; s <= (door.maxStateId as number); s++) {
    const props = Block.fromStateId(s, 0).getProperties() as Record<string, unknown>
    if (String(props.open) !== String(open)) continue
    if (String(props.facing) !== 'west') continue
    if (String(props.hinge) !== 'left') continue
    if (String(props.powered) !== 'false') continue
    if (String(props.half) === 'lower' && lower < 0) lower = s
    if (String(props.half) === 'upper' && upper < 0) upper = s
  }
  expect(lower, `${name} lower half (open=${String(open)})`).to.be.greaterThan(-1)
  expect(upper, `${name} upper half (open=${String(open)})`).to.be.greaterThan(-1)
  return { lower, upper }
}

/**
 * Scene 9's corridor: 1-wide corridor along x at z=0, stone walls at z=±1,
 * a cross-wall at x=3 whose passage cells (3,0,0)/(3,1,0) hold the two door
 * halves, stone above the door so it can't be jumped.
 */
function doorCorridorWorld (lowerState: number, upperState: number): VoxelWorld {
  const world = new VoxelWorld({ x0: -1, y0: -2, z0: -1, x1: 7, y1: 4, z1: 1 })
  world.fill(-1, -1, -1, 7, -1, 1, STONE) // floor
  world.fill(-1, 0, -1, 7, 2, -1, STONE) // south wall
  world.fill(-1, 0, 1, 7, 2, 1, STONE) // north wall
  world.fill(3, 0, 0, 3, 2, 0, STONE) // cross wall in the corridor
  world.set(3, 0, 0, lowerState)
  world.set(3, 1, 0, upperState)
  return world
}

describe('Solver over hand-built scenes', function () {
  // LUT builds iterate every block state; allow headroom on slow CI.
  this.timeout(30000)

  it('scene 1: flat 10-block walk on stone floor', () => {
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 11, y1: 3, z1: 2 })
    world.fill(-2, -1, -2, 11, -1, 2, STONE)
    const start = { x: 0, y: 0, z: 0 }
    const res = solve(world, start, new GoalBlock(9, 0, 0))

    expect(res.status).to.equal('success')
    expect(res.path.length).to.be.at.least(9)
    // Cardinal walk costs 1/block; goal is axis-aligned so the optimum is 9.
    expect(res.cost).to.be.closeTo(9, 0.001)
    expect(lastNode(res)).to.deep.equal({ x: 9, y: 0, z: 0 })
    assertWalkable(world, start, res)
  })

  it('scene 2: 3-step staircase up (jump cost 2 per +1 step)', () => {
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 8, y1: 8, z1: 2 })
    world.fill(-2, -1, -2, 1, -1, 2, STONE) // base floor, stand y=0
    world.fill(2, -1, -2, 2, 0, 2, STONE) // step 1, stand y=1
    world.fill(3, -1, -2, 3, 1, 2, STONE) // step 2, stand y=2
    world.fill(4, -1, -2, 6, 2, 2, STONE) // top, stand y=3
    const start = { x: 0, y: 0, z: 0 }
    const res = solve(world, start, new GoalBlock(4, 3, 0))

    expect(res.status).to.equal('success')
    // walk (1) + three jump-ups (2 each) = 7
    expect(res.cost).to.be.closeTo(7, 0.001)
    expect(lastNode(res)).to.deep.equal({ x: 4, y: 3, z: 0 })
    assertWalkable(world, start, res)
  })

  it('scene 3: drop-down ledge of 3 succeeds, ledge of 6 is noPath (maxDropDown 4)', () => {
    // Ledge of 3: platform stand y=4, lower floor stand y=1 (block drop 4 ≤ maxDropDown).
    const world3 = new VoxelWorld({ x0: -3, y0: -1, z0: -2, x1: 7, y1: 9, z1: 2 })
    world3.fill(-2, 3, -1, 1, 3, 1, STONE) // high platform
    world3.fill(2, 0, -2, 6, 0, 2, STONE) // lower floor
    const start3 = { x: 0, y: 4, z: 0 }
    const res3 = solve(world3, start3, new GoalBlock(4, 1, 0))
    expect(res3.status).to.equal('success')
    // walk (1) + drop (1, flat cost regardless of height) + two walks (2) = 4
    expect(res3.cost).to.be.closeTo(4, 0.001)
    expect(lastNode(res3)).to.deep.equal({ x: 4, y: 1, z: 0 })
    assertWalkable(world3, start3, res3)

    // Ledge of 6: same shape, platform raised — landing scan exceeds maxDropDown
    // everywhere and every other column is a bottomless void inside the box.
    const world6 = new VoxelWorld({ x0: -3, y0: -1, z0: -2, x1: 7, y1: 12, z1: 2 })
    world6.fill(-2, 6, -1, 1, 6, 1, STONE) // high platform, stand y=7
    world6.fill(2, 0, -2, 6, 0, 2, STONE) // lower floor, stand y=1 (drop 7 > 4)
    const res6 = solve(world6, { x: 0, y: 7, z: 0 }, new GoalBlock(4, 1, 0))
    expect(res6.status).to.equal('noPath')
  })

  it('scene 4: 1-block gap needs parkour; noPath with allowParkour=false', () => {
    const world = new VoxelWorld({ x0: -2, y0: -4, z0: -2, x1: 6, y1: 3, z1: 2 })
    world.fill(-1, -1, -1, 0, -1, 1, STONE) // platform A
    world.fill(2, -1, -1, 4, -1, 1, STONE) // platform B (gap column x=1 is void)
    const start = { x: 0, y: 0, z: 0 }

    const res = solve(world, start, new GoalBlock(3, 0, 0))
    expect(res.status).to.equal('success')
    // parkour hop (1) + walk (1)
    expect(res.cost).to.be.closeTo(2, 0.001)
    const parkourNodes = res.path.filter(n => n.parkour)
    expect(parkourNodes.length).to.be.at.least(1)
    expect(parkourNodes[0]).to.include({ x: 2, y: 0, z: 0 })
    expect(lastNode(res)).to.deep.equal({ x: 3, y: 0, z: 0 })
    assertWalkable(world, start, res)

    const resNoParkour = solve(world, start, new GoalBlock(3, 0, 0), {
      overrides: { allowParkour: false }
    })
    expect(resNoParkour.status).to.equal('noPath')
  })

  it('scene 5: fully walled-in start → noPath, few visited nodes, no boundary contact', () => {
    // 3x3 interior floor at y=0 surrounded by a 3-high stone ring; the box is
    // padded so no probe from the interior ever leaves the snapshot.
    const world = new VoxelWorld({ x0: -3, y0: -2, z0: -3, x1: 3, y1: 3, z1: 3 })
    world.fill(-2, -1, -2, 2, -1, 2, STONE) // floor (extends under the walls)
    world.fill(-2, 0, -2, 2, 2, 2, STONE) // solid block...
    world.fill(-1, 0, -1, 1, 2, 1, AIR) // ...hollowed to a 3x3x3 air pocket
    const res = solve(world, { x: 0, y: 0, z: 0 }, new GoalBlock(6, 0, 0))

    expect(res.status).to.equal('noPath')
    // Only the 9 interior floor cells are ever expandable.
    expect(res.visitedNodes).to.be.at.least(1)
    expect(res.visitedNodes).to.be.at.most(9)
    expect(res.boundaryLimited).to.equal(false)
  })

  it('scene 6: searchRadius slack prunes a long detour; unbounded finds it', () => {
    // Wall at x=2 spanning z=-6..6, height 3: the only crossing is the z=±7
    // edge rows, a detour costing far more than heuristic(start)=4 + slack 3.
    const world = new VoxelWorld({ x0: -3, y0: -2, z0: -7, x1: 7, y1: 3, z1: 7 })
    world.fill(-3, -1, -7, 7, -1, 7, STONE)
    world.fill(2, 0, -6, 2, 2, 6, STONE)
    const start = { x: 0, y: 0, z: 0 }
    const goal = new GoalBlock(4, 0, 0)

    const bounded = solve(world, start, goal, { searchRadius: 3 })
    expect(bounded.status).to.equal('noPath')

    const unbounded = solve(world, start, goal, { searchRadius: -1 })
    expect(unbounded.status).to.equal('success')
    expect(unbounded.cost).to.be.greaterThan(7) // > h0 + slack: the pruning was decisive
    expect(lastNode(unbounded)).to.deep.equal({ x: 4, y: 0, z: 0 })
    assertWalkable(world, start, unbounded)
  })

  it('scene 7: ladder column up 4 and back down', () => {
    // Up: ladder at (0, 0..3, 0), platform (stand y=4) alongside the column top.
    const worldUp = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 4, y1: 8, z1: 2 })
    worldUp.fill(-1, -1, -1, 1, -1, 1, STONE)
    worldUp.fill(0, 0, 0, 0, 3, 0, LADDER)
    worldUp.fill(1, 3, -1, 3, 3, 1, STONE) // top platform
    const startUp = { x: 0, y: 0, z: 0 }
    const resUp = solve(worldUp, startUp, new GoalBlock(1, 4, 0))
    expect(resUp.status).to.equal('success')
    // cheapest: 3 climbs (1 each) + jump-up onto the platform (2), or
    // 4 climbs + walk — both cost 5
    expect(resUp.cost).to.be.closeTo(5, 0.001)
    expect(lastNode(resUp)).to.deep.equal({ x: 1, y: 4, z: 0 })
    assertWalkable(worldUp, startUp, resUp)

    // Down: ladder shaft through a solid 3x3 tower — descending is stepwise
    // (the landing scan stops on each ladder cell because ladders are
    // physical in 1.21.1), one cell per move.
    const worldDown = new VoxelWorld({ x0: -3, y0: -1, z0: -3, x1: 3, y1: 8, z1: 3 })
    worldDown.fill(-1, 0, -1, 1, 0, 1, STONE) // tower base
    worldDown.fill(-1, 1, -1, 1, 4, 1, STONE) // tower body
    worldDown.fill(0, 1, 0, 0, 4, 0, LADDER) // shaft
    const startDown = { x: 0, y: 4, z: 0 }
    const resDown = solve(worldDown, startDown, new GoalBlock(0, 1, 0))
    expect(resDown.status).to.equal('success')
    expect(resDown.cost).to.be.closeTo(3, 0.001)
    expect(lastNode(resDown)).to.deep.equal({ x: 0, y: 1, z: 0 })
    assertWalkable(worldDown, startDown, resDown)
  })

  it('scene 8: shallow water channel is crossable and costs more than dry ground', () => {
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 8, y1: 3, z1: 2 })
    world.fill(-2, -1, -2, 8, -1, 2, STONE)
    world.fill(2, 0, -2, 4, 0, 2, WATER) // 3-wide feet-deep strip across all z
    const start = { x: 0, y: 0, z: 0 }
    const res = solve(world, start, new GoalBlock(6, 0, 0))

    expect(res.status).to.equal('success')
    // 6 walks + liquidCost(1) for each of the 3 moves departing a water cell.
    expect(res.cost).to.be.closeTo(9, 0.001)
    expect(res.cost).to.be.greaterThan(6) // dry-equivalent straight-line cost
    expect(lastNode(res)).to.deep.equal({ x: 6, y: 0, z: 0 })
    assertWalkable(world, start, res)
  })

  // (Found by this suite, then FIXED in src/moveGen.ts: a closed door is two
  // blocks tall, so in door mode the closed UPPER half must not veto the
  // move — activating the lower half opens both.)
  it('scene 9a: closed oak door in a corridor is opened when canOpenDoors && canOpenRealDoors', () => {
    const closed = findDoorStates('oak_door', false)
    const world = doorCorridorWorld(closed.lower, closed.upper)
    const start = { x: 0, y: 0, z: 0 }
    const res = solve(world, start, new GoalBlock(5, 0, 0), {
      overrides: { canOpenDoors: true, canOpenRealDoors: true }
    })

    expect(res.status).to.equal('success')
    expect(res.cost).to.be.closeTo(5, 0.001)
    const doorNode = res.path.find(n => n.x === 3 && n.y === 0 && n.z === 0)
    expect(doorNode, 'path must pass through the door cell').to.not.equal(undefined)
    expect(doorNode?.useOne).to.deep.equal({ x: 3, y: 0, z: 0 })
    assertWalkable(world, start, res)
  })

  it('scene 9b: open oak door is passable in door mode; closed door with canOpenDoors=false is noPath', () => {
    // Open door + door mode: the DOOR_OPEN improvement makes both halves
    // passable, no activation needed.
    const open = findDoorStates('oak_door', true)
    const worldOpen = doorCorridorWorld(open.lower, open.upper)
    const start = { x: 0, y: 0, z: 0 }
    const resOpen = solve(worldOpen, start, new GoalBlock(5, 0, 0), {
      overrides: { canOpenDoors: true, canOpenRealDoors: true }
    })
    expect(resOpen.status).to.equal('success')
    expect(resOpen.cost).to.be.closeTo(5, 0.001)
    const doorNode = resOpen.path.find(n => n.x === 3 && n.y === 0 && n.z === 0)
    expect(doorNode, 'path must pass through the door cell').to.not.equal(undefined)
    expect(doorNode?.useOne).to.equal(null) // open door needs no activation
    assertWalkable(worldOpen, start, resOpen)

    // Closed door without canOpenDoors: strict upstream parity — blocked.
    const closed = findDoorStates('oak_door', false)
    const worldClosed = doorCorridorWorld(closed.lower, closed.upper)
    const resClosed = solve(worldClosed, start, new GoalBlock(5, 0, 0), {
      overrides: { canOpenDoors: false }
    })
    expect(resClosed.status).to.equal('noPath')
  })

  it('scene 10: solving the staircase twice is fully deterministic', () => {
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 8, y1: 8, z1: 2 })
    world.fill(-2, -1, -2, 1, -1, 2, STONE)
    world.fill(2, -1, -2, 2, 0, 2, STONE)
    world.fill(3, -1, -2, 3, 1, 2, STONE)
    world.fill(4, -1, -2, 6, 2, 2, STONE)
    const start = { x: 0, y: 0, z: 0 }

    const res1 = solve(world, start, new GoalBlock(4, 3, 0))
    const res2 = solve(world, start, new GoalBlock(4, 3, 0))

    expect(res1.status).to.equal('success')
    expect(res2.status).to.equal('success')
    expect(res2.path).to.deep.equal(res1.path)
    expect(res2.visitedNodes).to.equal(res1.visitedNodes)
    expect(res2.cost).to.equal(res1.cost)
  })

  it('scene 11: route exiting the snapshot box reports boundaryLimited on noPath', () => {
    // Floor runs to the box edge; the goal lies beyond it. The frontier probes
    // outside the snapshot, so the noPath must carry the growth signal.
    const world = new VoxelWorld({ x0: -2, y0: -2, z0: -2, x1: 5, y1: 3, z1: 2 })
    world.fill(-2, -1, -2, 5, -1, 2, STONE)
    const res = solve(world, { x: 0, y: 0, z: 0 }, new GoalBlock(10, 0, 0))

    expect(res.status).to.equal('noPath')
    expect(res.boundaryLimited).to.equal(true)
  })

  it('scene 12: diagonal pillar course needs allowParkourExtended; off (default) is noPath', () => {
    // 1x1 pillars over void, each landing offset (2,1) from the previous —
    // two knight jumps chain to the goal. No cardinal or walking route exists.
    const world = new VoxelWorld({ x0: -3, y0: -4, z0: -3, x1: 7, y1: 4, z1: 7 })
    world.set(0, -1, 0, STONE)
    world.set(2, -1, 1, STONE)
    world.set(4, -1, 2, STONE)
    const start = { x: 0, y: 0, z: 0 }

    const res = solve(world, start, new GoalBlock(4, 0, 2), {
      overrides: { allowParkourExtended: true }
    })
    expect(res.status).to.equal('success')
    expect(res.cost).to.be.closeTo(2 * (Math.hypot(2, 1) + 0.5), 1e-9)
    expect(res.path.map(n => [n.x, n.y, n.z, n.parkour])).to.deep.equal([
      [2, 0, 1, true],
      [4, 0, 2, true]
    ])
    assertWalkable(world, start, res)

    expect(solve(world, start, new GoalBlock(4, 0, 2)).status).to.equal('noPath')
    expect(solve(world, start, new GoalBlock(4, 0, 2), {
      overrides: { allowParkourExtended: true, allowSprinting: false }
    }).status).to.equal('noPath')
  })

  it('scene 13: up-jump, drop-jump and a ladder catch chain to the goal', () => {
    const world = new VoxelWorld({ x0: -3, y0: -4, z0: -3, x1: 9, y1: 5, z1: 7 })
    world.set(0, -1, 0, STONE) // start pillar, stand y=0
    world.set(2, 0, 1, STONE) // up-jump (2,1): block at flight level, land on top (y=1)
    world.set(4, -1, 2, STONE) // drop-jump (2,1) from y=1 back down to y=0
    world.set(6, 0, 2, LADDER) // cardinal (2,0) gap-jump into the ladder...
    world.set(6, 1, 2, LADDER)
    world.fill(7, 0, 2, 7, 1, 2, STONE) // ...mounted on this wall
    // Cap the ladder: upstream's parkour treats a ladder cell as physical
    // support and jump-ups onto its top for cost 1 (type-level bbox quirk,
    // mirrored) — the cap suppresses that so the CATCH is the only way in.
    world.set(6, 2, 2, STONE)
    const start = { x: 0, y: 0, z: 0 }

    const res = solve(world, start, new GoalBlock(6, 0, 2), {
      overrides: { allowParkourExtended: true }
    })
    expect(res.status).to.equal('success')
    // up (knight+1) + drop (knight) + ladder catch (2.5) = 2·knight + 3.5
    expect(res.cost).to.be.closeTo(2 * (Math.hypot(2, 1) + 0.5) + 3.5, 1e-9)
    expect(res.path.map(n => [n.x, n.y, n.z, n.parkour])).to.deep.equal([
      [2, 1, 1, true],
      [4, 0, 2, true],
      [6, 0, 2, true]
    ])
    assertWalkable(world, start, res)

    expect(solve(world, start, new GoalBlock(6, 0, 2)).status).to.equal('noPath')
  })
})

describe('momentum search state (allowParkourMomentum)', function () {
  this.timeout(30000)
  const MOM = { allowParkourExtended: true, allowParkourMomentum: true }
  /** A → B (3,0) onto a post → C (6,1) four down onto a head → D (6,2) four down onto a head (movegen.test.ts). */
  function chainWorld (): VoxelWorld {
    const world = new VoxelWorld({ x0: -4, y0: -11, z0: -4, x1: 19, y1: 7, z1: 8 })
    world.set(0, 0, 0, STONE) // A: stand (0,1,0)
    world.set(3, 0, 0, OAK_FENCE_DRY) // B: stand (3,1,0), feet 1.5
    world.set(9, -3, 1, CREEPER_HEAD) // C: a head, stand (9,-2,1), feet -2.5
    world.set(15, -7, 3, CREEPER_HEAD) // D: a head, stand (15,-6,3), feet -6.5
    return world
  }

  it('a three-hop chain reaches what the compound two-hop chain cannot', () => {
    const world = chainWorld()
    const start = { x: 0, y: 1, z: 0 }
    const off = solve(world, start, new GoalBlock(15, -6, 3), { overrides: { allowParkourExtended: true } })
    expect(off.status).to.equal('noPath')
    const on = solve(world, start, new GoalBlock(15, -6, 3), { overrides: MOM })
    expect(on.status).to.equal('success')
    // The path is the cells themselves: stone, chain landing, chain landing —
    // each chain node's `via` is the node before it (its stone).
    const cells = on.path.map(n => `${n.x},${n.y},${n.z}`)
    expect(cells).to.deep.equal(['3,1,0', '9,-2,1', '15,-6,3'])
    expect(on.path.map(n => n.parkour)).to.deep.equal([true, true, true])
    expect(on.path.map(n => n.chain === true)).to.deep.equal([false, true, true])
    expect(on.path[1].via).to.deep.equal({ x: 3, y: 1, z: 0 })
    expect(on.path[2].via).to.deep.equal({ x: 9, y: -2, z: 1 })
    expect(on.cost).to.be.closeTo(3.5 + Math.max(Math.hypot(6, 1) + 0.5, 5 + Math.SQRT2) + Math.max(Math.hypot(6, 2) + 0.5, 4 + 2 * Math.SQRT2), 1e-9)
    // Executor form: no stone is duplicated, each stone is aimed at its re-jump.
    const moves = Move.expandRawPath(on.path)
    expect(moves.map(m => `${m.x},${m.y},${m.z}`)).to.deep.equal(cells)
    expect(moves[0].aimDx).to.be.closeTo(6 / Math.hypot(6, 1), 1e-12)
    expect(moves[1].aimDx).to.be.closeTo(6 / Math.hypot(6, 2), 1e-12)
    expect(moves[2].aimDx).to.equal(0)
    // The compound form (flag off) still expands into stone + node.
    const two = solve(world, start, new GoalBlock(9, -2, 1), { overrides: { allowParkourExtended: true } })
    expect(two.status).to.equal('success')
    expect(two.path.map(n => `${n.x},${n.y},${n.z}`)).to.deep.equal(['9,-2,1'])
    expect(Move.expandRawPath(two.path).map(m => `${m.x},${m.y},${m.z}`)).to.deep.equal(['3,1,0', '9,-2,1'])
    // Same two-hop goal with momentum: the stone is a real node, same cost.
    const twoOn = solve(world, start, new GoalBlock(9, -2, 1), { overrides: MOM })
    expect(twoOn.status).to.equal('success')
    expect(twoOn.path.map(n => `${n.x},${n.y},${n.z}`)).to.deep.equal(['3,1,0', '9,-2,1'])
    expect(twoOn.cost).to.be.closeTo(two.cost, 1e-9)
  })

  it('the secondary arena grows past its reserve inside one solve', () => {
    // A field of posts, every one a landing from many directions: far more
    // (cell, momentum) states than the 4096-slot reserve, all visited because
    // the goal is walled off.
    const world = new VoxelWorld({ x0: -2, y0: -3, z0: -2, x1: 62, y1: 6, z1: 62 })
    for (let x = 0; x <= 58; x += 2) for (let z = 0; z <= 58; z += 2) world.set(x, 0, z, OAK_FENCE_DRY)
    world.set(0, 0, 0, STONE)
    world.fill(59, 0, 59, 61, 4, 61, STONE) // a 3x3 plateau 3.5 above the posts: unreachable
    const bot = makeFakeBot(world, new Vec3(0.5, 1, 0.5))
    const movements = makeOurMovements(bot, MOM)
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut)
    const n = snap.meta.w * snap.meta.h * snap.meta.l
    const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(new GoalBlock(60, 5, 60)), { x: 0, y: 1, z: 0 }, { timeout: 20000, searchRadius: -1 })
    let res = solver.compute(1e9)
    while (res.status === 'partial') res = solver.compute(1e9)
    expect(res.status).to.equal('noPath')
    expect(solver.momentumStates).to.be.greaterThan(Math.max(4096, n >> 3))
    expect(res.visitedNodes).to.be.greaterThan(4096)
  })
})
