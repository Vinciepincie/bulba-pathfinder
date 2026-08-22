// Goal-class parity vs upstream mineflayer-pathfinder/lib/goals: identical
// constructor args, identical heuristic()/isEnd() results over hundreds of
// random integer nodes. GoalLookAtBlock is checked over the same raycast
// backend (our SnapshotRaycastWorld) on both sides, so what's compared is
// the face-visibility LOGIC, with our raycast serving both implementations.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { createRequire } from 'node:module'
import {
  VoxelWorld,
  makeFakeBot,
  makeOurMovements,
  lutFor,
  snapshotFromWorld,
  STONE
} from './helpers/voxelWorld.js'
import {
  Goal,
  GoalBlock,
  GoalNear,
  GoalXZ,
  GoalNearXZ,
  GoalY,
  GoalGetToBlock,
  GoalInvert,
  GoalCompositeAny,
  GoalCompositeAll,
  GoalFollow,
  GoalLookAtBlock
} from '../src/goals.js'
import { SnapshotRaycastWorld } from '../src/raycast.js'

const require = createRequire(import.meta.url)
// eslint-disable-next-line @typescript-eslint/no-explicit-any
const upstream: any = require('mineflayer-pathfinder/lib/goals')

/** Deterministic PRNG (mulberry32) so failures are reproducible. */
function mulberry32 (seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a += 0x6D2B79F5
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

function randInt (rng: () => number, lo: number, hi: number): number {
  return lo + Math.floor(rng() * (hi - lo + 1))
}

/** Random integer nodes as Vec3 (upstream GoalLookAtBlock needs distanceTo). */
function randomNodes (count: number, rng: () => number, lo = -40, hi = 40): Vec3[] {
  const nodes: Vec3[] = []
  for (let i = 0; i < count; i++) {
    nodes.push(new Vec3(randInt(rng, lo, hi), randInt(rng, lo, hi), randInt(rng, lo, hi)))
  }
  return nodes
}

interface UpstreamGoalLike {
  heuristic (node: Vec3): number
  isEnd (node: Vec3): boolean
}

function expectParity (our: Goal, up: UpstreamGoalLike, nodes: Vec3[], label: string): void {
  for (const node of nodes) {
    const at = `${label} @ (${node.x},${node.y},${node.z})`
    expect(our.heuristic(node), `heuristic ${at}`).to.equal(up.heuristic(node))
    expect(our.isEnd(node), `isEnd ${at}`).to.equal(up.isEnd(node))
  }
}

describe('goal parity vs upstream (500 random nodes each)', function () {
  it('GoalBlock', function () {
    const nodes = randomNodes(500, mulberry32(0xB10C))
    expectParity(
      new GoalBlock(3.7, -2.2, 5.9),
      new upstream.GoalBlock(3.7, -2.2, 5.9),
      nodes, 'GoalBlock(3.7,-2.2,5.9)'
    )
    // Second instance with plain integers so isEnd can actually hit.
    const target = nodes[13]
    expectParity(
      new GoalBlock(target.x, target.y, target.z),
      new upstream.GoalBlock(target.x, target.y, target.z),
      nodes, 'GoalBlock(int)'
    )
  })

  it('GoalNear', function () {
    const nodes = randomNodes(500, mulberry32(0x4EA2))
    expectParity(
      new GoalNear(1.2, 3, -7.8, 20),
      new upstream.GoalNear(1.2, 3, -7.8, 20),
      nodes, 'GoalNear(1.2,3,-7.8,20)'
    )
  })

  it('GoalXZ', function () {
    const nodes = randomNodes(500, mulberry32(0x2CF))
    expectParity(
      new GoalXZ(5.5, -3.1),
      new upstream.GoalXZ(5.5, -3.1),
      nodes, 'GoalXZ(5.5,-3.1)'
    )
  })

  it('GoalNearXZ', function () {
    const nodes = randomNodes(500, mulberry32(0xA12F))
    expectParity(
      new GoalNearXZ(2, -9.4, 12),
      new upstream.GoalNearXZ(2, -9.4, 12),
      nodes, 'GoalNearXZ(2,-9.4,12)'
    )
  })

  it('GoalY', function () {
    const nodes = randomNodes(500, mulberry32(0x9))
    expectParity(
      new GoalY(4.6),
      new upstream.GoalY(4.6),
      nodes, 'GoalY(4.6)'
    )
  })

  it('GoalGetToBlock', function () {
    const nodes = randomNodes(500, mulberry32(0x6E7))
    expectParity(
      new GoalGetToBlock(-3, 2.9, 7),
      new upstream.GoalGetToBlock(-3, 2.9, 7),
      nodes, 'GoalGetToBlock(-3,2.9,7)'
    )
    // Nodes clustered around the block so the ===1 adjacency branch fires.
    const rng = mulberry32(0x6E8)
    const close = randomNodes(500, rng, -6, 10)
    expectParity(
      new GoalGetToBlock(-3, 2.9, 7),
      new upstream.GoalGetToBlock(-3, 2.9, 7),
      close, 'GoalGetToBlock close-range'
    )
  })

  it('GoalInvert(GoalBlock)', function () {
    const nodes = randomNodes(500, mulberry32(0x1417))
    expectParity(
      new GoalInvert(new GoalBlock(1, 2, 3)),
      new upstream.GoalInvert(new upstream.GoalBlock(1, 2, 3)),
      nodes, 'GoalInvert(GoalBlock(1,2,3))'
    )
  })

  it('GoalCompositeAny of 3 GoalBlocks', function () {
    const nodes = randomNodes(500, mulberry32(0xC0A))
    const args: Array<[number, number, number]> = [[0, 1, 2], [-5, 3, 7], [10, -4, -10]]
    expectParity(
      new GoalCompositeAny(args.map(([x, y, z]) => new GoalBlock(x, y, z))),
      new upstream.GoalCompositeAny(args.map(([x, y, z]) => new upstream.GoalBlock(x, y, z))),
      nodes, 'GoalCompositeAny'
    )
  })

  it('GoalCompositeAll of 2 GoalNears', function () {
    const nodes = randomNodes(500, mulberry32(0xC0B))
    const args: Array<[number, number, number, number]> = [[0, 0, 0, 30], [5, 5, 5, 28]]
    expectParity(
      new GoalCompositeAll(args.map(([x, y, z, r]) => new GoalNear(x, y, z, r))),
      new upstream.GoalCompositeAll(args.map(([x, y, z, r]) => new upstream.GoalNear(x, y, z, r))),
      nodes, 'GoalCompositeAll'
    )
  })

  it('GoalFollow: parity, hasChanged on far move, parity after', function () {
    // One shared fake entity so both goals observe the same position.
    const entity = { position: new Vec3(2.3, 1, -4.7) }
    const our = new GoalFollow(entity, 5)
    const up = new upstream.GoalFollow(entity, 5)

    const nodes = randomNodes(500, mulberry32(0xF0110))
    expectParity(our, up, nodes, 'GoalFollow before move')

    expect(our.hasChanged(), 'no move → hasChanged false (ours)').to.equal(false)
    expect(up.hasChanged(), 'no move → hasChanged false (upstream)').to.equal(false)

    // Move the entity far (way beyond range 5).
    entity.position.set(30.9, 8.2, -35.4)
    expect(our.hasChanged(), 'far move → hasChanged true (ours)').to.equal(true)
    expect(up.hasChanged(), 'far move → hasChanged true (upstream)').to.equal(true)

    // hasChanged updated internal targets on both; heuristics must still agree.
    const nodes2 = randomNodes(500, mulberry32(0xF0111))
    expectParity(our, up, nodes2, 'GoalFollow after move')
  })
})

describe('GoalLookAtBlock parity over a shared raycast backend', function () {
  it('agrees with upstream face-visibility logic on 200 nodes around a wall block', function () {
    // Flat stone floor at y=0, stone wall at x=10 (z 0..8, y 1..3); the
    // target block sits inside the wall at (10,2,4), so its -x and +x
    // faces are exposed while y/z neighbours are wall or floor.
    const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 14, y1: 6, z1: 12 })
    world.fill(-2, 0, -2, 14, 0, 12, STONE)
    world.fill(10, 1, 0, 10, 3, 8, STONE)

    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot)
    const lut = lutFor(bot, movements)
    const snap = snapshotFromWorld(world, lut, true)
    expect(snap.states, 'snapshot must carry raw states for raycasting').to.not.equal(null)
    const rcWorld = new SnapshotRaycastWorld(snap.meta, snap.states as Uint16Array, lut)

    const target = new Vec3(10, 2, 4)
    const our = new GoalLookAtBlock(target, rcWorld, { reach: 4.5 })
    // Upstream gets a world shim delegating to OUR raycast, so any isEnd
    // divergence is in the face-visibility logic, not the backend.
    const shim = {
      raycast: (from: Vec3, dir: Vec3, range: number) => rcWorld.raycast(from, dir, range)
    }
    const up = new upstream.GoalLookAtBlock(target.clone(), shim, { reach: 4.5 })

    const rng = mulberry32(0x100CA7)
    let trues = 0
    let falses = 0
    for (let i = 0; i < 200; i++) {
      const node = new Vec3(randInt(rng, 4, 13), randInt(rng, 1, 5), randInt(rng, 0, 8))
      const at = `node (${node.x},${node.y},${node.z})`
      expect(our.heuristic(node), `heuristic ${at}`).to.equal(up.heuristic(node))
      const ourEnd = our.isEnd(node)
      expect(ourEnd, `isEnd ${at}`).to.equal(up.isEnd(node))
      if (ourEnd) trues++
      else falses++
    }
    // The scene must actually exercise both outcomes.
    expect(trues, 'some sampled nodes see a face of the target').to.be.greaterThan(0)
    expect(falses, 'some sampled nodes cannot see the target').to.be.greaterThan(0)
  })
})
