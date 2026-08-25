// Unit-level MoveGen checks on tiny snapshots: each move family (forward,
// jumpUp, dropDown, diagonal, parkour, ladder-up) probed in isolation against
// hand-built 3x3x3-ish scenes, plus entity-weight costs and the
// boundaryTouched flag. Complements the differential scene tests — here we
// assert exact target cells, exact costs and exact meta bits.
import { expect } from 'chai'
import {
  VoxelWorld,
  makeFakeBot,
  makeOurMovements,
  lutFor,
  snapshotFromWorld,
  mcData,
  Block,
  AIR,
  STONE,
  WATER,
  LADDER,
  OAK_FENCE,
  OAK_FENCE_DRY,
  LADDER_DRY
} from './helpers/voxelWorld.js'
import { MoveGen, META_PARKOUR, META_BOUNCE, META_CHAIN, MOM_NONE, momentumOf, momentumDx, momentumDz } from '../src/moveGen.js'
import { Snapshot } from '../src/snapshot.js'

interface GenMove {
  x: number
  y: number
  z: number
  cost: number
  meta: number
  /** Slime stand cell index of a bounce move, -1 otherwise. */
  via: number
}

interface GenCtx {
  gen: MoveGen
  snap: Snapshot
}

/** Build bot → movements → lut → snapshot → MoveGen over one voxel world. */
function makeGen (world: VoxelWorld, overrides: Record<string, unknown> = {}): GenCtx {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot, overrides)
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  const gen = new MoveGen(snap, movements.toConfig(), null)
  return { gen, snap }
}

/** Same but with sparse entity weights injected before MoveGen construction. */
function makeGenWithEntities (
  world: VoxelWorld,
  entities: Array<{ x: number, y: number, z: number, weight: number }>,
  overrides: Record<string, unknown> = {}
): GenCtx {
  const bot = makeFakeBot(world)
  const movements = makeOurMovements(bot, overrides)
  const lut = lutFor(bot, movements)
  const snap = snapshotFromWorld(world, lut)
  snap.entityIdx = Int32Array.from(entities.map(e => idxOf(snap, e.x, e.y, e.z)))
  snap.entityWeight = Int32Array.from(entities.map(e => e.weight))
  const gen = new MoveGen(snap, movements.toConfig(), null)
  return { gen, snap }
}

function idxOf (snap: Snapshot, x: number, y: number, z: number): number {
  return snap.index(x, y, z)
}

/** Run generate() and decode outIdx back into world coordinates. */
function movesOf (ctx: GenCtx, x: number, y: number, z: number): GenMove[] {
  const { gen, snap } = ctx
  gen.generate(x, y, z)
  const m = snap.meta
  const out: GenMove[] = []
  for (let i = 0; i < gen.outCount; i++) {
    const idx = gen.outIdx[i]
    const lx = idx % m.w
    const rest = (idx - lx) / m.w
    const lz = rest % m.l
    const ly = (rest - lz) / m.l
    const mv: GenMove = {
      x: lx + m.x0,
      y: ly + m.y0,
      z: lz + m.z0,
      cost: gen.outCost[i],
      meta: gen.outMeta[i],
      via: gen.outVia[i]
    }
    // Decode round-trip: our arithmetic must agree with Snapshot.index().
    expect(idxOf(snap, mv.x, mv.y, mv.z)).to.equal(idx)
    out.push(mv)
  }
  return out
}

function at (moves: GenMove[], x: number, y: number, z: number): GenMove[] {
  return moves.filter(mv => mv.x === x && mv.y === y && mv.z === z)
}

/** A bottom stone slab state (collision top 0.5) for fractional-height steps. */
function bottomSlabState (): number {
  const t = mcData.blocksByName.stone_slab as unknown as { minStateId: number, maxStateId: number }
  for (let s = t.minStateId; s <= t.maxStateId; s++) {
    const b = Block.fromStateId(s, 0) as unknown as { getProperties: () => Record<string, unknown>, shapes: number[][] }
    if (b.getProperties().type === 'bottom') {
      expect(b.shapes[0][4]).to.equal(0.5)
      return s
    }
  }
  throw new Error('no bottom stone_slab state in 1.21.1 registry')
}

describe('MoveGen', () => {
  describe('forward', () => {
    it('flat floor: exactly one forward move per cardinal, cost 1, plus 4 diagonals', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 5, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      const ctx = makeGen(world)
      const moves = movesOf(ctx, 0, 1, 0)

      // 4 forward + 4 diagonal; no jump/drop/parkour/up applies on open floor.
      expect(ctx.gen.outCount).to.equal(8)
      for (const [dx, dz] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
        const hits = at(moves, dx, 1, dz)
        expect(hits, `forward to (${dx},1,${dz})`).to.have.length(1)
        expect(hits[0].cost).to.equal(1)
        expect(hits[0].meta).to.equal(0)
      }
    })

    it('hole ahead: forward rejected, dropDown fires to the landing cell (cost 1)', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 4, y1: 8, z1: 2 })
      world.fill(-2, 3, -2, 0, 3, 2, STONE) // start platform, stand at y=4
      world.fill(1, 1, -2, 4, 1, 2, STONE) // landing floor, stand at y=2
      const moves = movesOf(makeGen(world), 0, 4, 0)

      expect(at(moves, 1, 4, 0), 'no forward over the hole').to.have.length(0)
      const drop = at(moves, 1, 2, 0)
      expect(drop, 'dropDown to (1,2,0)').to.have.length(1)
      expect(drop[0].cost).to.equal(1)
      expect(drop[0].meta).to.equal(0)
    })

    it('2-high wall ahead: no move into that column at all', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      world.set(1, 1, 0, STONE)
      world.set(1, 2, 0, STONE)
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(moves.filter(mv => mv.x === 1 && mv.z === 0)).to.have.length(0)
    })
  })

  describe('jumpUp', () => {
    it('1-high step with headroom: cost 2 to (dx, y+1, dz)', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      world.set(1, 1, 0, STONE) // the step
      const moves = movesOf(makeGen(world), 0, 1, 0)

      const up = at(moves, 1, 2, 0)
      expect(up).to.have.length(1)
      expect(up[0].cost).to.equal(2)
      expect(up[0].meta).to.equal(0)
      // and the plain forward into the step's feet cell is rejected
      expect(at(moves, 1, 1, 0)).to.have.length(0)
    })

    it('no headroom (block at y+2 over current cell): jumpUp rejected', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      world.set(1, 1, 0, STONE) // the step
      world.set(0, 3, 0, STONE) // ceiling above the start (blockA)
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(at(moves, 1, 2, 0)).to.have.length(0)
    })

    it('step of effective height 1.5 (slab floor → full block): rejected (>1.2)', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 3, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 3, 0, 2, STONE)
      world.set(0, 0, 0, bottomSlabState()) // stand on collision top 0.5
      world.set(1, 1, 0, STONE) // target top at 2.0 → rise 1.5
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(at(moves, 1, 2, 0)).to.have.length(0)
    })
  })

  describe('dropDown', () => {
    // Upstream (and our findLanding) measures maxDropDown to the landing
    // BLOCK: node.y - blockY <= maxDropDown. A stand-cell drop of d puts the
    // block at node.y - d - 1, so with maxDropDown=4 drops d=1..3 pass and
    // d=4 (ledge face of 5 counting to the block) is rejected.
    function dropWorld (d: number): VoxelWorld {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 3, y1: d + 6, z1: 2 })
      world.fill(1, 1, -2, 3, 1, 2, STONE) // landing floor, stand at y=2
      world.fill(-2, 1 + d, -2, 0, 1 + d, 2, STONE) // start platform, stand at 2+d
      return world
    }

    for (const d of [1, 2, 3]) {
      it(`stand-cell drop of ${d}: one move to the landing cell, cost 1`, () => {
        const moves = movesOf(makeGen(dropWorld(d)), 0, 2 + d, 0)
        const drop = at(moves, 1, 2, 0)
        expect(drop).to.have.length(1)
        expect(drop[0].cost).to.equal(1)
        expect(drop[0].meta).to.equal(0)
      })
    }

    it('stand-cell drop of 4: rejected with maxDropDown=4, allowed with 5', () => {
      const rejected = movesOf(makeGen(dropWorld(4)), 0, 6, 0)
      expect(at(rejected, 1, 2, 0)).to.have.length(0)

      const allowed = movesOf(makeGen(dropWorld(4), { maxDropDown: 5 }), 0, 6, 0)
      const drop = at(allowed, 1, 2, 0)
      expect(drop).to.have.length(1)
      expect(drop[0].cost).to.equal(1)
    })

    function waterDropWorld (): VoxelWorld {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 3, y1: 12, z1: 2 })
      world.fill(-2, 7, -2, 0, 7, 2, STONE) // start platform, stand at y=8
      world.fill(1, 0, -2, 3, 0, 2, STONE)
      world.fill(1, 1, -2, 3, 2, 2, WATER) // pool, surface stand cell y=2
      return world
    }

    it('6-drop into water: allowed with infiniteLiquidDropdownDistance=true', () => {
      const moves = movesOf(makeGen(waterDropWorld()), 0, 8, 0)
      const drop = at(moves, 1, 2, 0)
      expect(drop).to.have.length(1)
      expect(drop[0].cost).to.equal(1)
    })

    it('6-drop into water: rejected with infiniteLiquidDropdownDistance=false', () => {
      const moves = movesOf(makeGen(waterDropWorld(), { infiniteLiquidDropdownDistance: false }), 0, 8, 0)
      expect(at(moves, 1, 2, 0)).to.have.length(0)
    })
  })

  describe('diagonal', () => {
    function diagWorld (): VoxelWorld {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 5, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      return world
    }

    it('free corners: diagonal move cost SQRT2', () => {
      const moves = movesOf(makeGen(diagWorld()), 0, 1, 0)
      const diag = at(moves, 1, 1, 1)
      expect(diag).to.have.length(1)
      expect(diag[0].cost).to.be.closeTo(Math.SQRT2, 1e-12)
      expect(diag[0].meta).to.equal(0)
    })

    it('one corner blocked: still allowed via the free side, cost SQRT2', () => {
      const world = diagWorld()
      world.set(0, 1, 1, STONE) // corner C1 blocked (min(cost1,cost2) → free side)
      const moves = movesOf(makeGen(world), 0, 1, 0)
      const diag = at(moves, 1, 1, 1)
      expect(diag).to.have.length(1)
      expect(diag[0].cost).to.be.closeTo(Math.SQRT2, 1e-12)
    })

    it('both corners blocked: rejected (min side cost 100 → total > 100)', () => {
      const world = diagWorld()
      world.set(0, 1, 1, STONE) // corner C1
      world.set(1, 1, 0, STONE) // corner C2
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(at(moves, 1, 1, 1)).to.have.length(0)
    })
  })

  describe('parkour', () => {
    function gapWorld (gap: number): VoxelWorld {
      const far = gap + 1
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: far + 2, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 0, 0, 2, STONE) // start side
      world.fill(far, 0, -2, far + 2, 0, 2, STONE) // far side
      return world
    }

    it('1-cell gap (distance-2 jump): META_PARKOUR move to (2,1,0), cost 1', () => {
      const moves = movesOf(makeGen(gapWorld(1)), 0, 1, 0)
      expect(at(moves, 1, 1, 0), 'no plain forward over the gap').to.have.length(0)
      const parkour = moves.filter(mv => (mv.meta & META_PARKOUR) !== 0)
      expect(parkour).to.have.length(1)
      expect(parkour[0]).to.deep.equal({ x: 2, y: 1, z: 0, cost: 1, meta: META_PARKOUR, via: -1 })
    })

    it('3-cell gap (distance-4 jump): present with sprinting, absent without', () => {
      const sprint = movesOf(makeGen(gapWorld(3)), 0, 1, 0)
      const jump = at(sprint, 4, 1, 0)
      expect(jump).to.have.length(1)
      expect(jump[0].meta).to.equal(META_PARKOUR)
      expect(jump[0].cost).to.equal(1)

      const noSprint = movesOf(makeGen(gapWorld(3), { allowSprinting: false }), 0, 1, 0)
      expect(noSprint.filter(mv => (mv.meta & META_PARKOUR) !== 0)).to.have.length(0)
    })

    it('ceiling over the start (y+2): forward parkour absent', () => {
      const world = gapWorld(1)
      world.set(0, 3, 0, STONE) // ceilingClear = false from the start
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(at(moves, 2, 1, 0)).to.have.length(0)
      expect(moves.filter(mv => (mv.meta & META_PARKOUR) !== 0)).to.have.length(0)
    })
  })

  describe('extended parkour (allowParkourExtended)', () => {
    const FLAG = { allowParkourExtended: true }
    const KNIGHT = Math.hypot(2, 1) + 0.5
    const FULL = Math.hypot(2, 2) + 0.5

    /** Start pillar at (0,0) and one landing pillar per +x/+z diagonal target, over void.
     * y0 sits below the pillars: the landing scan (upstream findLanding
     * convention) cannot land on floors at the world-box bottom. */
    function pillarWorld (): VoxelWorld {
      const world = new VoxelWorld({ x0: -3, y0: -2, z0: -3, x1: 5, y1: 7, z1: 5 })
      world.set(0, 0, 0, STONE)
      world.set(2, 0, 1, STONE) // knight (2,1)
      world.set(1, 0, 2, STONE) // knight (1,2)
      world.set(2, 0, 2, STONE) // full (2,2) — its corner cells (2,1)/(1,2) hold
      return world //             exactly-level pillar tops, which must NOT veto
    }

    it('generates exactly the three +x/+z jumps, META_PARKOUR, exact costs', () => {
      const moves = movesOf(makeGen(pillarWorld(), FLAG), 0, 1, 0)
      expect(moves).to.have.length(3)
      expect(moves.every(mv => mv.meta === META_PARKOUR)).to.equal(true)
      expect(at(moves, 2, 1, 1)[0].cost).to.be.closeTo(KNIGHT, 1e-12)
      expect(at(moves, 1, 1, 2)[0].cost).to.be.closeTo(KNIGHT, 1e-12)
      expect(at(moves, 2, 1, 2)[0].cost).to.be.closeTo(FULL, 1e-12)
    })

    it('flag off (default), sprint off or parkour off: nothing', () => {
      expect(movesOf(makeGen(pillarWorld()), 0, 1, 0)).to.have.length(0)
      expect(movesOf(makeGen(pillarWorld(), { ...FLAG, allowSprinting: false }), 0, 1, 0)).to.have.length(0)
      expect(movesOf(makeGen(pillarWorld(), { ...FLAG, allowParkour: false }), 0, 1, 0)).to.have.length(0)
    })

    it('ceiling over the start (y+2): head-hitter class — flat hops survive', () => {
      // A lid 2 above the feet no longer kills the jumps: the arc bonks at
      // +0.2 and flies flattened (J_LOW rows). The knight hops fit the
      // bonked reach; the (2,2) full diagonal does NOT survive here — its
      // bonked arc dips below takeoff level over the corner cells, which
      // hold the same-level neighbor pillars (real clip, correctly vetoed).
      const world = pillarWorld()
      world.set(0, 3, 0, STONE)
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(at(moves, 2, 1, 1)).to.have.length(1)
      expect(at(moves, 1, 1, 2)).to.have.length(1)
      expect(at(moves, 2, 1, 2)).to.have.length(0)

      // With clear corners the bonked (2,2) is legal.
      const clear = new VoxelWorld({ x0: -3, y0: -2, z0: -3, x1: 5, y1: 7, z1: 5 })
      clear.set(0, 0, 0, STONE)
      clear.set(2, 0, 2, STONE)
      clear.set(0, 3, 0, STONE)
      expect(at(movesOf(makeGen(clear, FLAG), 0, 1, 0), 2, 1, 2)).to.have.length(1)
    })

    it('head-hitter up-jumps are impossible; y+1 blockage still vetoes', () => {
      // +1 landing under a lid: cannot rise a block with 0.2 of headroom.
      const up = pillarWorld()
      up.set(0, 3, 0, STONE)
      up.set(2, 1, 1, STONE) // raise the (2,1) landing to flight level
      expect(at(movesOf(makeGen(up, FLAG), 0, 1, 0), 2, 2, 1)).to.have.length(0)

      // A block at HEAD height over a line cell blocks the body — veto.
      const body = pillarWorld()
      body.set(1, 2, 1, STONE) // y+1 over shared line cell (1,1)
      expect(movesOf(makeGen(body, FLAG), 0, 1, 0).filter(mv => (mv.meta & META_PARKOUR) !== 0)).to.have.length(0)
    })

    it('corner nick is cleared (vanilla slides); real cuts and line cells veto', () => {
      // (0,1) is only NICKED by the (2,1) flight — under 0.15 penetration on
      // the shallow axis, which vanilla resolves as a per-axis slide — so
      // that jump survives. The same cell is a LINE cell of (1,2) and a real
      // corner cut of (2,2): a fence poking 1.5 up vetoes both.
      const world = pillarWorld()
      world.set(0, 0, 1, OAK_FENCE)
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(at(moves, 2, 1, 1)).to.have.length(1)
      expect(at(moves, 1, 1, 2)).to.have.length(0)
      expect(at(moves, 2, 1, 2)).to.have.length(0)
    })

    it('fence under the apex is cleared; the same fence near takeoff vetoes', () => {
      // Long running (4,0): over the middle line cell the feet are at apex
      // (mf ≈ 0.87 → clears a 1.5 fence top), but right after takeoff the
      // feet are still at ground level (mf = 0 → the fence pokes in).
      const build = (fenceX: number): VoxelWorld => {
        const w = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 8, y1: 7, z1: 4 })
        w.set(0, 0, 0, STONE) // takeoff, stand y=1
        w.set(-1, 0, 0, STONE) // run-up cell
        w.set(4, 0, 0, STONE) // landing
        w.set(fenceX, 0, 0, OAK_FENCE)
        return w
      }
      const ext = (w: VoxelWorld): unknown[] => at(movesOf(makeGen(w, FLAG), 0, 1, 0), 4, 1, 0)
      expect(ext(build(2)), 'apex clears the fence').to.have.length(1)
      expect(ext(build(1)), 'fence right after takeoff vetoes').to.have.length(0)
    })

    it('supersedes upstream\'s cardinal parkour rather than coexisting with it', () => {
      // Upstream's generator charges a flat 1 for a jump covering up to four
      // blocks — less than one walking step — and applies neither the reach
      // envelope nor the swept-corridor clearance. Left switched on beside the
      // table it wins every tie, so it re-offers, at a cheaper price, exactly
      // the jumps parkourExtTarget just vetoed (the fence case above), and A*
      // buys distance with jumps: on 2b2t spawn that produced a plan costing
      // 74.1 against upstream's 76.6 that was 3.5 blocks LONGER to walk. It
      // also makes the octile heuristic inadmissible (h = 4 over a cost-1
      // edge), so the search is not optimal under its own model either.
      //
      // Keeping it is not a reachability argument: over 120 seeded worlds and
      // 108k parkour-bearing nodes it reached 22.5k targets the table does
      // not, and a prismarine-physics rollout of a sample of those — from the
      // cell centre, the take-off corner, and one and two blocks of run-up —
      // could fly 1.1% of them. The rest are jumps the bot cannot make, each
      // one a planned stall.
      // A plain cardinal gap: take-off, three empty cells, landing.
      const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 8, y1: 7, z1: 4 })
      world.set(0, 0, 0, STONE)
      world.set(-1, 0, 0, STONE) // run-up cell
      world.set(4, 0, 0, STONE)
      const withTable = movesOf(makeGen(world, FLAG), 0, 1, 0).filter(mv => (mv.meta & META_PARKOUR) !== 0)
      const upstreamOnly = movesOf(makeGen(world), 0, 1, 0).filter(mv => (mv.meta & META_PARKOUR) !== 0)

      expect(upstreamOnly.some(mv => mv.cost === 1), 'upstream prices a jump at one walking step').to.equal(true)
      expect(withTable.some(mv => mv.cost === 1), 'the table never does').to.equal(false)
      for (const mv of withTable) expect(mv.cost).to.be.greaterThan(2)
    })

    it('walkable floor on the flight line: not a gap, all vetoed', () => {
      const world = pillarWorld()
      world.set(1, 0, 1, STONE) // line cell (1,1) is shared by all three targets
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(moves.filter(mv => (mv.meta & META_PARKOUR) !== 0)).to.have.length(0)
      expect(at(moves, 1, 1, 1)).to.have.length(1) // plain diagonal walk instead
    })

    it('blocked apex cell over the flight line: head-hitter class, knight hops survive', () => {
      const world = pillarWorld()
      world.set(1, 3, 1, STONE) // y+2 over line cell (1,1) — a lid, not a wall
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      const parkour = moves.filter(mv => (mv.meta & META_PARKOUR) !== 0)
      expect(at(parkour, 2, 1, 1)).to.have.length(1)
      expect(at(parkour, 1, 1, 2)).to.have.length(1)
      // (2,2): bonked arc clips the same-level corner pillars — vetoed.
      expect(at(parkour, 2, 1, 2)).to.have.length(0)
    })

    it('up-jump: flight-level block at the target lands on top, cost +1', () => {
      const world = pillarWorld()
      world.set(2, 1, 1, STONE) // raise the (2,1) landing to flight level
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      const up = at(moves, 2, 2, 1)
      expect(up).to.have.length(1)
      expect(up[0].meta).to.equal(META_PARKOUR)
      expect(up[0].cost).to.be.closeTo(KNIGHT + 1, 1e-12)
    })

    it('drop-jumps: land below takeoff, capped by maxDropDown', () => {
      const world = new VoxelWorld({ x0: -3, y0: -2, z0: -3, x1: 5, y1: 9, z1: 5 })
      world.fill(0, 0, 0, 0, 3, 0, STONE) // start pillar, stand y=4
      world.set(2, 0, 1, STONE) // landing stands y=1 — a 3-block drop
      const moves = movesOf(makeGen(world, FLAG), 0, 4, 0)
      const drop = at(moves, 2, 1, 1)
      expect(drop).to.have.length(1)
      expect(drop[0].cost).to.be.closeTo(KNIGHT, 1e-12)
      expect(drop[0].meta).to.equal(META_PARKOUR)

      const capped = movesOf(makeGen(world, { ...FLAG, maxDropDown: 2 }), 0, 4, 0)
      expect(at(capped, 2, 1, 1)).to.have.length(0)
    })

    it('cardinal ladder catch: jump the gap and grab the ladder at flight level', () => {
      const world = new VoxelWorld({ x0: -3, y0: -2, z0: -3, x1: 5, y1: 7, z1: 5 })
      world.set(0, 0, 0, STONE) // stand y=1
      world.set(2, 1, 0, LADDER)
      world.set(2, 2, 0, LADDER)
      world.fill(3, 1, 0, 3, 2, 0, STONE) // backing wall
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      const grab = at(moves, 2, 1, 0)
      expect(grab).to.have.length(1)
      expect(grab[0].cost).to.equal(2.5)
      expect(grab[0].meta).to.equal(META_PARKOUR)
    })

    it('water catch: diagonal jump into a shallow pool', () => {
      const world = pillarWorld()
      world.set(2, 0, 1, WATER) // the (2,1) column holds water instead of a pillar
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      const splash = at(moves, 2, 0, 1)
      expect(splash).to.have.length(1)
      expect(splash[0].cost).to.be.closeTo(KNIGHT, 1e-12)
      expect(splash[0].meta).to.equal(META_PARKOUR)
    })

    it('long jumps: a 1x1 block is its own run-up — (4,2) works, (5,2) never', () => {
      // A player sprints across the block from its rear edge and jumps at
      // the lip (the run-length envelope, parkourEnvelope.ts J_RUN), so the
      // diagonals up to (4,2) go from a lone block; (5,2) is beyond the
      // physics with or without a cell behind.
      const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 6, y1: 7, z1: 6 })
      world.set(0, 0, 0, STONE)
      world.set(3, 0, 1, STONE)
      world.set(3, 0, 2, STONE)
      const lone = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(at(lone, 3, 1, 1)[0].cost).to.be.closeTo(Math.hypot(3, 1) + 0.5, 1e-12)
      expect(at(lone, 3, 1, 2)[0].cost).to.be.closeTo(Math.hypot(3, 2) + 0.5, 1e-12)
      const four = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 6, y1: 7, z1: 6 })
      four.set(0, 0, 0, STONE)
      four.set(4, 0, 2, STONE) // nothing under the flight line: a real gap
      expect(at(movesOf(makeGen(four, FLAG), 0, 1, 0), 4, 1, 2)[0].cost).to.be.closeTo(Math.hypot(4, 2) + 0.5, 1e-12)

      const far = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 7, y1: 7, z1: 6 })
      far.set(0, 0, 0, STONE)
      far.set(5, 0, 2, STONE)
      expect(at(movesOf(makeGen(far, FLAG), 0, 1, 0), 5, 1, 2)).to.have.length(0)
      far.set(-1, 0, 0, STONE) // a run-up cell adds nothing: sprint saturates within a block
      expect(at(movesOf(makeGen(far, FLAG), 0, 1, 0), 5, 1, 2)).to.have.length(0)
    })

    it('MLG drop-boost: (5,1) is beyond running flat reach but lands 1 below', () => {
      const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 7, y1: 9, z1: 6 })
      world.fill(-1, 0, 0, 0, 1, 0, STONE) // takeoff (stand y=2) + same-level run-up
      world.fill(5, 0, 1, 5, 1, 1, STONE) // flat (5,1) landing at takeoff level…
      const flat = movesOf(makeGen(world, FLAG), 0, 2, 0)
      expect(at(flat, 5, 2, 1)).to.have.length(0) // …3.81 flight > flat running: refused

      world.set(5, 1, 1, AIR) // lower the landing by one: drop adds airtime
      const drop = movesOf(makeGen(world, FLAG), 0, 2, 0)
      const mlg = at(drop, 5, 1, 1)
      expect(mlg).to.have.length(1)
      expect(mlg[0].cost).to.be.closeTo(Math.hypot(5, 1) + 0.5, 1e-12)

      world.set(-1, 1, 0, AIR) // no run-up cell: the block's own 1.38 of run still makes it
      world.set(-1, 0, 0, AIR)
      expect(at(movesOf(makeGen(world, FLAG), 0, 2, 0), 5, 1, 1)).to.have.length(1)
    })
  })

  describe('up (ladder)', () => {
    function ladderWorld (): VoxelWorld {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 6, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      return world
    }

    it('ladder at the node with clear y+2: up move to (0,2,0), cost 1', () => {
      const world = ladderWorld()
      world.set(0, 1, 0, LADDER)
      const moves = movesOf(makeGen(world), 0, 1, 0)
      const up = at(moves, 0, 2, 0)
      expect(up).to.have.length(1)
      expect(up[0].cost).to.equal(1)
      expect(up[0].meta).to.equal(0)
    })

    it('ladder but block at y+2: up rejected (safeOrBreak of y+2 is 100)', () => {
      const world = ladderWorld()
      world.set(0, 1, 0, LADDER)
      world.set(0, 3, 0, STONE)
      const moves = movesOf(makeGen(world), 0, 1, 0)
      expect(at(moves, 0, 2, 0)).to.have.length(0)
    })

    it('up cost includes the y+2 safeOrBreak entity weight', () => {
      const world = ladderWorld()
      world.set(0, 1, 0, LADDER)
      const ctx = makeGenWithEntities(world, [{ x: 0, y: 3, z: 0, weight: 2 }])
      const moves = movesOf(ctx, 0, 1, 0)
      const up = at(moves, 0, 2, 0)
      expect(up).to.have.length(1)
      expect(up[0].cost).to.equal(3) // 1 move + 2 entity weight at y+2, entityCost=1
    })

    it('no ladder: no up move', () => {
      const moves = movesOf(makeGen(ladderWorld()), 0, 1, 0)
      expect(at(moves, 0, 2, 0)).to.have.length(0)
    })
  })

  describe('entity weights', () => {
    it('weight 3 at the forward target cell C: forward cost 1 + 3', () => {
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 5, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)

      // control: no entities → cost 1
      const clean = movesOf(makeGen(world), 0, 1, 0)
      expect(at(clean, 1, 1, 0)[0].cost).to.equal(1)

      const ctx = makeGenWithEntities(world, [{ x: 1, y: 1, z: 0, weight: 3 }])
      const moves = movesOf(ctx, 0, 1, 0)
      const fwd = at(moves, 1, 1, 0)
      expect(fwd).to.have.length(1)
      expect(fwd[0].cost).to.equal(4) // 1 move + 3*entityCost via safeOrBreak(C)
    })
  })

  describe('boundaryTouched', () => {
    function bigWorld (): VoxelWorld {
      const world = new VoxelWorld({ x0: -6, y0: 0, z0: -6, x1: 6, y1: 8, z1: 6 })
      world.fill(-6, 2, -6, 6, 2, 6, STONE) // floor at y=2, stand at y=3
      return world
    }

    it('interior expansion leaves boundaryTouched false', () => {
      const ctx = makeGen(bigWorld())
      const moves = movesOf(ctx, 0, 3, 0)
      expect(moves.length).to.be.greaterThan(0)
      expect(ctx.gen.boundaryTouched).to.equal(false)
    })

    it('expansion near the box edge sets boundaryTouched', () => {
      const ctx = makeGen(bigWorld())
      movesOf(ctx, 6, 3, 0) // node on the x1 edge: forward east probes x=7, outside the box
      expect(ctx.gen.boundaryTouched).to.equal(true)
    })
  })

  describe('extended parkour: narrow supports, fence tops, ladders, slime', () => {
    const FLAG = { allowParkourExtended: true }
    function voidWorld (): VoxelWorld {
      return new VoxelWorld({ x0: -4, y0: -3, z0: -4, x1: 8, y1: 9, z1: 8 })
    }

    it('fence top is a landing: a (2,0) hop onto a post, feet 0.5 into the node cell', () => {
      const world = voidWorld()
      world.set(0, 0, 0, STONE)
      world.set(2, 0, 0, OAK_FENCE_DRY) // top 1.5 → stand (2,1,0)
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      const hop = at(moves, 2, 1, 0)
      expect(hop).to.have.length(1)
      expect(hop[0].meta).to.equal(META_PARKOUR)
      expect(hop[0].cost).to.be.closeTo(2.5, 1e-12)
      // Upstream classification: not physical, not safe — no landing at all.
      expect(at(movesOf(makeGen(world), 0, 1, 0), 2, 1, 0)).to.have.length(0)
      // Head needs the cell above the usual pair: a lid at y+3 vetoes.
      world.set(2, 3, 0, STONE)
      expect(at(movesOf(makeGen(world, FLAG), 0, 1, 0), 2, 1, 0)).to.have.length(0)
    })

    it('narrow credits: a (3,1)+1 hop between posts is real (recorded on parkouradv1), a (3,2)+1 is not', () => {
      const posts = voidWorld()
      posts.set(0, 0, 0, OAK_FENCE_DRY) // stand (0,1,0), feet 1.5
      posts.fill(1, 0, 3, 1, 1, 3, OAK_FENCE_DRY) // 2-stack, top 2.5 → stand (1,2,3): rise 1.0
      posts.fill(2, 0, 3, 2, 1, 3, OAK_FENCE_DRY) // (2,3): the same rise, half a block further out
      const fromPost = movesOf(makeGen(posts, FLAG), 0, 1, 0)
      expect(at(fromPost, 1, 2, 3)).to.have.length(1) // 2.21 needed, 2.35 usable off a 0.81 run
      expect(at(fromPost, 2, 2, 3)).to.have.length(0) // 2.53 needed

      const blocks = voidWorld()
      blocks.set(0, 0, 0, STONE)
      blocks.set(1, 1, 3, STONE) // flight-level block: up landing (1,2,3), rise 1.0
      const hop = at(movesOf(makeGen(blocks, FLAG), 0, 1, 0), 1, 2, 3)
      expect(hop).to.have.length(1)
      expect(hop[0].meta).to.equal(META_PARKOUR)
    })

    it('narrow credits: landing a (2,1) hop on a head from a post fits standing reach', () => {
      const world = voidWorld()
      world.set(0, 0, 0, OAK_FENCE_DRY) // stand (0,1,0), feet 1.5
      const head = mcData.blocksByName.creeper_head.minStateId as number
      world.set(2, 1, 1, head) // top 1.5 → stand (2,2,1): flat from the post
      const hop = at(movesOf(makeGen(world, FLAG), 0, 1, 0), 2, 2, 1)
      expect(hop).to.have.length(1)
      expect(hop[0].meta).to.equal(META_PARKOUR)
    })

    it('sunk fence: walk onto a post whose top is 0.5 above the floor', () => {
      const world = voidWorld()
      world.set(0, 0, 0, STONE) // stand (0,1,0)
      world.set(1, 0, 0, OAK_FENCE_DRY) // top 1.5: step of 0.5 → node (1,1,0)
      const step = at(movesOf(makeGen(world, FLAG), 0, 1, 0), 1, 1, 0)
      expect(step).to.have.length(1)
      expect(step[0].cost).to.equal(1)
      expect(step[0].meta).to.equal(0)
      expect(at(movesOf(makeGen(world), 0, 1, 0), 1, 1, 0)).to.have.length(0)
      // A post one higher is a 1.5 step: not walkable, and not jumpable either (> 1.2).
      world.set(1, 1, 0, OAK_FENCE_DRY)
      const moves = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(at(moves, 1, 1, 0)).to.have.length(0)
      expect(at(moves, 1, 2, 0)).to.have.length(0)
    })

    it('jump up from a post onto the next post one higher (rise 1.0), cost 2', () => {
      const world = voidWorld()
      world.set(0, 0, 0, OAK_FENCE_DRY) // stand (0,1,0), feet 1.5
      world.set(1, 1, 0, OAK_FENCE_DRY) // top 2.5 → stand (1,2,0)
      const up = at(movesOf(makeGen(world, FLAG), 0, 1, 0), 1, 2, 0)
      expect(up).to.have.length(1)
      expect(up[0].cost).to.equal(2)
      expect(up[0].meta).to.equal(0)
    })

    it('drop onto a fence top below: node one above the post, capped by maxDropDown', () => {
      const world = voidWorld()
      world.fill(0, 0, 0, 0, 2, 0, STONE) // stand (0,3,0)
      world.set(1, -1, 0, OAK_FENCE_DRY) // top 0.5 → stand (1,0,0): drop 3
      const drop = at(movesOf(makeGen(world, FLAG), 0, 3, 0), 1, 0, 0)
      expect(drop).to.have.length(1)
      expect(drop[0].cost).to.equal(1)
      expect(at(movesOf(makeGen(world), 0, 3, 0), 1, 0, 0)).to.have.length(0)
    })

    it('step into a free-hanging ladder cell from an adjacent stand', () => {
      const world = voidWorld()
      world.set(0, 0, 0, STONE) // stand (0,1,0)
      world.set(1, 1, 0, LADDER) // ladder over void, mounted on...
      world.fill(2, 0, 0, 2, 3, 0, STONE) // ...this wall
      const step = at(movesOf(makeGen(world, FLAG), 0, 1, 0), 1, 1, 0)
      expect(step).to.have.length(1)
      expect(step[0].cost).to.equal(1)
      expect(at(movesOf(makeGen(world), 0, 1, 0), 1, 1, 0)).to.have.length(0)
    })

    it('no extended jumps from a ladder cell; climb transfers round the pillar instead', () => {
      const world = voidWorld()
      world.fill(0, 0, 0, 0, 5, 0, STONE) // 1x1 pillar
      world.set(0, 1, -1, LADDER) // north face
      world.set(-1, 2, 0, LADDER) // west face, one up
      world.set(0, 3, 1, LADDER) // south face
      world.set(1, 4, 0, LADDER) // east face
      world.set(3, 0, -1, STONE) // a landing an ext jump from the ladder would otherwise take
      const moves = movesOf(makeGen(world, FLAG), 0, 1, -1)
      expect(moves.every(mv => mv.meta === 0)).to.equal(true)
      expect(at(moves, 3, 1, -1)).to.have.length(0)
      const west = at(moves, -1, 2, 0)
      expect(west).to.have.length(1)
      expect(west[0].cost).to.be.closeTo(Math.SQRT2 + 0.5 + 1, 1e-12)
      // The straight climb is still there, and nothing goes to the other ladders directly.
      expect(at(moves, 0, 2, -1)).to.have.length(1)
      expect(at(moves, 0, 3, 1)).to.have.length(0)
      // Whole spiral, one transfer per corner.
      const gen = makeGen(world, FLAG)
      expect(at(movesOf(gen, -1, 2, 0), 0, 3, 1)).to.have.length(1)
      expect(at(movesOf(gen, 0, 3, 1), 1, 4, 0)).to.have.length(1)
      // Flag off: no transfer.
      expect(at(movesOf(makeGen(world), 0, 1, -1), -1, 2, 0)).to.have.length(0)
      // Both corner columns blocked: no way round.
      world.fill(-1, 1, -1, -1, 3, -1, STONE)
      expect(at(movesOf(makeGen(world, FLAG), 0, 1, -1), -1, 2, 0)).to.have.length(0)
    })

    it('momentum chain: a stepping-stone re-jump reaches what a jump from the stone cannot', () => {
      // A → B is a plain (3,0) hop onto a fence post. B → C is (5,2) a block
      // and a half down: off the post's 0.81 of run it needs 3.93 against
      // 3.90 usable; the chain row from the landing point (4.35, half the
      // creep credit) covers it at 4.15. So A → C exists, via B.
      const world = new VoxelWorld({ x0: -3, y0: -3, z0: -4, x1: 12, y1: 7, z1: 6 })
      world.set(0, 0, 0, STONE)
      world.set(3, 0, 0, OAK_FENCE_DRY) // stand (3,1,0), feet 1.5
      world.set(8, -1, 2, STONE) // stand (8,0,2)
      const ctx = makeGen(world, FLAG)
      const moves = movesOf(ctx, 0, 1, 0)
      const chain = at(moves, 8, 0, 2)
      expect(chain).to.have.length(1)
      expect(chain[0].meta).to.equal(META_PARKOUR | META_CHAIN)
      expect(chain[0].via).to.equal(ctx.snap.index(3, 1, 0))
      expect(chain[0].cost).to.be.closeTo(3.5 + Math.hypot(5, 2) + 0.5, 1e-12)
      // B itself is a plain parkour landing, and its own expansion cannot reach C.
      expect(at(moves, 3, 1, 0)[0].meta).to.equal(META_PARKOUR)
      expect(at(movesOf(ctx, 3, 1, 0), 8, 0, 2)).to.have.length(0)
      // Turning away kills the momentum: a (0,5) hop off B is not chained.
      world.set(3, -1, 5, STONE)
      expect(at(movesOf(makeGen(world, FLAG), 0, 1, 0), 3, 0, 5).filter(mv => (mv.meta & META_CHAIN) !== 0)).to.have.length(0)
      // A stone with a walkable neighbour is not a stepping stone: its own
      // jumps run from there, so no chain is emitted.
      world.set(2, 0, 0, STONE)
      const wide = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(wide.filter(mv => (mv.meta & META_CHAIN) !== 0)).to.have.length(0)
      // Flag off: nothing.
      expect(movesOf(makeGen(world), 0, 1, 0).filter(mv => mv.via >= 0)).to.have.length(0)
    })

    it('slime bounce: a drop of 3 onto slime reaches a ledge 1.5 up beside it, via the slime', () => {
      const slime = mcData.blocksByName.slime_block.minStateId as number
      const world = voidWorld()
      world.fill(0, 0, 0, 0, 2, 0, STONE) // stand (0,3,0)
      world.set(1, -1, 0, slime) // stand (1,0,0): drop 3 → rebound apex 2.10
      world.set(2, 1, 0, bottomSlabState()) // top 1.5 → stand (2,2,0): under 0 + 2.10 - 0.2
      world.set(1, 1, 2, STONE) // ring-2 (0,+2): top 2 > 0 + 2.10 - 1.0 → out of reach
      world.set(-1, 1, 1, STONE) // ring-1 (-1,+1): stand (-1,2,1), top 2 > 1.9 → out of reach
      const ctx = makeGen(world, FLAG)
      const moves = movesOf(ctx, 0, 3, 0)
      const drop = at(moves, 1, 0, 0)
      expect(drop).to.have.length(1)
      expect(drop[0].meta).to.equal(0)
      // The ledge is also a plain drop-jump from the takeoff; the bounce
      // edge is the one carrying META_BOUNCE and the slime as `via`.
      const bounces = (ms: GenMove[]): GenMove[] => ms.filter(mv => (mv.meta & META_BOUNCE) !== 0)
      const bounce = bounces(at(moves, 2, 2, 0))
      expect(bounce).to.have.length(1)
      expect(bounce[0].meta).to.equal(META_PARKOUR | META_BOUNCE)
      expect(bounce[0].cost).to.be.closeTo(2 + 1 + 1, 1e-12) // octile(2,0) + |dy| + 1
      expect(bounce[0].via).to.equal(ctx.snap.index(1, 0, 0))
      expect(bounces(at(moves, 1, 2, 2))).to.have.length(0)
      expect(bounces(at(moves, -1, 2, 1))).to.have.length(0)
      expect(moves.filter(mv => mv.via >= 0)).to.have.length(1)
      expect(moves.filter(mv => mv.via >= 0 && (mv.meta & META_BOUNCE) === 0)).to.have.length(0)
      // Flag off: no bounce (and no slime marking at all).
      expect(bounces(movesOf(makeGen(world), 0, 3, 0))).to.have.length(0)
      // A drop of 2 rebounds 1.30: the same ledge is out of reach.
      const low = voidWorld()
      low.fill(0, 0, 0, 0, 1, 0, STONE) // stand (0,2,0)
      low.set(1, -1, 0, slime)
      low.set(2, 1, 0, bottomSlabState())
      expect(bounces(movesOf(makeGen(low, FLAG), 0, 2, 0))).to.have.length(0)
    })
  })
})

describe('momentum search state (allowParkourMomentum)', () => {
  const FLAG = { allowParkourExtended: true }
  const MOM = { allowParkourExtended: true, allowParkourMomentum: true }
  /** movesOf with the node's momentum state, plus each neighbour's outgoing momentum. */
  function movesFrom (ctx: GenCtx, x: number, y: number, z: number, mom: number): Array<GenMove & { mom: number }> {
    const { gen, snap } = ctx
    gen.generate(x, y, z, mom)
    const m = snap.meta
    const out: Array<GenMove & { mom: number }> = []
    for (let i = 0; i < gen.outCount; i++) {
      const idx = gen.outIdx[i]
      const lx = idx % m.w
      const rest = (idx - lx) / m.w
      const lz = rest % m.l
      const ly = (rest - lz) / m.l
      out.push({ x: lx + m.x0, y: ly + m.y0, z: lz + m.z0, cost: gen.outCost[i], meta: gen.outMeta[i], via: gen.outVia[i], mom: gen.outMom[i] })
    }
    return out
  }
  /** A → B (3,0) onto a post → C (5,2) a block and a half down → D (6,2) three down. */
  function chainWorld (): VoxelWorld {
    const world = new VoxelWorld({ x0: -3, y0: -6, z0: -4, x1: 18, y1: 7, z1: 8 })
    world.set(0, 0, 0, STONE) // A: stand (0,1,0)
    world.set(3, 0, 0, OAK_FENCE_DRY) // B: stand (3,1,0), feet 1.5
    world.set(8, -1, 2, STONE) // C: stand (8,0,2)
    world.set(14, -4, 4, STONE) // D: stand (14,-3,4)
    return world
  }

  it('encodes the primitive landing direction exactly and round-trips it', () => {
    for (const [dx, dz] of [[3, 0], [5, 2], [6, 2], [-4, 1], [0, -5], [-6, -6], [2, -6]]) {
      const m = momentumOf(dx, dz)
      expect(m).to.be.greaterThan(MOM_NONE)
      expect(m).to.be.at.most(255)
      let a = Math.abs(dx); let b = Math.abs(dz)
      while (b !== 0) { const t = a % b; a = b; b = t }
      expect([momentumDx(m), momentumDz(m)]).to.deep.equal([dx / a, dz / a])
    }
    expect(momentumOf(6, 2)).to.equal(momentumOf(3, 1))
    expect(momentumOf(3, 1)).to.not.equal(momentumOf(1, 3))
  })

  it('a support landing carries its flight direction; a catch carries none', () => {
    const world = chainWorld()
    world.set(3, 1, 3, LADDER_DRY) // a ladder catch beside B
    world.fill(3, 0, 4, 3, 2, 4, STONE) // ...mounted on this wall
    const moves = movesFrom(makeGen(world, MOM), 0, 1, 0, MOM_NONE)
    const post = at(moves, 3, 1, 0)
    expect(post).to.have.length(1)
    expect(post[0].meta).to.equal(META_PARKOUR)
    expect((post[0] as { mom: number }).mom).to.equal(momentumOf(3, 0))
    const catchMove = at(moves, 3, 1, 3)
    expect(catchMove).to.have.length(1)
    expect((catchMove[0] as { mom: number }).mom).to.equal(MOM_NONE)
    // Flag off (compound chains): no momentum on anything.
    expect(movesFrom(makeGen(world, FLAG), 0, 1, 0, MOM_NONE).every(mv => (mv as { mom: number }).mom === MOM_NONE)).to.equal(true)
  })

  it('the chain is a transition from the landing state, not a compound edge', () => {
    const world = chainWorld()
    const ctx = makeGen(world, MOM)
    // From A nothing is folded: B is a plain landing, C is not an edge of A.
    const fromA = movesFrom(ctx, 0, 1, 0, MOM_NONE)
    expect(at(fromA, 8, 0, 2)).to.have.length(0)
    expect(fromA.filter(mv => (mv.meta & META_CHAIN) !== 0)).to.have.length(0)
    // From B at rest: C is out of reach (as before).
    expect(at(movesFrom(ctx, 3, 1, 0, MOM_NONE), 8, 0, 2)).to.have.length(0)
    // From B carrying the (3,0) landing: C is a chain edge priced as its own hop,
    // via the stone itself, and lands carrying (5,2).
    const fromB = movesFrom(ctx, 3, 1, 0, momentumOf(3, 0))
    const c = at(fromB, 8, 0, 2)
    expect(c).to.have.length(1)
    expect(c[0].meta).to.equal(META_PARKOUR | META_CHAIN)
    expect(c[0].via).to.equal(ctx.snap.index(3, 1, 0))
    expect(c[0].cost).to.be.closeTo(Math.hypot(5, 2) + 0.5, 1e-12)
    expect((c[0] as { mom: number }).mom).to.equal(momentumOf(5, 2))
    // The landing state is a complete node: the ordinary moves are there too.
    expect(fromB.filter(mv => (mv.meta & META_CHAIN) === 0).length).to.be.greaterThan(0)
    // Turning away kills the momentum: the same offset mirrored is not chained.
    world.set(-2, -1, 2, STONE)
    expect(at(movesFrom(makeGen(world, MOM), 3, 1, 0, momentumOf(3, 0)), -2, 0, 2)).to.have.length(0)
    // The compound model still exists with the flag off, as A → C via B.
    const compound = at(movesFrom(makeGen(world, FLAG), 0, 1, 0, MOM_NONE), 8, 0, 2)
    expect(compound).to.have.length(1)
    expect(compound[0].meta).to.equal(META_PARKOUR | META_CHAIN)
  })

  it('chains chain: the second landing re-jumps again, and without momentum it cannot', () => {
    const ctx = makeGen(chainWorld(), MOM)
    // C at rest: D (6,2) three down needs 4.67 against 4.63 of run reach.
    expect(at(movesFrom(ctx, 8, 0, 2, MOM_NONE), 14, -3, 4)).to.have.length(0)
    // C landed from (5,2): the chain row covers it (5.02 against 5.10).
    const d = at(movesFrom(ctx, 8, 0, 2, momentumOf(5, 2)), 14, -3, 4)
    expect(d).to.have.length(1)
    expect(d[0].meta).to.equal(META_PARKOUR | META_CHAIN)
    expect(d[0].via).to.equal(ctx.snap.index(8, 0, 2))
    expect((d[0] as { mom: number }).mom).to.equal(momentumOf(6, 2))
    // A momentum too far off the line does not.
    expect(at(movesFrom(ctx, 8, 0, 2, momentumOf(0, 1)), 14, -3, 4)).to.have.length(0)
  })

  it('a lid over the stone forbids the chain (no bonked chain row)', () => {
    const world = chainWorld()
    world.set(3, 3, 0, STONE) // head+1 over B
    const ctx = makeGen(world, MOM)
    expect(at(movesFrom(ctx, 3, 1, 0, momentumOf(3, 0)), 8, 0, 2)).to.have.length(0)
  })
})
