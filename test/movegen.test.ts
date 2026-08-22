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
  OAK_FENCE
} from './helpers/voxelWorld.js'
import { MoveGen, META_PARKOUR } from '../src/moveGen.js'
import { Snapshot } from '../src/snapshot.js'

interface GenMove {
  x: number
  y: number
  z: number
  cost: number
  meta: number
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
      meta: gen.outMeta[i]
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
      expect(parkour[0]).to.deep.equal({ x: 2, y: 1, z: 0, cost: 1, meta: META_PARKOUR })
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
      // Upstream's own cardinal parkour also reaches (4,1,0) for cost 1 —
      // the extended move is the one costing dist + 0.5.
      const ext = (w: VoxelWorld): unknown[] =>
        at(movesOf(makeGen(w, FLAG), 0, 1, 0), 4, 1, 0).filter(mv => mv.cost > 2)
      expect(ext(build(2)), 'apex clears the fence').to.have.length(1)
      expect(ext(build(1)), 'fence right after takeoff vetoes').to.have.length(0)
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

    it('long jumps: (3,2) works corner-standing, (4,2) needs a run-up cell', () => {
      // 1x1 standing start: per-axis corner credits cover the short
      // diagonals (players jam these from the block corner).
      const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 6, y1: 7, z1: 6 })
      world.set(0, 0, 0, STONE)
      world.set(3, 0, 1, STONE)
      world.set(3, 0, 2, STONE)
      const standing = movesOf(makeGen(world, FLAG), 0, 1, 0)
      expect(at(standing, 3, 1, 1)[0].cost).to.be.closeTo(Math.hypot(3, 1) + 0.5, 1e-12)
      expect(at(standing, 3, 1, 2)[0].cost).to.be.closeTo(Math.hypot(3, 2) + 0.5, 1e-12)

      // (4,2) still needs running speed — clear corridor, 1x1 takeoff.
      const far = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 6, y1: 7, z1: 6 })
      far.set(0, 0, 0, STONE)
      far.set(4, 0, 2, STONE)
      expect(at(movesOf(makeGen(far, FLAG), 0, 1, 0), 4, 1, 2)).to.have.length(0)

      far.set(-1, 0, 0, STONE) // run-up cell behind the takeoff
      const running = movesOf(makeGen(far, FLAG), 0, 1, 0)
      expect(at(running, 4, 1, 2)[0].cost).to.be.closeTo(Math.hypot(4, 2) + 0.5, 1e-12)
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

      world.set(-1, 1, 0, AIR) // no run-up: standing can't make it at -1 either
      world.set(-1, 0, 0, AIR)
      expect(at(movesOf(makeGen(world, FLAG), 0, 2, 0), 5, 1, 1)).to.have.length(0)
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
})
