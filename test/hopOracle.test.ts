// HopOracle (src/hopOracle.ts): the program search against the flight it
// promises. The search flies its programs as a tree — branches from copies of
// shared run-ups and first air phases — so every program it returns must fly
// the same from scratch (fly(), the reference the executor replays), land on
// its node, and do so from nudged starts; and the headings it aims at a
// landing must come down in the landing's column.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE, SLIME, OAK_FENCE_DRY } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody } from '../src/playerSim.js'
import type { SimWorld } from '../src/playerSim.js'
import { HopOracle, ROBUST_NUDGE, LIVE_NUDGE, LAND_OVERLAP, LOOK_STEP, sneakLanding } from '../src/hopOracle.js'
import type { HopProgram } from '../src/hopOracle.js'
import { vantages, dropTakeoffs, frontierSources, streamablePrefix } from '../src/hopPipeline.js'
import type { RawPathNode } from '../src/types.js'
import type { RawSolveResult } from '../src/solver.js'

const lut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
  lutFingerprint: () => 'oracle', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
  liquids: new Set(), fences: new Set(), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
} as never)
const tables = simTables(lut, mcData.blocksArray as never, s => {
  const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
  return p.waterlogged === true || p.waterlogged === 'true'
})

/** A dry ladder facing north: its plate on the cell's south face, climbed from the north. */
const LADDER_NORTH = ((): number => {
  const b = mcData.blocksByName.ladder
  for (let st = b.minStateId as number; st <= (b.maxStateId as number); st++) {
    const p = (Block.fromStateId(st, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    if (p.facing === 'north' && (p.waterlogged === false || p.waterlogged === 'false')) return st
  }
  throw new Error('no dry north ladder')
})()

/** An open top-half oak trapdoor whose plate stands on the cell's west face: a 3/16 ledge at the cell's top. */
const TRAPDOOR_LEDGE = ((): number => {
  const b = mcData.blocksByName.oak_trapdoor
  for (let st = b.minStateId as number; st <= (b.maxStateId as number); st++) {
    const blk = Block.fromStateId(st, 0) as { getProperties: () => Record<string, unknown>, shapes: number[][] }
    const p = blk.getProperties()
    if (String(p.open) === 'true' && p.half === 'top' && String(p.waterlogged) === 'false' && blk.shapes.length === 1 && blk.shapes[0][3] < 0.2) return st
  }
  throw new Error('no west-plate open trapdoor')
})()

/** The first dry state of block `name` with the given properties. */
function stateOf (name: string, want: Record<string, string>): number {
  const b = mcData.blocksByName[name]
  for (let st = b.minStateId as number; st <= (b.maxStateId as number); st++) {
    const p = (Block.fromStateId(st, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    if (Object.entries(want).every(([k, v]) => String(p[k]) === v)) return st
  }
  throw new Error(`no ${name}`)
}

/** A glass pane joined east and west: a 1/8-thick rail along x through the cell's middle. */
const PANE_EW = stateOf('glass_pane', { east: 'true', west: 'true', north: 'false', south: 'false', waterlogged: 'false' })
/** A closed bottom-half oak trapdoor: a 3/16 slab over the whole cell. */
const TRAPDOOR_LOW = stateOf('oak_trapdoor', { half: 'bottom', open: 'false', waterlogged: 'false', powered: 'false' })

/** An oracle over the stone cells `solid` (air elsewhere inside the box, unloaded outside), plus `extra` cells of given states. */
function oracleOver (solid: Array<[number, number, number]>, grid: boolean, extra: Array<[number, number, number, number]> = []): HopOracle {
  const x0 = -8
  const y0 = -4
  const z0 = -8
  const w = 17
  const h = 12
  const l = 20
  const states = new Int32Array(w * h * l).fill(AIR)
  for (const [x, y, z] of solid) states[((y - y0) * l + (z - z0)) * w + (x - x0)] = STONE
  for (const [x, y, z, st] of extra) states[((y - y0) * l + (z - z0)) * w + (x - x0)] = st
  const stateAt = (x: number, y: number, z: number): number => {
    const lx = x - x0
    const ly = y - y0
    const lz = z - z0
    if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
    return states[(ly * l + lz) * w + lx]
  }
  const world: SimWorld = { ...tables, stateAt, grid: grid ? { states, x0, y0, z0, w, h, l } : undefined }
  return new HopOracle(world, (x, y, z) => { const s = stateAt(x, y, z); return s < 0 ? 0 : lut.flags[s] }, 0.30001, 1.80001)
}

/** A floor of stone at y = 0 over the given x/z ranges (inclusive). */
function floor (xs: [number, number], zs: [number, number]): Array<[number, number, number]> {
  const out: Array<[number, number, number]> = []
  for (let x = xs[0]; x <= xs[1]; x++) for (let z = zs[0]; z <= zs[1]; z++) out.push([x, 0, z])
  return out
}

/** Every program must be the flight it claims: from scratch, the same ticks, on its node, from nudged starts too (verify()). */
function expectFlies (o: HopOracle, p: HopProgram): void {
  expect(o.fly(p), 'from scratch').to.equal(p.ticks)
  expect(o.verify(p), 'robust').to.equal(p.ticks)
  // a single flight, not a chain, holds from nudged starts all the way down
  if (p.legs !== true) {
    const n = p.start !== undefined || p.seq !== undefined ? LIVE_NUDGE : ROBUST_NUDGE
    for (const [nx, nz] of [[n, 0], [-n, 0], [0, n], [0, -n]]) {
      expect(o.fly(p, nx, nz), `nudged ${nx},${nz}`).to.be.greaterThan(0)
    }
  }
}

describe('HopOracle', function () {
  this.timeout(60000)

  // a three-block gap: take-off z ≤ 1, landing z ≥ 5
  const gap = [...floor([-2, 2], [-4, 1]), ...floor([-2, 2], [5, 8])]

  it('finds a jump over a gap that flies the same from scratch, grid or not', () => {
    for (const grid of [true, false]) {
      const o = oracleOver(gap, grid)
      const p = o.hop([0, 1, 0], [0, 1, 5], performance.now() + 20000)
      expect(p, `grid ${String(grid)}`).to.not.equal(null)
      expectFlies(o, p as HopProgram)
      expect((p as HopProgram).tz).to.equal(5)
    }
  })

  it('finds nothing where nothing can land (a nine-block gap)', () => {
    const o = oracleOver([...floor([-2, 2], [-4, 1]), ...floor([-2, 2], [11, 11])], true)
    expect(o.hop([0, 1, 0], [0, 1, 11], performance.now() + 20000, 'all')).to.equal(null)
  })

  it('finds the neo round a pillar in the line', () => {
    // take-off block, a two-high pillar in the line, the landing block behind it
    const neo: Array<[number, number, number]> = [[0, 0, 0], [0, 0, -1], [0, 1, 1], [0, 2, 1], [0, 0, 2]]
    const o = oracleOver(neo, true)
    // (the run it takes is 0.93 rad off the bearing: an air-turn program, not the yaw grid's)
    const p = o.hop([0, 1, 0], [0, 1, 2], performance.now() + 20000, 'all', true)
    expect(p).to.not.equal(null)
    expectFlies(o, p as HopProgram)
  })

  it('finds a live program from a moving body, which flies the same from that exact state', () => {
    const o = oracleOver(gap, true)
    const b = newBody(0.5, 1, -2.5)
    b.vz = 0.18
    b.onGround = true
    const p = o.hopFrom(b, [0, 1, 5], performance.now() + 20000)
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.start).to.not.equal(undefined)
    expect(q.start?.vz).to.equal(0.18)
    expectFlies(o, q)
  })

  it('ranks a live take-off by its landing and the next leg from it, within a tick of the quickest (hopFrom next)', () => {
    const o = oracleOver(gap, true)
    const mk = (): ReturnType<typeof newBody> => {
      const b = newBody(0.5, 1, -2.5)
      b.vz = 0.18
      b.onGround = true
      return b
    }
    const quick = o.hopFrom(mk(), [0, 1, 5], performance.now() + 20000) as HopProgram
    const ranked = o.hopFrom(mk(), [0, 1, 5], performance.now() + 20000, NaN, undefined, [0, 1, 8]) as HopProgram
    expect(ranked).to.not.equal(null)
    expect(ranked.ticks).to.be.at.most(quick.ticks + 1)
    expectFlies(o, ranked)
    // its landing leaves a next leg no slower than the quickest's did
    const ox = o as unknown as { touchdownOf: (p: HopProgram) => ReturnType<typeof newBody> | null, nextLegTicks: (b: ReturnType<typeof newBody>, t: [number, number, number], d: number) => number }
    const legOf = (p: HopProgram): number => { const td = ox.touchdownOf(p); return td === null ? Infinity : ox.nextLegTicks(td, [0, 1, 8], performance.now() + 20000) }
    const lq = legOf(quick)
    const lr = legOf(ranked)
    expect(lr).to.be.at.least(0)
    expect(ranked.ticks + lr).to.be.at.most(quick.ticks + (lq < 0 ? Infinity : lq))
  })

  it('bounces off a slime pad below onto a ladder no plain jump reaches', () => {
    // a ledge at y 8, a slime pad eight below and three out, and a ladder
    // column ten blocks from the ledge: too far for any jump off it, and the
    // rebound off the pad (to y ~6.7: drag takes the rest) carries a body up
    // to its rungs at y 5
    const x0 = -8
    const y0 = -4
    const z0 = -8
    const w = 17
    const h = 20
    const l = 24
    const states = new Int32Array(w * h * l).fill(AIR)
    const put = (x: number, y: number, z: number, s: number): void => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = s }
    for (const [x, , z] of floor([-2, 2], [-2, 1])) put(x, 8, z, STONE)
    for (let x = -1; x <= 1; x++) for (let z = 3; z <= 6; z++) put(x, 0, z, SLIME)
    for (let y = 3; y <= 7; y++) { put(0, y, 10, LADDER_NORTH); put(0, y, 11, STONE) }
    const stateAt = (x: number, y: number, z: number): number => {
      const lx = x - x0
      const ly = y - y0
      const lz = z - z0
      if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
      return states[(ly * l + lz) * w + lx]
    }
    const o = new HopOracle({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } },
      (x, y, z) => { const s = stateAt(x, y, z); return s < 0 ? 0 : lut.flags[s] }, 0.30001, 1.80001)
    expect(o.slimeTops(0, 9, 0).length).to.be.greaterThan(0)
    const p = o.hop([0, 9, 0], [0, 5, 10], performance.now() + 20000, 'all')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.catchTarget).to.equal(true)
    expectFlies(o, q)
    // it rides the rebound: down to the pad's top, then back up to the rungs
    const track: number[] = []
    o.fly(q, 0, 0, track)
    let low = Infinity
    for (let i = 1; i < track.length; i += 3) low = Math.min(low, track[i])
    expect(low).to.be.below(1.05)
    expect(track[track.length - 2]).to.be.at.least(5 - 0.001)
  })

  it('beam: finds the neo too, as controls tick for tick that fly the same from scratch', () => {
    const neo: Array<[number, number, number]> = [[0, 0, 0], [0, 0, -1], [0, 1, 1], [0, 2, 1], [0, 0, 2]]
    const o = oracleOver(neo, true)
    const p = o.hop([0, 1, 0], [0, 1, 2], performance.now() + 30000, 'beam')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.seq).to.not.equal(undefined)
    expect((q.seq as number[]).length).to.equal(2 * q.ticks)
    expectFlies(o, q)
  })

  it('beam: touches down on a stepping block and goes on (a chain no single jump makes)', () => {
    // one-block stepping stones four apart: no jump clears eight, two do
    const o = oracleOver([...floor([-2, 2], [-4, 1]), [0, 0, 5], ...floor([-2, 2], [10, 12])], true)
    expect(o.hop([0, 1, 0], [0, 1, 10], performance.now() + 20000, 'all')).to.equal(null)
    const p = o.hop([0, 1, 0], [0, 1, 10], performance.now() + 30000, 'beam', false, [{ x: 0, y: 1, z: 5 }])
    expect(p).to.not.equal(null)
    expect((p as HopProgram).legs).to.equal(true)
    expect(o.fly(p as HopProgram)).to.equal((p as HopProgram).ticks)
  })

  it("stands on a thin ledge (an open trapdoor's top edge) and lands a jump on it", () => {
    // a two-block gap to a cell whose only footing is the 3/16 plate of an open trapdoor
    const o = oracleOver(floor([-2, 2], [-4, 1]), true, [[0, 0, 4, TRAPDOOR_LEDGE]])
    const st = o.stands(0, 1, 4)
    expect(st.length).to.be.greaterThan(0)
    for (const s of st) expect(s.x, 'over the plate').to.be.below(0.1875 + 0.30001 - LAND_OVERLAP + 1e-9)
    const p = o.hop([0, 1, 0], [0, 1, 4], performance.now() + 20000, 'all')
    expect(p).to.not.equal(null)
    expectFlies(o, p as HopProgram)
  })

  it('stands beside a fence post on its strip, over the strip\'s edge (the cell\'s free corners)', () => {
    // A 1-deep strip with an oak fence post in its middle cell: the grid's
    // every point is inside the post or the fence over it; the body stands
    // beside the post, hanging over the strip's edge (arena opk-1). Fences are
    // not PHYSICAL in the planner's tables (the node is the post's own cell),
    // so this oracle reads them as the plugin's LUT does.
    const fenceLut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
      lutFingerprint: () => 'oracle-fence', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
      liquids: new Set(), fences: new Set([mcData.blocksByName.oak_fence.id]), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
    } as never)
    const x0 = -8; const y0 = -4; const z0 = -8; const w = 17; const hh = 12; const l = 20
    const states = new Int32Array(w * hh * l).fill(AIR)
    const set = (x: number, y: number, z: number, st: number): void => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = st }
    for (const [x, y, z] of floor([-2, 2], [-4, 1])) set(x, y, z, STONE)
    for (let x = -1; x <= 1; x++) set(x, 0, 4, STONE) // the strip
    set(0, 1, 4, OAK_FENCE_DRY) // the post on it
    const stateAt = (x: number, y: number, z: number): number => {
      const lx = x - x0; const ly = y - y0; const lz = z - z0
      if (lx < 0 || lx >= w || ly < 0 || ly >= hh || lz < 0 || lz >= l) return -1
      return states[(ly * l + lz) * w + lx]
    }
    const world: SimWorld = { ...tables, stateAt, grid: { states, x0, y0, z0, w, h: hh, l } }
    const o = new HopOracle(world, (x, y, z) => { const s = stateAt(x, y, z); return s < 0 ? 0 : fenceLut.flags[s] }, 0.30001, 1.80001)
    const st = o.stands(0, 1, 4)
    expect(st.length, 'stands beside the post').to.be.greaterThan(0)
    // (the post spans x 0.375..0.625: a body beside it has its centre within 0.075 of a face)
    for (const s of st) expect(s.x < 0.0751 || s.x > 0.9249, 'clear of the post').to.equal(true)
  })

  it('files a pane rail\'s node as three places: its top and the floor either side, no straight walk between them', () => {
    // a 1-wide stone beam along x with a pane rail on it (arena mcc-6-1's
    // hoops): the planner's node over the rail is its top; beside the rail,
    // on the beam, the body rests a block lower in the same column
    const beam: Array<[number, number, number, number]> = []
    for (let x = -3; x <= 3; x++) beam.push([x, 1, 0, PANE_EW])
    const o = oracleOver([[-3, 0, 0], [-2, 0, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [0, 0, 4]], true, [...beam, [0, 1, 4, TRAPDOOR_LOW]])
    const regs = o.regions(0, 2, 0)
    expect(regs.map(r => r.under), 'the top, then the two sides').to.deep.equal([false, true, true])
    const north = regs[1].stands[0].z < 0.5 ? regs[1] : regs[2]
    const south = north === regs[1] ? regs[2] : regs[1]
    for (const st of north.stands) expect(st.z < 0.4375 - 0.3 + 1e-3 && st.y === 1, 'north of the rail, on the beam').to.equal(true)
    for (const st of south.stands) expect(st.z > 0.5625 + 0.3 - 1e-3 && st.y === 1, 'south of the rail, on the beam').to.equal(true)
    const n = north.stands[0]
    expect(o.besideRest([0, 2, 0], n), 'beside the rail').to.equal(true)
    expect(o.besideRest([0, 2, 0], regs[0].stands[0]), 'on its top').to.equal(false)
    expect(o.walkLink(n, south.stands[0].x, south.stands[0].z), 'through the pane').to.equal(false)
    const own = o.regionOf([0, 2, 0], n)
    expect(own.some(st => st.z > 0.5), 'the north rest\'s place holds nothing south of the rail').to.equal(false)
    // a bottom trapdoor's top is the planner's own place: no under place
    expect(o.underStands(0, 2, 4)).to.deep.equal([])
    expect(o.regions(0, 2, 4).map(r => r.under)).to.deep.equal([false])
    expect(o.besideRest([0, 2, 4], o.stands(0, 2, 4)[0])).to.equal(false)
  })

  it('hops from beside a rail onto its top and over it, each held to its landing place (HopProgram.land)', () => {
    const beam: Array<[number, number, number, number]> = []
    for (let x = -3; x <= 3; x++) beam.push([x, 1, 0, PANE_EW])
    const o = oracleOver([[-3, 0, 0], [-2, 0, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], true, beam)
    const regs = o.regions(0, 2, 0)
    const n = regs.find(r => r.under && r.stands[0].z < 0.5)!.stands[0]
    // only the rest's own node in range: its other two places
    const edges = o.discoverFromRest({ x: 0, y: 2, z: 0 }, n, (x, y, z) => (x === 0 && y === 2 && z === 0 ? 0 : 50), 5000, null)
    const places = new Set<string>()
    for (const e of edges) {
      const p = e.program
      expect(p.land, 'held to a place').to.not.equal(undefined)
      expectFlies(o, p)
      const rest = o.restOf(p)
      expect(rest, 'comes to rest').to.not.equal(null)
      expect(Math.abs(rest!.y - p.land!.y), 'at its place\'s level').to.be.lessThan(1e-3)
      expect(o.regionOf([p.tx, p.ty, p.tz], p.land!).some(st => o.walkLink(st, rest!.x, rest!.z)), 'in its place').to.equal(true)
      places.add(o.besideRest([0, 2, 0], rest!) ? (rest!.z > 0.5 ? 'south' : 'north') : 'top')
    }
    expect([...places].sort(), 'onto the top and over the rail').to.deep.equal(['south', 'top'])
  })

  it('offers the ledges over slime the search reached as drop take-offs, nearest the slime first', () => {
    // a stalled tail on a high platform; past a wall the platform goes on to a
    // ledge over a pit with slime at its bottom — reached, off the tail
    const solid: Array<[number, number, number]> = [...floor([-6, -2], [-6, 2])]
    for (let z = -6; z <= 2; z++) solid.push([-1, 1, z], [-1, 2, z]) // a wall the tail stands behind
    for (const [x, z] of [[-6, 3], [-5, 3], [-4, 3], [-3, 3], [-2, 3], [-1, 3], [0, 3]]) solid.push([x, 0, z]) // the way round, reached
    const o = oracleOver(solid, true, [[2, -3, 4, SLIME], [3, -3, 4, SLIME]])
    const tail = [{ x: -3, y: 1, z: 0 }, { x: -2, y: 1, z: 0 }]
    const reached = (x: number, y: number, z: number): boolean => y === 1 && ((x >= -6 && x <= -2 && z >= -6 && z <= 2) || (z === 3 && x >= -6 && x <= 0))
    const drops = dropTakeoffs(o, reached, tail)
    expect(drops.length, 'a ledge over the slime').to.be.greaterThan(0)
    expect(drops[0], 'the end of the way round, beside the pit').to.deep.equal({ x: 0, y: 1, z: 3 })
    // none of the tail's own cells, and none without a drop beside it
    for (const d of drops) expect(tail.some(t => t.x === d.x && t.z === d.z)).to.equal(false)
  })

  it('offers reached take-offs off the tail that look out on unreached ground as frontier sources', () => {
    // reached: a floor (stand y 1) at x -6..0, z -6..0; the tail ran along its
    // far edge; a platform 3 blocks off its near corner is unreached
    const solid = [...floor([-6, 0], [-6, 0]), ...floor([3, 6], [3, 6])]
    const o = oracleOver(solid, true)
    const h = (x: number, y: number, z: number): number => Math.hypot(x - 5, y - 1, z - 5)
    const tail = [{ x: -6, y: 1, z: -6 }, { x: -5, y: 1, z: -6 }]
    const onA = (x: number, y: number, z: number): boolean => y === 1 && x >= -6 && x <= 0 && z >= -6 && z <= 0
    const front = frontierSources(o, onA, h, tail)
    expect(front[0], 'the corner nearest the platform').to.deep.equal({ x: 0, y: 1, z: 0 })
    // apart from the tail and from each other
    for (const f of front) expect(tail.some(t => Math.max(Math.abs(t.x - f.x), Math.abs(t.z - f.z)) <= 1)).to.equal(false)
    for (let i = 0; i < front.length; i++) for (let j = i + 1; j < front.length; j++) expect(Math.max(Math.abs(front[i].x - front[j].x), Math.abs(front[i].z - front[j].z))).to.be.greaterThan(1)
    // with the platform reached too, nothing looks out on new ground
    const onAB = (x: number, y: number, z: number): boolean => onA(x, y, z) || (y === 1 && x >= 3 && x <= 6 && z >= 3 && z <= 6)
    expect(frontierSources(o, onAB, h, tail)).to.deep.equal([])
  })

  it('streams the failed search\'s partial only up to the node nearest an off-path take-off', () => {
    // the drop-take-off scene: the partial runs along the platform behind
    // the wall; the ledge over the slime pit (0,1,3) is the pipeline's take-off
    const solid: Array<[number, number, number]> = [...floor([-6, -2], [-6, 2])]
    for (let z = -6; z <= 2; z++) solid.push([-1, 1, z], [-1, 2, z])
    for (const [x, z] of [[-6, 3], [-5, 3], [-4, 3], [-3, 3], [-2, 3], [-1, 3], [0, 3]]) solid.push([x, 0, z])
    const o = oracleOver(solid, true, [[2, -3, 4, SLIME], [3, -3, 4, SLIME]])
    const reached = (x: number, y: number, z: number): boolean => y === 1 && ((x >= -6 && x <= -2 && z >= -6 && z <= 2) || (z === 3 && x >= -6 && x <= 0))
    const node = (x: number, z: number): RawPathNode => ({ x, y: 1, z, cost: 1, parkour: false, useOne: null })
    const first = { status: 'noPath', path: [node(-5, 0), node(-4, 0), node(-3, 0), node(-2, 0)] } as unknown as RawSolveResult
    const prefix = streamablePrefix(first, { x: -6, y: 1, z: 0 }, o, reached, null)
    // every cell of the z=3 row is a drop edge: the partial node nearest any
    // of them, (-3,1,0) to (-3,1,3), and all after it stay for the pipeline
    expect(prefix.map(n => n.x)).to.deep.equal([-5, -4])
    // no off-path take-off anywhere: the whole partial
    const bare = oracleOver(solid, true)
    expect(streamablePrefix(first, { x: -6, y: 1, z: 0 }, bare, (x, y, z) => y === 1 && x >= -6 && x <= -2 && z >= -6 && z <= 2, null)).to.have.length(4)
  })

  it('sneaks onto a slime top on the touchdown tick, and on no other', () => {
    const p = { slimeLand: true, tx: 0, ty: 5, tz: 0 } as HopProgram
    const hw = 0.30001
    // in the air, coming down through the top this tick, the box over the column
    expect(sneakLanding(p, false, 0.5, 5.3, 0.5, -0.4, hw)).to.equal(true)
    expect(sneakLanding(p, false, 1.25, 5.3, 0.5, -0.4, hw), 'box over the column, centre past it').to.equal(true)
    expect(sneakLanding(p, false, 0.5, 5.6, 0.5, -0.4, hw), 'a tick early').to.equal(false)
    expect(sneakLanding(p, false, 0.5, 5.3, 0.5, 0.4, hw), 'rising').to.equal(false)
    expect(sneakLanding(p, true, 0.5, 5, 0.5, -0.0784, hw), 'standing').to.equal(false)
    expect(sneakLanding(p, false, 1.4, 5.3, 0.5, -0.4, hw), 'beside it').to.equal(false)
    expect(sneakLanding({ ...p, slimeLand: false }, false, 0.5, 5.3, 0.5, -0.4, hw), 'no slime landing').to.equal(false)
  })

  it('lands on a slime top, sneaking on the touchdown tick (let go, it throws the body back up)', () => {
    const o = oracleOver(floor([-2, 2], [-4, 1]), true, [[0, 0, 4, SLIME]])
    const p = o.hop([0, 1, 0], [0, 1, 4], performance.now() + 20000, 'all')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.slimeLand).to.equal(true)
    expectFlies(o, q)
    expect(o.fly({ ...q, slimeLand: false }), 'without the sneak').to.equal(-1)
  })

  it('takes off from a ladder, hanging on it, onto a ledge below and out from the wall', () => {
    // a ladder column against a wall, and a ledge three below and three out
    const wall: Array<[number, number, number]> = []
    const ladder: Array<[number, number, number, number]> = []
    for (let y = 1; y <= 6; y++) wall.push([0, y, 1])
    for (let y = 1; y <= 5; y++) ladder.push([0, y, 0, LADDER_NORTH])
    const o = oracleOver([...wall, ...floor([-1, 1], [-4, -2])], true, ladder)
    expect(o.stands(0, 4, 0)[0].catch).to.equal(true)
    const p = o.hop([0, 4, 0], [0, 1, -3], performance.now() + 20000, 'all')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.climb).to.equal(true)
    expectFlies(o, q)
  })

  it('flies a deep bounce off a 40-block drop, gated on its pool, and re-checks it live (deepVerdict)', () => {
    // a ledge 40 over a 9x9 slime pool and a landing 12 up beyond it: only
    // the rebound off the pool carries there (HopProgram.deep; at 10 up the
    // fall off the rebound's apex is a landing that hurts, FALL_VETO)
    const x0 = -8
    const y0 = -4
    const z0 = -8
    const w = 17
    const h = 52
    const l = 30
    const states = new Int32Array(w * h * l).fill(AIR)
    const put = (x: number, y: number, z: number, s: number): void => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = s }
    for (const [x, , z] of floor([-2, 2], [-2, 1])) put(x, 40, z, STONE)
    for (let x = -4; x <= 4; x++) for (let z = 4; z <= 12; z++) put(x, 0, z, SLIME)
    for (const [x, , z] of floor([-2, 2], [14, 16])) put(x, 12, z, STONE)
    const stateAt = (x: number, y: number, z: number): number => {
      const lx = x - x0
      const ly = y - y0
      const lz = z - z0
      if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
      return states[(ly * l + lz) * w + lx]
    }
    const o = new HopOracle({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } },
      (x, y, z) => { const s = stateAt(x, y, z); return s < 0 ? 0 : lut.flags[s] }, 0.30001, 1.80001)
    expect(o.slimeTops(0, 41, 0), 'too deep for the shallow scan').to.deep.equal([])
    const p = o.hop([0, 41, 0], [0, 13, 15], performance.now() + 30000, 'bounce')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.deep).to.equal(true)
    expectFlies(o, q)
    // the abort's targets: the pool's inside, a block and a half in, never its rim
    expect(q.basin?.length).to.equal(36)
    for (const [x, , z] of q.basin ?? []) expect(Math.abs(x) < 4 && z > 4 && z < 12, `inside: ${x},${z}`).to.equal(true)
    // live: on plan from its start it arrives; a body falling over the pool's
    // middle still aborts into it, one beside the pool can do neither
    expect(o.deepVerdict(q, newBody(q.sx, q.sy, q.sz), 0, false, 0, 0, NaN).arrives).to.equal(true)
    const over = newBody(0.5, 25, 8.5)
    over.vy = -1
    expect(o.deepVerdict(q, over, 40, true, 30, 0, NaN).abortable).to.equal(true)
    const beside = newBody(7.5, 25, 8.5)
    beside.vy = -1
    expect(o.deepVerdict(q, beside, 40, true, 30, 0, NaN)).to.deep.equal({ arrives: false, abortable: false })
    // and nothing lands on the pool itself: sneaking onto slime takes the whole 40-block fall (FALL_VETO)
    expect(o.hop([0, 41, 0], [0, 1, 8], performance.now() + 30000, 'simple')).to.equal(null)
  })

  it('bounces a chain of slime pads to a landing far past the first', () => {
    // a ledge at y 8, a pad eight below and four out, a second pad six on,
    // and a landing two up and four on from that: ten from the first pad
    const x0 = -8
    const y0 = -4
    const z0 = -8
    const w = 17
    const h = 22
    const l = 32
    const states = new Int32Array(w * h * l).fill(AIR)
    const put = (x: number, y: number, z: number, s: number): void => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = s }
    for (const [x, , z] of floor([-2, 2], [-2, 1])) put(x, 8, z, STONE)
    for (let x = -1; x <= 1; x++) for (let z = 3; z <= 5; z++) put(x, 0, z, SLIME)
    for (let x = -1; x <= 1; x++) for (let z = 9; z <= 11; z++) put(x, 0, z, SLIME)
    for (const [x, , z] of floor([-2, 2], [13, 15])) put(x, 2, z, STONE)
    const stateAt = (x: number, y: number, z: number): number => {
      const lx = x - x0
      const ly = y - y0
      const lz = z - z0
      if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
      return states[(ly * l + lz) * w + lx]
    }
    const o = new HopOracle({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } },
      (x, y, z) => { const s = stateAt(x, y, z); return s < 0 ? 0 : lut.flags[s] }, 0.30001, 1.80001)
    const p = o.hop([0, 9, 0], [0, 3, 14], performance.now() + 30000, 'simple')
    expect(p).to.not.equal(null)
    const q = p as HopProgram
    expect(q.bounces?.length).to.equal(1)
    expect(o.fly(q), 'from scratch').to.equal(q.ticks)
    expect(o.verify(q), 'robust to its first rebound').to.equal(q.ticks)
    // two rebounds on the way: the track comes down to the pads' top twice
    const track: number[] = []
    o.fly(q, 0, 0, track)
    let downs = 0
    for (let i = 4; i + 3 < track.length; i += 3) if (track[i] < 1.05 && track[i] < track[i - 3] && track[i] <= track[i + 3]) downs++
    expect(downs).to.equal(2)
  })

  it('offers the highest reached ground near the goal as vantage take-offs, above the stalled tail', () => {
    // reached: a floor at y 1 under the goal, and a ladder column (x -5) up to y 9
    const reached = (x: number, y: number, z: number): boolean =>
      (y === 1 && Math.abs(x) <= 3 && Math.abs(z) <= 3) || (x === -5 && z === 0 && y >= 1 && y <= 9)
    const goal = { x: 0, y: 6, z: 0 }
    const tail = { x: 0, y: 1, z: 0 }
    const v = vantages(reached, goal, tail, [tail])
    expect(v).to.deep.equal([{ x: -5, y: 9, z: 0 }])
    // nothing higher than the tail by VANTAGE_ABOVE: none
    expect(vantages((x, y, z) => y === 1, goal, tail, [tail])).to.deep.equal([])
  })

  it('re-anchors a program at a body a hair off its start, and refuses one too far off', () => {
    const o = oracleOver(gap, true)
    const p = o.hop([0, 1, 0], [0, 1, 5], performance.now() + 20000) as HopProgram
    const near = newBody(p.sx + 0.004, p.sy, p.sz - 0.004)
    near.onGround = true
    near.vy = -0.0784
    const q = o.anchored(p, near)
    expect(q).to.not.equal(null)
    expect((q as HopProgram).start).to.not.equal(undefined)
    const far = newBody(p.sx, p.sy, p.sz - 2.2)
    far.onGround = true
    far.vy = -0.0784
    expect(o.anchored(p, far)).to.equal(null)
  })

  it('places a body at rest onto a precision program\'s start by one or two sneaking presses (the exact line-up)', () => {
    const o = oracleOver(gap, true)
    const p = o.hop([0, 1, 0], [0, 1, 5], performance.now() + 20000) as HopProgram
    expect(p).to.not.equal(null)
    for (const [ox, oz] of [[0.05, 0], [-0.04, 0.03], [0.07, -0.05]]) {
      const b = newBody(p.sx + ox, p.sy, p.sz + oz)
      b.onGround = true
      b.vy = -0.0784
      const yaw0 = 0.3
      const lu = o.lineUp(p, b, yaw0, performance.now() + 5000)
      expect(lu, `from ${ox},${oz}`).to.not.equal(null)
      const { yaws, rest, prog } = lu as NonNullable<typeof lu>
      expect(yaws.length).to.be.within(1, 2)
      // each press turns in whole LOOK_STEPs, as bot.look does
      for (const y of yaws) {
        const m = (y - yaw0) / LOOK_STEP
        expect(Math.abs(m - Math.round(m)), 'whole look steps').to.be.lessThan(1e-6)
      }
      expect(Math.hypot(rest.x - p.sx, rest.z - p.sz), 'rest on the start').to.be.lessThan(0.01)
      expect(rest.y).to.equal(p.sy)
      expect(prog.start, 'a live program from that rest').to.not.equal(undefined)
      expect(o.fly(prog)).to.equal(prog.ticks)
    }
    // two presses move a body ~0.12 at most: a half-block miss is no line-up
    const far = newBody(p.sx + 0.5, p.sy, p.sz)
    far.onGround = true
    far.vy = -0.0784
    expect(o.lineUp(p, far, 0, performance.now() + 5000)).to.equal(null)
  })

  it('starts a carry chain at the last firm node before a path\'s icy end', () => {
    const ICE = mcData.blocksByName.ice.minStateId as number
    const ice: Array<[number, number, number, number]> = [[2, 0, 0, ICE], [3, 0, 0, ICE], [4, 0, 0, ICE]]
    const o = oracleOver([[0, 0, 0], [1, 0, 0], [5, 0, 0]], true, ice)
    const n = (x: number): { x: number, y: number, z: number } => ({ x, y: 1, z: 0 })
    expect(o.chainStart([n(0), n(1), n(2), n(3)]), 'ends on ice').to.equal(1)
    expect(o.chainStart([n(0), n(1), n(2), n(3), n(4)])).to.equal(1)
    expect(o.chainStart([n(1), n(2), n(5)]), 'ends on stone').to.equal(-1)
    expect(o.chainStart([n(2), n(3), n(4)]), 'no firm node behind').to.equal(-1)
  })

  it('aims headings that bring an unobstructed flight down in the landing column', () => {
    const o = oracleOver(gap, true) as unknown as {
      aimHeadings: (b: unknown, sprint: boolean, tx: number, tz: number, landY: number, out: Array<{ x: number, z: number, fixed: boolean }>) => void
      ballisticLanding: (b: unknown, h: unknown, sprint: boolean, landY: number) => [number, number] | null
    }
    let aimed = 0
    for (let k = 0; k < 400; k++) {
      // a body in the air anywhere round the gap, rising or falling
      const b = newBody(-1 + (k % 7) * 0.3, 1.2 + (k % 5) * 0.3, -0.5 + (k % 11) * 0.35)
      b.vx = ((k % 13) - 6) * 0.03
      b.vy = ((k % 9) - 3) * 0.1
      b.vz = ((k % 17) - 4) * 0.05
      const out: Array<{ x: number, z: number, fixed: boolean }> = []
      o.aimHeadings(b, k % 2 === 0, 0, 5, 1, out)
      for (const h of out) {
        const land = o.ballisticLanding(b, h, k % 2 === 0, 1)
        expect(land, `flight ${k}`).to.not.equal(null)
        // supported by the landing block: the box over its column by LAND_OVERLAP at least
        const [lx, lz] = land as [number, number]
        expect(Math.abs(lx - 0.5), `flight ${k} x`).to.be.below(0.5 + 0.30001 - LAND_OVERLAP)
        expect(Math.abs(lz - 5.5), `flight ${k} z`).to.be.below(0.5 + 0.30001 - LAND_OVERLAP)
        aimed++
      }
    }
    expect(aimed).to.be.greaterThan(50)
  })

  it('never rules out a column a flight can come down in (the reach test)', () => {
    // open air over a floor far below, and walls to hit: flights on headings
    // changed every few ticks, each column passed at or above a level asked
    const o = oracleOver([...floor([-8, 8], [-8, 11]).map(([x, , z]): [number, number, number] => [x, -4, z]),
      [2, 1, 2], [2, 2, 2], [-3, 0, 4], [-3, 1, 4]], true) as unknown as {
      canReach: (b: unknown, sprint: boolean, tx: number, ty: number, tz: number) => boolean
      sim: { step: (b: unknown, c: unknown) => void }
    }
    let seed = 12345
    const rnd = (): number => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff }
    let asked = 0
    for (let k = 0; k < 300; k++) {
      const sprint = k % 3 !== 0
      const b0 = newBody(-1 + rnd() * 3, 1 + rnd() * 2, rnd() * 3)
      b0.vx = (rnd() - 0.5) * 0.6
      b0.vy = rnd() * 0.42 - 0.1
      b0.vz = (rnd() - 0.5) * 0.6
      const ty = Math.floor(b0.y) - (k % 4)
      const b = { ...b0 }
      const c = { forward: true, back: false, left: false, right: false, jump: false, sprint, sneak: false, hx: 0, hz: 1 }
      for (let t = 0; t < 60 && !b.onGround && b.y >= ty; t++) {
        if (t % 3 === 0) { const a = rnd() * 2 * Math.PI; c.hx = Math.cos(a); c.hz = Math.sin(a) }
        o.sim.step(b, c)
        if (b.y < ty) break
        expect(o.canReach(b0, sprint, Math.floor(b.x), ty, Math.floor(b.z)), `flight ${k} tick ${t}`).to.equal(true)
        asked++
      }
    }
    expect(asked).to.be.greaterThan(1500)
    // and it does rule out: a column nine blocks off a standing jump
    const up = newBody(0.5, 1, 0.5)
    up.vy = 0.42
    expect(o.canReach(up, true, 0, 1, 9)).to.equal(false)
  })
})
