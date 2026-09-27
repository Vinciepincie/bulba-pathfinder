// HopOracle (src/hopOracle.ts): the program search against the flight it
// promises. The search flies its programs as a tree — branches from copies of
// shared run-ups and first air phases — so every program it returns must fly
// the same from scratch (fly(), the reference the executor replays), land on
// its node, and do so from nudged starts; and the headings it aims at a
// landing must come down in the landing's column.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE, SLIME } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody } from '../src/playerSim.js'
import type { SimWorld } from '../src/playerSim.js'
import { HopOracle, ROBUST_NUDGE, LIVE_NUDGE, LAND_OVERLAP, sneakLanding } from '../src/hopOracle.js'
import type { HopProgram } from '../src/hopOracle.js'
import { vantages } from '../src/hopPipeline.js'

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
})
