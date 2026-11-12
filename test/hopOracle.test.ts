// HopOracle (src/hopOracle.ts): the program search against the flight it
// promises. The search flies its programs as a tree — branches from copies of
// shared run-ups and first air phases — so every program it returns must fly
// the same from scratch (fly(), the reference the executor replays), land on
// its node, and do so from nudged starts; and the headings it aims at a
// landing must come down in the landing's column.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE, SLIME, OAK_FENCE_DRY, SOUL_SAND } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody, SIM, isVanillaJump, setVanillaJump } from '../src/playerSim.js'
import type { PlayerSim, SimBody, SimControl, SimWorld } from '../src/playerSim.js'
import { HopOracle, ROBUST_NUDGE, LIVE_NUDGE, LAND_OVERLAP, LOOK_STEP, F_JUMP, sneakLanding, LANE_KICK_TICKS, LIVE_PREFIX, LIVE_PREFIX_TICKS, BONK_FIRE, jumpRearmed, F_FWD, F_SPRINT, REGION_SETTLE } from '../src/hopOracle.js'
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
function expectFlies (o: HopOracle, p: HopProgram, offsets?: ReadonlyArray<readonly [number, number]>): void {
  expect(o.fly(p), 'from scratch').to.equal(p.ticks)
  expect(o.verify(p), 'robust').to.equal(p.ticks)
  // a single flight, not a chain, holds from nudged starts all the way down
  if (p.legs !== true) {
    const n = p.start !== undefined || p.seq !== undefined ? LIVE_NUDGE : ROBUST_NUDGE
    for (const [nx, nz] of offsets ?? [[n, 0], [-n, 0], [0, n], [0, -n]]) {
      expect(o.fly(p, nx, nz), `nudged ${nx},${nz}`).to.be.greaterThan(0)
    }
  }
}

describe('HopOracle', function () {
  this.timeout(60000)

  // a three-block gap: take-off z ≤ 1, landing z ≥ 5
  const gap = [...floor([-2, 2], [-4, 1]), ...floor([-2, 2], [5, 8])]

  it('refuses a landing that costs more than fallVeto allows', () => {
    // From feet at 6 to feet at 1: at least 5 blocks of fall, 2 points of damage or more.
    const ledge = floor([-2, 2], [-4, 1]).map(([x, , z]): [number, number, number] => [x, 5, z])
    const solid = [...ledge, ...floor([-2, 2], [5, 8])]
    const free = oracleOver(solid, true)
    expect(free.hop([0, 6, 0], [0, 1, 6], performance.now() + 20000)).to.not.equal(null)
    const capped = oracleOver(solid, true)
    capped.fallVeto = 1
    expect(capped.hop([0, 6, 0], [0, 1, 6], performance.now() + 20000)).to.equal(null)
  })

  describe('bonked take-offs', () => {
    type Flight = { b: SimBody, t: number, fired: boolean, air: number, rebounds: number, yaw: number, leg: number, top: number }
    type FlightOracle = { ctl: SimControl, tickFlight: (f: Flight, p: HopProgram, track: number[] | null) => number }
    const roof = floor([-2, 2], [-4, 0]).map(([x, , z]): [number, number, number] => [x, 3, z])
    const solid = [...floor([-2, 2], [-4, 0]), ...floor([-2, 2], [3, 8]), ...roof]
    const body = (): SimBody => Object.assign(newBody(0.5, 1, 0.5), { onGround: true, vy: -0.08 * SIM.airdrag })
    const flight = (b: SimBody): Flight => ({ b: { ...b }, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: b.y })
    const program = (b: SimBody): HopProgram => {
      const h = { fixed: true, x: 0, z: 1 }
      return { sx: b.x, sy: b.y, sz: b.z, start: b, run: h, jumpAt: 0, air1: h, turnAir: Infinity,
        air2: h, sprint: true, ticks: 0, tx: 0, ty: 1, tz: 4, family: 'straight' }
    }
    function tick (o: HopOracle, f: Flight, p: HopProgram, jump: boolean): number {
      const i = o as unknown as FlightOracle
      Object.assign(i.ctl, { hx: 0, hz: 1, forward: false, back: false, left: false, right: false, sprint: false, sneak: false, jump })
      return i.tickFlight(f, p, null)
    }

    it('recognises a re-arm, not an inherited cooldown or a decrement', () => {
      expect(jumpRearmed(0, 10)).to.equal(BONK_FIRE)
      expect(jumpRearmed(1, 10)).to.equal(BONK_FIRE)
      expect(jumpRearmed(10, 10)).to.equal(false)
      expect(jumpRearmed(11, 10)).to.equal(false)
      expect(jumpRearmed(2, 1)).to.equal(false)
      expect(jumpRearmed(undefined, 10)).to.equal(false)
      expect(jumpRearmed(0, undefined)).to.equal(false)
    })

    it('recognises the impulse after a bonk and honours the held-jump cooldown', () => {
      const o = oracleOver(solid, true)
      const b = body()
      b.jumpTicks = 2
      const p = program(b)
      const f = flight(b)
      tick(o, f, p, true)
      expect(f.b.jumpTicks).to.equal(1)
      expect(f.fired).to.equal(false)
      tick(o, f, p, true)
      expect(f.b.jumpTicks).to.equal(SIM.autojumpCooldown)
      expect(f.b.y).to.be.closeTo(3 - 1.80001, 1e-12)
      expect(f.b.vy).to.equal(-SIM.gravity * SIM.airdrag)
      expect(f.b.onGround).to.equal(false)
      expect(f.fired).to.equal(BONK_FIRE)
      expect(f.air).to.equal(0)
      tick(o, f, p, false)
      expect(f.b.jumpTicks).to.equal(0)
      expect(f.b.onGround).to.equal(false)
      tick(o, f, p, false)
      expect(f.b.onGround).to.equal(true)
      expect(f.b.y).to.equal(1)
    })

    it('finds and replays a live bonk over a two-wide gap, grid or not', function () {
      if (!BONK_FIRE) this.skip()
      for (const grid of [true, false]) {
        const o = oracleOver(solid, grid)
        // climb2's low roof, with synthetic carried speed at the lip.
        const b = Object.assign(body(), { z: 1.18, vz: 0.93 })
        const p = o.hopFrom(b, [0, 1, 4], performance.now() + 20000)
        expect(p).to.not.equal(null)
        const q = p as HopProgram
        expect(q.jumpAt).to.equal(0)
        expectFlies(o, q)
        const seq: HopProgram = { ...program(b), family: 'beam', legs: true,
          seq: [Math.PI, F_FWD | F_SPRINT | F_JUMP, Math.PI, F_FWD | F_SPRINT] }
        seq.ticks = o.fly(seq)
        expect(seq.ticks).to.be.greaterThan(0)
        expectFlies(o, seq)
      }
    })

    it('restores the missed bonk with PF_BONK_FIRE=0', () => {
      const o = oracleOver(solid, true)
      const p = program(Object.assign(body(), { z: 1.18, vz: 0.93 }))
      expect(o.fly(p)).to.equal(BONK_FIRE ? 3 : -1)
    })

    it('keeps the open-sky fired tick and air counter identical to the velocity rule', () => {
      const o = oracleOver(floor([-2, 2], [-4, 8]), true)
      const b = body()
      const p = { ...program(b), jumpAt: 2 }
      const f = flight(b)
      let oldFired = false
      let oldAir = 0
      for (let t = 0; t < 5; t++) {
        tick(o, f, p, t >= p.jumpAt && !f.fired)
        if (!oldFired) oldFired = f.b.vy > 0.3 || (!f.b.onGround && f.b.vy < -0.3)
        else oldAir++
        expect(f.fired).to.equal(oldFired)
        expect(f.air).to.equal(oldAir)
        if (t === p.jumpAt) {
          expect(f.fired).to.equal(true)
          expect(f.t).to.equal(3)
          expect(f.air).to.equal(0)
        }
      }
    })

    it('registers honey impulses and preserves both slime rebound jump rules', () => {
      const honey = stateOf('honey_block', {})
      const h = oracleOver([], true, [[0, 0, 0, honey], [0, -1, 0, honey]])
      const b = Object.assign(body(), { y: 15 / 16 })
      const f = flight(b)
      tick(h, f, program(b), true)
      expect(f.b.vy).to.be.closeTo((SIM.jumpVelocity * 0.4 - SIM.gravity) * SIM.airdrag, 1e-12)
      expect(f.b.jumpTicks).to.equal(SIM.autojumpCooldown)
      expect(f.fired).to.equal(BONK_FIRE)
      const saved = isVanillaJump()
      try {
        for (const vanilla of [false, true]) {
          setVanillaJump(vanilla)
          const o = oracleOver([], true, [[0, 0, 0, SLIME]])
          const b = Object.assign(body(), { vy: 0.9 })
          const f = flight(b)
          tick(o, f, program(b), true)
          expect(f.b.vy).to.be.closeTo(((vanilla ? 0.9 : SIM.jumpVelocity) - SIM.gravity) * SIM.airdrag, 1e-12)
          expect(f.b.jumpTicks).to.equal(SIM.autojumpCooldown)
          expect(f.fired).to.equal(true)
          expect(f.air).to.equal(0)
        }
      } finally { setVanillaJump(saved) }
    })

    it('ends a touch-and-go flight pinned under a roof with jump held (FLIGHT_CAP)', function () {
      if (process.env.PF_FLIGHT_CAP === '0') this.skip()
      this.timeout(10000)
      const o = oracleOver(solid, true)
      const b = body()
      // (jumping on the spot under the roof: every bonk re-fires, every landing starts a leg - no leg limit is ever hit)
      const seq: HopProgram = { ...program(b), tz: 6, family: 'beam', legs: true, seq: [NaN, F_JUMP] }
      expect(o.fly(seq)).to.equal(-1)
    })

    it('re-arms each touch-and-go leg after a bonked touchdown', function () {
      if (!BONK_FIRE) this.skip()
      const o = oracleOver(solid, true)
      const b = body()
      const p = { ...program(b), legs: true }
      const f = flight(b)
      tick(o, f, p, true)
      tick(o, f, p, false)
      tick(o, f, p, false)
      expect(f.leg).to.equal(3)
      expect(f.fired).to.equal(false)
      tick(o, f, p, true)
      expect(f.fired).to.equal(true)
      expect(f.air).to.equal(0)
      expect(f.leg).to.equal(3)
    })
  })

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

  describe('live ground prefixes', () => {
    type PrefixOracle = {
      sim: PlayerSim
      requiredNudges: (p: HopProgram, n: number) => Array<[number, number]>
      prefixBudget: { end: number, deadline: number } | null
    }
    const inspect = (o: HopOracle): PrefixOracle => o as unknown as PrefixOracle

    function fixture (thin = false, grid = true): { o: HopOracle, b: SimBody, target: [number, number, number] } {
      // mcc-6-2: the three-block face and its return obstruct the flush launch.
      const solid: Array<[number, number, number]> = [[3, 0, -2]]
      for (const [x, z] of [[-1, -1], [0, -1], [1, -1], [1, -2]]) {
        for (let y = 0; y <= 4; y++) solid.push([x, y, z])
      }
      if (!thin) solid.push([0, 0, 0])
      const o = thin
        ? oracleOver(solid.map(([x, y, z]): [number, number, number] => [z, y, x]), grid, [[0, 0, 0, TRAPDOOR_LEDGE]])
        : oracleOver(solid, grid)
      const b = thin ? newBody(3 / 16 + o.halfWidth - 0.012, 1, 0.26) : newBody(0.26, 1, o.halfWidth)
      b.onGround = true
      b.vy = -0.0784
      b.collidedH = true
      b.collidedV = true
      b.jumpTicks = 3
      return { o, b, target: thin ? [-2, 1, 3] : [3, 1, -2] }
    }

    function counted<T> (o: HopOracle, run: () => T, after?: (b: SimBody, c: SimControl) => void): { value: T, steps: number } {
      const sim = inspect(o).sim
      const step = sim.step
      let steps = 0
      sim.step = (b, c): void => { steps++; step.call(sim, b, c); after?.(b, c) }
      try { return { value: run(), steps } } finally { sim.step = step }
    }

    it('retreats from a pillar face and flies the retained original-start twins', function () {
      if (!LIVE_PREFIX || process.env.PF_LIVE_NUDGE_SAFE === '0') this.skip()
      const yaw0 = 0.3
      const n = LIVE_NUDGE
      const retained: Array<[number, number]> = [[n, 0], [-n, 0], [0, n]]
      for (const grid of [true, false]) {
        const plain = fixture(false, grid)
        plain.o.tickLimit = 200_000
        expect(plain.o.hopFrom(plain.b, plain.target, Infinity, yaw0,
          undefined, undefined, true, ['kick'], 150_000, false), 'ordinary rest search').to.equal(null)
        const { o, b, target } = fixture(false, grid)
        const before = { ...b }
        const ticks = o.ticksFlown
        const found = counted(o, () => o.hopPrefix(b, target, Infinity, yaw0))
        expect(found.steps).to.be.at.most(LIVE_PREFIX_TICKS)
        expect(o.ticksFlown - ticks, 'every kernel step charged once').to.equal(found.steps)
        expect(found.value).to.not.equal(null)
        const p = found.value as HopProgram
        expect(b).to.deep.equal(before)
        expect(p.start).to.deep.equal(before)
        expect(p.start).to.not.equal(b)
        expect([p.sx, p.sy, p.sz, p.yaw0]).to.deep.equal([b.x, b.y, b.z, yaw0])
        expect(p.seq).to.equal(undefined)
        expect(p.legs).to.not.equal(true)
        const prefix = p.groundPrefix!
        expect(prefix.length).to.be.within(1, 4)
        expect(p.jumpAt).to.be.at.least(prefix.length)
        expect(inspect(o).requiredNudges(p, n)).to.deep.equal(retained)
        const sim = inspect(o).sim
        for (const [nx, nz] of [[0, 0], ...retained]) {
          const q = { ...b, x: b.x + nx, z: b.z + nz }
          let yaw = yaw0
          for (const press of prefix) {
            yaw += Math.round((press.yaw - yaw) / LOOK_STEP) * LOOK_STEP
            const c: SimControl = {
              forward: true, back: false, left: false, right: false, jump: false,
              sprint: press.sprint, sneak: press.sneak,
              hx: -Math.sin(Math.PI - yaw), hz: Math.cos(Math.PI - yaw)
            }
            sim.step(q, c)
            expect(q.onGround, `grounded ${nx},${nz}`).to.equal(true)
            expect(sim.carried(q), `carried ${nx},${nz}`).to.equal(true)
            expect(q.y).to.equal(b.y)
            expect(q.inLiquid).to.equal(false)
            expect(sim.collides(q)).to.equal(false)
          }
        }
        expectFlies(o, p, retained)
      }
    })

    it('rejects a retreat off a thin support even while onGround stays set', function () {
      if (!LIVE_PREFIX) this.skip()
      const { o, b, target } = fixture(true)
      const sim = inspect(o).sim
      const n = LIVE_NUDGE
      for (const [nx, nz] of [[0, 0], [n, 0], [-n, 0], [0, n], [0, -n]]) {
        expect(sim.carried({ ...b, x: b.x + nx, z: b.z + nz })).to.equal(true)
      }
      const before = { ...b }
      let lostSupport = false
      const found = counted(o, () => o.hopPrefix(b, target, Infinity, 0.3), (q, c) => {
        if (c.forward && !c.jump && q.onGround && !sim.carried(q)) lostSupport = true
      })
      expect(found.value).to.equal(null)
      expect(lostSupport).to.equal(true)
      expect(found.steps).to.be.at.most(LIVE_PREFIX_TICKS)
      expect(o.ticksFlown).to.equal(found.steps)
      expect(b).to.deep.equal(before)
    })

    it('returns null at exhaustion without exceeding even a one-tick allowance', () => {
      for (const spent of [0, 1]) {
        const { o, b, target } = fixture()
        o.tickLimit = 1
        o.ticksFlown = spent
        const found = counted(o, () => o.hopPrefix(b, target, Infinity, 0.3))
        expect(found.value).to.equal(null)
        expect(found.steps).to.be.at.most(LIVE_PREFIX ? 1 - spent : 0)
        expect(o.ticksFlown - spent).to.equal(found.steps)
        expect(inspect(o).prefixBudget).to.equal(null)
      }
    })

    it('discards a candidate whose original-start verification exhausts the allowance', function () {
      if (!LIVE_PREFIX || process.env.PF_LIVE_NUDGE_SAFE === '0') this.skip()
      const { o, b, target } = fixture()
      const fly = o.fly
      let verifying = false
      o.fly = (p, nx = 0, nz = 0, track = null): number => {
        if (!verifying) {
          verifying = true
          expect(inspect(o).prefixBudget).to.not.equal(null)
          inspect(o).prefixBudget!.end = o.ticksFlown + 1
        }
        return fly.call(o, p, nx, nz, track)
      }
      try {
        const found = counted(o, () => o.hopPrefix(b, target, Infinity, 0.3))
        expect(verifying).to.equal(true)
        expect(found.value).to.equal(null)
        expect(found.steps).to.be.at.most(LIVE_PREFIX_TICKS)
        expect(o.ticksFlown).to.equal(found.steps)
        expect(inspect(o).prefixBudget).to.equal(null)
      } finally { o.fly = fly }
    })
  })

  describe('wall-safe live nudges', () => {
    const safe = process.env.PF_LIVE_NUDGE_SAFE !== '0'
    const n = LIVE_NUDGE
    const offsets: Array<[number, number]> = [[0, 0], [n, 0], [-n, 0], [0, n], [0, -n]]
    type NudgeOracle = {
      sim: { collides: (b: ReturnType<typeof newBody>) => boolean }
      requiredNudges: (p: HopProgram, n: number) => Array<[number, number]>
      origin: (p: HopProgram, nx: number, nz: number) => unknown
      beamSearch: (...args: unknown[]) => HopProgram | null
    }
    const inspect = (o: HopOracle): NudgeOracle => o as unknown as NudgeOracle

    function fixture (wall: boolean, grid: boolean): { o: HopOracle, p: HopProgram } {
      const solid = [...floor([-2, 1], [0, 2]), ...floor([3, 6], [0, 2])]
      if (wall) for (let x = -2; x <= 6; x++) solid.push([x, 1, -1], [x, 2, -1])
      const o = oracleOver(solid, grid)
      const b = newBody(0.5, 1, o.halfWidth)
      b.vx = 0.18
      b.vy = -0.0784
      b.onGround = true
      const h = { fixed: true, x: 1, z: 0 }
      const p: HopProgram = {
        sx: b.x, sy: b.y, sz: b.z, start: b, run: h, jumpAt: 3,
        air1: h, turnAir: Infinity, air2: h, sprint: false, ticks: 0,
        tx: 3, ty: 1, tz: 0, catchTarget: false, family: 'run-turn'
      }
      return { o, p }
    }

    it('flies along a flush wall and over the gap without the embedded twin (mcc-6-2)', () => {
      for (const grid of [true, false]) {
        const { o, p } = fixture(true, grid)
        const q = inspect(o)
        const b = p.start as ReturnType<typeof newBody>
        const before = { ...b }
        // mcc-6-2: the inward twin catches the next wall block's X face;
        // the real body and the other twins run along the wall to the gap.
        expect(offsets.map(([nx, nz]) => q.sim.collides({ ...b, x: b.x + nx, z: b.z + nz })))
          .to.deep.equal([false, false, false, false, true])
        const flights = offsets.map(([nx, nz]) => o.fly(p, nx, nz))
        expect(flights.map(t => t > 0), JSON.stringify(flights)).to.deep.equal([true, true, true, true, false])
        expect(flights[4]).to.equal(-1)
        p.ticks = flights[0]
        expect(q.requiredNudges(p, n)).to.deep.equal(safe ? offsets.slice(1, 4) : offsets.slice(1))
        expect(o.verify(p)).to.equal(safe ? p.ticks : -1)
        for (const [nx, nz] of offsets.slice(0, 4)) {
          expect(o.fly(p, nx, nz), `retained ${nx},${nz}`).to.be.greaterThan(0)
        }
        expect(o.fly(p)).to.equal(p.ticks)
        expect(b).to.deep.equal(before)
        for (const planned of [{ ...p, start: undefined }, { ...p, start: undefined, seq: [0, 0], precise: true }]) {
          expect(q.requiredNudges(planned, n), 'planned starts stay strict').to.deep.equal(offsets.slice(1))
        }
      }
    })

    it('keeps all four nudges in free space', () => {
      for (const grid of [true, false]) {
        const { o, p } = fixture(false, grid)
        expect(inspect(o).requiredNudges(p, n)).to.deep.equal(offsets.slice(1))
        p.ticks = o.fly(p)
        expect(p.ticks).to.be.greaterThan(0)
        expectFlies(o, p)
      }
    })

    it('keeps every nudge when the nominal body already overlaps a block', () => {
      const { o, p } = fixture(true, true)
      const b = p.start as ReturnType<typeof newBody>
      b.z -= n / 2
      p.sz = b.z
      expect(inspect(o).sim.collides(b)).to.equal(true)
      expect(inspect(o).requiredNudges(p, n)).to.deep.equal(offsets.slice(1))
      const seen: Array<[number, number]> = []
      o.fly = (_p, nx = 0, nz = 0) => {
        seen.push([nx, nz])
        return nx === 0 && nz === -n ? -1 : 7
      }
      expect(o.verify(p), 'an embedded twin still vetoes').to.equal(-1)
      expect(seen).to.deep.equal(offsets)
    })

    it('keeps both sides of an axis when both collide', () => {
      const { p } = fixture(false, true)
      const o = oracleOver([[0, 1, -1], [0, 2, -1], [0, 1, 1], [0, 2, 1]], true)
      const b = p.start as ReturnType<typeof newBody>
      b.z = 0.5
      expect(inspect(o).sim.collides(b)).to.equal(false)
      expect([0.25, -0.25].map(nz => inspect(o).sim.collides({ ...b, z: b.z + nz })))
        .to.deep.equal([true, true])
      expect(inspect(o).requiredNudges(p, 0.25)).to.deep.equal([[0.25, 0], [-0.25, 0], [0, 0.25], [0, -0.25]])
    })

    it('builds and advances the beam with every required twin', () => {
      const { o, p } = fixture(true, true)
      const q = inspect(o)
      const origin = q.origin.bind(o)
      const f = origin(p, 0, 0)
      const seen: Array<[number, number]> = []
      q.origin = (program, nx, nz) => {
        seen.push([nx, nz])
        return origin(program, nx, nz)
      }
      expect(q.beamSearch([{ f, live: p.start }], [{ x: 3.5, y: 1, z: 0.5, catch: false }],
        [3, 1, 0], Infinity, [], undefined, 1)).to.equal(null)
      expect(seen).to.deep.equal(safe ? offsets.slice(1, 4) : offsets.slice(1))
      expect(o.ticksFlown, 'the reduced ensemble is not rejected as incomplete').to.be.greaterThan(0)
    })
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

  it('searches seed families on a budget of their own, first or only where the live families find nothing (hopFrom seed)', () => {
    const mk = (): ReturnType<typeof newBody> => {
      const b = newBody(0.5, 1, -2.5)
      b.vz = 0.18
      b.onGround = true
      return b
    }
    // the live families out of budget at once: only the seed's own work finds a flight
    const o = oracleOver(gap, true)
    o.tickLimit = 1
    const seeded = o.hopFrom(mk(), [0, 1, 5], performance.now() + 20000, NaN, undefined, undefined, false, ['pivot', 'kick'], 200_000, false)
    expect(seeded).to.not.equal(null)
    expect((seeded as HopProgram).family).to.equal('kick')
    expectFlies(o, seeded as HopProgram)
    // the families with a budget find theirs; a seed after them never runs
    const plain = oracleOver(gap, true).hopFrom(mk(), [0, 1, 5], performance.now() + 20000) as HopProgram
    const after = oracleOver(gap, true).hopFrom(mk(), [0, 1, 5], performance.now() + 20000, NaN, undefined, undefined, true, ['kick'], 200_000, false) as HopProgram
    expect(after.family).to.equal(plain.family)
    expect(after.ticks).to.equal(plain.ticks)
    // first: the families still run on their own budget, never slower than they are alone
    const o4 = oracleOver(gap, true)
    o4.tickLimit = 100_000
    const first = o4.hopFrom(mk(), [0, 1, 5], performance.now() + 20000, NaN, undefined, undefined, true, ['kick', 'pivot'], 200_000, true) as HopProgram
    expect(first).to.not.equal(null)
    expect(first.ticks).to.be.at.most(plain.ticks)
    expectFlies(o4, first)
  })
  it('pursues a kick lane from a body drifting off a 3/16 strip (PF_LANE_KICK)', () => {
    // mcc-8-2: an outward drift leaves the strip before a parallel run gets
    // within jumping distance; a point ahead on the lane steers back in.
    const strips: Array<[number, number, number, number]> = [[1, 0, 5, TRAPDOOR_LEDGE]]
    for (let z = -2; z <= 1; z++) strips.push([0, 0, z, TRAPDOOR_LEDGE])
    const world = (): HopOracle => oracleOver([], true, strips)
    const target: [number, number, number] = [1, 1, 5]
    const plan: HopProgram = {
      sx: 3 / 32, sy: 1, sz: -1.1,
      run: { fixed: true, x: 0, z: 1 }, jumpAt: 11, kick: true,
      air1: { fixed: true, x: Math.sin(0.52), z: Math.cos(0.52) }, turnAir: 1,
      air2: { fixed: true, x: 0, z: 1 }, sprint: true, ticks: 0,
      tx: target[0], ty: target[1], tz: target[2], catchTarget: false, family: 'kick'
    }
    const planned = world()
    plan.ticks = planned.verify(plan)
    expect(plan.ticks, 'the kick from rest on the lane').to.be.greaterThan(0)
    expectFlies(planned, plan)

    const b = newBody(plan.sx + 0.3, plan.sy, plan.sz)
    b.onGround = true
    b.vy = -0.0784
    b.vx = 0.055
    const plain = world()
    const search = plain as unknown as {
      search: (pads: unknown[], lands: unknown[], t: [number, number, number], family: 'kick', deadline: number, walk: boolean, grid: boolean) => HopProgram | null
    }
    const pads = [{
      f: { b: { ...b }, t: 0, fired: false, air: 0, rebounds: 0, yaw: NaN, leg: 0, top: b.y },
      live: { ...b }
    }]
    expect(search.search(pads, plain.stands(...target), target, 'kick', Infinity, true, true), 'every plain kick class').to.equal(null)
    expect(plain.noVerdict, 'exhausted the family, not a budget').to.equal(false)

    const guided = world()
    guided.ticksFlown = 1
    guided.tickLimit = 1
    const p = guided.hopFrom(b, target, Infinity, NaN, undefined, undefined, false, undefined, 0, true, plan)
    if (LANE_KICK_TICKS === 0) {
      expect(p, 'disabled: no allowance beyond the spent live budget').to.equal(null)
      return
    }
    expect(p, 'the lane pass has its own budget').to.not.equal(null)
    const q = p as HopProgram
    expect(q.family).to.equal('kick')
    expect(q.run.fixed, 'pursues the lane').to.equal(false)
    expect(q.start).to.deep.equal(b)
    expect(guided.tickLimit, 'the spent live budget stays spent').to.equal(guided.ticksFlown)
    expectFlies(guided, q)
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

  describe('hopChain: the leg onto a stone the plan goes straight on from (arena mcc-5-2)', () => {
    // a run of stone, a 1/8 pane rail four on, a stone 4.5 past the rail's middle at the same level
    const world = (): HopOracle => oracleOver(
      [[0, 0, -1], [0, 0, 0], [-1, 0, 9], [0, 0, 9], [1, 0, 9]], true,
      [[-1, 0, 4, PANE_EW], [0, 0, 4, PANE_EW], [1, 0, 4, PANE_EW]])
    const stone: [number, number, number] = [0, 1, 4]
    const next: [number, number, number] = [0, 1, 9]
    const running = (z: number): ReturnType<typeof newBody> => {
      const b = newBody(0.5, 1, z)
      b.vz = 0.24
      b.vy = -0.0784
      b.onGround = true
      return b
    }
    // (the touchdown a go-landing program comes down at, for the arrival's own search)
    const touchdown = (o: HopOracle, p: HopProgram): ReturnType<typeof newBody> => {
      const q = o as unknown as { goLanding: boolean, touchdownOf: (p: HopProgram) => ReturnType<typeof newBody> | null }
      q.goLanding = true
      const td = q.touchdownOf(p)
      q.goLanding = false
      expect(td, 'touches down').to.not.equal(null)
      return td as ReturnType<typeof newBody>
    }

    // (each search on a budget of its own: a null that only ran out says nothing)
    const fresh = (o: HopOracle, ticks = 400_000): void => { o.tickLimit = o.ticksFlown + ticks }
    const ranOut = (o: HopOracle): boolean => o.ticksFlown >= o.tickLimit

    it('from the run-up\'s phases that have one, a touchdown the next leg really flies from; a stopping landing has none from any', () => {
      // (z 0.16 and 0.2 have no two-leg flight in these families at all: not asked)
      let stoppers = 0
      const phases = [0.0, 0.04, 0.08, 0.12, 0.24, 0.28, 0.32]
      for (const z of phases) {
        const o = world()
        fresh(o)
        // (the plain search, a landing it can stop on: too slow to go on from)
        const plain = o.hopFrom(running(z), stone, performance.now() + 30000)
        expect(ranOut(o), `the plain search ran to its end from z ${z}`).to.equal(false)
        if (plain !== null) {
          fresh(o)
          const pd = (o as unknown as { touchdownOf: (p: HopProgram) => ReturnType<typeof newBody> | null }).touchdownOf(plain)
          fresh(o)
          // (the same families as the chain's next leg is judged by: no beam either side)
          const on = pd === null ? null : o.hopFrom(pd, next, performance.now() + 30000, NaN, undefined, undefined, false)
          expect(ranOut(o), `the stopper's next leg ran to its end from z ${z}`).to.equal(false)
          if (on === null) stoppers++
        } else {
          stoppers++
        }
        fresh(o)
        const p = o.hopChain(running(z), stone, next, performance.now() + 30000)
        expect(p, `a stone from z ${z}`).to.not.equal(null)
        const q = p as HopProgram
        expect(q.tz).to.equal(4)
        const td = touchdown(o, q)
        expect(Math.floor(td.z)).to.equal(4)
        fresh(o)
        const on = o.hopFrom(td, next, performance.now() + 30000, NaN, undefined, undefined, false)
        expect(on, `the next leg from z ${z}`).to.not.equal(null)
        expect((on as HopProgram).tz).to.equal(9)
      }
      expect(stoppers, 'the plain search strands the body on the rail').to.equal(phases.length)
    })

    it('no stone without a next leg: from a phase that has one, a landing past reach is null', () => {
      // (the same rail and phase: a pick with the stone at z 9, none with it nine blocks on)
      const near = world()
      fresh(near)
      expect(near.hopChain(running(0.24), stone, next, performance.now() + 30000), 'reachable').to.not.equal(null)
      const far = oracleOver([[0, 0, -1], [0, 0, 0], [0, 0, 13]], true, [[-1, 0, 4, PANE_EW], [0, 0, 4, PANE_EW], [1, 0, 4, PANE_EW]])
      fresh(far)
      expect(far.hopChain(running(0.24), stone, [0, 1, 13], performance.now() + 30000)).to.equal(null)
      expect(ranOut(far), 'judged, not run out').to.equal(false)
    })

    it('leaves the oracle as it found it: an ordinary search after it still wants a stop', () => {
      const plain = world()
      plain.tickLimit = 400_000
      const before = plain.hopFrom(running(0.24), stone, performance.now() + 30000)
      const o = world()
      o.tickLimit = 300_000
      o.hopChain(running(0.24), stone, next, performance.now() + 30000)
      const q = o as unknown as { goLanding: boolean, collect: unknown, collectSlack: number, collectCap: number }
      expect(q.goLanding).to.equal(false)
      expect(q.collect).to.equal(null)
      expect(q.collectSlack, 'slack').to.equal(1)
      expect(q.collectCap, 'cap').to.equal(6)
      expect(o.tickLimit, 'the caller\'s budget').to.equal(300_000)
      o.tickLimit = 400_000
      const after = o.hopFrom(running(0.24), stone, performance.now() + 30000)
      expect(after?.ticks, 'the same search as on a fresh oracle').to.equal(before?.ticks)
      expect(after?.family).to.equal(before?.family)
    })

    it('stays within the budget it is given, a search\'s grace past it at most (observed bound)', () => {
      // (nothing past the rail flies: every round spent, the whole budget asked of it)
      for (const limit of [20_000, 60_000, 150_000]) {
        const far = oracleOver([[0, 0, -1], [0, 0, 0], [0, 0, 11]], true, [[-1, 0, 4, PANE_EW], [0, 0, 4, PANE_EW], [1, 0, 4, PANE_EW]])
        far.ticksFlown = 0
        far.tickLimit = limit
        far.hopChain(running(0.24), stone, [0, 1, 11], performance.now() + 30000)
        expect(far.ticksFlown, `of ${limit}`).to.be.at.most(limit + 12_000)
      }
    })

    it('goes on to a stone past the next one (nextGo): the next rail\'s touchdown need not stop, and the pick is the quicker for it', () => {
      // (a second rail four on: a stopping landing on it wants a slower first leg, 15 ticks
      // against 14 - measured, nextGoProbe.ts; nextGo weighs the touchdown as the landing)
      const rails = (): HopOracle => oracleOver([[0, 0, -1], [0, 0, 0], [0, 0, 12]], true,
        [[-1, 0, 4, PANE_EW], [0, 0, 4, PANE_EW], [1, 0, 4, PANE_EW], [-1, 0, 8, PANE_EW], [0, 0, 8, PANE_EW], [1, 0, 8, PANE_EW]])
      let quicker = 0
      for (const z of [0.0, 0.08, 0.24]) {
        const a = rails()
        fresh(a)
        const stop = a.hopChain(running(z), stone, [0, 1, 8], performance.now() + 30000, NaN, false)
        const b = rails()
        fresh(b)
        const go = b.hopChain(running(z), stone, [0, 1, 8], performance.now() + 30000, NaN, true)
        expect(stop, `a stopping pick from z ${z}`).to.not.equal(null)
        expect(go, `a going pick from z ${z}`).to.not.equal(null)
        expect((go as HopProgram).tz).to.equal(4)
        expect((go as HopProgram).ticks, `never slower from z ${z}`).to.be.at.most((stop as HopProgram).ticks)
        if ((go as HopProgram).ticks < (stop as HopProgram).ticks) quicker++
      }
      expect(quicker, 'quicker from some phase').to.be.greaterThan(0)
    })
  })

  it('beam: ranks by the way round two panes across a strip, one flight round both quicker than the straight line finds (BEAM_DETOUR)', () => {
    // arena opk-2's double neo: a one-wide strip along z, two-high panes across it at z 1 and 3, the landing at z 4
    const strip = floor([0, 0], [-4, 4])
    const panes: Array<[number, number, number, number]> = [[0, 1, 1, PANE_EW], [0, 2, 1, PANE_EW], [0, 1, 3, PANE_EW], [0, 2, 3, PANE_EW]]
    // (the body as it comes down on the strip before the first pane, running at it)
    const mk = (): ReturnType<typeof newBody> => {
      const b = newBody(0.5, 1, 0.16)
      b.vz = 0.266
      b.onGround = true
      return b
    }
    const run = (detour: boolean): HopProgram | null => {
      const o = oracleOver(strip, true, panes)
      o.beamDetour = detour
      o.tickLimit = 800_000
      return o.hopFrom(mk(), [0, 1, 4], performance.now() + 60000)
    }
    // (the straight line: a touch-and-go on the strip between the panes here, 20 ticks; none at all in the arena)
    const straight = run(false)
    const p = run(true)
    expect(p, 'detour field').to.not.equal(null)
    const q = p as HopProgram
    expect(q.ticks).to.be.at.most(18)
    if (straight !== null) expect(q.ticks, 'quicker than the straight line').to.be.lessThan(straight.ticks)
    const o = oracleOver(strip, true, panes)
    expectFlies(o, q)
    // one flight, held to the landing from its nudged starts (the skip-ahead's test)
    expect(o.singleFlight(q)).to.equal(true)
  })

  it('beam: with nothing in the way, or only a step to jump, the detour field is off and the beam the same (BEAM_DETOUR)', () => {
    const run = (solid: Array<[number, number, number]>, detour: boolean): HopProgram | null => {
      const o = oracleOver(solid, true)
      o.beamDetour = detour
      return o.hop([0, 1, 0], [0, 1, 5], performance.now() + 30000, 'beam')
    }
    expect(run(gap, true)).to.deep.equal(run(gap, false))
    // a one-high wall across the landing's near edge is jumped, not walked round
    const step: Array<[number, number, number]> = [...gap, [-2, 1, 5], [-1, 1, 5], [1, 1, 5], [2, 1, 5]]
    const fieldOf = (solid: Array<[number, number, number]>): unknown => {
      const o = oracleOver(solid, true) as unknown as { detourField: (t: [number, number, number], y: number, pads: unknown[]) => unknown }
      return o.detourField([0, 1, 6], 1, [{ f: { b: newBody(0.5, 1, 0.5) }, live: undefined }])
    }
    expect(fieldOf(gap), 'nothing in the way').to.equal(null)
    expect(fieldOf(step), 'a step').to.equal(null)
  })

  it('beam: the detour field seeds a landing under a head-hitter, free at its own floor only (DetourField)', () => {
    // a pad of soul sand (feet 0.875), a two-high wall across the way, the
    // landing two blocks under a ceiling: free at its floor's level alone
    const solid: Array<[number, number, number]> = [...floor([-3, 3], [-1, 6]), [0, 3, 5]]
    for (let x = -1; x <= 1; x++) solid.push([x, 1, 2], [x, 2, 2])
    const o = oracleOver(solid, true, [[0, 0, 0, SOUL_SAND]]) as unknown as { detourField: (t: [number, number, number], y: number, pads: unknown[]) => { at: (x: number, z: number) => number } | null }
    const f = o.detourField([0, 1, 5], 1, [{ f: { b: newBody(0.5, 0.875, 0.5) }, live: undefined }])
    expect(f, 'a field').to.not.equal(null)
    // round the wall's end, not the cap every point reads when no landing point is free
    expect((f as { at: (x: number, z: number) => number }).at(0.5, 0.5)).to.be.within(1, 3)
  })

  it('tells a program whose landing hurts from one whose does not (landingHurts)', () => {
    // a five-block drop: a jump off it falls six and a quarter, three points
    const high = oracleOver([[0, 5, 0], ...floor([-2, 2], [3, 6])], true)
    const p = high.hop([0, 6, 0], [0, 1, 4], performance.now() + 20000)
    expect(p, 'off the tower').to.not.equal(null)
    expect(high.landingHurts(p as HopProgram)).to.equal(true)
    // one block down: a fall of two and a quarter, nothing
    const low = oracleOver([[0, 1, 0], ...floor([-2, 2], [3, 6])], true)
    const q = low.hop([0, 2, 0], [0, 1, 4], performance.now() + 20000)
    expect(q, 'off the step').to.not.equal(null)
    expect(low.landingHurts(q as HopProgram)).to.equal(false)
  })

  it('beam: the detour field is the way round a wall, and nothing where the line is clear (DetourField)', () => {
    // a two-high wall three wide across the way at z 2, the landing at z 4
    const wall: Array<[number, number, number]> = [...floor([-3, 3], [-1, 5])]
    for (let x = -1; x <= 1; x++) wall.push([x, 1, 2], [x, 2, 2])
    const o = oracleOver(wall, true) as unknown as { detourField: (t: [number, number, number], y: number, pads: unknown[]) => { at: (x: number, z: number) => number } | null }
    const f = o.detourField([0, 1, 4], 1, [{ f: { b: newBody(0.5, 1, 0.5) }, live: undefined }])
    expect(f).to.not.equal(null)
    const field = f as { at: (x: number, z: number) => number }
    // straight behind the wall: round its end and back, about 2.2 more than the line (octile)
    expect(field.at(0.5, 0.5)).to.be.within(1.8, 2.8)
    // past it, the line clear
    expect(field.at(0.5, 3.6)).to.be.below(0.05)
    // off its end, the line clear
    expect(field.at(3.5, 4)).to.be.below(0.05)
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
    // (a flight down on the rail's top, for the floor past it: its ticks run on to that floor, the executor's arrival - REGION_SETTLE)
    const priced = o as unknown as { goLanding: boolean, touch: SimBody | null }
    let sawOff = false
    for (const e of edges) {
      const p = e.program
      const td = newBody(0, 0, 0)
      priced.goLanding = true
      priced.touch = td
      const touched = o.fly(p)
      priced.touch = null
      priced.goLanding = false
      const off = Math.abs(td.y - p.land!.y) >= 0.5
      if (off) sawOff = true
      // (the next leg leaves from the arrival the price means, not the rail's top)
      const arrived = newBody(0, 0, 0)
      priced.touch = arrived
      o.fly(p)
      priced.touch = null
      if (off && REGION_SETTLE) expect(Math.abs(arrived.y - p.land!.y), 'arrival body at its level').to.be.lessThan(0.5)
      if (off && REGION_SETTLE) expect(p.ticks, 'its settle priced').to.be.greaterThan(touched)
      else expect(p.ticks, 'its touchdown').to.equal(touched)
    }
    expect(sawOff, 'one of them touches the rail first').to.equal(true)
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
  it('lines up a moving body by a verified glide or one press, replayed to an anchored rest', function () {
    if (process.env.PF_MOVING_LINEUP === '0') this.skip()
    const o = oracleOver(floor([0, 2], [0, 0]), true)
    const sim = (o as unknown as { sim: PlayerSim }).sim
    const yaw = Math.PI / 2
    // A vertical beam hop near the landing's overlap limit: the extra glide
    // misses it, while one press puts it back (mcc-7-2's two-rest line-up).
    const h = { fixed: true, x: 0, z: 1 }
    const p: HopProgram = {
      sx: 1.18, sy: 1, sz: 0.5, run: h, jumpAt: 0, air1: h, turnAir: 99, air2: h,
      sprint: false, ticks: 0, tx: 0, ty: 1, tz: 0, catchTarget: false,
      family: 'beam', precise: true, seq: [0, F_JUMP, NaN, 0]
    }
    p.ticks = o.verify(p)
    expect(p.ticks).to.be.greaterThan(0)
    const replay = (b: SimBody, yaws: number[], delay = 0): SimBody => {
      const q = { ...b }
      const c: SimControl = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: true, hx: 0, hz: -1 }
      for (let k = 0; k < delay; k++) sim.step(q, c)
      for (const y of yaws) {
        c.forward = true
        c.hx = -Math.sin(Math.PI - y); c.hz = Math.cos(Math.PI - y)
        sim.step(q, c)
      }
      c.forward = false
      for (let k = 0; k < 60; k++) {
        sim.step(q, c)
        expect(q.onGround).to.equal(true)
        expect(sim.carried(q)).to.equal(true)
        expect(q.y).to.equal(p.sy)
        expect(q.inLiquid).to.equal(false)
        if (q.vx === 0 && q.vz === 0) break
      }
      expect([q.vx, q.vz]).to.deep.equal([0, 0])
      return q
    }
    for (const presses of [0, 1]) {
      const seed = newBody(0.5, 1, 0.5)
      seed.onGround = true
      seed.vy = -0.0784
      seed.vx = 0.02
      const end = replay(seed, presses === 0 ? [] : [yaw])
      const b = { ...seed, x: seed.x + p.sx - end.x, z: seed.z + p.sz - end.z }
      const before = { ...b }
      const glide = replay(b, [])
      const zero = o.anchored(p, glide, yaw)
      if (presses === 0) expect(zero).to.not.equal(null)
      else expect(zero, 'the uncorrected glide cannot anchor').to.equal(null)
      // mcc-7-2's prefix must release scratch controls left by an earlier anchor.
      Object.assign((o as unknown as { ctl: SimControl }).ctl, {
        forward: true, back: true, left: true, right: true, jump: true, sprint: true, sneak: false
      })
      const moving = { reach: 0.11, tried: false }
      const ticks = o.ticksFlown
      o.tickLimit = ticks + 60_000
      const lu = o.lineUp(p, b, yaw, performance.now() + 5000, moving)
      expect(moving.tried).to.equal(true)
      expect(lu).to.not.equal(null)
      expect(o.ticksFlown - ticks).to.be.at.most(60_000)
      expect(b).to.deep.equal(before)
      const found = lu as NonNullable<typeof lu>
      expect(found.yaws.length).to.equal(presses)
      expect(found.delay ?? 0).to.be.within(0, 1)
      for (const y of found.yaws) {
        const m = (y - yaw) / LOOK_STEP
        expect(Math.abs(m - Math.round(m))).to.be.lessThan(1e-6)
      }
      const observed = replay(b, found.yaws, found.delay)
      expect(observed).to.deep.equal(found.rest)
      expect(o.fly(found.prog)).to.equal(found.prog.ticks)
      const live = o.anchored(p, observed, found.yaws[found.yaws.length - 1] ?? yaw)
      expect(live, 'anchor the observed rest again').to.not.equal(null)
      expect(o.fly(live as HopProgram)).to.equal((live as HopProgram).ticks)
    }
  })

  it('does not spend a moving attempt on a distant glide or a coast off a panel', function () {
    if (process.env.PF_MOVING_LINEUP === '0') this.skip()
    for (const panel of [false, true]) {
      const o = panel ? oracleOver([], true, [[0, 0, 0, TRAPDOOR_LEDGE]]) : oracleOver(floor([0, 2], [0, 0]), true)
      const b = newBody(0.45, 1, 0.5)
      b.onGround = true
      b.vy = -0.0784
      b.vx = 0.04
      const p = { sx: panel ? 0.45 : 2, sy: 1, sz: 0.5 } as HopProgram
      const moving = { reach: 0.11, tried: false }
      o.tickLimit = 60_000
      expect(o.lineUp(p, b, 0, performance.now() + 5000, moving)).to.equal(null)
      expect(moving.tried).to.equal(false)
    }
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
