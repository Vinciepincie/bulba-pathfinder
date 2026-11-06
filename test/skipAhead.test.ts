// The skip-ahead's leg floor (src/physics.ts legFloor): the least a flight
// between two nodes can take, against which a live program straight past a
// stood node is weighed (PhysicsSim.skipAhead, plugin.ts SKIP_AHEAD). It must
// be a FLOOR - never more than a real flight takes - or a slower skip passes.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody, PlayerSim } from '../src/playerSim.js'
import type { SimControl } from '../src/playerSim.js'
import { legFloor } from '../src/physics.js'

const lut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
  lutFingerprint: () => 'skip', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
  liquids: new Set(), fences: new Set(), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
} as never)
const tables = simTables(lut, mcData.blocksArray as never, s => {
  const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
  return p.waterlogged === true || p.waterlogged === 'true'
})

function world (fill: (put: (x: number, y: number, z: number) => void) => void): PlayerSim {
  const x0 = -16; const y0 = -8; const z0 = -16; const w = 48; const h = 20; const l = 48
  const states = new Int32Array(w * h * l).fill(AIR)
  fill((x, y, z) => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = STONE })
  const stateAt = (x: number, y: number, z: number): number => {
    const lx = x - x0; const ly = y - y0; const lz = z - z0
    if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
    return states[(ly * l + lz) * w + lx]
  }
  return new PlayerSim({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } }, 0.30001, 1.80001)
}

/** Ticks a sprint jump from a run at (0.5, 1, z0) along +z flies until it comes down at level y (the kernel's own flight). */
function sprintJumpTicks (k: PlayerSim, toY: number): number {
  const b = newBody(0.5, 1, 0.5)
  b.onGround = true
  b.vy = -0.0784
  b.vz = 0.28
  const c: SimControl = { forward: true, back: false, left: false, right: false, jump: true, sprint: true, sneak: false, hx: 0, hz: 1 }
  for (let t = 1; t <= 60; t++) {
    k.step(b, c)
    c.jump = false
    if (t > 1 && b.onGround) return t
    if (b.vy < 0 && b.y < toY - 3) return Infinity
  }
  return Infinity
}

describe('skip-ahead: the leg floor (legFloor)', () => {
  it('is a floor under a real sprint jump on the flat, an up step and a drop', () => {
    for (const [dz, dy] of [[3, 0], [4, 0], [3, 1], [3, -1], [4, -2]] as const) {
      // a pad at the start and a landing pad dz ahead, dy up or down
      const k = world(put => {
        put(0, 0, 0)
        for (let x = -1; x <= 1; x++) put(x, dy, dz)
      })
      const floor = legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1 + dy, z: dz + 0.5 })
      const real = sprintJumpTicks(k, 1 + dy)
      expect(Number.isFinite(floor), `dz ${dz} dy ${dy}: a floor`).to.equal(true)
      if (Number.isFinite(real)) expect(floor, `dz ${dz} dy ${dy}: floor ${floor} vs flight ${real}`).to.be.at.most(real)
    }
  })

  it('a standing jump on the flat takes about 12 ticks; a walk-off a block down a handful', () => {
    const k = world(put => { put(0, 0, 0) })
    expect(legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 2.5 })).to.be.within(10, 13)
    expect(legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 0, z: 2.5 })).to.be.within(3, 7)
  })

  it('is a floor for a run off a half-block step down across a gap (a walk-off at any drop)', () => {
    // the take-off block, a one-block gap, a landing half a block lower
    const k = world(put => { put(0, 0, 0) })
    const floor = legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 0.5, z: 2.5 })
    // a body run off the edge at a sprint is down half a block in 4 ticks: no jump arc
    expect(floor).to.be.at.most(5)
  })

  it('is shorter under a header: the roof cuts the rise', () => {
    const open = world(put => { put(0, 0, 0) })
    const roofed = world(put => { put(0, 0, 0); put(0, 3, 0) })
    const a = legFloor(open, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 2.5 })
    const b = legFloor(roofed, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 2.5 })
    expect(b, `roofed ${b} vs open ${a}`).to.be.lessThan(a)
  })

  it('is shorter under a roof over the gap, with open sky at the take-off: the lowest roof on the way cuts the rise', () => {
    const open = world(put => { put(0, 0, 0) })
    // a roof two blocks over the gap's floor level, between the take-off and the landing, none over the take-off
    const roofed = world(put => { put(0, 0, 0); put(0, 3, 1); put(0, 3, 2) })
    const a = legFloor(open, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 3.5 })
    const b = legFloor(roofed, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 3.5 })
    expect(b, `roof over the gap ${b} vs open ${a}`).to.be.lessThan(a)
  })

  it('has no floor to a node higher than a standing jump reaches (no skip is judged then)', () => {
    const k = world(put => { put(0, 0, 0) })
    expect(legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 3, z: 1.5 })).to.equal(Infinity)
  })

  it('counts the ground between: far apart, the distance bounds it', () => {
    const k = world(put => { put(0, 0, 0) })
    const far = legFloor(k, { x: 0.5, y: 1, z: 0.5 }, { x: 0.5, y: 1, z: 12.5 })
    expect(far).to.be.at.least(Math.ceil((12 - 1.8) / 0.5))
  })
})
