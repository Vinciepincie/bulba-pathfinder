// gaitSearch catch landings (GaitGoal.catchAt): a jump onto a ladder is caught and climbed in the kernel.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody, copyBody, PlayerSim } from '../src/playerSim.js'
import type { SimBody, SimControl } from '../src/playerSim.js'
import { gaitSearch } from '../src/gaitSearch.js'
import type { GaitNode } from '../src/gaitSearch.js'

const lut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
  lutFingerprint: () => 'gait', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
  liquids: new Set(), fences: new Set(), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
} as never)
const tables = simTables(lut, mcData.blocksArray as never, s => {
  const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
  return p.waterlogged === true || p.waterlogged === 'true'
})
const ladder = mcData.blocksByName.ladder
let LADDER_N = -1
for (let s = ladder.minStateId as number; s <= (ladder.maxStateId as number); s++) {
  const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
  if (p.facing === 'north' && String(p.waterlogged) === 'false') { LADDER_N = s; break }
}

function world (fill: (put: (x: number, y: number, z: number, s?: number) => void) => void): PlayerSim {
  const x0 = -16; const y0 = -4; const z0 = -16; const w = 48; const h = 24; const l = 64
  const states = new Int32Array(w * h * l).fill(AIR)
  fill((x, y, z, s = STONE) => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = s })
  const stateAt = (x: number, y: number, z: number): number => {
    const lx = x - x0; const ly = y - y0; const lz = z - z0
    if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
    return states[(ly * l + lz) * w + lx]
  }
  return new PlayerSim({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } }, 0.30001, 1.80001)
}

// mcc-4-1's second ladder (x mirrored): a 1x1 take-off pillar (top 5) and, 3 across and 6 on, a 1x1
// pillar (top 5) with ladders on its north face at 2 and 3 — the catch node stands on the ladder's top (4).
function scene (far = 0): PlayerSim {
  return world(put => {
    for (let y = 0; y <= 4; y++) put(0, y, 0)
    for (let y = 2; y <= 4; y++) put(3, y, 6 + far)
    for (let y = 2; y <= 3; y++) put(3, y, 5 + far, LADDER_N)
  })
}
const body = (x: number, y: number, z: number, vx = 0, vz = 0): SimBody => {
  const b = newBody(x, y, z); b.vx = vx; b.vz = vz; b.vy = -0.0784; b.onGround = true; return b
}

describe('gaitSearch: ladder catch landings', () => {
  it('finds a jump that catches the ladder', () => {
    const k = scene()
    const path: GaitNode[] = [{ x: 3.5, y: 4, z: 5.7, parkour: true }]
    const b = body(0.52, 5, 0.27, 0, 0.121)
    const on = gaitSearch(k, b, path, { index: 0, last: false, catchAt: 0 })
    expect(Math.min(on.hop, on.run)).to.be.lessThan(60)
  })

  it('a schedule driven to its jump catches the column and climbs to the node', () => {
    const k = scene()
    const path: GaitNode[] = [{ x: 3.5, y: 4, z: 5.7, parkour: true }]
    const b = copyBody(newBody(0, 0, 0), body(0.52, 5, 0.27, 0, 0.121))
    const c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: 1 }
    let caught = -1
    let predicted = -1
    for (let t = 0; t < 60 && caught < 0; t++) {
      expect(b.onGround, 'grounded until the jump').to.equal(true)
      const r = gaitSearch(k, b, path, { index: 0, last: false, catchAt: 0 })
      if (predicted < 0) predicted = Math.min(r.hop, r.run)
      if (r.hop !== Infinity && r.run === Infinity) {
        for (let f = 0; f < 40; f++) {
          const yaw = r.flight[Math.min(f, r.flight.length - 1)]
          c.hx = -Math.sin(yaw); c.hz = -Math.cos(yaw); c.jump = f === 0; c.forward = true; c.sprint = true
          k.step(b, c)
          if (!b.onGround && k.onClimbable(b)) { caught = t + f + 1; break }
          expect(b.y).to.be.greaterThan(0)
        }
        break
      }
      expect(r.run, 'a schedule from here').to.not.equal(Infinity)
      const s = r.step as { yaw: number, sprint: boolean }
      c.hx = -Math.sin(s.yaw); c.hz = -Math.cos(s.yaw); c.jump = false; c.sprint = s.sprint
      k.step(b, c)
    }
    expect(caught).to.be.greaterThan(0)
    expect(Math.floor(b.x)).to.equal(3)
    expect(Math.floor(b.z)).to.equal(5)
    // the catch branch: into the wall (+z), sneaking; the node's level when the search said
    let t = caught
    for (; t < 80 && b.y < 4 - 0.01; t++) {
      c.hx = 0; c.hz = 1; c.jump = false; c.sprint = false; c.sneak = true
      k.step(b, c)
    }
    expect(Math.abs(t - predicted)).to.be.at.most(1)
  })

  it('a ladder out of reach gives no verdict', () => {
    const k = scene(4)
    const path: GaitNode[] = [{ x: 3.5, y: 4, z: 9.7, parkour: true }]
    const r = gaitSearch(k, body(0.52, 5, 0.27, 0, 0.121), path, { index: 0, last: false, catchAt: 0 })
    expect(r.hop).to.equal(Infinity)
    expect(r.run).to.equal(Infinity)
  })
})
