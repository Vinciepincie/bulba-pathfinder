// gaitSearch (src/gaitSearch.ts): the schedules of hops, flown in the kernel.
// What it promises the executor: a hop it says to take reaches the goal
// sooner than keeping the feet down does, comes down on the path, and is
// handed over as the flight that was judged.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody, copyBody, PlayerSim } from '../src/playerSim.js'
import type { SimBody, SimControl } from '../src/playerSim.js'
import { gaitSearch, kWalkable, kJumpLands, kCaught, strafeYaw } from '../src/gaitSearch.js'
import type { GaitNode } from '../src/gaitSearch.js'

const lut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
  lutFingerprint: () => 'gait', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
  liquids: new Set(), fences: new Set(), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
} as never)
const tables = simTables(lut, mcData.blocksArray as never, s => {
  const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
  return p.waterlogged === true || p.waterlogged === 'true'
})

/** A kernel over the stone cells `fill` puts (air elsewhere inside the box, unloaded outside). */
function world (fill: (put: (x: number, y: number, z: number) => void) => void): PlayerSim {
  const x0 = -16; const y0 = -4; const z0 = -16; const w = 48; const h = 16; const l = 64
  const states = new Int32Array(w * h * l).fill(AIR)
  fill((x, y, z) => { states[((y - y0) * l + (z - z0)) * w + (x - x0)] = STONE })
  const stateAt = (x: number, y: number, z: number): number => {
    const lx = x - x0; const ly = y - y0; const lz = z - z0
    if (lx < 0 || lx >= w || ly < 0 || ly >= h || lz < 0 || lz >= l) return -1
    return states[(ly * l + lz) * w + lx]
  }
  return new PlayerSim({ ...tables, stateAt, grid: { states, x0, y0, z0, w, h, l } }, 0.30001, 1.80001)
}

/** A body on the ground at (x, y, z), moving (vx, vz). */
function body (x: number, y: number, z: number, vx = 0, vz = 0): SimBody {
  const b = newBody(x, y, z)
  b.vx = vx; b.vz = vz; b.vy = -0.0784; b.onGround = true
  return b
}

/** Nodes up the +z axis at x 0.5, from z0 + 0.5 to z1 + 0.5. */
function line (z0: number, z1: number): GaitNode[] {
  const out: GaitNode[] = []
  for (let z = z0; z <= z1; z++) out.push({ x: 0.5, y: 1, z: z + 0.5 })
  return out
}

/** Ticks a body running at the nodes in turn (no hop) takes to pass node `index`. */
function runTicks (k: PlayerSim, b0: SimBody, path: GaitNode[], index: number): number {
  const b = copyBody(newBody(0, 0, 0), b0)
  const c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: 1 }
  let i = 0
  for (let t = 1; t <= 200; t++) {
    const n = path[Math.min(i, path.length - 1)]
    const d = Math.hypot(n.x - b.x, n.z - b.z)
    if (d > 1e-9) { c.hx = (n.x - b.x) / d; c.hz = (n.z - b.z) / d }
    k.step(b, c)
    while (i <= index && Math.abs(path[i].x - b.x) <= 0.35 && Math.abs(path[i].z - b.z) <= 0.35) i++
    if (i > index) return t
  }
  return Infinity
}

describe('gaitSearch', function () {
  this.timeout(30000)

  const open = world(put => { for (let x = -3; x <= 3; x++) for (let z = -4; z <= 44; z++) put(x, 0, z) })

  it('hops over open ground, and sooner there than the feet get', () => {
    const path = line(1, 30)
    const b = body(0.5, 1, 0.5, 0, 0.28)
    const r = gaitSearch(open, b, path, { index: 8, jump: false, last: false }, 1)
    expect(r.hop).to.be.lessThan(Infinity)
    expect(r.run).to.equal(Infinity)
    expect(r.hop).to.be.lessThan(runTicks(open, b, path, 8))
  })

  it('hands the hop over as flown: one yaw a tick, from the take-off to the touchdown', () => {
    const path = line(1, 30)
    const r = gaitSearch(open, body(0.5, 1, 0.5, 0, 0.28), path, { index: 8, jump: false, last: false }, 1)
    // an open-sky hop is twelve ticks in the air
    expect(r.flight.length).to.equal(12)
    // replayed in the kernel it comes down where the path runs, a hop further on
    const b = body(0.5, 1, 0.5, 0, 0.28)
    const c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: 1 }
    r.flight.forEach((yaw, f) => {
      c.hx = -Math.sin(yaw)
      c.hz = -Math.cos(yaw)
      c.jump = f === 0
      open.step(b, c)
    })
    expect(b.onGround).to.equal(true)
    expect(b.z).to.be.greaterThan(4)
    expect(Math.abs(b.x - 0.5)).to.be.lessThan(0.05)
  })

  it('flies and runs with the 45° strafe: sooner there, the flight handed over with its looks', () => {
    const path = line(1, 30)
    const b = body(0.5, 1, 0.5, 0, 0.28)
    const plain = gaitSearch(open, b, path, { index: 8, jump: false, last: false }, 1)
    const r = gaitSearch(open, b, path, { index: 8, jump: false, last: false, strafe: true }, 1)
    expect(r.strafe).to.equal(true)
    expect(r.hop).to.be.lessThan(plain.hop)
    // the take-off looks down the line (the boost goes where the body looks), the air an eighth of a turn off it
    expect(Math.abs(Math.abs(r.flight[0]) - Math.PI)).to.be.lessThan(1e-9)
    expect(Math.cos(r.flight[1] - strafeYaw(r.flight[0]))).to.be.closeTo(1, 1e-9)
    // replayed with the left key held after the take-off, it flies the line: no drift
    const f = body(0.5, 1, 0.5, 0, 0.28)
    const c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: 1 }
    r.flight.forEach((yaw, t) => {
      c.hx = -Math.sin(yaw)
      c.hz = -Math.cos(yaw)
      c.jump = t === 0
      c.left = t > 0
      open.step(f, c)
    })
    expect(f.onGround).to.equal(true)
    expect(Math.abs(f.x - 0.5)).to.be.lessThan(1e-6)
    const g = body(0.5, 1, 0.5, 0, 0.28)
    plain.flight.forEach((yaw, t) => {
      c.hx = -Math.sin(yaw)
      c.hz = -Math.cos(yaw)
      c.jump = t === 0
      c.left = false
      open.step(g, c)
    })
    expect(f.z).to.be.greaterThan(g.z)
  })

  it('takes no hop that comes down off the path', () => {
    // the floor ends four blocks on: a hop from here lands past it
    const k = world(put => { for (let x = -3; x <= 3; x++) for (let z = -4; z <= 4; z++) put(x, 0, z) })
    const r = gaitSearch(k, body(0.5, 1, 1.5, 0, 0.28), line(2, 4), { index: 2, jump: false, last: true }, 1)
    expect(r.hop).to.equal(Infinity)
  })

  it('arrives at the last node on the ground: a hop that overflies it is no arrival', () => {
    const path = line(1, 5)
    const r = gaitSearch(open, body(0.5, 1, 0.5, 0, 0.28), path, { index: 4, jump: false, last: true }, 1)
    // whatever it chooses, the time is a body standing in the node's box
    expect(Math.min(r.hop, r.run)).to.be.greaterThan(10)
  })

  it('times the hop into a jump: from too near the lip it keeps its feet', () => {
    // a run of six, a gap of three, the landing
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 6; z++) put(x, 0, z)
      for (let x = -2; x <= 2; x++) for (let z = 10; z <= 16; z++) put(x, 0, z)
    })
    const path: GaitNode[] = [...line(1, 6), { x: 0.5, y: 1, z: 10.5, parkour: true }]
    const from = (z: number): ReturnType<typeof gaitSearch> => {
      const p = path.filter(n => n.z > z + 0.35)
      return gaitSearch(k, body(0.5, 1, z, 0, 0.28), p, { index: p.findIndex(n => n.parkour === true), jump: true, last: false }, 1)
    }
    // a hop from four blocks before the lip comes down in the gap
    expect(from(3.5).hop).to.equal(Infinity)
    // from further back it lands on the lip's side of it, and the jump follows
    const far = from(1.5)
    expect(far.hop).to.be.lessThan(Infinity)
    // and the jump it counts on is one the gate takes
    expect(kJumpLands(k, body(0.5, 1, 6.4, 0, 0.28), path[path.length - 1])).to.be.greaterThan(0)
  })

  it('knows ground the plan jumps from a jump', () => {
    const flat: GaitNode = { x: 0.5, y: 1, z: 4.5, parkour: true }
    expect(kWalkable(open, { x: 0.5, y: 1, z: 1.5 }, flat)).to.equal(true)
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 2; z++) put(x, 0, z)
      for (let x = -2; x <= 2; x++) for (let z = 5; z <= 9; z++) put(x, 0, z)
    })
    expect(kWalkable(k, { x: 0.5, y: 1, z: 1.5 }, { x: 0.5, y: 1, z: 5.5, parkour: true })).to.equal(false)
  })

  it('counts a hop that comes down short of a jump as a take-off for it, not its landing', () => {
    // platforms every other block, a head-hitter over each, open over the gaps
    // (arena tenways-headhitters): a jump from the lip past a platform's
    // head-hitter rises into the side of the next one
    const k = world(put => {
      for (let z = 1; z <= 15; z += 2) { put(0, 0, z); put(0, 3, z) }
      for (let z = -4; z <= 20; z++) for (let y = 0; y <= 3; y++) put(-1, y, z)
    })
    const path: GaitNode[] = []
    for (let z = 3; z <= 13; z += 2) path.push({ x: 0.5, y: 1, z: z + 0.5, parkour: true })
    const r = gaitSearch(k, body(0.5, 1, 1.56, 0, 0.2), path, { index: 0, jump: true, last: false, from: { x: 0.5, z: 1.5 } }, 1)
    expect(r.hop === Infinity || r.run < r.hop, JSON.stringify({ hop: r.hop, run: r.run })).to.equal(true)
    // from the lip the hop IS the jump: under the head-hitter, down on the next platform
    const lip = gaitSearch(k, body(0.5, 1, 2.2, 0, 0.25), path, { index: 0, jump: true, last: false, from: { x: 0.5, z: 1.5 } }, 1)
    expect(Math.min(lip.hop, lip.run)).to.be.lessThan(8)
  })

  it('keeps its feet to a ledge a hop would turn into a fall that hurts', () => {
    // three down after z 5: run off, the fall is three blocks and free; hopped
    // off, it is 4.25 from the hop's apex, and the server takes the speed
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 5; z++) for (let y = -3; y <= 0; y++) put(x, y, z)
      for (let x = -2; x <= 2; x++) for (let z = 6; z <= 30; z++) put(x, -3, z)
    })
    const path: GaitNode[] = [...line(3, 5)]
    for (let z = 6; z <= 20; z++) path.push({ x: 0.5, y: -2, z: z + 0.5 })
    const b = body(0.5, 1, 3.3, 0, 0.28)
    const r = gaitSearch(k, b, path, { index: 9, jump: false, last: false }, -2)
    expect(r.hop === Infinity || r.run < r.hop, JSON.stringify({ hop: r.hop, run: r.run })).to.equal(true)
    // the older servers leave the landing tick out and round up: there even the run-off hurts, and the hop is a hop
    const old = gaitSearch(k, b, path, { index: 9, jump: false, last: false, wholeFall: false }, -2)
    expect(old.hop).to.be.lessThan(Infinity)
  })

  it('gives no verdict when the work runs out before the feet had their say', () => {
    const path = line(1, 30)
    const r = gaitSearch(open, body(0.5, 1, 0.5, 0, 0.28), path, { index: 8, jump: false, last: false }, 1)
    expect(r.spent).to.equal(false)
  })

  it('takes a touchdown that ends past its edge for no landing', () => {
    // the floor ends at z 5: a body with the ground flag and its box wholly past the edge
    const k = world(put => { for (let x = -2; x <= 2; x++) for (let z = -4; z <= 4; z++) put(x, 0, z) })
    const on = body(0.5, 1, 4.5)
    const off = body(0.5, 1, 5.31)
    expect(k.carried(on)).to.equal(true)
    expect(k.carried(off)).to.equal(false)
    expect(kCaught(k, on)).to.equal(true)
    expect(kCaught(k, off)).to.equal(false)
    // (overhanging the edge it is carried still)
    expect(k.carried(body(0.5, 1, 5.29))).to.equal(true)
  })

  it('times a jump onto the last node of a path to standing in it', () => {
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 6; z++) put(x, 0, z)
      for (let x = -2; x <= 2; x++) for (let z = 10; z <= 16; z++) put(x, 0, z)
    })
    const node: GaitNode = { x: 0.5, y: 1, z: 10.5, parkour: true }
    // from the lip at a hop's speed the jump lands well past the node's box
    const b = body(0.5, 1, 6.9, 0, 0.3)
    const lands = kJumpLands(k, b, node)
    const stands = kJumpLands(k, b, node, true)
    expect(lands).to.be.greaterThan(0)
    expect(stands).to.be.greaterThan(lands)
  })

  it('does not hop into a one-wide corner it would overshoot', () => {
    // a tunnel two high running +z, turning +x at z 6, with a pocket a cell past the corner
    const k = world(put => {
      for (let x = -3; x <= 12; x++) {
        for (let z = -4; z <= 10; z++) {
          put(x, 0, z)
          put(x, 3, z)
          const leg1 = x === 0 && z >= -2 && z <= 7
          const leg2 = z === 6 && x >= 0 && x <= 12
          if (!leg1 && !leg2) { put(x, 1, z); put(x, 2, z) }
        }
      }
    })
    const path: GaitNode[] = [...line(1, 6)]
    for (let x = 1; x <= 10; x++) path.push({ x: x + 0.5, y: 1, z: 6.5 })
    // a hop from here comes down in the pocket, past the corner: the feet
    // get round it sooner, by more than the tick a hop is given for nothing
    for (const z of [2.5, 3.5]) {
      const p = path.filter(n => n.z > z + 0.35 || n.x > 1)
      const r = gaitSearch(k, body(0.5, 1, z, 0, 0.3), p, { index: 8, jump: false, last: false }, 1)
      expect(r.hop === Infinity || r.run <= r.hop - 1, `from z ${z}`).to.equal(true)
    }
  })
})
