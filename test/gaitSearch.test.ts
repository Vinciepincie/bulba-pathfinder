// gaitSearch (src/gaitSearch.ts): the schedules of jumps, flown in the kernel.
// What it promises the executor: a jump it says to take reaches the goal
// sooner than keeping the feet down does, comes down on the path, and is
// handed over as the flight that was judged; a tick on the ground it says to
// take is handed over as it was run; and a body driven by those two alone
// crosses what the path crosses.
import { expect } from 'chai'
import { mcData, Block, AIR, STONE } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { simTables, newBody, copyBody, PlayerSim } from '../src/playerSim.js'
import type { SimBody, SimControl } from '../src/playerSim.js'
import { gaitSearch, kWalkable, onLegPast, fallAim, strafeYaw, EASE_NEVER } from '../src/gaitSearch.js'
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
function line (z0: number, z1: number, y = 1): GaitNode[] {
  const out: GaitNode[] = []
  for (let z = z0; z <= z1; z++) out.push({ x: 0.5, y, z: z + 0.5 })
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

/**
 * A body driven down `path` by the search alone, the way the executor takes
 * its word: a jump replayed look for look, a tick on the ground taken as it
 * was run, nodes retired as the executor retires them. Stops at the path's
 * end, when the kernel has no schedule, or when the body leaves the ground
 * it was on for good (`fell`).
 */
function drive (k: PlayerSim, b0: SimBody, path: GaitNode[], strafe = false): { b: SimBody, ticks: number, jumps: number, at: number, fell: boolean, lost: boolean } {
  const b = copyBody(newBody(0, 0, 0), b0)
  const c: SimControl = { forward: true, back: false, left: false, right: false, jump: false, sprint: true, sneak: false, hx: 0, hz: 1 }
  const low = Math.min(b.y, ...path.map(n => n.y)) - 1.5
  let i = 0
  let jumps = 0
  let flight: number[] = []
  let ease = 99
  let f = 0
  let flown = false
  let from: { x: number, z: number } | null = { x: b.x, z: b.z }
  let wasGround = true
  for (let t = 0; t < 400; t++) {
    // retire
    for (;;) {
      if (i >= path.length) return { b, ticks: t, jumps, at: i, fell: false, lost: false }
      const n = path[i]
      const boxed = Math.abs(n.x - b.x) <= 0.35 && Math.abs(n.z - b.z) <= 0.35 && Math.abs(n.y - b.y) < (flown ? 1.45 : 1)
      let ok = n.parkour === true ? boxed && b.y - n.y <= 0.5 : boxed
      if (!ok && n.parkour === true && b.onGround && !wasGround && from !== null && i < path.length - 1 &&
          Math.abs(n.y - b.y) < 1 &&
          (Math.hypot(n.x - b.x, n.z - b.z) <= 1 || (Math.hypot(n.x - b.x, n.z - b.z) <= 2.5 && kWalkable(k, b, n)))) {
        const fx = n.x - from.x
        const fz = n.z - from.z
        ok = fx * fx + fz * fz > 1 && (b.x - n.x) * fx + (b.z - n.z) * fz > 0
      }
      // (on the leg to the next node, past this one)
      if (!ok && n.parkour !== true && b.onGround && i < path.length - 1 &&
          Math.abs(n.y - b.y) < (flown ? 1.45 : 1) && onLegPast(b.x, b.z, n, path[i + 1])) ok = true
      if (ok && i === path.length - 1 && (!b.onGround || Math.floor(b.x) !== Math.floor(n.x) || Math.floor(b.z) !== Math.floor(n.z))) ok = false
      if (!ok) break
      from = { x: n.x, z: n.z }
      i++
    }
    wasGround = b.onGround
    if (b.onGround && f >= flight.length) {
      flown = false
      // (ten blocks of path, as PhysicsSim.gaitDrive asks)
      const rest: GaitNode[] = []
      let reach = 0
      for (let j = i; j < path.length && reach < 10; j++) {
        const p = j === i ? b : path[j - 1]
        reach += Math.hypot(path[j].x - p.x, path[j].z - p.z)
        rest.push(path[j])
      }
      const r = gaitSearch(k, b, rest, { index: rest.length - 1, last: i + rest.length === path.length, from, strafe })
      if (r.hop !== Infinity && r.run === Infinity) {
        flight = r.flight
        ease = r.ease
        f = 0
        flown = true
        jumps++
      } else if (r.run !== Infinity && r.step !== null) {
        flight = []
        f = 0
        c.hx = -Math.sin(r.step.yaw)
        c.hz = -Math.cos(r.step.yaw)
        c.jump = false
        c.forward = true
        c.sprint = r.step.sprint
        c.left = r.step.strafe
        k.step(b, c)
        if (b.y < low) return { b, ticks: t, jumps, at: i, fell: true, lost: false }
        continue
      } else {
        return { b, ticks: t, jumps, at: i, fell: false, lost: true }
      }
    }
    if (f < flight.length) {
      c.hx = -Math.sin(flight[f])
      c.hz = -Math.cos(flight[f])
      c.jump = f === 0
      c.sprint = true
      c.forward = f < ease
      c.left = strafe && f > 0 && f < ease
      f++
    } else {
      // (in the air past the flight's end, or fallen off: nothing held but forward — nor that, eased)
      c.jump = false
      c.left = false
      c.forward = !(flown && !b.onGround && ease < flight.length)
      // (walked off a ledge: steered as the executor steers the fall)
      if (!flown && !b.onGround && i < path.length) {
        const to = path[fallAim(path, i, b.x, b.z, b.vx, b.vz)]
        const d = Math.hypot(to.x - b.x, to.z - b.z)
        if (d > 1e-9) { c.hx = (to.x - b.x) / d; c.hz = (to.z - b.z) / d }
        c.sprint = true
      }
    }
    k.step(b, c)
    if (b.y < low) return { b, ticks: t, jumps, at: i, fell: true, lost: false }
  }
  return { b, ticks: 400, jumps, at: i, fell: false, lost: true }
}

describe('gaitSearch', function () {
  this.timeout(30000)

  const open = world(put => { for (let x = -3; x <= 3; x++) for (let z = -4; z <= 44; z++) put(x, 0, z) })

  it('hops over open ground, and sooner there than the feet get', () => {
    const path = line(1, 30)
    const b = body(0.5, 1, 0.5, 0, 0.28)
    const r = gaitSearch(open, b, path, { index: 8, last: false })
    expect(r.hop).to.be.lessThan(Infinity)
    expect(r.run).to.equal(Infinity)
    expect(r.hop).to.be.lessThan(runTicks(open, b, path, 8))
  })

  it('hands the hop over as flown: one look a tick, from the take-off to the touchdown', () => {
    const path = line(1, 30)
    const r = gaitSearch(open, body(0.5, 1, 0.5, 0, 0.28), path, { index: 8, last: false })
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

  it('hands a tick on the ground over as it was run', () => {
    // the path ends a block on: any hop comes down past it and walks back, the feet's schedule it is
    const r = gaitSearch(open, body(0.5, 1, 1.5, 0, 0.28), line(2, 2), { index: 0, last: true })
    expect(r.run).to.be.lessThan(r.hop)
    expect(r.step).to.not.equal(null)
    // straight down the line, sprinting, no strafe asked for
    expect(Math.abs(Math.sin((r.step as { yaw: number }).yaw))).to.be.lessThan(1e-9)
    expect(r.step).to.include({ sprint: true, strafe: false })
  })

  it('flies and runs with the 45° strafe: sooner there, the flight handed over with its looks', () => {
    const path = line(1, 30)
    const b = body(0.5, 1, 0.5, 0, 0.28)
    const plain = gaitSearch(open, b, path, { index: 8, last: false })
    const r = gaitSearch(open, b, path, { index: 8, last: false, strafe: true })
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

  it('eases a jump that would come down past the end of the path: forward let go, it lands on it', () => {
    // the floor and the path end four blocks on: a hop at full stretch lands past both
    const k = world(put => { for (let x = -3; x <= 3; x++) for (let z = -4; z <= 4; z++) put(x, 0, z) })
    const path = line(2, 4)
    const b = body(0.5, 1, 1.5, 0, 0.28)
    const r = gaitSearch(k, b, path, { index: 2, last: true })
    expect(r.hop).to.be.lessThan(Infinity)
    expect(r.ease).to.be.lessThan(EASE_NEVER)
    expect(r.ease).to.be.lessThan(r.flight.length)
    // driven, the body comes down in the last node's box and stays on the floor
    const d = drive(k, b, path)
    expect(d.fell).to.equal(false)
    expect(d.at).to.equal(path.length)
    expect(Math.abs(d.b.z - 4.5)).to.be.at.most(0.35)
    expect(d.b.y).to.equal(1)
  })

  it('passes a node from the leg beyond it, not only from its box', () => {
    // level with the node or past it, within a box's half-width of the line to the next
    const n = { x: 0.5, z: 3.5 }
    const m = { x: 0.5, z: 4.5 }
    expect(onLegPast(0.8, 3.9, n, m)).to.equal(true)
    expect(onLegPast(0.8, 6.2, n, m)).to.equal(true)
    expect(onLegPast(0.8, 3.4, n, m), 'short of it').to.equal(false)
    expect(onLegPast(0.9, 3.9, n, m), 'off the line').to.equal(false)
    // a body that crossed a node's corner between two ticks is not sent back for it
    const path = line(1, 12)
    const r = gaitSearch(open, body(0.84, 1, 2.9, 0, 0.3), path.slice(2), { index: 8, last: false })
    expect(Math.min(r.hop, r.run)).to.be.lessThan(runTicks(open, body(0.5, 1, 2.9, 0, 0.3), path.slice(2), 8) + 1)
  })

  it('arrives at the last node on the ground: a hop that overflies it is no arrival', () => {
    const path = line(1, 5)
    const r = gaitSearch(open, body(0.5, 1, 0.5, 0, 0.28), path, { index: 4, last: true })
    // whatever it chooses, the time is a body standing in the node's box
    expect(Math.min(r.hop, r.run)).to.be.greaterThan(10)
    const d = drive(open, body(0.5, 1, 0.5, 0, 0.28), path)
    expect(d.at).to.equal(path.length)
    expect(d.b.onGround).to.equal(true)
    expect(Math.abs(d.b.z - 5.5)).to.be.at.most(0.35)
  })

  it('jumps a gap on the tick that lands it: a body driven by the search alone crosses', () => {
    // a run of six, a gap of three, the far side
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 6; z++) put(x, 0, z)
      for (let x = -2; x <= 2; x++) for (let z = 10; z <= 20; z++) put(x, 0, z)
    })
    const path: GaitNode[] = [...line(1, 6), { x: 0.5, y: 1, z: 10.5, parkour: true }, ...line(11, 14)]
    // from four blocks before the lip a hop NOW comes down in the gap: not on offer
    const early = gaitSearch(k, body(0.5, 1, 3.5, 0, 0.28), path.filter(n => n.z > 3.85), { index: 3, last: false })
    expect(early.hop).to.equal(Infinity)
    expect(early.run).to.be.lessThan(Infinity)
    for (const z of [0.5, 1.5, 2.5, 3.5]) {
      const d = drive(k, body(0.5, 1, z, 0, z === 0.5 ? 0 : 0.28), path.filter(n => n.z > z + 0.35))
      expect(d.fell, `from z ${z}`).to.equal(false)
      expect(d.lost, `from z ${z}`).to.equal(false)
      expect(d.b.z, `from z ${z}`).to.be.greaterThan(14)
    }
  })

  it('jumps up a rise, and hops on from the top of it', () => {
    // a step up a full block six blocks on
    const k = world(put => {
      for (let x = -2; x <= 2; x++) for (let z = -4; z <= 30; z++) put(x, 0, z)
      for (let x = -2; x <= 2; x++) for (let z = 7; z <= 30; z++) put(x, 1, z)
    })
    const path: GaitNode[] = [...line(1, 6), ...line(7, 16, 2)]
    const d = drive(k, body(0.5, 1, 0.5), path)
    expect(d.lost).to.equal(false)
    expect(d.at).to.equal(path.length)
    expect(d.b.y).to.equal(2)
    // sooner than the feet and one jump at the wall do
    expect(d.jumps).to.be.greaterThan(2)
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

  it('crosses platforms under head-hitters, a jump from each lip', () => {
    // platforms every other block, a head-hitter over each, open over the gaps
    // (arena tenways-headhitters): a jump from the lip past a platform's
    // head-hitter rises into the side of the next one
    const k = world(put => {
      for (let z = 1; z <= 15; z += 2) { put(0, 0, z); put(0, 3, z) }
      for (let z = -4; z <= 20; z++) for (let y = 0; y <= 3; y++) put(-1, y, z)
    })
    const path: GaitNode[] = []
    for (let z = 3; z <= 13; z += 2) path.push({ x: 0.5, y: 1, z: z + 0.5, parkour: true })
    const d = drive(k, body(0.5, 1, 1.5), path)
    expect(d.fell).to.equal(false)
    expect(d.lost).to.equal(false)
    expect(d.at).to.equal(path.length)
    // five platforms on: a short hop each (a jump risen into a head-hitter's side hangs there twelve ticks)
    expect(d.ticks).to.be.lessThan(45)
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
    const r = gaitSearch(k, b, path, { index: 9, last: false })
    expect(r.run, JSON.stringify({ hop: r.hop, run: r.run })).to.be.lessThan(Infinity)
    // and driven, the body runs off the ledge: it comes down with its speed
    const d = drive(k, b, path)
    expect(d.lost).to.equal(false)
    expect(d.b.y).to.equal(-2)
  })

  it('takes a touchdown that ends past its edge for no landing', () => {
    // the floor ends at z 5: a body with the ground flag and its box wholly past the edge
    const k = world(put => { for (let x = -2; x <= 2; x++) for (let z = -4; z <= 4; z++) put(x, 0, z) })
    expect(k.carried(body(0.5, 1, 4.5))).to.equal(true)
    expect(k.carried(body(0.5, 1, 5.31))).to.equal(false)
    // (overhanging the edge it is carried still)
    expect(k.carried(body(0.5, 1, 5.29))).to.equal(true)
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
      const r = gaitSearch(k, body(0.5, 1, z, 0, 0.3), p, { index: 8, last: false })
      expect(r.hop === Infinity || r.run !== Infinity, `from z ${z}`).to.equal(true)
    }
  })

  it('under a low roof with a hole in the floor: a verdict, for a fraction of the work', () => {
    // a one-wide tunnel two high running +z, a hole one deep at z 8, the path a jump over it
    const k = world(put => {
      for (let x = -2; x <= 2; x++) {
        for (let z = -4; z <= 24; z++) {
          if (!(x === 0 && z === 8)) put(x, 0, z)
          put(x, -1, z)
          put(x, 3, z)
          if (x !== 0) { put(x, 1, z); put(x, 2, z) }
        }
      }
    })
    const path: GaitNode[] = [...line(1, 7), { x: 0.5, y: 1, z: 9.5, parkour: true }, ...line(10, 14)]
    // (the hops the roof clips, three ticks each, were flown four deep and
    // then run into the hole to the end of the budget, every one of them)
    const d = drive(k, body(0.5, 1, 0.5), path)
    expect(d.lost, 'lost').to.equal(false)
    expect(d.fell, 'fell').to.equal(false)
    expect(d.at).to.equal(path.length)
    for (const z of [0.5, 2.5, 4.5]) {
      const p = path.filter(n => n.z > z + 0.35)
      const r = gaitSearch(k, body(0.5, 1, z, 0, 0.15), p, { index: Math.min(p.length - 1, 9), last: false })
      expect(r.hop !== Infinity || r.run !== Infinity, `from z ${z}`).to.equal(true)
      expect(r.spent, `from z ${z}`).to.equal(false)
      expect(r.work, `from z ${z}`).to.be.lessThan(4000)
    }
  })
})
