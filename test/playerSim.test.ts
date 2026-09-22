// PlayerSim (src/playerSim.ts) against prismarine-physics, tick for tick,
// bit for bit: random worlds of awkward shapes (slabs, stairs, fences, panes,
// trapdoors, walls, heads, pots, ladders, ice, slime), random bodies, random
// controls. The heading is derived from the yaw exactly as prismarine derives
// its sin/cos, so any difference at all is a porting error.
import { expect } from 'chai'
import { createRequire } from 'node:module'
import { Vec3 } from 'vec3'
import { mcData, Block, AIR, STONE } from './helpers/voxelWorld.js'
import { buildLut } from '../src/lut.js'
import { PlayerSim, simTables, newBody } from '../src/playerSim.js'
import type { SimControl } from '../src/playerSim.js'

const require = createRequire(import.meta.url)
const { Physics, PlayerState } = require('prismarine-physics')

/** Deterministic PRNG (mulberry32). */
function rng (seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6D2B79F5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

function statesOf (name: string, pred: (p: Record<string, unknown>) => boolean = () => true): number[] {
  const b = mcData.blocksByName[name]
  const out: number[] = []
  for (let s = b.minStateId as number; s <= (b.maxStateId as number); s++) {
    const props = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    if (props.waterlogged === true || props.waterlogged === 'true') continue
    if (pred(props)) out.push(s)
  }
  return out
}

const PALETTE: number[][] = [
  [STONE],
  statesOf('stone_slab'),
  statesOf('oak_stairs'),
  statesOf('oak_fence'),
  statesOf('glass_pane'),
  statesOf('oak_trapdoor'),
  statesOf('cobblestone_wall').slice(0, 40),
  statesOf('ladder'),
  statesOf('creeper_head'),
  [mcData.blocksByName.flower_pot.minStateId as number],
  [mcData.blocksByName.ice.minStateId as number],
  [mcData.blocksByName.blue_ice.minStateId as number],
  [mcData.blocksByName.slime_block.minStateId as number],
  statesOf('iron_bars'),
  statesOf('snow', p => Number(p.layers) <= 4),
  statesOf('oak_leaves').slice(0, 1)
]

interface TestWorld { get: (x: number, y: number, z: number) => number }

/** The same world as a flat state grid (the kernel's inline fast path). */
function gridOf (tw: TestWorld): { states: Int32Array, x0: number, y0: number, z0: number, w: number, h: number, l: number } {
  const g = { states: new Int32Array(17 * 15 * 17), x0: -8, y0: -2, z0: -8, w: 17, h: 15, l: 17 }
  for (let y = 0; y < g.h; y++) for (let z = 0; z < g.l; z++) for (let x = 0; x < g.w; x++) g.states[(y * g.l + z) * g.w + x] = tw.get(x + g.x0, y + g.y0, z + g.z0)
  return g
}

function makeWorld (seed: number): TestWorld {
  const r = rng(seed)
  const cells = new Map<string, number>()
  // a floor, then random scatter up to 5 above it
  for (let x = -6; x <= 6; x++) {
    for (let z = -6; z <= 6; z++) {
      cells.set(`${x},0,${z}`, r() < 0.85 ? STONE : AIR)
      for (let y = 1; y <= 5; y++) {
        if (r() < 0.16) {
          const group = PALETTE[Math.floor(r() * PALETTE.length)]
          cells.set(`${x},${y},${z}`, group[Math.floor(r() * group.length)])
        }
      }
    }
  }
  return { get: (x, y, z) => (x < -8 || x > 8 || z < -8 || z > 8 || y < -2 || y > 12) ? -1 : (cells.get(`${x},${y},${z}`) ?? AIR) }
}

describe('PlayerSim vs prismarine-physics', function () {
  this.timeout(60000)
  const lut = buildLut({ registry: mcData, version: mcData.version.minecraftVersion } as never, {
    lutFingerprint: () => 'sim', climbables: new Set(), carpets: new Set(), blocksToAvoid: new Set(),
    liquids: new Set(), fences: new Set(), openable: new Set(), doors: new Set(), useBubbleColumns: false, allowParkourExtended: false
  } as never)
  const tables = simTables(lut, mcData.blocksArray as never, s => {
    const p = (Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    return p.waterlogged === true || p.waterlogged === 'true'
  })

  it('matches every tick on random worlds and controls', () => {
    let ticks = 0
    for (let seed = 1; seed <= 60; seed++) {
      const tw = makeWorld(seed)
      const pworld = {
        getBlock: (pos: Vec3) => {
          const x = Math.floor(pos.x); const y = Math.floor(pos.y); const z = Math.floor(pos.z)
          const s = tw.get(x, y, z)
          if (s < 0) return null
          const b = Block.fromStateId(s, 0) as { position?: Vec3 }
          b.position = new Vec3(x, y, z)
          return b
        }
      }
      const physics = Physics(mcData, pworld)
      physics.playerHalfWidth = 0.30001
      physics.playerHeight = 1.80001
      const sim = new PlayerSim({ ...tables, stateAt: tw.get }, 0.30001, 1.80001)
      const gsim = new PlayerSim({ ...tables, stateAt: tw.get, grid: gridOf(tw) }, 0.30001, 1.80001)
      const r = rng(seed * 7919)
      for (let body = 0; body < 6; body++) {
        const x0 = -3 + r() * 6
        const z0 = -3 + r() * 6
        const y0 = 1 + Math.floor(r() * 5) + 0.2
        const fake = {
          version: mcData.version.minecraftVersion,
          entity: { position: new Vec3(x0, y0, z0), velocity: new Vec3(0, 0, 0), onGround: false, isInWater: false, isInLava: false, isInWeb: false, isCollidedHorizontally: false, isCollidedVertically: false, elytraFlying: false, attributes: {}, effects: {}, yaw: 0, pitch: 0 },
          jumpTicks: 0, jumpQueued: false, fireworkRocketDuration: 0, inventory: { slots: [] }
        }
        const ps = new PlayerState(fake, { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false })
        const kb = newBody(x0, y0, z0)
        const gb = newBody(x0, y0, z0)
        let yaw = r() * Math.PI * 2
        const ctl: SimControl = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false, hx: 0, hz: 0 }
        for (let t = 0; t < 120; t++) {
          if (t % 7 === 0) {
            ctl.forward = r() < 0.75
            ctl.back = !ctl.forward && r() < 0.3
            ctl.left = r() < 0.15
            ctl.right = !ctl.left && r() < 0.15
            ctl.jump = r() < 0.4
            ctl.sprint = ctl.forward && r() < 0.7
            ctl.sneak = r() < 0.12
          }
          if (r() < 0.2) yaw += (r() - 0.5) * 1.2
          ps.yaw = yaw
          Object.assign(ps.control, { forward: ctl.forward, back: ctl.back, left: ctl.left, right: ctl.right, jump: ctl.jump, sprint: ctl.sprint, sneak: ctl.sneak })
          ctl.hx = -Math.sin(Math.PI - yaw)
          ctl.hz = Math.cos(Math.PI - yaw)
          physics.simulatePlayer(ps, pworld)
          sim.step(kb, ctl)
          gsim.step(gb, ctl)
          expect(gb, `grid path seed ${seed} body ${body} t ${t}`).to.deep.equal(kb)
          ticks++
          if (ps.isInWater === true || ps.isInLava === true || kb.inLiquid) {
            expect(kb.inLiquid, `liquid flag seed ${seed} body ${body} t ${t}`).to.equal(ps.isInWater === true || ps.isInLava === true)
            break
          }
          const where = `seed ${seed} body ${body} tick ${t}`
          if (kb.x !== ps.pos.x || kb.y !== ps.pos.y || kb.z !== ps.pos.z || kb.vx !== ps.vel.x || kb.vy !== ps.vel.y || kb.vz !== ps.vel.z) {
            const near: string[] = []
            for (let yy = Math.floor(kb.y) - 1; yy <= Math.floor(kb.y) + 2; yy++) {
              for (let zz = Math.floor(kb.z) - 1; zz <= Math.floor(kb.z) + 1; zz++) {
                for (let xx = Math.floor(kb.x) - 1; xx <= Math.floor(kb.x) + 1; xx++) {
                  const s = tw.get(xx, yy, zz)
                  if (s > 0) near.push(`${xx},${yy},${zz}:${(Block.fromStateId(s, 0) as { name: string }).name} ${JSON.stringify((Block.fromStateId(s, 0) as { shapes: number[][] }).shapes)}`)
                }
              }
            }
            console.log(`MISMATCH ${where} ctl ${JSON.stringify(ctl)}\n kernel ${kb.x},${kb.y},${kb.z} v ${kb.vx},${kb.vy},${kb.vz} g ${kb.onGround} ch ${kb.collidedH}\n prism  ${ps.pos.x},${ps.pos.y},${ps.pos.z} v ${ps.vel.x},${ps.vel.y},${ps.vel.z} g ${ps.onGround} ch ${ps.isCollidedHorizontally}\n ${near.join('\n ')}`)
          }
          expect(kb.x, `x ${where}`).to.equal(ps.pos.x)
          expect(kb.y, `y ${where}`).to.equal(ps.pos.y)
          expect(kb.z, `z ${where}`).to.equal(ps.pos.z)
          expect(kb.vx, `vx ${where}`).to.equal(ps.vel.x)
          expect(kb.vy, `vy ${where}`).to.equal(ps.vel.y)
          expect(kb.vz, `vz ${where}`).to.equal(ps.vel.z)
          expect(kb.onGround, `onGround ${where}`).to.equal(ps.onGround)
          expect(kb.collidedH, `collidedH ${where}`).to.equal(ps.isCollidedHorizontally)
          if (kb.y < -3) break
        }
      }
    }
    expect(ticks).to.be.greaterThan(20000)
  })
})
