// buildSnapshot's direct bit-array path (snapshot.ts noSpanBits) against the
// generic getBlockStateId path over the SAME real prismarine-chunk column:
// indirect palettes of several widths, a direct-palette section (more than
// 256 distinct states), uniform sections and missing ones. The two builds
// must be byte-identical — the fast path exists only to be faster.
import { describe, it } from 'mocha'
import { expect } from 'chai'
import { createRequire } from 'node:module'
import { buildSnapshot } from '../src/snapshot.js'
import { Movements } from '../src/movements.js'
import { makeFakeBot, VoxelWorld, TEST_VERSION, mcData, lutFor, applyProfile } from './helpers/voxelWorld.js'

const require2 = createRequire(import.meta.url)
const ChunkColumn = require2('prismarine-chunk')(TEST_VERSION) as new (o: { minY: number, worldHeight: number }) => {
  setBlockStateId (pos: { x: number, y: number, z: number }, id: number): void
  getBlockStateId (pos: { x: number, y: number, z: number }): number
  sections: unknown[]
  minY: number
}

function mulberry32 (seed: number): () => number {
  let a = seed >>> 0
  return function () {
    a |= 0
    a = (a + 0x6D2B79F5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

describe('snapshot: direct palette unpacking', () => {
  it('matches the generic per-cell path byte for byte on a real chunk column', () => {
    const minY = -64
    const worldHeight = 384
    const col = new ChunkColumn({ minY, worldHeight })
    const rand = mulberry32(7)
    const maxState = Math.max(...mcData.blocksArray.map(b => b.maxStateId as number))

    // Section 0..15 of the box (y 0..63) gets four flavours:
    //  y 0-15  : indirect, ~12 states (4-bit palette)
    //  y 16-31 : indirect, ~60 states (6-bit palette)
    //  y 32-47 : direct (300+ distinct states → beyond the 8-bit palette cap)
    //  y 48-63 : left untouched (uniform air), and y 64.. is outside the box
    const few = [0, 1, 9, 10, 79, 80, 100, 101, 102, 103, 104, 105].map(i => Math.min(i, maxState))
    const many = Array.from({ length: 60 }, (_, i) => Math.min(i * 37, maxState))
    for (let y = 0; y < 48; y++) {
      for (let z = 0; z < 16; z++) {
        for (let x = 0; x < 16; x++) {
          let id: number
          if (y < 16) id = few[Math.floor(rand() * few.length)]
          else if (y < 32) id = many[Math.floor(rand() * many.length)]
          else id = Math.min(Math.floor(rand() * 400), maxState)
          col.setBlockStateId({ x, y, z }, id)
        }
      }
    }
    // A carpet, a slab, a fence, water: classes the LUT distinguishes.
    const by = (name: string): number => (mcData.blocksByName[name] as { minStateId: number }).minStateId
    col.setBlockStateId({ x: 3, y: 5, z: 3 }, by('white_carpet'))
    col.setBlockStateId({ x: 4, y: 5, z: 3 }, by('stone_slab'))
    col.setBlockStateId({ x: 5, y: 5, z: 3 }, by('oak_fence'))
    col.setBlockStateId({ x: 6, y: 5, z: 3 }, by('water'))

    const world = new VoxelWorld({ x0: 0, y0: 0, z0: 0, x1: 15, y1: 63, z1: 15 })
    const bot = makeFakeBot(world)
    const movements = applyProfile(new Movements(bot as never))
    movements.useBubbleColumns = true
    const lut = lutFor(bot, movements)

    // Fast build: the real column, sections and all.
    bot.world = { getColumn: () => col }
    const fast = buildSnapshot(bot as never, lut, { x0: -3, y0: -2, z0: -3, x1: 18, y1: 66, z1: 18 }, true, 1)
    // Reference build: the same column behind a shim that hides `sections`,
    // so every cell goes through getBlockStateId.
    bot.world = { getColumn: () => ({ getBlockStateId: (p: { x: number, y: number, z: number }) => col.getBlockStateId(p) }) }
    const slow = buildSnapshot(bot as never, lut, { x0: -3, y0: -2, z0: -3, x1: 18, y1: 66, z1: 18 }, true, 2)

    // The fast path must actually have been taken for the packed sections.
    const secs = col.sections as Array<{ data: { data?: { data?: unknown, valuesPerLong?: number } } }>
    expect(secs[(0 - minY) >> 4].data.data?.data).to.be.instanceOf(Uint32Array)
    expect(secs[(32 - minY) >> 4].data.data?.data).to.be.instanceOf(Uint32Array)

    expect(Buffer.compare(Buffer.from(fast.flags), Buffer.from(slow.flags))).to.equal(0)
    expect(Buffer.compare(Buffer.from(fast.heights), Buffer.from(slow.heights))).to.equal(0)
    expect(fast.states).to.not.equal(null)
    expect(Buffer.compare(Buffer.from(fast.states!.buffer), Buffer.from(slow.states!.buffer))).to.equal(0)
    if (fast.special !== null || slow.special !== null) {
      expect(fast.special).to.not.equal(null)
      expect(slow.special).to.not.equal(null)
      expect(Buffer.compare(Buffer.from(fast.special!), Buffer.from(slow.special!))).to.equal(0)
    }
    // And the cells are not trivially all air: the carpet/slab/fence/water cells differ from stone.
    const idx = (x: number, y: number, z: number): number => fast.index(x, y, z)
    expect(fast.states![idx(3, 5, 3)]).to.equal(by('white_carpet'))
    expect(fast.states![idx(6, 5, 3)]).to.equal(by('water'))
    expect(fast.heights[idx(4, 5, 3)]).to.not.equal(fast.heights[idx(5, 5, 3)])
  })
})
