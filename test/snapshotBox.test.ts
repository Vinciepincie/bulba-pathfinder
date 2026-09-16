// computeBox under the memory cap. A long narrow start/goal hull used to be
// shrunk on both horizontal axes by 5% of the x extent, which turned the short
// axis inside out; the negative cell count threw from SharedArrayBuffer inside
// the physics tick and killed the racer (arena spiral3-d, a racer left at
// world spawn 600 blocks from its goal).
import { describe, it } from 'mocha'
import { expect } from 'chai'
import { computeBox } from '../src/snapshot.js'

const bot = { game: { minY: -64, height: 384 } } as never

function mulberry32 (seed: number): () => number {
  let a = seed >>> 0
  return function () {
    a = (a + 0x6D2B79F5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

describe('computeBox cap', () => {
  it('never inverts an axis or exceeds the cap, and always holds the start', () => {
    const rnd = mulberry32(7)
    for (let i = 0; i < 20000; i++) {
      const span = [20, 200, 2000, 20000][i % 4]
      const s = { x: Math.round((rnd() - 0.5) * span), y: Math.round(rnd() * 380 - 64), z: Math.round((rnd() - 0.5) * span) }
      const t = { x: Math.round((rnd() - 0.5) * span), y: Math.round(rnd() * 380 - 64), z: Math.round((rnd() - 0.5) * span * (i % 3 === 0 ? 0.01 : 1)) }
      const grow = [1, 1.8, 3.24, 5.832, 10.4976, 18.9][i % 6]
      const cap = [8_000_000, 500_000][i % 2]
      const b = computeBox(bot, s, [t], [-1, 40][i % 2], grow, cap)
      const w = b.x1 - b.x0 + 1; const h = b.y1 - b.y0 + 1; const l = b.z1 - b.z0 + 1
      expect(w > 0 && h > 0 && l > 0, JSON.stringify({ s, t, grow, cap, b })).to.equal(true)
      expect(w * h * l, JSON.stringify({ s, t, grow, cap, b })).to.be.at.most(cap)
      expect(s.x >= b.x0 && s.x <= b.x1 && s.z >= b.z0 && s.z <= b.z1 && s.y >= b.y0 && s.y <= b.y1).to.equal(true)
    }
  })

  it('keeps the arena spiral3-d crash case valid through every growth step', () => {
    let grow = 1
    for (let step = 0; step < 8; step++, grow *= 1.8) {
      const b = computeBox(bot, { x: -6, y: 194, z: -6 }, [{ x: -583, y: 201, z: 208 }], -1, grow, 8_000_000)
      const n = (b.x1 - b.x0 + 1) * (b.y1 - b.y0 + 1) * (b.z1 - b.z0 + 1)
      expect(n).to.be.greaterThan(0)
      expect(n).to.be.at.most(8_000_000)
    }
  })

  it('leaves a box under the cap untouched', () => {
    const b = computeBox(bot, { x: 0, y: 64, z: 0 }, [{ x: 40, y: 64, z: 10 }], -1, 1, 8_000_000)
    // margin 0.3·d + 20 with d = hypot(40, 10)
    const m = Math.ceil(Math.hypot(40, 10) * 0.3 + 20)
    expect(b).to.deep.equal({ x0: -m, y0: 48, z0: -m, x1: 40 + m, y1: 80, z1: 10 + m })
  })
})
