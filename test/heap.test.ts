// MinHeap unit tests: ordering vs a sort-based reference model, interleaved
// random operations, growth past the initial capacity, and poppedF pairing.
import { expect } from 'chai'
import { MinHeap } from '../src/heap.js'

/** Deterministic PRNG (mulberry32) so failures reproduce. */
function makeRng (seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6d2b79f5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

describe('MinHeap', () => {
  it('pops 10k randomized entries in ascending f order (duplicates allowed)', () => {
    const rng = makeRng(0xb01ba)
    const heap = new MinHeap()
    const N = 10000
    const fByNode = new Map<number, number>()
    const pushedFs: number[] = []
    for (let i = 0; i < N; i++) {
      // Quantize so duplicate f values definitely occur.
      const f = Math.floor(rng() * 500) / 8
      fByNode.set(i, f)
      pushedFs.push(f)
      heap.push(i, f)
    }
    expect(heap.size).to.equal(N)

    const reference = [...pushedFs].sort((a, b) => a - b)
    const poppedFs: number[] = []
    for (let i = 0; i < N; i++) {
      const node = heap.pop()
      // poppedF must be exactly the f this node was pushed with (nodes unique).
      expect(heap.poppedF).to.equal(fByNode.get(node))
      poppedFs.push(heap.poppedF)
    }
    expect(heap.size).to.equal(0)
    expect(heap.isEmpty()).to.equal(true)
    // Ascending order and exact multiset match with the sorted reference.
    expect(poppedFs).to.deep.equal(reference)
  })

  it('matches a sort-based reference model under interleaved random push/pop', () => {
    const rng = makeRng(1337)
    const heap = new MinHeap(4) // tiny initial capacity: interleaving exercises grow too
    const ref: Array<{ node: number, f: number }> = []
    let nextNode = 0

    for (let op = 0; op < 20000; op++) {
      if (ref.length === 0 || rng() < 0.55) {
        // Small f domain forces plenty of duplicate keys.
        const f = Math.floor(rng() * 64) / 4
        const node = nextNode++
        heap.push(node, f)
        ref.push({ node, f })
      } else {
        let minF = Infinity
        for (const e of ref) if (e.f < minF) minF = e.f
        const node = heap.pop()
        expect(heap.poppedF).to.equal(minF)
        // The popped node must be one of the reference entries tied at minF.
        const idx = ref.findIndex(e => e.f === minF && e.node === node)
        expect(idx, `popped node ${node} with f=${minF} not found in reference`).to.be.at.least(0)
        ref.splice(idx, 1)
      }
      expect(heap.size).to.equal(ref.length)
    }

    // Drain what's left; must come out as the reference sorted by f.
    const remaining = ref.map(e => e.f).sort((a, b) => a - b)
    const drained: number[] = []
    while (!heap.isEmpty()) {
      heap.pop()
      drained.push(heap.poppedF)
    }
    expect(drained).to.deep.equal(remaining)
  })

  it('grows past its initial capacity (5000 pushes into capacity 8)', () => {
    const rng = makeRng(7)
    const heap = new MinHeap(8)
    const fs: number[] = []
    for (let i = 0; i < 5000; i++) {
      const f = rng() * 1000
      fs.push(f)
      heap.push(i, f)
    }
    expect(heap.size).to.equal(5000)
    fs.sort((a, b) => a - b)
    for (let i = 0; i < 5000; i++) {
      heap.pop()
      expect(heap.poppedF).to.equal(fs[i])
    }
    expect(heap.isEmpty()).to.equal(true)
  })

  it('poppedF always matches the f the popped node was pushed with', () => {
    const heap = new MinHeap(2)
    const pairs: Array<[number, number]> = [
      [101, 3.5],
      [102, 0.25],
      [103, 9],
      [104, 0.25], // duplicate f, distinct node
      [105, -2],
      [106, 7.125]
    ]
    const fByNode = new Map(pairs)
    for (const [node, f] of pairs) heap.push(node, f)

    let prev = -Infinity
    for (let i = 0; i < pairs.length; i++) {
      const node = heap.pop()
      expect(fByNode.has(node)).to.equal(true)
      expect(heap.poppedF).to.equal(fByNode.get(node))
      expect(heap.poppedF).to.be.at.least(prev)
      prev = heap.poppedF
      fByNode.delete(node)
    }
    expect(fByNode.size).to.equal(0)
  })

  it('clear() empties the heap and it stays usable', () => {
    const heap = new MinHeap(4)
    heap.push(1, 5)
    heap.push(2, 1)
    heap.clear()
    expect(heap.isEmpty()).to.equal(true)
    heap.push(3, 2)
    heap.push(4, 0.5)
    expect(heap.pop()).to.equal(4)
    expect(heap.poppedF).to.equal(0.5)
    expect(heap.pop()).to.equal(3)
    expect(heap.poppedF).to.equal(2)
  })
})
