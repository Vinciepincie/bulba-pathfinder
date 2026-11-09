// Improved nodes are pushed again; the solver skips stale entries on pop.
// Grow-only buffers avoid allocation while capacity is sufficient.
export class MinHeap {
  private nodes: Int32Array
  private fs: Float64Array
  private n = 0
  /** f of the most recently popped entry. */
  poppedF = 0

  constructor (capacity = 1024) {
    this.nodes = new Int32Array(capacity)
    this.fs = new Float64Array(capacity)
  }

  get size (): number {
    return this.n
  }

  isEmpty (): boolean {
    return this.n === 0
  }

  clear (): void {
    this.n = 0
  }

  push (node: number, f: number): void {
    if (this.n === this.nodes.length) this.grow()
    const nodes = this.nodes
    const fs = this.fs
    let i = this.n++
    while (i > 0) {
      const parent = (i - 1) >> 1
      if (fs[parent] <= f) break
      nodes[i] = nodes[parent]
      fs[i] = fs[parent]
      i = parent
    }
    nodes[i] = node
    fs[i] = f
  }

  /** Pops the min-f node id; poppedF holds its f. Heap must be non-empty. */
  pop (): number {
    const nodes = this.nodes
    const fs = this.fs
    const top = nodes[0]
    this.poppedF = fs[0]
    const n = --this.n
    if (n > 0) {
      const node = nodes[n]
      const f = fs[n]
      let i = 0
      const half = n >> 1
      while (i < half) {
        let child = (i << 1) + 1
        const right = child + 1
        if (right < n && fs[right] < fs[child]) child = right
        if (fs[child] >= f) break
        nodes[i] = nodes[child]
        fs[i] = fs[child]
        i = child
      }
      nodes[i] = node
      fs[i] = f
    }
    return top
  }

  private grow (): void {
    const cap = this.nodes.length * 2
    const nodes = new Int32Array(cap)
    const fs = new Float64Array(cap)
    nodes.set(this.nodes)
    fs.set(this.fs)
    this.nodes = nodes
    this.fs = fs
  }
}
