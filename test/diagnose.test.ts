// The arena's auto-debug, tested offline.
//
// The fixture is a trimmed copy of a real losing run (route `simple2`, 2b2t
// spawn): both engines planned the identical 70-node path, upstream walked it
// in 19.3 s, ours wedged 29 blocks short and sat there until the timeout. That
// is the exact shape the auto-debug exists to explain, so it is what the
// renderer is held to.
import { strict as assert } from 'node:assert'
import { analyse, focusPoints, judge, renderNotes } from '../bench/arena/diagnose.js'
import type { RouteReport } from '../bench/arena/report.js'
import type { Impl, RunResult, WorldProbe } from '../bench/arena/runner.js'
import type { Route } from '../bench/arena/routes.js'

const ROUTE: Route = {
  id: 'simple2',
  name: 'simple2',
  scenario: 'mixed',
  start: [2, 132, 255],
  end: [-17, 132, 184]
}

/** A path both engines agreed on, walked from the start to the goal. */
const PATH: Array<[number, number, number]> = [
  [-1.5, 132, 255.5], [-1.5, 132, 252.5], [-1.5, 132, 251.5],
  [-7.5, 132, 213.5], [-8.5, 132, 210.5], [-9.5, 132, 207.5],
  [-16.5, 132, 184.5]
]

type Sample = [number, number, number, number, number]

/**
 * A trace with the density a real run has (one sample per 200 ms). The stray
 * detector compares corridors, so a sparse fixture would report a divergence
 * on nothing more than the gaps between waypoints.
 */
function traceAlong (waypoints: Array<[number, number, number]>, msPerLeg = 4000): Sample[] {
  const out: Sample[] = []
  let ms = 0
  for (let i = 0; i < waypoints.length - 1; i++) {
    const [ax, ay, az] = waypoints[i]
    const [bx, by, bz] = waypoints[i + 1]
    const steps = Math.max(1, Math.round(msPerLeg / 200))
    for (let s = 0; s < steps; s++) {
      const t = s / steps
      out.push([ms, ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t, 0])
      ms += 200
    }
  }
  const [lx, ly, lz] = waypoints[waypoints.length - 1]
  out.push([ms, lx, ly, lz, 0])
  return out
}

/** The corridor both bots walked, start to goal. */
const CORRIDOR: Array<[number, number, number]> = [
  [2.5, 132, 255.5], [-1.5, 132, 240], [-7.5, 132, 213], [-9.5, 132, 207], [-16.5, 132, 184.4]
]

function run (impl: Impl, over: Partial<RunResult> = {}): RunResult {
  return {
    impl,
    routeId: 'simple2',
    outcome: 'arrived',
    promiseOutcome: 'arrived',
    wallMs: 19317,
    firstSolveMs: 492,
    firstMoveMs: 700,
    solveMsTotal: 492,
    solves: 1,
    visitedNodes: 900,
    generatedNodes: 1200,
    firstPathNodes: PATH.length,
    firstPathCost: 120,
    replans: 0,
    travelled: 116,
    jumps: 4,
    damage: 0,
    deaths: 0,
    worstStallMs: 944,
    stalls: [],
    endDistance: 0.1,
    endPos: [-16.5, 132, 184.4],
    startedAt: 0,
    statuses: { success: 1 },
    resetReasons: {},
    gameMode: 'survival',
    firstPath: PATH,
    firstPathParkour: [1],
    trace: traceAlong(CORRIDOR),
    ...over
  }
}

/** Ours: wedged at the same spot for 12.9 s with 34 nodes still queued. */
const OURS = run('bulba', {
  outcome: 'timeout',
  promiseOutcome: 'timeout',
  wallMs: 25013,
  firstSolveMs: 185,
  travelled: 85.2,
  replans: 3,
  worstStallMs: 12886,
  endDistance: 29.2,
  endPos: [-7.5, 131.4, 212.3],
  statuses: { success: 3, partial: 2 },
  resetReasons: { stuck: 2, goal_updated: 1 },
  // Walks the shared corridor as far as the wedge, then sits there.
  trace: [
    ...traceAlong([CORRIDOR[0], CORRIDOR[1], [-7.5, 131.4, 212.3]]),
    ...Array.from({ length: 60 }, (_, i): Sample => [12000 + i * 200, -7.5, 131.4, 212.3, 34])
  ],
  stalls: [{
    atMs: 12000,
    stalledMs: 12886,
    pos: [-7.5, 131.4, 212.3],
    remaining: 34,
    ahead: [
      { x: -8, y: 132, z: 210, parkour: true, cost: 3.2 },
      { x: -9, y: 132, z: 207, parkour: false, cost: 1 }
    ]
  }]
})

const UPSTREAM = run('upstream')

const report = (over: Partial<RouteReport> = {}): RouteReport => ({
  route: ROUTE,
  attempt: 1,
  tolerance: 0,
  solve: [],
  race: [OURS, UPSTREAM],
  tickMs: 2,
  ...over
})

/**
 * The bot rests at y 131.4, so its feet are in cell 131 standing on the snow
 * that fills the bottom of it — the case `boundingBox` alone cannot explain.
 */
const PROBE: WorldProbe = {
  blocks: [
    [-8, 130, 212, 'cobblestone'],
    [-7, 130, 212, 'cobblestone'],
    [-8, 131, 212, 'snow'],
    [-8, 132, 210, 'obsidian']
  ],
  unloaded: [[-11, 131, 212]],
  focus: [
    { pos: [-8, 130, 212], label: 'stall: under', name: 'cobblestone', boundingBox: 'block', properties: {}, shapes: [[0, 0, 0, 1, 1, 1]] },
    { pos: [-8, 131, 212], label: 'stall: feet', name: 'snow', boundingBox: 'block', properties: { layers: 3 }, shapes: [[0, 0, 0, 1, 0.375, 1]] },
    { pos: [-8, 132, 212], label: 'stall: head', name: 'air', boundingBox: 'empty', properties: {}, shapes: [] }
  ]
}

describe('arena auto-debug', () => {
  describe('judge', () => {
    it('flags a run that did not arrive, and names where it stopped', () => {
      const v = judge(report())
      assert.equal(v.interesting, true)
      assert.equal(v.subject?.impl, 'bulba')
      assert.match(v.reasons.join('\n'), /bulba did not arrive \(timeout\), 29\.2 blocks short at -7\.5 131\.4 212\.3/)
    })

    it('flags the stall separately from the failure', () => {
      assert.match(judge(report()).reasons.join('\n'), /bulba stalled 12\.89s at -7\.5 131\.4 212\.3/)
    })

    it('flags upstream finishing a route we could not', () => {
      assert.match(judge(report()).reasons.join('\n'), /upstream arrived in 19\.32s on the same route bulba could not finish/)
    })

    it('flags upstream being meaningfully faster on a route we did win', () => {
      const slow = run('bulba', { wallMs: 30000, travelled: 116 })
      const v = judge(report({ race: [slow, UPSTREAM] }))
      assert.equal(v.interesting, true)
      assert.match(v.reasons.join('\n'), /upstream beat bulba by 36%/)
    })

    it('stays quiet when we win, and when we lose by less than the margin', () => {
      const fast = run('bulba', { wallMs: 18000 })
      assert.equal(judge(report({ race: [fast, UPSTREAM] })).interesting, false)
      // 19.317s vs 20.5s is a 6% lead, inside the default 10% margin.
      const nearly = run('bulba', { wallMs: 20500 })
      assert.equal(judge(report({ race: [nearly, UPSTREAM] })).interesting, false)
    })

    it('picks the worst of our engines as the subject', () => {
      const wasm = run('bulba-wasm', { outcome: 'timeout', endDistance: 60, endPos: [10, 132, 240] })
      const v = judge(report({ race: [OURS, wasm, UPSTREAM] }))
      assert.equal(v.subject?.impl, 'bulba-wasm')
    })

    it('works with no upstream in the lineup', () => {
      const v = judge(report({ race: [OURS] }))
      assert.equal(v.upstream, null)
      assert.equal(v.interesting, true)
    })
  })

  describe('analyse', () => {
    it('sees that both engines planned the identical path', () => {
      const d = analyse({ report: report(), probe: null })
      assert.equal(d.planIdentical, true)
      assert.equal(d.planDivergence, null)
    })

    it('finds where the plans split when they do', () => {
      const other: Array<[number, number, number]> = [...PATH]
      other[3] = [-7.5, 133, 213.5]
      const d = analyse({ report: report({ race: [OURS, run('upstream', { firstPath: other })] }), probe: null })
      assert.equal(d.planIdentical, false)
      assert.equal(d.planDivergence?.index, 3)
      assert.deepEqual(d.planDivergence?.upstream, [-7.5, 133, 213.5])
    })

    it('finds where we went that upstream never did', () => {
      const strayer = run('bulba', {
        outcome: 'timeout',
        endDistance: 40,
        endPos: [40, 132, 200],
        trace: [[0, 2.5, 132, 255.5, 7], [5000, 40, 132, 200, 3]]
      })
      const d = analyse({ report: report({ race: [strayer, UPSTREAM] }), probe: null })
      assert.deepEqual(d.ourStray?.pos, [40, 132, 200])
      assert.equal(d.ourStray?.atMs, 5000)
    })

    it('reports no stray when both engines walked the same corridor', () => {
      const d = analyse({ report: report({ race: [run('bulba', { wallMs: 19000 }), UPSTREAM] }), probe: null })
      assert.equal(d.ourStray, null)
      assert.equal(d.upstreamStray, null)
    })

    it('reports the rest of the route as ground only upstream covered', () => {
      const d = analyse({ report: report(), probe: null })
      // We never strayed: we walked the shared corridor right up to the wedge.
      assert.equal(d.ourStray, null)
      // Upstream carried on past it, which is the whole gap.
      assert.notEqual(d.upstreamStray, null)
      assert.ok((d.upstreamStray?.pos[2] ?? 999) < 212)
    })

    it('points an execution failure at the executor, not the solver', () => {
      const d = analyse({ report: report(), probe: null })
      const leads = d.leads.join('\n')
      assert.match(leads, /execution bug, not a search bug/)
      assert.match(leads, /parkour jump/) // the node it stalled on was one
      assert.match(leads, /futility timer/) // it reset with `stuck` twice
    })

    it('says we walked further when we won the slow way', () => {
      const long = run('bulba', { wallMs: 30000, travelled: 200 })
      const d = analyse({ report: report({ race: [long, UPSTREAM] }), probe: null })
      assert.match(d.practical.join('\n'), /we walked 84 blocks further/)
    })
  })

  describe('focusPoints', () => {
    it('covers the stall and the nodes the executor was aiming at', () => {
      const points = focusPoints(OURS)
      assert.deepEqual(points[0], { pos: [-8, 131, 212], label: 'stall' })
      assert.deepEqual(points[1], { pos: [-8, 132, 210], label: 'node+1 (parkour)' })
      assert.deepEqual(points[2], { pos: [-9, 132, 207], label: 'node+2' })
    })

    it('falls back to where the run ended when it never stalled', () => {
      assert.deepEqual(focusPoints(UPSTREAM), [{ pos: [-17, 132, 184], label: 'end' }])
    })
  })

  describe('renderNotes', () => {
    const notes = (): string =>
      renderNotes(analyse({ report: report(), probe: PROBE }), { resultsFile: '.run/results', timeoutMs: 25000 })

    it('leads with the route and why it was flagged', () => {
      const text = notes()
      assert.match(text, /# arena debug: simple2 \(simple2\), attempt 1/)
      assert.match(text, /## Why this was flagged/)
      assert.match(text, /bulba did not arrive/)
    })

    it('writes out the nodes the executor was still trying to reach', () => {
      const text = notes()
      assert.match(text, /34 path nodes still queued/)
      assert.match(text, /node\+1\s+-8\s+132\s+210\s+.*PARKOUR/)
      assert.match(text, /node\+2\s+-9\s+132\s+207\s+.*walk/)
    })

    it('draws the terrain around the stall with a legend', () => {
      const text = notes()
      assert.match(text, /## The blocks it was stuck against/)
      assert.match(text, /y=131 {2}\(bot feet\)/)
      assert.match(text, /y=130 {2}\(standing on\)/)
      assert.match(text, /legend: .*cobblestone/)
      assert.match(text, /\? = chunk not loaded/)
      assert.match(text, /@/) // the bot's own cell is marked
    })

    it('marks the path nodes on the map even though they arrive as centres', () => {
      // The engines hand back block centres (-7.5, 132, 211.5). Keying the
      // overlay on those matches no cell, and the nodes silently never draw —
      // which is exactly what the first live bundle came out looking like.
      const centred = run('bulba', {
        outcome: 'timeout',
        endDistance: 29.2,
        endPos: [-7.6, 131.4, 212.3],
        worstStallMs: 12960,
        stalls: [{
          atMs: 15900,
          stalledMs: 12960,
          pos: [-7.6, 131.4, 212.3],
          remaining: 34,
          ahead: [{ x: -7.5, y: 132, z: 211.5, parkour: true, cost: 1 }]
        }]
      })
      const probe: WorldProbe = {
        blocks: [[-8, 131, 211, 'cobblestone'], [-8, 130, 212, 'cobblestone']],
        unloaded: [],
        focus: []
      }
      const text = renderNotes(analyse({ report: report({ race: [centred, UPSTREAM] }), probe }),
        { resultsFile: '.run/results', timeoutMs: 25000 })
      assert.match(text, /1 = the next path nodes/)
      assert.match(text, /^ {2}z= +211 {2}\.*1/m) // the node cell is drawn as "1"
      // and the node table speaks block coordinates, not centres
      assert.match(text, /node\+1\s+-8\s+132\s+211/)
      assert.doesNotMatch(text, /-7\.5\s+132\s+211\.5/)
    })

    it('lists only the blocks that are actually on the slice', () => {
      const probe: WorldProbe = {
        blocks: [[-8, 130, 212, 'cobblestone'], [-400, 4, 60, 'netherrack']],
        unloaded: [],
        focus: []
      }
      const text = renderNotes(analyse({ report: report(), probe }), { resultsFile: '.run/results', timeoutMs: 25000 })
      assert.match(text, /legend: a=cobblestone/)
      assert.doesNotMatch(text, /netherrack/)
    })

    it('carries the block state that explains a fractional Y', () => {
      const text = notes()
      assert.match(text, /stall: feet\s+-8 131 212\s+snow\s+bb=block\s+\{"layers":3\}/)
      assert.match(text, /shapes \[\[0,0,0,1,0\.375,1\]\]/)
    })

    it('gives a repro command for the failing segment, not just the route', () => {
      const text = notes()
      assert.match(text, /--routes simple2 --timeout 25/)
      // `--from=` with the equals sign, not `--from `: node's parseArgs
      // refuses a value starting with a dash, and half of 2b2t spawn is
      // negative, so the command it printed could not be pasted.
      assert.match(text, /--from=-8,131,212 --to=-17,132,184/)
    })

    it('renders without a terrain probe', () => {
      const text = renderNotes(analyse({ report: report(), probe: null }), { resultsFile: '.run/results', timeoutMs: 25000 })
      assert.doesNotMatch(text, /## The blocks it was stuck against/)
      assert.match(text, /## Where it got stuck/)
    })
  })
})
