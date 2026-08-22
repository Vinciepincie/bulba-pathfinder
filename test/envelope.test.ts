// Guards the pasted physics constants in src/parkourEnvelope.ts and the
// generated offset table in src/parkourTable.ts.
import { expect } from 'chai'
import { deriveJumpEnvelope, measureFlightCurve } from './helpers/jumpEnvelope.js'
import {
  J_STANDING, J_RUNNING, J_LOW_STANDING, J_LOW_RUNNING, reachBucket, ENVELOPE_BUCKETS,
  FLIGHT_STANDING, FLIGHT_RUNNING, FLIGHT_LOW_STANDING, FLIGHT_LOW_RUNNING,
  feetAt, flightNeeded, takeoffStand, takeoffRun
} from '../src/parkourEnvelope.js'
import { getParkourExtTable } from '../src/parkourTable.js'
import type { ParkourExtEntry } from '../src/parkourTable.js'

describe('parkour reach envelope', function () {
  this.timeout(20000)

  it('constants match a fresh prismarine-physics derivation exactly', () => {
    const [standing, running] = deriveJumpEnvelope()
    expect(standing).to.have.length(ENVELOPE_BUCKETS)
    expect(running).to.have.length(ENVELOPE_BUCKETS)
    for (let i = 0; i < ENVELOPE_BUCKETS; i++) {
      expect(J_STANDING[i]).to.be.closeTo(standing[i], 1e-12, `standing bucket ${i}`)
      expect(J_RUNNING[i]).to.be.closeTo(running[i], 1e-12, `running bucket ${i}`)
    }
    // Anchor against the parkour community's tick math (mcpk.wiki): the flat
    // sprint-jam flight is 2.6234 blocks (J is that minus the 0.2 margin).
    expect(J_STANDING[reachBucket(0)]).to.be.closeTo(2.6234315139003863 - 0.2, 1e-9)
    // Head-hitter class (lid 2 above the feet, arc bonks at +0.2).
    const [lowStanding, lowRunning] = deriveJumpEnvelope(true)
    for (let i = 0; i < ENVELOPE_BUCKETS; i++) {
      expect(J_LOW_STANDING[i]).to.be.closeTo(lowStanding[i], 1e-12, `low standing bucket ${i}`)
      expect(J_LOW_RUNNING[i]).to.be.closeTo(lowRunning[i], 1e-12, `low running bucket ${i}`)
    }
    // +1 landings are impossible under a 2-high lid (sentinel row ≤ 0).
    expect(J_LOW_STANDING[reachBucket(1)]).to.be.at.most(0)
    expect(J_LOW_RUNNING[reachBucket(1)]).to.be.at.most(0)
  })

  it('flight curves match a fresh prismarine-physics derivation exactly', () => {
    const standing = measureFlightCurve(0)
    const running = measureFlightCurve(1)
    const lowStanding = measureFlightCurve(0, true)
    const lowRunning = measureFlightCurve(1, true)
    expect(FLIGHT_STANDING).to.deep.equal(standing)
    expect(FLIGHT_RUNNING).to.deep.equal(running)
    expect(FLIGHT_LOW_STANDING).to.deep.equal(lowStanding)
    expect(FLIGHT_LOW_RUNNING).to.deep.equal(lowRunning)
    // The corridor bound assumes rise-then-fall (endpoint-min): after the
    // first strict decrease the curve must never rise again (plateaus from
    // the ceiling bonk are fine).
    for (const curve of [standing, running, lowStanding, lowRunning]) {
      let peaked = false
      for (let i = 3; i < curve.length; i += 2) {
        if (peaked) expect(curve[i]).to.be.at.most(curve[i - 2] + 1e-12)
        else if (curve[i] < curve[i - 2]) peaked = true
      }
    }
    // feetAt clamps to ground before takeoff and to the last sample beyond.
    expect(feetAt(FLIGHT_RUNNING, 0)).to.equal(0)
    expect(feetAt(FLIGHT_RUNNING, 99)).to.equal(FLIGHT_RUNNING[FLIGHT_RUNNING.length - 1])
  })

  function fnStand (a: number, b: number): number {
    const [tx, tz] = takeoffStand(a, b)
    return flightNeeded(a, b, tx, tz)
  }
  function fnRun (a: number, b: number): number {
    const [tx, tz] = takeoffRun(a, b)
    return flightNeeded(a, b, tx, tz)
  }

  it('anchors: matches real player capability (per-axis credit model)', () => {
    const flat = reachBucket(0)
    const up = reachBucket(1)
    // Pillar-to-pillar (2,2) hops work from a standing start (live-verified).
    expect(fnStand(2, 2)).to.be.at.most(J_STANDING[flat])
    // The storage-room pillar course (prod dump 2026-08-20): diagonal 1x1
    // hops players make from a standstill — (1,3)+1, (3,3) flat, (2,2)+1.
    expect(fnStand(1, 3)).to.be.at.most(J_STANDING[up])
    expect(fnStand(3, 3)).to.be.at.most(J_STANDING[flat])
    expect(fnStand(2, 2)).to.be.at.most(J_STANDING[up])
    // The classic running 4-block jump (upstream parkour d=4) must fit.
    expect(fnRun(4, 0)).to.be.at.most(J_RUNNING[flat])
    // Upstream's 3-forward-1-up sprint jump must fit at running speed.
    expect(fnRun(3, 0)).to.be.at.most(J_RUNNING[up])
    // Impossible jumps stay impossible: flat 5-cardinal standing, 4+1 up.
    expect(fnStand(5, 0)).to.be.greaterThan(J_STANDING[flat])
    expect(fnRun(4, 1)).to.be.greaterThan(J_RUNNING[up] - 1e-9)
    // Drops must extend reach monotonically (more airtime, more distance).
    for (let k = 0; k < ENVELOPE_BUCKETS - 1; k++) {
      expect(J_STANDING[k + 1]).to.be.greaterThan(J_STANDING[k] - 1e-9)
      expect(J_RUNNING[k + 1]).to.be.greaterThan(J_RUNNING[k] - 1e-9)
    }
  })
})

describe('generated parkour table', () => {
  const table = getParkourExtTable()
  const all = [...table.diag, ...table.cardX, ...table.cardZ]

  function entry (tx: number, tz: number): ParkourExtEntry | undefined {
    return all.find(e => e.tx === tx && e.tz === tz)
  }

  function cellSets (e: ParkourExtEntry): { line: string[], corner: string[] } {
    const line: string[] = []
    const corner: string[] = []
    for (let k = 0; k * 2 < e.cells.length; k++) {
      (k < e.nLine ? line : corner).push(`${e.cells[k * 2]},${e.cells[k * 2 + 1]}`)
    }
    return { line: line.sort(), corner: corner.sort() }
  }

  it('reproduces the hand-derived swept-cell sets (regression)', () => {
    // Corner cells are those the hitbox penetrates by ≥ CORNER_NICK (0.15)
    // on BOTH axes — shallower nicks are slides in vanilla, not stops.
    expect(cellSets(entry(2, 1)!)).to.deep.equal({ line: ['1,0', '1,1'], corner: [] })
    expect(cellSets(entry(1, 2)!)).to.deep.equal({ line: ['0,1', '1,1'], corner: [] })
    expect(cellSets(entry(2, 2)!)).to.deep.equal({ line: ['1,1'], corner: ['0,1', '1,0', '1,2', '2,1'] })
    expect(cellSets(entry(3, 1)!)).to.deep.equal({ line: ['1,0', '2,1'], corner: ['1,1', '2,0'] })
    expect(cellSets(entry(3, 2)!)).to.deep.equal({ line: ['1,0', '1,1', '2,1', '2,2'], corner: ['0,1', '3,1'] })
    expect(cellSets(entry(2, 0)!)).to.deep.equal({ line: ['1,0'], corner: [] })
    expect(cellSets(entry(4, 0)!)).to.deep.equal({ line: ['1,0', '2,0', '3,0'], corner: [] })
    // The prod corner jump (1,3)+1: the pillar beside the landing sits at
    // (0,3), which the flight only nicks — it must NOT be a corridor cell.
    expect(cellSets(entry(1, 3)!)).to.deep.equal({ line: ['0,1', '1,2'], corner: ['0,2', '1,1'] })
  })

  it('enumerates every feasible offset and none outside', () => {
    for (const e of all) {
      expect(e.dist).to.be.at.least(2 - 1e-9)
      expect(Math.max(Math.abs(e.tx), Math.abs(e.tz))).to.be.at.most(6)
      expect(e.cost).to.be.closeTo(e.dist + 0.5, 1e-12)
      expect(e.fnRun).to.be.at.most(J_RUNNING[ENVELOPE_BUCKETS - 1] + 1e-9)
      expect(e.runX).to.equal(-e.cells[0])
      expect(e.runZ).to.equal(-e.cells[1])
      expect(e.nLine).to.be.at.least(1)
    }
    // MLG offsets exist (drop-jumps unlock them at runtime)…
    for (const [a, b] of [[4, 1], [3, 3], [4, 2], [4, 4], [5, 1], [5, 3], [6, 1], [6, 4]]) {
      expect(entry(a, b), `(${a},${b})`).to.not.equal(undefined)
    }
    expect(table.cardX.map(e => e.tx)).to.deep.equal([2, 3, 4, 5, 6])
    // …but nothing beyond feasibility ((6,6) needs 6.75 flight > deepest
    // running bucket) or the octile-admissible major.
    expect(entry(6, 6)).to.equal(undefined)
    expect(entry(6, 5)).to.equal(undefined)
    expect(all.some(e => Math.abs(e.tx) > 6 || Math.abs(e.tz) > 6)).to.equal(false)
    expect(all.some(e => Math.abs(e.tx) <= 1 && Math.abs(e.tz) <= 1)).to.equal(false)
  })
})
