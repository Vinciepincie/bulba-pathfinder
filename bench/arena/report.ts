// Result formatting for the arena benchmark.
import { mkdir, writeFile } from 'node:fs/promises'
import { join } from 'node:path'
import { RESULTS_DIR } from './config.js'
import type { RunResult } from './runner.js'
import type { Route } from './routes.js'

export interface SolveSample {
  impl: string
  /** Cold: a fresh snapshot per solve (what the synchronous API does). */
  ms: number[]
  /** Warm: the live path's cached snapshot, i.e. steady state. */
  warmMs: number[]
  visited: number
  cost: number | null
  nodes: number | null
  status: string
}

export interface RouteReport {
  route: Route
  attempt: number
  /** Endpoints after snapping to standable blocks — what was actually run. */
  resolved?: { start: number[], end: number[] }
  /** Goal radius actually used (chat flag > route field > CLI default). */
  tolerance: number
  solve: SolveSample[]
  race: RunResult[]
  tickMs: number | null
  /** Spread between the racers' actual launch instants. Should be a few ms. */
  startSkewMs?: number
}

export function table (header: string[], rows: string[][]): string {
  const widths = header.map((h, i) => Math.max(h.length, ...rows.map(r => (r[i] ?? '').length)))
  const line = (cells: string[]): string =>
    cells.map((c, i) => (i === 0 ? c.padEnd(widths[i]) : c.padStart(widths[i]))).join('  ')
  return [line(header), widths.map(w => '─'.repeat(w)).join('  '), ...rows.map(line)].join('\n')
}

const ms = (n: number | null): string => n === null ? '—' : n < 1000 ? `${n.toFixed(0)}ms` : `${(n / 1000).toFixed(2)}s`
const num = (n: number | null): string => n === null ? '—' : n.toLocaleString('en-US')

export function solveTable (reports: RouteReport[]): string {
  const rows: string[][] = []
  for (const r of reports) {
    for (const s of r.solve) {
      const cold = [...s.ms].sort((a, b) => a - b)
      const warm = [...(s.warmMs ?? [])].sort((a, b) => a - b)
      rows.push([
        `${r.route.id} ${s.impl}`,
        s.status,
        ms(cold[0]),
        ms(cold[Math.floor(cold.length / 2)]),
        warm.length > 0 ? ms(warm[0]) : '—',
        warm.length > 0 ? ms(warm[Math.floor(warm.length / 2)]) : '—',
        num(s.visited),
        num(s.nodes),
        s.cost === null ? '—' : s.cost.toFixed(1)
      ])
    }
  }
  return [
    table(
      ['route / engine', 'status', 'cold best', 'cold med', 'warm best', 'warm med', 'visited', 'path', 'cost'],
      rows
    ),
    '',
    'cold = a fresh world snapshot per solve (the synchronous API); warm = the',
    'cached snapshot a running bot has. Upstream builds no snapshot, so its two',
    'columns should agree — that agreement is the control for the comparison.'
  ].join('\n')
}

export function raceTable (reports: RouteReport[]): string {
  const rows: string[][] = []
  for (const r of reports) {
    for (const run of r.race) {
      rows.push([
        `${r.route.id} ${run.impl}`,
        run.outcome,
        ms(run.wallMs),
        ms(run.firstSolveMs),
        String(run.solves),
        String(run.replans),
        num(run.visitedNodes),
        run.travelled.toFixed(1),
        String(run.jumps),
        run.damage.toFixed(1),
        run.endDistance.toFixed(1)
      ])
    }
  }
  return table(
    ['route / engine', 'outcome', 'wall', '1st solve', 'solves', 'replans', 'visited', 'blocks', 'jumps', 'dmg', 'left'],
    rows
  )
}

export function summarise (reports: RouteReport[]): string {
  const byImpl = new Map<string, { arrived: number, total: number, wall: number, died: number, wins: number }>()
  const acc = (impl: string): { arrived: number, total: number, wall: number, died: number, wins: number } => {
    const existing = byImpl.get(impl)
    if (existing !== undefined) return existing
    const fresh = { arrived: 0, total: 0, wall: 0, died: 0, wins: 0 }
    byImpl.set(impl, fresh)
    return fresh
  }

  for (const r of reports) {
    for (const run of r.race) {
      const a = acc(run.impl)
      a.total++
      if (run.outcome === 'died') a.died++
      if (run.outcome === 'arrived') { a.arrived++; a.wall += run.wallMs }
    }
    // A win is the fastest actual arrival on that route; a route nobody
    // finished has no winner.
    const finishers = r.race.filter(x => x.outcome === 'arrived').sort((a, b) => a.wallMs - b.wallMs)
    if (finishers.length > 0) acc(finishers[0].impl).wins++
  }

  const rows = [...byImpl.entries()].map(([impl, a]) => [
    impl,
    `${a.arrived}/${a.total}`,
    String(a.wins),
    String(a.died),
    a.arrived > 0 ? ms(a.wall / a.arrived) : '—'
  ])
  const contested = reports.filter(r => r.race.some(x => x.outcome === 'arrived')).length
  return [
    table(['engine', 'arrived', 'wins', 'deaths', 'mean wall (arrived)'], rows),
    '',
    `${contested} of ${reports.length} route runs were finished by at least one engine`
  ].join('\n')
}

export async function writeResults (reports: RouteReport[], meta: Record<string, unknown>): Promise<string> {
  await mkdir(RESULTS_DIR, { recursive: true })
  const stamp = new Date().toISOString().replace(/[:.]/g, '-')
  const path = join(RESULTS_DIR, `arena-${stamp}.json`)
  await writeFile(path, `${JSON.stringify({ meta, reports }, null, 2)}\n`)
  return path
}
