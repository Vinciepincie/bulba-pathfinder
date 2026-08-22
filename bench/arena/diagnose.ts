// Auto-debug: turn a losing route into an investigation packet.
//
// A race that ends `timeout, 29.2 blocks left` is a score, not a bug report.
// The two things that actually explain it are thrown away by the time the run
// ends: the path the executor was still holding, and the blocks it was stuck
// against. This module collects both while they still exist, works out what
// our engine did differently from upstream *in practice* (not in the plan),
// and writes it as markdown an agent can act on without re-running anything.
//
// Triggers (see `judge`): our engine did not arrive, our engine stalled, or
// upstream arrived when we did not / arrived meaningfully sooner.
import { mkdir, readFile, writeFile } from 'node:fs/promises'
import { join } from 'node:path'
import { DEBUG_DIR, HISTORY_FILE } from './config.js'
import type { RouteReport } from './report.js'
import type { Impl, RunResult, WorldProbe } from './runner.js'
import type { Route } from './routes.js'

type Block3 = [number, number, number]
type Sample = [number, number, number, number, number] // ms, x, y, z, nodesLeft

/** How much faster upstream has to be before a win counts as a loss for us. */
const FASTER_BY = 0.10
/** A stall this long is worth investigating even on a route we won. */
const STALL_MS = 3000
/**
 * Two trajectories further apart than this went different ways.
 *
 * Measured, not guessed: on a 116-block route both engines walked the same
 * way round, their lines still sat up to 6.2 blocks apart — different corner
 * cuts and jump landings on the same corridor. A threshold under that reports
 * a divergence on nothing. `maxSeparation` carries the nuance below it.
 */
const STRAY_BLOCKS = 8

export interface Verdict {
  interesting: boolean
  reasons: string[]
  /** The run this is about: ours, worst first. */
  subject: RunResult | null
  upstream: RunResult | null
}

export interface Diagnosis {
  route: Route
  attempt: number
  verdict: Verdict
  /** One line per engine, the practical differences side by side. */
  practical: string[]
  planDivergence: { index: number, ours: Block3, upstream: Block3 } | null
  planIdentical: boolean
  ourStray: { atMs: number, pos: Block3 } | null
  upstreamStray: { atMs: number, pos: Block3 } | null
  /** Furthest our line ever got from any point upstream walked, in blocks. */
  maxSeparation: number | null
  probe: WorldProbe | null
  leads: string[]
}

const impls = (report: RouteReport, want: 'ours' | 'upstream'): RunResult[] =>
  report.race.filter(r => (r.impl === 'upstream') === (want === 'upstream'))

const fmtMs = (n: number): string => n < 1000 ? `${Math.round(n)}ms` : `${(n / 1000).toFixed(2)}s`
/** Same, for a number a run may not carry (an older results file, a no-path). */
const optMs = (n: number | null | undefined): string => typeof n === 'number' && !Number.isNaN(n) ? fmtMs(n) : '—'
const fmtPos = (p: number[]): string => p.map(n => Number(n).toFixed(1)).join(' ')
const block = (p: number[]): Block3 => [Math.floor(p[0]), Math.floor(p[1]), Math.floor(p[2])]

/**
 * Is this route run worth debugging, and which of our runs is the subject?
 *
 * Both `bulba` and `bulba-wasm` are the same algorithm, so when they both
 * fail they fail together; the worse outcome is picked as the subject and the
 * other is reported alongside as corroboration.
 */
export function judge (report: RouteReport, opts: { fasterBy?: number, stallMs?: number } = {}): Verdict {
  const fasterBy = opts.fasterBy ?? FASTER_BY
  const stallMs = opts.stallMs ?? STALL_MS
  const ours = impls(report, 'ours')
  const upstream = impls(report, 'upstream')[0] ?? null
  const reasons: string[] = []

  // Worst first: a failure outranks a slow win, and among failures the one
  // that got least far is the one with the most to explain.
  const rank = (r: RunResult): number => (r.outcome === 'arrived' ? 0 : 1000) + r.endDistance
  const subject = [...ours].sort((a, b) => rank(b) - rank(a))[0] ?? null

  for (const r of ours.filter(r => r.outcome !== 'arrived')) {
    reasons.push(`${r.impl} did not arrive (${r.outcome}), ${r.endDistance.toFixed(1)} blocks short at ${fmtPos(r.endPos)}`)
  }
  for (const r of ours) {
    if (r.worstStallMs >= stallMs) {
      const where = r.stalls[0]?.pos ?? r.endPos
      reasons.push(`${r.impl} stalled ${fmtMs(r.worstStallMs)} at ${fmtPos(where)}`)
    }
  }
  // Compared against the subject only. `bulba` and `bulba-wasm` are the same
  // algorithm, so saying it once per engine is the same sentence twice.
  if (upstream !== null && upstream.outcome === 'arrived' && subject !== null) {
    if (subject.outcome !== 'arrived') {
      reasons.push(`upstream arrived in ${fmtMs(upstream.wallMs)} on the same route ${subject.impl} could not finish`)
    } else if (upstream.wallMs < subject.wallMs * (1 - fasterBy)) {
      const pct = Math.round((1 - upstream.wallMs / subject.wallMs) * 100)
      reasons.push(`upstream beat ${subject.impl} by ${pct}% (${fmtMs(upstream.wallMs)} vs ${fmtMs(subject.wallMs)})`)
    }
  }
  return { interesting: reasons.length > 0, reasons, subject, upstream }
}

/** First index where the two engines planned a different next node. */
function planDivergence (ours: RunResult, upstream: RunResult): Diagnosis['planDivergence'] {
  const a = ours.firstPath
  const b = upstream.firstPath
  if (a === undefined || b === undefined) return null
  const n = Math.min(a.length, b.length)
  for (let i = 0; i < n; i++) {
    if (a[i][0] !== b[i][0] || a[i][1] !== b[i][1] || a[i][2] !== b[i][2]) {
      return { index: i, ours: a[i], upstream: b[i] }
    }
  }
  return a.length === b.length ? null : { index: n, ours: a[n] ?? a[n - 1], upstream: b[n] ?? b[n - 1] }
}

/**
 * The first place `a` went that `b` never did.
 *
 * Comparing the two bots position-by-position at the same timestamp would
 * flag every speed difference as a divergence. What matters is the corridor:
 * the first sample of `a` that no sample of `b` ever came close to.
 */
function strayed (a: Sample[] | undefined, b: Sample[] | undefined, radius = STRAY_BLOCKS): { atMs: number, pos: Block3 } | null {
  if (a === undefined || b === undefined || b.length === 0) return null
  for (const [ms, x, y, z] of a) {
    let near = false
    for (const [, bx, by, bz] of b) {
      if (Math.hypot(x - bx, y - by, z - bz) <= radius) { near = true; break }
    }
    if (!near) return { atMs: ms, pos: [x, y, z] }
  }
  return null
}

/** Furthest `a` ever got from anywhere `b` went. */
function maxSeparation (a: Sample[] | undefined, b: Sample[] | undefined): number | null {
  if (a === undefined || b === undefined || a.length === 0 || b.length === 0) return null
  let worst = 0
  for (const [, x, y, z] of a) {
    let near = Infinity
    for (const [, bx, by, bz] of b) near = Math.min(near, Math.hypot(x - bx, y - by, z - bz))
    worst = Math.max(worst, near)
  }
  return Number(worst.toFixed(1))
}

/** What our engine did differently, in practice rather than on paper. */
function practicalDiff (all: RunResult[], ours: RunResult, upstream: RunResult | null): string[] {
  const lines: string[] = []
  const pk = (r: RunResult): number => r.firstPathParkour?.length ?? 0
  const one = (r: RunResult): string =>
    `${r.impl.padEnd(11)} ${r.outcome.padEnd(8)} wall ${fmtMs(r.wallMs).padStart(7)}  ` +
    `1st solve ${optMs(r.firstSolveMs).padStart(7)}  ` +
    `1st move ${optMs(r.firstMoveMs).padStart(7)}  ` +
    `plan ${String(r.firstPathNodes ?? '—').padStart(4)} nodes (${pk(r)} parkour)  ` +
    `walked ${r.travelled.toFixed(0)}b  jumps ${r.jumps}  replans ${r.replans}  ` +
    `worst stall ${fmtMs(r.worstStallMs)}  dmg ${r.damage.toFixed(0)}  left ${r.endDistance.toFixed(1)}`
  for (const r of all) lines.push(one(r))
  if (upstream !== null) {
    if (pk(ours) !== pk(upstream)) {
      lines.push(`→ our plan used ${pk(ours)} parkour jumps against upstream's ${pk(upstream)}`)
    }
    if (ours.travelled > upstream.travelled * 1.15 && ours.outcome === 'arrived') {
      lines.push(`→ we walked ${(ours.travelled - upstream.travelled).toFixed(0)} blocks further for the same route`)
    }
    if (ours.replans > upstream.replans) {
      const ourReasons = Object.entries(ours.resetReasons).map(([k, v]) => `${k}×${v}`).join(' ')
      lines.push(`→ we replanned ${ours.replans} times (${ourReasons || 'none'}), upstream ${upstream.replans}`)
    }
    if (upstream.worstStallMs < STALL_MS && ours.worstStallMs >= STALL_MS) {
      lines.push('→ upstream never stalled here; the plan is not the problem, the walking is')
    }
  }
  return lines
}

/**
 * Where to look first, chosen from the failure's shape.
 *
 * Deliberately short and specific. A generic "check the pathfinder" line
 * costs an agent a full exploration pass; naming the handler that owns this
 * exact symptom does not.
 */
function leadsFor (d: Omit<Diagnosis, 'leads'>): string[] {
  const leads: string[] = []
  const s = d.verdict.subject
  if (s === null) return leads
  const stall = s.stalls[0]
  const nextIsParkour = stall?.ahead[0]?.parkour === true

  if (d.planIdentical && s.outcome !== 'arrived') {
    leads.push('Both engines planned the identical path, so this is an execution bug, not a search bug: start in `src/plugin.ts` (`monitorMovement`), not the solver.')
  }
  if (stall !== undefined && nextIsParkour) {
    leads.push('The node it stalled on is a parkour jump. Check the takeoff logic in `src/plugin.ts` (the `TAKEOFF_STAND` sneak-creep branch) and the reach rules in `src/parkourEnvelope.ts`.')
  }
  if (stall !== undefined && !nextIsParkour) {
    leads.push('It stalled on a plain walk node, so the move was reachable on paper. Suspect geometry the executor mis-reads (`src/shapes.ts`, thin floors) or a server position correction it keeps forgiving.')
  }
  if ((s.resetReasons.stuck ?? 0) > 0) {
    leads.push('The engine noticed and reset with `stuck`, then re-solved into the same wedge. The 3.5s futility timer in `src/plugin.ts` fired but the replan did not route around the obstacle.')
  }
  if ((s.resetReasons.forced_move ?? 0) > 0) {
    leads.push('`forced_move` resets mean the server was correcting the bot. `FORCED_MOVE_GRACE_MS` in `src/plugin.ts` bounds how long lagbacks excuse a lack of progress — check it is not being refreshed.')
  }
  if ((s.statuses.noPath ?? 0) > 0) {
    leads.push('At least one solve returned `noPath` mid-route. Reproduce it offline with `PF_DUMP_NOPATH=1` and `scripts/replayNoPath.ts`.')
  }
  if (s.outcome === 'arrived' && d.verdict.reasons.some(r => r.includes('beat'))) {
    leads.push('We arrived but slower. Compare the two trajectories below: a longer walk is a cost-model difference, an equal walk is an executor speed difference (sprint, jump timing, corner cutting).')
  }
  return leads
}

export interface DiagnoseInput {
  report: RouteReport
  /** Terrain around the failure, read by the racer that failed. */
  probe: WorldProbe | null
  opts?: { fasterBy?: number, stallMs?: number }
}

export function analyse ({ report, probe, opts }: DiagnoseInput): Diagnosis {
  const verdict = judge(report, opts)
  const subject = verdict.subject
  const upstream = verdict.upstream
  const base = {
    route: report.route,
    attempt: report.attempt,
    verdict,
    practical: subject === null ? [] : practicalDiff(report.race, subject, upstream),
    planDivergence: subject !== null && upstream !== null ? planDivergence(subject, upstream) : null,
    planIdentical: false,
    ourStray: strayed(subject?.trace, upstream?.trace),
    upstreamStray: strayed(upstream?.trace, subject?.trace),
    maxSeparation: maxSeparation(subject?.trace, upstream?.trace),
    probe
  }
  base.planIdentical = subject !== null && upstream !== null &&
    subject.firstPath !== undefined && upstream.firstPath !== undefined &&
    base.planDivergence === null
  return { ...base, leads: leadsFor(base) }
}

/** The focus points a probe should cover: the stall and the nodes ahead of it. */
export function focusPoints (run: RunResult): Array<{ pos: Block3, label: string }> {
  const points: Array<{ pos: Block3, label: string }> = []
  const stall = run.stalls[0]
  if (stall !== undefined) {
    points.push({ pos: block(stall.pos), label: 'stall' })
    for (const [i, n] of stall.ahead.slice(0, 4).entries()) {
      points.push({ pos: block([n.x, n.y, n.z]), label: `node+${i + 1}${n.parkour ? ' (parkour)' : ''}` })
    }
  } else {
    points.push({ pos: block(run.endPos), label: 'end' })
  }
  return points
}

// ── rendering ────────────────────────────────────────────────────────────

/**
 * An ASCII slice of the terrain, one grid per Y layer.
 *
 * A list of block coordinates is unreadable; the shape of the hole a bot is
 * wedged in is obvious at a glance. Overlays win over terrain so the bot and
 * the path nodes stay visible whatever they are standing in.
 */
function renderMap (probe: WorldProbe, centre: Block3, marks: Array<{ pos: Block3, char: string }>, radius = 3): string {
  const [cx, cy, cz] = centre
  const at = new Map<string, string>()
  const legend = new Map<string, string>()
  const chars = 'abcdefghijklmnopqrstuvwxyz'
  for (const [x, y, z, name] of probe.blocks) {
    let ch = legend.get(name)
    if (ch === undefined) {
      ch = chars[legend.size] ?? '+'
      legend.set(name, ch)
    }
    at.set(`${x},${y},${z}`, ch)
  }
  for (const [x, y, z] of probe.unloaded) at.set(`${x},${y},${z}`, '?')
  // Path nodes arrive as block centres (-7.5), the map is keyed by block, so
  // an unfloored mark matches no cell and silently never draws.
  const marked = new Map(marks.map(m => [key(m.pos), m.char]))

  const out: string[] = []
  const drawn = new Set<string>()
  for (let y = cy + 2; y >= cy - 2; y--) {
    const rows: string[] = []
    for (let z = cz - radius; z <= cz + radius; z++) {
      let row = ''
      for (let x = cx - radius; x <= cx + radius; x++) {
        const cell = `${x},${y},${z}`
        const ch = marked.get(cell) ?? at.get(cell) ?? '.'
        drawn.add(ch)
        row += ch
      }
      rows.push(`  z=${String(z).padStart(5)}  ${row}`)
    }
    out.push(`y=${y}${y === cy ? '  (bot feet)' : y === cy - 1 ? '  (standing on)' : ''}`)
    out.push(`           x=${cx - radius} .. ${cx + radius}`)
    out.push(...rows)
    out.push('')
  }
  // Only what is actually on this slice: the probe covers every focus point,
  // so a global legend names blocks that are nowhere in the picture.
  const used = [...legend.entries()].filter(([, ch]) => drawn.has(ch))
  const nodes = marks.filter(m => m.char !== '@' && drawn.has(m.char)).map(m => m.char)
  out.push(`legend: ${used.map(([name, ch]) => `${ch}=${name}`).join('  ') || '(all air)'}`)
  out.push(`        . = air   ? = chunk not loaded   @ = the bot` +
    (nodes.length > 0 ? `   ${nodes.join('')} = the next path nodes, in order` : ''))
  return out.join('\n')
}

const key = (p: number[]): string => `${Math.floor(p[0])},${Math.floor(p[1])},${Math.floor(p[2])}`

export function renderNotes (d: Diagnosis, meta: { resultsFile: string, timeoutMs: number }): string {
  const s = d.verdict.subject
  const up = d.verdict.upstream
  const out: string[] = []
  const r = d.route
  const end = d.route.end

  out.push(`# arena debug: ${r.id} (${r.name}), attempt ${d.attempt}`)
  out.push('')
  out.push(`route \`${r.start.join(' ')}\` → \`${r.end.join(' ')}\`, scenario ${r.scenario}, tolerance ${d.route.tolerance ?? 0}`)
  out.push('')
  out.push('## Why this was flagged')
  out.push('')
  for (const reason of d.verdict.reasons) out.push(`- ${reason}`)
  out.push('')

  out.push('## What each engine did')
  out.push('')
  out.push('```')
  out.push(...d.practical)
  out.push('```')
  out.push('')

  out.push('## Plan vs practice')
  out.push('')
  if (d.planIdentical) {
    out.push(`Both engines produced the **same first path** (${s?.firstPathNodes ?? '?'} nodes). Whatever went wrong happened while walking it.`)
  } else if (d.planDivergence !== null) {
    const pd = d.planDivergence
    out.push(`The plans agree for ${pd.index} nodes, then split: we go to \`${pd.ours.join(' ')}\`, upstream to \`${pd.upstream.join(' ')}\`.`)
  } else {
    out.push('No comparable plan from upstream (it did not produce a first path).')
  }
  out.push('')
  if (d.maxSeparation !== null) {
    out.push(`- Our line never got further than **${d.maxSeparation} blocks** from anywhere upstream walked.`)
  }
  if (d.ourStray !== null) {
    out.push(`- We first went somewhere upstream never did at ${fmtMs(d.ourStray.atMs)}: \`${fmtPos(d.ourStray.pos)}\`.`)
  } else if (up !== null) {
    out.push(`- We never left upstream's corridor (nothing beyond ${STRAY_BLOCKS} blocks of it), so we walked the same way round.`)
  }
  if (d.upstreamStray !== null) {
    // When our run stopped short, "upstream went where we did not" is just
    // the rest of the route, and reading it as a routing difference sends an
    // investigation the wrong way.
    const rest = s !== null && s.outcome !== 'arrived'
    out.push(rest
      ? `- Upstream carried on past where we stopped, from \`${fmtPos(d.upstreamStray.pos)}\` at ${fmtMs(d.upstreamStray.atMs)}. That is the rest of the route, not a routing difference.`
      : `- Upstream first went somewhere we never did at ${fmtMs(d.upstreamStray.atMs)}: \`${fmtPos(d.upstreamStray.pos)}\`.`)
  }
  out.push('')

  if (s !== null && s.stalls.length > 0) {
    out.push('## Where it got stuck')
    out.push('')
    for (const [i, stall] of s.stalls.entries()) {
      out.push(`### stall ${i + 1}: ${fmtMs(stall.stalledMs)} motionless at \`${fmtPos(stall.pos)}\` (${fmtMs(stall.atMs)} into the run)`)
      out.push('')
      out.push(`${stall.remaining} path nodes still queued. The executor was trying to reach, in order:`)
      out.push('')
      out.push('```')
      // Block coordinates throughout: the engines hand these back as centres
      // (-7.5), and mixing the two conventions in one report is a trap.
      let from: number[] = stall.pos
      for (const [j, n] of stall.ahead.entries()) {
        const b = [Math.floor(n.x), Math.floor(n.y), Math.floor(n.z)]
        const gap = Math.hypot(b[0] + 0.5 - from[0], b[2] + 0.5 - from[2])
        const dy = b[1] - Math.floor(from[1])
        out.push(`  node+${j + 1}  ${String(b[0]).padStart(5)} ${String(b[1]).padStart(4)} ${String(b[2]).padStart(5)}  ` +
          `${gap.toFixed(1)} blocks away, dy ${dy >= 0 ? `+${dy}` : dy}  ` +
          `${n.parkour ? 'PARKOUR' : 'walk'}  cost ${n.cost.toFixed(2)}`)
        from = [b[0] + 0.5, b[1], b[2] + 0.5]
      }
      out.push('```')
      out.push('')
    }
  } else if (s !== null) {
    out.push('## Where it ended')
    out.push('')
    out.push(`No stall over ${STALL_MS} ms was recorded. The run ended at \`${fmtPos(s.endPos)}\`, ${s.endDistance.toFixed(1)} blocks from \`${end.join(' ')}\`.`)
    out.push('')
  }

  if (d.probe !== null && s !== null) {
    const stall = s.stalls[0]
    const centre = block(stall?.pos ?? s.endPos)
    const marks: Array<{ pos: Block3, char: string }> = [{ pos: centre, char: '@' }]
    for (const [i, n] of (stall?.ahead ?? []).slice(0, 6).entries()) {
      marks.push({ pos: [n.x, n.y, n.z], char: String(i + 1) })
    }
    out.push('## The blocks it was stuck against')
    out.push('')
    out.push('```')
    out.push(renderMap(d.probe, centre, marks))
    out.push('```')
    out.push('')
    out.push('Full state for the blocks that decide the move:')
    out.push('')
    out.push('```')
    for (const f of d.probe.focus) {
      const props = Object.keys(f.properties).length > 0 ? `  ${JSON.stringify(f.properties)}` : ''
      const shapes = f.shapes.length > 0 ? `  shapes ${JSON.stringify(f.shapes)}` : ''
      out.push(`  ${f.label.padEnd(22)} ${f.pos.join(' ').padEnd(18)} ${f.name.padEnd(24)} bb=${f.boundingBox}${props}${shapes}`)
    }
    out.push('```')
    out.push('')
    out.push('`boundingBox` reports `block` for carpets, slabs and snow layers alike, so read `shapes` when a bot rests at a fractional Y.')
    out.push('')
  }

  if (d.leads.length > 0) {
    out.push('## Where to look')
    out.push('')
    for (const lead of d.leads) out.push(`- ${lead}`)
    out.push('')
  }

  out.push('## Reproduce')
  out.push('')
  out.push('With the arena server already running (`--keep` from the last race, or `npm run arena:server`):')
  out.push('')
  out.push('```sh')
  out.push(`# the whole route, our engine only`)
  out.push(`npm run arena:race -- --attach --engines bulba --routes ${d.route.id} --timeout ${Math.round(meta.timeoutMs / 1000)} --keep`)
  if (s !== null && s.stalls[0] !== undefined) {
    const from = block(s.stalls[0].pos)
    out.push('')
    out.push('# just the segment it failed on, from the stall to the goal')
    // --from=x,y,z, not --from x,y,z: node parseArgs rejects a value that
    // starts with a dash, and half of 2b2t spawn has negative coordinates.
    out.push(`npm run arena:race -- --attach --engines bulba,upstream --from=${from.join(',')} --to=${end.join(',')} --timeout 30 --keep`)
  }
  out.push('```')
  out.push('')
  out.push('## Files')
  out.push('')
  out.push('- `bundle.json` — the full RouteReport for this run: per-tick trace, every `path_update`, reset reasons, stalls, and the terrain probe.')
  out.push(`- \`${meta.resultsFile}\` — the results file the whole race batch wrote.`)
  out.push('- `../../history.json` — what this route has done on previous runs.')
  out.push('')
  return out.join('\n')
}

// ── output ───────────────────────────────────────────────────────────────

export interface Bundle { dir: string, notes: string }

export async function writeBundle (d: Diagnosis, report: RouteReport, meta: { resultsFile: string, timeoutMs: number, stamp: string }): Promise<Bundle> {
  const dir = join(DEBUG_DIR, `${meta.stamp}-${d.route.id}-a${d.attempt}`)
  await mkdir(dir, { recursive: true })
  const notes = join(dir, 'NOTES.md')
  await writeFile(notes, renderNotes(d, meta))
  await writeFile(join(dir, 'bundle.json'), `${JSON.stringify({
    route: d.route,
    attempt: d.attempt,
    verdict: d.verdict.reasons,
    planIdentical: d.planIdentical,
    planDivergence: d.planDivergence,
    ourStray: d.ourStray,
    upstreamStray: d.upstreamStray,
    probe: d.probe,
    report
  }, null, 2)}\n`)
  return { dir, notes }
}

export interface HistoryEntry {
  at: string
  attempt: number
  verdict: string[]
  engines: Array<{ impl: Impl, outcome: string, wallMs: number, travelled: number, replans: number, worstStallMs: number, endDistance: number }>
  planIdentical: boolean
  /** Where our engine stopped making progress, if it did. */
  stuckAt?: Block3
  bundle?: string
}

export interface History { routes: Record<string, HistoryEntry[]> }

/**
 * What this route has done before, kept next to the route book.
 *
 * Committed on purpose: the useful signal is "this route has wedged in the
 * same spot four runs running", and that is lost if the record lives under
 * the gitignored `.run/`. Capped so a long session cannot bloat it.
 */
export async function appendHistory (routeId: string, entry: HistoryEntry, keep = 10): Promise<void> {
  let history: History = { routes: {} }
  try {
    history = JSON.parse(await readFile(HISTORY_FILE, 'utf8')) as History
  } catch { /* first run */ }
  history.routes[routeId] = [...(history.routes[routeId] ?? []), entry].slice(-keep)
  await writeFile(HISTORY_FILE, `${JSON.stringify(history, null, 2)}\n`)
}

export function historyEntry (d: Diagnosis, bundleDir?: string): HistoryEntry {
  const stall = d.verdict.subject?.stalls[0]
  return {
    at: new Date().toISOString(),
    attempt: d.attempt,
    verdict: d.verdict.reasons,
    engines: [d.verdict.subject, d.verdict.upstream].filter((r): r is RunResult => r !== null).map(r => ({
      impl: r.impl,
      outcome: r.outcome,
      wallMs: Math.round(r.wallMs),
      travelled: Number(r.travelled.toFixed(1)),
      replans: r.replans,
      worstStallMs: Math.round(r.worstStallMs),
      endDistance: Number(r.endDistance.toFixed(1))
    })),
    planIdentical: d.planIdentical,
    stuckAt: stall === undefined ? undefined : block(stall.pos),
    bundle: bundleDir
  }
}
