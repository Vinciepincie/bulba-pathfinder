// The arena benchmark: @bulba/pathfinder vs mineflayer-pathfinder, racing
// the same routes through real 2b2t spawn terrain on a real server.
//
//   npm run arena:race
//   npm run arena:race -- --routes r01 --repeat 3 --wait-for 1
//
// By default a route is ONE step: the bots are teleported onto the same
// start block, held, and released together, and the planning cost is read
// out of that same run (the `1st solve` column). Human spectators are
// teleported along to watch.
//
// `--mode both` adds a separate planning phase of N repeated solves, and
// runs it AFTER the race on purpose: timing a warm solve means driving the
// live path, which warms the custom engine's snapshot cache. Measuring
// first would hand it a pre-loaded first solve in the race it never earned,
// while upstream has no cache to warm — a free head start on the wall
// clock. `--mode solve` skips the race entirely.
import { fork, spawn, type ChildProcess } from 'node:child_process'
import { fileURLToPath } from 'node:url'
import { parseArgs } from 'node:util'
import type { Bot } from 'mineflayer'
import {
  connectReferee, countdown, parkAbove, prepareWorld, spectators, stageRoute, tickHealth, watchDeaths
} from './arena.js'
import { startServer, writeServerConfig, type ServerHandle } from './server.js'
import { loadRoutes, saveRoutes, selectRoutes, type Route } from './routes.js'
import { raceTable, solveTable, summarise, writeResults, type RouteReport } from './report.js'
import type { FromChild } from './racerProcess.js'
import type { Impl, MovementProfile, RunResult } from './runner.js'
import { analyse, appendHistory, focusPoints, historyEntry, judge, writeBundle } from './diagnose.js'
import { pairSigns, scanSigns } from './signs.js'
import { EDIT_HELP, equipEditor, equipEditorsOnJoin, persistSignRoutes, scanWorld } from './edit.js'
import { ARENA_RADIUS, BOT_BULBA, BOT_BULBA_NOHOP, BOT_BULBA_WASM, BOT_UPSTREAM, HOST, PORT, RESULTS_DIR, WORLD_DIR } from './config.js'

const { values } = parseArgs({
  options: {
    routes: { type: 'string', default: '' },
    scenario: { type: 'string', default: '' },
    edit: { type: 'boolean', default: false }, // author routes in-world with signs, don't race
    signs: { type: 'boolean', default: false }, // race the !PF signs in the world, not routes.json
    from: { type: 'string', default: '' }, // ad-hoc route: --from x,y,z --to x,y,z
    to: { type: 'string', default: '' },
    'save-route': { type: 'string', default: '' }, // id to persist the ad-hoc route under
    mode: { type: 'string', default: 'race' }, // race | both | solve — see the header
    repeat: { type: 'string', default: '1' },
    'solve-repeats': { type: 'string', default: '5' },
    timeout: { type: 'string', default: '120' }, // seconds per race
    think: { type: 'string', default: '5000' }, // ms per solve
    tolerance: { type: 'string', default: '0' }, // goal radius in blocks
    'max-drop': { type: 'string', default: '4' },
    countdown: { type: 'string', default: '3' },
    'wait-for': { type: 'string', default: '0' }, // human spectators to wait for
    engines: { type: 'string', default: '' }, // subset of upstream,bulba,bulba-wasm
    parity: { type: 'boolean', default: false }, // disable extended parkour
    attach: { type: 'boolean', default: false }, // use an already-running server
    keep: { type: 'boolean', default: false }, // leave the server up afterwards
    debug: { type: 'string', default: 'auto' }, // auto | off | always — see runDebug
    'debug-cmd': { type: 'string', default: '' }, // run this with the bundle path appended
    'faster-by': { type: 'string', default: '0.10' } // upstream lead that counts as a loss
  }
})

const profile: MovementProfile = {
  extendedParkour: !values.parity,
  maxDropDown: Number(values['max-drop']),
  thinkTimeout: Number(values.think)
}
const timeoutMs = Number(values.timeout) * 1000
const tolerance = Number(values.tolerance)
const repeats = Number(values.repeat)
const solveRepeats = Number(values['solve-repeats'])
const mode = values.mode as 'both' | 'solve' | 'race'
const debugMode = values.debug as 'auto' | 'off' | 'always'
const fasterBy = Number(values['faster-by'])

const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))

/** One stamp per process, so a batch's debug bundles sort together. */
const runStamp = new Date().toISOString().replace(/[:.]/g, '-')

/** Set once the referee is connected; maps racer name -> vanilla death message. */
let deathsSince: (since: number) => Map<string, string> = () => new Map()

class Child {
  private readonly proc: ChildProcess
  private waiter: { resolve: (m: FromChild) => void, reject: (e: Error) => void } | null = null

  constructor (readonly impl: Impl, readonly username: string) {
    // tsx runs the parent through a loader flag; children need the same one.
    const execArgv = process.execArgv.length > 0 ? process.execArgv : ['--import', 'tsx']
    this.proc = fork(fileURLToPath(new URL('./racerProcess.ts', import.meta.url)), [], {
      execArgv,
      stdio: ['inherit', 'inherit', 'inherit', 'ipc']
    })
    this.proc.on('message', (msg: FromChild) => {
      const w = this.waiter
      this.waiter = null
      w?.resolve(msg)
    })
    this.proc.on('exit', code => {
      const w = this.waiter
      this.waiter = null
      w?.reject(new Error(`${this.impl} racer exited (${code})`))
    })
  }

  async request<T extends FromChild['t']> (msg: object, expect: T): Promise<Extract<FromChild, { t: T }>> {
    const reply = await new Promise<FromChild>((resolve, reject) => {
      this.waiter = { resolve, reject }
      this.proc.send(msg as never)
    })
    if (reply.t === 'failed') throw new Error(`${this.impl} racer: ${reply.message}`)
    if (reply.t !== expect) throw new Error(`${this.impl} racer: expected ${expect}, got ${reply.t}`)
    return reply as Extract<FromChild, { t: T }>
  }

  send (msg: object): void { this.proc.send(msg as never) }
  kill (): void { this.proc.kill() }
}

async function waitForSpectators (ref: Bot, want: number): Promise<void> {
  if (want <= 0) return
  console.log(`\nwaiting for ${want} spectator(s) — join ${HOST}:${PORT} (offline mode, any name)`)
  while (spectators(ref).length < want) await sleep(1000)
  console.log(`spectators: ${spectators(ref).join(', ')}`)
}

/** Chat flag beats the route's own field, which beats the CLI default. */
function toleranceFor (route: Route, override?: number): number {
  return override ?? route.tolerance ?? tolerance
}

async function runRoute (
  ref: Bot,
  children: Child[],
  route: Route,
  attempt: number,
  override?: number
): Promise<RouteReport> {
  const tol = toleranceFor(route, override)
  const report: RouteReport = { route, attempt, tolerance: tol, solve: [], race: [], tickMs: null }
  const label = `${route.id} · ${route.name} · ${route.scenario}${tol > 0 ? ` · ±${tol}` : ''}`
  console.log(`\n── ${label}${repeats > 1 ? ` (attempt ${attempt}/${repeats})` : ''}`)
  console.log(`   ${route.start.join(' ')}  →  ${route.end.join(' ')}`)

  // Load the area from spectator first: nobody can be pushed out of a block
  // or fall while ~1000 chunks arrive, and the endpoints can be inspected
  // before anyone has to stand on them.
  const names = children.map(c => c.username)
  await parkAbove(ref, names, route.start)
  await Promise.all(children.map(async c =>
    await c.request({ t: 'awaitStaged', start: route.start, end: route.end, timeoutMs: 60000, parked: true }, 'staged')
  ))

  // Resolved once, by one child, and handed to both — the two engines must
  // never be asked to solve slightly different problems.
  const fixed = await children[0].request({ t: 'resolve', start: route.start, end: route.end }, 'resolved')
  const start = fixed.start
  const end = fixed.end
  if (fixed.startMoved !== 0 || fixed.endMoved !== 0) {
    console.log(`   snapped to standable blocks: start ${start.join(' ')} (${fixed.startMoved >= 0 ? `+${fixed.startMoved}` : fixed.startMoved}y), end ${end.join(' ')} (${fixed.endMoved >= 0 ? `+${fixed.endMoved}` : fixed.endMoved}y)`)
  }
  report.resolved = { start, end }

  await stageRoute(ref, names, start, end)
  const staged = await Promise.all(children.map(async c =>
    await c.request({ t: 'awaitStaged', start, end, timeoutMs: 30000, parked: false }, 'staged')
  ))
  for (const [i, s] of staged.entries()) {
    const who = children[i].impl
    if (!s.ok) console.warn(`   ! ${who} did not settle on the start block (at ${s.position.map(n => n.toFixed(1)).join(' ')})`)
    if (!s.goalLoaded) console.warn(`   ! ${who} never received the destination chunk — raise ARENA_VIEW_DISTANCE`)
    if (!s.settled) console.warn(`   ! ${who} was still receiving chunks when the hold expired`)
    if (s.gameMode !== 'survival') console.warn(`   ! ${who} is in ${s.gameMode}, not survival`)
  }
  report.tickMs = await tickHealth(ref)

  // Deliberately defined here but RUN AFTER the race. Timing a warm solve
  // means driving the live path, which also warms the custom engine's
  // snapshot cache — doing that first would hand it a pre-loaded first solve
  // in the race it has not earned, while upstream has no cache to warm. That
  // is a free head start on the wall clock, so the race goes first and every
  // engine starts the route cold.
  const runSolvePhase = async (): Promise<void> => {
    for (const c of children) {
      const r = await c.request({ t: 'solve', end, repeats: solveRepeats }, 'solveResult')
      // Cold measures a fresh snapshot per solve; warm measures the cached
      // one a running bot actually has. On short routes they differ by 10x.
      const warm = await c.request({ t: 'solveWarm', end, repeats: solveRepeats }, 'solveWarmResult')
      report.solve.push({
        impl: c.impl, ms: r.ms, warmMs: warm.ms, visited: r.visited, cost: r.cost, nodes: r.nodes, status: r.status
      })
      const best = Math.min(...r.ms)
      const bestWarm = warm.ms.length > 0 ? Math.min(...warm.ms) : NaN
      console.log(`   solve ${c.impl.padEnd(11)} ${r.status.padEnd(8)} cold ${best.toFixed(1)}ms  warm ${Number.isNaN(bestWarm) ? '—' : `${bestWarm.toFixed(1)}ms`}  visited ${r.visited}`)
    }
  }

  if (mode !== 'solve') {
    // Re-stage: the solve phase leaves the bots where they stood, but a long
    // main-thread solve can drift them off the block.
    await stageRoute(ref, names, start, end)
    await Promise.all(children.map(async c =>
      await c.request({ t: 'awaitStaged', start, end, timeoutMs: 30000, parked: false }, 'staged')
    ))
    announceRoute(ref, route, start, end, children.map(c => c.impl))
    await countdown(ref, Number(values.countdown), label)

    // Every child is told to launch at the same wall-clock instant rather
    // than "as soon as you get this": the go messages go out in order and
    // each process schedules its own event loop, which gave whoever was told
    // first a visible head start off the line.
    const goAt = Date.now()
    const startAt = goAt + 700
    const races = children.map(async c => await c.request(
      { t: 'race', routeId: route.id, end, tolerance: tol, timeoutMs, startAt }, 'raceResult'
    ))
    const results = (await Promise.all(races)).map(r => r.result)
    const launches = results.map(r => r.startedAt).filter(t => t > 0)
    if (launches.length > 1) {
      const skew = Math.max(...launches) - Math.min(...launches)
      report.startSkewMs = skew
      console.log(`   start skew ${skew} ms${skew > 60 ? '  ← more than a tick, treat the wall times with suspicion' : ''}`)
    }
    // "died" alone is not a useful result — drowned and fell-from-a-high-place
    // point at completely different gaps.
    const deaths = deathsSince(goAt)
    for (const [i, r] of results.entries()) {
      const cause = deaths.get(children[i].username)
      if (cause !== undefined) r.deathCause = cause
    }
    report.race.push(...results)
    for (const r of results) {
      console.log(`   race  ${r.impl.padEnd(8)} ${r.outcome.padEnd(8)} ${(r.wallMs / 1000).toFixed(2)}s  ` +
        `1st solve ${r.firstSolveMs === null ? '—' : `${r.firstSolveMs.toFixed(0)}ms`}  ` +
        `1st move ${r.firstMoveMs === null ? '—' : `${r.firstMoveMs.toFixed(0)}ms`}  ` +
        `replans ${r.replans}  ${r.travelled.toFixed(0)} blocks  ${r.endDistance.toFixed(1)} left`)
      if (r.outcome !== 'arrived') {
        const fmt = (o: Record<string, number>): string =>
          Object.entries(o).map(([k, v]) => `${k}×${v}`).join(' ') || 'none'
        const lied = r.promiseOutcome !== r.outcome ? ` | goto() claimed ${r.promiseOutcome}` : ''
        const cause = r.deathCause === undefined ? '' : ` | "${r.deathCause}"`
        console.log(`         solve status: ${fmt(r.statuses)} | reset reasons: ${fmt(r.resetReasons)} | mode ${r.gameMode}${cause}${lied}${r.error === undefined ? '' : ` | ${r.error}`}`)
      }
    }
    announce(ref, results)
    await runDebug(children, report)
  }

  if (mode !== 'race') await runSolvePhase()
  return report
}

/**
 * Auto-debug, run before anything moves the bots again.
 *
 * The two facts that explain a loss have a short shelf life: the path the
 * executor was still holding is gone once the goal is cleared, and the blocks
 * around the failure are only readable from the racer that failed, while its
 * chunks are still loaded. So this runs inside the route, immediately after
 * the race, not from the results file afterwards.
 *
 * `--debug auto` (the default) writes a bundle only for a flagged route,
 * `always` writes one for every route, `off` skips it.
 */
async function runDebug (children: Child[], report: RouteReport, force = false): Promise<void> {
  if (debugMode === 'off' && !force) return
  const verdict = judge(report, { fasterBy })
  if (!verdict.interesting && debugMode !== 'always' && !force) return

  const subject = verdict.subject
  let probe = null
  if (subject !== null) {
    const child = children.find(c => c.impl === subject.impl)
    if (child !== undefined) {
      try {
        const reply = await child.request(
          { t: 'probeWorld', focus: focusPoints(subject), radius: 3 }, 'worldProbe'
        )
        probe = reply.probe
      } catch (error) {
        console.warn(`   ! could not read the terrain: ${(error as Error).message}`)
      }
    }
  }

  const diagnosis = analyse({ report, probe, opts: { fasterBy } })
  const bundle = await writeBundle(diagnosis, report, {
    resultsFile: RESULTS_DIR,
    timeoutMs,
    stamp: runStamp
  })
  await appendHistory(report.route.id, historyEntry(diagnosis, bundle.dir))

  console.log(`\n   ⚑ flagged: ${verdict.reasons[0] ?? 'debug always'}`)
  for (const reason of verdict.reasons.slice(1)) console.log(`     ${reason}`)
  if (diagnosis.planIdentical) console.log('     both engines planned the identical path, so this is execution, not search')
  for (const lead of diagnosis.leads.slice(0, 2)) console.log(`     → ${lead.replace(/`/g, '')}`)
  console.log(`     ${bundle.notes}`)

  if (values['debug-cmd'] !== '') {
    const cmd = `${values['debug-cmd']} ${JSON.stringify(bundle.notes)}`
    console.log(`     running: ${cmd}`)
    await new Promise<void>(resolve => {
      const proc = spawn(cmd, { shell: true, stdio: 'inherit' })
      proc.on('exit', () => resolve())
      proc.on('error', error => { console.warn(`     debug-cmd failed: ${error.message}`); resolve() })
    })
  }
}

/** Tell the world which route is about to run, and who is in it. */
function announceRoute (ref: Bot, route: Route, start: number[], end: number[], engines: string[]): void {
  ref.chat(`/say ── ${route.name} [${route.scenario}] · ${engines.join(' vs ')}`)
  ref.chat(`/say ${start.join(' ')} → ${end.join(' ')}  (${Math.round(Math.hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2]))} blocks apart)`)
  // Subtitle only, so the countdown that follows owns the big text and the
  // route name stays legible underneath it.
  try {
    ref.chat(`/title @a subtitle {"text":"${route.name.replace(/"/g, '')} · ${route.scenario}","color":"gray"}`)
  } catch { /* cosmetic */ }
}

function announce (ref: Bot, results: RunResult[]): void {
  const line = results
    .map(r => `${r.impl}: ${r.outcome} ${(r.wallMs / 1000).toFixed(1)}s`)
    .join('  |  ')
  try { ref.chat(`/say ${line}`) } catch { /* cosmetic */ }
}

/** `--from 91,145,150 --to 19,132,176` — race a route without editing the book. */
function parseBlock (text: string, flag: string): [number, number, number] {
  const parts = text.split(',').map(s => Number(s.trim()))
  if (parts.length !== 3 || parts.some(n => !Number.isFinite(n))) {
    throw new Error(`${flag} must be "x,y,z" (got "${text}")`)
  }
  return parts as [number, number, number]
}

async function main (): Promise<void> {
  const book = await loadRoutes()
  let routes: Route[] = []

  if (values.edit) {
    routes = [] // nothing runs until you ask for it in chat
  } else if (values.from !== '' || values.to !== '') {
    if (values.from === '' || values.to === '') throw new Error('--from and --to must be given together')
    const id = values['save-route'] !== '' ? values['save-route'] : 'adhoc'
    routes = [{
      id,
      name: 'ad-hoc',
      scenario: 'mixed',
      start: parseBlock(values.from, '--from'),
      end: parseBlock(values.to, '--to')
    }]
    if (values['save-route'] !== '') {
      if (book.routes.some(r => r.id === id)) throw new Error(`route ${id} already exists`)
      book.routes.push({ ...routes[0], name: id, notes: 'added with --save-route' })
      await saveRoutes(book)
      console.log(`saved route ${id} to routes.json`)
    }
  } else if (values.signs) {
    // Read the world directly. The server is not up yet, so nothing can have
    // been placed since the last save — no flush needed.
    const { routes: found, problems } = pairSigns(await scanSigns(WORLD_DIR))
    for (const p of problems) console.warn(`! ${p}`)
    if (found.length === 0) throw new Error('no complete !PF sign routes in the world — place Start/Finish signs with --edit')
    console.log(`${found.length} route(s) from !PF signs: ${found.map(r => r.id).join(', ')}`)
    routes = selectRoutes({ ...book, routes: found }, values.routes, values.scenario)
  } else {
    routes = selectRoutes(book, values.routes, values.scenario)
  }
  if (routes.length === 0 && !values.edit) throw new Error('no routes selected')

  let server: ServerHandle | null = null
  const children: Child[] = []
  let ref: Bot | null = null

  const shutdown = async (): Promise<void> => {
    for (const c of children) { try { c.send({ t: 'stop' }) } catch { /* gone */ } }
    await sleep(600)
    for (const c of children) c.kill()
    try { ref?.quit() } catch { /* gone */ }
    if (server !== null && !values.keep) await server.stop()
  }
  process.on('SIGINT', () => { void shutdown().then(() => process.exit(130)) })

  try {
    if (!values.attach) {
      // Rewrite the config first: ops.json has to list every racer, and the
      // lineup changes as engines are added.
      await writeServerConfig()
      console.log('starting arena server ...')
      server = await startServer()
    }
    ref = await connectReferee()
    deathsSince = watchDeaths(ref)
    await prepareWorld(ref, ARENA_RADIUS)
    console.log(`arena live at ${HOST}:${PORT} — join any time to watch`)

    const lineup = ([
      ['upstream', BOT_UPSTREAM],
      ['bulba', BOT_BULBA],
      ['bulba-wasm', BOT_BULBA_WASM],
      ['bulba-nohop', BOT_BULBA_NOHOP]
    ] as Array<[Impl, string]>).filter(([impl]) => values.engines === '' || values.engines.split(',').includes(impl))
    for (const [impl, username] of lineup) {
      const child = new Child(impl, username)
      await child.request({ t: 'init', impl, username, profile }, 'ready')
      children.push(child)
      console.log(`racer online: ${username} (${impl})`)
    }

    await waitForSpectators(ref, Number(values['wait-for']))

    const reports: RouteReport[] = []
    const summarize = async (): Promise<void> => {
      if (reports.length === 0) return
      console.log('')
      if (mode !== 'race') console.log(`\nplanning only (${solveRepeats} solves each)\n${solveTable(reports)}`)
      if (mode !== 'solve') console.log(`\nfull race\n${raceTable(reports)}\n\n${summarise(reports)}`)
      if (mode !== 'solve' && reports.every(r => r.race.every(x => x.outcome === 'no-path'))) {
        console.log('\nevery engine reported no-path: the goal block may not be stand-able. Retry with --tolerance 1.')
      }
      const path = await writeResults(reports, {
        world: book.world, worldDigest: book.worldDigest, profile, tolerance, timeoutMs, mode
      })
      console.log(`\nresults: ${path}`)
    }

    for (let attempt = 1; attempt <= repeats; attempt++) {
      for (const route of routes) {
        reports.push(await runRoute(ref, children, route, attempt))
      }
    }
    await summarize()

    // Stay in the world and take orders from chat: `--edit` starts here with
    // nothing run yet, `--keep` lands here after the batch.
    if (values.edit || values.keep) {
      await interactive(ref, children, reports, summarize)
    }
  } finally {
    await shutdown()
  }
}

/** Chat-driven session: author routes with signs and race them on demand. */
async function interactive (
  ref: Bot,
  children: Child[],
  reports: RouteReport[],
  summarize: () => Promise<void>
): Promise<void> {
  if (values.edit) equipEditorsOnJoin(ref)
  console.log(`\ninteractive on ${HOST}:${PORT} — say !help in chat (ctrl-c to stop)`)
  ref.chat('/say arena ready — say !help')

  let busy = false
  let done = false

  /** Look a route up by id or name, in the book first and then the world. */
  async function findRoutes (key: string): Promise<Route[]> {
    const current = await loadRoutes()
    if (key === 'all') {
      if (current.routes.length > 0) return current.routes
      return (await scanWorld(ref)).routes
    }
    const match = (r: Route): boolean =>
      r.id.toLowerCase() === key || r.name.toLowerCase() === key
    const fromBook = current.routes.filter(match)
    if (fromBook.length > 0) return fromBook
    return (await scanWorld(ref)).routes.filter(match)
  }

  const say = (text: string): void => { ref.chat(`/say ${text}`) }

  ref.on('chat', (username: string, message: string) => {
    if (!message.startsWith('!')) return
    const [cmd, ...args] = message.slice(1).trim().split(/\s+/)
    void handle(username, cmd.toLowerCase(), args).catch((error: Error) => say(`error: ${error.message}`))
  })

  async function handle (username: string, cmd: string, args: string[]): Promise<void> {
    switch (cmd) {
      case 'help':
        for (const line of EDIT_HELP) ref.chat(`/tell ${username} ${line}`)
        return
      case 'race': {
        if (busy) { say('a race is already running'); return }
        // `!race <name> tol=1` — some goal blocks cannot be stood on, and
        // then nothing ever arrives at tolerance 0.
        let override: number | undefined
        const words = args.filter(a => {
          const m = /^tol(?:erance)?=(\d+(?:\.\d+)?)$/i.exec(a)
          if (m === null) return true
          override = Number(m[1])
          return false
        })
        const key = (words.join(' ') || 'all').toLowerCase()
        const found = await findRoutes(key)
        if (found.length === 0) { say(`no route "${key}" — try !list or !scan`); return }
        busy = true
        try {
          for (const route of found) {
            for (let attempt = 1; attempt <= repeats; attempt++) {
              reports.push(await runRoute(ref, children, route, attempt, override))
            }
          }
          await summarize()
        } finally { busy = false }
        return
      }
      case 'tol': {
        // `!tol <id> <n>` — remember it on the route so you stop retyping it.
        const value = Number(args[args.length - 1])
        const key = args.slice(0, -1).join(' ').toLowerCase()
        if (!Number.isFinite(value) || key === '') { say('usage: !tol <route> <blocks>'); return }
        const current = await loadRoutes()
        const route = current.routes.find(r => r.id.toLowerCase() === key || r.name.toLowerCase() === key)
        if (route === undefined) { say(`no saved route ${key}`); return }
        route.tolerance = value
        await saveRoutes(current)
        say(`${route.id} tolerance = ${value}`)
        return
      }
      case 'edit': {
        // Any session can become an editing session. Without this the only
        // way to get out of spectator was to stop the run and restart it with
        // --edit, and a spectator's attempts to place a sign just vanish.
        equipEditor(ref, username)
        say(`${username} is in creative with signs — place a Start and a Finish, then !scan`)
        return
      }
      case 'scan': {
        say('scanning for !PF signs ...')
        const { routes: found, problems } = await scanWorld(ref)
        for (const p of problems) say(`! ${p}`)
        if (found.length === 0) {
          say('no complete !PF routes found — if your signs vanished as you placed them, you are a spectator: say !edit')
          return
        }
        for (const r of found) say(`${r.id} [${r.scenario}] ${r.start.join(' ')} -> ${r.end.join(' ')}`)
        say(`${found.length} route(s) — !save to keep, !race <name> to run one`)
        return
      }
      case 'save': {
        const { routes: found, problems } = await scanWorld(ref)
        for (const p of problems) say(`! ${p}`)
        if (found.length === 0) { say('no !PF routes in the world — nothing saved, routes.json untouched'); return }
        say(`saved ${await persistSignRoutes(found)} sign route(s) to routes.json`)
        return
      }
      case 'list': {
        const current = await loadRoutes()
        if (current.routes.length === 0) { say('no saved routes'); return }
        for (const r of current.routes) {
          ref.chat(`/tell ${username} ${r.id} [${r.scenario}] ${r.start.join(' ')} -> ${r.end.join(' ')} — ${r.name}`)
        }
        return
      }
      case 'tp': {
        const found = await findRoutes((args.join(' ') || '').toLowerCase())
        if (found.length === 0) { say(`no route ${args.join(' ')}`); return }
        const r = found[0]
        ref.chat(`/tp ${username} ${r.start[0] + 0.5} ${r.start[1] + 1} ${r.start[2] + 0.5}`)
        return
      }
      case 'drop': {
        const current = await loadRoutes()
        const key = (args.join(' ') || '').toLowerCase()
        const before = current.routes.length
        current.routes = current.routes.filter(r => r.id.toLowerCase() !== key && r.name.toLowerCase() !== key)
        if (current.routes.length === before) { say(`no route ${key}`); return }
        await saveRoutes(current)
        say(`dropped ${key}`)
        return
      }
      case 'debug': {
        // The terrain probe reads from the racer where it stands now, so this
        // is only truthful straight after a race — say so rather than quietly
        // filling the map with '?'.
        const last = reports[reports.length - 1]
        if (last === undefined) { say('nothing has raced yet'); return }
        if (busy) { say('a race is running'); return }
        busy = true
        try {
          say(`writing a debug bundle for ${last.route.id} ...`)
          await runDebug(children, last, true)
          say('bundle written, see the console')
        } finally { busy = false }
        return
      }
      case 'stop':
        say('stopping')
        done = true
        return
      default:
        say(`unknown command !${cmd} — try !help`)
    }
  }

  while (!done) await sleep(500)
}

await main()
