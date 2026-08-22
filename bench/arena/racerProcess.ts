// One racer, one OS process.
//
// Each engine gets its own Node process on purpose. Upstream solves on the
// main thread in per-tick slices; the custom engine solves in a worker. Put
// both bots in one process and upstream's slices would stall the other bot's
// packet handling, so the headline wall-clock number would measure the
// harness instead of the engines.
import { Vec3 } from 'vec3'
import { connectBot } from './arena.js'
import { Racer, type Impl, type MovementProfile, type RunResult, type WorldProbe } from './runner.js'

type Block3 = [number, number, number]

type FromParent =
  | { t: 'init', impl: Impl, username: string, profile: MovementProfile }
  | { t: 'resolve', start: Block3, end: Block3 }
  | { t: 'solve', end: [number, number, number], repeats: number }
  | { t: 'solveWarm', end: [number, number, number], repeats: number }
  | { t: 'awaitStaged', start: Block3, end: Block3, timeoutMs: number, parked: boolean }
  | { t: 'race', routeId: string, end: Block3, tolerance: number, timeoutMs: number, startAt: number }
  | { t: 'probeWorld', focus: Array<{ pos: Block3, label: string }>, radius: number }
  | { t: 'stop' }

export type FromChild =
  | { t: 'ready' }
  | { t: 'resolved', start: Block3, end: Block3, startMoved: number, endMoved: number }
  | { t: 'solveResult', ms: number[], visited: number, cost: number | null, nodes: number | null, status: string }
  | { t: 'solveWarmResult', ms: number[] }
  | {
    t: 'staged'
    ok: boolean
    goalLoaded: boolean
    settled: boolean
    gameMode: string
    position: [number, number, number]
    food: number
    health: number
  }
  | { t: 'raceResult', result: RunResult }
  | { t: 'worldProbe', probe: WorldProbe }
  | { t: 'failed', message: string }

const send = (msg: FromChild): void => { process.send?.(msg) }

let racer: Racer | null = null

process.on('message', (msg: FromParent) => {
  handle(msg).catch((error: Error) => send({ t: 'failed', message: error.stack ?? error.message }))
})

async function handle (msg: FromParent): Promise<void> {
  switch (msg.t) {
    case 'init': {
      const bot = await connectBot(msg.username)
      racer = await Racer.attach(bot, msg.impl, msg.profile)
      send({ t: 'ready' })
      return
    }
    case 'resolve': {
      const bot = must().bot
      const start = snapToStandable(bot, msg.start)
      const end = snapToStandable(bot, msg.end)
      send({
        t: 'resolved',
        start: start ?? msg.start,
        end: end ?? msg.end,
        startMoved: start === null ? -1 : start[1] - msg.start[1],
        endMoved: end === null ? -1 : end[1] - msg.end[1]
      })
      return
    }
    case 'solve': {
      const r = await must().solveOnly(msg.end, msg.repeats)
      send({ t: 'solveResult', ...r })
      return
    }
    case 'solveWarm': {
      send({ t: 'solveWarmResult', ms: await must().solveWarm(msg.end, msg.repeats) })
      return
    }
    case 'awaitStaged': {
      const bot = must().bot
      const target = new Vec3(msg.start[0] + 0.5, msg.start[1], msg.start[2] + 0.5)
      const goal = new Vec3(msg.end[0], msg.end[1], msg.end[2])
      const deadline = Date.now() + msg.timeoutMs
      // A teleport streams in ~1000 chunks, and both engines drop the current
      // path on every chunk that lands. Starting before that settles means the
      // race measures chunk delivery, not pathfinding: the first run this way
      // produced 531 path resets and zero completed solves.
      let lastChunk = Date.now()
      const onChunk = (): void => { lastChunk = Date.now() }
      bot.on('chunkColumnLoad', onChunk)
      let ok = false
      let goalLoaded = false
      let settled = false
      while (Date.now() < deadline) {
        const p = bot.entity?.position
        // While parked in spectator the bot is deliberately above the start,
        // so only the chunk conditions apply.
        ok = msg.parked || (p !== undefined && p.distanceTo(target) < 1.2 && Math.abs(bot.entity.velocity.y) < 0.08)
        goalLoaded = bot.blockAt(goal) !== null
        settled = Date.now() - lastChunk > 2000
        if (ok && goalLoaded && settled) break
        await new Promise(resolve => setTimeout(resolve, 60))
      }
      bot.removeListener('chunkColumnLoad', onChunk)
      const p = bot.entity.position
      send({
        t: 'staged',
        ok,
        goalLoaded,
        settled,
        gameMode: bot.game?.gameMode ?? 'unknown',
        position: [p.x, p.y, p.z],
        food: bot.food,
        health: bot.health
      })
      return
    }
    case 'race': {
      const result = await must().race(msg.routeId, msg.end, msg.tolerance, msg.timeoutMs, msg.startAt)
      send({ t: 'raceResult', result })
      return
    }
    case 'probeWorld': {
      send({ t: 'worldProbe', probe: must().probeWorld(msg.focus, msg.radius) })
      return
    }
    case 'stop': {
      racer?.stop()
      racer?.bot.quit()
      setTimeout(() => process.exit(0), 250)
    }
  }
}

function must (): Racer {
  if (racer === null) throw new Error('racer process used before init')
  return racer
}

/**
 * Nudge a route endpoint onto a block a player could actually stand on.
 *
 * Coordinates get written down two ways — the block you are standing IN and
 * the block you are standing ON — and the r01 start turned out to be solid
 * cobblestone, which teleports the bot inside a wall and makes the server
 * rubber-band it forever. Searching outward from the authored y (nearest
 * first) fixes both conventions without changing which spot is meant. The
 * parent resolves once and hands the same answer to both engines, so this can
 * never favour one of them.
 */
function snapToStandable (bot: Racer['bot'], [x, y, z]: Block3): Block3 | null {
  const solidEnough = (dy: number): boolean => {
    const b = bot.blockAt(new Vec3(x, y + dy, z))
    return b !== null && b.boundingBox !== 'empty'
  }
  const clear = (dy: number): boolean => {
    const b = bot.blockAt(new Vec3(x, y + dy, z))
    return b !== null && b.boundingBox === 'empty'
  }
  for (let d = 0; d <= 5; d++) {
    for (const dy of d === 0 ? [0] : [d, -d]) {
      if (clear(dy) && clear(dy + 1) && solidEnough(dy - 1)) return [x, y + dy, z]
    }
  }
  return null
}
