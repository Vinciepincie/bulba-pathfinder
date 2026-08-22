// Route scout: from one start block, which destinations are actually
// walkable-to? 2b2t spawn is mostly lava, void and unclimbable towers, so
// picking route endpoints by eye produces a lot of honest but useless
// no-path results.
//
//   npm run arena:probe -- --from 91,145,150 --min 40 --max 150
//   npm run arena:probe -- --from 91,145,150 --save r
//
// Uses the custom engine as the oracle (it settles no-path in a fraction of a
// second where upstream just burns its think budget), then prints — or saves —
// the destinations it can reach.
import { parseArgs } from 'node:util'
import { performance } from 'node:perf_hooks'
import { Vec3 } from 'vec3'
import { connectBot } from './arena.js'
import { startServer, type ServerHandle } from './server.js'
import { loadRoutes, saveRoutes, type Route } from './routes.js'
import { Racer } from './runner.js'
import { ARENA_RADIUS, BOT_BULBA, HOST, PORT } from './config.js'
import { table } from './report.js'

const { values } = parseArgs({
  options: {
    from: { type: 'string' },
    min: { type: 'string', default: '40' },
    max: { type: 'string', default: '150' },
    rings: { type: 'string', default: '4' },
    spokes: { type: 'string', default: '12' },
    think: { type: 'string', default: '5000' },
    save: { type: 'string', default: '' }, // id prefix; saves every reachable hit
    attach: { type: 'boolean', default: false }
  }
})
if (values.from === undefined) {
  console.error('usage: arena:probe -- --from x,y,z [--min 40] [--max 150] [--save r]')
  process.exit(2)
}

const from = (values.from).split(',').map(Number) as [number, number, number]
const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))

/** Lowest feet block at (x,z) with a solid floor and two blocks of headroom. */
function standableY (bot: ReturnType<typeof connectBot> extends Promise<infer B> ? B : never, x: number, z: number): number | null {
  for (let y = 250; y > 0; y--) {
    const floor = bot.blockAt(new Vec3(x, y, z))
    if (floor === null) return null
    if (floor.boundingBox !== 'block') continue
    const feet = bot.blockAt(new Vec3(x, y + 1, z))
    const head = bot.blockAt(new Vec3(x, y + 2, z))
    if (feet?.boundingBox === 'empty' && head?.boundingBox === 'empty') return y + 1
    return null // something solid sits on the first floor we found
  }
  return null
}

async function main (): Promise<void> {
  let server: ServerHandle | null = null
  if (!values.attach) {
    console.log('starting arena server ...')
    server = await startServer()
  }
  const bot = await connectBot(BOT_BULBA)
  const racer = await Racer.attach(bot, 'bulba', {
    extendedParkour: true,
    maxDropDown: 4,
    thinkTimeout: Number(values.think)
  })

  bot.chat(`/gamemode spectator ${BOT_BULBA}`)
  bot.chat(`/tp ${BOT_BULBA} ${from[0] + 0.5} ${from[1]} ${from[2] + 0.5}`)
  console.log(`probing from ${from.join(' ')} — waiting for chunks ...`)
  let lastChunk = Date.now()
  bot.on('chunkColumnLoad', () => { lastChunk = Date.now() })
  const deadline = Date.now() + 60_000
  while (Date.now() - lastChunk < 3000 && Date.now() < deadline) await sleep(200)
  bot.chat(`/gamemode survival ${BOT_BULBA}`)
  await sleep(500)

  const rings = Number(values.rings)
  const spokes = Number(values.spokes)
  const min = Number(values.min)
  const max = Number(values.max)
  const hits: Array<{ end: [number, number, number], dist: number, cost: number, ms: number, nodes: number }> = []
  const rows: string[][] = []

  for (let r = 0; r < rings; r++) {
    const dist = min + (max - min) * (rings === 1 ? 0 : r / (rings - 1))
    for (let s = 0; s < spokes; s++) {
      const angle = (2 * Math.PI * s) / spokes
      const x = Math.round(from[0] + Math.cos(angle) * dist)
      const z = Math.round(from[2] + Math.sin(angle) * dist)
      if (Math.abs(x) > ARENA_RADIUS || Math.abs(z) > ARENA_RADIUS) continue
      const y = standableY(bot, x, z)
      if (y === null) continue

      const t0 = performance.now()
      const solved = await racer.solveOnly([x, y, z], 1)
      const ms = performance.now() - t0
      rows.push([
        `${x} ${y} ${z}`,
        dist.toFixed(0),
        solved.status,
        `${ms.toFixed(0)}ms`,
        String(solved.visited),
        solved.cost === null ? '—' : solved.cost.toFixed(0)
      ])
      if (solved.status === 'success' && solved.cost !== null) {
        hits.push({ end: [x, y, z], dist, cost: solved.cost, ms, nodes: solved.nodes ?? 0 })
      }
    }
  }

  console.log(`\n${table(['destination', 'dist', 'status', 'solve', 'visited', 'cost'], rows)}`)
  console.log(`\n${hits.length} reachable of ${rows.length} standable samples`)

  if (values.save !== '' && hits.length > 0) {
    const book = await loadRoutes()
    let n = 0
    for (const hit of hits.sort((a, b) => a.cost - b.cost)) {
      let id: string
      do { id = `${values.save}${String(++n).padStart(2, '0')}` } while (book.routes.some(x => x.id === id))
      const route: Route = {
        id,
        name: `probe ${hit.dist.toFixed(0)}b`,
        scenario: 'mixed',
        start: from,
        end: hit.end,
        notes: `found by arena:probe; solver cost ${hit.cost.toFixed(0)}`
      }
      book.routes.push(route)
    }
    await saveRoutes(book)
    console.log(`saved ${hits.length} route(s) to routes.json`)
  }

  bot.quit()
  if (server !== null) await server.stop()
  process.exit(0)
}

console.log(`arena at ${HOST}:${PORT}`)
await main()
