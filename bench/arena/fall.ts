// Live fall-recovery checks on the arena server: knock the bot off a pillar (or pull its floor, or
// plan a drop) with set items and modes, and read back health, death and the plugin's fall events.
//   npx tsx bench/arena/fall.ts [--attach] [--only id,id] [--repeat n]
// Lanes are built at runtime in empty sky (x -672..-449, z -688..-641, y 179..262) and rebuilt per run.
import { parseArgs } from 'node:util'
import { performance } from 'node:perf_hooks'
import type { Bot } from 'mineflayer'
import { Vec3 } from 'vec3'
import { createPathfinder, Movements, goals } from 'bulba-pathfinder'
import type { FallInfo, FallOutcome } from 'bulba-pathfinder'
import { connectBot, connectReferee, prepareWorld, watchDeaths, spectators } from './arena.js'
import { startServer, writeServerConfig } from './server.js'
import type { ServerHandle } from './server.js'
import { ARENA_RADIUS, BOT_BULBA } from './config.js'

const { values } = parseArgs({
  options: {
    attach: { type: 'boolean', default: false },
    only: { type: 'string', default: '' },
    repeat: { type: 'string', default: '1' },
    // Wait for a player to join, bring them to each lane, and pause before each fall.
    watch: { type: 'boolean', default: false }
  }
})

const FLOOR_Y = 180
const LANE_Z = -664
const laneX = (i: number): number => -660 + 24 * i

interface Scenario {
  id: string
  /** Height of the pillar top above the floor's stand level. */
  height: number
  items: Array<[string, string, number?]>
  mode: 'observe' | 'recover'
  trigger: 'knock' | 'pull' | 'planned'
  /** For 'planned': knock the bot sideways once it is in the air. */
  knockMid?: boolean
  /** Extra build commands, origin-relative: {x} {y} {z} are the lane origin (floor stand level). */
  build?: string[]
  note: string
  expect: { maxDamage?: number, minDamage?: number, survive?: boolean, events?: string[], noEvents?: string[] }
}

const SCENARIOS: Scenario[] = [
  { id: 'water30', height: 30, items: [['hotbar.3', 'water_bucket']], mode: 'recover', trigger: 'knock', note: 'knocked off 30 up, water bucket in the hotbar', expect: { maxDamage: 0, survive: true, events: ['fall_start', 'fall_finish'] } },
  { id: 'water8', height: 8, items: [['hotbar.3', 'water_bucket']], mode: 'recover', trigger: 'knock', note: 'knocked off 8 up: the landing move is under a block, no fast-fall ray', expect: { maxDamage: 0, survive: true } },
  { id: 'waterInv', height: 30, items: [['inventory.5', 'water_bucket']], mode: 'recover', trigger: 'knock', note: 'bucket in the main inventory: one number-key swap mid-air', expect: { maxDamage: 0, survive: true } },
  { id: 'cod40', height: 40, items: [['hotbar.1', 'cod_bucket']], mode: 'recover', trigger: 'knock', note: 'a fish bucket places water too', expect: { maxDamage: 0, survive: true } },
  { id: 'hay30', height: 30, items: [['hotbar.0', 'hay_block', 4]], mode: 'recover', trigger: 'knock', note: 'hay keeps a fifth of the damage', expect: { minDamage: 4, maxDamage: 6, survive: true } },
  { id: 'web30', height: 30, items: [['hotbar.0', 'cobweb', 4]], mode: 'recover', trigger: 'knock', note: 'cobweb cancels the fall', expect: { maxDamage: 0, survive: true } },
  { id: 'pool36', height: 36, items: [], mode: 'recover', trigger: 'knock', build: ['/fill {x+3} {y} {z-2} {x+6} {y} {z+2} minecraft:water'], note: 'no items, a pool a few blocks past the natural landing: steer into it', expect: { maxDamage: 0, survive: true } },
  { id: 'pull25', height: 25, items: [['hotbar.3', 'water_bucket']], mode: 'recover', trigger: 'pull', note: 'the pillar top removed under an idle bot', expect: { maxDamage: 0, survive: true } },
  { id: 'observe30', height: 30, items: [['hotbar.3', 'water_bucket']], mode: 'observe', trigger: 'knock', note: 'observe mode: reports, uses nothing, dies', expect: { survive: false, events: ['fall_start', 'fall_lethal'] } },
  { id: 'totem40', height: 40, items: [['hotbar.6', 'totem_of_undying']], mode: 'recover', trigger: 'knock', note: 'no breaker, a totem in the hotbar: held before landing', expect: { survive: true, events: ['fall_start', 'fall_lethal'] } },
  { id: 'plannedKnock', height: 30, items: [['hotbar.4', 'water_bucket'], ['hotbar.5', 'cobweb', 2]], mode: 'recover', trigger: 'planned', knockMid: true, note: 'a planned clutch drop knocked sideways mid-air: recovery takes over', expect: { maxDamage: 0, survive: true } },
  { id: 'planned25', height: 25, items: [['hotbar.4', 'water_bucket']], mode: 'recover', trigger: 'planned', note: 'goto the floor below: a planned clutch drop', expect: { maxDamage: 0, survive: true, noEvents: ['fall_start', 'fall_lethal'] } }
]

const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))
/** A player that has just joined or respawned ignores damage for 60 ticks: wait that out before a knock. */
let lastSpawn = performance.now()

function fillTemplate (cmd: string, o: Vec3): string {
  return cmd.replace(/\{([xyz])([+-]\d+)?\}/g, (_m, axis: 'x' | 'y' | 'z', off?: string) => String(o[axis] + Number(off ?? 0)))
}

async function main (): Promise<void> {
  let server: ServerHandle | null = null
  if (!values.attach) {
    await writeServerConfig()
    console.log('starting arena server ...')
    server = await startServer()
  }
  const ref = await connectReferee()
  const deaths = watchDeaths(ref)
  await prepareWorld(ref, ARENA_RADIUS)
  ref.chat('/forceload add -672 -688 -449 -641')
  const bot = await connectBot(BOT_BULBA)
  bot.loadPlugin(createPathfinder({ useWorkerThreads: true }) as never)
  bot.on('spawn', () => { lastSpawn = performance.now() })
  const pf = (bot as unknown as { pathfinder: any }).pathfinder
  // A joining player ignores damage until the server counts it loaded: poke until a hit lands, then heal.
  ref.chat(`/gamemode survival ${bot.username}`)
  for (let t = 0; t < 40; t++) {
    const before = bot.health
    ref.chat(`/damage ${bot.username} 0.5 minecraft:generic`)
    await sleep(500)
    if (bot.health < before) break
  }
  ref.chat(`/effect give ${bot.username} minecraft:instant_health 1 5 true`)
  await sleep(500)
  if (values.watch) {
    console.log('waiting for you to join ...')
    while (spectators(ref).length === 0) await sleep(1000)
    await sleep(3000)
  }
  const results: string[] = []
  const only = values.only === '' ? null : new Set(values.only.split(','))
  const list = SCENARIOS.filter(s => only === null || only.has(s.id))
  const repeat = Number(values.repeat)
  try {
    for (let rep = 0; rep < repeat; rep++) {
      for (const [i, sc] of list.entries()) {
        const line = await runScenario(ref, bot, pf, sc, new Vec3(laneX(i % 9), FLOOR_Y, LANE_Z), deaths)
        results.push(line)
        console.log(line)
        if (values.watch) {
          ref.chat(`/title @a actionbar {"text":"${line.slice(0, 90).replace(/"/g, "'")}","color":"white"}`)
          await sleep(3000)
        }
      }
    }
  } finally {
    console.log('\n' + results.join('\n'))
    try { ref.chat('/forceload remove -672 -688 -449 -641') } catch {}
    bot.quit()
    ref.quit()
    if (server !== null) await server.stop()
  }
}

async function runScenario (ref: Bot, bot: Bot, pf: any, sc: Scenario, o: Vec3, deaths: (since: number) => Map<string, string>): Promise<string> {
  const name = bot.username
  const top = o.y + sc.height
  // The lane: clear it, a floor, a one-block pillar.
  // Wide enough that steering cannot carry the bot past the edge (a 30-block fall reaches ~6 sideways).
  ref.chat(`/fill ${o.x - 8} ${o.y - 1} ${o.z - 8} ${o.x + 12} ${o.y + 1} ${o.z + 8} minecraft:air`)
  ref.chat(`/fill ${o.x - 8} ${o.y + 2} ${o.z - 8} ${o.x + 12} ${top + 3} ${o.z + 8} minecraft:air`)
  ref.chat(`/fill ${o.x - 8} ${o.y - 1} ${o.z - 8} ${o.x + 12} ${o.y - 1} ${o.z + 8} minecraft:stone`)
  ref.chat(`/fill ${o.x} ${o.y} ${o.z} ${o.x} ${top - 1} ${o.z} minecraft:stone`)
  for (const c of sc.build ?? []) ref.chat(fillTemplate(c, o))
  // The bot: survival, healed and fed, its items only.
  ref.chat(`/gamemode survival ${name}`)
  ref.chat(`/clear ${name}`)
  ref.chat(`/effect clear ${name}`)
  for (const [slot, item, count] of sc.items) ref.chat(`/item replace entity ${name} ${slot} with minecraft:${item} ${count ?? 1}`)
  ref.chat(`/tp ${name} ${o.x + 0.5} ${top} ${o.z + 0.5} -90 0`)
  ref.chat(`/tp ${ref.username} ${o.x + 8} ${top + 4} ${o.z + 8}`)
  await sleep(1500)
  // The lane's chunks and blocks have to be on the client before anything moves.
  for (let t = 0; t < 100; t++) {
    const floor = bot.blockAt(new Vec3(o.x + 2, o.y - 1, o.z))
    const pillar = bot.blockAt(new Vec3(o.x, top - 1, o.z))
    if (floor?.name === 'stone' && pillar?.name === 'stone' && bot.entity.onGround === true) break
    await sleep(100)
  }
  ref.chat(`/effect give ${name} minecraft:instant_health 1 5 true`)
  ref.chat(`/effect give ${name} minecraft:saturation 1 255 true`)
  for (let t = 0; t < 50 && (bot.health < 20 || bot.entity.onGround !== true); t++) await sleep(100)
  bot.setQuickBarSlot(0)
  pf.fallRecovery.mode = sc.mode
  const mv = new Movements(bot)
  mv.maxClutchDropDown = sc.trigger === 'planned' ? 60 : 0
  pf.setMovements(mv)
  await sleep(300)
  if (values.watch) {
    // Beside the pillar, a little above its top, looking at it.
    for (const s of spectators(ref)) ref.chat(`/tp ${s} ${o.x + 7} ${top + 3} ${o.z + 9} facing ${o.x + 2} ${o.y + 8} ${o.z}`)
    ref.chat(`/title @a times 5 60 10`)
    ref.chat(`/title @a title {"text":"${sc.id}","color":"yellow"}`)
    ref.chat(`/title @a subtitle {"text":"${sc.note.replace(/"/g, "'")}","color":"gray"}`)
    await sleep(4000)
  }

  const events: Array<[string, number, unknown]> = []
  const t0 = performance.now()
  const onEv = (n: string) => (p: unknown) => { events.push([n, performance.now() - t0, p]) }
  const handlers: Array<[string, (p: unknown) => void]> = ['fall_start', 'fall_lethal', 'fall_finish'].map(n => [`pathfinder:${n}`, onEv(n)])
  for (const [ev, h] of handlers) bot.on(ev as never, h as never)
  let minHealth = bot.health
  const onHealth = (): void => { minHealth = Math.min(minHealth, bot.health) }
  bot.on('health', onHealth)
  let died = false
  const onDeath = (): void => { died = true }
  bot.on('death', onDeath)
  const start = bot.health
  const deathSince = Date.now()
  let totemPopped = false
  const onStatus = (p: { entityId: number, entityStatus: number }): void => { if (p.entityId === bot.entity.id && p.entityStatus === 35) totemPopped = true }
  ;(bot as unknown as { _client: { on: (e: string, f: (p: never) => void) => void } })._client.on('entity_status', onStatus as never)

  const sinceSpawn = performance.now() - lastSpawn
  if (sinceSpawn < 3500) await sleep(3500 - sinceSpawn)
  let knockHp = 0
  if (sc.trigger === 'knock') {
    knockHp = 0.5
    ref.chat(`/damage ${name} 0.5 minecraft:mob_attack at ${o.x - 0.5} ${top} ${o.z + 0.5}`)
    // Did it land? A knock that never arrives would read as a clean landing.
    for (let t = 0; t < 30 && bot.health >= start; t++) await sleep(50)
    if (bot.health >= start) console.log(`  [${sc.id}] the knock did not register (health ${bot.health})`)
  } else if (sc.trigger === 'pull') {
    // The whole pillar: the bot was standing on a column, removing its top alone drops it one block.
    ref.chat(`/fill ${o.x} ${o.y} ${o.z} ${o.x} ${top - 1} ${o.z} minecraft:air`)
  } else {
    void pf.goto(new goals.GoalBlock(o.x + 3, o.y, o.z)).catch(() => {})
    if (sc.knockMid === true) {
      for (let t = 0; t < 100 && bot.entity.position.y > top - 3; t++) await sleep(20)
      const q = bot.entity.position
      ref.chat(`/damage ${name} 0.5 minecraft:mob_attack at ${q.x.toFixed(2)} ${q.y.toFixed(2)} ${(q.z - 1).toFixed(2)}`)
      knockHp = 0.5
    }
  }
  // Wait for the fall to finish: the landing, plus time for the server's damage.
  let landedAt = -1
  for (let t = 0; t < 300; t++) {
    await sleep(50)
    if (died) break
    const supported = bot.entity.onGround === true || (bot.entity as { isInWater?: boolean }).isInWater === true
    const done = sc.trigger === 'planned' ? !pf.isMoving() && supported && bot.entity.position.y < top - 2 : supported && bot.entity.position.y < top - 2
    if (done && landedAt < 0) landedAt = t
    if (landedAt >= 0 && t - landedAt > 30) break
  }
  for (const [ev, h] of handlers) bot.removeListener(ev as never, h as never)
  bot.removeListener('health', onHealth)
  bot.removeListener('death', onDeath)
  const deathMsg = deaths(deathSince).get(name) ?? ''
  const damage = Math.max(0, Math.round((start - minHealth - knockHp) * 100) / 100)
  const names = events.map(e => e[0])
  const lethal = events.find(e => e[0] === 'fall_lethal')
  const finish = events.find(e => e[0] === 'fall_finish')?.[2] as FallOutcome | undefined
  const startInfo = events.find(e => e[0] === 'fall_start')?.[2] as FallInfo | undefined
  const bucket = bot.inventory.items().map(i => i.name).filter(n => n.endsWith('bucket'))
  // Verdict against the expectation.
  const e = sc.expect
  const fails: string[] = []
  if (e.survive === true && died) fails.push('died')
  if (e.survive === false && !died) fails.push('survived')
  if (!died && e.maxDamage !== undefined && damage > e.maxDamage) fails.push(`damage ${damage} > ${e.maxDamage}`)
  if (!died && e.minDamage !== undefined && damage < e.minDamage) fails.push(`damage ${damage} < ${e.minDamage}`)
  for (const ev of e.events ?? []) if (!names.includes(ev)) fails.push(`no ${ev}`)
  for (const ev of e.noEvents ?? []) if (names.includes(ev)) fails.push(`unexpected ${ev}`)
  if (died) {
    ref.chat(`/gamemode spectator ${name}`)
    await sleep(1500)
  }
  const L = startInfo?.plan.landing
  const plan = startInfo === undefined ? '-' : `${startInfo.plan.kind}${startInfo.plan.item !== null ? ':' + startInfo.plan.item : ''}->${L?.medium}@${L?.x.toFixed(1)},${L?.y.toFixed(1)},${L?.z.toFixed(1)} pred ${Number.isFinite(startInfo.unaided.loss) ? startInfo.unaided.loss.toFixed(1) : startInfo.unaided.loss}`
  const p = bot.entity.position
  const feet = bot.blockAt(p)?.name ?? '?'
  let water = 0
  for (let x = o.x - 8; x <= o.x + 12; x++) for (let z = o.z - 8; z <= o.z + 8; z++) if (bot.blockAt(new Vec3(x, o.y, z))?.name === 'water') water++
  const where = `end ${p.x.toFixed(1)},${p.y.toFixed(1)},${p.z.toFixed(1)} in ${feet} water ${water}`
  return [
    fails.length === 0 ? 'PASS' : 'FAIL',
    sc.id.padEnd(10),
    `dmg ${String(damage).padEnd(5)}`,
    died ? `DIED(${deathMsg})` : 'alive',
    `events ${names.join(',') || '-'}`,
    `plan ${plan}`,
    lethal !== undefined ? `lethal@${Math.round(lethal[1])}ms` : '',
    finish !== undefined ? `clutched ${String(finish.clutched)} pickedUp ${String(finish.pickedUp)}` : '',
    totemPopped ? 'TOTEM' : '',
    `buckets ${bucket.join(',') || '-'}`,
    where,
    fails.length > 0 ? `<- ${fails.join('; ')}` : ''
  ].filter(s => s !== '').join('  ') + `   (${sc.note})`
}

main().catch(err => {
  console.error(err)
  process.exit(1)
})
