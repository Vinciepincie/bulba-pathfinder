// The referee: an opped spectator bot that owns the world state, stages each
// route, and starts both racers together.
//
// It never pathfinds. Keeping every command on a third connection means the
// two engines under test are only ever asked to do one thing — walk.
import mineflayer, { type Bot } from 'mineflayer'
import { BOT_NAMES, BOT_REFEREE, HOST, MC_VERSION, PORT, VIEW_DISTANCE } from './config.js'

export async function connectBot (username: string, viewDistance: number = VIEW_DISTANCE): Promise<Bot> {
  const bot = mineflayer.createBot({
    host: HOST,
    port: PORT,
    username,
    auth: 'offline',
    version: MC_VERSION,
    viewDistance,
    checkTimeoutInterval: 120_000
  })
  await new Promise<void>((resolve, reject) => {
    bot.once('spawn', resolve)
    bot.once('error', reject)
    bot.once('kicked', reason => reject(new Error(`${username} kicked: ${JSON.stringify(reason)}`)))
  })
  return bot
}

/** The referee never walks, so it takes the smallest possible chunk feed —
 *  every chunk it loads is bandwidth stolen from the two bots under test. */
export async function connectReferee (): Promise<Bot> {
  return await connectBot(BOT_REFEREE, 2)
}

const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))

/**
 * Freeze everything that could make two runs of the same route differ:
 * daylight, weather, fire spread, mobs, random ticks. Also puts each racer on
 * its own no-collision team so they can share a start block without shoving
 * each other, and draws the world border at the trimmed edge so nobody walks
 * into freshly generated terrain.
 */
export async function prepareWorld (ref: Bot, radius: number): Promise<void> {
  // ⚠ 1.21.11 renamed every gamerule to snake_case, and not mechanically:
  // doDaylightCycle is now advance_time, doInsomnia is spawn_phantoms,
  // announceAdvancements is show_advancement_messages. The old camelCase names
  // are rejected outright ("Incorrect argument for command"), which the arena
  // never noticed because nothing checks command replies — so the world was
  // never actually frozen: daylight cycled, weather changed, mobs spawned and
  // random ticks ran through every benchmark run to date.
  const rules: Array<[string, string]> = [
    ['advance_time', 'false'], // was doDaylightCycle
    ['advance_weather', 'false'], // was doWeatherCycle
    ['spawn_mobs', 'false'], // was doMobSpawning
    ['spawn_monsters', 'false'],
    ['spawn_patrols', 'false'], // was doPatrolSpawning
    ['spawn_wandering_traders', 'false'], // was doTraderSpawning
    ['spawn_phantoms', 'false'], // was doInsomnia
    ['spawn_wardens', 'false'],
    ['raids', 'false'], // was disableRaids (note: inverted sense)
    ['mob_griefing', 'false'],
    ['block_drops', 'false'], // was doTileDrops
    ['mob_drops', 'false'],
    ['random_tick_speed', '0'],
    ['keep_inventory', 'true'],
    ['immediate_respawn', 'true'], // was doImmediateRespawn
    ['show_death_messages', 'true'],
    ['show_advancement_messages', 'false'], // was announceAdvancements
    ['spectators_generate_chunks', 'false'],
    ['log_admin_commands', 'false'],
    ['fall_damage', 'true'],
    ['drowning_damage', 'true'],
    ['fire_damage', 'true'],
    // No direct successor to doFireTick; 0 stops fire spreading near players,
    // which is what mattered for a lava-heavy world staying identical between
    // runs.
    ['fire_spread_radius_around_player', '0'],
    ['tnt_explodes', 'false']
  ]
  for (const [rule, value] of rules) ref.chat(`/gamerule ${rule} ${value}`)
  ref.chat('/difficulty peaceful')
  ref.chat('/time set noon')
  ref.chat('/weather clear 1000000')
  ref.chat('/worldborder center 0 0')
  ref.chat(`/worldborder set ${radius * 2}`)
  ref.chat('/worldborder warning distance 0')
  ref.chat('/worldborder damage amount 0')
  // Scenery (item frames, paintings, armor stands) is part of the terrain and
  // stays; anything else would drift between runs.
  ref.chat('/kill @e[type=!minecraft:player,type=!minecraft:item_frame,type=!minecraft:painting,type=!minecraft:armor_stand]')

  // One team per racer, all with collision off, so they can share a start
  // block without shoving each other. The colours are how a spectator tells
  // them apart once they diverge.
  const colours = ['red', 'green', 'aqua', 'gray']
  for (const [i, member] of BOT_NAMES.entries()) {
    const team = `pf${i}`
    ref.chat(`/team add ${team}`)
    ref.chat(`/team modify ${team} collisionRule never`)
    ref.chat(`/team modify ${team} color ${colours[i] ?? 'white'}`)
    ref.chat(`/team join ${team} ${member}`)
  }
  await sleep(500)
}

export function yawTo (from: number[], to: number[]): number {
  // Minecraft yaw: 0 = +Z (south), rotating towards -X (west) as it grows.
  return -Math.atan2(to[0] - from[0], to[2] - from[2]) * 180 / Math.PI
}

/** Names of every connected player that is not one of our bots. */
export function spectators (ref: Bot): string[] {
  return Object.keys(ref.players).filter(n => !BOT_NAMES.includes(n))
}

/**
 * Park the racers in spectator just above the start while the chunks stream
 * in. A spectator does not collide, fall or get position-corrected, so the
 * ~1000 chunks a teleport pulls cannot rubber-band anyone — and the endpoints
 * can be inspected before anyone has to stand on them.
 */
export async function parkAbove (ref: Bot, racers: string[], start: [number, number, number]): Promise<void> {
  const [x, y, z] = start
  for (const name of racers) {
    ref.chat(`/gamemode spectator ${name}`)
    ref.chat(`/tp ${name} ${x + 0.5} ${y + 6} ${z + 0.5}`)
  }
  await sleep(300)
}

/**
 * Put both racers on the start block (same block, same facing) in survival
 * with hunger topped up, and bring every human along to watch.
 */
export async function stageRoute (
  ref: Bot,
  racers: string[],
  start: [number, number, number],
  end: [number, number, number]
): Promise<void> {
  const yaw = yawTo(start, end).toFixed(1)
  const [x, y, z] = start
  for (const name of racers) {
    ref.chat(`/gamemode survival ${name}`)
    ref.chat(`/effect clear ${name}`)
    // Sprint-jumps need food > 6. A long benchmark would otherwise starve the
    // bots into a walk halfway through and silently change the comparison.
    ref.chat(`/effect give ${name} minecraft:saturation 1 255 true`)
    ref.chat(`/tp ${name} ${x + 0.5} ${y} ${z + 0.5} ${yaw} 0`)
  }
  for (const name of spectators(ref)) {
    ref.chat(`/tp ${name} ${x + 0.5} ${y + 4} ${z + 0.5} ${yaw} 20`)
  }
  // The referee waits at the finish line, facing back down the route: it
  // marks the destination for anyone watching, and gives spectators a
  // second vantage point to teleport to mid-race.
  ref.chat(`/tp ${BOT_REFEREE} ${end[0] + 0.5} ${end[1] + 2} ${end[2] + 0.5} ${yawTo(end, start).toFixed(1)} 0`)
  await sleep(400)
}

export async function countdown (ref: Bot, seconds: number, label: string): Promise<void> {
  if (seconds <= 0) return
  try {
    ref.chat('/title @a times 0 25 5')
    ref.chat(`/title @a actionbar {"text":"${label}","color":"gray"}`)
  } catch { /* cosmetic only */ }
  for (let i = seconds; i > 0; i--) {
    try { ref.chat(`/title @a title {"text":"${i}","color":"yellow"}`) } catch { /* cosmetic */ }
    await sleep(1000)
  }
  try { ref.chat('/title @a title {"text":"GO","color":"green"}') } catch { /* cosmetic */ }
}

/**
 * Collect vanilla death messages for the racers.
 *
 * mineflayer's `death` event carries no cause, and "died" on its own is not a
 * useful benchmark result — "drowned" and "fell from a high place" point at
 * completely different gaps. The server broadcasts the reason to chat, so the
 * referee just reads it. Returns a lookup for messages since a timestamp.
 */
export function watchDeaths (ref: Bot): (since: number) => Map<string, string> {
  const log: Array<{ at: number, name: string, text: string }> = []
  ref.on('message', (msg: { toString: () => string }) => {
    const text = msg.toString().trim()
    // Death messages are broadcast as "<player> <what happened>"; command
    // echoes and /say output are bracketed, so they never match.
    const name = BOT_NAMES.find(n => text.startsWith(`${n} `))
    if (name === undefined || text.includes('issued server command')) return
    // Stems only, with no trailing word boundary: vanilla conjugates these
    // ("drowned", "burned"), and a closing \b silently dropped every one of
    // them — the first run captured "was doomed to fall" and missed two
    // drownings.
    if (!/\b(die|slain|shot|blown|fell|fall|burn|lava|drown|suffocat|starv|wither|prick|squash|kinetic|doom|impaled|skewered|froze|discovered the floor|walked into|went off|removed from|out of the world)/i.test(text)) return
    log.push({ at: Date.now(), name, text })
  })
  return since => {
    const found = new Map<string, string>()
    for (const e of log) if (e.at >= since) found.set(e.name, e.text)
    return found
  }
}

/** Server tick health, via the vanilla `/tick query` readout. Best effort. */
export async function tickHealth (ref: Bot): Promise<number | null> {
  return await new Promise(resolve => {
    const timer = setTimeout(() => { ref.removeListener('message', onMessage); resolve(null) }, 1500)
    const onMessage = (msg: { toString: () => string }): void => {
      // The server formats this number with the host locale's decimal mark.
      const m = /average time per tick:?\s*([\d.,]+)\s*ms/i.exec(msg.toString())
      if (m === null) return
      clearTimeout(timer)
      ref.removeListener('message', onMessage)
      resolve(Number(m[1].replace(',', '.')))
    }
    ref.on('message', onMessage)
    ref.chat('/tick query')
  })
}
