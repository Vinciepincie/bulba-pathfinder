// Helpers for authoring routes in-world with signs. The session itself lives
// in race.ts, so editing and racing are the same long-lived process and
// `!race <id>` can run something you just placed.
import type { Bot } from 'mineflayer'
import { spectators } from './arena.js'
import { loadRoutes, mergeSignRoutes, saveRoutes, type Route } from './routes.js'
import { pairSigns, scanSigns, type PairedSigns } from './signs.js'
import { WORLD_DIR } from './config.js'

/** Marks routes that came from signs, so re-saving replaces rather than duplicates. */
export const SIGN_NOTE = 'defined by in-world !PF signs'

export const EDIT_HELP = [
  'Define a route with two signs:',
  '  line 1: !PF   line 2: <name>   line 3: Start (or Finish)   line 4: scenario (optional)',
  '!edit — creative + a stack of signs (you join as a spectator and cannot build)',
  '!scan — re-read every !PF sign      !save — keep them in routes.json',
  '!race <name|id|all> [tol=1] — run it now    !list — saved routes',
  '!tol <route> <blocks> — remember a goal radius (0 = the exact block)',
  '!tp <id> — go to a route start      !drop <id> — delete a saved route',
  '!debug — write a debug bundle for the last race, flagged or not',
  '!stop — end the session'
]

const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))

/**
 * Push the world to disk and wait until the server says it landed.
 *
 * The scan reads region files, so a sign placed a moment ago is only visible
 * once the server has written it. A fixed sleep was a guess in both
 * directions: too short on a big world and the scan reports no signs, too long
 * and every `!scan` stalls. `/save-all flush` announces "Saved the game" when
 * it is done, and the referee is opped, so it can just wait for that.
 */
export async function flushWorld (ref: Bot, timeoutMs = 20000): Promise<boolean> {
  const saved = await new Promise<boolean>(resolve => {
    const onMessage = (msg: { toString: () => string }): void => {
      if (/saved the game/i.test(msg.toString())) finish(true)
    }
    const timer = setTimeout(() => finish(false), timeoutMs)
    const finish = (ok: boolean): void => {
      clearTimeout(timer)
      ref.removeListener('message', onMessage)
      resolve(ok)
    }
    ref.on('message', onMessage)
    ref.chat('/save-all flush')
  })
  // The region bytes land just behind the announcement.
  await sleep(500)
  return saved
}

/** Flush the world so freshly placed signs are on disk, then read them. */
export async function scanWorld (ref: Bot): Promise<PairedSigns> {
  await flushWorld(ref)
  return pairSigns(await scanSigns(WORLD_DIR))
}

/** Fold the scanned signs into the book without dropping anything else. */
export async function persistSignRoutes (routes: Route[]): Promise<number> {
  const book = await loadRoutes()
  book.routes = mergeSignRoutes(book.routes, routes, SIGN_NOTE)
  await saveRoutes(book)
  return routes.length
}

/** Creative plus a stack of signs: what it takes to author a route. */
export function equipEditor (ref: Bot, name: string): void {
  ref.chat(`/gamemode creative ${name}`)
  ref.chat(`/give ${name} minecraft:oak_sign 64`)
  for (const line of EDIT_HELP) ref.chat(`/tell ${name} ${line}`)
}

/**
 * Everyone joins as a spectator because that is what a race needs, and a
 * spectator cannot place a block — the sign appears for a tick and the server
 * takes it straight back. An editor needs to build, so hand out creative and
 * signs on arrival.
 */
export function equipEditorsOnJoin (ref: Bot): NodeJS.Timeout {
  const equipped = new Set<string>()
  const timer = setInterval(() => {
    for (const name of spectators(ref)) {
      if (equipped.has(name)) continue
      equipped.add(name)
      equipEditor(ref, name)
    }
  }, 2000)
  timer.unref()
  return timer
}
