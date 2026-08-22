// Helpers for authoring routes in-world with signs. The session itself lives
// in race.ts, so editing and racing are the same long-lived process and
// `!race <id>` can run something you just placed.
import type { Bot } from 'mineflayer'
import { spectators } from './arena.js'
import { loadRoutes, saveRoutes, type Route } from './routes.js'
import { pairSigns, scanSigns, type PairedSigns } from './signs.js'
import { WORLD_DIR } from './config.js'

/** Marks routes that came from signs, so re-saving replaces rather than duplicates. */
export const SIGN_NOTE = 'defined by in-world !PF signs'

export const EDIT_HELP = [
  'Define a route with two signs:',
  '  line 1: !PF   line 2: <name>   line 3: Start (or Finish)   line 4: scenario (optional)',
  '!scan — re-read every !PF sign      !save — keep them in routes.json',
  '!race <name|id|all> [tol=1] — run it now    !list — saved routes',
  '!tol <route> <blocks> — remember a goal radius (0 = the exact block)',
  '!tp <id> — go to a route start      !drop <id> — delete a saved route',
  '!stop — end the session'
]

const sleep = async (ms: number): Promise<void> => await new Promise(resolve => setTimeout(resolve, ms))

/** Flush the world so freshly placed signs are on disk, then read them. */
export async function scanWorld (ref: Bot): Promise<PairedSigns> {
  ref.chat('/save-all flush')
  await sleep(2500)
  return pairSigns(await scanSigns(WORLD_DIR))
}

/** Replace the sign-defined routes in the book, leaving hand-written ones. */
export async function persistSignRoutes (routes: Route[]): Promise<number> {
  const book = await loadRoutes()
  book.routes = [...book.routes.filter(r => r.notes !== SIGN_NOTE), ...routes.map(r => ({ ...r, notes: SIGN_NOTE }))]
  await saveRoutes(book)
  return routes.length
}

/**
 * Everyone joins as a spectator because that is what a race needs; an editor
 * needs to build, so hand out creative and signs on arrival.
 */
export function equipEditorsOnJoin (ref: Bot): NodeJS.Timeout {
  const equipped = new Set<string>()
  const timer = setInterval(() => {
    for (const name of spectators(ref)) {
      if (equipped.has(name)) continue
      equipped.add(name)
      ref.chat(`/gamemode creative ${name}`)
      ref.chat(`/give ${name} minecraft:oak_sign 64`)
      for (const line of EDIT_HELP) ref.chat(`/tell ${name} ${line}`)
    }
  }, 2000)
  timer.unref()
  return timer
}
