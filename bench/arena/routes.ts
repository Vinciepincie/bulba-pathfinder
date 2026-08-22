// Route book for the arena. Coordinates are block positions in the trimmed
// 2b2t spawn world; `worldDigest` lets a run assert it is measuring the same
// terrain the routes were authored against.
import { readFile, writeFile } from 'node:fs/promises'
import { ROUTES_FILE } from './config.js'

export type Scenario = 'walk' | 'parkour-simple' | 'parkour-advanced' | 'mixed'

export interface Route {
  id: string
  name: string
  scenario: Scenario
  start: [number, number, number]
  end: [number, number, number]
  /**
   * Goal radius in blocks; 0 means the exact block. Some destinations simply
   * cannot be stood on — every engine ends a block short and nothing ever
   * "arrives" — so a route is allowed to carry its own tolerance rather than
   * making you remember a flag for it.
   */
  tolerance?: number
  notes?: string
}

export interface RouteBook {
  world: string
  worldDigest?: string
  scenarios: Scenario[]
  routes: Route[]
}

export async function loadRoutes (): Promise<RouteBook> {
  const book = JSON.parse(await readFile(ROUTES_FILE, 'utf8')) as RouteBook
  const seen = new Set<string>()
  for (const r of book.routes) {
    if (seen.has(r.id)) throw new Error(`duplicate route id ${r.id}`)
    seen.add(r.id)
    for (const key of ['start', 'end'] as const) {
      if (r[key].length !== 3 || r[key].some(n => !Number.isFinite(n))) {
        throw new Error(`route ${r.id}: ${key} must be [x, y, z]`)
      }
    }
  }
  return book
}

export async function saveRoutes (book: RouteBook): Promise<void> {
  await writeFile(ROUTES_FILE, `${JSON.stringify(book, null, 2)}\n`)
}

/** `--routes r01,r02` / `--scenario parkour-advanced`; empty selects all.
 *  Sign-defined routes are named by hand, so ids and names both match, and
 *  capitalisation never matters. */
export function selectRoutes (book: RouteBook, ids?: string, scenario?: string): Route[] {
  let routes = book.routes
  if (ids !== undefined && ids !== '') {
    const want = ids.split(',').map(s => s.trim().toLowerCase()).filter(s => s !== '')
    const matches = (r: Route, key: string): boolean =>
      r.id.toLowerCase() === key || r.name.toLowerCase() === key
    routes = routes.filter(r => want.some(key => matches(r, key)))
    const missing = want.filter(key => !routes.some(r => matches(r, key)))
    if (missing.length > 0) throw new Error(`unknown route(s): ${missing.join(', ')}`)
  }
  if (scenario !== undefined && scenario !== '') {
    routes = routes.filter(r => r.scenario === scenario)
  }
  return routes
}
