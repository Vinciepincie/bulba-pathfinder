// Routes defined by signs placed in the world.
//
//   line 1: !PF          (case insensitive)
//   line 2: <route name>
//   line 3: Start | Finish
//   line 4: <scenario>   (optional: walk | parkour-simple | parkour-advanced)
//
// Place a Start and a Finish sign with the same name and the pair becomes a
// route. Defining a benchmark by standing somewhere and putting a sign down
// beats typing coordinates: the route is visible in-world, survives restarts,
// and you can see what it was meant to test.
//
// Scanning reads the region files rather than sweeping bots around the map:
// a 1500x1500 arena is far more than one view distance, and the files are
// exact. The server must have flushed (`/save-all flush`) for freshly placed
// signs to be there.
import { readdir, readFile } from 'node:fs/promises'
import { existsSync } from 'node:fs'
import { join } from 'node:path'
import { gunzipSync, inflateSync } from 'node:zlib'
import nbt from 'prismarine-nbt'
import { parseRegionName } from './region.js'
import type { Route, Scenario } from './routes.js'

const SECTOR = 4096
const SCENARIOS: Scenario[] = ['walk', 'parkour-simple', 'parkour-advanced', 'mixed']

export interface SignEntry {
  name: string
  role: 'start' | 'finish'
  scenario: Scenario
  /** From a `tol=N` token on line 4. */
  tolerance?: number
  pos: [number, number, number]
}

/** Every `!PF` sign in the world, in region-file order. */
export async function scanSigns (worldDir: string): Promise<SignEntry[]> {
  const regionDir = join(worldDir, 'region')
  if (!existsSync(regionDir)) throw new Error(`no region folder at ${regionDir}`)
  const found: SignEntry[] = []

  for (const file of (await readdir(regionDir)).filter(n => parseRegionName(n) !== null).sort()) {
    const raw = await readFile(join(regionDir, file))
    for (const chunk of chunkPayloads(raw)) {
      // Full NBT parsing of ~9000 chunks is minutes of work; the marker is
      // stored as literal text, so a byte scan skips all but a handful.
      if (!hasMarker(chunk)) continue
      let simplified: Record<string, unknown>
      try {
        const { parsed } = await nbt.parse(chunk)
        simplified = nbt.simplify(parsed) as Record<string, unknown>
      } catch { continue }
      for (const be of (simplified.block_entities ?? []) as Array<Record<string, unknown>>) {
        const entry = readSign(be)
        if (entry !== null) found.push(entry)
      }
    }
  }
  return found
}

/** Decompressed chunk NBT payloads inside one region file. */
function * chunkPayloads (region: Buffer): Generator<Buffer> {
  if (region.length < SECTOR * 2) return
  for (let i = 0; i < 1024; i++) {
    const offset = (region[i * 4] << 16) | (region[i * 4 + 1] << 8) | region[i * 4 + 2]
    const sectors = region[i * 4 + 3]
    if (offset === 0 || sectors === 0) continue
    const at = offset * SECTOR
    if (at + 5 > region.length) continue
    const length = region.readUInt32BE(at)
    const compression = region[at + 4]
    const body = region.subarray(at + 5, at + 4 + length)
    if (body.length === 0) continue
    try {
      if (compression === 1) yield gunzipSync(body)
      else if (compression === 2) yield inflateSync(body)
      else if (compression === 3) yield body
    } catch { /* a corrupt chunk is not worth failing the scan over */ }
  }
}

/** `!PF` in any capitalisation, as raw bytes. */
function hasMarker (buf: Buffer): boolean {
  for (const p of ['!pf', '!pF', '!Pf', '!PF']) {
    if (buf.includes(p, 0, 'latin1')) return true
  }
  return false
}

function readSign (be: Record<string, unknown>): SignEntry | null {
  if (!String(be.id ?? '').includes('sign')) return null
  for (const side of ['front_text', 'back_text']) {
    const text = be[side] as { messages?: unknown[] } | undefined
    const messages = (text?.messages ?? []).map(plainText)
    if (messages[0]?.trim().toLowerCase() !== '!pf') continue

    const name = messages[1]?.trim() ?? ''
    const role = messages[2]?.trim().toLowerCase() ?? ''
    if (name === '') continue
    if (role !== 'start' && role !== 'finish') continue

    // Line 4 is free-form options: a scenario, a `tol=N`, or both.
    const options = (messages[3] ?? '').trim().toLowerCase().split(/\s+/).filter(s => s !== '')
    const scenario = options.find(o => (SCENARIOS as string[]).includes(o)) as Scenario | undefined
    const tol = options.map(o => /^tol(?:erance)?=(\d+(?:\.\d+)?)$/.exec(o)).find(m => m !== null)
    return {
      name,
      role,
      scenario: scenario ?? 'mixed',
      tolerance: tol === null || tol === undefined ? undefined : Number(tol[1]),
      pos: [Number(be.x), Number(be.y), Number(be.z)]
    }
  }
  return null
}

/** Sign lines are text components — stored as JSON strings on some versions
 *  and as real NBT compounds on others. Both reduce to their plain text. */
function plainText (message: unknown): string {
  if (typeof message === 'string') {
    const trimmed = message.trim()
    if (!trimmed.startsWith('{') && !trimmed.startsWith('"') && !trimmed.startsWith('[')) return message
    try { return plainText(JSON.parse(trimmed)) } catch { return message }
  }
  if (Array.isArray(message)) return message.map(plainText).join('')
  if (message !== null && typeof message === 'object') {
    const o = message as { text?: unknown, extra?: unknown[] }
    return `${typeof o.text === 'string' ? o.text : ''}${(o.extra ?? []).map(plainText).join('')}`
  }
  return ''
}

export interface PairedSigns { routes: Route[], problems: string[] }

/** Match Start signs to Finish signs by name. */
export function pairSigns (entries: SignEntry[]): PairedSigns {
  const byName = new Map<string, SignEntry[]>()
  for (const e of entries) {
    const key = e.name.toLowerCase()
    byName.set(key, [...(byName.get(key) ?? []), e])
  }

  const routes: Route[] = []
  const problems: string[] = []
  for (const [key, group] of [...byName.entries()].sort()) {
    const starts = group.filter(e => e.role === 'start')
    const finishes = group.filter(e => e.role === 'finish')
    const label = group[0].name
    if (starts.length === 0) { problems.push(`"${label}": Finish sign with no Start`); continue }
    if (finishes.length === 0) { problems.push(`"${label}": Start sign with no Finish`); continue }
    if (starts.length > 1 || finishes.length > 1) {
      problems.push(`"${label}": ${starts.length} Start / ${finishes.length} Finish signs, using the first of each`)
    }
    routes.push({
      id: key.replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || key,
      name: label,
      scenario: starts[0].scenario !== 'mixed' ? starts[0].scenario : finishes[0].scenario,
      start: starts[0].pos,
      end: finishes[0].pos,
      // Either sign may carry it; the Finish sign is the natural place.
      tolerance: finishes[0].tolerance ?? starts[0].tolerance,
      notes: 'defined by in-world !PF signs'
    })
  }
  return { routes, problems }
}
