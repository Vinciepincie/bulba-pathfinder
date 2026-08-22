// One-time arena setup: fetch Paper, write the server config, optionally add
// ViaVersion (so a newer Minecraft client can spectate), optionally convert
// every chunk to the server version up front.
//
//   npm run arena:server -- --upgrade --via
import { createWriteStream } from 'node:fs'
import { access, mkdir } from 'node:fs/promises'
import { Readable } from 'node:stream'
import { pipeline } from 'node:stream/promises'
import { join } from 'node:path'
import { parseArgs } from 'node:util'
import { downloadPaper, upgradeWorld, writeServerConfig } from './server.js'
import { HOST, MC_VERSION, PORT, RUN_DIR, WORLD_DIR } from './config.js'

const UA = 'bulba-pathfinder-arena/1.0 (+https://github.com/bulbastore)'

/** Optional: lets a spectator on any newer client join the 1.21.11 server. */
async function downloadViaVersion (): Promise<void> {
  const dir = join(RUN_DIR, 'plugins')
  await mkdir(dir, { recursive: true })
  const res = await fetch('https://api.modrinth.com/v2/project/viaversion/version', { headers: { 'User-Agent': UA } })
  if (!res.ok) throw new Error(`modrinth: ${res.status} ${res.statusText}`)
  const versions = await res.json() as Array<{
    version_number: string
    loaders: string[]
    game_versions: string[]
    files: Array<{ url: string, filename: string, primary: boolean }>
  }>
  const pick = versions.find(v =>
    v.loaders.some(l => ['paper', 'spigot', 'bukkit', 'purpur', 'folia'].includes(l)) &&
    v.game_versions.includes(MC_VERSION)
  ) ?? versions.find(v => v.loaders.some(l => ['paper', 'spigot', 'bukkit'].includes(l)))
  const file = pick?.files.find(f => f.primary) ?? pick?.files[0]
  if (file === undefined) throw new Error('no suitable ViaVersion build found')

  const out = join(dir, file.filename)
  try { await access(out); console.log(`viaversion: ${file.filename} (cached)`); return } catch { /* download */ }
  const jar = await fetch(file.url, { headers: { 'User-Agent': UA } })
  if (!jar.ok || jar.body === null) throw new Error(`viaversion download failed: ${jar.status}`)
  await pipeline(Readable.fromWeb(jar.body as never), createWriteStream(out))
  console.log(`viaversion: ${file.filename} (${pick?.version_number})`)
}

const { values } = parseArgs({
  options: {
    upgrade: { type: 'boolean', default: false },
    via: { type: 'boolean', default: false }
  }
})

try { await access(WORLD_DIR) } catch {
  console.error(`no arena world at ${WORLD_DIR} — run "npm run arena:world -- --src <world.zip>" first`)
  process.exit(2)
}

console.log('Writing eula.txt=true — this accepts the Minecraft EULA for the local benchmark server.')
await downloadPaper()
await writeServerConfig()
if (values.via) {
  try { await downloadViaVersion() } catch (error) {
    console.warn(`viaversion skipped: ${(error as Error).message}`)
    console.warn(`spectate with a ${MC_VERSION} client instead, or drop a ViaVersion jar in ${join(RUN_DIR, 'plugins')}`)
  }
}
if (values.upgrade) await upgradeWorld()

console.log(`\narena ready at ${HOST}:${PORT} (Minecraft ${MC_VERSION})`)
console.log('next: npm run arena:race')
