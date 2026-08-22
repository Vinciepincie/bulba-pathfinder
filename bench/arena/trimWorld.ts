// Build the benchmark arena world: everything inside |x|,|z| <= radius of
// 2b2t spawn, nothing else.
//
//   npm run arena:world -- --src "<path to 2b2t_org.zip or an unpacked save>"
//
// Accepts a World Downloader zip directly (no 7 GB unpack) or a save folder,
// in either the classic (`region/`) or the modern (`dimensions/minecraft/
// overworld/region/`) layout, and always emits the classic layout the server
// expects. Chunk payloads are copied verbatim, so the output keeps whatever
// version the source was — the server upgrades it once, later.
import { mkdir, readdir, readFile, rm, writeFile } from 'node:fs/promises'
import { existsSync, statSync } from 'node:fs'
import { basename, join } from 'node:path'
import { createHash } from 'node:crypto'
import { gzipSync } from 'node:zlib'
import { parseArgs } from 'node:util'
import nbt from 'prismarine-nbt'
import { ZipReader, type ZipEntry } from './zipReader.js'
import { chunkBounds, parseRegionName, trimRegion, type TrimStats } from './region.js'
import { ARENA_RADIUS, WORLD_DIR, WORLD_NAME } from './config.js'

/** Region-holding subfolders we mirror into the output, in source order. */
const DIMENSION_FOLDERS = ['region', 'entities', 'poi'] as const

interface Source {
  describe: string
  /** Files under one of DIMENSION_FOLDERS, keyed `<folder>/<name>`. */
  list: (folder: string) => Promise<string[]>
  read: (folder: string, name: string) => Promise<Buffer>
  levelDat: () => Promise<Buffer>
  close: () => Promise<void>
}

async function openDirSource (root: string): Promise<Source> {
  // 26.x moved every dimension under `dimensions/`; older saves keep the
  // overworld at the world root.
  const modern = join(root, 'dimensions', 'minecraft', 'overworld')
  const base = existsSync(join(modern, 'region')) ? modern : root
  return {
    describe: `${root} (${base === root ? 'classic' : 'dimensions/'} layout)`,
    list: async folder => {
      const dir = join(base, folder)
      if (!existsSync(dir)) return []
      return (await readdir(dir)).filter(n => n.endsWith('.mca'))
    },
    read: async (folder, name) => await readFile(join(base, folder, name)),
    levelDat: async () => await readFile(join(root, 'level.dat')),
    close: async () => {}
  }
}

async function openZipSource (path: string): Promise<Source> {
  const zip = await ZipReader.open(path)
  const levelEntry = zip.entries
    .filter(e => e.name.endsWith('level.dat'))
    .sort((a, b) => a.name.split('/').length - b.name.split('/').length)[0]
  if (levelEntry === undefined) throw new Error(`${path}: no level.dat inside the archive`)
  const root = levelEntry.name.slice(0, -'level.dat'.length)
  const modernPrefix = `${root}dimensions/minecraft/overworld/`
  const hasModern = zip.entries.some(e => e.name.startsWith(`${modernPrefix}region/`))
  const prefix = hasModern ? modernPrefix : root

  const byPath = new Map<string, ZipEntry>()
  for (const e of zip.entries) byPath.set(e.name, e)

  return {
    describe: `${basename(path)} (${hasModern ? 'dimensions/' : 'classic'} layout, root "${root || '/'}")`,
    list: async folder => zip.entries
      .filter(e => e.name.startsWith(`${prefix}${folder}/`) && e.name.endsWith('.mca'))
      .map(e => e.name.slice(prefix.length + folder.length + 1)),
    read: async (folder, name) => {
      const entry = byPath.get(`${prefix}${folder}/${name}`)
      if (entry === undefined) throw new Error(`missing zip entry ${prefix}${folder}/${name}`)
      return await zip.read(entry)
    },
    levelDat: async () => await zip.read(levelEntry),
    close: async () => { await zip.close() }
  }
}

/** Point the world spawn at the arena — the original is far outside it, and
 *  the server would generate fresh terrain there on boot. */
async function patchLevelDat (raw: Buffer, spawn: { x: number, y: number, z: number }): Promise<Buffer> {
  const { parsed } = await nbt.parse(raw)
  const data = (parsed.value as Record<string, { value: Record<string, unknown> }>).Data
  if (data === undefined) throw new Error('level.dat has no Data compound')
  const d = data.value

  d.LevelName = { type: 'string', value: WORLD_NAME }
  d.DayTime = { type: 'long', value: [0, 6000] } // noon, so a first boot is lit
  if ('spawn' in d) {
    // 26.x form: a compound holding a pos int-array.
    const compound = d.spawn as { value: Record<string, unknown> }
    compound.value.pos = { type: 'intArray', value: [spawn.x, spawn.y, spawn.z] }
  } else {
    d.SpawnX = { type: 'int', value: spawn.x }
    d.SpawnY = { type: 'int', value: spawn.y }
    d.SpawnZ = { type: 'int', value: spawn.z }
  }
  return gzipSync(nbt.writeUncompressed(parsed, 'big'))
}

async function main (): Promise<void> {
  const { values } = parseArgs({
    options: {
      src: { type: 'string' },
      out: { type: 'string', default: WORLD_DIR },
      radius: { type: 'string', default: String(ARENA_RADIUS) },
      spawn: { type: 'string', default: '0,120,0' },
      force: { type: 'boolean', default: false }
    }
  })
  if (values.src === undefined) {
    console.error('usage: arena:world -- --src <world.zip | save folder> [--out DIR] [--radius 750]')
    process.exit(2)
  }

  const radius = Number(values.radius)
  const out = values.out as string
  const [sx, sy, sz] = (values.spawn as string).split(',').map(Number)
  const { min, max } = chunkBounds(radius)
  const keep = (cx: number, cz: number): boolean => cx >= min && cx <= max && cz >= min && cz <= max

  const src = statSync(values.src).isDirectory()
    ? await openDirSource(values.src)
    : await openZipSource(values.src)

  console.log(`source : ${src.describe}`)
  console.log(`arena  : |x|,|z| <= ${radius}  ->  chunks ${min}..${max} (${(max - min + 1) ** 2} chunks)`)
  console.log(`output : ${out}`)

  if (existsSync(out)) {
    if (!values.force) {
      console.error(`refusing to overwrite ${out} — pass --force (it will be deleted)`)
      process.exit(2)
    }
    await rm(out, { recursive: true })
  }
  await mkdir(out, { recursive: true })

  const hashes: string[] = []
  const stats: TrimStats = { kept: 0, dropped: 0 }
  let bytesOut = 0

  for (const folder of DIMENSION_FOLDERS) {
    const names = await src.list(folder)
    const wanted = names.filter(name => {
      const r = parseRegionName(name)
      // A region spans 32 chunks; keep it if its span overlaps the arena.
      return r !== null && r.x * 32 <= max && (r.x + 1) * 32 > min && r.z * 32 <= max && (r.z + 1) * 32 > min
    }).sort()
    if (names.length === 0) continue
    console.log(`\n${folder}/: ${names.length} files in source, ${wanted.length} overlap the arena`)
    if (wanted.length === 0) continue

    await mkdir(join(out, folder), { recursive: true })
    for (const name of wanted) {
      const r = parseRegionName(name)!
      const trimmed = trimRegion(await src.read(folder, name), r.x, r.z, keep, stats)
      if (trimmed === null) {
        console.log(`  ${name}: no chunks inside the arena, skipped`)
        continue
      }
      await writeFile(join(out, folder, name), trimmed)
      hashes.push(`${folder}/${name} ${createHash('sha1').update(trimmed).digest('hex')}`)
      bytesOut += trimmed.length
      console.log(`  ${name}: ${(trimmed.length / 1048576).toFixed(1)} MB`)
    }
  }

  await writeFile(join(out, 'level.dat'), await patchLevelDat(await src.levelDat(), { x: sx, y: sy, z: sz }))
  await src.close()

  const digest = createHash('sha1').update(hashes.sort().join('\n')).digest('hex')
  await writeFile(join(out, 'arena-manifest.json'), JSON.stringify({
    source: src.describe,
    radius,
    chunkRange: [min, max],
    chunksKept: stats.kept,
    chunksDropped: stats.dropped,
    bytes: bytesOut,
    // Lets two people confirm they trimmed the same world before comparing
    // numbers. Covers region bytes only — level.dat is rewritten here.
    worldDigest: digest
  }, null, 2))

  console.log(`\nkept ${stats.kept} chunks (dropped ${stats.dropped}) — ${(bytesOut / 1048576).toFixed(1)} MB`)
  console.log(`world digest: ${digest}`)
}

await main()
