// Anvil region (.mca) chunk-level trimming.
//
// Deliberately format-agnostic: chunk payloads are copied byte-for-byte, so
// this works on a 1.12 World Downloader archive and on a 1.21 save alike.
// Only the 8 KiB header is rewritten.
const SECTOR = 4096

export interface TrimStats { kept: number, dropped: number }

/**
 * Rewrite one region file keeping only the chunks `keep` accepts.
 * Returns null when nothing survives (caller should not write the file).
 */
export function trimRegion (
  src: Buffer,
  regionX: number,
  regionZ: number,
  keep: (chunkX: number, chunkZ: number) => boolean,
  stats: TrimStats
): Buffer | null {
  if (src.length < SECTOR * 2) return null

  const locations = Buffer.alloc(SECTOR)
  const timestamps = Buffer.alloc(SECTOR)
  const payloads: Buffer[] = []
  let nextSector = 2
  let kept = 0

  for (let i = 0; i < 1024; i++) {
    const offsetSectors = (src[i * 4] << 16) | (src[i * 4 + 1] << 8) | src[i * 4 + 2]
    const sectorCount = src[i * 4 + 3]
    if (offsetSectors === 0 || sectorCount === 0) continue

    const chunkX = regionX * 32 + (i % 32)
    const chunkZ = regionZ * 32 + Math.floor(i / 32)
    if (!keep(chunkX, chunkZ)) { stats.dropped++; continue }

    const start = offsetSectors * SECTOR
    const end = start + sectorCount * SECTOR
    if (end > src.length) { stats.dropped++; continue } // truncated source chunk

    payloads.push(src.subarray(start, end))
    locations[i * 4] = (nextSector >> 16) & 0xff
    locations[i * 4 + 1] = (nextSector >> 8) & 0xff
    locations[i * 4 + 2] = nextSector & 0xff
    locations[i * 4 + 3] = sectorCount
    timestamps.set(src.subarray(SECTOR + i * 4, SECTOR + i * 4 + 4), i * 4)
    nextSector += sectorCount
    kept++
    stats.kept++
  }

  if (kept === 0) return null
  return Buffer.concat([locations, timestamps, ...payloads])
}

/** `r.<x>.<z>.mca` → region coords, or null for anything else. */
export function parseRegionName (name: string): { x: number, z: number } | null {
  const m = /^r\.(-?\d+)\.(-?\d+)\.mca$/.exec(name)
  return m ? { x: Number(m[1]), z: Number(m[2]) } : null
}

/** Inclusive chunk bounds covering |x| <= radius and |z| <= radius. */
export function chunkBounds (radius: number): { min: number, max: number } {
  return { min: Math.floor(-radius / 16), max: Math.floor(radius / 16) }
}
