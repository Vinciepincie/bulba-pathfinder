// Minimal random-access zip reader (ZIP64 aware, zero dependencies).
//
// Exists so the trimmer can pull 16 region files straight out of a World
// Downloader archive instead of making everyone unpack ~7 GB first. Those
// archives are routinely over 4 GB, which forces ZIP64 — hence the 64-bit
// paths below. Only STORE and DEFLATE are supported; that is everything a
// world download uses.
import { open, type FileHandle } from 'node:fs/promises'
import { inflateRaw } from 'node:zlib'
import { promisify } from 'node:util'

const inflateRawAsync = promisify(inflateRaw)

const SIG_EOCD = 0x06054b50
const SIG_EOCD64 = 0x06064b50
const SIG_EOCD64_LOCATOR = 0x07064b50
const SIG_CENTRAL = 0x02014b50
const SIG_LOCAL = 0x04034b50

export interface ZipEntry {
  name: string
  method: number
  compressedSize: number
  uncompressedSize: number
  localHeaderOffset: number
}

export class ZipReader {
  private constructor (
    private readonly fh: FileHandle,
    /** Entries by name, in central-directory order. */
    readonly entries: ZipEntry[]
  ) {}

  static async open (path: string): Promise<ZipReader> {
    const fh = await open(path, 'r')
    try {
      const { size } = await fh.stat()
      const { cdOffset, cdSize, cdCount } = await readEndRecord(fh, size)
      const cd = await readAt(fh, cdOffset, cdSize)
      return new ZipReader(fh, parseCentralDirectory(cd, cdCount))
    } catch (error) {
      await fh.close()
      throw error
    }
  }

  find (name: string): ZipEntry | undefined {
    return this.entries.find(e => e.name === name)
  }

  /** Decompressed bytes of one entry. */
  async read (entry: ZipEntry): Promise<Buffer> {
    const head = await readAt(this.fh, entry.localHeaderOffset, 30)
    if (head.readUInt32LE(0) !== SIG_LOCAL) throw new Error(`bad local header for ${entry.name}`)
    // The local header repeats name/extra with its own lengths — the central
    // directory's are not necessarily the same, so trust the local ones.
    const dataStart = entry.localHeaderOffset + 30 + head.readUInt16LE(26) + head.readUInt16LE(28)
    const raw = await readAt(this.fh, dataStart, entry.compressedSize)
    if (entry.method === 0) return raw
    if (entry.method === 8) return await inflateRawAsync(raw) as Buffer
    throw new Error(`unsupported compression method ${entry.method} for ${entry.name}`)
  }

  async close (): Promise<void> {
    await this.fh.close()
  }
}

async function readAt (fh: FileHandle, position: number, length: number): Promise<Buffer> {
  const buf = Buffer.allocUnsafe(length)
  let read = 0
  while (read < length) {
    const { bytesRead } = await fh.read(buf, read, length - read, position + read)
    if (bytesRead === 0) throw new Error(`short read at ${position + read}`)
    read += bytesRead
  }
  return buf
}

interface EndRecord { cdOffset: number, cdSize: number, cdCount: number }

async function readEndRecord (fh: FileHandle, size: number): Promise<EndRecord> {
  // The EOCD sits at the end, behind an optional comment of up to 64 KiB.
  const tailLen = Math.min(size, 0x10000 + 22)
  const tail = await readAt(fh, size - tailLen, tailLen)
  let eocd = -1
  for (let i = tail.length - 22; i >= 0; i--) {
    if (tail.readUInt32LE(i) === SIG_EOCD) { eocd = i; break }
  }
  if (eocd < 0) throw new Error('not a zip file (no end-of-central-directory record)')

  let cdCount = tail.readUInt16LE(eocd + 10)
  let cdSize = tail.readUInt32LE(eocd + 12)
  let cdOffset = tail.readUInt32LE(eocd + 16)

  // Any 0xFFFF/0xFFFFFFFF sentinel means the real value lives in the ZIP64
  // record, which the locator immediately before the EOCD points at.
  const needs64 = cdCount === 0xffff || cdSize === 0xffffffff || cdOffset === 0xffffffff
  if (needs64) {
    const loc = eocd - 20
    if (loc < 0 || tail.readUInt32LE(loc) !== SIG_EOCD64_LOCATOR) {
      throw new Error('zip needs ZIP64 but has no ZIP64 locator')
    }
    const eocd64Offset = Number(tail.readBigUInt64LE(loc + 8))
    const rec = await readAt(fh, eocd64Offset, 56)
    if (rec.readUInt32LE(0) !== SIG_EOCD64) throw new Error('bad ZIP64 end-of-central-directory record')
    cdCount = Number(rec.readBigUInt64LE(32))
    cdSize = Number(rec.readBigUInt64LE(40))
    cdOffset = Number(rec.readBigUInt64LE(48))
  }
  return { cdOffset, cdSize, cdCount }
}

function parseCentralDirectory (cd: Buffer, count: number): ZipEntry[] {
  const entries: ZipEntry[] = []
  let p = 0
  for (let i = 0; i < count && p + 46 <= cd.length; i++) {
    if (cd.readUInt32LE(p) !== SIG_CENTRAL) throw new Error(`bad central directory header at ${p}`)
    const method = cd.readUInt16LE(p + 10)
    const nameLen = cd.readUInt16LE(p + 28)
    const extraLen = cd.readUInt16LE(p + 30)
    const commentLen = cd.readUInt16LE(p + 32)
    const name = cd.toString('utf8', p + 46, p + 46 + nameLen)

    let uncompressedSize = cd.readUInt32LE(p + 24)
    let compressedSize = cd.readUInt32LE(p + 20)
    let localHeaderOffset = cd.readUInt32LE(p + 42)
    if (uncompressedSize === 0xffffffff || compressedSize === 0xffffffff || localHeaderOffset === 0xffffffff) {
      const extra = cd.subarray(p + 46 + nameLen, p + 46 + nameLen + extraLen)
      // ZIP64 extra packs only the fields that overflowed, in a fixed order.
      let q = 0
      while (q + 4 <= extra.length) {
        const id = extra.readUInt16LE(q)
        const len = extra.readUInt16LE(q + 2)
        if (id === 0x0001) {
          let r = q + 4
          if (uncompressedSize === 0xffffffff) { uncompressedSize = Number(extra.readBigUInt64LE(r)); r += 8 }
          if (compressedSize === 0xffffffff) { compressedSize = Number(extra.readBigUInt64LE(r)); r += 8 }
          if (localHeaderOffset === 0xffffffff) { localHeaderOffset = Number(extra.readBigUInt64LE(r)); r += 8 }
          break
        }
        q += 4 + len
      }
    }

    entries.push({ name, method, compressedSize, uncompressedSize, localHeaderOffset })
    p += 46 + nameLen + extraLen + commentLen
  }
  return entries
}
