// Offline replay of PF_DUMP_NOPATH captures: re-runs the exact solve the bot
// failed and renders the geometry the solver saw, so "No path in a room it
// should cross" is debuggable without touching prod.
//
//   npx tsx scripts/replayNoPath.ts <pf-nopath.jsonl> [entryIndex]
//
// With no entryIndex, the LAST entry replays. Renders flag/height layers
// around the start↔goal region: '#' full solid, digits 1-9 partial height in
// tenths (slab = 5, carpet = 1), '.' air, '~' liquid, 'L' climbable,
// 'x' unsafe-other, 'S' start column, 'G' goal column.
import fs from 'node:fs'
import { Snapshot } from '../src/snapshot.js'
import { Solver } from '../src/solver.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { instantiateGoal } from '../src/goalSerde.js'
import { LutFlags } from '../src/types.js'
import type { SnapshotMeta, MovementsConfig } from '../src/types.js'

const [file, idxArg] = process.argv.slice(2)
if (!file) {
  console.error('usage: npx tsx scripts/replayNoPath.ts <pf-nopath.jsonl> [entryIndex]')
  process.exit(1)
}

interface DumpLine {
  ts: number
  meta: SnapshotMeta
  flags: string
  heights: string
  start: { x: number, y: number, z: number }
  goal: never
  cfg: MovementsConfig
  visitedNodes: number
}

const rawBuf = fs.readFileSync(file)
const text = rawBuf[0] === 0xFF && rawBuf[1] === 0xFE
  ? rawBuf.toString('utf16le', 2)
  : rawBuf.toString('utf8').replace(/^﻿/, '')
const lines = text.trim().split('\n').map(l => l.trim()).filter(l => l.length > 0)
const entry = JSON.parse(lines[idxArg !== undefined ? Number(idxArg) : lines.length - 1]) as DumpLine
console.log(`entry ${idxArg ?? lines.length - 1}/${lines.length - 1} — ts ${new Date(entry.ts).toISOString()}, visited ${entry.visitedNodes}`)
console.log('start', entry.start, 'goal', JSON.stringify(entry.goal))
console.log('box', entry.meta.x0, entry.meta.y0, entry.meta.z0, `${entry.meta.w}x${entry.meta.h}x${entry.meta.l}`)

const snap = new Snapshot(entry.meta, 'replay')
snap.flags.set(Buffer.from(entry.flags, 'base64'))
snap.heights.set(Buffer.from(entry.heights, 'base64'))

// null world: fine for coordinate goals; LookAt goals need snapshot states
// (not captured) and will throw here — replay those as a GoalNear instead.
const goal = instantiateGoal(entry.goal, null)
const solver = new Solver(snap, entry.cfg, new GoalAdapter(goal), entry.start, { timeout: 10000, searchRadius: -1 })
let res = solver.compute(2000)
while (res.status === 'partial') res = solver.compute(2000)
console.log(`replay: ${res.status} visited=${res.visitedNodes} pathLen=${res.path.length}`)
if (res.path.length > 0) {
  console.log('path tail:', res.path.slice(-6).map(n => `(${n.x},${n.y},${n.z}${n.parkour ? ' P' : ''})`).join(' '))
}

// ── geometry rendering around start↔goal ────────────────────────────────────
const g = entry.goal as { gx?: number, gy?: number, gz?: number, x?: number, y?: number, z?: number }
const gx = Math.floor(g.gx ?? g.x ?? entry.start.x)
const gy = Math.floor(g.gy ?? g.y ?? entry.start.y)
const gz = Math.floor(g.gz ?? g.z ?? entry.start.z)
const x0 = Math.min(entry.start.x, gx) - 3
const x1 = Math.max(entry.start.x, gx) + 3
const z0 = Math.min(entry.start.z, gz) - 3
const z1 = Math.max(entry.start.z, gz) + 3
const y0 = Math.min(entry.start.y, gy) - 2
const y1 = Math.max(entry.start.y, gy) + 3

function cellChar (x: number, y: number, z: number): string {
  if (!snap.contains(x, y, z)) return '?'
  const i = snap.index(x, y, z)
  const f = snap.flags[i]
  const h = snap.heights[i] & 63 // bits 6–7 carry the top-catch class
  if ((f & LutFlags.LIQUID) !== 0) return '~'
  if ((f & LutFlags.CLIMBABLE) !== 0) return 'L'
  if ((f & LutFlags.PHYSICAL) !== 0) {
    if (h >= 32) return '#'
    return String(Math.max(1, Math.min(9, Math.round((h / 32) * 10))))
  }
  if ((f & LutFlags.SAFE) !== 0) return '.'
  return 'x'
}

for (let y = y1; y >= y0; y--) {
  console.log(`\n─ layer y=${y} ─ ('#'=full, 1-9=height/10, '.'=air, 'x'=unsafe, '~'=liquid)`)
  for (let z = z0; z <= z1; z++) {
    let row = ''
    for (let x = x0; x <= x1; x++) {
      if (x === entry.start.x && z === entry.start.z && y === entry.start.y) { row += 'S'; continue }
      if (x === gx && z === gz && y === gy) { row += 'G'; continue }
      row += cellChar(x, y, z)
    }
    console.log(`z=${String(z).padStart(6)} ${row}`)
  }
}
