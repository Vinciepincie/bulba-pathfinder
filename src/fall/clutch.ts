// When and where each breaker can be used before a predicted landing, and what the landing then costs.
// Server rules (1.21.11): a bucket's ray runs from the eye the server last saw, 4.5 long, along the
// packet's rotation; a block placement needs the clicked block within 5.5 of that eye and, for full
// blocks, no overlap with the body. Both are judged at the position of the movement packet before it.
import type { Landing, FallPredictor } from './predict.js'
import type { FallItemRef, FallItemKind } from './items.js'
import { copyBody, newBody, SIM_CUSHION } from '../playerSim.js'
import { landingPoints, healthLoss, healthAfter } from './model.js'
import type { FallBody } from './model.js'

/** Standing eye height; recovery never sneaks, so the server's pose stays standing. */
export const EYE = 1.62
/** Reach margins under the server's limits, for float error and a server-side rounding. */
export const BUCKET_REACH = 4.5 - 0.08
export const PLACE_REACH = 5.5 - 0.15

/** One way to save a landing with an item. */
export interface ClutchOption {
  kind: FallItemKind
  ref: FallItemRef
  /** Fire after the movement packet of this track tick (0: this tick's packet). */
  fireTick: number
  /** How many ticks of the track allow it: one leaves no room for error. */
  windowTicks: number
  /** The block clicked (bucket rays aim at its top face too). */
  support: { x: number, y: number, z: number }
  /** The cell the water or block goes into. */
  cell: { x: number, y: number, z: number }
  /** Rotation for the packet, mineflayer convention. */
  yaw: number
  pitch: number
  /** Health the landing costs when the clutch works. */
  loss: number
  healthAfter: number
}

export interface ClutchContext {
  predictor: FallPredictor
  body: FallBody
  whole: boolean
  /** Water evaporates here (the Nether). */
  waterEvaporates: boolean
  /** Blocks a click opens, toggles or uses instead of placing against. */
  interactable: (name: string) => boolean
  /** State id of a block by name, for imagined placements. */
  stateOf: (name: string) => number | null
  /** Food allows sprinting, as in the rollout the landing came from. */
  sprintOk: boolean
  /**
   * The body a clutch from this slot lands with: an item used from the main hand replaces a held
   * totem, so only an offhand totem still counts.
   */
  bodyFor: (slot: number) => FallBody
}

const AIRLIKE = new Set(['air', 'cave_air', 'void_air'])
/** A bucket's ray ignores fluids and passes cells with an empty outline. */
const RAY_CLEAR = new Set(['air', 'cave_air', 'void_air', 'water', 'lava', 'bubble_column'])

/** All the ways the items can save this landing, best first is the caller's job. */
export function clutchOptions (landing: Landing, refs: readonly FallItemRef[], ctx: ClutchContext): ClutchOption[] {
  const out: ClutchOption[] = []
  if (landing.medium !== 'ground' || landing.support === null || landing.tick < 1) return out
  const p = ctx.predictor
  const sup = landing.support
  const supState = p.world.stateAt(sup.x, sup.y, sup.z)
  const supInfo = p.blockInfo(supState)
  if (supInfo === null || !p.fullCube(supState)) return out
  const cell = { x: sup.x, y: sup.y + 1, z: sup.z }
  const cellInfo = p.blockInfo(p.world.stateAt(cell.x, cell.y, cell.z))
  if (cellInfo === null || !AIRLIKE.has(cellInfo.name)) return out
  const kinds = new Set(refs.map(r => r.kind))
  for (const kind of kinds) {
    if (kind === 'water' && (ctx.waterEvaporates || supInfo.waterloggable)) continue
    if (kind !== 'water' && ctx.interactable(supInfo.name)) continue
    const win = windowFor(kind, landing, sup, ctx)
    if (win === null) continue
    // The cheapest matching item of this kind, and the body it lands with (a held totem it replaces is gone).
    const ref = refs.filter(r => r.kind === kind).sort((a, b) => a.cost - b.cost)[0]
    const body = ctx.bodyFor(ref.slot)
    let loss = 0
    let after = body.health
    if (kind === 'cushion') {
      const r = cushionLanding(landing, win.first, cell, { ...ctx, body })
      if (r === null) continue
      loss = r.loss
      after = r.healthAfter
    }
    out.push({ kind, ref, fireTick: win.first, windowTicks: win.count, support: sup, cell, yaw: win.yaw, pitch: win.pitch, loss, healthAfter: after })
  }
  return out
}

/** The first track tick each kind can fire from, and how many ticks qualify. */
function windowFor (kind: FallItemKind, landing: Landing, sup: { x: number, y: number, z: number }, ctx: ClutchContext): { first: number, count: number, yaw: number, pitch: number } | null {
  let first = -1
  let count = 0
  let yaw = 0
  let pitch = 0
  // Positions sent before the landing packet: track[0 .. tick-1].
  for (let i = 0; i < landing.tick; i++) {
    const b = landing.track[i]
    if (b.onGround) continue
    const ex = b.x; const ey = b.y + EYE; const ez = b.z
    let ok = false
    let aim: { x: number, y: number, z: number } | null = null
    if (kind === 'water') {
      // Aim at the nearest point of the top face; the ray must reach it and cross nothing on the way.
      const q = { x: clamp(ex, sup.x + 0.02, sup.x + 0.98), y: sup.y + 1, z: clamp(ez, sup.z + 0.02, sup.z + 0.98) }
      const d = Math.hypot(q.x - ex, q.y - ey, q.z - ez)
      ok = d <= BUCKET_REACH && rayClear(ex, ey, ez, q, sup, ctx)
      aim = q
    } else {
      // Distance from the eye to the clicked block's box.
      const dx = Math.max(sup.x - ex, 0, ex - (sup.x + 1))
      const dy = Math.max(sup.y - ey, 0, ey - (sup.y + 1))
      const dz = Math.max(sup.z - ez, 0, ez - (sup.z + 1))
      ok = Math.hypot(dx, dy, dz) <= PLACE_REACH
      // A full block may not overlap the body the server knows.
      if (ok && kind === 'cushion') ok = b.y >= sup.y + 2 - 1e-7 || !overlapsColumn(b, sup)
      aim = { x: sup.x + 0.5, y: sup.y + 1, z: sup.z + 0.5 }
    }
    if (!ok) continue
    count++
    if (first < 0) {
      first = i
      const dx = aim.x - ex; const dy = aim.y - ey; const dz = aim.z - ez
      yaw = Math.atan2(-dx, -dz)
      pitch = Math.atan2(dy, Math.hypot(dx, dz))
    }
  }
  return first < 0 ? null : { first, count, yaw, pitch }
}

function overlapsColumn (b: { x: number, z: number }, sup: { x: number, z: number }): boolean {
  const hw = 0.3
  return b.x + hw > sup.x && b.x - hw < sup.x + 1 && b.z + hw > sup.z && b.z - hw < sup.z + 1
}

/** Every cell the segment from the eye to the aim point crosses is open to a bucket's ray (an exact voxel walk). */
function rayClear (ex: number, ey: number, ez: number, q: { x: number, y: number, z: number }, sup: { x: number, y: number, z: number }, ctx: ClutchContext): boolean {
  const p = ctx.predictor
  const dx = q.x - ex; const dy = q.y - ey; const dz = q.z - ez
  let x = Math.floor(ex); let y = Math.floor(ey); let z = Math.floor(ez)
  const sx = Math.sign(dx); const sy = Math.sign(dy); const sz = Math.sign(dz)
  const tdx = sx !== 0 ? Math.abs(1 / dx) : Infinity
  const tdy = sy !== 0 ? Math.abs(1 / dy) : Infinity
  const tdz = sz !== 0 ? Math.abs(1 / dz) : Infinity
  let tx = sx > 0 ? (x + 1 - ex) * tdx : sx < 0 ? (ex - x) * tdx : Infinity
  let ty = sy > 0 ? (y + 1 - ey) * tdy : sy < 0 ? (ey - y) * tdy : Infinity
  let tz = sz > 0 ? (z + 1 - ez) * tdz : sz < 0 ? (ez - z) * tdz : Infinity
  for (let i = 0; i < 64; i++) {
    if (x === sup.x && y === sup.y && z === sup.z) return true
    const inf = p.blockInfo(p.world.stateAt(x, y, z))
    if (inf === null || !RAY_CLEAR.has(inf.name)) return false
    if (tx > 1 && ty > 1 && tz > 1) return true
    if (tx <= ty && tx <= tz) { x += sx; tx += tdx } else if (ty <= tz) { y += sy; ty += tdy } else { z += sz; tz += tdz }
  }
  return false
}

/** Fly the rest of the fall with a hay-like block placed in the landing cell. */
function cushionLanding (landing: Landing, fire: number, cell: { x: number, y: number, z: number }, ctx: ClutchContext): { loss: number, healthAfter: number } | null {
  const p = ctx.predictor
  const state = ctx.stateOf('hay_block')
  if (state === null) return null
  p.world.setOverlay(cell.x, cell.y, cell.z, state)
  try {
    const from = copyBody(newBody(0, 0, 0), landing.track[fire])
    // The fall up to the fire tick, then the rest with the block in place.
    let fall = landing.fall
    for (let i = fire + 1; i < landing.track.length; i++) {
      const drop = landing.track[i - 1].y - landing.track[i].y
      if (drop > 0) fall -= drop
    }
    // The program goes on from the fire tick: a forward released before then stays released.
    const prog = { ...landing.program, releaseAt: Math.max(0, landing.program.releaseAt - fire) }
    const r = p.predict(from, fall, prog, ctx.body, ctx.whole, ctx.sprintOk)
    if (r.medium !== 'ground') return null
    const onCushion = (r.kind & SIM_CUSHION) !== 0
    const points = landingPoints(r.fall, r.last, r.kind, false, ctx.whole, ctx.body)
    const loss = healthLoss(points, ctx.body)
    if (!onCushion) return null
    return { loss, healthAfter: healthAfter(loss, ctx.body) }
  } finally {
    p.world.clearOverlay()
  }
}

function clamp (v: number, lo: number, hi: number): number { return v < lo ? lo : v > hi ? hi : v }
