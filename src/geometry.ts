// Collision shapes distinguish partial blocks; epsilon keeps touching faces from overlapping.
import { Vec3 } from 'vec3'
import type { Bot } from 'mineflayer'
import { waitTicks } from './actions/reach.js'
import type { BotLike } from './actions/reach.js'

const EPS = 0.02
/** Fallback body half-width in blocks. */
const HALF = 0.3
const TALL = 1.8

/** Match the physics engine's body dimensions so boundary probes predict its collisions. */
function bodyHalf (bot: Bot): number {
  const ph = (bot as unknown as { physics?: { playerHalfWidth?: number } }).physics
  return ph?.playerHalfWidth ?? HALF
}

function bodyTall (bot: Bot): number {
  const ph = (bot as unknown as { physics?: { playerHeight?: number } }).physics
  return ph?.playerHeight ?? TALL
}

interface ShapedBlock {
  shapes?: number[][]
}

/** Cache repeated shape probes within a tick; reads outside a tick stay live. */
let shapeCache: Map<number, number[][]> | null = null
export function beginTick (): void { shapeCache = new Map() }
export function endTick (): void { shapeCache = null }
/** Coordinates wrap every 2^20 cells in X/Z and 512 in Y, beyond a tick's reach. */
function cellKey (x: number, y: number, z: number): number {
  return ((x & 0xFFFFF) * 0x100000 + (z & 0xFFFFF)) * 512 + (y & 0x1FF)
}

function shapesRaw (bot: Bot, x: number, y: number, z: number): number[][] {
  const b = bot.blockAt(new Vec3(x, y, z)) as ShapedBlock | null
  if (!b || !b.shapes || b.shapes.length === 0) return []
  return b.shapes.map((s: number[]) => [
    x + s[0], y + s[1], z + s[2],
    x + s[3], y + s[4], z + s[5]
  ])
}

/** A block's collision boxes in world coordinates, by integer cell. */
export function shapesAt (bot: Bot, x: number, y: number, z: number): number[][] {
  if (shapeCache === null || !Number.isInteger(x) || !Number.isInteger(y) || !Number.isInteger(z)) {
    return shapesRaw(bot, x, y, z)
  }
  const k = cellKey(x, y, z)
  let hit = shapeCache.get(k)
  if (hit === undefined) {
    hit = shapesRaw(bot, x, y, z)
    shapeCache.set(k, hit)
  }
  return hit
}

/** A block's collision boxes in world coordinates. */
export function blockShapes (bot: Bot, pos: Vec3): number[][] {
  return shapesAt(bot, pos.x, pos.y, pos.z)
}

/** Does a player standing at (x, feetY, z) overlap any block collision box? */
export function playerCollides (bot: Bot, x: number, feetY: number, z: number): boolean {
  const half = bodyHalf(bot)
  return boxCollides(bot,
    x - half + EPS, feetY + EPS, z - half + EPS,
    x + half - EPS, feetY + bodyTall(bot) - EPS, z + half - EPS)
}

/** Probe a thin floor column for side margins, so sideways drift still has ground to land on. */
export function floorUnder (bot: Bot, x: number, feetY: number, z: number): boolean {
  return boxCollides(bot, x - 0.05, feetY - 0.55, z - 0.05, x + 0.05, feetY - EPS, z + 0.05)
}

/** A face of a support a body can rest on, as a world-space XZ rectangle. */
export interface FaceRect { x0: number, x1: number, z0: number, z1: number, top: number }

/** Support faces in world XZ coordinates, within 0.6 blocks of the highest top.
 * Uses actual faces because narrow supports need not be centred in their cells. */
export function supportFaces (bot: Bot, x: number, y: number, z: number): FaceRect[] {
  const shapes = shapesAt(bot, x, y, z)
  if (shapes.length === 0) return []
  let top = -Infinity
  for (const s of shapes) if (s[4] > top) top = s[4]
  const out: FaceRect[] = []
  for (const s of shapes) {
    if (s[4] > top - 0.6) out.push({ x0: s[0], x1: s[3], z0: s[2], z1: s[5], top: s[4] })
  }
  return out
}

/** Is a body centred at (x, z), its hitbox shrunk by `margin` on every side, resting on one of these faces? */
export function supportedAt (bot: Bot, faces: readonly FaceRect[], x: number, z: number, margin = 0): boolean {
  const h = bodyHalf(bot) - margin
  for (const f of faces) {
    if (x + h > f.x0 && x - h < f.x1 && z + h > f.z0 && z - h < f.z1) return true
  }
  return false
}

/** Distance along unit direction (ux, uz) before leaving all currently supporting faces.
 * Returns -1 when unsupported; excludes faces reached only later. */
export function supportReach (bot: Bot, faces: readonly FaceRect[], ox: number, oz: number, ux: number, uz: number): number {
  const h = bodyHalf(bot)
  let best = -1
  for (const f of faces) {
    const ax0 = f.x0 - h
    const ax1 = f.x1 + h
    const az0 = f.z0 - h
    const az1 = f.z1 + h
    if (ox <= ax0 || ox >= ax1 || oz <= az0 || oz >= az1) continue
    let t = Infinity
    if (ux > 1e-9) t = Math.min(t, (ax1 - ox) / ux)
    else if (ux < -1e-9) t = Math.min(t, (ax0 - ox) / ux)
    if (uz > 1e-9) t = Math.min(t, (az1 - oz) / uz)
    else if (uz < -1e-9) t = Math.min(t, (az0 - oz) / uz)
    if (t > best) best = t
  }
  return best
}

/** Maximum over faces of the smaller X/Z overlap with the body at (x, z). */
export function supportDepth (bot: Bot, faces: readonly FaceRect[], x: number, z: number): number {
  const h = bodyHalf(bot)
  let best = 0
  for (const f of faces) {
    const d = Math.min(Math.min(x + h, f.x1) - Math.max(x - h, f.x0), Math.min(z + h, f.z1) - Math.max(z - h, f.z0))
    if (d > best) best = d
  }
  return best
}

/** Does this world-space box overlap any block collision box? */
export function boxCollides (
  bot: Bot,
  minX: number, minY: number, minZ: number,
  maxX: number, maxY: number, maxZ: number
): boolean {
  for (let bx = Math.floor(minX); bx <= Math.floor(maxX); bx++) {
    for (let by = Math.floor(minY); by <= Math.floor(maxY); by++) {
      for (let bz = Math.floor(minZ); bz <= Math.floor(maxZ); bz++) {
        for (const s of shapesAt(bot, bx, by, bz)) {
          if (minX < s[3] && maxX > s[0] &&
              minY < s[4] && maxY > s[1] &&
              minZ < s[5] && maxZ > s[2]) {
            return true
          }
        }
      }
    }
  }
  return false
}

/** Whether a wall is within `margin`, including exact contact left by collision resolution.
 * Some servers reject sprinting while the body touches a wall. */
export function nearWall (bot: Bot, margin = 0.03): boolean {
  const p = bot.entity.position
  const half = bodyHalf(bot)
  const tall = bodyTall(bot)
  const minX = p.x - half - margin
  const maxX = p.x + half + margin
  const minZ = p.z - half - margin
  const maxZ = p.z + half + margin
  const y0 = Math.floor(p.y + EPS)
  const y1 = Math.floor(p.y + tall - EPS)
  for (let bx = Math.floor(minX); bx <= Math.floor(maxX); bx++) {
    for (let by = y0; by <= y1; by++) {
      for (let bz = Math.floor(minZ); bz <= Math.floor(maxZ); bz++) {
        for (const sh of blockShapes(bot, new Vec3(bx, by, bz))) {
          if (minX < sh[3] && maxX > sh[0] &&
              p.y + EPS < sh[4] && p.y + tall - EPS > sh[1] &&
              minZ < sh[5] && maxZ > sh[2]) {
            return true
          }
        }
      }
    }
  }
  return false
}

/** Travel distance along (dirX, dirZ) before body collision, capped at `max`.
 * Floor beneath the feet does not count as an obstruction ahead. */
export function clearanceAhead (bot: Bot, dirX: number, dirZ: number, max = 0.3, step = 0.05): number {
  const p = bot.entity.position
  for (let d = step; d <= max + 1e-9; d += step) {
    if (playerCollides(bot, p.x + dirX * d, p.y, p.z + dirZ * d)) return d - step
  }
  return max
}

/** Is the bot's hitbox intersecting a solid block right now? */
export function isStuck (bot: Bot): boolean {
  const p = bot.entity.position
  return playerCollides(bot, p.x, p.y, p.z)
}

/** Can a player stand in this cell? Shape-aware (slab/carpet/stair = floor). */
export function isStandable (bot: Bot, pos: Vec3): boolean {
  const feetShapes = blockShapes(bot, pos)
  const feetTop = feetShapes.reduce((m, s) => Math.max(m, s[4]), pos.y)
  if (feetTop - pos.y > 0.6) return false
  if (feetShapes.length === 0 && blockShapes(bot, pos.offset(0, -1, 0)).length === 0) return false
  return !playerCollides(bot, pos.x + 0.5, feetTop, pos.z + 0.5)
}

/** Public standable check for callers choosing a destination cell. */
export function canStandAt (bot: Bot, pos: Vec3): boolean {
  return isStandable(bot, pos.floored())
}

/** Check straight-line walking clearance and continuous support at floor level `y`.
 * The lowered body probe detects floor; `avoid` and `cellCost` preserve non-geometric hazards. */
export function walkableLine (
  bot: Bot,
  x0: number, z0: number,
  x1: number, z1: number,
  y: number,
  avoid: Set<number> | null = null,
  cellCost: ((x: number, y: number, z: number) => number) | null = null,
  step = 0.25,
  sideMargin = 0,
  sideMarginLen = Infinity
): boolean {
  const dx = x1 - x0
  const dz = z1 - z0
  const len = Math.hypot(dx, dz)
  if (len < 1e-6) return true
  const n = Math.ceil(len / step)
  const cy = Math.floor(y)
  const nx = -dz / len * sideMargin
  const nz = dx / len * sideMargin
  let lastCx = NaN
  let lastCz = NaN
  for (let i = 1; i <= n; i++) {
    const t = i / n
    const x = x0 + dx * t
    const z = z0 + dz * t
    if (playerCollides(bot, x, y, z)) return false
    if (!playerCollides(bot, x, y - 0.55, z)) return false
    // Side margins leave landing room for momentum while turning onto a shortcut.
    if (sideMargin > 0 && t * len <= sideMarginLen &&
        (!floorUnder(bot, x + nx, y, z + nz) || !floorUnder(bot, x - nx, y, z - nz))) return false
    const cx = Math.floor(x)
    const cz = Math.floor(z)
    if (cx === lastCx && cz === lastCz) continue
    lastCx = cx
    lastCz = cz
    // Preserve entity detours per cell; entities elsewhere must not disable all shortcuts.
    if (cellCost !== null) {
      if (cellCost(cx, cy, cz) > 0 || cellCost(cx, cy + 1, cz) > 0) return false
    }
    if (avoid !== null && avoid.size > 0) {
      const under = bot.blockAt(new Vec3(cx, cy - 1, cz)) as { type?: number } | null
      if (under !== null && under.type !== undefined && avoid.has(under.type)) return false
      const at = bot.blockAt(new Vec3(cx, cy, cz)) as { type?: number } | null
      if (at !== null && at.type !== undefined && avoid.has(at.type)) return false
    }
  }
  return true
}

/** Nearest standable cell to the bot, nearest-first. */
export function findEscape (bot: Bot, radius = 4): Vec3 | null {
  const origin = bot.entity.position.floored()
  const candidates: Array<{ pos: Vec3, dist: number }> = []
  for (let dx = -radius; dx <= radius; dx++) {
    for (let dz = -radius; dz <= radius; dz++) {
      for (let dy = -2; dy <= 2; dy++) {
        if (dx === 0 && dz === 0 && dy === 0) continue
        const p = origin.offset(dx, dy, dz)
        if (!isStandable(bot, p)) continue
        candidates.push({ pos: p, dist: p.distanceTo(bot.entity.position) })
      }
    }
  }
  candidates.sort((a, b) => a.dist - b.dist)
  return candidates.length ? candidates[0].pos : null
}

/** Nearest cell whose feet+head are open air and which is standable. */
export function findNearestOpenStandable (bot: Bot, radius = 6): Vec3 | null {
  const origin = bot.entity.position.floored()
  const open = (p: Vec3): boolean => {
    const b = bot.blockAt(p) as { boundingBox?: string } | null
    return !b || b.boundingBox === 'empty'
  }
  const cands: Array<{ pos: Vec3, d: number }> = []
  for (let dx = -radius; dx <= radius; dx++) {
    for (let dz = -radius; dz <= radius; dz++) {
      for (let dy = -3; dy <= 2; dy++) {
        const p = origin.offset(dx, dy, dz)
        if (!open(p) || !open(p.offset(0, 1, 0)) || !isStandable(bot, p)) continue
        cands.push({ pos: p, d: p.distanceTo(bot.entity.position) })
      }
    }
  }
  cands.sort((a, b) => a.d - b.d)
  return cands.length ? cands[0].pos : null
}

function waitForTicks (bot: Bot, n: number): Promise<void> {
  const asWaiter = bot as unknown as { waitForTicks?: (n: number) => Promise<void> }
  if (typeof asWaiter.waitForTicks === 'function') return waitTicks(bot as unknown as BotLike, n)
  return new Promise((resolve) => {
    let count = 0
    const handler = (): void => {
      if (++count >= n) {
        bot.removeListener('physicsTick' as never, handler as never)
        resolve()
      }
    }
    bot.on('physicsTick' as never, handler as never)
  })
}

/** Walk and jump toward the nearest standable cell to escape a block, then release controls. */
export async function unstick (bot: Bot, attempts = 3): Promise<boolean> {
  if (!isStuck(bot)) return true

  for (let attempt = 1; attempt <= attempts; attempt++) {
    const escape = findEscape(bot)
    if (!escape) return false

    const target = escape.offset(0.5, 0, 0.5)
    try {
      try {
        await bot.lookAt(target.offset(0, 1.6, 0), true)
        bot.setControlState('forward', true)
        bot.setControlState('jump', true)
        for (let tick = 0; tick < 40; tick++) {
          await waitForTicks(bot, 1)
          if (!isStuck(bot)) break
        }
      } finally {
        bot.setControlState('forward', false)
        bot.setControlState('jump', false)
      }
      await waitForTicks(bot, 5)
    } catch {
      continue
    }

    if (!isStuck(bot)) return true
  }
  return false
}

/** Wait for position drift to end; momentum outlives control release.
 * @returns true once still, false after 40 ticks of continued drift. */
export async function settle (bot: Bot): Promise<boolean> {
  let last = bot.entity.position.clone()
  for (let attempt = 0; attempt < 20; attempt++) {
    await waitForTicks(bot, 2)
    const now = bot.entity.position
    if (now.distanceTo(last) < 0.01) return true
    last = now.clone()
  }
  return false
}

/** Centre the body in an already reached cell so its hitbox does not straddle a neighbour. */
export async function centreInCell (bot: Bot, cell: Vec3, tolerance = 0.15): Promise<boolean> {
  const centre = cell.floored().offset(0.5, 0, 0.5)
  const offset = (): number => {
    const p = bot.entity.position
    return Math.hypot(p.x - centre.x, p.z - centre.z)
  }
  if (offset() <= tolerance) return true

  try {
    await bot.lookAt(centre.offset(0, bot.entity.height ?? 1.62, 0), true)
    bot.setControlState('forward', true)
    for (let tick = 0; tick < 20; tick++) {
      await waitForTicks(bot, 1)
      if (offset() <= tolerance) break
    }
  } catch {
  } finally {
    bot.setControlState('forward', false)
  }

  await waitForTicks(bot, 2)
  if (isStuck(bot)) {
    await unstick(bot)
    return false
  }
  return offset() <= tolerance
}

/** Jump once. */
export async function jumpOnce (bot: Bot): Promise<void> {
  bot.setControlState('jump', true)
  await waitForTicks(bot, 1)
  bot.setControlState('jump', false)
}
