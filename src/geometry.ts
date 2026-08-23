// Shape-aware body geometry + walk-only recovery, absorbed from the
// bulbastore wrapper (its generic navigation half — crop/dig semantics stay
// with the app). Everything here reads block.shapes — the REAL collision
// AABBs — so carpets, slabs, stairs and chests classify correctly, and uses
// an epsilon so "touching" never counts as "overlapping" (never trust exact
// boundary equality — the 1.21 hitbox-freeze class).
import { Vec3 } from 'vec3'
import type { Bot } from 'mineflayer'

const EPS = 0.02
/** Fallback body half-width / height — see bodyHalf/bodyTall. */
const HALF = 0.3
const TALL = 1.8

/**
 * The body dimensions the PHYSICS is actually using, not the nominal ones.
 *
 * The 1.21.x hitbox-precision fix (plugin.ts hitboxPrecisionFix) nudges
 * prismarine-physics to 0.30001 / 1.80001 so collision resolutions stop
 * landing exactly on block boundaries. Every probe in this file exists to
 * predict what that engine will do, so they have to measure the same body it
 * does — otherwise the checks are 1e-5 optimistic about exactly the contacts
 * the nudge was added to disambiguate.
 */
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

/** A block's collision boxes in WORLD coordinates. */
export function blockShapes (bot: Bot, pos: Vec3): number[][] {
  const b = bot.blockAt(pos) as ShapedBlock | null
  if (!b || !b.shapes || b.shapes.length === 0) return []
  return b.shapes.map((s: number[]) => [
    pos.x + s[0], pos.y + s[1], pos.z + s[2],
    pos.x + s[3], pos.y + s[4], pos.z + s[5]
  ])
}

/** Does a player standing at (x, feetY, z) overlap any block collision box? */
export function playerCollides (bot: Bot, x: number, feetY: number, z: number): boolean {
  const half = bodyHalf(bot)
  const minX = x - half + EPS
  const maxX = x + half - EPS
  const minY = feetY + EPS
  const maxY = feetY + bodyTall(bot) - EPS
  const minZ = z - half + EPS
  const maxZ = z + half - EPS

  for (let bx = Math.floor(minX); bx <= Math.floor(maxX); bx++) {
    for (let by = Math.floor(minY); by <= Math.floor(maxY); by++) {
      for (let bz = Math.floor(minZ); bz <= Math.floor(maxZ); bz++) {
        for (const s of blockShapes(bot, new Vec3(bx, by, bz))) {
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

/**
 * Is the body within `margin` of a solid block sideways — touching a wall,
 * standing in a doorway, wedged in a corner? Unlike playerCollides this counts
 * EXACT contact, which is the state prismarine-physics leaves the bot in after
 * every horizontal collision (it clamps to the block face), and the state the
 * server refuses to accept a sprinting client in.
 */
export function nearWall (bot: Bot, margin = 0.03): boolean {
  const p = bot.entity.position
  const half = bodyHalf(bot)
  const tall = bodyTall(bot)
  const minX = p.x - half - margin
  const maxX = p.x + half + margin
  const minZ = p.z - half - margin
  const maxZ = p.z + half + margin
  // Feet and head cells only: the body is what a wall can touch.
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

/**
 * How far the body can travel along (dirX, dirZ) before it touches a block
 * face, capped at `max`.
 *
 * Only faces the body would actually hit count: playerCollides tests the
 * 1.8-tall box from the FEET up, so the step the bot is standing on top of is
 * invisible to it and the step in front of it is not. That is exactly the
 * distinction the riser standoff needs.
 */
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

/**
 * Could a walking body go from (x0,z0) to (x1,z1) in a STRAIGHT line at floor
 * level `y`, with something under its feet the whole way?
 *
 * This is the corner cut's safety check. The planner routes cell centre to
 * cell centre, so a run that drifts a few degrees off a cardinal comes back as
 * a zig-zag of alternating steps, and a follower that visits every centre
 * walks every zig. Skipping them is only sound if the line the body would
 * actually sweep is checked, which is what this does: the full body box at
 * samples along the segment, plus a support probe half a block down.
 *
 * The support probe is the same box dropped by 0.55. Anything it newly hits is
 * below the feet by construction — a head-height obstruction would already
 * have failed the first test — so "no new contact" means "no floor", which is
 * how a gap, a lava pool or an open ledge is refused: none of them have
 * anything to stand on. `avoid` catches what geometry cannot see, the blocks
 * the profile says not to walk on (magma, cactus, fire).
 */
export function walkableLine (
  bot: Bot,
  x0: number, z0: number,
  x1: number, z1: number,
  y: number,
  avoid: Set<number> | null = null,
  cellCost: ((x: number, y: number, z: number) => number) | null = null,
  step = 0.25
): boolean {
  const dx = x1 - x0
  const dz = z1 - z0
  const len = Math.hypot(dx, dz)
  if (len < 1e-6) return true
  const n = Math.ceil(len / step)
  for (let i = 1; i <= n; i++) {
    const t = i / n
    const x = x0 + dx * t
    const z = z0 + dz * t
    if (playerCollides(bot, x, y, z)) return false
    if (!playerCollides(bot, x, y - 0.55, z)) return false
    // Cost the geometry cannot see — an entity the profile is avoiding, which
    // the planner paid to detour around. Checked PER CELL: a world simply
    // containing entities is every world, so a global "are there any" test
    // switches the cut off permanently.
    if (cellCost !== null) {
      const cx = Math.floor(x)
      const cz = Math.floor(z)
      const cy = Math.floor(y)
      if (cellCost(cx, cy, cz) > 0 || cellCost(cx, cy + 1, cz) > 0) return false
    }
    if (avoid !== null && avoid.size > 0) {
      const under = bot.blockAt(new Vec3(Math.floor(x), Math.floor(y) - 1, Math.floor(z))) as { type?: number } | null
      if (under !== null && under.type !== undefined && avoid.has(under.type)) return false
      const at = bot.blockAt(new Vec3(Math.floor(x), Math.floor(y), Math.floor(z))) as { type?: number } | null
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
  if (typeof asWaiter.waitForTicks === 'function') return asWaiter.waitForTicks(n)
  // Fallback (bare/fake bots): count physicsTick events.
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

/**
 * Walk-only recovery from being embedded in a block: face the nearest
 * standable cell and walk at it holding jump. Never digs. Always releases
 * the controls it set.
 */
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

/**
 * Come to a verified standstill: stop driving, then wait until position
 * drift actually ends (momentum outlives control release).
 * @returns true once still, false if still drifting after ~40 ticks.
 */
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

/**
 * Drive the body to the CENTRE of a cell it already arrived in (a block
 * goal is satisfied by the node, not the body — a 0.6-wide hitbox on a cell
 * corner can straddle a neighbouring column and wedge).
 */
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
    // fall through to the release below
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
