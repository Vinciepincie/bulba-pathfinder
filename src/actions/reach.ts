// Reach, aim and face selection — the geometry every world interaction needs
// before it is allowed to send a packet.
//
// The rule this file enforces is "vanilla range": the server measures block
// interaction from the EYE to the closest point of the target block's cell,
// not centre-to-centre. A bot that measures centre-to-centre thinks it can
// reach a block ~0.87 further away than the server does on a diagonal, sends
// the click, and gets silence — no window, no break, no error, just a dropped
// packet. Measuring the same way the server does turns that whole failure
// class into an explicit `OutOfReach` the caller can act on.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { BlockLike, BlockTarget } from './types.js'

/** Eye height of a standing player. */
export const EYE_HEIGHT = 1.62

/**
 * How far the cursor may be from the block centre on the face plane. Vanilla
 * block models rarely fill the cell — a chest lid stops at 0.9375 — so aiming
 * at the exact geometric face centre can raycast past a slim block entirely.
 * 7/16 keeps the aim point inside every common model.
 */
export const FACE_INSET = 7 / 16

/** Beyond this the geometric face fallback is not credible; see `pickFace`. */
export const POINT_BLANK = 2.2

export interface BotLike {
  entity: { position: Vec3, velocity?: Vec3, onGround?: boolean, height?: number }
  blockAt: (pos: Vec3, extraInfos?: boolean) => BlockLike | null
  blockAtCursor?: (maxDistance?: number) => (BlockLike & { face?: number, intersect?: Vec3 }) | null
  world?: { raycast?: (from: Vec3, dir: Vec3, range: number) => { position: Vec3, face: number } | null }
  look: (yaw: number, pitch: number, force?: boolean) => Promise<void>
  lookAt: (pos: Vec3, force?: boolean) => Promise<void>
  waitForTicks?: (ticks: number) => Promise<void>
  setControlState: (name: string, value: boolean) => void
  emit: (event: string, ...args: unknown[]) => boolean
}

/** The six block faces, in protocol order (`block_place` direction ids). */
export const FACE_VECTORS: Vec3[] = [
  new Vec3(0, -1, 0), // 0 down
  new Vec3(0, 1, 0), // 1 up
  new Vec3(0, 0, -1), // 2 north
  new Vec3(0, 0, 1), // 3 south
  new Vec3(-1, 0, 0), // 4 west
  new Vec3(1, 0, 0) // 5 east
]

/** Protocol direction id for a unit face vector, or -1. */
export function faceId (v: Vec3): number {
  for (let i = 0; i < FACE_VECTORS.length; i++) {
    const f = FACE_VECTORS[i]
    if (f.x === v.x && f.y === v.y && f.z === v.z) return i
  }
  return -1
}

/** Where the server thinks the bot is looking from. */
export function eyePos (bot: BotLike): Vec3 {
  const h = bot.entity.height
  return bot.entity.position.offset(0, typeof h === 'number' && h > 0 ? h * 0.9 : EYE_HEIGHT, 0)
}

/**
 * Distance from `eye` to the CLOSEST point of the block cell at `pos` — the
 * same measurement the server makes. Not centre-to-centre.
 */
export function reachToCell (eye: Vec3, pos: Vec3): number {
  const cx = Math.min(Math.max(eye.x, pos.x), pos.x + 1)
  const cy = Math.min(Math.max(eye.y, pos.y), pos.y + 1)
  const cz = Math.min(Math.max(eye.z, pos.z), pos.z + 1)
  const dx = eye.x - cx
  const dy = eye.y - cy
  const dz = eye.z - cz
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

/** Is the block at `pos` inside vanilla interaction range right now? */
export function inReach (bot: BotLike, pos: Vec3, reach: number): boolean {
  return reachToCell(eyePos(bot), pos) <= reach
}

/** Normalise whatever the caller passed into a live block, or throw NoTarget. */
export function resolveBlock (bot: BotLike, target: BlockTarget): BlockLike {
  const pos = toVec3(target)
  const block = bot.blockAt(pos, false)
  if (!block) {
    throw new ActionError(ActionErrors.NO_TARGET, `No block loaded at ${fmt(pos)}.`, pos)
  }
  return block
}

/** Normalise whatever the caller passed into a floored block position. */
export function toVec3 (target: BlockTarget): Vec3 {
  const anyTarget = target as { position?: Vec3, x?: number, y?: number, z?: number }
  if (anyTarget.position instanceof Vec3) return anyTarget.position.floored()
  return new Vec3(
    Math.floor(anyTarget.x as number),
    Math.floor(anyTarget.y as number),
    Math.floor(anyTarget.z as number)
  )
}

export function fmt (v: Vec3): string {
  return `${v.x},${v.y},${v.z}`
}

/** Centre of the block cell. */
export function cellCentre (pos: Vec3): Vec3 {
  return new Vec3(pos.x + 0.5, pos.y + 0.5, pos.z + 0.5)
}

/**
 * A point on `face` of the cell at `pos`, inset from the edges so the ray
 * lands on the block model rather than sliding past a slim one.
 */
export function facePoint (pos: Vec3, face: Vec3): Vec3 {
  return new Vec3(
    pos.x + 0.5 + face.x * FACE_INSET,
    pos.y + 0.5 + face.y * FACE_INSET,
    pos.z + 0.5 + face.z * FACE_INSET
  )
}

/** Cursor position within the block, as `block_place` wants it (0..1 per axis). */
export function cursorFor (face: Vec3): Vec3 {
  return new Vec3(0.5 + face.x * FACE_INSET, 0.5 + face.y * FACE_INSET, 0.5 + face.z * FACE_INSET)
}

export interface Aim {
  /** Outward face normal of the target block. */
  face: Vec3
  /** World-space point to look at. */
  point: Vec3
  /** Cursor offset inside the block for the protocol packet. */
  cursor: Vec3
  /** Protocol direction id. */
  id: number
}

/**
 * Choose the face to click. Prefers whatever the bot's own raycast reports —
 * that is the face the SERVER will compute too, and a mismatch between the
 * face in the packet and the face the server derives from the rotation is one
 * of the things anticheats look at. Falls back to the nearest geometric face
 * only when explicitly allowed and the bot is nearly touching the block, where
 * every face is plausibly clickable.
 */
export function pickFace (
  bot: BotLike,
  pos: Vec3,
  options: { requireCursor?: boolean, allowGeometric?: boolean, reach?: number } = {}
): Aim | null {
  const requireCursor = options.requireCursor !== false
  const reach = options.reach ?? 4.5
  const eye = eyePos(bot)

  if (typeof bot.blockAtCursor === 'function') {
    const hit = bot.blockAtCursor(reach)
    if (hit?.position && hit.position.equals(pos) && typeof hit.face === 'number' && hit.face >= 0) {
      const face = FACE_VECTORS[hit.face]
      if (face) {
        return { face, point: facePoint(pos, face), cursor: cursorFor(face), id: hit.face }
      }
    }
    if (requireCursor && !options.allowGeometric) return null
  }

  // No cursor information (a fake bot in tests, or a version without the
  // helper), or the caller opted into the escape hatch.
  const centre = cellCentre(pos)
  const pointBlank = eye.distanceTo(centre) <= POINT_BLANK
  if (typeof bot.blockAtCursor === 'function' && !pointBlank) return null

  let best: Aim | null = null
  let bestDist = Number.POSITIVE_INFINITY
  for (let i = 0; i < FACE_VECTORS.length; i++) {
    const face = FACE_VECTORS[i]
    const neighbour = pos.plus(face)
    const outside = bot.blockAt(neighbour, false)
    // A face buried in another solid block cannot be clicked.
    if (outside && outside.boundingBox === 'block') continue
    const point = facePoint(pos, face)
    const d = eye.distanceTo(point)
    if (d < bestDist) {
      bestDist = d
      best = { face, point, cursor: cursorFor(face), id: i }
    }
  }
  return best
}

/**
 * Look at `point` and wait until the bot's own raycast agrees it is aimed at
 * `pos` (and, when given, at `wantFace`). Rotation reaches the server as a
 * position packet, so clicking on the same tick as the look is how a click
 * arrives with the previous rotation attached.
 */
export async function settleAim (
  bot: BotLike,
  pos: Vec3,
  point: Vec3,
  options: { minTicks?: number, maxTicks?: number, wantFace?: number, reach?: number } = {}
): Promise<boolean> {
  const minTicks = options.minTicks ?? 3
  const maxTicks = options.maxTicks ?? 8
  const reach = options.reach ?? 4.5

  await bot.lookAt(point, true)
  for (let tick = 0; tick < maxTicks; tick++) {
    await waitTicks(bot, 1)
    if (tick + 1 < minTicks) continue
    if (typeof bot.blockAtCursor !== 'function') return true // nothing to verify against
    const hit = bot.blockAtCursor(reach)
    if (!hit?.position || !hit.position.equals(pos)) continue
    if (options.wantFace !== undefined && hit.face !== options.wantFace) continue
    return true
  }
  return typeof bot.blockAtCursor !== 'function'
}

/** Wait N server ticks, falling back to wall-clock when the bot has no helper. */
export async function waitTicks (bot: BotLike, ticks: number): Promise<void> {
  if (typeof bot.waitForTicks === 'function') {
    await bot.waitForTicks(ticks)
    return
  }
  await new Promise<void>(resolve => setTimeout(resolve, ticks * 50))
}

/**
 * Wait until the body has actually stopped. A swing sent while the bot is
 * still sliding or falling is sent from a position the server has not agreed
 * to yet.
 */
export async function settleGrounded (bot: BotLike, maxTicks = 20): Promise<boolean> {
  for (let i = 0; i < maxTicks; i++) {
    const e = bot.entity
    const v = e.velocity
    const still = v
      ? Math.abs(v.y) < 0.08 && Math.sqrt(v.x * v.x + v.z * v.z) < 0.03
      : true
    if (e.onGround === true && still) return true
    await waitTicks(bot, 1)
  }
  return bot.entity.onGround === true
}

/** Does the bot's own hitbox occupy the cell at `pos`? */
export function bodyOverlaps (bot: BotLike, pos: Vec3): boolean {
  const p = bot.entity.position
  const half = 0.3
  const tall = 1.8
  return (
    p.x + half > pos.x && p.x - half < pos.x + 1 &&
    p.z + half > pos.z && p.z - half < pos.z + 1 &&
    p.y + tall > pos.y && p.y < pos.y + 1
  )
}

/**
 * Is there a clear line from the eye to `point`, ending on `pos`? Uses the
 * bot's own world raycast when available — the same one mineflayer's
 * `blockAtCursor` uses — so the answer matches what the server will compute.
 */
export function hasLineOfSight (bot: BotLike, pos: Vec3, point: Vec3, reach: number): boolean {
  const raycast = bot.world?.raycast
  if (typeof raycast !== 'function') return true // cannot tell; let the server judge
  const eye = eyePos(bot)
  const dir = point.minus(eye)
  const dist = dir.norm()
  if (dist === 0) return true
  const hit = raycast.call(bot.world, eye, dir.scaled(1 / dist), Math.min(dist + 0.5, reach + 1))
  return hit === null || hit.position.equals(pos)
}
