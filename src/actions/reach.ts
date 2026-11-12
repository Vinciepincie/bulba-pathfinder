// Match server reach using eye-to-cell distance; inconsistent reach checks cause silent interaction drops.
import { Vec3 } from 'vec3'
import { ActionError, ActionErrors } from './types.js'
import type { BlockLike, BlockTarget } from './types.js'

/** Eye height of a standing player. */
export const EYE_HEIGHT = 1.62

/** Distance from block centre toward a face; 7/16 stays inside common partial block models. */
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
  once?: (event: string, listener: (...args: unknown[]) => void) => unknown
  removeListener?: (event: string, listener: (...args: unknown[]) => void) => unknown
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

/** Distance from `eye` to the closest point of the block cell at `pos`, as used for server reach. */
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
  const anyTarget = target as { position?: { x: number, y: number, z: number }, x?: number, y?: number, z?: number }
  // By shape, not instanceof: a block from a second copy of vec3 is still a block.
  const p = anyTarget.position
  if (p != null && typeof p.x === 'number') return new Vec3(Math.floor(p.x), Math.floor(p.y), Math.floor(p.z))
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

/** Aim point toward a cell face, kept inside partial block models. */
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

/** Prefer the raycast face so the packet agrees with the server's rotation-based face check.
 * Geometric fallback requires explicit opt-in and nearly touching the block. */
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

  // Use geometry only without cursor data or when fallback is explicitly allowed.
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

/** Aim and wait for the cursor to match `pos` and optional `wantFace`.
 * Rotation needs time to reach the server before the click. */
export async function settleAim (
  bot: BotLike,
  pos: Vec3,
  point: Vec3,
  options: { minTicks?: number, maxTicks?: number, wantFace?: number, reach?: number } = {}
): Promise<boolean> {
  const minTicks = options.minTicks ?? 3
  const maxTicks = options.maxTicks ?? 8
  const reach = options.reach ?? 4.5

  await untilStopped(bot, () => bot.lookAt(point, true))
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

const lifecycles = new WeakMap<object, AbortSignal>()

/** Fires when the connection or the plugin is gone, and waits on this bot should give up. */
export function bindLifecycle (bot: object, signal: AbortSignal): void {
  lifecycles.set(bot, signal)
}

/**
 * Start `work` unless the bot or `signal` has stopped, and reject with ActionAborted if either stops first.
 * Mineflayer's waits on ticks and rotations need physics ticks, which end with the connection and pause
 * with physics or an unloaded chunk.
 */
export async function untilStopped<T> (bot: BotLike, work: () => PromiseLike<T> | T, signal?: AbortSignal): Promise<T> {
  const life = lifecycles.get(bot)
  const stop = life !== undefined && signal !== undefined ? AbortSignal.any([life, signal]) : life ?? signal
  if (stop === undefined) return await work()
  const stopped = (): ActionError => new ActionError(ActionErrors.ABORTED, (stop.reason as Error | undefined)?.message ?? 'The action was aborted.')
  if (stop.aborted) throw stopped()
  const running = Promise.resolve(work())
  // It may fail after this has given up on it.
  running.catch(() => {})
  let onStop = (): void => {}
  const aborted = new Promise<never>((_resolve, reject) => {
    onStop = () => reject(stopped())
    stop.addEventListener('abort', onStop, { once: true })
  })
  try {
    return await Promise.race([running, aborted])
  } finally {
    stop.removeEventListener('abort', onStop)
  }
}

/** Wait N server ticks, falling back to wall-clock when the bot has no helper. */
export async function waitTicks (bot: BotLike, ticks: number): Promise<void> {
  await untilStopped(bot, async () => {
    if (typeof bot.waitForTicks === 'function') await bot.waitForTicks(ticks)
    else await new Promise<void>(resolve => setTimeout(resolve, ticks * 50))
  })
}

/** Wait until sliding and falling stop so the server can accept the interaction position. */
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

/** Check eye-to-point visibility ending on `pos` using the world's raycast, when available. */
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
