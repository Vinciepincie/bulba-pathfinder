// Structural types keep interactions independent of mineflayer's concrete bot types.
import type { Vec3 } from 'vec3'

/** The subset of prismarine-block this layer touches. */
export interface BlockLike {
  type: number
  name: string
  stateId?: number
  position: Vec3
  boundingBox?: string
  shapes?: number[][]
  // Method syntax, so that prismarine-block's typed digTime is accepted.
  digTime? (...args: unknown[]): number
  getProperties?: () => Record<string, unknown>
}

/** A live block or block position accepted by interaction methods. */
export type BlockTarget = BlockLike | Vec3 | { x: number, y: number, z: number }

/** Window as returned by mineflayer's `openBlock` (already `extendWindow`'d). */
export interface WindowLike {
  id: number
  type: string
  title?: unknown
  slots: unknown[]
  close?: () => void
}

/** Stable error names for callers to match across interaction failures. */
export const ActionErrors = {
  /** The block is not within vanilla interaction range from where the bot is. */
  OUT_OF_REACH: 'OutOfReach',
  /** No line of sight to any face; a block occludes the target. */
  OCCLUDED: 'Occluded',
  /** The target cell holds nothing / no longer holds what the caller named. */
  NO_TARGET: 'NoTarget',
  /** Every attempt was spent without the world changing the way it should have. */
  DIG_FAILED: 'DigFailed',
  PLACE_FAILED: 'PlaceFailed',
  OPEN_FAILED: 'OpenFailed',
  ACTIVATE_FAILED: 'ActivateFailed',
  /** The bot has nothing to place / no free slot to collect into. */
  MISSING_ITEM: 'MissingItem',
  /** Walking to a spot the action could be performed from failed. */
  UNREACHABLE: 'Unreachable',
  /** An AbortSignal fired. */
  ABORTED: 'ActionAborted'
} as const

export type ActionErrorName = typeof ActionErrors[keyof typeof ActionErrors]

/** Error carrying a stable `name` plus the target it refers to. */
export class ActionError extends Error {
  readonly target?: Vec3
  /** Refused before acting; the caller may reposition and retry. */
  readonly refused: boolean

  constructor (name: ActionErrorName, message: string, target?: Vec3, refused = false) {
    super(message)
    this.name = name
    this.target = target
    this.refused = refused
  }
}

/** Options common to every action. */
export interface BaseActionOptions {
  /** Attempts before giving up (default 3). Each retry re-reads the world. */
  retries?: number
  /** Give up after this many ms across all attempts (default 30_000; 0 = no cap). */
  timeout?: number
  /** Cancel mid-flight. */
  signal?: AbortSignal
  /** Approach out-of-range targets (default true); false preserves the caller's goal and throws OutOfReach. */
  approach?: boolean
  /** Reach used for the approach goal and the range gate (default 4.5). */
  reach?: number
}

export interface DigOptions extends BaseActionOptions {
  /** Equip the fastest harvest tool first (default true). */
  equipTool?: boolean
  /** Use the raycast face (default true); false uses geometry, accepted only at very close range. */
  requireCursor?: boolean
  /** Allow geometric fallback at very close range, for escape from an enclosure (default false). */
  allowGeometric?: boolean
  /** Refuse to swing while airborne or sliding (default true). */
  requireGrounded?: boolean
  /** Pacing profile (default from the action config). */
  pacing?: PacingProfile
}

export interface PlaceOptions extends BaseActionOptions {
  /** Item to place. A name, an item id, or an inventory item object. */
  item?: string | number | { type: number, name?: string }
  /** Supporting faces, best first: below, four sides, then above; below keeps shulker boxes openable. */
  faces?: Vec3[]
  /** Sneak while clicking an interactable reference block to place against it (default true). */
  sneak?: boolean
  /** Reject supports whose opposite side is blocked, so the placed block can open there.
   * Default: true for shulker boxes, false for everything else. */
  requireOpenable?: boolean
  /** Wait for the body to stop before clicking (default true). */
  requireGrounded?: boolean
  /** Ticks to wait for the block update before judging the placement (default 10). */
  settleTicks?: number
  pacing?: PacingProfile
}

export interface OpenOptions extends BaseActionOptions {
  /** ms to wait for `windowOpen` per attempt (default 3000). */
  windowTimeout?: number
  /** Close an existing window first so it cannot cause a silent open rejection (default true). */
  closeStale?: boolean
  /** Reuse currentWindow when it belongs to this block (default true). */
  reuse?: boolean
}

export interface ActivateOptions extends BaseActionOptions {
  /** Sneak while clicking (default false). */
  sneak?: boolean
  /** Wait for rotation before clicking (default true); the path executor skips this when already facing a door. */
  settle?: boolean
}

/** How aggressively breaks and placements are spaced out. */
export type PacingProfile = 'none' | 'default' | 'drill'

/** Built-in action settings; changes via bot.pathfinder.actions.config affect the next action. */
export interface ActionConfig {
  /** Vanilla block-interaction range (1.20.5+ survival is 4.5). */
  reach: number
  /** Default retry count for every action. */
  retries: number
  /** Default overall deadline per action, ms. */
  timeout: number
  /** Default pacing profile for digs and placements. */
  pacing: PacingProfile
  /** ms to wait for `windowOpen`. */
  windowTimeout: number
  /** Refuse actions for this many ms after a server position correction rejects the interaction position. */
  forcedMoveGrace: number
}

export const DEFAULT_ACTION_CONFIG: ActionConfig = {
  reach: 4.5,
  retries: 3,
  timeout: 30_000,
  pacing: 'default',
  windowTimeout: 3000,
  forcedMoveGrace: 800
}

/** All pathfinder interactions use this table; replacing an action changes both path execution and direct calls. */
export type ActionName = 'dig' | 'place' | 'open' | 'activate' | 'equip'

export interface ActionTable {
  /** Change its fields. The actions keep this object, so replacing it would change nothing. */
  readonly config: ActionConfig
  /**
   * Layer over an action while keeping what is under it. `next` is the action as it was.
   * Returns a function that removes this layer, in any order. Later layers run first.
   */
  wrap: <K extends ActionName>(name: K, wrapper: (next: ActionTable[K]) => ActionTable[K]) => () => void
  dig: (block: BlockLike, options?: DigOptions) => Promise<void>
  place: (target: Vec3, options?: PlaceOptions) => Promise<void>
  open: (block: BlockLike, options?: OpenOptions) => Promise<WindowLike>
  activate: (block: BlockLike, options?: ActivateOptions) => Promise<void>
  equip: (item: unknown, destination?: string) => Promise<void>
}
