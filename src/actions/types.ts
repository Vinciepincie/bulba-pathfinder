// Shared shapes for the world-interaction layer (dig / place / open /
// activate). These are deliberately structural rather than importing
// mineflayer's own types: the executor already treats `bot` structurally, the
// package only peer-depends on mineflayer, and tests drive a fake bot.
import type { Vec3 } from 'vec3'

/** The subset of prismarine-block this layer touches. */
export interface BlockLike {
  type: number
  name: string
  stateId?: number
  position: Vec3
  boundingBox?: string
  shapes?: number[][]
  digTime?: (...args: unknown[]) => number
  getProperties?: () => Record<string, unknown>
}

/** Anything a caller may hand a convenience method as "the block over there". */
export type BlockTarget = BlockLike | Vec3 | { x: number, y: number, z: number }

/** Window as returned by mineflayer's `openBlock` (already `extendWindow`'d). */
export interface WindowLike {
  id: number
  type: string
  title?: unknown
  slots: unknown[]
  close?: () => void
}

/**
 * Error names thrown by this layer. Stable strings, matched the way the
 * upstream `NoPath` / `Timeout` / `GoalChanged` / `PathStopped` names are —
 * consumers string-match, so treat these as API.
 */
export const ActionErrors = {
  /** The block is not within vanilla interaction range from where the bot is. */
  OUT_OF_REACH: 'OutOfReach',
  /** No line of sight to any face — something is between the eye and the block. */
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
  /** Set when the action stopped short rather than failing outright — the
   *  caller may reposition and try again without anything being wrong. */
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
  /**
   * Walk to a spot the action can be performed from when the target is out of
   * range (default true). With `false` the action throws `OutOfReach` instead
   * of touching the goal — the right choice inside a caller that owns the goal.
   */
  approach?: boolean
  /** Reach used for the approach goal and the range gate (default 4.5). */
  reach?: number
}

export interface DigOptions extends BaseActionOptions {
  /** Equip the fastest harvest tool first (default true). */
  equipTool?: boolean
  /**
   * Aim at the face the raycast actually reports. With `false` the face is
   * derived geometrically, which the server rejects unless the bot is nearly
   * point-blank — only useful when digging out of an enclosure (default true).
   */
  requireCursor?: boolean
  /**
   * Allow the geometric fallback above when point-blank. This is how a buried
   * bot digs itself out, and why `digOutIfBuried` works at all (default false).
   */
  allowGeometric?: boolean
  /** Refuse to swing while airborne or sliding (default true). */
  requireGrounded?: boolean
  /** Pacing profile — see `pacing.ts` (default from the action config). */
  pacing?: PacingProfile
}

export interface PlaceOptions extends BaseActionOptions {
  /** Item to place. A name, an item id, or an inventory item object. */
  item?: string | number | { type: number, name?: string }
  /**
   * Faces to try placing against, best first. Defaults to "support below,
   * then the four sides, then hung from above" — the order that keeps a
   * shulker box openable.
   */
  faces?: Vec3[]
  /**
   * Sneak while clicking so an interactable reference block (chest, hopper,
   * barrel, crafting table) is placed against rather than opened. Default
   * true, and it is what makes placing onto a container station work at all.
   */
  sneak?: boolean
  /**
   * Refuse any supporting face whose opposite side is obstructed (default
   * true). This is what keeps a placed shulker box openable: a shulker opens
   * along the face it was placed against, and one whose opening is sealed by
   * another block can never be opened again.
   */
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
  /**
   * Close an already-open window first (default true). A stale window makes
   * the server drop the next open silently.
   */
  closeStale?: boolean
  /**
   * Reuse `bot.currentWindow` when it is already the window for this block
   * (default true).
   */
  reuse?: boolean
}

export interface ActivateOptions extends BaseActionOptions {
  /** Sneak while clicking (default false — activating IS the point here). */
  sneak?: boolean
  /**
   * Aim at the block and wait for the rotation to reach the server before
   * clicking (default true). The path executor opens doors with this off:
   * it is already facing the door it is walking through, and spending ticks
   * re-aiming mid-path is how a door becomes a stall.
   */
  settle?: boolean
}

/** How aggressively breaks and placements are spaced out. */
export type PacingProfile = 'none' | 'default' | 'drill'

/**
 * Tunables for the built-in actions. Reachable and mutable at runtime via
 * `bot.pathfinder.actions.config` — changing a field affects the next action.
 */
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
  /**
   * Refuse to act for this long after a server position correction — a swing
   * sent from a position the server has already rejected is a wasted packet
   * at best and a flagged one at worst.
   */
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

/**
 * The overridable interaction table. Every world interaction the pathfinder
 * performs — from the executor's own dig branch to `bot.pathfinder.dig()` —
 * goes through exactly one of these, so replacing one replaces it everywhere.
 *
 * ```js
 * // wrap rather than replace, to keep the built-in guards
 * const inner = bot.pathfinder.actions.dig
 * bot.pathfinder.actions.dig = async (block, opts) => {
 *   console.log('digging', block.name)
 *   return await inner(block, opts)
 * }
 * ```
 */
export interface ActionTable {
  config: ActionConfig
  dig: (block: BlockLike, options?: DigOptions) => Promise<void>
  place: (target: Vec3, options?: PlaceOptions) => Promise<void>
  open: (block: BlockLike, options?: OpenOptions) => Promise<WindowLike>
  activate: (block: BlockLike, options?: ActivateOptions) => Promise<void>
  equip: (item: unknown, destination?: string) => Promise<void>
}
