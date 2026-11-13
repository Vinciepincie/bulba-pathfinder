// The public contract of `bot.pathfinder`: what other packages may build on.
import type { Vec3 } from 'vec3'
import type { Goal } from './goals.js'
import type { Movements } from './movements.js'
import type { Move } from './move.js'
import type { ComputedPathResult } from './plugin.js'
import type { GotoOptions, ComputePathOptions } from './types.js'
import type { InterruptHandle, InterruptOptions, MotionPhase } from './interrupt.js'
import type { AutoEatControl } from './integrations/autoEat.js'
import type { FallInfo, FallOutcome, FallRecoveryOptions } from './fall/guard.js'
import type {
  ActionTable, ActivateOptions, BlockTarget, DigOptions, OpenOptions, PlaceOptions, WindowLike
} from './actions/types.js'

/**
 * Why a goal ended without arriving. Carried by `path_stop` and by the
 * `PathStopped` error's `reason`.
 */
export type PathStopReason =
  /** `pathfinder.stop()`. */
  | 'stop'
  /** Health fell below `pathfinder.minHealth`. */
  | 'low_health'
  /** The bot died, with `pathfinder.stopOnDeath` on. */
  | 'death'
  /** The connection ended. */
  | 'disconnected'
  /** `pathfinder.dispose()`. */
  | 'disposed'
  /** A `goto` signal aborted. */
  | 'aborted'
  /** The movement profile cannot be solved. */
  | 'unsupported'
  /** A `goto` found no path. A plain `setGoal` keeps its goal instead. */
  | 'no_path'
  /** A `goto`'s search ran out of `thinkTimeout`. */
  | 'timeout'

/** Why the current path was thrown away. The goal stays, and a new path follows. */
export type PathResetReason =
  | 'goal_updated' | 'movements_updated' | 'block_updated' | 'chunk_loaded' | 'goal_moved'
  | 'dig_error' | 'place_error' | 'stuck' | 'forced_move' | 'respawn' | 'basin'
  /** A fall recovery moved the body off the path. */
  | 'fall'
  /** The water bucket a planned drop needs is gone. */
  | 'clutch_missing'
  /** The landing of a planned clutch drop no longer takes water (changed, or it would soak in). */
  | 'clutch_unfit'
  | (string & {})

/** The error `goto` rejects with when a goal is stopped. */
export interface PathStoppedError extends Error {
  name: 'PathStopped'
  reason: PathStopReason
}

/** Everything the pathfinder emits on the bot. */
export interface PathfinderEvents {
  goal_reached: (goal: Goal) => void
  path_update: (results: ComputedPathResult) => void
  goal_updated: (goal: Goal | null, dynamic: boolean) => void
  path_reset: (reason: PathResetReason) => void
  path_stop: (reason: PathStopReason) => void
  /** Health fell below `minHealth` and the goal is being stopped. */
  'pathfinder:low_health': (health: number, minHealth: number) => void
  'pathfinder:interrupt_requested': (reason: string) => void
  'pathfinder:paused': (reasons: string[]) => void
  'pathfinder:resumed': (reason: string, pausedMs: number) => void
  'pathfinder:dig_start': (block: unknown) => void
  'pathfinder:dig_finish': (position: Vec3) => void
  'pathfinder:dig_aborted': (position: Vec3, why: string) => void
  /** A dig the executor started for a path failed. */
  'pathfinder:dig_error': (error: unknown) => void
  'pathfinder:place_start': (position: Vec3, item: unknown) => void
  'pathfinder:place_finish': (position: Vec3, placed: unknown) => void
  'pathfinder:open_start': (position: Vec3) => void
  'pathfinder:open_finish': (position: Vec3, window: unknown) => void
  'pathfinder:open_retry': (position: Vec3, rung: string, message: string) => void
  'pathfinder:activate': (position: Vec3, block: unknown) => void
  /** A door the executor opened for a path failed to respond. */
  'pathfinder:activate_error': (error: unknown) => void
  /**
   * The bot is falling in a way the executor did not plan (knocked, its floor removed, or not driving)
   * and the landing will hurt. Once per fall, with the predicted landing and the plan.
   */
  'pathfinder:fall_start': (info: FallInfo) => void
  /** No known plan survives this fall. Usually emitted well before the landing, so your own logic can act. */
  'pathfinder:fall_lethal': (info: FallInfo) => void
  /** The fall is over: what it cost and what recovery did. */
  'pathfinder:fall_finish': (outcome: FallOutcome) => void
  /** Only with the `autoEatIntegration` plugin loaded. */
  'pathfinder:autoeat_start': () => void
  'pathfinder:autoeat_finish': () => void
  'pathfinder:autoeat_failed': (error: unknown) => void
}

/** Read-only view of what the executor is doing, for anything that times around it. */
export interface MotionView {
  readonly phase: MotionPhase
  /** Taking the controls now would break the run: in flight, or half way through a dig or an interaction. */
  readonly critical: boolean
  /** The next node is a jump not yet committed to. */
  readonly jumpPending: boolean
  readonly paused: boolean
  readonly holders: string[]
  /** The next node of the path, live: read it, do not change it. */
  readonly node: Move | null
}

/** `bot.pathfinder`. */
export interface Pathfinder {
  readonly goal: Goal | null
  readonly movements: Movements
  readonly actions: ActionTable
  readonly motion: MotionView

  /** Search budget per solve, ms (default 5000). */
  thinkTimeout: number
  /** Search per tick on the main thread and the partial-result cadence on the worker, ms (default 40). */
  tickTimeout: number
  /** Extra search cost allowed past the heuristic (default -1, unlimited). */
  searchRadius: number
  enablePathShortcut: boolean
  LOSWhenPlacingBlocks: boolean
  /** Ms without reaching the next node before the goal is stopped (default -1, off). */
  stuckTimeout: number
  /** Ms a goal may run before it is stopped (default -1, off). */
  executionTimeout: number
  /** Keep walking the still-valid path while a replan runs (default true). */
  keepPathDuringRecompute: boolean
  /**
   * Health the bot must keep (default 0, off). Below it the goal stops once the body is on
   * something. Each planned fall is also limited to what health can pay when it is planned.
   */
  minHealth: number
  /** End the goal when the bot dies, instead of walking back after the respawn (default true). */
  stopOnDeath: boolean
  /** Fall recovery: report unplanned falls, or also steer and use a breaker. Change its fields. */
  readonly fallRecovery: FallRecoveryOptions

  /** Walk to a goal. Rejects with `NoPath`, `Timeout`, `GoalChanged` or `PathStopped`, or the signal's reason. */
  goto: (goal: Goal, options?: GotoOptions) => Promise<void>
  /** Set a goal without waiting for it. A dynamic goal is followed as it moves. */
  setGoal: (goal: Goal | null, dynamic?: boolean) => void
  setMovements: (movements: Movements) => void
  /** Stop within a tick. A goal in flight lands first. */
  stop: () => void
  isMoving: () => boolean
  isMining: () => boolean
  isBuilding: () => boolean
  bestHarvestTool: (block: unknown) => unknown

  /** Plan on the worker without moving the bot or touching the current goal. */
  computePath: (goal: Goal, options?: ComputePathOptions) => Promise<ComputedPathResult>
  /** Plan synchronously on this thread. Prefer `computePath`, which searches on the worker when it can. */
  getPathTo: (movements: Movements, goal: Goal, timeout?: number) => ComputedPathResult
  getPathFromTo: (movements: Movements, startPos: Vec3, goal: Goal, options?: {
    optimizePath?: boolean
    resetEntityIntersects?: boolean
    timeout?: number
    tickTimeout?: number
    searchRadius?: number
    startMove?: { x: number, y: number, z: number }
  }) => Generator<{ result: ComputedPathResult, astarContext: unknown }>

  /** Walk into range if needed, then break the block. */
  dig: (target: BlockTarget, options?: DigOptions) => Promise<void>
  /** Walk into range if needed, then place a block at the position. */
  place: (target: BlockTarget, options?: PlaceOptions) => Promise<void>
  /** Walk into range if needed, then open whatever window the block has. */
  open: (target: BlockTarget, options?: OpenOptions) => Promise<WindowLike>
  /** Walk into range if needed, then right-click the block. */
  activate: (target: BlockTarget, options?: ActivateOptions) => Promise<void>

  /** Borrow the bot at a safe moment. The path resumes from the same node on release. */
  interrupt: (reason: string, options?: InterruptOptions) => Promise<InterruptHandle>
  withInterrupt: <T>(reason: string, fn: () => Promise<T>, options?: InterruptOptions) => Promise<T>

  isStuck: () => boolean
  canStandAt: (pos: Vec3) => boolean
  findEscape: (radius?: number) => Vec3 | null
  findNearestOpenStandable: (radius?: number) => Vec3 | null
  unstick: () => Promise<boolean>
  centreInCell: (cell: Vec3, tolerance?: number) => Promise<boolean>
  jumpOnce: () => Promise<void>
  settle: () => Promise<boolean>

  /** Stop any goal, remove every listener and take the plugin off the bot. Never touches the shared worker. */
  dispose: () => void
}

declare module 'mineflayer' {
  interface Bot {
    pathfinder: Pathfinder
    /** Present once `autoEatIntegration()` is loaded. */
    pathfinderAutoEat?: AutoEatControl
  }
  interface BotEvents extends PathfinderEvents {}
}
