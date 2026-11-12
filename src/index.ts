// The stable surface. Solver, snapshot and worker building blocks are in ./internals.
import { pathfinder, createPathfinder } from './plugin.js'
import { Movements } from './movements.js'
import * as goals from './goals.js'
import { autoEatIntegration } from './integrations/autoEat.js'

export { pathfinder, createPathfinder } from './plugin.js'
export type { ComputedPathResult } from './plugin.js'
export { Movements } from './movements.js'
export type { MovementPreset } from './movements.js'
export * as goals from './goals.js'
export { Move } from './move.js'
export type {
  ComputePathOptions, GotoOptions, MovementsConfig, PathfinderLogger, PathfinderOptions, RawPathNode, SolveStatus
} from './types.js'
export type {
  MotionView, Pathfinder, PathfinderEvents, PathResetReason, PathStoppedError, PathStopReason
} from './api.js'

export type { InterruptHandle, InterruptOptions, MotionPhase } from './interrupt.js'
export { ActionError, ActionErrors, DEFAULT_ACTION_CONFIG } from './actions/types.js'
export type {
  ActionConfig, ActionName, ActionTable, ActivateOptions, BlockTarget, DigOptions,
  OpenOptions, PacingProfile, PlaceOptions, WindowLike
} from './actions/types.js'
export { autoEatIntegration } from './integrations/autoEat.js'
export type { AutoEatControl, AutoEatIntegrationOptions } from './integrations/autoEat.js'

/** Default export includes `{ pathfinder, Movements, goals }` for compatibility. */
export default { pathfinder, createPathfinder, Movements, goals, autoEatIntegration }
