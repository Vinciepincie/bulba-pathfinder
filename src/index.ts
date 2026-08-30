// @bulba/pathfinder — drop-in replacement for mineflayer-pathfinder.
//
// ESM-first with real named exports (kills the default-import-and-
// destructure wart), while the default export keeps the upstream
// `{ pathfinder, Movements, goals }` shape so existing import styles and
// `require()` both keep working unchanged.
import { pathfinder, createPathfinder } from './plugin.js'
import { Movements } from './movements.js'
import * as goals from './goals.js'
import { autoEatIntegration } from './integrations/autoEat.js'

export { pathfinder, createPathfinder } from './plugin.js'
export type { ComputedPathResult } from './plugin.js'
export { Movements } from './movements.js'
export * as goals from './goals.js'
export { Move } from './move.js'
export type { PathfinderOptions, MovementsConfig, RawPathNode, SolveStatus, GotoOptions } from './types.js'

// Internals exported for tests / advanced embedding (reachability oracle use
// per plan §6 M1); NOT part of the stable upstream-parity API surface.
export { Solver } from './solver.js'
export type { GoalEvaluator, RawSolveResult, SolveOptions } from './solver.js'
export { MoveGen } from './moveGen.js'
export type { SnapshotView } from './moveGen.js'
export { MinHeap } from './heap.js'
export { getLut, buildLut } from './lut.js'
export type { BlockLut } from './lut.js'
export { Snapshot, buildSnapshot, computeBox, applySnapshotBlockUpdate, bakeEntityIndex } from './snapshot.js'
export { GoalAdapter } from './goalAdapter.js'
export { fastEvaluator } from './fastEvaluator.js'
export { serializeGoal, instantiateGoal, goalNeedsRaycast, descriptorTargets } from './goalSerde.js'
export { SnapshotRaycastWorld } from './raycast.js'
export { getSharedWorkerHost, SolverWorkerHost } from './worker/host.js'
export { LutFlags } from './types.js'
export * as geometry from './geometry.js'

// ── interactions and interrupts (additive; upstream has no equivalent) ────
export { InterruptController } from './interrupt.js'
export type { InterruptHandle, InterruptOptions, MotionPhase } from './interrupt.js'
export { ActionError, ActionErrors, DEFAULT_ACTION_CONFIG } from './actions/types.js'
export type {
  ActionConfig, ActionTable, ActivateOptions, BlockTarget, DigOptions,
  OpenOptions, PacingProfile, PlaceOptions, WindowLike
} from './actions/types.js'
export { createActionTable } from './actions/index.js'
export type { ActionContext, DiggingBot } from './actions/context.js'
export { Pacer } from './actions/pacing.js'
export * as reach from './actions/reach.js'
export { autoEatIntegration } from './integrations/autoEat.js'
export type { AutoEatControl, AutoEatIntegrationOptions } from './integrations/autoEat.js'

/** Upstream-compatible default export: `{ pathfinder, Movements, goals }`. */
export default { pathfinder, createPathfinder, Movements, goals, autoEatIntegration }
