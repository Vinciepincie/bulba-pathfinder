// Snapshots let worker searches run without reading the live world.
// Partial paths let the bot move while a solve continues.
import { performance } from 'node:perf_hooks'
import { AsyncLocalStorage } from 'node:async_hooks'
import fs from 'node:fs'
import { Vec3 } from 'vec3'
import nbt from 'prismarine-nbt'
import type { Bot } from 'mineflayer'
import { Movements, VINE_CLIMBABLES, physicsClimbsVines } from './movements.js'
import { Move } from './move.js'
import { Goal } from './goals.js'
import { PhysicsSim, SEED_FAMILY, seedFamilies } from './physics.js'
import { Solver } from './solver.js'
import type { RawSolveResult } from './solver.js'
import { GoalAdapter } from './goalAdapter.js'
import { getLut } from './lut.js'
import type { BlockLut } from './lut.js'
import { computeDigData, digFingerprint } from './digData.js'
import { fastEvaluator } from './fastEvaluator.js'
import type { DigContext } from './moveGen.js'
import type { DigData, MovementsConfig, SolveStatus } from './types.js'
import {
  Snapshot, buildSnapshot, computeBox, slimeUnderBox, applySnapshotBlockUpdate, bakeEntityIndex, nextSnapshotGeneration, BAN_WEIGHT
} from './snapshot.js'
import type { Box } from './snapshot.js'
import { serializeGoal, goalNeedsRaycast, descriptorTargets } from './goalSerde.js'
import { TAKEOFF_STAND, TAKEOFF_NARROW_MARGIN, CHAIN_TAKEOFF_FRACTION } from './parkourEnvelope.js'
import { CATCH_HALF, topCatchClass } from './shapes.js'
import { headingDir, programHeading, sneakLanding, F_FWD, F_SPRINT, F_JUMP, LAND_OVERLAP, CLIMB_ON_SLACK, BEAM_DEPTH, ONE_FLIGHT_REACH, LIVE_PREFIX, jumpRearmed, STAY_TICKS, REGION_SETTLE, FLIGHT_CAP, FLIGHT_TICKS_MAX } from './hopOracle.js'
import { HOP_TURN_COS, PARKOUR_LAND_DY, CLIMB_HOLD_ENABLED } from './gaitSearch.js'
import { landingHurts, fallDamagePoints, dropForHealth, fallVetoForHealth, FALL_VETO, SAFE_FALL } from './fallDamage.js'
import { simKindOf } from './playerSim.js'
import { sideStandoff, wallSlide, againstWall as wallAhead, SPRINT_WALL_MARGIN, STEP_UP_MIN, WALK_STEP_REACH } from './wallSteer.js'
import type { Heading as SteerHeading } from './wallSteer.js'
import type { HopProgram } from './hopOracle.js'
import { getSharedWorkerHost } from './worker/host.js'
import * as geometry from './geometry.js'
import { InterruptController } from './interrupt.js'
import type { InterruptHandle, InterruptOptions, MotionPhase } from './interrupt.js'
import { createActionTable } from './actions/index.js'
import { Pacer } from './actions/pacing.js'
import { DEFAULT_ACTION_CONFIG, ActionError, ActionErrors } from './actions/types.js'
import type {
  ActionTable, ActivateOptions, BlockTarget, DigOptions, OpenOptions, PlaceOptions, WindowLike
} from './actions/types.js'
import type { ActionContext, DiggingBot } from './actions/context.js'
import { bindLifecycle, fmt, resolveBlock, toVec3, inReach } from './actions/reach.js'
import { throwIfAborted } from './actions/dig.js'
import { GoalLookAtBlock, GoalNear } from './goals.js'
import type { ComputePathOptions, PathfinderOptions, GoalDescriptor, PhysicsLike, GotoOptions, GaitDrive } from './types.js'
import type { PathStopReason, PathStoppedError } from './api.js'
import { onLegPast, fallAim } from './gaitSearch.js'

class Lock {
  private locked = false

  tryAcquire (): boolean {
    if (this.locked) return false
    this.locked = true
    return true
  }

  release (): void {
    this.locked = false
  }
}

export interface ComputedPathResult {
  status: SolveStatus
  cost: number
  time: number
  visitedNodes: number
  generatedNodes: number
  path: Move[]
  context: { visitedChunks: Set<string>, compute?: () => ComputedPathResult } | null
}

interface BlockLike {
  type: number
  stateId?: number
  position: Vec3
  shapes: number[][]
  boundingBox?: string
}

/** Bounds how long server corrections can delay stuck detection. */
const FORCED_MOVE_GRACE_MS = 3000

/** Mineflayer position packets and post edges can cause a one-tick onGround flicker.
 * A real jump or fall also changes velocity or height. */
const AIRBORNE_TICKS = 2

/** Run-up attempts back off, then sprint along the planned take-off line. */
const RUN_UP_ATTEMPTS = 2
const RUN_UP_MAX_TICKS = 12
const RUN_UP_REAR = 1.5
/** Steer ahead on the take-off centre line so the run-up converges onto it.
 * Mirror the aim when backing because back moves away from the look point. */
const LINE_UP_AHEAD = 1.0
/** Limit waiting after line-up attempts fail; ban the landing to avoid repeating the same jump. */
const LIP_WAIT_TICKS = 30
const BAN_MS = 120_000

/** Sneak only near the lip; walking from this distance cannot leave the support
 * before the edge guard engages on the next tick. */
const CREEP_SNEAK_ZONE = 0.35

/** Margin inside a narrow support's overhang limit (half-width + 0.30).
 * It covers the final sneaking press, but holdSupport must also guard the coast. */
const NARROW_LIP_SAFETY = 0.08

/** Try releasing keys before braking; a landing may need walking force to cancel its speed. */
const HOLD_KEYS: ReadonlyArray<{ forward?: boolean, back?: boolean, sneak?: boolean }> = [
  {}, { back: true, sneak: true }, { forward: true, sneak: true }, { back: true, sneak: false }, { forward: true, sneak: false }
]
/** Horizontal speed below which unkeyed drift is negligible for support holding. */
const HOLD_STILL = 1e-3

/** Keep the resting start inside the position tolerance every accepted hop survives. */
const PROG_LINEUP_TOL = 0.02
/** Enables braking when line-up controls or coasting would leave the support. */
const PROG_LINEUP_BRAKE = process.env.PF_PROG_LINEUP_BRAKE !== '0'
const PROG_BRAKE_TICKS = 3
const PROG_SNEAK_ZONE = 0.6
const PROG_COAST = 1 / (1 - 0.6 * 0.91)
/** Held-jump ticks allowed without take-off before abandoning a replay. */
const PROG_JUMP_GRACE = 6
/** Limits live line-up searches per node near the planned start. */
const PROG_LIVE_TRIES = 3
const PROG_LIVE_NEAR = 0.15
/** Enables comparing live flights with the planned flight's remaining line-up cost. */
const PROG_LIVE_PLAN = process.env.PF_PROG_LIVE_PLAN !== '0'
/** Enables re-anchoring planned flights at the moving body to avoid unnecessary stops. */
const PROG_MOVE_ANCHOR = process.env.PF_PROG_MOVE_ANCHOR !== '0'
const PROG_LINEUP_SPEED = 0.1
const PROG_LINEUP_SETTLE = 4
const PROG_LIVE_SLACK = 2
/** Enables live flights that skip intermediate nodes. */
const SKIP_AHEAD = process.env.PF_SKIP_AHEAD !== '0'
const SKIP_TRIES = 3
/** Enables live flights past a neo's landing to the following node. */
const SKIP_NEO = process.env.PF_SKIP_NEO !== '0'
/** Enables direct flights past walk nodes when they beat the planned approach. */
const SKIP_WALK = process.env.PF_SKIP_WALK !== '0'
/** Enables checking upcoming neos early so an impossible take-off can be replanned while walking. */
const NEO_AHEAD = process.env.PF_NEO_AHEAD !== '0'
const NEO_AHEAD_NODES = 3
/** Enables physics-kernel flights for momentum chains. */
const CHAIN_KERNEL = process.env.PF_CHAIN_KERNEL !== '0'
const CHAIN_PICK_BUDGET = 250_000
/** Enables replanning when a chain loses the momentum its next jump requires. */
const CHAIN_LOST = process.env.PF_CHAIN_LOST !== '0'
/** Enables abandoning a take-off when the lined-up search proves no flight lands. */
const LINED_GIVEUP = process.env.PF_LINED_GIVEUP !== '0'
/** Enables installing stitched hop legs as the search finds them. */
const STREAM_STITCH = process.env.PF_STREAM_STITCH !== '0'
/** Enables taking a new mid-flight path up from the committed landing. */
const FLIGHT_TRIM = process.env.PF_FLIGHT_TRIM !== '0'
/** Enables coasting at a partial path's end to preserve position and momentum for the next update. */
const PARTIAL_END_COAST = process.env.PF_PARTIAL_END_COAST !== '0'
/** Maximum height loss while coasting at a partial path's end. */
const PARTIAL_COAST_DROP = 0.01
/** Enables replanning an initial solve after a teleport moves the body away from its start. */
const FORCED_IDLE_REPLAN = process.env.PF_FORCED_IDLE_REPLAN !== '0'
const FORCED_IDLE_REACH = 2
/** Enables resetting lost-take-off counts for each new fixed goal. */
const LOST_PER_GOAL = process.env.PF_LOST_PER_GOAL !== '0'
// Limits correction retries so repeated lagbacks cannot trigger a search each tick.
const PREFIX_RETRY_CAP = process.env.PF_PREFIX_RETRY_CAP !== '0'
// Bound prefix searches per goal because off-path corrections reset per-solve marks.
const PREFIX_GOAL_MAX = 8
/** Enables releasing stale movement controls after an off-path correction. */
const FORCED_RELEASE = process.env.PF_FORCED_RELEASE !== '0'
/** Try kick seeds when the live neo search families find no flight. */
const NEO_SEED: ReadonlyArray<'kick' | 'pivot'> = ['kick']
const LINED_NEAR = 1.5
/** Bound replans from one rest because complete paths reset the futility timer. */
const PROG_LOST_TRIES = 2
/** Require consecutive still ticks before ending a deep-bounce abort;
 * residual bounces can persist, so BASIN_MAX_MS also bounds the wait. */
const BASIN_REST_TICKS = 5
/** Maximum depth below the snapshot to search for a reachable slime pool. */
const DEEP_DROP = 72
const BASIN_MAX_MS = 20_000
/** Fall damage above which a slime landing must bounce instead of sneaking. */
const SLIME_SNEAK_MAX_DAMAGE = 8
/** Precision programs need a tightly aligned start. Re-anchor at rest or use
 * bounded one-tick sneaking presses to reach it. */
const PROG_EXACT_TOL = 0.05
const PROG_EXACT_REACH = 0.11
const PROG_PRESS_TRIES = 2
const LINEUP_TICKS = 60_000
/** Enables lining up precision programs before the first rest. */
const MOVING_LINEUP = process.env.PF_MOVING_LINEUP !== '0'
/** Replan if a distant program start is not walkable; lining up across a gap is unsafe. */
const PROG_FAR_START = 2
/** Feet-height tolerance when climbing to a program's take-off. */
const PROG_CLIMB_TOL = 0.05
const PROG_PRESS_SHIFT = 0.1 * 0.98 * 0.3 * PROG_COAST
/** Retire a passed landing only nearby, or farther away on a platform the body can walk back across. */
const LAND_PAST = 1
const LAND_PAST_WALKED = 2.5
const TAKEOFF_KERNEL = process.env.PF_TAKEOFF_KERNEL !== '0'
const TAKEOFF_SEARCH_TICKS = 100_000
/** Kernel-tick budget for the stand-point fallback at a blocked take-off. */
const TAKEOFF_LINED_TICKS = 200_000
/** Kernel-tick budget for the neo search from rest, twice the moving search's budget. */
const TAKEOFF_REST_TICKS = 200_000
/** Kernel-tick budget for deriving a planned beam flight from the actual lined-up rest. */
const TAKEOFF_REST_BEAM_TICKS = 300_000
/** Chunk-arrival grace before final noPath, disabled because present but empty columns evade detection. */
const CHUNK_WAIT_MS = 0
/** Limit partial-path drops to the jump's rounded-up rise so the body can return. */
const PARTIAL_ONE_WAY_DROP = 1.25
/** Slippery blocks that cannot safely hold a line-up at rest. */
const ICE_NAMES = new Set(['ice', 'packed_ice', 'blue_ice', 'frosted_ice'])
/** Distance within which a node removed by a streamed update counts as reached. */
const PARTIAL_PROGRESS_REACH = 1.5
/** Keep short partial paths grounded because the search can reverse direction;
 * an airborne reversal commits the body to the wrong way for a whole flight. */
const PARTIAL_HOP_MIN = 3.5

/** Enables scripted jump searches. */
const JUMP_SOLVER = process.env.PF_JUMP_SOLVER !== '0'
/** Resting ticks before searching for a parkour flight no gate authorises. */
const SCRIPT_STILL_TICKS = 6
/** Candidate scripts flown per executor tick while searching. */
const SCRIPT_BUDGET = 400
/** How far the body may be from a script's predicted track before the script is dropped. */
const SCRIPT_TRACK_TOL = 0.06
/** Enables in-flight landing control on narrow supports. */
const AIR_CONTROL = process.env.PF_AIR_CONTROL !== '0'
const CLIMB_PRESS = 0.6
/** Along-wall coasting distance per unit speed under 0.91 air drag. */
const CLIMB_DRIFT = 0.91 / 0.09
/** Use landing aim only for deep falls; parkour descents need to clear a block face. */
const DROP_AIM_ENTER = 8
const CLIMB_HOLD = 0.15
/** A final node on a climbable is held until the body is within this of its height. */
const CLIMB_FINISH_DY = 0.2
/** Lateral error allowed before creeping onto a shifted flight line.
 * The planner sweeps with a hitbox reduced by 5 cm; vanilla slides past small corner nicks. */
const SHIFT_LINE_TOL = 0.1

/** Skip narrow-support creeps whose gain is mostly lost to the coast after sneaking. */
const THIN_CREEP_MIN = 0.2

/** The 1.25-block hop apex needs a taller arrival box so flown-over nodes
 * retire instead of turning the bot back. */
const HOP_ARRIVE_DY = 1.45


/** Disable corner cuts near jumps so the body lines up on the take-off rather than the chord. */
const CUT_JUMP_GUARD = Number(process.env.PF_CUT_JUMP_GUARD ?? 5) || 5

/** Spread visibility scans across ticks to bound per-tick work.
 * Extensions stay farther along the same path to avoid heading flicker. */
const CUT_SCAN_INIT = 8
const CUT_EXTEND = 4

/** Allow consecutive visibility misses: a corner can hide a nearer node but expose a farther one. */
const CUT_MISSES = 2

/** Cap chord length because longer lines add sweep work with little heading improvement. */
const CUT_MAX_CHORD = 32

/** Keep synthetic chord nodes at the one-block spacing the rollouts and hop score expect. */
const CUT_GATE_STEP = 1.0

/** Aim a short distance ahead on the line so lateral drift produces a strong correction.
 * A distant target barely turns the body back before it can drift off a ledge. */
const CUT_CARROT = Number(process.env.PF_CUT_CARROT ?? 1.5) || 1.5

/** Lateral error from the line past which the cut is dropped and re-picked from where the body is. */
const CUT_MAX_OFF = 0.6

/** Extra ground clearance beside a new cut while the body turns onto it. */
const CUT_SIDE_MARGIN = Number(process.env.PF_CUT_SIDE_MARGIN ?? 0)
const CUT_MARGIN_LEN = Number(process.env.PF_CUT_MARGIN_LEN ?? 5) || 5

/** Keep a cut through small corrections: its chord can legitimately lie far from every path node. */
const CUT_LAG_NEAR = 2

/** Fall damage triggers a velocity packet that zeros horizontal motion.
 * Wait for it before choosing a take-off; this timeout plus ping bounds the wait. */
const HURT_WAIT_TICKS = 8
/** The most a body falls in a tick (terminal velocity, 3.92). */
const MAX_TICK_DROP = 4

/** Surface below this air level (mineflayer's 0..20 scale).
 * Vanilla's 300-tick air supply leaves time to swim out of planned crossings. */
const AIR_RESERVE = 7

/** Movement below this, over a tick, is jitter rather than progress. */
const WEDGE_MOVE = 0.1

/** Allow turns and landings to settle before treating jitter-only movement as a wedge. */
const WEDGE_TICKS = 12

/** Consecutive ticks allowed per escape before trying another direction. */
const ESCAPE_PATIENCE = 6

/** Bound angled take-offs to roughly a jump's duration before trying other escapes. */
const ANGLE_PATIENCE = 14

/** Cycle escapes per node to avoid repeating one that fails; simulate each to avoid unsafe falls. */
const ESCAPES: Array<{ back?: boolean, left?: boolean, right?: boolean, forward?: boolean, jump?: boolean }> = [
  { back: true },
  { left: true },
  { right: true }
]

const DEFAULT_OPTIONS: Required<Omit<PathfinderOptions, 'physicsFactory' | 'onNoPath'>> = {
  useWorkerThreads: true,
  core: 'wasm',
  workerEntryPath: '',
  maxSnapshotCells: 8_000_000,
  hitboxPrecisionFix: true,
  logger: console
}

const AIR_NAMES: ReadonlySet<string> = new Set(['air', 'cave_air', 'void_air'])

/** The error a stopped goal's `goto` rejects with. */
function pathStoppedError (reason: PathStopReason): PathStoppedError {
  const message = 'Path was stopped before it could be completed! Thus, the desired goal was not reached.'
  const err = new Error(reason === 'stop' ? message : `${message} (${reason})`) as PathStoppedError
  err.name = 'PathStopped'
  err.reason = reason
  return err
}

/** What a cancelled operation rejects with: the signal's own reason, or an AbortError. */
function abortReason (signal: AbortSignal): Error {
  const reason = (signal as { reason?: unknown }).reason
  if (reason instanceof Error) return reason
  const error = new Error('The operation was aborted.')
  error.name = 'AbortError'
  return error
}

/** Reject when the signal fires, instead of waiting for the promise. */
async function abortable<T> (promise: Promise<T>, signal: AbortSignal | undefined, onAbort: () => void): Promise<T> {
  if (signal === undefined) return await promise
  if (signal.aborted) {
    onAbort()
    throw abortReason(signal)
  }
  let listener = (): void => {}
  const aborted = new Promise<never>((_resolve, reject) => {
    listener = () => {
      onAbort()
      reject(abortReason(signal))
    }
    signal.addEventListener('abort', listener, { once: true })
  })
  try {
    return await Promise.race([promise, aborted])
  } finally {
    signal.removeEventListener('abort', listener)
  }
}

/** Preserve a streamed line-up only for an identical program;
 * flights into the same cell can land on opposite sides of a pane. */
function sameProgram (a: HopProgram, b: HopProgram): boolean {
  const h = (u: { fixed: boolean, x: number, z: number }, v: { fixed: boolean, x: number, z: number }): boolean => u.fixed === v.fixed && u.x === v.x && u.z === v.z
  const land = a.land === undefined ? b.land === undefined : b.land !== undefined && a.land.x === b.land.x && a.land.y === b.land.y && a.land.z === b.land.z
  const seq = a.seq === undefined ? b.seq === undefined : b.seq !== undefined && a.seq.length === b.seq.length && a.seq.every((v, i) => v === b.seq![i] || (Number.isNaN(v) && Number.isNaN(b.seq![i])))
  return a.sx === b.sx && a.sy === b.sy && a.sz === b.sz && a.tx === b.tx && a.ty === b.ty && a.tz === b.tz &&
    a.ticks === b.ticks && a.family === b.family && a.jumpAt === b.jumpAt && a.turnAir === b.turnAir &&
    a.sprint === b.sprint && a.kick === b.kick && a.precise === b.precise && a.climb === b.climb && a.deep === b.deep &&
    h(a.run, b.run) && h(a.air1, b.air1) && h(a.air2, b.air2) && land && seq
}

/** Create a configured plugin for `bot.loadPlugin`; `pathfinder` is the default instance. */
export function createPathfinder (options: PathfinderOptions = {}) {
  // An option passed as undefined keeps its default.
  const given = Object.fromEntries(Object.entries(options).filter(([, v]) => v !== undefined)) as PathfinderOptions
  const opts = { ...DEFAULT_OPTIONS, ...given }

  return function inject (bot: Bot): void {
    const registry = bot.registry
    const warn = (message: string): void => { opts.logger.warn(message) }
    /** What dispose() undoes: every listener the plugin added. */
    const disposers: Array<() => void> = []
    function listen (event: string, listener: (...args: never[]) => void): void {
      bot.on(event as never, listener as never)
      disposers.push(() => { bot.removeListener(event as never, listener as never) })
    }
    const waterType: number = registry.blocksByName.water.id
    /** Climbable block IDs matching the physics ladder test, including climbable vines. */
    const climbableIds = new Set<number>()
    for (const name of ['ladder', 'vine', ...(physicsClimbsVines(bot) ? VINE_CLIMBABLES : [])]) {
      const b = registry.blocksByName[name]
      if (b !== undefined) climbableIds.add(b.id)
    }
    const bubbleColumnId: number = registry.blocksByName.bubble_column ? registry.blocksByName.bubble_column.id : -1

    let stateMovements = new Movements(bot)
    let stateGoal: Goal | null = null
    let dynamicGoal = false
    let path: Move[] = []
    let pathUpdated = false
    // Last dispatched solve's inputs, for the onNoPath capture hook.
    let lastSolveStart: { x: number, y: number, z: number } = { x: 0, y: 0, z: 0 }
    /** Last solve's start cell, used to detect progress after its path runs out. */
    let lastSolveFrom: { x: number, y: number, z: number } | null = null
    let lastSolveGoal: unknown = null
    let lastSolveCfg: MovementsConfig | null = null
    let placing = false
    let placingBlock: { x: number, y: number, z: number, useOne?: boolean } | null = null
    let digging = false
    /** Sprint-hop latched at take-off, re-pressed on each landing (allowSprintHop). */
    let hopHold = false
    /** A live skip retains removed nodes and the planned program so a failed take-off can restore them. */
    let skip: { target: Move, removed: Move[], planned: HopProgram | null | undefined } | null = null
    /** Cache upcoming neo verdicts by edge cells because replans create new node objects. */
    const neoChecked = new Map<string, boolean>()
    /** Node whose lined-up search has no verdict and still needs the fallback search. */
    let linedOpen: object | null = null
    /** Take-offs lost without a ban, by edge cells, across replans (lostReplan). */
    const lostEdges = new Map<string, number>()
    /** Skip-ahead searches flown per target node, and the targets a skip failed for. */
    const skipTries = new WeakMap<Move, number>()
    const skipFailed = new WeakSet<Move>()
    /** Last rest positions searched, to avoid repeating skip-ahead work at the same spot. */
    let skipRestX = NaN
    let skipRestZ = NaN
    let walkRestX = NaN
    let walkRestZ = NaN
    /** A kernel-selected hop keeps the controls its flight was verified with. */
    let kernelHop = false
    /** The kernel owns this tick's controls when its last grounded search finds a schedule. */
    let kernelLed = false
    /** Collision probe used by wall steering. */
    const boxProbe = (x: number, y: number, z: number): boolean => geometry.playerCollides(bot, x, y, z)
    const steer: SteerHeading = { dx: 0, dz: 0 }
    let khopFlight: number[] | null = null
    let khopTick = 0
    /** Strafe held after the take-off tick by the current kernel flight. */
    let khopStrafe = false
    let khopEase = 99
    /** The gait pressed the strafe key this tick: released at the top of the next. */
    let gaitStrafed = false
    /** A support hold pressed forward/back this tick (holdSupport): released at the next tick's start. */
    let holdKeyed = false
    /** The node being walked round a corner over nothing (the hug-corner rule), until it is retired. */
    let hugNode: unknown = null
    /** Program phase (0 line-up, 1 replay) and counters mirror the oracle
     * so execution uses the verified aim schedule. */
    let progNode: Move | null = null
    let progPhase = 0
    let progTick = 0
    let progFired = false
    let progDrivenAt = -1
    let progJumpTicks: number | undefined
    let progAir = 0
    let progRebounds = 0
    /** An aborted deep bounce stays steered inside its pool until still, then replans. */
    let basin: { tops: Array<[number, number, number]>, still: number, since: number } | null = null
    /** Live searches the line-up has run for the program node, and where the last at-rest one ran. */
    let progLiveTries = 0
    let progRestX = Infinity
    let progRestZ = Infinity
    /** Queued exact line-up yaws, next press index, and per-node attempt count. */
    let progPress: { yaws: number[], i: number, delay: number } | null = null
    let progPresses = 0
    let progMovingTries = 0
    /** Settling ticks after a region program lands off its target level. */
    let progSettle = 0
    /** Tracks ownership of climb controls so only the hold's keys are released. */
    let climbKeysHeld = false
    /** Climb-control ownership at tick start, before any branch writes keys. */
    let climbHeldPrev = false
    /** Prefix searches of this goal (PREFIX_GOAL_MAX), and the plan a found one replaced. */
    let prefixGoalCount = 0
    const prefixPlan = new WeakMap<Move, HopProgram>()
    /** Region programs lost in a row from about one rest, and that rest (PROG_LOST_TRIES). */
    let progLostCount = 0
    let progLostAt: { x: number, y: number, z: number } | null = null
    /** The node the kernel take-off search last ran for, and whether that search was from rest. */
    let takeoffSearched: unknown = null
    let takeoffSearchedAtRest = false
    /** The node the lined-up search (hopLinedUp) last ran for: once per node. */
    let linedSearched: unknown = null
    /** A program installed by the lined-up search skips a duplicate search from the same body state. */
    let linedInstalled: unknown = null
    /** What a take-off search found this tick (PhysicsSim.hopNote), for the exec trace. */
    let takeoffNote = ''
    /** Sneak latched by fullStop to hold the body on a climbable (released when a path is installed). */
    let climbHold = false
    /** Keep forward and sprint until touchdown after a path reset or stop;
     * clearing them in flight can shorten the jump and miss its landing. */
    let coastHold = false

    /** Sneak's cell-based edge guard can miss a narrow panel inside the cell.
     * Simulate both controls and coast, then use the first keys that retain support. */
    function holdSupport (): boolean {
      const e = bot.entity as { onGround?: boolean, isInWater?: boolean, isInLava?: boolean }
      if (e.onGround !== true || e.isInWater === true || e.isInLava === true) return false
      const cs = bot.controlState as unknown as Record<string, boolean>
      if (physics.holdsOn === undefined || cs.jump === true) return false
      const v = bot.entity.velocity
      const keyed = cs.forward === true || cs.back === true || cs.left === true || cs.right === true
      if (!keyed && Math.hypot(v.x, v.z) < HOLD_STILL) return false
      if (physics.holdsOn(null)) return false
      for (const k of HOLD_KEYS) {
        if (!physics.holdsOn(k)) continue
        bot.setControlState('forward', k.forward === true)
        bot.setControlState('back', k.back === true)
        bot.setControlState('left', false)
        bot.setControlState('right', false)
        bot.setControlState('sprint', false)
        if (k.sneak !== undefined) bot.setControlState('sneak', k.sneak)
        holdKeyed = k.forward === true || k.back === true
        return true
      }
      return false
    }

    /** In the air, with nothing but momentum carrying the body. */
    function flyingFree (): boolean {
      const e = bot.entity as { onGround?: boolean, isInWater?: boolean, isInLava?: boolean }
      if (e.onGround === true || e.isInWater === true || e.isInLava === true) return false
      const feet = bot.blockAt(bot.entity.position) as { type: number } | null
      return feet === null || !climbableIds.has(feet.type)
    }

    function coastIfFlying (): boolean {
      if (!flyingFree()) return false
      if (typeof bot.getControlState !== 'function') return false
      const forward = bot.getControlState('forward')
      const sprint = bot.getControlState('sprint')
      bot.clearControlStates()
      bot.setControlState('forward', forward)
      bot.setControlState('sprint', sprint)
      coastHold = true
      return true
    }
    /** Final-path edge cells; partial paths may safely retain drops already planned on these edges. */
    let finalEdges: Set<string> | null = null
    /** Where the neo branch last searched a take-off from rest: the program branch does not repeat it from the same spot. */
    let restSearchedAt: { node: unknown, x: number, z: number } | null = null
    /** The neo and program branches share one attempt to make room for take-off. */
    const prefixSearched = new WeakSet<Move>()
    const prefixRetried = new WeakSet<Move>()
    /** The node the corner cut is steering at, held until reached or invalid. */
    let cutTarget: Move | null = null
    /** Sliding anchor of the corner-cut line to cutTarget. */
    let cutFrom: Vec3 | null = null
    /** Body position at the last tick, for the lagback handler's cut-aware near test (CUT_LAG_NEAR). */
    let prevTickPos: Vec3 | null = null
    /** Physics ticks seen, and the one prevTickPos was taken at. */
    let physTicks = 0
    let prevTickAt = 0
    /** physTicks at the last server position (forcedMove): a teleport across a stale gap. */
    let forcedAtTick = -1
    let wedgeAnchor: Vec3 | null = null
    let wedgeTicks = 0
    let recoverTicks = 0
    let recoverKind = 0
    let nextEscape = 0
    let recoverNode = ''
    /** Angle-solver state: the offset that lands the current node, if any. */
    let headingNode = ''
    let headingBias: number | null = null
    let headingSprint = false
    let biasFlight = false
    let biasTicks = 0
    /** Support cell anchored at the start of a parkour creep. */
    let creepCell: { x: number, z: number } | null = null
    /** Slime-bounce phase, latched per node: has the rebound started? */
    let bounceNode: Move | null = null
    /** The node a long drop is being flown to by landing aim (the drop-aim branch), latched to its landing. */
    let dropAimNode: unknown = null
    let bounceRisen = false
    /** Momentum chain, latched per node: has the re-jump left the stone? */
    let chainNode: Move | null = null
    let chainFired = false
    let chainDrivenAt = -1
    let chainJumpTicks: number | undefined
    /** Run-up state: node, attempts, phase (0 idle, 1 backing, 2 sprinting), and ticks in phase. */
    let runUpNode: Move | null = null
    let runUpAttempts = 0
    let runUpPhase = 0
    let runUpTicks = 0
    let runUpLastPos: Vec3 | null = null
    /** Consecutive grounded ticks spent waiting at a lip with nothing left to try (LIP_WAIT_TICKS). */
    let lipWaitTicks = 0
    /** Failed landing cells keyed by coordinates, with expiry in performance.now milliseconds. */
    const bannedCells = new Map<string, number>()
    /** The parkour node `creepCell` was latched for. */
    let creepNode: Move | null = null
    /** The fall so far, as the server counts it, for the landings that hurt (HURT_WAIT_TICKS). */
    let fallen = 0
    /** Ticks left to wait for the velocity packet of a landing that hurt, and whether it came. */
    let landSettle = 0
    /** Timestamp of the landing that owes a velocity-packet wait, so stale waits expire. */
    let landSettleAt = 0
    let hurtSeen = false
    /** Fall-distance rule for this version, until a velocity packet indicates otherwise. */
    let wholeFall = (bot.registry as unknown as { isNewerOrEqualTo?: (v: string) => boolean }).isNewerOrEqualTo?.('1.21.5') === true
    /** Ticks after a landing that hurts only under the older rule, awaiting a confirming velocity packet. */
    let spared = 0
    /** Consecutive ticks with onGround=false (see AIRBORNE_TICKS). */
    let airTicks = 0
    let prevTickY = 0
    /** Grounded ticks since the last touchdown (large while airborne); read by the arrival loop. */
    let landedTicksAgo = 99
    let lastTickGrounded = false
    /** Most recently retired node, used as the next parkour take-off. */
    let prevRetired: { x: number, y: number, z: number, cell?: readonly [number, number, number] } | null = null
    /** Record decisions in a second physicsTick listener so early returns are covered. */
    let execBranch = 'idle'
    /** In-flight landing control: the node it was last sized up for, and whether its support is narrow. */
    let airNode: unknown = null
    let airNarrow = false
    /** Script replay state and resting ticks before searching for an unauthorised parkour flight. */
    let jumpScript: { yawA: number, yawB: number, turnTick: number, sprint: boolean, jumpAfter: number, ticks: number, track: Array<{ x: number, y: number, z: number }>, miss: number } | null = null
    let jumpScriptNode: unknown = null
    let jumpScriptTick = 0
    let jumpScriptAir = 0
    let jumpStillTicks = 0
    let jumpSolveFailedNode: unknown = null
    let jumpSolveStartNode: unknown = null
    /** Latched for the flight once the default was predicted to miss its node. */
    let airEngaged = false
    let execTick = 0
    const execTraceFile = process.env.PF_EXEC_TRACE ? `${process.env.PF_EXEC_TRACE}-${bot.username}.jsonl` : null
    const execTraceBuf: string[] = []
    let lastNodeTime = performance.now()
    /** The path installed is a streamed partial (handleDriveResult: not final). */
    let pathPartial = false
    /** The landing of the last leg flown off a streamed partial, until a whole path has taken it up (STREAM_STITCH). */
    let streamedLanding: readonly [number, number, number] | null = null
    /** The solve the installed path came from (handleDriveResult's generation). */
    let pathGen = -1
    /** Last actual node arrival, never refreshed by server corrections;
     * bounds how long corrections may excuse a lack of progress. */
    let lastNodeArrival = performance.now()
    let goalSetTime = performance.now()
    let stopPathing = false
    /** Why the next internalStop stops: carried by path_stop and PathStopped. */
    let stopReason: PathStopReason = 'stop'
    /** The plugin's life is over: no tick will ever serve another goal. Set before anything is told. */
    let ended: PathStopReason | null = null
    /** A stop the goal still owes once the body is supported, so that it never lands in mid-flight. */
    let pendingStop: PathStopReason | null = null
    /** Last server correction; clicks from a rejected position must wait for the correction window. */
    let lastForcedMoveAt = 0

    const physics = options.physicsFactory ? options.physicsFactory(bot) : new PhysicsSim(bot)
    const lockUseBlock = new Lock()
    // Exact block-boundary alignment can trigger repeated server corrections on 1.21.x.
    // A 1e-5 size increase breaks that alignment; only stock dimensions are changed
    // so an application's existing adjustment is preserved.
    let restoreHitbox = (): void => {}
    if (opts.hitboxPrecisionFix) {
      const ph = bot.physics as unknown as { playerHalfWidth?: number, playerHeight?: number }
      if (ph) {
        const { playerHalfWidth, playerHeight } = ph
        restoreHitbox = () => {
          ph.playerHalfWidth = playerHalfWidth
          ph.playerHeight = playerHeight
        }
        if (ph.playerHalfWidth === 0.3) ph.playerHalfWidth = 0.30001
        if (ph.playerHeight === 1.8) ph.playerHeight = 1.80001
      }
    }

    const workerHost = getSharedWorkerHost()
    if (opts.workerEntryPath) workerHost.setEntryPath(opts.workerEntryPath)
    if (options.logger !== undefined) workerHost.warn = warn
    // Prewarm the worker and WASM while idle to avoid delaying the first goal.
    if (opts.useWorkerThreads) workerHost.prewarm()

    let solveGeneration = 0
    // Enables per-solve latency diagnostics on stdout.
    const solveTimingOn = process.env.PF_SOLVE_TIMING === '1'
    let solveDispatchedAt = 0
    let goalSetAt = 0
    let firstDriveLogged = false
    /** Apply first controls between ticks so the next simulation uses them.
     * Mineflayer emits physicsTick after simulating movement. */
    const eagerStart = process.env.PF_NO_EAGER !== '1'
    /** True while monitorMovement is running (reentrancy guard for the eager drive). */
    let inTick = false
    /** Wall time of the last tick handler, for the exec trace. */
    let lastTickMs = 0
    /** Enables diagonal squeeze attempts before jump gates on refused flat steps. */
    const earlySqueeze = process.env.PF_NO_EARLY_SQUEEZE !== '1'
    let activeSolveCancel: (() => void) | null = null
    let activeSolveGeneration = -1
    let mainSolver: Solver | null = null
    let growFactor = 1
    let growAttempts = 0
    /** Wait for a column after an unloaded-column failure when chunk-arrival grace is enabled. */
    let awaitingChunks = false
    /** When the current growth sequence's first solve was dispatched. */
    let growStartedAt = 0
    let lastTouchedChunks: Set<string> | null = null

    let cachedSnapshot: Snapshot | null = null
    let cachedLut: BlockLut | null = null
    let snapshotStale = true
    const pendingPatches: Array<{ x: number, y: number, z: number, stateId: number }> = []

    const pf: Record<string, unknown> = {}
    ;(bot as unknown as { pathfinder: typeof pf }).pathfinder = pf

    pf.thinkTimeout = 5000 // ms
    pf.tickTimeout = 40 // ms per search slice; also the worker partial-path cadence
    pf.searchRadius = -1 // cost slack, -1: don't limit the search
    pf.enablePathShortcut = false
    pf.LOSWhenPlacingBlocks = true
    pf.stuckTimeout = -1 // ms without reaching the next node → path_stop (goto rejects PathStopped)
    pf.executionTimeout = -1 // ms per goal overall → path_stop
    pf.keepPathDuringRecompute = true // keep walking the valid prefix while replanning
    pf.minHealth = 0 // below it the goal stops once supported, and planned drops are capped; 0 = off
    pf.stopOnDeath = true // end the goal on death rather than walking back after the respawn

    pf.bestHarvestTool = (block: { digTime: (...args: unknown[]) => number }) => {
      const availableTools = bot.inventory.items()
      const effects = (bot.entity as { effects?: unknown }).effects

      let fastest = Number.MAX_VALUE
      let bestTool = null
      for (const tool of availableTools) {
        const enchants = (tool && tool.nbt) ? (nbt.simplify(tool.nbt) as { Enchantments?: unknown[] }).Enchantments : []
        const digTime = block.digTime(tool ? tool.type : null, false, false, false, enchants, effects)
        if (digTime < fastest) {
          fastest = digTime
          bestTool = tool
        }
      }
      return bestTool
    }

    Object.defineProperties(pf, {
      goal: { get () { return stateGoal } },
      movements: { get () { return stateMovements } }
    })

    pf.isMoving = () => path.length > 0
    pf.isMining = () => digging // only ever true when movements.canDig is enabled
    pf.isBuilding = () => placing // true while activating a door/gate

    pf.isStuck = () => geometry.isStuck(bot)
    pf.canStandAt = (pos: Vec3) => geometry.canStandAt(bot, pos)
    pf.findEscape = (radius?: number) => geometry.findEscape(bot, radius)
    pf.findNearestOpenStandable = (radius?: number) => geometry.findNearestOpenStandable(bot, radius)
    pf.unstick = async (): Promise<boolean> => {
      // An active goal's tick loop would override the escape controls.
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      return geometry.unstick(bot)
    }
    pf.centreInCell = (cell: Vec3, tolerance?: number) => geometry.centreInCell(bot, cell, tolerance)
    pf.jumpOnce = () => geometry.jumpOnce(bot)
    pf.settle = async (): Promise<boolean> => {
      // Clear the goal so the movement loop cannot fight settling or escape controls.
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      bot.clearControlStates()
      if (geometry.isStuck(bot)) await geometry.unstick(bot)
      return geometry.settle(bot)
    }

    /** On the ground, in water or on a climbable: letting go of the keys here drops the body nowhere. */
    function bodySupported (): boolean {
      const e = bot.entity as { onGround?: boolean, isInWater?: boolean } | undefined
      // Before login there is no body, and an interrupt's deadline timer can still ask.
      if (e === undefined) return false
      if (e.onGround === true || e.isInWater === true) return true
      // A climbable supports the bot even when onGround is false.
      const feet = bot.blockAt(bot.entity.position, false) as { type: number } | null
      return feet !== null && (climbableIds.has(feet.type))
    }

    // Yield only when the body is supported and no interaction is half-finished.
    const interrupts = new InterruptController(bot as unknown as { emit: (e: string, ...a: unknown[]) => boolean }, {
      isSafe: () => !digging && !placing && bodySupported(),
      onPause: () => {
        // The last writes before the holder owns the controls: release every executor key, with no
        // coast, so nothing stays held for the length of the pause. Before login there is nothing to release.
        handingOver = true
        try {
          if (bot.entity !== undefined) fullStop()
          clearWedge()
        } finally {
          handingOver = false
        }
        // A coast would keep writing; the holder decides what a flight does now.
        coastHold = false
        // Only a forced or timed-out request is granted mid-dig: that dig is the executor's, so it stops.
        if (digging || placing) cancelExecInteraction()
      },
      onResume: (pausedMs: number) => {
        // Exclude deliberate pauses from progress timers so they do not trigger stuck recovery.
        lastNodeTime += pausedMs
        lastNodeArrival += pausedMs
        goalSetTime += pausedMs
        clearWedge()
      }
    })

    /** True only inside onPause, whose writes release the executor's keys before the holder takes over. */
    let handingOver = false
    /** The body is still moving from this plugin's own stop, so idle ticks keep braking it. */
    let settling = false
    /** An exclusive interrupt holder owns every control: the executor writes none until release. */
    function handedOver (): boolean {
      return !handingOver && interrupts.exclusive
    }

    pf.interrupt = async (reason: string, options?: InterruptOptions): Promise<InterruptHandle> =>
      await interrupts.acquire(reason, options)
    /** Set inside an action or a withInterrupt: the caller already holds the bot, so nested actions run inline. */
    const holding = new AsyncLocalStorage<true>()

    pf.withInterrupt = async <T>(reason: string, fn: () => Promise<T>, options?: InterruptOptions): Promise<T> =>
      await interrupts.run(reason, async () => await holding.run(true, fn), options)

    /** Current movement phase for coordinating other actions. */
    function motionPhase (): MotionPhase {
      if (interrupts.paused) return 'paused'
      if (digging) return 'digging'
      if (placing) return 'interacting'
      if (path.length === 0) return 'idle'
      const e = bot.entity as { onGround?: boolean, isInWater?: boolean }
      if (e.isInWater === true) return 'swimming'
      if (e.onGround !== true) {
        const feet = bot.blockAt(bot.entity.position, false) as { type: number } | null
        if (feet && (climbableIds.has(feet.type))) return 'climbing'
        return 'airborne'
      }
      return 'walking'
    }

    pf.motion = {
      get phase (): MotionPhase { return motionPhase() },
      /** True when taking controls would interrupt a flight or an interaction.
       * Use interrupt() to wait for a safe handover. */
      get critical (): boolean {
        const phase = motionPhase()
        return phase === 'airborne' || phase === 'digging' || phase === 'interacting'
      },
      /** True when the next node is a jump the executor has not committed to yet. */
      get jumpPending (): boolean {
        return path.length > 0 && path[0].parkour === true &&
          (bot.entity as { onGround?: boolean }).onGround === true
      },
      get paused (): boolean { return interrupts.paused },
      get holders (): string[] { return interrupts.holders },
      get node (): Move | null { return path.length > 0 ? path[0] : null }
    }

    // Capture bot.dig before application overrides can delegate here and recurse.
    const rawDig = (bot as unknown as DiggingBot).dig.bind(bot)
    const pacer = new Pacer()

    /** Only a failed search leaves a nearer stance to try. Any other failure means the walk was taken away. */
    function approachFailure (error: unknown, pos: Vec3): void {
      const { name, message } = error as Error
      if (name === 'NoPath' || name === 'Timeout') return
      throw new ActionError(ActionErrors.ABORTED, `The walk to ${fmt(pos)} ended early: ${message}`, pos)
    }

    async function approachBlock (pos: Vec3, reach: number, signal?: AbortSignal): Promise<boolean> {
      if (inReach(bot as unknown as never, pos, reach)) return true
      if (signal?.aborted === true) throw new ActionError(ActionErrors.ABORTED, `The walk to ${fmt(pos)} was aborted.`, pos)
      // Release the interrupt while approaching, or the executor cannot move.
      const session = activeSession
      if ((session === null || session.handle === null) && interrupts.paused) {
        // Someone else's hold is what keeps the bot still, so a walk would never start.
        throw new ActionError(ActionErrors.OUT_OF_REACH, `${fmt(pos)} is out of reach, and the bot is paused by an interrupt.`, pos, true)
      }
      if (session?.handle) {
        session.handle.release()
        session.handle = null
      }
      let arrived = false
      try {
        await gotoImpl(new GoalLookAtBlock(pos, bot.world as never, { reach }), { signal })
        arrived = true
      } catch (error) {
        approachFailure(error, pos)
        try {
          await gotoImpl(new GoalNear(pos.x, pos.y, pos.z, Math.max(1, Math.floor(reach) - 1)), { signal })
          arrived = inReach(bot as unknown as never, pos, reach)
        } catch (nearError) {
          approachFailure(nearError, pos)
          arrived = false
        }
      }
      if (session) session.handle = await acquireForAction(session.reason, signal)
      return arrived
    }

    const actionCtx: Omit<ActionContext, 'equip'> = {
      bot: bot as unknown as DiggingBot,
      config: { ...DEFAULT_ACTION_CONFIG },
      pacer,
      rawDig,
      msSinceForcedMove: () => lastForcedMoveAt === 0 ? Number.POSITIVE_INFINITY : performance.now() - lastForcedMoveAt,
      serverTps: () => {
        const tps = (bot as unknown as { getServerTps?: () => number }).getServerTps
        return typeof tps === 'function' ? tps.call(bot) : null
      },
      bestHarvestTool: (block) => (pf.bestHarvestTool as (b: unknown) => unknown)(block),
      approach: approachBlock
    }

    /** Shared interaction table; replacing an entry also changes executor interactions. */
    const actions: ActionTable = createActionTable(actionCtx)
    pf.actions = actions

    // Approach only when out of reach, then hold an interrupt during the interaction.
    // Serialise actions because the bot has only one hand.
    interface ActionSession { reason: string, handle: InterruptHandle | null }
    let activeSession: ActionSession | null = null
    let actionChain: Promise<unknown> = Promise.resolve()

    /** Fires when the bot goes away, so that nothing keeps swinging or walking for an action. */
    const lifecycle = new AbortController()
    bindLifecycle(bot, lifecycle.signal)

    async function acquireForAction (reason: string, signal: AbortSignal | undefined): Promise<InterruptHandle> {
      try {
        return await interrupts.acquire(reason, { timeout: 5000, signal })
      } catch (error) {
        throw new ActionError(ActionErrors.ABORTED, (error as Error).message)
      }
    }

    async function runAction<T> (reason: string, signal: AbortSignal, fn: () => Promise<T>): Promise<T> {
      const abortedWaiting = (): ActionError => new ActionError(ActionErrors.ABORTED, `'${reason}' was aborted while waiting for its turn.`)
      // Queueing behind the caller's own hold would wait forever.
      if (holding.getStore() === true) {
        if (signal.aborted) throw abortedWaiting()
        return await fn()
      }
      let started = false
      const start = async (): Promise<T> => {
        started = true
        if (signal.aborted) throw abortedWaiting()
        const session: ActionSession = { reason, handle: null }
        session.handle = await acquireForAction(reason, signal)
        activeSession = session
        try {
          return await holding.run(true, fn)
        } finally {
          activeSession = null
          session.handle?.release()
        }
      }
      const run = actionChain.then(start, start)
      actionChain = run.then(() => undefined, () => undefined)
      // An abort needs no turn. Once running, the action answers the signal itself.
      let onAbort = (): void => {}
      const abandoned = new Promise<never>((_resolve, reject) => {
        onAbort = () => { if (!started) reject(abortedWaiting()) }
        signal.addEventListener('abort', onAbort, { once: true })
      })
      try {
        return await Promise.race([run, abandoned])
      } finally {
        signal.removeEventListener('abort', onAbort)
      }
    }

    /** Queue an action, walk into range unless told not to, then act. */
    async function interaction<O extends { approach?: boolean, reach?: number, signal?: AbortSignal }, T> (
      reason: string, target: BlockTarget, options: O, act: (pos: Vec3, options: O) => Promise<T>
    ): Promise<T> {
      const signal = options.signal === undefined ? lifecycle.signal : AbortSignal.any([options.signal, lifecycle.signal])
      return await runAction(reason, signal, async () => {
        const pos = toVec3(target)
        if (options.approach !== false) await approachBlock(pos, options.reach ?? actions.config.reach, signal)
        throwIfAborted(signal, pos)
        return await act(pos, { ...options, signal })
      })
    }

    pf.dig = async (target: BlockTarget, options: DigOptions = {}): Promise<void> => {
      // Given a block, dig that block: something else in its place by the time the bot arrives is not it.
      const expected = typeof (target as { name?: unknown }).name === 'string' ? (target as { name: string }).name : null
      await interaction('pathfinder:dig', target, options, async (pos, o) => {
        const live = resolveBlock(bot as unknown as never, target)
        if (expected !== null && live.name !== expected && !AIR_NAMES.has(live.name)) {
          throw new ActionError(ActionErrors.NO_TARGET, `Block at ${fmt(pos)} is ${live.name}, expected ${expected}.`, pos, true)
        }
        await actions.dig(live, o)
      })
    }

    pf.place = async (target: BlockTarget, options: PlaceOptions = {}): Promise<void> =>
      await interaction('pathfinder:place', target, options, async (pos, o) => await actions.place(pos, o))

    pf.open = async (target: BlockTarget, options: OpenOptions = {}): Promise<WindowLike> =>
      await interaction('pathfinder:open', target, options, async (_pos, o) =>
        await actions.open(resolveBlock(bot as unknown as never, target), o))

    pf.activate = async (target: BlockTarget, options: ActivateOptions = {}): Promise<void> =>
      await interaction('pathfinder:activate', target, options, async (_pos, o) =>
        await actions.activate(resolveBlock(bot as unknown as never, target), o))


    function flushPatches (): void {
      // Never mutate shared snapshot memory while a solve reads it.
      // Queue every update so an older queued write cannot overwrite a newer one.
      if (solveInFlight()) return
      if (!cachedSnapshot || !cachedLut) {
        pendingPatches.length = 0
        return
      }
      for (const p of pendingPatches) {
        applySnapshotBlockUpdate(cachedSnapshot, cachedLut, p.x, p.y, p.z, p.stateId)
      }
      pendingPatches.length = 0
    }

    function solveInFlight (): boolean {
      return activeSolveCancel !== null || mainSolver !== null
    }

    /** Goal timestamp the cached snapshot was last built or checked for. */
    let snapshotGoal = -1
    /** Maximum cached-box size relative to a new goal's required box;
     * reusing a much larger box makes follow-up searches needlessly expensive. */
    const INHERITED_BOX_RATIO = 4

    /** Snapshot floor for deep bounces, two blocks below the lowest reachable slime top.
     * The hop pipeline can only search pools included in the snapshot. */
    let deepFloorGoal = -1
    let deepFloorY: number | null = null
    /** Floor probe remains valid until a chunk arrives in the box or movements change. */
    let deepFloorFresh = false

    function deepFloor (start: Vec3, targets: Array<{ x: number, y?: number, z: number }>, slack: number, lut: BlockLut): number | null {
      if (deepFloorGoal === goalSetTime && deepFloorFresh) return deepFloorY
      // Repeated probes retain the lowest floor found for this goal.
      const prev = deepFloorGoal === goalSetTime ? deepFloorY : null
      deepFloorGoal = goalSetTime
      deepFloorFresh = true
      deepFloorY = prev
      if (!(stateMovements.allowParkourPhysics && stateMovements.allowParkourExtended)) return deepFloorY
      const ys = [start.y, ...targets.map(t => t.y ?? start.y)]
      const top = Math.max(...ys)
      const box = computeBox(bot, start, targets, slack, 1, opts.maxSnapshotCells)
      const tops = slimeUnderBox(bot, lut, box, DEEP_DROP, Math.min(...ys)).filter(t => t[1] >= top - DEEP_DROP)
      if (tops.length > 0) {
        const y = Math.min(...tops.map(t => t[1])) - 2
        deepFloorY = deepFloorY === null ? y : Math.min(deepFloorY, y)
      }
      return deepFloorY
    }

    function ensureSnapshot (start: Vec3, descriptor: GoalDescriptor | null, needStates: boolean): Snapshot {
      const lut = getLut(bot, stateMovements)
      const slack = pf.searchRadius as number
      const targets = descriptor ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })

      const lutChanged = cachedLut !== lut
      const floorY = deepFloor(start, targets, slack, lut) ?? undefined
      let rebuild = snapshotStale || lutChanged || !cachedSnapshot
      if (!rebuild && cachedSnapshot) {
        // The start and targets need margin inside the cached snapshot.
        const m = cachedSnapshot.meta
        const inside = (x: number, y: number, z: number): boolean =>
          x >= m.x0 + 8 && x <= m.x0 + m.w - 9 && y >= m.y0 + 4 && y <= m.y0 + m.h - 5 &&
          z >= m.z0 + 8 && z <= m.z0 + m.l - 9
        if (!inside(start.x, start.y, start.z)) rebuild = true
        for (const t of targets) {
          if (!inside(t.x, t.y ?? start.y, t.z)) rebuild = true
        }
        if (needStates && !cachedSnapshot.states) rebuild = true
        // Include a deep pool if the cached box does not yet reach it.
        if (floorY !== undefined && m.y0 > Math.max((bot.game as { minY?: number }).minY ?? 0, floorY)) rebuild = true
        // Avoid inheriting an oversized snapshot from another goal.
        if (!rebuild && snapshotGoal !== goalSetTime) {
          const b = computeBox(bot, start, targets, slack, growFactor, opts.maxSnapshotCells, floorY)
          const want = (b.x1 - b.x0 + 1) * (b.y1 - b.y0 + 1) * (b.z1 - b.z0 + 1)
          if (m.w * m.h * m.l > INHERITED_BOX_RATIO * want) rebuild = true
        }
      }
      snapshotGoal = goalSetTime

      if (rebuild) {
        const box: Box = computeBox(bot, start, targets, slack, growFactor, opts.maxSnapshotCells, floorY)
        cachedSnapshot = buildSnapshot(bot, lut, box, needStates, nextSnapshotGeneration())
        cachedLut = lut
        snapshotStale = false
        pendingPatches.length = 0
      } else {
        flushPatches()
      }
      return cachedSnapshot as Snapshot
    }


    function cancelActiveSolve (): void {
      solveGeneration++
      if (activeSolveCancel) {
        activeSolveCancel()
        activeSolveCancel = null
      }
      activeSolveGeneration = -1
      mainSolver = null
      // Generations discard cancelled results; flush updates once the solve is no longer in flight.
      flushPatches()
    }

    function computeStartMove (startPos: Vec3): { x: number, y: number, z: number } {
      let p = startPos.floored()
      // An overhanging body's centre may be above air in the next column.
      // Start from the cell supporting its hitbox so the planner does not route into the gap.
      if (bot.entity.onGround === true &&
          geometry.blockShapes(bot, p).length === 0 &&
          geometry.blockShapes(bot, p.offset(0, -1, 0)).length === 0) {
        for (const [ox, oz] of [[-0.29, -0.29], [0.29, -0.29], [-0.29, 0.29], [0.29, 0.29]]) {
          const c = new Vec3(Math.floor(startPos.x + ox), p.y, Math.floor(startPos.z + oz))
          if ((c.x !== p.x || c.z !== p.z) &&
              geometry.blockShapes(bot, c).length === 0 &&
              geometry.blockShapes(bot, c.offset(0, -1, 0)).length > 0) {
            p = c
            break
          }
        }
      }
      const dy = startPos.y - p.y
      const b = bot.blockAt(p) as BlockLike | null
      // Uses stateMovements.emptyBlocks even for a query with different movements,
      // matching mineflayer-pathfinder's start-cell behaviour.
      const offset = (b && dy > 0.001 && bot.entity.onGround && !stateMovements.emptyBlocks.has(b.type)) ? 1 : 0
      return { x: p.x, y: p.y + offset, z: p.z }
    }

    function toResult (raw: RawSolveResult, context: ComputedPathResult['context']): ComputedPathResult {
      return {
        status: raw.status,
        cost: raw.cost,
        time: raw.time,
        visitedNodes: raw.visitedNodes,
        generatedNodes: raw.generatedNodes,
        path: Move.expandRawPath(raw.path),
        context
      }
    }

    /** Whether the search reaches a chunk adjacent to or containing unloaded columns. */
    function touchedMissing (raw: RawSolveResult, missing: Set<string>): boolean {
      for (const [cx, cz] of raw.touchedChunks) {
        for (let dx = -1; dx <= 1; dx++) {
          for (let dz = -1; dz <= 1; dz++) if (missing.has(`${cx + dx},${cz + dz}`)) return true
        }
      }
      return false
    }

    function contextFromRaw (raw: RawSolveResult): ComputedPathResult['context'] {
      const visitedChunks = new Set<string>()
      for (const [cx, cz] of raw.touchedChunks) visitedChunks.add(`${cx},${cz}`)
      return { visitedChunks }
    }

    function handleDriveResult (raw: RawSolveResult, generation: number, final: boolean): void {
      if (generation !== solveGeneration) return // stale (goal changed etc.)

      // During chunk-arrival grace, retry failures that reach unloaded columns when a column arrives.
      // Grow boundary-limited snapshots first; missing columns alone do not prove the route is blocked.
      if (final && raw.status === 'noPath' && !(raw.boundaryLimited && growAttempts < 5) &&
          cachedSnapshot !== null && cachedSnapshot.missingColumns.size > 0 &&
          performance.now() - goalSetAt < CHUNK_WAIT_MS && touchedMissing(raw, cachedSnapshot.missingColumns)) {
        awaitingChunks = true
        activeSolveCancel = null
        activeSolveGeneration = -1
        mainSolver = null
        pathUpdated = false
        lastSolveFrom = null
        flushPatches()
        return
      }

      // Retry boundary-limited failures with larger snapshots within one shared think budget.
      if (final && raw.status === 'noPath' && raw.boundaryLimited && growAttempts < 5 &&
          performance.now() - growStartedAt < (pf.thinkTimeout as number)) {
        const snap = cachedSnapshot
        if (snap && snap.cellCount < opts.maxSnapshotCells) {
          growAttempts++
          growFactor *= 1.8
          snapshotStale = true
          activeSolveCancel = null
          activeSolveGeneration = -1
          mainSolver = null
          // This retry emits no update, so clear the latch to let the tick loop start another solve.
          pathUpdated = false
          lastSolveFrom = null
          flushPatches()
          return // next physicsTick startSolve()s again with the bigger box
        }
      }

      // Capture solver inputs before queued patches mutate the snapshot.
      if (final && raw.status === 'noPath' && opts.onNoPath && cachedSnapshot && lastSolveGoal !== null && lastSolveCfg !== null) {
        try {
          opts.onNoPath({
            meta: cachedSnapshot.meta,
            flags: cachedSnapshot.flags,
            heights: cachedSnapshot.heights,
            start: lastSolveStart,
            goal: lastSolveGoal,
            cfg: lastSolveCfg,
            visitedNodes: raw.visitedNodes
          })
        } catch (error) {
          warn(`[bulba-pathfinder] the onNoPath hook threw: ${String(error)}`)
        }
      }

      if (final) {
        activeSolveCancel = null
        activeSolveGeneration = -1
        flushPatches()
      }
      // Keep deep-bounce replay and abort state across path updates, including grounded rebound ticks.
      // Replan only after landing or settling in the basin.
      if (deepHeld()) {
        if (final) pathUpdated = true
        return
      }

      const results = toResult(raw, contextFromRaw(raw))
      lastTouchedChunks = results.context?.visitedChunks ?? null
      const postT0 = solveTimingOn ? performance.now() : 0
      if (!inTick) physics.beginTick?.() // between ticks: never reuse the last tick's block cache
      results.path = postProcessPath(results.path)
      // A committed flight must keep its landing when a path arrives mid-air.
      // The nearest node in 3D can lie behind or beyond that landing and misdirect the descent.
      const flying = (bot.entity as { onGround?: boolean }).onGround !== true && path.length > 0 &&
        ((path[0] as { parkour?: boolean }).parkour === true || (progNode === path[0] && progPhase === 1 && progFired))
      const n0 = path.length > 0 ? path[0] as Move : undefined
      const hasProg = n0 !== undefined && progNode === n0 && n0.program !== null && n0.program !== undefined
      // Keep a replay's verified controls across updates, including a streamed run-up.
      // Replacing its node would restart line-up or hand the remaining flight to different gates.
      const replaying = hasProg && progPhase === 1 && ((STREAM_STITCH && pathPartial) || (FLIGHT_TRIM && progFired))
      // Identical streamed programs retain line-up presses, coast, and attempt counts.
      const liningUp = STREAM_STITCH && pathPartial && hasProg && progPhase === 0
      // Every update must connect to the streamed leg already flown by this solve.
      // Reject disconnected partials; replan complete paths from the body or its committed landing.
      const streamedHeld = streamedLanding
      if (STREAM_STITCH) {
        // Track this solve's streamed flight separately from executor-generated live flights.
        const committed = n0 !== undefined && pathPartial && (n0 as Move & { streamGen?: number }).streamGen === generation &&
          ((hasProg && progPhase === 1 && progFired) || (bot.entity as { onGround?: boolean }).onGround !== true)
        const held = committed ? (n0 as Move).cell : streamedLanding
        if (held !== null && !results.path.some(n => n.cell[0] === held[0] && n.cell[1] === held[1] && n.cell[2] === held[2])) {
          if (!final) return
          if (execTraceFile !== null) takeoffNote = 'final-off-landing'
          // A successful solve resets its growth budget.
          if (raw.status === 'success') {
            growFactor = 1
            growAttempts = 0
            awaitingChunks = false
          }
          streamedLanding = null
          lastSolveFrom = null
          if (flying) {
            path.length = 1
            pathUpdated = true
          } else {
            // Stop before any unflown leg of the abandoned branch.
            const k = path.findIndex(n => (n as Move).program !== null && (n as Move).program !== undefined && n !== progNode)
            if (k >= 0) path.length = k
            pathUpdated = false
          }
          return
        }
      }
      if (final) streamedLanding = null
      // Mark pipeline-owned partial legs before carrying an executor program onto their nodes.
      if (!final && STREAM_STITCH) for (const n of results.path) if ((n as Move).program !== null && (n as Move).program !== undefined) (n as Move & { streamGen?: number }).streamGen = generation
      let flightAt = -1
      if (flying || replaying || liningUp) {
        const [fx, fy, fz] = (n0 as Move).cell
        flightAt = results.path.findIndex(n => n.cell[0] === fx && n.cell[1] === fy && n.cell[2] === fz)
      }
      // Preserve line-up only for the same program at the same place.
      if (liningUp && !flying && !replaying && flightAt >= 0) {
        const a = (n0 as Move).program as HopProgram
        const b = (results.path[flightAt] as Move).program
        if (b === null || b === undefined || !sameProgram(a, b)) flightAt = -1
      }
      // A run-up is not committed; discard it if the new path omits its node.
      const carried = (replaying || liningUp) && flightAt >= 0
      if (flightAt >= 0) results.path.splice(0, flightAt)
      // Carry a live skip-ahead flight onto the new path's landing node.
      if (flightAt >= 0 && skip !== null && path[0] === skip.target && progNode === skip.target && progPhase === 1 && results.path.length > 0) {
        const landing = results.path[0] as Move
        landing.program = skip.target.program
        progNode = landing
        skip.target = landing
      } else if (carried && results.path.length > 0) {
        const landing = results.path[0] as Move
        landing.program = (n0 as Move).program
        // Carried pipeline legs stay streamed; executor-generated flights do not become streamed legs.
        const g = (n0 as Move & { streamGen?: number }).streamGen
        if (g !== undefined) (landing as Move & { streamGen?: number }).streamGen = g
        progNode = landing
        if (linedInstalled === n0) linedInstalled = landing
        if (restSearchedAt !== null && restSearchedAt.node === n0) restSearchedAt = { ...restSearchedAt, node: landing }
        if (prefixSearched.has(n0 as Move)) prefixSearched.add(landing)
        if (PREFIX_RETRY_CAP && prefixRetried.has(n0 as Move)) prefixRetried.add(landing)
        if (takeoffSearched === n0) takeoffSearched = landing
        if (linedSearched === n0) linedSearched = landing
        // A chain node retains ownership of its in-flight controls.
        if (chainNode === n0) chainNode = landing
      }
      // Continue after the streamed landing already reached instead of returning to it.
      if (flightAt < 0 && !flying && STREAM_STITCH && streamedHeld !== null) {
        const k = results.path.findIndex(n => n.cell[0] === streamedHeld[0] && n.cell[1] === streamedHeld[1] && n.cell[2] === streamedHeld[2])
        if (k >= 0) results.path.splice(0, k + 1)
      }
      if (flightAt >= 0) {
      } else if (flying && FLIGHT_TRIM) {
        // Trim from the committed landing; the node nearest an airborne body may lie behind it.
        const n0 = path[0]
        const pr = progNode === n0 && progPhase === 1 ? n0.program : null
        // A region program's landing position matters because one cell can hold distinct places beside a pane.
        const land = pr !== null && pr !== undefined
          ? (pr.land !== undefined ? new Vec3(pr.land.x, pr.land.y, pr.land.z) : new Vec3(pr.tx + 0.5, pr.ty, pr.tz + 0.5))
          : new Vec3(n0.x, n0.y, n0.z)
        pathFromPlayer(results.path, land)
        results.path.unshift(n0)
      } else {
        pathFromPlayer(results.path)
        if (flying) results.path.unshift(path[0])
      }
      // Partial paths can change direction, so cut drops the body cannot climb back from.
      // Keep descents from the last final path, but wait for a complete path before jumping onto ice.
      if (!final && results.path.length > 0) {
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        // Preserve a committed landing; other airborne motion starts the drop check at the first node.
        const from = flying || carried || !grounded ? 1 : 0
        let prevY = from === 0 ? bot.entity.position.y : results.path[0].y
        for (let i = from; i < results.path.length; i++) {
          const n = results.path[i]
          const onIce = (n as { parkour?: boolean }).parkour === true && (() => {
            const under = bot.blockAt(new Vec3(Math.floor(n.x), Math.floor(n.y) - 1, Math.floor(n.z)), false) as { name?: string } | null
            return under !== null && under.name !== undefined && ICE_NAMES.has(under.name)
          })()
          if ((n.y < prevY - PARTIAL_ONE_WAY_DROP || onIce) &&
              !(i > 0 && finalEdges !== null && finalEdges.has(edgeKey(results.path[i - 1], n)))) {
            results.path.length = i
            break
          }
          prevY = n.y
        }
      }
      // An empty partial means the search is still running; emitting it makes goto() report arrival.
      // Stop the current partial without replacing it with an empty path.
      if (!final && results.path.length === 0) {
        if (path.length > 0 && (bot.entity as { onGround?: boolean }).onGround === true) {
          path = []
          fullStop()
        }
        return
      }
      if (solveTimingOn) {
        const now = performance.now()
        console.log(`[pf-timing] result ${raw.status} engine=${raw.engine ?? 'js'} final=${final} engineMs=${raw.time.toFixed(1)} visited=${raw.visitedNodes} nodes=${results.path.length} postMs=${(now - postT0).toFixed(2)} wallSinceDispatch=${(now - solveDispatchedAt).toFixed(1)} sinceGoal=${(now - goalSetAt).toFixed(1)}`)
      }
      if (final) {
        finalEdges = new Set()
        for (let i = 1; i < results.path.length; i++) finalEdges.add(edgeKey(results.path[i - 1], results.path[i]))
      }
      if (results.path.length > 0) coastHold = false
      if (climbHold && results.path.length > 0) {
        climbHold = false
        if (!handedOver()) bot.setControlState('sneak', false)
      }
      bot.emit('path_update' as never, results as never)
      // A listener can have replaced or stopped the goal: this path is no longer wanted.
      // A failed goto's stop waits for support only when a flight needs this path to land.
      if (generation !== solveGeneration) return
      checkPendingStop()
      if (stopPathing) {
        internalStop()
        return
      }
      const wasEmpty = path.length === 0
      // Streamed updates can trim a reached node before the arrival loop sees it.
      // Count a nearby retired head as progress without resetting the timer for every partial.
      if (!final && path.length > 0 && results.path.length > 0) {
        const old = path[0]
        const gone = !results.path.some(n => n.x === old.x && n.y === old.y && n.z === old.z)
        if (gone && bot.entity.position.distanceTo(old) <= PARTIAL_PROGRESS_REACH) {
          lastNodeTime = performance.now()
          lastNodeArrival = lastNodeTime
        }
      }
      const firstOfSolve = pathGen !== generation
      // A failed retreat carries no program; replacing its Move must not grant another prefix search.
      if (LIVE_PREFIX && !firstOfSolve && n0 !== undefined &&
          (prefixSearched.has(n0) || (PREFIX_RETRY_CAP && prefixRetried.has(n0)))) {
        const [x, y, z] = n0.cell
        for (const n of results.path) {
          if (n.cell[0] !== x || n.cell[1] !== y || n.cell[2] !== z) continue
          if (prefixSearched.has(n0)) prefixSearched.add(n as Move)
          if (PREFIX_RETRY_CAP && prefixRetried.has(n0)) prefixRetried.add(n as Move)
        }
      }
      path = results.path
      pathPartial = !final
      pathGen = generation
      // Anchor the first creep to the solve's supporting start cell, not an overhanging body's centre.
      // Later updates from that solve retain the most recently retired node as the anchor.
      if (!STREAM_STITCH || firstOfSolve || prevRetired === null ||
          Math.hypot(bot.entity.position.x - (lastSolveStart.x + 0.5), bot.entity.position.z - (lastSolveStart.z + 0.5)) <= 1.2) {
        prevRetired = { x: lastSolveStart.x + 0.5, y: lastSolveStart.y, z: lastSolveStart.z + 0.5, cell: [lastSolveStart.x, lastSolveStart.y, lastSolveStart.z] }
      }
      if (final) {
        pathUpdated = true
        // Give a complete path a fresh futility window; frequent partials must not prevent stuck detection.
        lastNodeTime = performance.now()
        // Reset growth after success so the next solve starts with its own box and budget.
        if (raw.status === 'success') {
          growFactor = 1
          growAttempts = 0
          awaitingChunks = false
        }
      }
      // A path update is not a node arrival and must not extend correction grace.
      // Drive the first path immediately between ticks; streamed replacements keep the normal cadence.
      if (eagerStart && wasEmpty && path.length > 0 && !inTick && stateGoal !== null) {
        monitorMovement()
      }
    }

    /** Hold a deep-bounce path from the first replay tick, including take-off delay,
     * walk-off delay, and any basin abort. */
    function deepHeld (): boolean {
      if (basin !== null) return true
      const n0 = path[0] as Move | undefined
      return n0 !== undefined && progNode === n0 && progPhase === 1 && n0.program?.deep === true
    }

    /** Has the bot left the block the last solve was dispatched from? */
    function movedSinceLastSolve (): boolean {
      if (lastSolveFrom === null) return true
      const p = bot.entity.position.floored()
      return p.x !== lastSolveFrom.x || p.y !== lastSolveFrom.y || p.z !== lastSolveFrom.z
    }

    function startSolve (): void {
      if (!stateGoal || !stateMovements) return
      // A solve starts at the body, so a streamed leg already behind it cannot validate the new path.
      streamedLanding = null
      try {
        stateMovements.assertSupported(warn)
      } catch (error) {
        // Movement changes can bypass setMovements; fail the goal rather than throw from physicsTick.
        warn(`[bulba-pathfinder] unsupported movements profile, stopping the goal: ${(error as Error).message}`)
        stopReason = 'unsupported'
        internalStop()
        return
      }

      const generation = ++solveGeneration
      const startPos = bot.entity.position
      const start = computeStartMove(startPos)
      if (growAttempts === 0) growStartedAt = performance.now()

      if (stateMovements.allowEntityDetection) {
        stateMovements.clearCollisionIndex()
        stateMovements.updateCollisionIndex()
      }

      const descriptor = serializeGoal(stateGoal)
      lastSolveStart = start
      lastSolveFrom = startPos.floored()
      lastSolveGoal = descriptor
      lastSolveCfg = solveConfig()
      const canUseWorker =
        opts.useWorkerThreads &&
        descriptor !== null &&
        stateMovements.exclusionAreasStep.length === 0 &&
        (!stateMovements.canDig || stateMovements.exclusionAreasBreak.length === 0) &&
        !workerHost.unavailable

      // The hop pipeline needs per-cell state IDs to read exact collision boxes.
      const needStates = (descriptor !== null && goalNeedsRaycast(descriptor)) || stateMovements.canDig ||
        (stateMovements.allowParkourPhysics && stateMovements.allowParkourExtended)
      const snapT0 = solveTimingOn ? performance.now() : 0
      const hadSnapshot = cachedSnapshot !== null && !snapshotStale
      const snapshot = ensureSnapshot(startPos, descriptor, needStates)
      bakeEntityIndex(snapshot, stateMovements)
      applyBans(snapshot)
      if (solveTimingOn) {
        solveDispatchedAt = performance.now()
        const m = snapshot.meta
        console.log(`[pf-timing] dispatch gen=${generation} snapshot=${hadSnapshot && cachedSnapshot === snapshot ? 'cached' : 'built'} ${m.w}x${m.h}x${m.l}=${snapshot.cellCount} cells snapMs=${(solveDispatchedAt - snapT0).toFixed(1)} sinceGoal=${(solveDispatchedAt - goalSetAt).toFixed(1)}`)
      }
      const digData = stateMovements.canDig ? getDigDataCached(stateMovements) : null

      if (canUseWorker && descriptor) {
        const req = {
          snapshot,
          lut: cachedLut as BlockLut,
          cfg: solveConfig(),
          goal: descriptor,
          start,
          timeout: pf.thinkTimeout as number,
          searchRadius: pf.searchRadius as number,
          sliceMs: Math.max(10, pf.tickTimeout as number),
          dig: digData,
          body: stateMovements.allowParkourPhysics
            ? {
                halfWidth: (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3,
                height: (bot.physics as unknown as { playerHeight?: number }).playerHeight ?? 1.8,
                speed: physics.walkSpeed?.() ?? 0.1
              }
            : undefined,
          hopsOnBoundary: growAttempts === 0,
          // A rest beside a pane can differ from the start node's position within the same cell.
          startPos: bot.entity.onGround === true ? { x: startPos.x, y: startPos.y, z: startPos.z } : undefined,
          // Match the worker kernel to the bot's jump rule.
          vanillaJump: physics.probeVanillaJump?.() ?? false,
          core: opts.core,
          onPartial: (raw: RawSolveResult) => handleDriveResult(raw, generation, false)
        }
        // Mark in-flight synchronously so the next tick doesn't double-start.
        activeSolveGeneration = generation
        activeSolveCancel = () => {} // placeholder until the handle arrives
        workerHost.solve(req).then((handle) => {
          if (!handle) {
            // Retry on the main thread if the worker is unavailable.
            if (generation === solveGeneration) {
              activeSolveCancel = null
              activeSolveGeneration = -1
              startMainThreadSolve(generation, snapshot, start)
            }
            return
          }
          if (generation !== solveGeneration) {
            handle.cancel()
            return
          }
          activeSolveCancel = handle.cancel
          handle.promise.then(
            (raw) => { handleDriveResult(raw, generation, true) },
            () => {
              // Worker crashed mid-solve: retry this goal on the main thread.
              if (generation === solveGeneration) {
                activeSolveCancel = null
                activeSolveGeneration = -1
                startMainThreadSolve(generation, snapshot, start)
              }
            }
          )
        }, () => {
          if (generation === solveGeneration) {
            activeSolveCancel = null
            activeSolveGeneration = -1
            startMainThreadSolve(generation, snapshot, start)
          }
        })
      } else {
        startMainThreadSolve(generation, snapshot, start)
      }
    }

    /** The profile as the solver sees it: with a health floor, drops are capped to what health can pay for. */
    function solveConfig (movements: Movements = stateMovements): MovementsConfig {
      const cfg = movements.toConfig()
      const spare = spareHealth()
      if (spare !== null) {
        cfg.maxDropDown = Math.min(cfg.maxDropDown, dropForHealth(spare))
        cfg.fallVeto = fallVetoForHealth(spare)
      }
      return cfg
    }

    /** Health above the floor, or null with no floor set. */
    function spareHealth (): number | null {
      const floor = pf.minHealth as number
      const health = (bot as unknown as { health?: number }).health
      return floor > 0 && typeof health === 'number' ? health - floor : null
    }

    function liveStepExclusion (x: number, y: number, z: number): number {
      const block = bot.blockAt(new Vec3(x, y, z), false)
      return block ? stateMovements.exclusionStep(block) : 0
    }

    function liveBreakExclusion (x: number, y: number, z: number): number {
      const block = bot.blockAt(new Vec3(x, y, z), false)
      return block ? stateMovements.exclusionBreak(block) : 0
    }

    // Dig tables cached by (lut × inventory × effects) fingerprint.
    let cachedDigData: DigData | null = null

    function getDigDataCached (movements: Movements): DigData {
      const lut = getLut(bot, movements)
      const fp = digFingerprint(bot, movements, lut)
      if (!cachedDigData || cachedDigData.fingerprint !== fp) {
        cachedDigData = computeDigData(bot, movements, lut)
      }
      return cachedDigData
    }

    function digContextFor (movements: Movements, snapshot: Snapshot, digData: DigData | null): DigContext | null {
      if (!movements.canDig || !digData) return null
      return {
        data: digData,
        states: snapshot.allocStates(),
        breakExclusion: movements.exclusionAreasBreak.length > 0 ? liveBreakExclusion : null
      }
    }

    function startMainThreadSolve (generation: number, snapshot: Snapshot, start: { x: number, y: number, z: number }): void {
      if (!stateGoal) return
      const stepExclusion = stateMovements.exclusionAreasStep.length > 0 ? liveStepExclusion : null
      const digData = stateMovements.canDig ? getDigDataCached(stateMovements) : null
      const descriptor = serializeGoal(stateGoal)
      const evaluator = (descriptor && fastEvaluator(descriptor)) ?? new GoalAdapter(stateGoal)
      const solver = new Solver(
        snapshot,
        solveConfig(),
        evaluator,
        start,
        { timeout: pf.thinkTimeout as number, searchRadius: pf.searchRadius as number },
        stepExclusion,
        digContextFor(stateMovements, snapshot, digData)
      )
      // Compute the first slice now; the movement loop continues later slices.
      const raw = solver.compute(pf.tickTimeout as number)
      if (raw.status === 'partial') {
        mainSolver = solver
        activeSolveGeneration = generation
      }
      handleDriveResult(raw, generation, raw.status !== 'partial')
    }


    function getPositionOnTopOf (block: BlockLike | null): Vec3 | null {
      if (!block || block.shapes.length === 0) return null
      const p = new Vec3(0.5, 0, 0.5)
      let n = 1
      for (const shape of block.shapes) {
        const h = shape[4]
        if (h === p.y) {
          p.x += (shape[0] + shape[3]) / 2
          p.z += (shape[2] + shape[5]) / 2
          n++
        } else if (h > p.y) {
          n = 2
          p.x = 0.5 + (shape[0] + shape[3]) / 2
          p.y = h
          p.z = 0.5 + (shape[2] + shape[5]) / 2
        }
      }
      p.x /= n
      p.z /= n
      return block.position.plus(p)
    }

    /** `from` and `movements` are the search's own: a query can start elsewhere, with another profile. */
    function postProcessPath (thePath: Move[], from: { x: number, y: number, z: number } = bot.entity.position, movements: Movements = stateMovements): Move[] {
      // Preserve node cells before landing aims move away from their centres.
      const cellXZ = thePath.map(m => [Math.floor(m.x), Math.floor(m.z)] as [number, number])
      for (let i = 0; i < thePath.length; i++) {
        const curPoint = thePath[i]
        if (curPoint.toBreak.length > 0 || curPoint.toPlace.length > 0) break
        const b = bot.blockAt(new Vec3(curPoint.x, curPoint.y, curPoint.z)) as BlockLike | null
        if (b && (b.type === waterType || b.type === bubbleColumnId ||
            ((climbableIds.has(b.type)) && i + 1 < thePath.length && thePath[i + 1].y < curPoint.y))) {
          curPoint.x = Math.floor(curPoint.x) + 0.5
          curPoint.y = Math.floor(curPoint.y)
          curPoint.z = Math.floor(curPoint.z) + 0.5
          continue
        }
        let np = getPositionOnTopOf(b)
        let support = b
        if (np === null) {
          support = bot.blockAt(new Vec3(curPoint.x, curPoint.y - 1, curPoint.z)) as BlockLike | null
          np = getPositionOnTopOf(support)
        }
        if (np) {
          curPoint.x = np.x
          curPoint.y = np.y
          curPoint.z = np.z
          if (support === b && b !== null && (climbableIds.has(b.type))) {
            // A ladder's top can sit inside a ceiling, making the bot climb into it indefinitely.
            // Aim within the ladder cell when the two cells above are not clear.
            const fx = Math.floor(curPoint.x)
            const fz = Math.floor(curPoint.z)
            const fy = Math.floor(np.y)
            const blocked = [fy, fy + 1].some(yy => {
              const bb = bot.blockAt(new Vec3(fx, yy, fz)) as BlockLike | null
              return bb !== null && !climbableIds.has(bb.type) && bb.shapes.length > 0
            })
            if (blocked) curPoint.y = fy - 1
          } else if (support !== null) {
            // Aim at the carrying face's centre to balance landing margin on narrow supports.
            // If a wall blocks that point, retain the wall-pressed aim.
            const faces = geometry.supportFaces(bot, support.position.x, support.position.y, support.position.z)
            if (faces.length > 0 && geometry.supportDepth(bot, faces, np.x, np.z) < 0.25) {
              let top = -Infinity
              for (const f of faces) if (f.top > top) top = f.top
              let sx = 0
              let sz = 0
              let n = 0
              for (const f of faces) {
                if (f.top === top) { sx += (f.x0 + f.x1) / 2; sz += (f.z0 + f.z1) / 2; n++ }
              }
              const cx = sx / n
              const cz = sz / n
              if (!geometry.playerCollides(bot, cx, np.y, cz)) {
                curPoint.x = cx
                curPoint.z = cz
              }
            }
          }
        } else {
          curPoint.x = Math.floor(curPoint.x) + 0.5
          curPoint.y = curPoint.y - 1
          curPoint.z = Math.floor(curPoint.z) + 0.5
        }
        // Land on the far side because the planner prices the re-jump with this support's creep credit.
        if (curPoint.aimDx !== 0 || curPoint.aimDz !== 0) {
          const half = support !== null && support.shapes.length > 0 ? CATCH_HALF[topCatchClass(support.shapes)] : 0.5
          const credit = Math.min(TAKEOFF_STAND, half + TAKEOFF_NARROW_MARGIN) * CHAIN_TAKEOFF_FRACTION
          const scale = credit / Math.max(Math.abs(curPoint.aimDx), Math.abs(curPoint.aimDz))
          curPoint.x += curPoint.aimDx * scale
          curPoint.z += curPoint.aimDz * scale
        }
      }
      // Shift take-off and landing aims to the collision-clearing flight line priced by the planner.
      for (let i = 0; i < thePath.length; i++) {
        const m = thePath[i]
        if ((m.pOff === 0 && m.qOff === 0) || m.toBreak.length > 0 || m.toPlace.length > 0) continue
        const [cx, cz] = cellXZ[i]
        const [px, pz] = i > 0 ? cellXZ[i - 1] : [Math.floor(from.x), Math.floor(from.z)]
        const ddx = cx - px
        const ddz = cz - pz
        const dl = Math.hypot(ddx, ddz)
        if (dl < 1e-6) continue
        const ux = -ddz / dl
        const uz = ddx / dl
        const prev = i > 0 ? thePath[i - 1] : { x: px + 0.5, z: pz + 0.5 }
        // A neo uses a run along the pillar face, not a fixed take-off point.
        m.takeoff = m.wOff !== 0 ? null : { x: prev.x + ux * m.pOff, z: prev.z + uz * m.pOff }
        m.x += ux * m.qOff
        m.z += uz * m.qOff
      }

      if (!(pf.enablePathShortcut as boolean) || movements.exclusionAreasStep.length !== 0 || thePath.length === 0) return thePath

      const newPath: Move[] = []
      let lastNode: { x: number, y: number, z: number } = from
      for (let i = 1; i < thePath.length; i++) {
        const node = thePath[i]
        if (Math.abs(node.y - lastNode.y) > 0.5 || node.toBreak.length > 0 || node.toPlace.length > 0 ||
            !physics.canStraightLineBetween(new Vec3(lastNode.x, lastNode.y, lastNode.z), node)) {
          newPath.push(thePath[i - 1])
          lastNode = thePath[i - 1]
        }
      }
      newPath.push(thePath[thePath.length - 1])
      return newPath
    }

    /** A path edge by its nodes' cells. */
    function edgeKey (a: Move, b: Move): string {
      return `${a.cell[0]},${a.cell[1]},${a.cell[2]}>${b.cell[0]},${b.cell[1]},${b.cell[2]}`
    }

    /** Trim from the body, or from the committed landing when a new path arrives mid-flight. */
    function pathFromPlayer (thePath: Move[], from: Vec3 = bot.entity.position): void {
      if (thePath.length === 0) return
      let minI = 0
      let minDistance = 1000
      for (let i = 0; i < thePath.length; i++) {
        const node = thePath[i]
        if (node.toBreak.length !== 0 || node.toPlace.length !== 0) break
        const dist = from.distanceSquared(node)
        if (dist < minDistance) {
          minDistance = dist
          minI = i
        }
      }
      const n1 = thePath[minI]
      const dx = n1.x - from.x
      const dy = n1.y - from.y
      const dz = n1.z - from.z
      const reached = Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1
      if (minI + 1 < thePath.length && n1.toBreak.length === 0 && n1.toPlace.length === 0) {
        const n2 = thePath[minI + 1]
        const d2 = from.distanceSquared(n2)
        const d12 = n1.distanceSquared(n2)
        minI += d12 > d2 || reached ? 1 : 0
      }
      thePath.splice(0, minI)
    }

    function isPositionNearPath (pos: Vec3 | undefined, thePath: Move[]): boolean {
      if (!pos) return false
      let prevNode: Move | null = null
      for (const node of thePath) {
        let comparisonPoint: { x: number, y: number, z: number } | null = null
        if (
          prevNode === null ||
          (
            Math.abs(prevNode.x - node.x) <= 2 &&
            Math.abs(prevNode.y - node.y) <= 2 &&
            Math.abs(prevNode.z - node.z) <= 2
          )
        ) {
          comparisonPoint = node
        } else {
          const minBound = prevNode.min(node)
          const maxBound = prevNode.max(node)
          if (
            pos.x - 0.5 < minBound.x - 1 ||
            pos.x - 0.5 > maxBound.x + 1 ||
            pos.y - 0.5 < minBound.y - 2 ||
            pos.y - 0.5 > maxBound.y + 2 ||
            pos.z - 0.5 < minBound.z - 1 ||
            pos.z - 0.5 > maxBound.z + 1
          ) {
            continue // Preserve prevNode across skipped segments for the next comparison.
          }
          comparisonPoint = closestPointOnLineSegment(pos, prevNode, node)
        }

        const dx = Math.abs(comparisonPoint.x - pos.x - 0.5)
        const dy = Math.abs(comparisonPoint.y - pos.y - 0.5)
        const dz = Math.abs(comparisonPoint.z - pos.z - 0.5)
        if (dx <= 1 && dy <= 2 && dz <= 1) return true

        prevNode = node
      }
      return false
    }

    /** Infer the supporting wall from a ladder's 3/16 collision strip.
     * Vines have no collision shape, so use a solid neighbour or return null when free-hanging. */
    function climbWall (feet: BlockLike, p: Vec3): [number, number] | null {
      const fx = Math.floor(p.x)
      const fy = Math.floor(p.y + 0.001)
      const fz = Math.floor(p.z)
      for (const s of feet.shapes) {
        if (s[3] - s[0] < 0.5) return [s[0] > 0.5 ? 1 : -1, 0]
        if (s[5] - s[2] < 0.5) return [0, s[2] > 0.5 ? 1 : -1]
      }
      for (const [ox, oz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nb = bot.blockAt(new Vec3(fx + ox, fy, fz + oz)) as BlockLike | null
        if (nb && !climbableIds.has(nb.type) && nb.boundingBox === 'block') return [ox, oz]
      }
      return null
    }

    function holeBeyond (node: Move, from: Vec3): boolean {
      const v = bot.entity.velocity
      let ux = v.x
      let uz = v.z
      let len = Math.hypot(ux, uz)
      if (len < 0.05) {
        ux = node.x - from.x
        uz = node.z - from.z
        len = Math.hypot(ux, uz)
        if (len < 1e-6) return false
      }
      ux /= len
      uz /= len
      for (const d of [1, 2]) {
        const x = node.x + ux * d
        const z = node.z + uz * d
        if (!geometry.playerCollides(bot, x, node.y, z) &&
            !geometry.playerCollides(bot, x, node.y - 0.55, z)) return true
      }
      return false
    }

    function closestPointOnLineSegment (point: Vec3, segmentStart: Vec3, segmentEnd: Vec3): Vec3 {
      const segmentLength = segmentEnd.minus(segmentStart).norm()
      if (segmentLength === 0) return segmentStart
      let t = (point.minus(segmentStart)).dot(segmentEnd.minus(segmentStart)) / segmentLength
      t = Math.max(0, Math.min(1, t))
      return segmentStart.plus(segmentEnd.minus(segmentStart).scaled(t))
    }


    /** Release climb controls when their path is gone so the bot cannot keep climbing
     * or walk off after a correction. Retain sneak only on climbables. */
    function releaseClimbHold (): void {
      if (!CLIMB_HOLD_ENABLED || (!climbKeysHeld && !climbHeldPrev)) return
      climbKeysHeld = false
      climbHeldPrev = false
      if (handedOver()) return
      bot.setControlState('jump', false)
      bot.setControlState('forward', false)
      const feet = bot.blockAt(bot.entity.position) as BlockLike | null
      bot.setControlState('sneak', (bot.entity as { onGround?: boolean }).onGround !== true && feet !== null && climbableIds.has(feet.type))
    }

    /** A grounded prefix is valid only from its verified start; pauses and corrections require a fresh search. */
    function dropUnfiredPrefix (): void {
      if (!LIVE_PREFIX || path.length === 0 || progNode !== path[0] || progPhase !== 1 || progFired) return
      const n = path[0] as Move
      if (n.program === null || n.program === undefined || n.program.groundPrefix === undefined) return
      // Restore a planned program, but clear a live hint whose verified start is stale.
      n.program = prefixPlan.get(n) ?? null
      progNode = null
      progPhase = 0
      if (!PREFIX_RETRY_CAP || !prefixRetried.has(n)) {
        prefixSearched.delete(n)
        if (PREFIX_RETRY_CAP) prefixRetried.add(n)
      }
    }

    function fullStop (): void {
      dropUnfiredPrefix()
      hopHold = false
      kernelHop = false
      // Stopping removes the momentum used to verify queued line-up presses.
      progPress = null
      cutTarget = null
      clearWedge()
      if (handedOver()) return
      // This stop's drift is ours to brake while idle, until the body is still or someone else drives.
      settling = true
      if (handingOver && flyingFree()) {
        // A handover in flight releases every key but keeps the flight's momentum.
        bot.clearControlStates()
        return
      }
      // Preserve an airborne flight to touchdown before stopping.
      if (coastIfFlying()) return
      bot.clearControlStates()
      // Sneak holds a climbable; releasing every key lets the bot slide off its bottom rung.
      // Release the hold when a path is installed.
      {
        const feet = bot.blockAt(bot.entity.position) as { type: number } | null
        climbHold = (bot.entity as { onGround?: boolean }).onGround !== true && feet !== null && climbableIds.has(feet.type)
        if (climbHold) bot.setControlState('sneak', true)
      }

      // These profiles release controls above and let normal physics settle the body.
      if (stateMovements.allowStopSnap === false) return

      // Cancel horizontal drift so inertia cannot carry the body past its support.
      bot.entity.velocity.x = 0
      bot.entity.velocity.z = 0

      const pos = bot.entity.position
      const blockX = Math.floor(pos.x) + 0.5
      const blockZ = Math.floor(pos.z) + 0.5
      // An overhanging hitbox can be supported while its centre is above air.
      // Snap only to a supported cell, or centring can push the body off its landing.
      const grounded = (bot.entity as { onGround?: boolean }).onGround === true
      if (grounded) {
        const under = bot.blockAt(new Vec3(blockX, pos.y - 0.01, blockZ)) as { shapes?: number[][], position?: Vec3 } | null
        const by = Math.floor(pos.y - 0.01)
        const carried = under !== null && (under.shapes ?? []).some(s => Math.abs(by + s[4] - pos.y) < 0.01)
        if (!carried) return
      }

      if (Math.abs(pos.x - blockX) > 0.2) { pos.x = blockX }
      if (Math.abs(pos.z - blockZ) > 0.2) { pos.z = blockZ }
    }


    /** Consecutive ticks the body has not left `wedgeAnchor`. */
    function updateWedge (p: Vec3): number {
      if (wedgeAnchor === null || p.distanceTo(wedgeAnchor) > WEDGE_MOVE) {
        wedgeAnchor = p.clone()
        wedgeTicks = 0
      } else {
        wedgeTicks++
      }
      return wedgeTicks
    }

    function applyRecovery (kind: number): void {
      const c = ESCAPES[kind]
      bot.setControlState('forward', c.forward === true)
      bot.setControlState('back', c.back === true)
      bot.setControlState('left', c.left === true)
      bot.setControlState('right', c.right === true)
      bot.setControlState('jump', c.jump === true)
      bot.setControlState('sprint', false)
      bot.setControlState('sneak', false)
    }

    /** Reconsider escape controls each tick to avoid unnecessary backtracking.
     * Keep a direction until ESCAPE_PATIENCE expires so recovery does not flicker. */
    function driveRecovery (nextPoint: Move): boolean {
      if (recoverNode !== nextPoint.hash) {
        recoverNode = nextPoint.hash
        nextEscape = 0
        recoverTicks = 0
      }
      if (recoverTicks > 0 && recoverTicks < ESCAPE_PATIENCE && physics.canNudge(ESCAPES[recoverKind])) {
        recoverTicks++
        applyRecovery(recoverKind)
        return true
      }
      for (let i = 0; i < ESCAPES.length; i++) {
        const kind = (nextEscape + i) % ESCAPES.length
        if (!physics.canNudge(ESCAPES[kind])) continue
        recoverKind = kind
        nextEscape = (kind + 1) % ESCAPES.length
        recoverTicks = 1
        applyRecovery(kind)
        return true
      }
      // With every escape blocked, wait for the futility replan and try a different direction next time.
      nextEscape = (nextEscape + 1) % ESCAPES.length
      recoverTicks = 0
      wedgeTicks = 0
      return false
    }

    /** Ban a failed jump's landing so replanning cannot keep returning the same take-off.
     * Snapshot entity weights make the ban apply to both solver cores. */
    function banTakeoff (node: { x: number, y: number, z: number, cell?: readonly [number, number, number] }): void {
      // Use the planner's cell because neo aims and slab heights can lie outside it.
      const c = node.cell
      bannedCells.set(
        c !== undefined ? `${c[0]},${c[1]},${c[2]}` : `${Math.floor(node.x)},${Math.floor(node.y + 0.001)},${Math.floor(node.z)}`,
        performance.now() + BAN_MS)
    }

    function applyBans (snap: Snapshot): void {
      if (bannedCells.size === 0) return
      const now = performance.now()
      const idxs: number[] = []
      const weights: number[] = []
      for (const [key, until] of bannedCells) {
        if (until <= now) { bannedCells.delete(key); continue }
        const [x, y, z] = key.split(',').map(Number)
        if (!snap.contains(x, y, z)) continue
        idxs.push(snap.index(x, y, z))
        weights.push(BAN_WEIGHT)
      }
      if (idxs.length === 0) return
      snap.entityIdx = Int32Array.from([...snap.entityIdx, ...idxs])
      snap.entityWeight = Int32Array.from([...snap.entityWeight, ...weights])
    }

    function futile (swimming: boolean): boolean {
      if (performance.now() - lastNodeTime <= (swimming ? 8000 : 3500)) return false
      // Swimming needs a longer progress window and keeps its controls while replanning.
      // Ban failed take-offs so the same jump cannot be chosen repeatedly.
      if (path.length > 0 && path[0].parkour) banTakeoff(path[0])
      resetPath('stuck', !swimming)
      return true
    }

    function clearWedge (): void {
      wedgeAnchor = null
      wedgeTicks = 0
      recoverTicks = 0
      recoverNode = ''
      nextEscape = 0
      headingNode = ''
      headingBias = null
      headingSprint = false
      biasFlight = false
      biasTicks = 0
    }

    /** Next jump's cell, used to plan the leg after a live take-off. */
    function nextLegCell (): readonly [number, number, number] | undefined {
      const n1 = path[1] as Move | undefined
      return n1 !== undefined && ((n1 as { parkour?: boolean }).parkour === true || (n1.program !== null && n1.program !== undefined)) ? n1.cell : undefined
    }

    /** Steer an aborted bounce toward the nearest inner pool top without sprint, jump, or sneak. */
    function steerBasin (p: Vec3): void {
      if (basin === null) return
      let best: [number, number, number] | null = null
      let bd = Infinity
      for (const t of basin.tops) {
        const d = (t[0] + 0.5 - p.x) ** 2 + (t[2] + 0.5 - p.z) ** 2
        if (d < bd) { bd = d; best = t }
      }
      const cx = best !== null ? best[0] + 0.5 - p.x : 0
      const cz = best !== null ? best[2] + 0.5 - p.z : 0
      const dd = Math.hypot(cx, cz)
      if (dd > 1e-9) bot.look(Math.atan2(-cx, -cz), 0, true)
      bot.setControlState('forward', dd > 0.2)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)
      bot.setControlState('jump', false)
      bot.setControlState('sprint', false)
      bot.setControlState('sneak', false)
    }

    // A head collision may prevent any rise. Wait for these keys to receive a physics tick
    // so an eager drive cannot count the preceding jump on the same body state.
    function jumpRearmedSince (tick: number, before: number | undefined): boolean {
      return tick >= 0 && physTicks === tick + 1 && jumpRearmed(before,
        (bot as unknown as { jumpTicks?: number }).jumpTicks,
        (bot.physics as unknown as { autojumpCooldown?: number }).autojumpCooldown)
    }

    /** Replay one program tick using the oracle's control schedule and replay counters. */
    function replayProgramTick (prog: HopProgram, p: Vec3): void {
      progDrivenAt = physTicks
      progJumpTicks = (bot as unknown as { jumpTicks?: number }).jumpTicks
      // End the retreat's walking and sneaking controls before the suffix jump.
      const prefix = LIVE_PREFIX ? prog.groundPrefix?.[progTick] : undefined
      if (prefix !== undefined) {
        bot.look(prefix.yaw, 0, true)
        bot.setControlState('forward', true)
        bot.setControlState('back', false)
        bot.setControlState('left', false)
        bot.setControlState('right', false)
        bot.setControlState('jump', false)
        bot.setControlState('sprint', prefix.sprint)
        bot.setControlState('sneak', prefix.sneak)
        progTick++
        return
      }
      // Sneak on slime touchdown to match the verified flight.
      const ent = bot.entity as { onGround?: boolean }
      const hw = (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3
      const sneak = sneakLanding(prog, ent.onGround === true, p.x, p.y, p.z, bot.entity.velocity.y, hw)
      if (prog.seq !== undefined) {
        // Replay beam controls exactly and hold the last input beyond the sequence.
        const k = Math.min(progTick, (prog.seq.length >> 1) - 1) << 1
        if (!Number.isNaN(prog.seq[k])) bot.look(prog.seq[k], 0, true)
        const flags = prog.seq[k + 1]
        bot.setControlState('forward', (flags & F_FWD) !== 0)
        bot.setControlState('back', false)
        bot.setControlState('left', false)
        bot.setControlState('right', false)
        bot.setControlState('sneak', sneak)
        bot.setControlState('sprint', (flags & F_SPRINT) !== 0)
        bot.setControlState('jump', (flags & F_JUMP) !== 0)
        progTick++
        return
      }
      const hd = headingDir(programHeading(prog, progFired, progAir, progRebounds, progTick), p.x, p.z)
      if (hd !== null) bot.look(Math.atan2(-hd[0], -hd[1]), 0, true)
      bot.setControlState('forward', true)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)
      bot.setControlState('sneak', sneak)
      bot.setControlState('sprint', prog.sprint !== false)
      // Leave programs fire on their latched tick; rebound jumps fire on the grounded tick after a bounce.
      bot.setControlState('jump', (!progFired && progTick >= prog.jumpAt && (prog.jumpOnLeave !== true || physics.latchedNow?.() === true)) ||
        (prog.reboundJump === true && progFired && progRebounds > 0 && ent.onGround === true))
      // A deep bounce caught below its target on a climbable keeps jumping toward the column centre.
      if (prog.deep === true && prog.catchTarget && progFired && p.y < prog.ty && Math.floor(p.x) === prog.tx && Math.floor(p.z) === prog.tz) {
        const feet = bot.blockAt(p) as BlockLike | null
        if (feet !== null && climbableIds.has(feet.type)) {
          const cx = prog.tx + 0.5 - p.x
          const cz = prog.tz + 0.5 - p.z
          const dd = Math.hypot(cx, cz)
          if (dd > 1e-9) bot.look(Math.atan2(-cx, -cz), 0, true)
          bot.setControlState('forward', dd > CLIMB_ON_SLACK)
          bot.setControlState('sprint', false)
          bot.setControlState('jump', true)
          bot.setControlState('sneak', false)
        }
      }
      progTick++
    }

    /** Search and cache a heading that clears the jump before trying escapes
     * that give up ground and require walking back. */
    function driveAngledJump (nextPoint: Move, dx: number, dz: number): boolean {
      if (headingNode !== nextPoint.hash) {
        headingNode = nextPoint.hash
        biasFlight = false
        biasTicks = 0
        headingSprint = false
        headingBias = physics.bestHeading(path, true, false)
        if (headingBias === null && stateMovements.allowSprinting) {
          headingBias = physics.bestHeading(path, true, true)
          headingSprint = headingBias !== null
        }
        // If straight-on already works, try positional escapes instead.
        if (headingBias === 0) headingBias = null
      }
      if (headingBias === null) return false
      // Bound this branch because a server-rejected heading must not bypass the gates forever.
      if (++biasTicks > ANGLE_PATIENCE) {
        headingBias = null
        return false
      }
      biasFlight = true
      bot.look(Math.atan2(-dx, -dz) + headingBias, 0)
      bot.setControlState('forward', true)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)
      bot.setControlState('sneak', false)
      bot.setControlState('jump', true)
      bot.setControlState('sprint', headingSprint)
      return true
    }

    /** `controls` false: the connection is gone, so nothing may be written to it. */
    function internalStop (controls = true): void {
      const reason = stopReason
      stopReason = 'stop'
      stopPathing = false
      pendingStop = null
      stateGoal = null
      path = []
      pathUpdated = false
      lastSolveFrom = null
      if (controls) stopDiggingIfNeeded()
      else digging = false
      // A door still being clicked belongs to the goal that just ended.
      if (placing) cancelExecInteraction()
      placing = false
      placingBlock = null
      cancelActiveSolve()
      bot.emit('path_stop' as never, reason as never)
      if (controls) fullStop()
    }

    function forgetDigListeners (): void {
      bot.removeListener('diggingAborted' as never, detectDiggingStopped as never)
      bot.removeListener('diggingCompleted' as never, detectDiggingStopped as never)
    }

    function detectDiggingStopped (): void {
      digging = false
      forgetDigListeners()
    }

    /** Aborts the dig or door the executor started for a path, including one still aiming. */
    let execInteraction: AbortController | null = null

    function execSignal (): AbortSignal {
      execInteraction = new AbortController()
      return AbortSignal.any([lifecycle.signal, execInteraction.signal])
    }

    function cancelExecInteraction (): void {
      execInteraction?.abort(new Error('The path no longer needs this interaction.'))
      execInteraction = null
    }

    function stopDiggingIfNeeded (): void {
      if (!digging) return
      // A dig still aiming has no swing for stopDigging to cancel.
      cancelExecInteraction()
      forgetDigListeners()
      bot.on('diggingAborted' as never, detectDiggingStopped as never)
      bot.on('diggingCompleted' as never, detectDiggingStopped as never)
      try {
        (bot as unknown as { stopDigging: () => void }).stopDigging()
      } catch {
        digging = false
      }
    }

    function resetPath (reason: string, clearStates = true): void {
      if (!stopPathing && path.length > 0) bot.emit('path_reset' as never, reason as never)
      if (execTraceFile !== null && path.length > 0) takeoffNote = `reset:${reason}${(bot.entity as { onGround?: boolean }).onGround === true ? '' : ':air'}`
      path = []
      cutTarget = null
      kernelHop = false
      progNode = null
      progPhase = 0
      progSettle = 0
      streamedLanding = null
      basin = null
      bounceNode = null
      bounceRisen = false
      chainNode = null
      chainFired = false
      runUpNode = null
      runUpAttempts = 0
      runUpPhase = 0
      runUpTicks = 0
      runUpLastPos = null
      lipWaitTicks = 0
      creepNode = null
      creepCell = null
      prevRetired = null
      stopDiggingIfNeeded()
      placing = false
      placingBlock = null
      pathUpdated = false
      // Each solve needs its own growth budget so later replans can still enlarge their snapshots.
      growFactor = 1
      growAttempts = 0
      clearWedge()
      cancelActiveSolve()
      lockUseBlock.release()
      stateMovements.clearCollisionIndex()
      if (handedOver()) {
        // The holder's controls stay as they are.
      } else if (clearStates && !coastIfFlying()) bot.clearControlStates()
      // Even when retaining controls, release climb holds that could carry the body past the path.
      else if (!clearStates) releaseClimbHold()
      if (stopPathing) internalStop()
    }

    /** Follow the valid prefix while recomputing, then splice onto the new path.
     * Reset immediately when there is no usable prefix or the option is disabled. */
    function softReplan (reason: string): void {
      if (!(pf.keepPathDuringRecompute as boolean) || path.length === 0 || placing) {
        resetPath(reason, false)
        return
      }
      if (!stopPathing) bot.emit('path_reset' as never, reason as never)
      pathUpdated = false
      cancelActiveSolve()
      stateMovements.clearCollisionIndex()
      if (stopPathing) internalStop()
    }


    pf.setGoal = (goal: Goal | null, dynamic = false) => {
      // A stop still pending belongs to the old goal, not this one.
      if (stopPathing) internalStop()
      pendingStop = null
      stateGoal = goal
      dynamicGoal = dynamic
      goalSetTime = performance.now()
      goalSetAt = goalSetTime
      // Losses from one fixed goal must not cause a ban on the next goal.
      if (LOST_PER_GOAL && !dynamic) lostEdges.clear()
      if (!dynamic) prefixGoalCount = 0
      firstDriveLogged = false
      growFactor = 1
      growAttempts = 0
      awaitingChunks = false
      bot.emit('goal_updated' as never, goal as never, dynamic as never)
      resetPath('goal_updated')
      // Solve immediately once the entity is in the world; the usual goal and in-flight guards still apply.
      if (eagerStart && stateGoal !== null && stateMovements !== undefined && !stopPathing &&
          bot.entity?.position !== undefined && Number.isFinite(bot.entity.position.x) &&
          !solveInFlight() && !stateGoal.isEnd(bot.entity.position.floored())) {
        startSolve()
      }
    }

    pf.setMovements = (movements: Movements) => {
      movements.assertSupported(warn)
      stateMovements = movements
      snapshotStale = true
      deepFloorFresh = false
      // Precompute the profile's cached block table to avoid delaying its first goal.
      // Failures are retried by the solve path.
      try {
        const lut = getLut(bot, movements)
        // Warm V8's snapshot loops on small local boxes before the first goal.
        const p = bot.entity?.position
        if (p !== undefined && Number.isFinite(p.x) && Number.isFinite(p.y)) {
          const box: Box = {
            x0: Math.floor(p.x) - 12, y0: Math.floor(p.y) - 8, z0: Math.floor(p.z) - 12,
            x1: Math.floor(p.x) + 12, y1: Math.floor(p.y) + 8, z1: Math.floor(p.z) + 12
          }
          for (let i = 0; i < 3; i++) buildSnapshot(bot, lut, box, false, nextSnapshotGeneration())
        }
      } catch { /* the solve path will retry */ }
      resetPath('movements_updated')
    }

    pf.stop = () => {
      // Unlike mineflayer-pathfinder, an idle stop does not cancel the next goal.
      if (!stateGoal && path.length === 0 && !solveInFlight()) return
      stopPathing = true
    }

    pf.goto = (goal: Goal, options?: GotoOptions) => {
      return gotoImpl(goal, options)
    }

    function gotoImpl (goal: Goal, options: GotoOptions = {}): Promise<void> {
      // Preserve mineflayer-pathfinder's event contract and error names for callers that match them.
      const bestEffort = options.bestEffort === true
      const maxLegs = options.maxBestEffortLegs ?? 3
      const signal = options.signal
      let legs = 0
      const refused = (): Error | null => signal?.aborted === true ? abortReason(signal) : ended !== null ? pathStoppedError(ended) : null
      let refusal = refused()
      // Before the listeners below: a stop still pending is the old goal's to report.
      // Its listeners can do anything, including abort this signal.
      if (refusal === null && stopPathing) {
        internalStop()
        refusal = refused()
      }
      if (refusal !== null) return Promise.reject(refusal)
      return new Promise((resolve, reject) => {
        let settled = false

        function makeError (name: string, message: string): Error {
          const err = new Error(message)
          err.name = name
          return err
        }

        function goalReached (): void {
          cleanup()
        }

        function noPathListener (results: ComputedPathResult): void {
          // An emit still calls a listener removed during it, and by then the goal may be someone else's.
          // An empty partial is a search still running, not an arrival.
          if (settled || results.status === 'partial') return
          if (results.status === 'success') {
            if (results.path.length === 0) cleanup()
            return
          }
          // A noPath can still provide a leg toward the closest reachable node.
          // Bound best-effort legs because a moving goal or changing world may never converge.
          if (results.status === 'noPath' && bestEffort && results.path.length > 0 && ++legs <= maxLegs) return
          // The promise is settling, so the goal cannot keep walking its partial path behind the caller's back.
          // It ends once the body is on something: a stop in flight would cut a committed landing.
          if (stateGoal === goal) pendingStop = results.status === 'noPath' ? 'no_path' : 'timeout'
          if (results.status === 'timeout') cleanup(makeError('Timeout', 'Took to long to decide path to goal!'))
          else cleanup(bestEffort ? undefined : makeError('NoPath', 'No path to the goal!'))
        }

        function goalChangedListener (newGoal: Goal): void {
          if (newGoal !== goal) {
            cleanup(makeError('GoalChanged', 'The goal was changed before it could be completed!'))
          }
        }

        function pathStopped (reason: PathStopReason = 'stop'): void {
          cleanup(pathStoppedError(reason))
        }

        function onAbort (): void {
          if (settled) return
          cleanup(abortReason(signal as AbortSignal))
          // A goal that has already replaced this one is someone else's to stop.
          if (stateGoal !== goal) return
          stopReason = 'aborted'
          internalStop()
        }

        function cleanup (err?: Error): void {
          if (settled) return
          settled = true
          bot.removeListener('goal_reached' as never, goalReached as never)
          bot.removeListener('path_update' as never, noPathListener as never)
          bot.removeListener('goal_updated' as never, goalChangedListener as never)
          bot.removeListener('path_stop' as never, pathStopped as never)
          signal?.removeEventListener('abort', onAbort)
          setTimeout(() => {
            if (err) reject(err)
            else resolve()
          }, 0)
        }

        bot.on('path_stop' as never, pathStopped as never)
        bot.on('goal_reached' as never, goalReached as never)
        bot.on('path_update' as never, noPathListener as never)
        bot.on('goal_updated' as never, goalChangedListener as never)
        signal?.addEventListener('abort', onAbort, { once: true })
        ;(pf.setGoal as (g: Goal | null, dynamic?: boolean) => void)(goal)
      })
    }

    pf.getPathFromTo = function * (movements: Movements, startPos: Vec3, goal: Goal, options: {
      optimizePath?: boolean
      resetEntityIntersects?: boolean
      timeout?: number
      tickTimeout?: number
      searchRadius?: number
      startMove?: { x: number, y: number, z: number }
    } = {}): Generator<{ result: ComputedPathResult, astarContext: unknown }> {
      movements.assertSupported(warn)
      const optimizePath = options.optimizePath ?? true
      const resetEntityIntersects = options.resetEntityIntersects ?? true
      const timeout = options.timeout ?? (pf.thinkTimeout as number)
      const tickTimeout = options.tickTimeout ?? (pf.tickTimeout as number)
      const searchRadius = options.searchRadius ?? (pf.searchRadius as number)

      let start: { x: number, y: number, z: number }
      if (options.startMove) {
        start = options.startMove
      } else {
        start = computeStartMove(startPos)
      }
      if (movements.allowEntityDetection) {
        if (resetEntityIntersects) movements.clearCollisionIndex()
        movements.updateCollisionIndex()
      }

      // Query snapshots are private so they cannot disturb the movement loop's cache.
      const lut = getLut(bot, movements)
      const descriptor = serializeGoal(goal)
      const targets = descriptor ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })
      const box = computeBox(bot, startPos, targets, searchRadius, 1, opts.maxSnapshotCells)
      const snap = buildSnapshot(bot, lut, box, (descriptor ? goalNeedsRaycast(descriptor) : true) || movements.canDig, nextSnapshotGeneration())
      bakeEntityIndex(snap, movements)
      applyBans(snap)

      const stepExclusion = movements.exclusionAreasStep.length > 0
        ? (x: number, y: number, z: number) => {
            const block = bot.blockAt(new Vec3(x, y, z), false)
            return block ? movements.exclusionStep(block) : 0
          }
        : null
      let queryDig: DigContext | null = null
      if (movements.canDig) {
        queryDig = {
          data: getDigDataCached(movements),
          states: snap.allocStates(),
          breakExclusion: movements.exclusionAreasBreak.length > 0
            ? (x: number, y: number, z: number) => {
                const block = bot.blockAt(new Vec3(x, y, z), false)
                return block ? movements.exclusionBreak(block) : 0
              }
            : null
        }
      }
      const solver = new Solver(snap, solveConfig(movements), new GoalAdapter(goal), start, { timeout, searchRadius }, stepExclusion, queryDig)
      const astarContext = {
        get visitedChunks () { return solver.visitedChunks },
        compute: () => toResult(solver.compute(tickTimeout), null)
      }

      // Read-only queries must not change lastTouchedChunks, which controls movement replans on chunk loads.
      let raw = solver.compute(tickTimeout)
      let result = toResult(raw, { visitedChunks: solver.visitedChunks })
      if (optimizePath) result.path = postProcessPath(result.path, startPos, movements)
      yield { result, astarContext }
      while (result.status === 'partial') {
        raw = solver.compute(tickTimeout)
        result = toResult(raw, { visitedChunks: solver.visitedChunks })
        if (optimizePath) result.path = postProcessPath(result.path, startPos, movements)
        yield { result, astarContext }
      }
    }

    pf.getPathTo = (movements: Movements, goal: Goal, timeout?: number) => {
      const generator = (pf.getPathFromTo as (...args: unknown[]) => Generator<{ result: ComputedPathResult, astarContext: unknown }>)(
        movements, bot.entity.position, goal, { timeout }
      )
      const { value } = generator.next()
      return (value as { result: ComputedPathResult }).result
    }

    /** One search of a computePath: on the worker when it can run there, else in slices on this thread. */
    async function runQuery (q: {
      movements: Movements
      snapshot: Snapshot
      lut: BlockLut
      goal: Goal
      descriptor: GoalDescriptor | null
      start: { x: number, y: number, z: number }
      startPos?: { x: number, y: number, z: number }
      timeout: number
      searchRadius: number
      first: boolean
      signal?: AbortSignal
    }): Promise<RawSolveResult> {
      const { movements, snapshot, descriptor, signal } = q
      const cfg = solveConfig(movements)
      const digData = movements.canDig ? getDigDataCached(movements) : null
      const useWorker = opts.useWorkerThreads && descriptor !== null && movements.exclusionAreasStep.length === 0 &&
        (!movements.canDig || movements.exclusionAreasBreak.length === 0) && !workerHost.unavailable
      if (useWorker && descriptor !== null) {
        const handle = await workerHost.solve({
          snapshot,
          lut: q.lut,
          cfg,
          goal: descriptor,
          start: q.start,
          timeout: q.timeout,
          searchRadius: q.searchRadius,
          sliceMs: Math.max(10, pf.tickTimeout as number),
          dig: digData,
          body: movements.allowParkourPhysics
            ? {
                halfWidth: (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3,
                height: (bot.physics as unknown as { playerHeight?: number }).playerHeight ?? 1.8,
                speed: physics.walkSpeed?.() ?? 0.1
              }
            : undefined,
          hopsOnBoundary: q.first,
          startPos: q.startPos,
          vanillaJump: physics.probeVanillaJump?.() ?? false,
          core: opts.core,
          onPartial: () => {}
        }).catch(() => null)
        if (handle !== null) {
          try {
            return await abortable(handle.promise, signal, handle.cancel)
          } catch (error) {
            if (signal?.aborted === true) throw error
            // The worker died mid-search: this thread takes over.
          }
        }
      }
      const blockFor = (x: number, y: number, z: number): Parameters<Movements['exclusionStep']>[0] | null =>
        bot.blockAt(new Vec3(x, y, z), false) as never
      const solver = new Solver(
        snapshot,
        cfg,
        (descriptor !== null ? fastEvaluator(descriptor) : null) ?? new GoalAdapter(q.goal),
        q.start,
        { timeout: q.timeout, searchRadius: q.searchRadius },
        movements.exclusionAreasStep.length > 0
          ? (x, y, z) => {
              const block = blockFor(x, y, z)
              return block !== null ? movements.exclusionStep(block) : 0
            }
          : null,
        digData !== null
          ? {
              data: digData,
              states: snapshot.allocStates(),
              breakExclusion: movements.exclusionAreasBreak.length > 0
                ? (x, y, z) => {
                    const block = blockFor(x, y, z)
                    return block !== null ? movements.exclusionBreak(block) : 0
                  }
                : null
            }
          : null
      )
      for (;;) {
        if (signal?.aborted === true) throw abortReason(signal)
        const raw = solver.compute(pf.tickTimeout as number)
        if (raw.status !== 'partial') return raw
        await new Promise(resolve => setImmediate(resolve))
      }
    }

    pf.computePath = async (goal: Goal, options: ComputePathOptions = {}): Promise<ComputedPathResult> => {
      const signal = options.signal
      if (signal?.aborted === true) throw abortReason(signal)
      const movements = options.movements ?? stateMovements
      movements.assertSupported(warn)
      // A copy: the bot keeps moving while this runs.
      const from = (options.from ?? bot.entity.position).clone()
      const timeout = options.timeout ?? (pf.thinkTimeout as number)
      const searchRadius = options.searchRadius ?? (pf.searchRadius as number)
      const start = computeStartMove(from)
      if (movements.allowEntityDetection) {
        movements.clearCollisionIndex()
        movements.updateCollisionIndex()
      }
      const descriptor = serializeGoal(goal)
      const targets = descriptor !== null ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })
      const lut = getLut(bot, movements)
      const needStates = descriptor === null || goalNeedsRaycast(descriptor) || movements.canDig ||
        (movements.allowParkourPhysics && movements.allowParkourExtended)
      const startPos = options.from === undefined && bot.entity.onGround === true
        ? { x: from.x, y: from.y, z: from.z }
        : undefined
      const startedAt = performance.now()
      let grow = 1
      for (let attempt = 0; ; attempt++) {
        // A private snapshot: the walking path's own one stays out of it.
        const box = computeBox(bot, from, targets, searchRadius, grow, opts.maxSnapshotCells)
        const snapshot = buildSnapshot(bot, lut, box, needStates, nextSnapshotGeneration())
        bakeEntityIndex(snapshot, movements)
        applyBans(snapshot)
        const raw = await runQuery({ movements, snapshot, lut, goal, descriptor, start, startPos, timeout, searchRadius, first: attempt === 0, signal })
        if (raw.status === 'noPath' && raw.boundaryLimited && attempt < 5 &&
            performance.now() - startedAt < timeout && snapshot.cellCount < opts.maxSnapshotCells) {
          grow *= 1.8
          continue
        }
        const result = toResult(raw, contextFromRaw(raw))
        result.path = postProcessPath(result.path, from, movements)
        return result
      }
    }


    listen('blockUpdate', ((oldBlock: BlockLike | null, newBlock: BlockLike | null) => {
      if (!oldBlock || !newBlock) return

      // Queue every patch in FIFO order so an older update cannot overwrite a newer one.
      if (cachedSnapshot && cachedSnapshot.contains(newBlock.position.x, newBlock.position.y, newBlock.position.z)) {
        pendingPatches.push({
          x: newBlock.position.x,
          y: newBlock.position.y,
          z: newBlock.position.z,
          stateId: newBlock.stateId ?? -1
        })
        flushPatches() // no-op while a solve is reading the SAB
      }

      if (isPositionNearPath(oldBlock.position, path) && oldBlock.type !== newBlock.type) {
        // Clear imminent obstructions; otherwise keep following the valid prefix during replanning.
        let imminent = false
        for (let i = 0; i < Math.min(3, path.length); i++) {
          const n = path[i]
          if (Math.abs(n.x - oldBlock.position.x) <= 1 && Math.abs(n.z - oldBlock.position.z) <= 1 &&
              Math.abs(n.y - oldBlock.position.y) <= 2) {
            imminent = true
            break
          }
        }
        // Deep flights and basin aborts keep their path until the per-tick verdict releases it.
        if (imminent && !deepHeld()) resetPath('block_updated', false)
        else softReplan('block_updated')
      }
    }) as never)

    listen('death', () => {
      releaseClimbHold()
      pendingStop = null
      if ((pf.stopOnDeath as boolean) && goalActive()) {
        stopReason = 'death'
        internalStop()
      }
      // A hold or a coast means nothing to a dead body, and must not outlive the respawn.
      climbHold = false
      coastHold = false
      if (!handedOver()) bot.clearControlStates()
    })

    // A respawn or a dimension change: the path and the snapshot describe another place.
    listen('respawn', () => {
      snapshotStale = true
      deepFloorFresh = false
      if (path.length > 0 || solveInFlight()) resetPath('respawn')
    })

    // No tick ever comes again: whatever is waiting on one has to be told now.
    listen('end', () => {
      ended = 'disconnected'
      interrupts.close('The bot disconnected.')
      if (goalActive()) {
        stopReason = 'disconnected'
        internalStop(false)
      }
      lifecycle.abort(new Error('The bot disconnected.'))
    })

    function goalActive (): boolean {
      return stateGoal !== null || path.length > 0 || solveInFlight()
    }

    listen('forcedMove', (() => {
      lastForcedMoveAt = performance.now()
      forcedAtTick = physTicks
      // Corrections invalidate line-up presses verified for the old body state.
      progPress = null
      dropUnfiredPrefix()
      if (path.length === 0) {
        // Replan from the corrected body if the initial solve's start is now too far away.
        const at = bot.entity.position
        if (FORCED_IDLE_REPLAN && solveInFlight() && stateGoal !== null && pathGen !== activeSolveGeneration &&
            (Math.hypot(at.x - (lastSolveStart.x + 0.5), at.z - (lastSolveStart.z + 0.5)) > FORCED_IDLE_REACH || Math.abs(at.y - lastSolveStart.y) > FORCED_IDLE_REACH)) {
          resetPath('forced_move', false)
        }
        return
      }
      // Deep-flight verdicts use the corrected body before deciding whether to release the path.
      if (deepHeld()) return
      const at = bot.entity.position.floored()
      // A corner-cut chord is intentionally off-node, so small corrections retain it.
      const nearCut = cutTarget !== null && prevTickPos !== null &&
        bot.entity.position.distanceTo(prevTickPos) <= CUT_LAG_NEAR
      if (nearCut || isPositionNearPath(at, path)) {
        if (!nearCut) pathFromPlayer(path)
        // Repeated corrections can mask a geometry wedge indefinitely.
        // Stop extending progress grace once the last actual arrival is too old.
        if (performance.now() - lastNodeArrival < FORCED_MOVE_GRACE_MS) {
          lastNodeTime = performance.now()
        }
      } else {
        resetPath('forced_move', false)
        if (FORCED_RELEASE) {
          bot.setControlState('forward', false)
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
          bot.setControlState('jump', false)
          bot.setControlState('sprint', false)
        }
      }
    }) as never)

    listen('chunkColumnLoad', ((chunk: Vec3) => {
      const cx = chunk.x >> 4
      const cz = chunk.z >> 4
      if (cachedSnapshot) {
        const m = cachedSnapshot.meta
        if ((cx << 4) + 15 >= m.x0 && (cx << 4) <= m.x0 + m.w - 1 &&
            (cz << 4) + 15 >= m.z0 && (cz << 4) <= m.z0 + m.l - 1) {
          snapshotStale = true
          deepFloorFresh = false
        }
      }
      if (lastTouchedChunks && (stateGoal !== null || path.length > 0)) {
        if (lastTouchedChunks.has(`${cx - 1},${cz}`) ||
            lastTouchedChunks.has(`${cx},${cz - 1}`) ||
            lastTouchedChunks.has(`${cx + 1},${cz}`) ||
            lastTouchedChunks.has(`${cx},${cz + 1}`)) {
          softReplan('chunk_loaded')
        }
      }
    }) as never)

    // ── the tick loop ────────────────────

    /**
     * A fire, lava or cobweb cell in the 3x3 columns round the body's, from the
     * cell under its feet to its head's: what a coast of a few tenths could
     * reach (PARTIAL_END_COAST - holdsOn sees neither fire nor lava: a coast
     * into them still "holds").
     */
    function hazardAround (): boolean {
      const p = bot.entity.position
      const bx = Math.floor(p.x)
      const by = Math.floor(p.y + 0.001)
      const bz = Math.floor(p.z)
      for (let x = bx - 1; x <= bx + 1; x++) {
        for (let z = bz - 1; z <= bz + 1; z++) {
          for (let y = by - 1; y <= by + 1; y++) {
            const n = (bot.blockAt(new Vec3(x, y, z), false) as { name?: string } | null)?.name
            if (n === 'fire' || n === 'soul_fire' || n === 'lava' || n === 'cobweb') return true
          }
        }
      }
      return false
    }

    /**
     * The lava and fire cells the body touches: its box (deflated by 0.001,
     * as vanilla tests it) against each cell's hazard volume — fire the
     * whole cell, lava up to its fluid height.
     */
    function hazardContacts (): Array<[number, number, number]> {
      const pos = bot.entity.position
      const hw = (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3
      const hh = (bot.physics as unknown as { playerHeight?: number }).playerHeight ?? 1.8
      const x0 = pos.x - hw + 0.001
      const x1 = pos.x + hw - 0.001
      const z0 = pos.z - hw + 0.001
      const z1 = pos.z + hw - 0.001
      const y0 = pos.y + 0.001
      const y1 = pos.y + hh - 0.001
      const out: Array<[number, number, number]> = []
      for (let x = Math.floor(x0); x <= Math.floor(x1); x++) {
        for (let z = Math.floor(z0); z <= Math.floor(z1); z++) {
          for (let y = Math.floor(y0); y <= Math.floor(y1); y++) {
            const b = bot.blockAt(new Vec3(x, y, z), false) as { name?: string, getProperties?: () => Record<string, unknown> } | null
            if (b === null || b.name === undefined) continue
            if (b.name === 'fire' || b.name === 'soul_fire') { out.push([x, y, z]); continue }
            if (b.name !== 'lava') continue
            const above = bot.blockAt(new Vec3(x, y + 1, z), false) as { name?: string } | null
            const level = Number(b.getProperties?.().level ?? 0)
            const top = above !== null && above.name === 'lava' ? y + 1 : y + (level === 0 || level >= 8 ? 8 : 8 - level) / 9
            if (top >= y0) out.push([x, y, z])
          }
        }
      }
      return out
    }

    /**
     * One tick of walking out of lava or fire, or false when the body
     * touches neither. The target is the body centre pushed clear of each
     * hazard face beside it (the column it stands on keeps it carried); a
     * hazard in its own column, or faces on both sides of an axis, sends it
     * to the nearest standable cell with no lava beside it instead.
     */
    function hazardEscape (): boolean {
      const hits = hazardContacts()
      if (hits.length === 0) return false
      const pos = bot.entity.position
      const hw = (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3
      const fx = Math.floor(pos.x)
      const fz = Math.floor(pos.z)
      const lo = [-Infinity, -Infinity]
      const hi = [Infinity, Infinity]
      let own = false
      for (const [hx, , hz] of hits) {
        if (hx === fx && hz === fz) { own = true; break }
        if (hx < fx) lo[0] = Math.max(lo[0], hx + 1 + hw + 0.01)
        if (hx > fx) hi[0] = Math.min(hi[0], hx - hw - 0.01)
        if (hz < fz) lo[1] = Math.max(lo[1], hz + 1 + hw + 0.01)
        if (hz > fz) hi[1] = Math.min(hi[1], hz - hw - 0.01)
      }
      let tx = pos.x
      let tz = pos.z
      if (!own && lo[0] <= hi[0] && lo[1] <= hi[1]) {
        tx = Math.min(hi[0], Math.max(lo[0], pos.x))
        tz = Math.min(hi[1], Math.max(lo[1], pos.z))
      } else {
        const lavaBeside = (c: Vec3): boolean => [[1, 0], [-1, 0], [0, 1], [0, -1], [0, 0]].some(([ox, oz]) => {
          const b = bot.blockAt(c.offset(ox, 0, oz), false) as { name?: string } | null
          return b !== null && (b.name === 'lava' || b.name === 'fire' || b.name === 'soul_fire')
        })
        let best: Vec3 | null = null
        let bestD = Infinity
        const o = pos.floored()
        for (let dx = -3; dx <= 3; dx++) {
          for (let dz = -3; dz <= 3; dz++) {
            for (let dy = -1; dy <= 1; dy++) {
              const c = o.offset(dx, dy, dz)
              if (!geometry.isStandable(bot, c) || lavaBeside(c)) continue
              const d = Math.hypot(c.x + 0.5 - pos.x, c.z + 0.5 - pos.z) + Math.abs(dy) * 0.5
              if (d < bestD) { bestD = d; best = c }
            }
          }
        }
        if (best !== null) { tx = best.x + 0.5; tz = best.z + 0.5 }
      }
      const ex = tx - pos.x
      const ez = tz - pos.z
      const away = Math.hypot(ex, ez) > 1e-3
      if (away) bot.look(Math.atan2(-ex, -ez), 0, true)
      bot.setControlState('forward', away)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)
      bot.setControlState('sprint', false)
      // (sneak keeps a walk-out on its ledge; in a pool, swim up instead)
      const inLava = (bot.entity as { isInLava?: boolean }).isInLava === true
      bot.setControlState('sneak', !inLava)
      bot.setControlState('jump', inLava)
      // (the futility clock is not running while the body gets clear)
      lastNodeTime = performance.now()
      if (execTraceFile !== null) takeoffNote = 'escape ' + tx.toFixed(2) + ',' + tz.toFixed(2)
      return true
    }

    function monitorMovement (): void {
      // Reentrancy guard: the eager first drive (handleDriveResult) may call
      // this between ticks, and a path_update listener may set a new goal
      // from inside it — never run two tick bodies nested.
      if (inTick) return
      inTick = true
      const t0 = performance.now()
      // Only the tick that renews the climb hold owns its control keys.
      climbHeldPrev = climbKeysHeld
      climbKeysHeld = false
      try {
        physics.beginTick?.()
        geometry.beginTick()
        monitorMovementInner()
      } finally {
        climbHeldPrev = false
        geometry.endTick()
        inTick = false
        lastTickMs = performance.now() - t0
      }
    }

    /** Below the health floor the goal stops, once a stop cannot drop the body somewhere the plan never meant. */
    function checkPendingStop (): void {
      if (pendingStop === null || stopPathing || !bodySupported()) return
      stopReason = pendingStop
      pendingStop = null
      stopPathing = true
    }

    function checkMinHealth (): void {
      const spare = spareHealth()
      physics.fallVeto = spare === null ? FALL_VETO : fallVetoForHealth(spare)
      if (spare === null || spare >= 0 || stopPathing || !goalActive() || !bodySupported()) return
      stopReason = 'low_health'
      stopPathing = true
      bot.emit('pathfinder:low_health' as never, (bot as unknown as { health: number }).health as never, pf.minHealth as never)
    }

    function monitorMovementInner (): void {
      if (skip !== null && (path.length === 0 || path[0] !== skip.target)) skip = null
      // A support brake lasts one tick; the no-path branch does not release it.
      if (holdKeyed) {
        holdKeyed = false
        bot.setControlState('forward', false)
        bot.setControlState('back', false)
      }
      // Release strafe each tick so braking cannot leave it held and walk the bot off support.
      if (gaitStrafed) {
        gaitStrafed = false
        bot.setControlState('left', false)
      }
      // Clear the hop replay on landing so a later flight cannot reuse its air controls.
      if (kernelHop && (bot.entity as { onGround?: boolean }).onGround === true) kernelHop = false
      // A reset preserves airborne momentum until touchdown unless a new path takes control.
      if (coastHold && !handedOver()) {
        if (path.length > 0) coastHold = false
        else if (!coastIfFlying()) {
          coastHold = false
          fullStop()
        }
      }
      checkPendingStop()
      checkMinHealth()
      // A requested stop releases controls even in mid-edge.
      if (stopPathing) {
        internalStop()
        return
      }

      // A pause preserves the current path so movement resumes from the same node.
      if (interrupts.gate()) {
        execBranch = 'interrupt'
        // Holding jump keeps a shared pause from sinking in water. An exclusive holder owns jump too.
        if (!handedOver() && (bot.entity as { isInWater?: boolean }).isInWater === true) {
          bot.setControlState('jump', true)
        }
        return
      }

      // A take-off search from liquid cannot find a jump, so hazard escape precedes replanning.
      if (stateGoal !== null && hazardEscape()) {
        execBranch = 'escape'
        return
      }

      const freeGoal = stateGoal as (Goal & { entity?: { position: Vec3 }, rangeSq?: number }) | null
      if (stateMovements && stateMovements.allowFreeMotion && freeGoal && freeGoal.entity) {
        const target = freeGoal.entity
        if (physics.canStraightLine([target.position])) {
          bot.lookAt(target.position.offset(0, 1.6, 0))
          // An entity goal without rangeSq leaves forward movement disabled.
          const rangeSq = freeGoal.rangeSq
          if (rangeSq !== undefined && target.position.distanceSquared(bot.entity.position) > rangeSq) {
            bot.setControlState('forward', true)
          } else {
            bot.clearControlStates()
          }
          return
        }
      }

      if (stateGoal) {
        if (!stateGoal.isValid()) {
          internalStop()
        } else if (stateGoal.hasChanged()) {
          softReplan('goal_moved')
        }
      }

      if (stateGoal) {
        const now = performance.now()
        if ((pf.executionTimeout as number) > 0 && now - goalSetTime > (pf.executionTimeout as number)) {
          internalStop()
          return
        }
        if ((pf.stuckTimeout as number) > 0 && path.length > 0 && !placing && !digging &&
            now - lastNodeTime > (pf.stuckTimeout as number)) {
          internalStop()
          return
        }
      }

      if (mainSolver) {
        const generation = activeSolveGeneration
        const raw = mainSolver.compute(pf.tickTimeout as number)
        if (raw.status !== 'partial') mainSolver = null
        handleDriveResult(raw, generation, raw.status !== 'partial')
      }

      if (path.length === 0) {
        lastNodeTime = performance.now()
        if (stateGoal && stateMovements) {
          if (stateGoal.isEnd(bot.entity.position.floored())) {
            if (!dynamicGoal) {
              cancelActiveSolve()
              // Settle first: a listener may set the next goal, and that one must survive.
              const reached = stateGoal
              stateGoal = null
              fullStop()
              bot.emit('goal_reached' as never, reached as never)
              return
            }
            // A dynamic goal can move again, so reaching its current endpoint does not end pathing.
          } else if (!solveInFlight() && (!pathUpdated || movedSinceLastSolve()) &&
              // Wait for a chunk in the search box or for the chunk timeout.
              !(awaitingChunks && !snapshotStale && performance.now() - goalSetAt < CHUNK_WAIT_MS)) {
            // An exhausted path can end short of the goal. Re-solve only after changing blocks
            // so a stationary bot does not repeatedly search for an unreachable goal.
            startSolve()
          }
        }
      } else if (stateGoal && stateMovements && !pathUpdated && !solveInFlight()) {
        // Soft replan: keep walking the old prefix while recomputing.
        startSolve()
      }

      if (path.length === 0) {
        execBranch = solveInFlight() ? 'nopath-solving' : 'nopath'
        // With no goal the controls belong to whoever wants them. Only the drift of this
        // plugin's own stop is still braked, until the body is still or someone else drives.
        if (!goalActive()) {
          const cs = bot.controlState as unknown as Record<string, boolean>
          const v = bot.entity.velocity
          const still = (bot.entity as { onGround?: boolean }).onGround === true && Math.hypot(v.x, v.z) < HOLD_STILL
          if (still || cs.forward === true || cs.back === true || cs.left === true || cs.right === true) settling = false
          if (!settling) {
            execBranch = 'idle'
            return
          }
        }
        // Soft replans preserve forward motion, but stale escape keys can steer off-path.
        // Hold jump in water to avoid sinking during a solve.
        // Release climb holds here because terminal results can bypass resetPath.
        releaseClimbHold()
        if ((bot.entity as { isInWater?: boolean }).isInWater === true) {
          bot.setControlState('jump', true)
        }
        if (recoverTicks > 0) {
          clearWedge()
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
        }
        // Brake residual momentum only when a soft replan is not driving forward.
        if ((bot.controlState as unknown as Record<string, boolean>).forward !== true && (bot.controlState as unknown as Record<string, boolean>).sprint !== true && holdSupport()) execBranch = 'hold'
        return
      }

      let nextPoint: Move = path[0]
      const p = bot.entity.position
      if (solveTimingOn && !firstDriveLogged) {
        firstDriveLogged = true
        console.log(`[pf-timing] first drive tick sinceGoal=${(performance.now() - goalSetAt).toFixed(1)} inTick=${inTick}`)
      }

      if (digging || nextPoint.toBreak.length > 0) {
        if (!digging && bot.entity.onGround) {
          digging = true
          const b = nextPoint.toBreak.shift() as Vec3
          const block = bot.blockAt(new Vec3(b.x, b.y, b.z), false)
          if (!block) {
            digging = false
            resetPath('dig_error')
            return
          }
          fullStop()
          const pathToken = path
          // Use the action table so application overrides and digging guards apply.
          // Disable approach because the planner has already chosen the standing position.
          actions.dig(block as never, { approach: false, equipTool: true, signal: execSignal() })
            .then(() => {
              if (path !== pathToken) return
              lastNodeTime = performance.now()
            }, (err: unknown) => {
              bot.emit('pathfinder:dig_error' as never, err as never)
              if (path === pathToken) resetPath('dig_error')
            })
            .then(() => {
              digging = false
            })
        }
        return
      }

      // Doors and fence gates require activation rather than block placement.
      if (placing || nextPoint.toPlace.length > 0) {
        if (!placing) {
          placing = true
          placingBlock = nextPoint.toPlace.shift() ?? null
          fullStop()
        }
        if (placingBlock?.useOne) {
          if (!lockUseBlock.tryAcquire()) return
          const block = bot.blockAt(new Vec3(placingBlock.x, placingBlock.y, placingBlock.z))
          if (!block) {
            lockUseBlock.release()
            resetPath('place_error')
            return
          }
          // Async callbacks must not modify a path installed while activation is pending.
          const pathToken = path
          // Use the action table so application overrides also apply to doors.
          // Disable settling because re-aiming during traversal can stall at the door.
          actions.activate(block as never, { approach: false, retries: 1, settle: false, signal: execSignal() }).then(() => {
            lockUseBlock.release()
            if (path !== pathToken) return
            placingBlock = nextPoint.toPlace.shift() ?? null
            if (!placingBlock) {
              placing = false
              lastNodeTime = performance.now()
            }
          }, (err: unknown) => {
            warn(`[bulba-pathfinder] opening a door failed: ${String(err)}`)
            bot.emit('pathfinder:activate_error' as never, err as never)
            lockUseBlock.release()
            if (path !== pathToken) return
            resetPath('place_error')
          })
          return
        }
        // Paths only contain activation actions; clear any other placement defensively.
        placing = false
        placingBlock = null
        return
      }

      // isInWater includes shallow contact, where the bot can still walk.
      // Require water at head height before switching to swimming controls.
      const head = bot.blockAt(bot.entity.position.offset(0, 1, 0)) as { type: number } | null
      const swimming = (bot.entity as { isInWater?: boolean }).isInWater === true &&
        head !== null && (head.type === waterType || head.type === bubbleColumnId)

      let dx = nextPoint.x - p.x
      let dy = nextPoint.y - p.y
      let dz = nextPoint.z - p.z
      // Sprint hops can pass above the arrival box and cross multiple nodes in one tick.
      // Consume each passed node to avoid turning back toward ground already crossed.
      const arriveDy = hopHold ? HOP_ARRIVE_DY : 1
      // Ground, water and climbables all count as a landing.
      const caughtNow = (() => {
        const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
        if (ent.onGround === true || ent.isInWater === true) return true
        const feet = bot.blockAt(p) as BlockLike | null
        return feet !== null && (climbableIds.has(feet.type))
      })()
      {
        const groundedNow = (bot.entity as { onGround?: boolean }).onGround === true
        if (!groundedNow) landedTicksAgo = 99
        else if (lastTickGrounded) landedTicksAgo = Math.min(99, landedTicksAgo + 1)
        else landedTicksAgo = 0
        lastTickGrounded = groundedNow
      }
      // Skipped-node retirement requires a cut target ahead of path[0] in the current path.
      if (cutTarget !== null && path.indexOf(cutTarget) < 1) cutTarget = null
      for (;;) {
        // A verified hop reaches its node only after its program jumps
        // and lands in the target column.
        const progHere = (nextPoint as Move).program
        let progLanded = false
        if (progHere !== null && progHere !== undefined) {
          progLanded = progNode === nextPoint && progPhase === 1 && progFired && caughtNow &&
            // Require body overlap with the target block by LAND_OVERLAP.
            Math.abs(p.x - progHere.tx - 0.5) < 0.8 - LAND_OVERLAP && Math.abs(p.z - progHere.tz - 0.5) < 0.8 - LAND_OVERLAP &&
            p.y >= progHere.ty - 1.001 && p.y < progHere.ty + 1 &&
            // A region landing must reach its chosen height;
            // a rail top and the adjacent floor differ by a block.
            (progHere.land === undefined || Math.abs(p.y - progHere.land.y) < 0.5) &&
            // A deep catch must reach the target height because the catch branch
            // has no wall to press into.
            (progHere.deep !== true || !progHere.catchTarget || p.y >= progHere.ty - 0.001) &&
            // The final landing must settle into the goal cell before the node is retired.
            (path.length > 1 || (Math.floor(p.x) === progHere.tx && Math.floor(p.z) === progHere.tz))
          if (!progLanded) break
        }
        // Finishing at jump apex releases forward and sprint too early and can shorten the landing.
        // Keep the final node active until the bot lands, including in water or on climbables.
        let airborneHold = false
        if (path.length === 1 && Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1) {
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
          if (ent.onGround !== true && ent.isInWater !== true) {
            const feet = bot.blockAt(p) as { type: number } | null
            airborneHold = feet === null || (!climbableIds.has(feet.type))
            // The arrival box also reaches below a ladder's top, so require the final climb
            // to reach goal height.
            if (!airborneHold && dy > CLIMB_FINISH_DY) airborneHold = true
          }
        }
        // Retiring a shelf landing too high steers the descent toward the gap beyond it.
        // Hold it until within PARKOUR_LAND_DY; holding longer sheds momentum for the next move.
        // Ground beyond the landing allows harmless overshoot, and climbable catches count as support.
        if ((nextPoint as { parkour?: boolean }).parkour === true &&
            p.y - nextPoint.y > PARKOUR_LAND_DY) {
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
          const feet = bot.blockAt(p) as BlockLike | null
          const caught = ent.onGround === true || ent.isInWater === true ||
            (feet !== null && (climbableIds.has(feet.type)))
          // A chained jump needs its landing node until touchdown
          // so the next jump fires on the landing tick.
          const stoneOfChain = path.length > 1 && (path[1] as Move).chain === true
          if (!caught && (stoneOfChain || holeBeyond(nextPoint, p))) break
        }
        if (airborneHold) break
        // Retiring a scripted jump mid-air drops its script and steers toward the following node.
        if (jumpScript !== null && jumpScriptNode === nextPoint &&
            (bot.entity as { onGround?: boolean }).onGround !== true) break
        // Corner cuts skip node centers, so retire skipped nodes once the body passes them along the cut.
        // Use the fixed cut direction: recovery can reverse velocity, and path legs can turn away from it.
        // Reaching the cut target also retires skipped nodes, even if the path doubles back.
        // The cut target and final goal still require arrival.
        let passed = false
        if (cutTarget !== null && cutTarget !== nextPoint && !swimming && path.length > 1 &&
            Math.abs(dy) < arriveDy &&
            (nextPoint as { parkour?: boolean }).parkour !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0) {
          // Use the anchored cut direction so displacement does not rotate the retirement plane.
          // Fall back to the direction from the body to the target when no anchor is known.
          const from = cutFrom ?? p
          const cx = cutTarget.x - from.x
          const cz = cutTarget.z - from.z
          passed = dx * cx + dz * cz <= 0 ||
            (Math.abs(cutTarget.x - p.x) <= 0.35 && Math.abs(cutTarget.z - p.z) <= 0.35 &&
             Math.abs(cutTarget.y - p.y) < arriveDy)
        }
        // Retire overshot landings only just after touchdown and along the incoming flight.
        // The next leg can point backward on switchbacks; momentum chains must re-jump from the stone.
        if (!passed && stateMovements.allowLandingRetire && !swimming && path.length > 1 &&
            caughtNow && landedTicksAgo <= 1 && prevRetired !== null && Math.abs(dy) < 1 &&
            (nextPoint as { parkour?: boolean }).parkour === true &&
            (path[1] as Move).chain !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0) {
          // Farther overshoots count only on the same walkable platform.
          const far = Math.hypot(dx, dz)
          const fx = nextPoint.x - prevRetired.x
          const fz = nextPoint.z - prevRetired.z
          passed = fx * fx + fz * fz > 1 && (p.x - nextPoint.x) * fx + (p.z - nextPoint.z) * fz > 0 &&
            (far <= LAND_PAST || (far <= LAND_PAST_WALKED && physics.walksTo?.(nextPoint) === true))
        }
        // A grounded body past a node on the next leg need not walk back into its arrival box.
        // Programs and chains still require their verified take-off positions.
        if (!passed && !swimming && path.length > 1 && Math.abs(dy) < arriveDy &&
            (bot.entity as { onGround?: boolean }).onGround === true &&
            (nextPoint as { parkour?: boolean }).parkour !== true &&
            ((path[1] as Move).program === null || (path[1] as Move).program === undefined) && (path[1] as Move).chain !== true &&
            ((path[1] as Move).via === null || (path[1] as Move).via === undefined) &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0 &&
            path[1].toBreak.length === 0 && path[1].toPlace.length === 0) {
          passed = onLegPast(p.x, p.z, nextPoint, path[1])
        }
        // Dip nodes crossed at exit height and climb nodes already below the body need no return visit.
        // Requiring their arrival boxes would send the body back down.
        if (!passed && !swimming && path.length > 1 && caughtNow && dy < 0 && p.y < path[1].y &&
            Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 &&
            (nextPoint as { parkour?: boolean }).parkour !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0 &&
            path[1].y > nextPoint.y && Math.floor(path[1].x) === Math.floor(nextPoint.x) && Math.floor(path[1].z) === Math.floor(nextPoint.z)) {
          const feet = bot.blockAt(p) as BlockLike | null
          passed = feet !== null && climbableIds.has(feet.type)
        }
        const overDip = !caughtNow && dy < 0 && -dy <= HOP_ARRIVE_DY && path.length > 1 &&
          (nextPoint as { parkour?: boolean }).parkour !== true &&
          path[1].y >= nextPoint.y + 1 && p.y >= path[1].y - 0.5
        // The final node requires the goal's cell: an off-centre arrival box can extend into its neighbour.
        const inCell = path.length > 1 || (Math.floor(p.x) === Math.floor(nextPoint.x) && Math.floor(p.z) === Math.floor(nextPoint.z))
        // Programs, chains and via moves require a grounded take-off;
        // retiring their start mid-air lets line-up controls interrupt the incoming flight.
        const stoodOn = path.length > 1 && (((path[1] as Move).program !== null && (path[1] as Move).program !== undefined) ||
          (path[1] as Move).chain === true || ((path[1] as Move).via !== null && (path[1] as Move).via !== undefined))
        if (stoodOn && !passed && !progLanded && !caughtNow) break
        // A jump from a ladder's top edge needs a standing arrival, not a hanging catch.
        // Planned programs can climb to their own take-off on the ladder.
        if (!passed && !progLanded && path.length > 1 && (path[1] as { parkour?: boolean }).parkour === true &&
            ((path[1] as Move).program === null || (path[1] as Move).program === undefined) &&
            (bot.entity as { onGround?: boolean }).onGround !== true) {
          const cx = Math.floor(nextPoint.x)
          const cy = Math.floor(nextPoint.y + 0.001)
          const cz = Math.floor(nextPoint.z)
          const sb = bot.blockAt(new Vec3(cx, cy - 1, cz), false) as { type: number } | null
          const own = bot.blockAt(new Vec3(cx, cy, cz), false) as { type: number } | null
          // Only stands above the ladder require grounding; catches within it can retire while hanging.
          if (sb !== null && climbableIds.has(sb.type) && !(own !== null && climbableIds.has(own.type))) break
        }
        if (!passed && !progLanded && !(Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && inCell && (Math.abs(dy) < arriveDy || overDip))) break

        lastNodeTime = performance.now()
        lastNodeArrival = lastNodeTime
        prevRetired = { x: nextPoint.x, y: nextPoint.y, z: nextPoint.z, cell: (nextPoint as Move).cell }
        // Stitch only a leg streamed by the current solve; an older solve's landing
        // can constrain a new path that does not contain its own start cell.
        if (pathPartial && (nextPoint as Move & { streamGen?: number }).streamGen === solveGeneration) streamedLanding = (nextPoint as Move).cell
        path.shift()
        // Retiring the cut target invalidates its chord for subsequent node retirement.
        if (cutTarget === nextPoint) cutTarget = null
        if (path.length === 0) { 
          if (execTraceFile !== null) takeoffNote = `pathend${(bot.entity as { onGround?: boolean }).onGround === true ? '' : ':air'}`
          if (!dynamicGoal && stateGoal && (stateGoal.isEnd(p.floored()) || stateGoal.isEnd(p.floored().offset(0, 1, 0)))) {
            // Cancel the pending soft replan so its late result cannot install a path after the goal clears.
            cancelActiveSolve()
            const reached = stateGoal
            stateGoal = null
            bot.emit('goal_reached' as never, reached as never)
            // A listener set the next goal, and its setGoal already reset the path.
            if (stateGoal !== null) return
          }
          // Coast at a partial path's end only if the body stays on the support
          // at the next path's take-off level while its solve continues.
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean, isInLava?: boolean }
          // Near hazards, the stop's snap keeps the body in its cell.
          if (PARTIAL_END_COAST && stateGoal !== null && pathPartial && solveInFlight() && physics.holdsOn !== undefined &&
              ent.onGround === true && ent.isInWater !== true && ent.isInLava !== true && !hazardAround()) {
            hopHold = false
            kernelHop = false
            cutTarget = null
            clearWedge()
            bot.clearControlStates()
            if (physics.holdsOn(null, undefined, 1, PARTIAL_COAST_DROP)) return
          }
          fullStop()
          return
        }
        nextPoint = path[0]
        if (nextPoint.toBreak.length > 0 || nextPoint.toPlace.length > 0) {
          fullStop()
          return
        }
        dx = nextPoint.x - p.x
        dy = nextPoint.y - p.y
        dz = nextPoint.z - p.z
      }


      // Wait for server velocity loss after a damaging landing before choosing a take-off.
      // A lone onGround flicker without ascent or descent is not flight; forward input can walk off a narrow top.
      let airborne = false
      {
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        if (!grounded && !swimming) {
          airTicks++
          airborne = airTicks >= AIRBORNE_TICKS || bot.entity.velocity.y > 0.05 || p.y < prevTickY - 0.02
        } else {
          airTicks = 0
        }
        // Accumulate descent through the landing tick until ground, water or a ladder ends the fall.
        const drop = prevTickY - p.y
        // Teleport-sized motion clears fall and landing-settle state.
        // Skipped monitoring ticks can contain real movement; only a server position overrides that gap.
        const forced = forcedAtTick > prevTickAt
        const stale = physTicks - prevTickAt > 1 && !forced
        const teleported = prevTickPos !== null && !stale && Math.hypot(p.x - prevTickPos.x, p.z - prevTickPos.z) > MAX_TICK_DROP
        if (teleported) landSettle = 0
        prevTickY = p.y
        prevTickPos = p.clone()
        prevTickAt = physTicks
        if (spared > 0) spared--
        // Water and climbables end the fall; descent across skipped monitoring ticks still counts.
        if ((caughtNow && !grounded) || (drop > MAX_TICK_DROP && !stale) || teleported) fallen = 0
        else if (drop > 0 && (!grounded || fallen > 0 || stale)) fallen += drop
        if (grounded) {
          if (fallen > SAFE_FALL) {
            const under = simKindOf((bot.blockAt(p.offset(0, -0.2, 0)) as { name?: string } | null)?.name ?? '', false)
            const sneaking = (bot.controlState as unknown as Record<string, boolean>).sneak
            const last = Math.max(0, drop)
            if (landingHurts(fallen, last, under, sneaking, wholeFall)) {
              const ping = (bot as unknown as { player?: { ping?: number } }).player?.ping ?? 0
              landSettle = HURT_WAIT_TICKS + Math.min(4, Math.ceil(ping / 50))
              landSettleAt = performance.now()
              hurtSeen = false
            } else if (wholeFall && landingHurts(fallen, last, under, sneaking, false)) {
              spared = HURT_WAIT_TICKS
            }
          }
          fallen = 0
        }
        if (hurtSeen) landSettle = 0
        // Expire old landing waits so time spent solving or between goals cannot delay a new path.
        if (landSettle > 0 && performance.now() - landSettleAt > (HURT_WAIT_TICKS + 8) * 50) landSettle = 0
        // A replay includes its touchdowns; inserting a settle tick would desynchronise every later phase.
        if (landSettle > 0 && grounded && progNode === nextPoint && progPhase === 1 &&
            (nextPoint as Move).program !== null && (nextPoint as Move).program !== undefined) {
          landSettle = 0
        }
        if (landSettle > 0 && grounded) {
          landSettle--
          execBranch = 'settle'
          cutTarget = null
          hopHold = false
          bot.setControlState('forward', false)
          bot.setControlState('sprint', false)
          bot.setControlState('jump', false)
          bot.setControlState('sneak', true)
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
          futile(swimming)
          return
        }
      }

      // After aborting a deep bounce, steer into the slime pool without sneak
      // so the rebound prevents damage, then replan once the body rests.
      if (basin !== null) {
        const v = bot.entity.velocity
        basin.still = Math.abs(v.y) < 0.1 && Math.hypot(v.x, v.z) < 0.05 ? basin.still + 1 : 0
        if (basin.still >= BASIN_REST_TICKS || performance.now() - basin.since > BASIN_MAX_MS) {
          execBranch = 'basin-rest'
          resetPath('basin', true)
          return
        }
        steerBasin(p)
        execBranch = 'basin'
        lastNodeTime = performance.now()
        return
      }

      // A live take-off program is valid only from the current body state
      // and must replay from the tick the kernel finds it.
      const kernelTakeoff = TAKEOFF_KERNEL && physics.hopFrom !== undefined && cachedLut !== null
      // Count lost take-offs by edge across replans; arriving again resets the futility clock.
      // Ban the landing on the second loss so replanning cannot repeat the same edge indefinitely.
      const lostReplan = (branch: string): boolean => {
        const from = prevRetired?.cell
        const to = (nextPoint as Move).cell
        const key = `${from !== undefined ? `${from[0]},${from[1]},${from[2]}` : `${Math.floor(p.x)},${Math.floor(p.y + 0.001)},${Math.floor(p.z)}`}>${to[0]},${to[1]},${to[2]}`
        if (lostEdges.size > 512) lostEdges.clear()
        const n = (lostEdges.get(key) ?? 0) + 1
        lostEdges.set(key, n)
        if (n >= 2) banTakeoff(nextPoint)
        return replanLost(p, branch)
      }
      // Search the chain stone's line-up positions before replanning a lost take-off.
      // Budget exhaustion is not an exhaustive refusal, so other recovery attempts remain available.
      const chainLost = (): boolean => {
        if (linedSearched === nextPoint || physics.hopLinedUp === undefined) return lostReplan('chain-lost')
        linedSearched = nextPoint
        const lined = physics.hopLinedUp((nextPoint as Move).cell, TAKEOFF_LINED_TICKS)
        takeoffNote += ' | ' + (physics.hopNote ?? '')
        if (lined !== null) {
          ;(nextPoint as Move).program = lined
          linedInstalled = nextPoint
          progNode = null
          jumpStillTicks = 0
          lastNodeTime = performance.now()
          bot.setControlState('forward', false)
          bot.setControlState('back', false)
          bot.setControlState('jump', false)
          bot.setControlState('sprint', false)
          bot.setControlState('sneak', true)
          execBranch = 'takeoff-lined'
          futile(swimming)
          return true
        }
        return physics.hopExhausted === true && lostReplan('chain-lost')
      }
      const replanLost = (at: Vec3, branch: string, keepKeys = false): boolean => {
        const again = progLostAt !== null && Math.abs(at.x - progLostAt.x) < PROG_LIVE_NEAR &&
          Math.abs(at.z - progLostAt.z) < PROG_LIVE_NEAR && Math.abs(at.y - progLostAt.y) < 0.5
        progLostCount = again ? progLostCount + 1 : 1
        progLostAt = { x: at.x, y: at.y, z: at.z }
        if (progLostCount > PROG_LOST_TRIES) return false
        ;(nextPoint as Move).program = null
        progNode = null
        execBranch = branch
        resetPath('stuck', !keepKeys)
        return true
      }
      const takeoffLive = (prog: HopProgram | null): boolean => {
        if (prog === null) return false
        ;(nextPoint as Move).program = prog
        progNode = nextPoint as Move
        progPhase = 1
        progTick = 0
        progFired = false
        progAir = 0
        progRebounds = 0
        progSettle = 0
        jumpStillTicks = 0
        lastNodeTime = performance.now()
        cutTarget = null
        hopHold = false
        kernelHop = false
        replayProgramTick(prog, p)
        execBranch = 'prog'
        futile(swimming)
        return true
      }
      // Allow one retreat from the rest the live search refuses.
      // Pass onClimb explicitly because this can run before climbing is initialised.
      const tryPrefix = (hint: HopProgram | undefined, onClimb: boolean, maxTicks = Infinity): boolean => {
        const n = nextPoint as Move
        const v = bot.entity.velocity
        if (!LIVE_PREFIX || !kernelTakeoff || physics.hopPrefix === undefined || prefixSearched.has(n) ||
            (bot.entity as { onGround?: boolean }).onGround !== true || Math.hypot(v.x, v.z) >= 0.01 ||
            onClimb || swimming || n.chain === true || hint?.seq !== undefined || hint?.legs === true ||
            hint?.climb === true || hint?.deep === true || hint?.bounces !== undefined) return false
        if (PREFIX_RETRY_CAP && prefixGoalCount >= PREFIX_GOAL_MAX) return false
        prefixGoalCount++
        prefixSearched.add(n)
        if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
        let live = physics.hopPrefix(n.cell, hint?.land, hint)
        takeoffNote += (takeoffNote !== '' ? ' | ' : '') + (physics.hopNote ?? '')
        // A live program must be no slower than the plan including its line-up.
        if (live !== null && live.ticks > maxTicks) { takeoffNote += ' | slower'; live = null }
        if (live !== null && hint !== undefined) prefixPlan.set(n, hint)
        return takeoffLive(live)
      }
      const installSkip = (sk: { program: HopProgram, cut: number }): void => {
        const removed = path.splice(0, sk.cut)
        nextPoint = path[0]
        skip = { target: nextPoint, removed, planned: nextPoint.program }
        lastNodeTime = performance.now()
        kernelLed = false
        takeoffLive(sk.program)
        execBranch = 'skip'
      }
      // A skip installs a live flight on its destination node, bypassing the planned standing start.
      const trySkip = (drive: GaitDrive, gp: ReadonlyArray<unknown>): boolean => {
        if (!SKIP_AHEAD || !kernelTakeoff || physics.skipAhead === undefined || gp !== path || pathPartial ||
            stateMovements.exclusionAreasStep.length > 0 || drive.cut === undefined || drive.cut < 1 || drive.cut >= path.length) return false
        const m = path[drive.cut]
        if (skipFailed.has(m)) return false
        const tries = skipTries.get(m) ?? 0
        if (tries >= SKIP_TRIES) return false
        const v = bot.entity.velocity
        const atRest = Math.hypot(v.x, v.z) < 0.01
        const freshRest = atRest && !(Math.abs(p.x - skipRestX) <= 0.005 && Math.abs(p.z - skipRestZ) <= 0.005)
        if (landedTicksAgo > 1 && !freshRest) return false
        if (atRest) { skipRestX = p.x; skipRestZ = p.z }
        if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
        const sk = physics.skipAhead(path, drive)
        if (physics.skipSearched === true) skipTries.set(m, tries + 1)
        if (physics.skipSearched === true) takeoffNote = physics.hopNote ?? ''
        if (sk === null) return false
        installSkip(sk)
        return true
      }
      // Judge at most one upcoming neo per tick, before the body reaches it.
      // A refused edge triggers a replan while walking continues.
      const neoAhead = (): boolean => {
        if (!NEO_AHEAD || !kernelTakeoff || physics.hopFromCell === undefined || pathPartial || stateMovements.exclusionAreasStep.length > 0) return false
        for (let k = 1; k < Math.min(path.length, NEO_AHEAD_NODES + 1); k++) {
          const n = path[k]
          if (n.parkour !== true || n.wOff === 0 || n.program !== null && n.program !== undefined) continue
          const from = path[k - 1]
          const key = `${from.cell[0]},${from.cell[1]},${from.cell[2]}>${n.cell[0]},${n.cell[1]},${n.cell[2]}`
          // Leave an already-refused edge to arrival handling if a replan returns it.
          if (neoChecked.has(key)) continue
          if (neoChecked.size > 512) neoChecked.clear()
          if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
          const found = physics.hopFromCell(from.cell, n.cell, TAKEOFF_LINED_TICKS)
          if (execTraceFile !== null) takeoffNote += `${takeoffNote !== '' ? ' | ' : ''}ahead ${physics.hopNote ?? ''}`
          neoChecked.set(key, found !== null || physics.hopExhausted !== true)
          if (found === null && physics.hopExhausted === true) {
            banTakeoff(n)
            softReplan('neo-refused')
            return true
          }
          return false
        }
        return false
      }
      // Skip the walk aside when a direct flight from touchdown or a fresh rest reaches the next jump.
      const trySkipWalk = (gp: ReadonlyArray<unknown>): boolean => {
        if (!SKIP_WALK || !kernelTakeoff || physics.skipWalk === undefined || gp !== path || pathPartial ||
            stateMovements.exclusionAreasStep.length > 0 || path.length < 2) return false
        const v = bot.entity.velocity
        const atRest = Math.hypot(v.x, v.z) < 0.01
        const freshRest = atRest && !(Math.abs(p.x - walkRestX) <= 0.005 && Math.abs(p.z - walkRestZ) <= 0.005)
        if (landedTicksAgo > 1 && !freshRest) return false
        if (atRest) { walkRestX = p.x; walkRestZ = p.z }
        if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
        const asked: { m: Move | null } = { m: null }
        const sk = physics.skipWalk(path, m => {
          asked.m = m as Move
          return !skipFailed.has(m as Move) && (skipTries.get(m as Move) ?? 0) < SKIP_TRIES
        })
        if (physics.skipSearched === true && asked.m !== null) {
          skipTries.set(asked.m, (skipTries.get(asked.m) ?? 0) + 1)
          takeoffNote = physics.hopNote ?? ''
        }
        if (sk === null) return false
        installSkip(sk)
        return true
      }
      // After finding a neo flight, check whether a direct flight to the following node is quicker.
      const trySkipNeo = (t0: number, rest: boolean): boolean => {
        if (!SKIP_NEO || physics.skipNeo === undefined || pathPartial || stateMovements.exclusionAreasStep.length > 0 || path.length < 2) return false
        const m = path[1]
        if (skipFailed.has(m)) return false
        const tries = skipTries.get(m) ?? 0
        if (tries >= SKIP_TRIES) return false
        const sk = physics.skipNeo(path, t0, rest)
        if (physics.skipSearched === true) {
          skipTries.set(m, tries + 1)
          takeoffNote += ' | ' + (physics.hopNote ?? '')
        }
        if (sk === null) return false
        installSkip(sk)
        return true
      }
      // Replay programs with the oracle's run, air-turn and rebound counters.
      // Try planned flights from the live state before falling back to a line-up.
      // Precision beam controls require live re-derivation; ordinary jump gates cannot substitute for the replay.
      let prog = (nextPoint as Move).program
      // A live program is valid only from the state where it is found; a lost replay is spent.
      if (prog !== null && prog !== undefined && prog.start !== undefined && progNode !== nextPoint) {
        (nextPoint as Move).program = null
        prog = null
      }
      if (prog !== null && prog !== undefined && !swimming) {
        if (progNode !== nextPoint) {
          progNode = nextPoint
          progPhase = 0
          progTick = 0
          progFired = false
          progAir = 0
          progRebounds = 0
          progLiveTries = 0
          progRestX = Infinity
          progRestZ = Infinity
          progPress = null
          progPresses = 0
          progMovingTries = 0
          progSettle = 0
        }
        cutTarget = null
        hopHold = false
        kernelHop = false
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        let spent = false
        // Climb to a ladder take-off while pressing into its wall.
        // Hold a hanging start under sneak and search from that held state.
        if (progPhase === 0 && !grounded && !swimming) {
          const feetB = bot.blockAt(p) as BlockLike | null
          if (feetB !== null && (climbableIds.has(feetB.type))) {
            const wall = climbWall(feetB, p)
            if (p.y < prog.sy - PROG_CLIMB_TOL && wall !== null) {
              bot.look(Math.atan2(-wall[0], -wall[1]), 0, true)
              bot.setControlState('forward', true)
              bot.setControlState('back', false)
              bot.setControlState('left', false)
              bot.setControlState('right', false)
              // Jump climbs without wall contact; sneak prevents the initial slide.
              bot.setControlState('jump', CLIMB_HOLD_ENABLED)
              climbKeysHeld = true
              bot.setControlState('sprint', false)
              bot.setControlState('sneak', CLIMB_HOLD_ENABLED && bot.entity.velocity.y < 0)
              execBranch = 'prog-climb'
              futile(swimming)
              return
            }
            if (prog.climb === true) {
              const v = bot.entity.velocity
              const held = Math.hypot(v.x, v.z) < 0.003 && v.y <= 0
              if (kernelTakeoff && progLiveTries < PROG_LIVE_TRIES && (progLiveTries === 0 || held)) {
                progLiveTries++
                if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
                let live = held ? (physics.hopAnchored?.((nextPoint as Move).cell, prog) ?? null) : null
                takeoffNote = held ? (physics.hopNote ?? '') : ''
                if (live === null) {
                  live = physics.hopFrom?.((nextPoint as Move).cell, TAKEOFF_SEARCH_TICKS, prog.land, nextLegCell()) ?? null
                  takeoffNote += (takeoffNote !== '' ? ' | ' : '') + (physics.hopNote ?? '')
                }
                if (takeoffLive(live)) return
              }
              if (progLiveTries >= PROG_LIVE_TRIES && held) {
                spent = true 
              } else {
                bot.setControlState('forward', false)
                bot.setControlState('back', false)
                bot.setControlState('left', false)
                bot.setControlState('right', false)
                bot.setControlState('jump', false)
                bot.setControlState('sprint', false)
                bot.setControlState('sneak', true)
                execBranch = 'prog-hold'
                futile(swimming)
                return
              }
            }
          }
        }
        if (spent) {
        } else if (progPhase === 0) {
          const ex = prog.sx - p.x
          const ez = prog.sz - p.z
          const d = Math.hypot(ex, ez)
          const v = bot.entity.velocity
          const atRest = grounded && Math.hypot(v.x, v.z) < 0.003
          // Retry live programs at each new rest; line-up presses can stop just outside the tolerance.
          // Precision beam programs require live re-derivation even at the lined-up rest.
          const beamPlan = prog.seq !== undefined
          // Precision plans, beam chains and pivots require the exact line-up.
          const precise = beamPlan || prog.precise === true
          const lineTol = precise && kernelTakeoff ? PROG_EXACT_TOL : PROG_LINEUP_TOL
          const release = (): void => {
            bot.setControlState('forward', false)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            execBranch = 'prog-press'
          }
          // Apply controls before updating futility, since a reset clears them.
          const letGo = (): void => { release(); futile(swimming) }
          const pressAt = (yaw: number): void => {
            release()
            bot.look(yaw, 0, true)
            bot.setControlState('forward', true)
            futile(swimming)
          }
          const pressNext = (): boolean => {
            if (progPress === null) return false
            if (progPress.delay > 0) {
              progPress.delay--
              letGo()
              return true
            }
            if (progPress.i >= progPress.yaws.length) return false
            pressAt(progPress.yaws[progPress.i++])
            return true
          }
          // Re-anchor a precision plan at rest before using exact line-up presses.
          // Beam chains and pivots cannot be replaced by the ordinary live search.
          const exact = (): boolean => {
            progRestX = p.x
            progRestZ = p.z
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            const live = physics.hopAnchored?.((nextPoint as Move).cell, prog as HopProgram) ?? null
            takeoffNote = physics.hopNote ?? ''
            if (takeoffLive(live)) return true
            if (progPresses >= PROG_PRESS_TRIES) return false
            progPresses++
            const lu = physics.hopLineUp?.(prog as HopProgram, LINEUP_TICKS) ?? null
            takeoffNote += ` | ${physics.hopNote ?? ''}`
            if (lu === null) return false
            progPress = { yaws: lu.yaws, i: 0, delay: lu.delay ?? 0 }
            if (!pressNext()) letGo()
            return true
          }
          // Check whether this rest is new before exact() marks it tried,
          // so failed anchoring still permits live re-derivation.
          const freshRest = atRest && Math.hypot(p.x - progRestX, p.z - progRestZ) > 0.005
          if (progPress !== null) {
            // Release each press into a sneaking coast to reach the kernel's rest state.
            if (pressNext()) return
            if (grounded && !atRest) {
              letGo()
              return
            }
            progPress = null
            if (atRest && exact()) return
          } else if (precise && kernelTakeoff && freshRest && d <= PROG_EXACT_REACH && exact()) {
            return
          }
          const newRest = freshRest && d <= PROG_LIVE_NEAR
          // Reuse the neo search's verdict from this same rest rather than searching it again.
          if (progLiveTries === 0 && (linedInstalled === nextPoint || (restSearchedAt !== null && restSearchedAt.node === nextPoint &&
              Math.hypot(p.x - restSearchedAt.x, p.z - restSearchedAt.z) < 0.005))) {
            progLiveTries = 1
            progRestX = p.x
            progRestZ = p.z
          }
          // Carry chains can exceed the live beam's length and span multiple flights,
          // so a live search cannot re-derive them.
          const futileChain = prog.seq !== undefined && prog.ticks > BEAM_DEPTH && Math.hypot(prog.tx + 0.5 - prog.sx, prog.tz + 0.5 - prog.sz) > ONE_FLIGHT_REACH
          if (kernelTakeoff && grounded && !futileChain && progLiveTries < PROG_LIVE_TRIES && (progLiveTries === 0 || newRest)) {
            progLiveTries++
            if (atRest) { progRestX = p.x; progRestZ = p.z }
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            // Try re-anchoring the plan at this exact state before searching for a new flight.
            let live = newRest ? (physics.hopAnchored?.((nextPoint as Move).cell, prog) ?? null) : null
            takeoffNote = newRest ? (physics.hopNote ?? '') : ''
            if (live === null) {
              // Keep the program's landing place: choosing another nearby place can
              // leave the next take-off behind a pane rail.
              // Give kick and pivot plans their own family search and budget first.
              const seed = SEED_FAMILY ? seedFamilies(prog) : undefined
              live = physics.hopFrom?.((nextPoint as Move).cell, beamPlan && newRest ? TAKEOFF_REST_BEAM_TICKS : TAKEOFF_SEARCH_TICKS, prog.land, nextLegCell(), true, seed, true, prog) ?? null
              takeoffNote += (takeoffNote !== '' ? ' | ' : '') + (physics.hopNote ?? '')
              // Prefer line-up when the live flight is slower than the plan including its approach.
              if (live !== null && PROG_LIVE_PLAN && !precise && d > PROG_LINEUP_TOL &&
                  live.ticks > prog.ticks + Math.ceil(d / PROG_LINEUP_SPEED) + PROG_LINEUP_SETTLE + PROG_LIVE_SLACK) {
                takeoffNote += ` | plan ${prog.ticks}t+lineup`
                live = null
              }
            }
            if (takeoffLive(live)) return
            // A retreat must meet the same time bound as a live search, including the plan's line-up.
            if (atRest && tryPrefix(prog, false, PROG_LIVE_PLAN && !precise && d > PROG_LINEUP_TOL ? prog.ticks + Math.ceil(d / PROG_LINEUP_SPEED) + PROG_LINEUP_SETTLE + PROG_LIVE_SLACK : Infinity)) return
          }
          // A moving anchor must be no slower than the plan including line-up.
          if (PROG_MOVE_ANCHOR && kernelTakeoff && grounded && !atRest && !futileChain && prog.climb !== true && prog.deep !== true && d <= PROG_FAR_START) {
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            const live = physics.hopAnchored?.((nextPoint as Move).cell, prog) ?? null
            const fits = live !== null && live.ticks <= prog.ticks + Math.ceil(d / PROG_LINEUP_SPEED) + PROG_LINEUP_SETTLE
            if (live !== null && execTraceFile !== null) takeoffNote += `${takeoffNote !== '' ? ' | ' : ''}moving ${physics.hopNote ?? ''}${fits ? '' : ' slower'}`
            if (fits && takeoffLive(live)) return
          }
          // Check a carry chain only after a verified glide; never launch it from a moving state.
          // A failed check does not consume a rest attempt or its presses.
          if (MOVING_LINEUP && precise && kernelTakeoff && grounded && !atRest && progMovingTries === 0 &&
              prog.climb !== true && prog.deep !== true) {
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            const moving = { reach: PROG_EXACT_REACH, tried: false }
            const lu = physics.hopLineUp?.(prog, LINEUP_TICKS, moving) ?? null
            if (moving.tried) progMovingTries++
            if (lu !== null && moving.tried) {
              takeoffNote = physics.hopNote ?? ''
              progPress = { yaws: lu.yaws, i: 0, delay: lu.delay ?? 0 }
              if (!pressNext()) letGo()
              return
            }
          }
          if (precise && atRest && d <= lineTol && (!kernelTakeoff || Math.hypot(p.x - progRestX, p.z - progRestZ) <= 0.005)) {
            spent = true 
          } else if (!precise && d <= PROG_LINEUP_TOL && atRest) {
            progPhase = 1
            progTick = 0
            lastNodeTime = performance.now()
          } else if (kernelTakeoff && grounded && d > PROG_FAR_START && physics.walksTo?.({ x: prog.sx, y: prog.sy, z: prog.sz }) === false) {
            // Never line up across an unwalkable gap; replan from the body's current position.
            // After retries run out, hold the edge guard until futility bans the take-off.
            if (replanLost(p, 'prog-far')) return
            release()
            execBranch = 'prog-far'
            futile(swimming)
            return
          } else {
            // Press toward the start only when the predicted coasting rest misses tolerance
            // and one press improves it; pressing on a nearer miss causes repeated overshoots.
            if (d > 1e-3) bot.look(Math.atan2(-ex, -ez), 0, true)
            // Keep sneak throughout an approach the kernel cannot walk at the start's level;
            // the edge guard stops the body at any intervening gap.
            const sneakZone = d < PROG_SNEAK_ZONE || (kernelTakeoff && Math.abs(prog.sy - p.y) <= 0.6 &&
              physics.walksTo?.({ x: prog.sx, y: prog.sy, z: prog.sz }) === false)
            let press = !sneakZone
            if (sneakZone && d > 1e-3) {
              const rx = p.x + v.x * PROG_COAST - prog.sx
              const rz = p.z + v.z * PROG_COAST - prog.sz
              const miss = Math.hypot(rx, rz)
              press = miss > lineTol &&
                Math.hypot(rx + (ex / d) * PROG_PRESS_SHIFT, rz + (ez / d) * PROG_PRESS_SHIFT) < miss
            }
            bot.setControlState('forward', press)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', sneakZone)
            execBranch = 'prog-lineup'
            // Brake landing momentum so line-up controls and their coast stay supported.
            // Release sneak for full braking on thin panels, whose edges the cell-based guard cannot protect.
            if (PROG_LINEUP_BRAKE && grounded && physics.holdsOn !== undefined && Math.abs(prog.sy - p.y) <= 0.6 &&
                Math.hypot(v.x, v.z) >= HOLD_STILL && !physics.holdsOn(null)) {
              const yaw0 = bot.entity.yaw
              bot.look(Math.atan2(-v.x, -v.z), 0, true)
              if (physics.holdsOn({ back: true, sneak: false }, undefined, PROG_BRAKE_TICKS)) {
                bot.setControlState('forward', false)
                bot.setControlState('back', true)
                bot.setControlState('sneak', false)
                // Release the brake next tick even if a reset retains other controls.
                holdKeyed = true
                execBranch = 'prog-brake'
              } else {
                bot.look(yaw0, 0, true)
                if (holdSupport()) execBranch = 'hold'
              }
            }
            futile(swimming)
            return
          }
        } else {
          // A final landing over the cell's edge settles toward its centre under sneak.
          // A rebound keeps its flight controls instead.
          // Do not centre an under landing beside a thin block; the centre is inside the block.
          if (progFired && grounded && path.length === 1 && bot.entity.velocity.y <= 0.05 &&
              (prog.land === undefined || prog.land.y >= prog.ty - 0.5) &&
              // Settle a region landing to its place's level before retiring it.
              (!REGION_SETTLE || prog.seq !== undefined || ((progSettle === 0 || progSettle >= STAY_TICKS) && (prog.land === undefined || Math.abs(p.y - prog.land.y) < 0.5))) &&
              Math.abs(p.x - prog.tx - 0.5) < 0.8 - LAND_OVERLAP && Math.abs(p.z - prog.tz - 0.5) < 0.8 - LAND_OVERLAP &&
              p.y >= prog.ty - 1.001 && p.y < prog.ty + 1) {
            bot.look(Math.atan2(-(prog.tx + 0.5 - p.x), -(prog.tz + 0.5 - p.z)), 0, true)
            bot.setControlState('forward', true)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            execBranch = 'prog-settle'
            futile(swimming)
            return
          }
          // Replay counters observe the state after the last tick: the firing tick precedes air ticks.
          // An off-node touchdown or blocked jump no longer follows the verified flight.
          if (!progFired) {
            const vy = bot.entity.velocity.y
            // Leaving a climbable counts as take-off.
            const feetB = prog.climb === true && !grounded ? bot.blockAt(p) as BlockLike | null : null
            const onClimb = feetB !== null && (climbableIds.has(feetB.type))
            if (vy > 0.3 || (!grounded && vy < -0.3) || (prog.climb === true && !grounded && !onClimb) ||
                (progTick > 0 && !onClimb && jumpRearmedSince(progDrivenAt, progJumpTicks))) progFired = true
            else if (prog.seq === undefined && progTick > (prog.jumpOnLeave === true ? prog.ticks : prog.jumpAt) + PROG_JUMP_GRACE && !onClimb) spent = true
            // A leave program needs the kernel to identify the latched take-off tick.
            else if (prog.jumpOnLeave === true && (physics.latchedNow?.() ?? null) === null) spent = true
          } else if (prog.seq === undefined) {
            progAir++
            // Continue a region settle under sneak without movement keys, including
            // brief air time after a stair-top touchdown, until arrival or the settle limit.
            if (REGION_SETTLE && progSettle > 0) {
              // Spend the program only on the ground; doing so airborne would hand it to the gates.
              if (progSettle >= STAY_TICKS && grounded) spent = true
              else {
                if (progSettle < STAY_TICKS) progSettle++
                bot.setControlState('forward', false)
                bot.setControlState('back', false)
                bot.setControlState('left', false)
                bot.setControlState('right', false)
                bot.setControlState('jump', false)
                bot.setControlState('sprint', false)
                bot.setControlState('sneak', true)
                execBranch = 'prog-region-settle'
                futile(swimming)
                return
              }
            } else if (grounded) {
              const under = bot.blockAt(p.offset(0, -0.2, 0)) as { name?: string } | null
              if (bot.entity.velocity.y > 0.05 && under !== null && under.name === 'slime_block') progRebounds++
              // A region touchdown on a stair or rail settles to its verified place
              // under sneak without movement keys before the arrival check retires it.
              // Deep programs continue their verified flight instead of settling.
              else if (REGION_SETTLE && prog.land !== undefined && prog.deep !== true && progSettle < STAY_TICKS && Math.abs(p.y - prog.land.y) >= 0.5 &&
                  p.y >= prog.ty - 1.001 && p.y < prog.ty + 1 &&
                  Math.abs(p.x - prog.tx - 0.5) < 0.8 - LAND_OVERLAP && Math.abs(p.z - prog.tz - 0.5) < 0.8 - LAND_OVERLAP) {
                progSettle++
                bot.setControlState('forward', false)
                bot.setControlState('back', false)
                bot.setControlState('left', false)
                bot.setControlState('right', false)
                bot.setControlState('jump', false)
                bot.setControlState('sprint', false)
                bot.setControlState('sneak', true)
                execBranch = 'prog-region-settle'
                futile(swimming)
                return
              }
              else spent = true
            }
          }
          // A beam program can continue across touchdowns; spend it only when controls
          // run out short of the landing or it passes the verified landing.
          if (prog.seq !== undefined && progTick > Math.max(prog.seq.length >> 1, prog.ticks) + PROG_JUMP_GRACE) spent = true
          // Cap replay duration so an off-course flight cannot refresh the futility clock indefinitely.
          if (FLIGHT_CAP && progTick > FLIGHT_TICKS_MAX) spent = true
        }
        // Recheck a lethal-fall bounce from the current state every tick.
        // Refuse an invalid take-off; after launch, abort only while the slime pool remains reachable.
        if (!spent && progPhase === 1 && prog.deep === true && prog.seq === undefined) {
          if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
          const verdict = physics.deepVerdict?.(prog, progTick, progFired, progAir, progRebounds) ?? null
          // A walk-off can fire several ticks after leaving support; airborne already counts as committed.
          if (!progFired && grounded) {
            if (verdict === null || !verdict.arrives) spent = true
          } else if (verdict !== null && !verdict.arrives && verdict.abortable && prog.basin !== undefined) {
            basin = { tops: prog.basin, still: 0, since: performance.now() }
            ;(nextPoint as Move).program = null
            progNode = null
            progPhase = 0
            takeoffNote = 'deep-abort'
            steerBasin(p)
            execBranch = 'basin'
            lastNodeTime = performance.now()
            return
          }
        }
        if (spent && skip !== null && skip.target === nextPoint) {
          const sk = skip
          skip = null
          skipFailed.add(sk.target)
          if (!progFired && grounded) {
            // Undo a skip that never takes off and brake under the edge guard;
            // the restored path can start on a post.
            sk.target.program = sk.planned ?? null
            path.unshift(...sk.removed)
            progNode = null
            progPhase = 0
            // Allow the neo take-off to be searched again after undoing the skip.
            takeoffSearched = null
            takeoffSearchedAtRest = false
            bot.setControlState('forward', false)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            holdSupport()
            execBranch = 'skip-undo'
            lastNodeTime = performance.now()
            return
          }
        }
        if (spent) {
          (nextPoint as Move).program = null
          progNode = null
          // A lost region program needs a replan: ordinary gates would steer into its pane cell.
          // Wait until grounded, since an exhausted beam can still be airborne.
          if (prog.deep === true && grounded) {
            // Brake an aborted deep run-up at the lip and retain sneak through the replan;
            // clearing every key would let the body coast off.
            bot.setControlState('forward', false)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            if (replanLost(p, 'prog-lost', true)) return
          } else if (prog.land !== undefined && grounded && replanLost(p, 'prog-lost')) return
        } else {
          replayProgramTick(prog, p)
          execBranch = 'prog'
          // Replay uses its own tick and loss limits; the futility timer must not reset a valid flight mid-air.
          if (progFired || prog.seq !== undefined) lastNodeTime = performance.now()
          futile(swimming)
          return
        }
      }

      // A slime bounce is one planned edge: aim at the slime until upward rebound, then at the node.
      // Walk to keep the drop on one block and release sneak so the bounce can occur.
      // A momentum chain re-jumps on the landing tick to preserve speed.
      // Ordinary gates model a jump from rest, so settle onto the stone first if still airborne.
      const chainVia = (nextPoint as Move).via
      if ((nextPoint as Move).chain && chainVia !== null) {
        if (chainNode !== nextPoint) {
          chainNode = nextPoint
          chainFired = false
          chainDrivenAt = -1
        }
        if (!chainFired) {
          cutTarget = null
          hopHold = false
          const ent = bot.entity as { onGround?: boolean }
          const onStone = Math.floor(p.x) === chainVia.x && Math.floor(p.z) === chainVia.z && p.y < chainVia.y + 1
          // Choose the kernel flight whose touchdown gives the quickest next leg
          // when the chain continues from the landing stone.
          if (CHAIN_KERNEL && kernelTakeoff && ent.onGround === true && !swimming && takeoffSearched !== nextPoint) {
            const v = bot.entity.velocity
            const restNow = Math.hypot(v.x, v.z) < 0.01
            takeoffSearched = nextPoint
            takeoffSearchedAtRest = restNow
            if (restNow) restSearchedAt = { node: nextPoint, x: p.x, z: p.z }
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            const n1 = path[1] as Move | undefined
            let live: HopProgram | null = null
            takeoffNote = ''
            if (n1 !== undefined && n1.chain === true && physics.hopChain !== undefined) {
              live = physics.hopChain((nextPoint as Move).cell, n1.cell, CHAIN_PICK_BUDGET, (path[2] as Move | undefined)?.chain === true)
              takeoffNote = physics.hopNote ?? ''
            }
            if (live === null) {
              live = physics.hopFrom?.((nextPoint as Move).cell, TAKEOFF_SEARCH_TICKS, undefined, nextLegCell()) ?? null
              takeoffNote += (takeoffNote !== '' ? ' | ' : '') + (physics.hopNote ?? '')
            }
            // A kernel chain flight does not settle back onto the stone if it misses the node.
            if (takeoffLive(live)) { chainFired = true; return }
            if (restNow && CHAIN_LOST && chainLost()) return
          }
          if ((bot.entity.velocity.y > 0.3 && ent.onGround !== true) || jumpRearmedSince(chainDrivenAt, chainJumpTicks)) {
            chainFired = true
          } else if (ent.onGround === true && onStone) {
            execBranch = 'chain'
            // The rollout includes the body's landing speed, so it can verify the chained re-jump.
            // A refused re-jump falls back to the ordinary gates.
            if (!physics.canSprintJump(path)) {
              chainFired = true
            } else {
            chainDrivenAt = physTicks
            chainJumpTicks = (bot as unknown as { jumpTicks?: number }).jumpTicks
            bot.look(Math.atan2(-dx, -dz), 0)
            bot.setControlState('forward', true)
            bot.setControlState('sprint', true)
            bot.setControlState('jump', true)
            bot.setControlState('sneak', false)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            futile(swimming)
            return
            }
          } else if (!onStone || ent.onGround !== true) {
            const sx = chainVia.x + 0.5 - p.x
            const sz = chainVia.z + 0.5 - p.z
            execBranch = 'chain-settle'
            const sd = Math.hypot(sx, sz)
            if (sd > 0.05) bot.look(Math.atan2(-sx, -sz), 0)
            bot.setControlState('forward', sd > 0.2)
            bot.setControlState('sprint', false)
            bot.setControlState('jump', false)
            bot.setControlState('sneak', false)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            futile(swimming)
            return
          }
        }
      }

      const bounceVia = (nextPoint as Move).via
      if (bounceVia !== null && !(nextPoint as Move).chain && (nextPoint as { parkour?: boolean }).parkour === true) {
        if (bounceNode !== nextPoint) {
          bounceNode = nextPoint
          bounceRisen = false
        }
        cutTarget = null
        hopHold = false
        const vx = bounceVia.x + 0.5
        const vz = bounceVia.z + 0.5
        if (!bounceRisen && bot.entity.velocity.y > 0.05 && p.y < bounceVia.y + 0.5 &&
            Math.abs(p.x - vx) < 0.8 && Math.abs(p.z - vz) < 0.8) {
          bounceRisen = true
        }
        const ax = bounceRisen ? dx : vx - p.x
        const az = bounceRisen ? dz : vz - p.z
        execBranch = bounceRisen ? 'bounce-rise' : 'bounce-drop'
        const horiz = Math.hypot(ax, az)
        if (horiz > 0.05) bot.look(Math.atan2(-ax, -az), 0)
        bot.setControlState('forward', horiz > 0.1)
        bot.setControlState('sprint', false)
        bot.setControlState('jump', false)
        bot.setControlState('sneak', false)
        bot.setControlState('back', false)
        bot.setControlState('left', false)
        bot.setControlState('right', false)
        futile(swimming)
        return
      }

      // Steer toward the predicted landing rather than the current target heading;
      // weak air control cannot cancel accumulated drift at the end of a long fall.
      {
        const ent = bot.entity as { onGround?: boolean, velocity: Vec3 }
        const drop = p.y - nextPoint.y
        if (ent.onGround !== true && !swimming && ent.velocity.y < 0 && (drop > DROP_AIM_ENTER || (dropAimNode === nextPoint && drop > 0))) {
          const feetB = bot.blockAt(p) as BlockLike | null
          if (feetB === null || (!climbableIds.has(feetB.type))) {
            dropAimNode = nextPoint
            const v = ent.velocity
            let y = p.y
            let vy = v.y
            let s = 0
            // Require alignment above the column before reaching its top,
            // so the body lands on the block instead of striking its side.
            while (y > nextPoint.y + 1 && s < 400) { y += vy; vy = (vy - 0.08) * 0.98; s++ }
            const sum = (1 - Math.pow(0.91, s)) / 0.09
            const ex = nextPoint.x - (p.x + v.x * sum)
            const ez = nextPoint.z - (p.z + v.z * sum)
            const err = Math.hypot(ex, ez)
            if (err > 1e-4) bot.look(Math.atan2(-ex, -ez), 0, true)
            bot.setControlState('forward', err > (0.02 * 0.98 * sum) / 2)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('sprint', false)
            bot.setControlState('jump', false)
            bot.setControlState('sneak', false)
            cutTarget = null
            hopHold = false
            execBranch = 'drop-aim'
            futile(swimming)
            return
          }
        }
      }

      // Cell-centre paths zig-zag across open ground; a swept chord avoids the extra turns.
      // Keep intermediate nodes until passed so path[0] still identifies the next unvisited node.
      // Keep angle offsets relative to path[0]; applying them to the cut's heading
      // would change the jump the solver verifies.
      if ((nextPoint as Move).parkour && (nextPoint as Move).narrowLanding === null) {
        const sup = bot.blockAt(new Vec3(Math.floor(nextPoint.x), Math.floor(nextPoint.y - 0.01), Math.floor(nextPoint.z))) as BlockLike | null
        ;(nextPoint as Move).narrowLanding = sup !== null && sup.shapes.length > 0 && topCatchClass(sup.shapes) !== 0
      }
      const nodeDx = dx
      const nodeDz = dz
      // Geometry cannot account for exclusion costs, entity avoidance or swimming costs.
      // End cuts before jump approaches so take-offs follow the planner's cells.
      let jumpAhead = false
      for (let k = 0; k < Math.min(CUT_JUMP_GUARD, path.length); k++) {
        if ((path[k] as { parkour?: boolean }).parkour === true) { jumpAhead = true; break }
      }
      const mayCut = stateMovements.allowCornerCut && !swimming && path.length > 1 &&
        !jumpAhead &&
        stateMovements.exclusionAreasStep.length === 0 &&
        (nextPoint as { parkour?: boolean }).parkour !== true &&
        nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0 &&
        Math.abs(nextPoint.y - p.y) <= HOP_ARRIVE_DY
      const cutAvoid = new Set(stateMovements.blocksToAvoid)
      for (const id of stateMovements.liquids) cutAvoid.add(id)
      /**
       * Returns 'stop' at planner constraints and 'miss' for an obstructed chord.
       * The forward reference prevents cuts back to nodes the body has passed.
       */
      const cuttableTo = (k: number, ox: number, oz: number, refDx: number, refDz: number, margin: boolean): 'ok' | 'miss' | 'stop' => {
        const n = path[k]
        if (n.toBreak.length > 0 || n.toPlace.length > 0) return 'stop'
        if (Math.abs(n.y - nextPoint.y) > 0.1) return 'stop'
        // End a cut at a node before the jump guard so the take-off approach
        // begins on the planner's line, rather than partway along an offset chord.
        for (let j = k; j < Math.min(k + CUT_JUMP_GUARD, path.length); j++) {
          if (path[j].parkour) return 'stop'
        }
        const cx = n.x - ox
        const cz = n.z - oz
        const clen = Math.hypot(cx, cz)
        if (clen > CUT_MAX_CHORD) return 'stop'
        if (clen < 0.3 || cx * refDx + cz * refDz <= 0) return 'miss'
        return geometry.walkableLine(
          bot, ox, oz, n.x, n.z, nextPoint.y, cutAvoid,
          (ex, ey, ez) => stateMovements.getNumEntitiesAt({ x: ex, y: ey, z: ez }, 0, 0, 0),
          0.25, margin ? CUT_SIDE_MARGIN : 0, CUT_MARGIN_LEN)
          ? 'ok'
          : 'miss'
      }
      /** The furthest legal node among path[from .. from+count) seen from (ox, oz), or null. */
      const scanCut = (from: number, count: number, ox: number, oz: number, refDx: number, refDz: number): Move | null => {
        let best: Move | null = null
        let misses = 0
        const end = Math.min(from + count, path.length)
        for (let k = from; k < end; k++) {
          const verdict = cuttableTo(k, ox, oz, refDx, refDz, true)
          if (verdict === 'stop') break
          if (verdict === 'miss') {
            if (++misses >= CUT_MISSES) break
            continue
          }
          misses = 0
          best = path[k]
        }
        return best
      }
      if (!mayCut) cutTarget = null
      else {
        // Keep a fixed cut line and extend its target only forward to avoid heading flicker.
        // Slide the anchor along that line so lateral error remains correctable.
        // Keep the take-off target while airborne; weak air control cannot follow mid-arc changes.
        const airborne = (bot.entity as { onGround?: boolean }).onGround !== true
        if (cutTarget !== null && !airborne) {
          const at = path.indexOf(cutTarget)
          const from = cutFrom ?? p
          const ldx = cutTarget.x - from.x
          const ldz = cutTarget.z - from.z
          const llen2 = ldx * ldx + ldz * ldz
          const t = llen2 > 1e-9 ? Math.max(0, Math.min(1, ((p.x - from.x) * ldx + (p.z - from.z) * ldz) / llen2)) : 1
          const fx = from.x + ldx * t
          const fz = from.z + ldz * t
          if (at < 1 || llen2 < 1e-9 || Math.hypot(p.x - fx, p.z - fz) > CUT_MAX_OFF) cutTarget = null
          else {
            if (cutFrom === null) cutFrom = new Vec3(fx, cutTarget.y, fz)
            else { cutFrom.x = fx; cutFrom.z = fz }
            if (cuttableTo(at, fx, fz, ldx, ldz, false) !== 'ok') cutTarget = null
            else {
              const further = scanCut(at + 1, CUT_EXTEND, fx, fz, ldx, ldz)
              if (further !== null) cutTarget = further
            }
          }
        }
        if (cutTarget !== null && path.indexOf(cutTarget) < 1) cutTarget = null
        // Use the path's next leg as the forward reference so a fresh cut
        // does not select an overshot node behind the body.
        if (cutTarget === null && !airborne) {
          cutTarget = scanCut(1, CUT_SCAN_INIT, p.x, p.z, path[1].x - p.x, path[1].z - p.z)
          if (cutTarget !== null) cutFrom = p.clone()
        }
      }

      // Rollouts follow synthetic nodes one block apart along the cut's actual heading.
      // Aim ahead of the body's projection onto the line so steering corrects lateral error.
      let gatePath: Array<{ x: number, y: number, z: number, parkour?: boolean }> = path
      if (cutTarget !== null) {
        const at = path.indexOf(cutTarget)
        const from = cutFrom ?? p
        const ldx = cutTarget.x - from.x
        const ldz = cutTarget.z - from.z
        const llen = Math.hypot(ldx, ldz)
        if (llen < 1e-6) cutTarget = null
        else {
          const ux = ldx / llen
          const uz = ldz / llen
          const t = Math.max(0, Math.min(1, ((p.x - from.x) * ldx + (p.z - from.z) * ldz) / (llen * llen)))
          const fx = from.x + ldx * t
          const fz = from.z + ldz * t
          const ahead = llen * (1 - t)
          const reach = Math.min(CUT_CARROT, ahead)
          dx = fx + ux * reach - p.x
          dz = fz + uz * reach - p.z
          const line: Array<{ x: number, y: number, z: number, parkour?: boolean }> = []
          for (let s = CUT_GATE_STEP; s < ahead - 0.01; s += CUT_GATE_STEP) {
            line.push({ x: fx + ux * s, y: cutTarget.y, z: fz + uz * s, parkour: false })
          }
          gatePath = [...line, ...path.slice(at)]
        }
      }

      // The slow stall check tolerates turns and landings; the fast check detects blocked walking.
      // Suppress the fast check during kernel line-up so recovery does not undo its approach.
      const stillTicks = updateWedge(p)
      const wedged = stillTicks > WEDGE_TICKS ||
        (!kernelLed && stillTicks >= 1 && (bot.entity as { onGround?: boolean }).onGround === true &&
         physics.isGrinding(gatePath))
      // A wedge contradicts the cut's swept line; recovery needs the original path[0].
      if (wedged && cutTarget !== null) {
        cutTarget = null
        gatePath = path
        dx = nodeDx
        dz = nodeDz
      }

      // The planner prices the open corner of a diagonal; aiming at its centre can clip the blocked corner.
      // Insert that corner only when wedged, or for a flat step physics already refuses.
      // Rising steps retain the jump gates to avoid unnecessary sidesteps.
      let squeezed = false
      const walkNode = (nextPoint as { parkour?: boolean }).parkour !== true &&
        nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0
      // A live cut already verifies its chord; inserting a waypoint would override that heading.
      const flatDiagonal = walkNode && !swimming && !wedged && cutTarget === null && earlySqueeze &&
        Math.abs(dy) <= 0.1 &&
        Math.abs(Math.floor(nextPoint.x) - Math.floor(p.x)) === 1 &&
        Math.abs(Math.floor(nextPoint.z) - Math.floor(p.z)) === 1
      if ((wedged && walkNode) || flatDiagonal) {
        const bx = Math.floor(p.x)
        const by = Math.floor(p.y + 0.001)
        const bz = Math.floor(p.z)
        const tx = Math.floor(nextPoint.x)
        const tz = Math.floor(nextPoint.z)
        if (Math.abs(tx - bx) === 1 && Math.abs(tz - bz) === 1 && Math.abs(dy) <= 1.3) {
          const yTest = Math.max(by, Math.floor(nextPoint.y + 0.001))
          const openA = !geometry.playerCollides(bot, tx + 0.5, yTest, bz + 0.5)
          const openB = !geometry.playerCollides(bot, bx + 0.5, yTest, tz + 0.5)
          if (openA !== openB && (wedged || !physics.canStraightLine(path, false))) {
            const wx = openA ? tx : bx
            const wz = openA ? bz : tz
            if (geometry.isStandable(bot, new Vec3(wx, by, wz))) {
              let np = getPositionOnTopOf(bot.blockAt(new Vec3(wx, by, wz)) as BlockLike | null)
              if (np === null) np = getPositionOnTopOf(bot.blockAt(new Vec3(wx, by - 1, wz)) as BlockLike | null)
              const waypoint = new Move(wx, by, wz, nextPoint.remainingBlocks, 1)
              if (np !== null) {
                waypoint.x = np.x
                waypoint.y = np.y
                waypoint.z = np.z
              } else {
                waypoint.x = wx + 0.5
                waypoint.y = by
                waypoint.z = wz + 0.5
              }
              squeezed = true
              clearWedge() // The waypoint provides recovery; avoid a second nudge.
              path.unshift(waypoint)
              nextPoint = waypoint
              dx = nextPoint.x - p.x
              dy = nextPoint.y - p.y
              dz = nextPoint.z - p.z
            }
          }
        }
      }

      // When diagonal supports meet beside a blocked corner with no floor around it,
      // wall contact lets the body slide between them; standoff would steer it over the hole.
      // Latch this to the node because the body enters the open corner's cell during the crossing.
      if (hugNode !== nextPoint && walkNode && !swimming && !squeezed && Math.abs(dy) <= 0.1) {
        const bx = Math.floor(p.x)
        const by = Math.floor(p.y + 0.001)
        const bz = Math.floor(p.z)
        const tx = Math.floor(nextPoint.x)
        const tz = Math.floor(nextPoint.z)
        if (Math.abs(tx - bx) === 1 && Math.abs(tz - bz) === 1) {
          const openA = !geometry.playerCollides(bot, tx + 0.5, by, bz + 0.5)
          const openB = !geometry.playerCollides(bot, bx + 0.5, by, tz + 0.5)
          if (openA !== openB && !geometry.isStandable(bot, new Vec3(openA ? tx : bx, by, openA ? bz : tz))) hugNode = nextPoint
        }
      }
      const hugCorner = hugNode === nextPoint

      // Climbables ascend through horizontal wall contact; a vertical target gives no useful yaw.
      // Press into the current wall until high enough to step around a pillar corner.
      let climbing = false
      /** Moving along a ladder wall this tick: the heading below is driven directly, not through the gates. */
      let traverse = false
      /** Letting go to slide down the ladder to a node below (no forward, no sneak). */
      let descend = false
      /** Jumping up into a ladder that starts at head height (moveGen.moveUp's head-height entry). */
      let jumpInto = false
      {
        const feet = bot.blockAt(p) as BlockLike | null
        const head = bot.blockAt(p.offset(0, 1, 0)) as BlockLike | null
        if (feet && !climbableIds.has(feet.type) &&
            head && (climbableIds.has(head.type)) &&
            (bot.entity as { onGround?: boolean }).onGround === true &&
            Math.abs(nodeDx) < 0.35 && Math.abs(nodeDz) < 0.35 && nextPoint.y > p.y + 0.5 && nextPoint.y < p.y + 1.6) {
          // Press into the wall while jumping into the head-height ladder,
          // so the first airborne tick already has the collision needed to climb.
          const wall = climbWall(head, p.offset(0, 1, 0))
          if (wall !== null) {
            dx = wall[0]
            dz = wall[1]
            climbing = true
            traverse = true
            jumpInto = true
          }
        } else if (feet && (climbableIds.has(feet.type))) {
          const hanging = (bot.entity as { onGround?: boolean }).onGround !== true
          const straightUp = Math.abs(nodeDx) < 0.2 && Math.abs(nodeDz) < 0.2
          const wall = climbWall(feet, p)
          // Traverse with both wall contact and sideways input; sideways input alone loses height.
          // Use only the along-wall node offset, since its aim can sit inside the ladder panel.
          // A large into-wall offset indicates a different face requiring a corner transfer.
          const perp = nodeDx * (wall?.[0] ?? 0) + nodeDz * (wall?.[1] ?? 0)
          const latX = nodeDx - perp * (wall?.[0] ?? 0)
          const latZ = nodeDz - perp * (wall?.[1] ?? 0)
          const lat = Math.hypot(latX, latZ)
          // Use a hanging traverse only when level with the node or blocked above;
          // ordinary climb-and-drift preserves upward progress while the body can rise.
          let canRise = nextPoint.y > p.y + 0.3
          if (canRise) {
            const hb = bot.blockAt(new Vec3(Math.floor(p.x), Math.floor(p.y + 2.0), Math.floor(p.z))) as BlockLike | null
            canRise = !(hb !== null && !climbableIds.has(hb.type) && hb.shapes.length > 0)
          }
          if (wall !== null && hanging && !canRise && lat >= 0.35 && Math.abs(perp) <= 0.55) {
            const nx = Math.floor(nextPoint.x)
            const nz = Math.floor(nextPoint.z)
            const ny = Math.floor(nextPoint.y + 0.001)
            const isClimb = (xx: number, yy: number, zz: number): boolean => {
              const nb = bot.blockAt(new Vec3(xx, yy, zz)) as BlockLike | null
              return nb !== null && (climbableIds.has(nb.type))
            }
            if (isClimb(nx, ny, nz) || isClimb(nx, ny - 1, nz)) {
              // The next column needs a ladder at the body's height for a level or rising transfer.
              // A descending transfer can drop into a ladder anywhere down to the node's height.
              const fy = Math.floor(p.y + 0.001)
              const sx = Math.floor(p.x + latX / lat * 0.6)
              const sz = Math.floor(p.z + latZ / lat * 0.6)
              // Require clearance for the body's full height beside the wall,
              // and a ladder it can catch at its current height or during the descent.
              let fits = true
              for (let yy = fy; fits && yy <= Math.floor(p.y + 1.8); yy++) {
                const nb = bot.blockAt(new Vec3(sx, yy, sz)) as BlockLike | null
                if (nb !== null && !climbableIds.has(nb.type) && nb.shapes.length > 0) fits = false
              }
              let hang = isClimb(sx, fy, sz)
              for (let yy = fy - 1; !hang && nextPoint.y < p.y - 0.3 && yy >= ny; yy--) hang = isClimb(sx, yy, sz)
              // Wall contact climbs at 0.2 blocks per tick; releasing it slides down at 0.15.
              // Alternate around the node's height to traverse level or fit below a step.
              const press = p.y < nextPoint.y - 0.05
              if (fits && hang) {
                // Keep wall contact under a beam; leaning outward to stop rising can drop the catch.
                dx = latX / lat + (press ? wall[0] * CLIMB_PRESS : 0)
                dz = latZ / lat + (press ? wall[1] * CLIMB_PRESS : 0)
                climbing = true
                traverse = true
              } else if (nextPoint.y < p.y - 0.3) {
                // The side column is solid here with ladders below; pressing across climbs into it.
                // Release and slide until the body fits beside the wall.
                dx = -wall[0]
                dz = -wall[1]
                climbing = true
                traverse = true
                descend = true
              } else if (nextPoint.y > p.y - 0.1) {
                dx = wall[0]
                dz = wall[1]
                climbing = true
                traverse = true
              }
            }
          }
          if (!traverse && wall !== null && nextPoint.y > p.y + 0.1 &&
              (straightUp || (hanging && nextPoint.y > p.y + 0.3))) {
            dx = wall[0]
            dz = wall[1]
            climbing = true
            // A wall press alone leaves along-wall momentum free to carry the body out of the column.
            // Counter its projected drift relative to the node's along-wall offset.
            if (hanging) {
              const v = bot.entity.velocity
              const ax = wall[1] !== 0 ? 1 : 0
              const az = wall[0] !== 0 ? 1 : 0
              const along = nodeDx * ax + nodeDz * az - (v.x * ax + v.z * az) * CLIMB_DRIFT
              if (Math.abs(along) > CLIMB_HOLD) {
                dx += ax * Math.sign(along)
                dz += az * Math.sign(along)
              }
            }
          }
          // Release forward and sneak to slide down the ladder at 0.15 blocks per tick.
          if (!traverse && !climbing && hanging && wall !== null && lat < 0.35 && nextPoint.y < p.y - 0.3) {
            dx = -wall[0]
            dz = -wall[1]
            climbing = true
            traverse = true
            descend = true
          }
        }
      }

      // Hold upward catch controls before a flight branch or gate can clear them.
      if (CLIMB_HOLD_ENABLED && !swimming && !descend && nextPoint.y > p.y + 0.01 &&
          (bot.entity as { onGround?: boolean }).onGround !== true) {
        const feet = bot.blockAt(p) as BlockLike | null
        const sameColumn = Math.floor(nextPoint.x) === Math.floor(p.x) && Math.floor(nextPoint.z) === Math.floor(p.z)
        // Use climb controls only when the current column holds a planned climb node.
        // A verified flight merely passing through a vine keeps its own controls.
        const inFeetColumn = (n: { x: number, z: number } | null | undefined): boolean =>
          n !== null && n !== undefined && Math.floor(n.x) === Math.floor(p.x) && Math.floor(n.z) === Math.floor(p.z)
        const planned = sameColumn || inFeetColumn(prevRetired) || inFeetColumn(path[1])
        const flightOwned = kernelHop || biasFlight || jumpScript !== null
        if (feet !== null && climbableIds.has(feet.type) && (planned || !flightOwned)) {
          const sideways = traverse && !sameColumn
          let hx = sameColumn ? nodeDx : dx
          let hz = sameColumn ? nodeDz : dz
          const wall = sameColumn ? climbWall(feet, p) : null
          if (wall !== null) {
            const v = bot.entity.velocity
            const ax = wall[1] !== 0 ? 1 : 0
            const az = wall[0] !== 0 ? 1 : 0
            const along = (Math.floor(p.x) + 0.5 - p.x) * ax + (Math.floor(p.z) + 0.5 - p.z) * az - (v.x * ax + v.z * az) * CLIMB_DRIFT
            hx = wall[0]
            hz = wall[1]
            if (Math.abs(along) > CLIMB_HOLD) {
              hx += ax * Math.sign(along)
              hz += az * Math.sign(along)
            }
          }
          if (Math.hypot(hx, hz) > 1e-9) bot.look(Math.atan2(-hx, -hz), 0, true)
          kernelHop = false
          hopHold = false
          biasFlight = false
          jumpScript = null
          bot.setControlState('forward', true)
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
          bot.setControlState('jump', true)
          climbKeysHeld = true
          bot.setControlState('sprint', false)
          // Sneak arrests a falling catch, but while rising it cuts steering to 30%.
          bot.setControlState('sneak', !sideways && bot.entity.velocity.y < 0)
          execBranch = sideways ? 'traverse' : 'catch'
          futile(swimming)
          return
        }
      }

      // A heading along a flush wall cannot create sprint clearance on its own.
      // Steer away where ground continues, before the sprint gate probes the resulting heading.
      if (!climbing && !swimming && !hugCorner && (nextPoint as { parkour?: boolean }).parkour !== true &&
          dy <= STEP_UP_MIN && Math.hypot(dx, dz) > 0.15) {
        steer.dx = dx
        steer.dz = dz
        sideStandoff(boxProbe, p.x, p.y, p.z, steer)
        dx = steer.dx
        dz = steer.dz
      }

      // Vanilla cancels sprint on horizontal collision; servers can reject sprinting wall contact.
      // Probe one sprinting tick ahead, since collision flags arrive after the position is sent.
      const againstWall = !climbing &&
        wallAhead(boxProbe, geometry.nearWall(bot, SPRINT_WALL_MARGIN), p.x, p.y, p.z, dx, dz)
      // An approved parkour flight retains sprint through corner contact;
      // cutting it mid-arc can leave the landing short.
      const flyingParkour = (nextPoint as { parkour?: boolean }).parkour === true &&
        (bot.entity as { onGround?: boolean }).onGround !== true
      const maySprint = stateMovements.allowSprinting && (flyingParkour || !againstWall)

      // Slide along walls during walking; diagonal clamping can cause server corrections.
      // End a cut when the wall contradicts its swept chord, using only axes the chord moves along.
      const slideLive = cutTarget !== null
      const slideDx = slideLive ? dx : nodeDx
      const slideDz = slideLive ? dz : nodeDz
      // Skip wall probes during kernel replay: jump-height overhangs do not
      // invalidate the ground chord, and replay already fixes the flight's headings.
      if (!kernelHop && !climbing && !hugCorner && (nextPoint as { parkour?: boolean }).parkour !== true &&
          dy <= STEP_UP_MIN && (slideLive || Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH)) {
        // Ignore tiny lateral components so standoff does not amplify floating-point noise.
        // A square-on approach has no slide direction; recovery handles it if blocked.
        // Require ground away from the wall so standoff cannot steer off a narrow support.
        steer.dx = dx
        steer.dz = dz
        if (wallSlide(boxProbe, p.x, p.y, p.z, slideDx, slideDz, nodeDx, nodeDz, steer)) {
          cutTarget = null
          gatePath = path
          dx = steer.dx
          dz = steer.dz
        }
      }

      // Low air takes priority over the path and wedge recovery because the planner does not track breath.
      // Surfacing counts as progress even when no path node is reached.
      const air = (bot as { oxygenLevel?: number }).oxygenLevel
      if (swimming && air !== undefined && air <= AIR_RESERVE) {
        bot.clearControlStates()
        bot.setControlState('jump', true)
        lastNodeTime = performance.now()
        return
      }

      // Physics predictions can disagree with the server, so recovery responds to lack of movement.
      // Simulate each escape before using it and retain an angled jump's heading through its arc.
      if (biasFlight && (bot.entity as { onGround?: boolean }).onGround !== true &&
          headingBias !== null) {
        execBranch = 'bias-flight'
        bot.look(Math.atan2(-nodeDx, -nodeDz) + headingBias, 0)
        bot.setControlState('forward', true)
        bot.setControlState('sprint', headingSprint)
        futile(swimming)
        return
      }
      if (biasFlight && (bot.entity as { onGround?: boolean }).onGround === true) biasFlight = false

      // A blocked parkour take-off can need a mid-air turn that ordinary gates cannot find.
      // Search a control script from the current state and stop replay if the body leaves its track.
      if (jumpScript !== null && jumpScriptNode !== nextPoint) jumpScript = null
      // Try the exact kernel before the scripted search.
      if (jumpScript === null && JUMP_SOLVER && physics.solveJump !== undefined && !climbing && !swimming &&
          (nextPoint as { parkour?: boolean }).parkour === true &&
          (bot.entity as { onGround?: boolean }).onGround === true && jumpSolveFailedNode !== nextPoint) {
        const v = bot.entity.velocity
        const speed = Math.hypot(v.x, v.z)
        // Neo jumps need a route around the pillar that the ordinary gates do not fly.
        // Search immediately rather than waiting for the wedge timer.
        const neo = (nextPoint as Move).wOff !== 0
        if (neo && kernelTakeoff && takeoffSearched !== nextPoint) {
          // A program from the moving arrival state can avoid a stop.
          takeoffSearched = nextPoint
          takeoffSearchedAtRest = speed < 0.01
          if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
          // Try a kick for a neo landing flush against the pillar when the other families find no flight.
          const live = physics.hopFrom?.((nextPoint as Move).cell, TAKEOFF_SEARCH_TICKS, undefined, nextLegCell(), true, SEED_FAMILY ? NEO_SEED : undefined, false) ?? null
          takeoffNote = physics.hopNote ?? ''
          // Skip the neo when a direct flight to the following node is quicker.
          if (live !== null && trySkipNeo(live.ticks, speed < 0.01)) return
          if (takeoffLive(live)) return
          if (tryPrefix(undefined, climbing)) return
        }
        if (neo && speed >= 0.01) {
          bot.setControlState('forward', false)
          bot.setControlState('back', false)
          bot.setControlState('jump', false)
          bot.setControlState('sprint', false)
          bot.setControlState('sneak', true)
          jumpStillTicks = 0
          execBranch = 'solve-brake'
          futile(swimming)
          return
        }
        jumpStillTicks = speed < 0.035 ? jumpStillTicks + 1 : 0
        // Delay scripted search until ordinary line-up and wedge recovery have had a chance.
        // Standing still can be part of a valid take-off approach.
        if (neo || (wedged && jumpStillTicks >= SCRIPT_STILL_TICKS)) {
          // Give the angled take-off priority over scripted search once wedged.
          if (!neo && !squeezed && wedged && driveAngledJump(nextPoint, nodeDx, nodeDz)) {
            execBranch = 'angled'
            futile(swimming)
            return
          }
          // Allow a fresh futility window for the search and its script.
          if (jumpStillTicks === SCRIPT_STILL_TICKS || (neo && jumpSolveStartNode !== nextPoint)) {
            jumpSolveStartNode = nextPoint
            lastNodeTime = performance.now()
          }
          if (kernelTakeoff && !(takeoffSearched === nextPoint && takeoffSearchedAtRest)) {
            takeoffSearched = nextPoint
            takeoffSearchedAtRest = true
            if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
            const restNow = speed < 0.01
            if (restNow) restSearchedAt = { node: nextPoint, x: p.x, z: p.z }
            const live = physics.hopFrom?.((nextPoint as Move).cell, restNow ? TAKEOFF_REST_TICKS : TAKEOFF_SEARCH_TICKS, undefined, nextLegCell(), true, SEED_FAMILY && neo ? NEO_SEED : undefined, false) ?? null
            takeoffNote = physics.hopNote ?? ''
            if (neo && live !== null && trySkipNeo(live.ticks, restNow)) return
            if (takeoffLive(live)) return
          }
          if (neo && tryPrefix(undefined, climbing)) return
          // Limit this search to once per node, independently of the live search.
          if (kernelTakeoff && linedSearched !== nextPoint) {
            linedSearched = nextPoint
            const lined = physics.hopLinedUp?.((nextPoint as Move).cell, TAKEOFF_LINED_TICKS) ?? null
            takeoffNote += ' | ' + (physics.hopNote ?? '')
            if (lined !== null) {
              ;(nextPoint as Move).program = lined
              linedInstalled = nextPoint
              progNode = null
              jumpStillTicks = 0
              lastNodeTime = performance.now()
              bot.setControlState('forward', false)
              bot.setControlState('back', false)
              bot.setControlState('jump', false)
              bot.setControlState('sprint', false)
              bot.setControlState('sneak', true)
              execBranch = 'takeoff-lined'
              futile(swimming)
              return
            }
            // An exhaustive refusal from every family rules out this take-off;
            // ban the landing and replan instead of repeating line-ups.
            if (LINED_GIVEUP && kernelTakeoff) {
              // A search that exhausts its budget has no verdict; retain the other recovery attempts.
              if (physics.hopExhausted !== true) linedOpen = nextPoint
              else {
                // Associate the verdict with the cell the search starts from.
                // At an overhanging lip, that is still the take-off cell, not the cell beneath the body's centre.
                const pc = prevRetired?.cell
                const lf = physics.linedFrom ?? null
                let judged = pc !== undefined && lf !== null
                  ? lf[0] === pc[0] && lf[1] === pc[1] && lf[2] === pc[2]
                  : prevRetired !== null && Math.floor(prevRetired.x) === Math.floor(p.x) && Math.floor(prevRetired.z) === Math.floor(p.z)
                if (!judged && prevRetired !== null && pc !== undefined && physics.hopFromCell !== undefined &&
                    Math.abs(prevRetired.x - p.x) <= LINED_NEAR && Math.abs(prevRetired.z - p.z) <= LINED_NEAR) {
                  const own = physics.hopFromCell(pc, (nextPoint as Move).cell, TAKEOFF_LINED_TICKS, true)
                  takeoffNote += ' | ' + (physics.hopNote ?? '')
                  if (own !== null) {
                    ;(nextPoint as Move).program = own
                    linedInstalled = nextPoint
                    progNode = null
                    jumpStillTicks = 0
                    lastNodeTime = performance.now()
                    bot.setControlState('forward', false)
                    bot.setControlState('back', false)
                    bot.setControlState('jump', false)
                    bot.setControlState('sprint', false)
                    bot.setControlState('sneak', true)
                    execBranch = 'takeoff-lined'
                    futile(swimming)
                    return
                  }
                  judged = physics.hopExhausted === true
                }
                if (judged) {
                  banTakeoff(nextPoint)
                  execBranch = 'giveup'
                  resetPath('stuck', true)
                  return
                }
                // A verdict from another cell triggers a replan; the second loss bans the edge.
                if (lostReplan('lined-lost')) return
              }
            } else if (physics.hopExhausted === true && prevRetired !== null &&
                Math.floor(prevRetired.x) === Math.floor(p.x) && Math.floor(prevRetired.z) === Math.floor(p.z)) {
              banTakeoff(nextPoint)
              execBranch = 'giveup'
              resetPath('stuck', true)
              return
            }
          }
          // Fall back to the scripted search only when the lined search has no verdict.
          if (LINED_GIVEUP && kernelTakeoff && linedOpen !== nextPoint) {
            jumpStillTicks = 0
            jumpSolveFailedNode = nextPoint
          } else {
          const found = physics.solveJump(nextPoint, SCRIPT_BUDGET)
          if (found === undefined) {
            // Hold the state the search starts from.
            bot.setControlState('forward', false)
            bot.setControlState('back', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            execBranch = 'solving'
            futile(swimming)
            return
          }
          jumpStillTicks = 0
          if (found === null) {
            jumpSolveFailedNode = nextPoint
          } else {
            jumpScript = found
            jumpScriptNode = nextPoint
            jumpScriptTick = 0
            jumpScriptAir = 0
            lastNodeTime = performance.now()
          }
          }
        }
      } else {
        jumpStillTicks = 0
      }

      if (jumpScript !== null) {
        const want = jumpScriptTick > 0 ? jumpScript.track[jumpScriptTick - 1] : null
        const strayed = want !== null && want !== undefined &&
          Math.hypot(want.x - p.x, want.y - p.y, want.z - p.z) > SCRIPT_TRACK_TOL
        if (strayed || jumpScriptTick >= jumpScript.ticks) {
          if (process.env.PF_SCRIPT_DEBUG === '1') console.warn(`[script] ${strayed ? 'STRAYED' : 'done'} at tick ${jumpScriptTick}/${jumpScript.ticks}: want ${want !== null && want !== undefined ? `${want.x.toFixed(3)},${want.y.toFixed(3)},${want.z.toFixed(3)}` : '-'} is ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)} (yawA ${jumpScript.yawA.toFixed(2)} yawB ${jumpScript.yawB.toFixed(2)} turn ${jumpScript.turnTick} ja ${jumpScript.jumpAfter} sprint ${String(jumpScript.sprint)})`)
          jumpScript = null
        } else {
          const onGroundNow = (bot.entity as { onGround?: boolean }).onGround === true
          if (jumpScriptAir > 0 || (!onGroundNow && bot.entity.velocity.y > 0)) jumpScriptAir++
          const turned = jumpScriptAir > jumpScript.turnTick
          bot.look(turned ? jumpScript.yawB : jumpScript.yawA, 0, true)
          bot.setControlState('forward', true)
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
          bot.setControlState('sneak', false)
          bot.setControlState('sprint', jumpScript.sprint)
          bot.setControlState('jump', jumpScriptAir === 0 && jumpScriptTick >= jumpScript.jumpAfter)
          jumpScriptTick++
          execBranch = 'script'
          futile(swimming)
          return
        }
      }
      // Apply catch controls immediately: sneak prevents a slide below the ladder,
      // while the wall press and drift correction keep the body in its column.
      if (airborne && !climbing && !swimming) {
        const fb = bot.blockAt(p) as BlockLike | null
        if (fb !== null && climbableIds.has(fb.type)) {
          const inColumn = (n: { x: number, y: number, z: number } | undefined): boolean =>
            n !== undefined && Math.floor(n.x) === Math.floor(p.x) && Math.floor(n.z) === Math.floor(p.z) &&
            n.y >= Math.floor(p.y) && p.y < n.y + 1
          if (inColumn(nextPoint) || inColumn(path[1])) {
            const wall = climbWall(fb, p)
            if (wall !== null) {
              const v = bot.entity.velocity
              const ax = wall[1] !== 0 ? 1 : 0
              const az = wall[0] !== 0 ? 1 : 0
              const along = (Math.floor(p.x) + 0.5 - p.x) * ax + (Math.floor(p.z) + 0.5 - p.z) * az - (v.x * ax + v.z * az) * CLIMB_DRIFT
              let hx = wall[0]
              let hz = wall[1]
              if (Math.abs(along) > CLIMB_HOLD) {
                hx += ax * Math.sign(along)
                hz += az * Math.sign(along)
              }
              bot.look(Math.atan2(-hx, -hz), 0, true)
            }
            kernelHop = false
            bot.setControlState('forward', true)
            bot.setControlState('back', false)
            bot.setControlState('left', false)
            bot.setControlState('right', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', true)
            execBranch = 'catch'
            futile(swimming)
            return
          }
        }
      }

      // Adjust air controls when forward and sprint miss a narrow landing.
      // Kernel flights keep their verified controls through touchdown.
      if (airborne && !climbing && !swimming && !kernelHop && (nextPoint as { parkour?: boolean }).parkour === true &&
          physics.airControl !== undefined && AIR_CONTROL) {
        // Full blocks have room to overrun; braking onto their centres only slows the run.
        if (airNode !== nextPoint) {
          airNode = nextPoint
          airEngaged = false
          airNarrow = (nextPoint as Move).narrowLanding === true
        }
        let air: { forward: boolean, back: boolean, sprint: boolean } | null = null
        if (airNarrow) {
          const r = physics.airControl(nextPoint, airEngaged)
          airEngaged = r.engaged
          air = r.control
        }
        if (air !== null) {
          bot.look(Math.atan2(-dx, -dz), 0)
          bot.setControlState('forward', air.forward)
          bot.setControlState('back', air.back)
          bot.setControlState('sprint', air.sprint)
          bot.setControlState('jump', false)
          bot.setControlState('sneak', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
          execBranch = 'aircontrol'
          futile(swimming)
          return
        }
      }

      // Keep the kernel's flight controls through touchdown; the gate cascade
      // can otherwise replace the verified arc with a different one.
      if (kernelHop) {
        if ((bot.entity as { onGround?: boolean }).onGround === true || climbing || swimming) {
          kernelHop = false
        } else {
          // Preserve the scheduled yaw until the sequence ends.
          khopTick++
          const taped = khopFlight !== null && khopTick < khopFlight.length
          if (taped) bot.look((khopFlight as number[])[khopTick], 0, true)
          else bot.look(Math.atan2(-nodeDx, -nodeDz), 0)
          // An eased flight coasts from its scheduled tick onward.
          const eased = khopTick >= khopEase
          bot.setControlState('forward', !eased)
          bot.setControlState('back', false)
          bot.setControlState('left', taped && khopStrafe && !eased)
          gaitStrafed = taped && khopStrafe && !eased
          bot.setControlState('right', false)
          bot.setControlState('sneak', false)
          bot.setControlState('jump', false)
          bot.setControlState('sprint', true)
          execBranch = 'khop'
          futile(swimming)
          return
        }
      }

      // Ground movement gates reject a ladder traverse and fall through to a stationary sneak.
      if (traverse) {
        bot.look(Math.atan2(-dx, -dz), 0)
        bot.setControlState('forward', !descend)
        bot.setControlState('back', false)
        bot.setControlState('sneak', false)
        bot.setControlState('sprint', false)
        bot.setControlState('jump', jumpInto)
        execBranch = jumpInto ? 'jumpinto' : descend ? 'descend' : 'traverse'
        futile(swimming)
        return
      }

      // Air control cannot quickly realign momentum with a new heading.
      // Let ground friction align the velocity before hopping into a turn.
      const groundedNow = (bot.entity as { onGround?: boolean }).onGround === true
      const speedNow = Math.hypot(bot.entity.velocity.x, bot.entity.velocity.z)
      const headNow = Math.hypot(dx, dz)
      const turning = speedNow > 0.1 && headNow > 1e-6 &&
        (bot.entity.velocity.x * dx + bot.entity.velocity.z * dz) / (speedNow * headNow) < HOP_TURN_COS
      // Replay the kernel's schedule before gates or recovery can change its timing.
      // Short streamed partial paths stay on the ground.
      let partialLeft = Infinity
      if (pathPartial && path.length > 0) {
        partialLeft = Math.hypot(path[0].x - p.x, path[0].z - p.z)
        for (let i = 1; i < path.length && partialLeft < PARTIAL_HOP_MIN; i++) partialLeft += Math.hypot(path[i].x - path[i - 1].x, path[i].z - path[i - 1].z)
      }
      if (groundedNow && !swimming && !climbing && stateMovements.allowSprintHop && stateMovements.allowHopIntoJump &&
          physics.gaitDrive !== undefined && partialLeft >= PARTIAL_HOP_MIN) {
        if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
        physics.gaitFrom = prevRetired
        physics.wholeFall = wholeFall
        physics.gaitStrafe = stateMovements.allowStrafe
        physics.gaitLandRetire = stateMovements.allowLandingRetire
        neoAhead()
        const drive = physics.gaitDrive(gatePath)
        kernelLed = drive.kind !== 'none'
        if (kernelLed && (trySkip(drive, gatePath) || trySkipWalk(gatePath))) return
        if (kernelLed) {
          hopHold = drive.kind === 'hop'
          kernelHop = hopHold
          if (drive.flight !== null) {
            khopFlight = drive.flight
            khopTick = 0
            khopStrafe = drive.strafe
            khopEase = drive.ease
          }
          gaitStrafed = drive.flight === null && drive.strafe
          bot.look(drive.yaw, 0, true)
          bot.setControlState('forward', true)
          bot.setControlState('back', false)
          bot.setControlState('left', gaitStrafed)
          bot.setControlState('right', false)
          bot.setControlState('sneak', false)
          bot.setControlState('jump', hopHold)
          bot.setControlState('sprint', drive.sprint)
          execBranch = hopHold ? 'hop' : 'krun'
          futile(swimming)
          return
        }
      } else if (groundedNow) {
        kernelLed = false
      }
      // A scheduled walk-off follows the kernel's fall aim down the path
      // once the lower node is behind the body.
      if (!groundedNow && kernelLed && !kernelHop && !climbing && !swimming && stateMovements.allowHopIntoJump) {
        const to = gatePath[fallAim(gatePath, 0, p.x, p.z, bot.entity.velocity.x, bot.entity.velocity.z)]
        bot.look(Math.atan2(-(to.x - p.x), -(to.z - p.z)), 0, true)
        bot.setControlState('forward', true)
        bot.setControlState('back', false)
        bot.setControlState('left', false)
        bot.setControlState('right', false)
        bot.setControlState('sneak', false)
        bot.setControlState('jump', false)
        bot.setControlState('sprint', true)
        execBranch = 'kfall'
        futile(swimming)
        return
      }

      // Reach the shifted take-off point before running the gates; the jump
      // is verified from that flight line, with sneak near the lip.
      {
        const shiftTo = (nextPoint as Move).takeoff
        if (shiftTo !== null && shiftTo !== undefined && !climbing && !swimming &&
            (bot.entity as { onGround?: boolean }).onGround === true) {
          const ex = p.x - shiftTo.x
          const ez = p.z - shiftTo.z
          const lx = nextPoint.x - shiftTo.x
          const lz = nextPoint.z - shiftTo.z
          const ll = Math.hypot(lx, lz)
          const lat = ll > 1e-6 ? Math.abs(ex * lz - ez * lx) / ll : Math.hypot(ex, ez)
          const along = ll > 1e-6 ? (ex * lx + ez * lz) / ll : 0
          if (lat > SHIFT_LINE_TOL && along < 0.3) {
            bot.look(Math.atan2(ex, ez), 0)
            bot.setControlState('forward', true)
            bot.setControlState('back', false)
            bot.setControlState('jump', false)
            bot.setControlState('sprint', false)
            bot.setControlState('sneak', Math.hypot(ex, ez) < 0.7)
            execBranch = 'shift'
            futile(swimming)
            return
          }
        }
      }

      if (!squeezed && wedged) {
        if (driveAngledJump(nextPoint, nodeDx, nodeDz)) { execBranch = 'angled'; futile(swimming); return }
        if (driveRecovery(nextPoint)) { execBranch = 'recovery'; futile(swimming); return }
      }

      bot.look(Math.atan2(-dx, -dz), 0)
      bot.setControlState('forward', true)
      bot.setControlState('jump', false)
      // Flat steps can use an angled walk before the jump gates try them.
      const flatWalkStep = walkNode && !swimming && !climbing && dy <= STEP_UP_MIN &&
        (cutTarget !== null || Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH)
      let walkBias: number | null = null
      // Release sneak before take-off so the jump retains its verified speed.
      bot.setControlState('sneak', false)
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)

      if (swimming) {
        // Water nodes follow the plan's level; unconditional swim-up can carry
        // the body up a waterfall instead of across it.
        hopHold = false
        const feet = bot.blockAt(bot.entity.position) as { type: number } | null
        const inColumn = feet !== null && feet.type === bubbleColumnId
        // Swim-up can stall a down-column, so assist only when the node is clearly above.
        bot.setControlState('jump', nextPoint.y > bot.entity.position.y + (inColumn ? 0.25 : -0.1))
        bot.setControlState('sprint', false)
        execBranch = 'swim'
      } else if (maySprint && physics.canStraightLine(gatePath, true)) {
        // Hopping preserves the sprint boost that ground friction removes.
        // Without a kernel schedule, choose it only when the rollout gains distance without losing height.
        if (groundedNow) {
          kernelHop = false
          hopHold = stateMovements.allowSprintHop && !stateMovements.allowHopIntoJump && !turning &&
            physics.sprintHopBetter(gatePath, stateMovements.allowLowCeilingHop)
        }
        // A held jump incurs prismarine-physics' 10-tick re-jump cooldown.
        // Release between jumps so a low ceiling does not force a wait after landing.
        bot.setControlState('jump', hopHold && (bot.entity as { onGround?: boolean }).onGround === true)
        bot.setControlState('sprint', true)
        execBranch = hopHold ? 'hop' : 'sprint'
      } else if (flatWalkStep && (walkBias = physics.bestWalkHeading(gatePath, maySprint)) !== null) {
        // Try walking around a clipped corner before jumping it; recompute the
        // offset each tick so straight walking resumes as soon as it fits.
        hopHold = false
        // Use the chord's heading during a cut.
        if (cutTarget !== null) bot.look(Math.atan2(-dx, -dz) + walkBias, 0)
        else bot.look(Math.atan2(-nodeDx, -nodeDz) + walkBias, 0)
        bot.setControlState('jump', false)
        bot.setControlState('sprint', maySprint)
        execBranch = 'walk-angled'
      } else if (maySprint && physics.canSprintJump(gatePath)) {
        hopHold = false
        bot.setControlState('jump', true)
        bot.setControlState('sprint', true)
        execBranch = 'sprintjump'
      } else if (physics.canStraightLine(gatePath)) {
        hopHold = false
        bot.setControlState('jump', false)
        bot.setControlState('sprint', false)
        execBranch = maySprint ? 'walk' : 'walk-nosprint'
      } else if (physics.canWalkJump(gatePath)) {
        hopHold = false
        bot.setControlState('jump', true)
        bot.setControlState('sprint', false)
        execBranch = 'walkjump'
      } else {
        hopHold = false
        // Search from the moving arrival state before lining up, then retry at
        // rest because a slippery landing can settle at a different take-off point.
        const restNow = Math.hypot(bot.entity.velocity.x, bot.entity.velocity.z) < 0.01
        if (kernelTakeoff && (nextPoint as { parkour?: boolean }).parkour === true && !climbing && !swimming &&
            (bot.entity as { onGround?: boolean }).onGround === true &&
            (takeoffSearched !== nextPoint || (restNow && !takeoffSearchedAtRest))) {
          takeoffSearched = nextPoint
          takeoffSearchedAtRest = restNow
          // Mark this rest as searched so the program branch does not repeat it.
          if (restNow) restSearchedAt = { node: nextPoint, x: bot.entity.position.x, z: bot.entity.position.z }
          if (cachedLut !== null) physics.setShapeTables?.(cachedLut)
          // Prefer landing on the ladder's top to catching below it and climbing.
          let live: HopProgram | null = null
          const up = path.length > 1 ? path[1] : null
          if (up !== null && up.x === nextPoint.x && up.z === nextPoint.z && up.y === nextPoint.y + 1) {
            const feet = bot.blockAt(new Vec3(Math.floor(nextPoint.x), Math.floor(nextPoint.y + 0.001), Math.floor(nextPoint.z)), false) as BlockLike | null
            if (feet !== null && climbableIds.has(feet.type)) live = physics.hopFrom?.((up as Move).cell, TAKEOFF_SEARCH_TICKS) ?? null
          }
          if (live === null) live = physics.hopFrom?.((nextPoint as Move).cell, TAKEOFF_SEARCH_TICKS, undefined, nextLegCell()) ?? null
          takeoffNote = physics.hopNote ?? ''
          if (takeoffLive(live)) return
          if (restNow && CHAIN_LOST && (nextPoint as Move).chain === true && chainLost()) return
        }
        // The planner's diagonal reach assumes a take-off at the support's corner.
        // Creep there under sneak so the edge guard holds the overhang stance.
        let creep = false
        let takeoffHalf = 0.5
        let takeoffCap = 0
        let takeoffProj = 0
        /** Body offset from the take-off cell centre, and the per-axis limit a narrow support allows. */
        let takeoffPx = 0
        let takeoffPz = 0
        let takeoffLip = Infinity
        /** The take-off support's carrying faces (empty when unknown) and how far back along the flight line it carries the body. */
        let takeoffFaces: geometry.FaceRect[] = []
        let takeoffRear = 0
        /** A support too thin along the flight line to be worth creeping on: the jump goes from where the body stands. */
        let thinTakeoff = false
        const parkourNode = (nextPoint as { parkour?: boolean }).parkour === true
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        if (creepNode !== nextPoint) {
          creepNode = nextPoint
          creepCell = null
        }
        if (parkourNode && grounded) {
          const len = Math.sqrt(dx * dx + dz * dz)
          if (len > 0.01) {
            // Anchor creep offsets to the retired take-off cell: an overhanging body
            // can already have its centre in the next cell, which reverses a cell-relative offset.
            if (creepCell === null) {
              const anchor = prevRetired !== null && Math.hypot(prevRetired.x - p.x, prevRetired.z - p.z) <= 1.2
                ? prevRetired
                : p
              creepCell = { x: Math.floor(anchor.x), z: Math.floor(anchor.z) }
            }
            // Narrow supports need a smaller creep cap so the edge guard does not stall it.
            // Probe one cell lower too: a fence's 1.5-block top extends above its own cell.
            let half = 0.5
            let faces: geometry.FaceRect[] = []
            for (const oy of [-0.2, -1.2]) {
              const sy = Math.floor(p.y + oy)
              const sup = bot.blockAt(new Vec3(creepCell.x, sy, creepCell.z)) as BlockLike | null
              if (sup !== null && sup.shapes.length > 0) {
                half = CATCH_HALF[topCatchClass(sup.shapes)]
                faces = geometry.supportFaces(bot, creepCell.x, sy, creepCell.z)
                break
              }
            }
            takeoffHalf = half
            // Off-centre supports cannot use a lip measured from the cell centre.
            // Use the retired node's supported aim and the actual carrying faces.
            let ox = creepCell.x + 0.5
            let oz = creepCell.z + 0.5
            if (faces.length > 0 && !geometry.supportedAt(bot, faces, ox, oz) &&
                prevRetired !== null && Math.floor(prevRetired.x) === creepCell.x && Math.floor(prevRetired.z) === creepCell.z &&
                geometry.supportedAt(bot, faces, prevRetired.x, prevRetired.z)) {
              ox = prevRetired.x
              oz = prevRetired.z
            }
            // A shifted flight line measures offsets from its own take-off point.
            const shiftTo = (nextPoint as Move).takeoff
            if (shiftTo !== null && shiftTo !== undefined) {
              ox = shiftTo.x
              oz = shiftTo.z
            }
            const px = p.x - ox
            const pz = p.z - oz
            const ux = dx / len
            const uz = dz / len
            /** Along-line distance per block of the widest axis: the planner's credits are per axis. */
            const axisScale = len / Math.max(Math.abs(dx), Math.abs(dz))
            // A position packet can clear onGround and disable the sneak edge guard.
            // Keep more than one sneaking tick of support inside the physical limit.
            const physicalLip = half + 0.3 - NARROW_LIP_SAFETY
            let lipAlong = physicalLip * axisScale
            let rearAlong = physicalLip
            if (faces.length > 0) {
              const fwd = geometry.supportReach(bot, faces, ox, oz, ux, uz)
              if (fwd >= 0) lipAlong = fwd - NARROW_LIP_SAFETY * axisScale
              const back = geometry.supportReach(bot, faces, ox, oz, -ux, -uz)
              if (back >= 0) rearAlong = back / axisScale - NARROW_LIP_SAFETY
            }
            takeoffCap = Math.min(Math.min(TAKEOFF_STAND, half + TAKEOFF_NARROW_MARGIN) * axisScale, lipAlong)
            takeoffProj = (px * dx + pz * dz) / len
            takeoffPx = px
            takeoffPz = pz
            takeoffFaces = faces
            takeoffRear = rearAlong
            creep = takeoffProj < takeoffCap
            // A thin panel offers about as much creep as the body coasts afterward.
            // Jump from the supported position and skip the run-back.
            if (faces.length > 0 && lipAlong < THIN_CREEP_MIN) {
              thinTakeoff = true
              creep = false
            }
            // A flight-line projection can hide an unsupported offset on the other axis.
            // Check both axes, or the carrying faces when known, including during back-off.
            if (half < 0.5) {
              takeoffLip = physicalLip
              const off = faces.length > 0
                ? !geometry.supportedAt(bot, faces, p.x, p.z, NARROW_LIP_SAFETY)
                : Math.abs(px) >= physicalLip || Math.abs(pz) >= physicalLip
              if (off) creep = false
            }
          }
        }
        // Narrow supports need sneak throughout the approach: a walking tick can
        // cross their remaining support margin before the near-lip sneak zone applies.
        const narrowSupport = takeoffHalf < 0.5
        // Landing momentum can carry the body off a narrow support before friction stops it.
        // Brake against velocity without sneak, which weakens braking, before lining up.
        const landVel = bot.entity.velocity
        const landSpeed = Math.hypot(landVel.x, landVel.z)
        if (parkourNode && grounded && !airborne && narrowSupport && landedTicksAgo <= 3 && landSpeed > 0.1) {
          bot.look(Math.atan2(-landVel.x, -landVel.z), 0)
          bot.setControlState('forward', false)
          bot.setControlState('back', true)
          bot.setControlState('sneak', false)
          bot.setControlState('sprint', false)
          bot.setControlState('jump', false)
          execBranch = 'brake'
          futile(swimming)
          return
        }
        // Sneak near every edge: a changed flight line can put an existing
        // overhang outside the flight line's near-lip sneak zone.
        const nearEdge = Math.max(Math.abs(takeoffPx), Math.abs(takeoffPz)) >= 0.25
        const creepSneak = creep && (narrowSupport || nearEdge || takeoffProj >= takeoffCap - CREEP_SNEAK_ZONE)
        // Momentum can carry a body at the lip off the block before friction stops it.
        // Sneak holds the edge guard until another branch takes off.
        const atLip = stateMovements.allowRunUp && parkourNode && grounded && takeoffCap > 0 && takeoffProj >= takeoffCap - 0.5
        // Preserve the approved flight's heading and forward input through touchdown;
        // removing air control can leave the body short of its landing.
        const inFlight = airborne
        // A slow slip off a step is not a take-off; land under sneak
        // instead of carrying the body off its support with forward and sprint.
        const vAir = bot.entity.velocity
        const slipping = airborne && vAir.y <= 0 && Math.hypot(vAir.x, vAir.z) <= 0.1
        // A ladder catch can slide below the lowest rung before reaching the wall.
        // Sneak holds its height while forward into the wall still allows climbing.
        let caughtOnClimbable = false
        if (inFlight) {
          const feetBlock = bot.blockAt(p) as BlockLike | null
          caughtOnClimbable = feetBlock !== null && (climbableIds.has(feetBlock.type))
          // Press into the wall and counter along-wall drift so landing momentum
          // does not carry the catch out of the ladder's column.
          if (caughtOnClimbable && feetBlock !== null) {
            const wall = climbWall(feetBlock, p)
            if (wall !== null) {
              const v = bot.entity.velocity
              const ax = wall[1] !== 0 ? 1 : 0
              const az = wall[0] !== 0 ? 1 : 0
              const along = (Math.floor(p.x) + 0.5 - p.x) * ax + (Math.floor(p.z) + 0.5 - p.z) * az - (v.x * ax + v.z * az) * CLIMB_DRIFT
              let hx = wall[0]
              let hz = wall[1]
              if (Math.abs(along) > CLIMB_HOLD) {
                hx += ax * Math.sign(along)
                hz += az * Math.sign(along)
              }
              bot.look(Math.atan2(-hx, -hz), 0, true)
            }
          }
        }
        // Ice makes line-ups slide: skip backward and run-up phases,
        // and hold sneak so the edge guard keeps the body on its support.
        const onIce = grounded && (() => {
          const under = bot.blockAt(p.offset(0, -0.2, 0), false) as { name?: string } | null
          return under !== null && under.name !== undefined && ICE_NAMES.has(under.name)
        })()

        // A riser needs clearance for the jump arc to pass its lip before wall contact.
        // Use the gates' refusal to trigger back-off; a separate threshold can back off on every step.
        const stepBack = !creep && !inFlight && !climbing && !swimming && !onIce &&
          dy > STEP_UP_MIN && (nextPoint as { parkour?: boolean }).parkour !== true &&
          physics.canNudge({ back: true }, 3)

        // Run-required jumps need a sprint from the support's rear to reach the landing.
        // Back off, then run in while the gates look for a safe take-off tick.
        let runBack = false
        let runIn = false
        let runSneak = false
        if (!inFlight && !climbing && !swimming && parkourNode && grounded && takeoffCap > 0) {
          if (runUpNode !== nextPoint) {
            runUpNode = nextPoint
            runUpAttempts = 0
            runUpPhase = 0
            runUpTicks = 0
            runUpLastPos = null
          }
          // A narrow support has no safe sprint-in phase; its delayed-jump
          // gates cover the available run from the rear lip.
          const narrow = narrowSupport
          // Disabling run-up still allows one back-off, without a sprint-in phase.
          const lineUp = stateMovements.allowRunUp
          const maxAttempts = thinTakeoff || onIce ? 0 : narrow || !lineUp ? 1 : (nextPoint as Move).run ? RUN_UP_ATTEMPTS : 1
          if (runUpPhase === 0 && runUpAttempts < maxAttempts) {
            runUpAttempts++
            runUpPhase = 1
            runUpTicks = 0
            runUpLastPos = null
          }
          if (runUpPhase === 1) {
            // Stop backing at the safe rear limit or when an edge guard or wall prevents movement.
            const rear = narrow ? -takeoffRear : -RUN_UP_REAR
            // Size the stall threshold to the gait: sneak moves about 0.06 blocks
            // per tick, compared with walking's 0.2.
            const stalled = runUpLastPos !== null && runUpTicks >= 2 &&
              p.distanceTo(runUpLastPos) < (narrow ? 0.02 : WEDGE_MOVE)
            runUpLastPos = p.clone()
            const axisOut = narrow && (takeoffFaces.length > 0
              ? !geometry.supportedAt(bot, takeoffFaces, p.x, p.z, NARROW_LIP_SAFETY)
              : Math.abs(takeoffPx) >= takeoffLip || Math.abs(takeoffPz) >= takeoffLip)
            if (runUpTicks < RUN_UP_MAX_TICKS && takeoffProj > rear && !stalled && !axisOut &&
                (narrow || physics.canNudge({ back: true }, 2))) {
              runUpTicks++
              runBack = true
              runSneak = narrow
            } else {
              runUpPhase = narrow || !lineUp ? 0 : 2
              runUpTicks = 0
            }
          }
          if (runUpPhase === 2) {
            // Leave room for the gates' lookahead and brake before the lip;
            // never run onto ground the physics cannot find.
            if (runUpTicks < RUN_UP_MAX_TICKS && takeoffProj < takeoffCap - 0.35 &&
                physics.canNudge({ forward: true }, 3)) {
              runUpTicks++
              runIn = true
            } else {
              runUpPhase = 0 
            }
          }
        }
        if (runBack || runIn) creep = false

        // Aim along the flight's centre line to correct lateral error during line-up.
        // Mirror the rear aim through the body so backing converges on that line too.
        if (parkourNode && grounded && takeoffCap > 0 && creepCell !== null && (runBack || runIn || creep)) {
          const cx0 = creepCell.x + 0.5
          const cz0 = creepCell.z + 0.5
          const lx = nextPoint.x - cx0
          const lz = nextPoint.z - cz0
          const ll = Math.hypot(lx, lz)
          if (ll > 1e-6) {
            const ux = lx / ll
            const uz = lz / ll
            const foot = (p.x - cx0) * ux + (p.z - cz0) * uz
            const t = runBack ? foot - LINE_UP_AHEAD : foot + LINE_UP_AHEAD
            const ax = cx0 + ux * t
            const az = cz0 + uz * t
            const lookX = runBack ? 2 * p.x - ax : ax
            const lookZ = runBack ? 2 * p.z - az : az
            bot.look(Math.atan2(-(lookX - p.x), -(lookZ - p.z)), 0)
          }
        }

        // Ban a take-off that exhausts its line-ups without any gate approving it,
        // or the futility timer can replan to the same impossible jump.
        const attemptsDone = runUpAttempts >= (thinTakeoff ? 0 : narrowSupport || !stateMovements.allowRunUp ? 1 : (nextPoint as Move).run ? RUN_UP_ATTEMPTS : 1)
        // An edge guard holding creep still means the requested lip is beyond the support.
        const creepStalled = creep && stillTicks >= 3
        const lipWait = parkourNode && grounded && !inFlight && !runBack && !runIn && !stepBack &&
          (!creep || creepStalled) && takeoffCap > 0 && runUpPhase === 0 && attemptsDone
        lipWaitTicks = lipWait ? lipWaitTicks + 1 : 0
        if (lipWaitTicks > LIP_WAIT_TICKS) {
          lipWaitTicks = 0
          banTakeoff(nextPoint)
          execBranch = 'giveup'
          resetPath('stuck', true)
          return
        }

        bot.setControlState('forward', creep || (inFlight && !slipping) || runIn)
        bot.setControlState('back', stepBack || runBack)
        bot.setControlState('sneak', (creep && creepSneak) || caughtOnClimbable || runSneak || (atLip && !runIn) || (onIce && !inFlight) || (slipping && !caughtOnClimbable))
        bot.setControlState('sprint', (flyingParkour && !slipping && !caughtOnClimbable) || runIn)
        execBranch = caughtOnClimbable ? 'catch' : inFlight ? 'inflight' : runBack ? 'runback' : runIn ? 'runin' : creep ? 'creep' : stepBack ? 'stepback' : 'wait'
        // Waiting or creeping must stay supported through the next tick's coast.
        // Back-off phases continue pressing next tick, so they use their own checks.
        if (!inFlight && !caughtOnClimbable && !runIn && !runBack && !stepBack && holdSupport()) execBranch = 'hold'
      }

      futile(swimming)
    }

    // Count before movement monitoring so the teleport check sees the physics tick gap.
    listen('physicsTick', () => { physTicks++ })
    listen('physicsTick', monitorMovement)

    // Sneak on slime touchdown to suppress an unplanned rebound.
    // Planned bounces and program replays control their own landings.
    const slimeLand = { slimeLand: true, tx: 0, ty: 0, tz: 0 } as HopProgram
    const SLIME_KIND = simKindOf('slime_block', false)
    listen('physicsTick', () => {
      if (path.length === 0 || stateGoal === null || interrupts.paused) return
      const ent = bot.entity as { onGround?: boolean, position: Vec3, velocity: Vec3 }
      const vy = ent.velocity.y
      const n0 = path[0] as Move
      if (n0.via !== null && n0.chain !== true) return
      if (progNode === n0 && progPhase === 1) return
      const p = ent.position
      // Hold sneak on slime between jumps: small rebounds clear onGround,
      // disabling the edge guard while the body still has landing momentum.
      if (ent.onGround === true) {
        if ((n0 as { parkour?: boolean }).parkour !== true || (bot as { getControlState?: (c: string) => boolean }).getControlState?.('jump') === true) return
        const under = bot.blockAt(p.offset(0, -0.2, 0)) as { name?: string } | null
        if (under !== null && under.name === 'slime_block') bot.setControlState('sneak', true)
        return
      }
      if (vy >= 0) return
      const top = Math.floor(p.y + 1e-9)
      if (p.y + vy > top + 1e-9) return 
      // A deep fall needs the slime rebound to avoid damage; sneak cancels it.
      const health = (bot as unknown as { health?: number }).health ?? 20
      const dmg = fallDamagePoints(fallen + (p.y - top), p.y - top, SLIME_KIND, true, wholeFall)
      if (dmg > 0 && dmg >= Math.max(1, Math.min(SLIME_SNEAK_MAX_DAMAGE, health - 2))) {
        bot.setControlState('sneak', false)
        return
      }
      const hw = (bot.physics as unknown as { playerHalfWidth?: number }).playerHalfWidth ?? 0.3
      for (let x = Math.floor(p.x - hw); x <= Math.floor(p.x + hw); x++) {
        for (let z = Math.floor(p.z - hw); z <= Math.floor(p.z + hw); z++) {
          const under = bot.blockAt(new Vec3(x, top - 1, z)) as { name?: string } | null
          if (under === null || under.name !== 'slime_block') continue
          slimeLand.tx = x
          slimeLand.ty = top
          slimeLand.tz = z
          if (sneakLanding(slimeLand, false, p.x, p.y, p.z, vy, hw)) {
            bot.setControlState('sneak', true)
            return
          }
        }
      }
    })
    const client = (bot as unknown as { _client?: { on: (ev: string, fn: (p: Record<string, unknown>) => void) => void, removeListener?: (ev: string, fn: (p: Record<string, unknown>) => void) => void } })._client
    const onPacket = (ev: string, listener: (p: Record<string, unknown>) => void): void => {
      client?.on(ev, listener)
      disposers.push(() => { client?.removeListener?.(ev, listener) })
    }
    // lpVec3 velocity packets carry blocks per tick, so apply them directly
    // instead of mineflayer's fixed-point scaling to preserve the server's motion.
    const motionType = (bot.registry as unknown as { protocol?: { play?: { toClient?: { types?: Record<string, unknown> } } } })
      .protocol?.play?.toClient?.types?.packet_entity_velocity
    const motionExact = JSON.stringify(motionType ?? null).includes('lpVec3')
    onPacket('entity_velocity', (packet) => {
      const self = bot.entity as { id?: number, velocity: Vec3 } | undefined
      if (self === undefined || packet.entityId !== self.id) return
      const v = packet.velocity as { x: number, y: number, z: number } | undefined
      if (motionExact && v !== undefined) self.velocity.set(v.x, v.y, v.z)
      hurtSeen = true
      // Damage on an exempt landing indicates the server uses the older fall rule.
      if (spared > 0) {
        wholeFall = false
        spared = 0
      }
    })
    if (execTraceFile !== null) {
      // Packet traces distinguish physics decisions from server motion corrections.
      for (const ev of ['entity_velocity', 'sync_entity_position', 'position']) {
        onPacket(ev, (packet) => {
          if (ev !== 'position' && packet.entityId !== (bot.entity as { id?: number }).id) return
          execTraceBuf.push(JSON.stringify([execTick, 'packet', ev, packet.velocity ?? [packet.dx, packet.dy, packet.dz], packet.x ?? null, packet.y ?? null, packet.z ?? null, packet.flags ?? null]))
        })
      }
      listen('physicsTick', () => {
        execTick++
        const p = bot.entity.position
        const v = bot.entity.velocity
        const c = bot.controlState as unknown as Record<string, boolean>
        const n = path[0]
        execTraceBuf.push(JSON.stringify([
          execTick, +p.x.toFixed(2), +p.y.toFixed(2), +p.z.toFixed(2),
          (bot.entity as { onGround?: boolean }).onGround === true ? 1 : 0,
          execBranch, c.forward ? 1 : 0, c.sprint ? 1 : 0, c.jump ? 1 : 0, c.sneak ? 1 : 0, c.back ? 1 : 0,
          cutTarget !== null ? +Math.hypot(cutTarget.x - p.x, cutTarget.z - p.z).toFixed(1) : 0, path.length,
          n ? [+n.x.toFixed(3), +n.y.toFixed(3), +n.z.toFixed(3), (n as { parkour?: boolean }).parkour ? 1 : 0] : null,
          [+v.x.toFixed(3), +v.y.toFixed(3), +v.z.toFixed(3)],
          +lastTickMs.toFixed(2),
          takeoffNote !== ''
            ? takeoffNote
            : execBranch === 'sprint' || execBranch === 'hop' || execBranch === 'krun'
            ? `${(physics as { hopRefusal?: string }).hopRefusal ?? ''}${(physics as { hopJumpNote?: string }).hopJumpNote ? ':' + (physics as { hopJumpNote?: string }).hopJumpNote : ''}`
            : ''
        ]))
        takeoffNote = ''
        execBranch = 'idle'
        // Flush idle ticks so arrival data is not left buffered when the process exits.
        if (execTraceBuf.length >= 40 || (execTraceBuf.length > 0 && path.length === 0)) {
          fs.appendFileSync(execTraceFile, execTraceBuf.join('\n') + '\n')
          execTraceBuf.length = 0
        }
      })
    }

    let disposed = false
    pf.dispose = () => {
      if (disposed) return
      disposed = true
      if (ended === null) ended = 'disposed'
      interrupts.close('The pathfinder was disposed.')
      if (goalActive()) {
        stopReason = 'disposed'
        internalStop()
      }
      lifecycle.abort(new Error('The pathfinder was disposed.'))
      // A dig being aimed may never send the events that would remove these.
      forgetDigListeners()
      digging = false
      // No tick will finish a flight's coast now.
      if (coastHold) {
        coastHold = false
        bot.clearControlStates()
      }
      for (const off of disposers) off()
      disposers.length = 0
      restoreHitbox()
      delete (bot as unknown as { pathfinder?: unknown }).pathfinder
    }
  }
}

/** The plugin instance for `bot.loadPlugin(pathfinder)`. */
export const pathfinder = createPathfinder()
