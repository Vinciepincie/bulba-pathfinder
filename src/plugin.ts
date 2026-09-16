// The bot.pathfinder plugin: API-compatible with mineflayer-pathfinder
// (same methods, fields, events, error names and tick-loop semantics), with
// the A* solve moved onto a snapshot + worker thread. Path FOLLOWING is an
// exact port of upstream's monitorMovement (minus dig/place, which this
// package never does), driven by prismarine-physics simulation like
// upstream, so the walking behavior is unchanged.
//
// Improvements (all off by default or strictly additive — see README):
//   - solves run off the event loop (worker_thread), interim partial paths
//     stream back so the bot walks while thinking
//   - keepPathDuringRecompute: keep following the still-valid path while a
//     recompute (goal moved / world changed) is in flight
//   - stuckTimeout / executionTimeout knobs (default off)
//   - stop() releases controls within a tick and never wedges a later goal
import { performance } from 'node:perf_hooks'
import fs from 'node:fs'
import { Vec3 } from 'vec3'
import nbt from 'prismarine-nbt'
import type { Bot } from 'mineflayer'
import { Movements } from './movements.js'
import { Move } from './move.js'
import { Goal } from './goals.js'
import { PhysicsSim } from './physics.js'
import { Solver } from './solver.js'
import type { RawSolveResult } from './solver.js'
import { GoalAdapter } from './goalAdapter.js'
import { getLut } from './lut.js'
import type { BlockLut } from './lut.js'
import { computeDigData, digFingerprint } from './digData.js'
import { fastEvaluator } from './fastEvaluator.js'
import type { DigContext } from './moveGen.js'
import type { DigData, MovementsConfig } from './types.js'
import {
  Snapshot, buildSnapshot, computeBox, applySnapshotBlockUpdate, bakeEntityIndex, nextSnapshotGeneration
} from './snapshot.js'
import type { Box } from './snapshot.js'
import { serializeGoal, goalNeedsRaycast, descriptorTargets } from './goalSerde.js'
import { TAKEOFF_STAND, TAKEOFF_NARROW_MARGIN, CHAIN_TAKEOFF_FRACTION } from './parkourEnvelope.js'
import { CATCH_HALF, topCatchClass } from './shapes.js'
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
import { resolveBlock, toVec3, inReach } from './actions/reach.js'
import { GoalLookAtBlock, GoalNear } from './goals.js'
import type { PathfinderOptions, GoalDescriptor, PhysicsLike, GotoOptions } from './types.js'

// Upstream lib/lock.js.
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
  status: string
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

/**
 * How long server position corrections may keep excusing a lack of progress.
 * Generous enough to cover a real lagback burst, short enough that a bot
 * wedged in geometry still trips the 3.5 s futility check.
 */
const FORCED_MOVE_GRACE_MS = 3000

/**
 * How far from a wall the body has to be for sprinting to be safe. Any
 * positive margin makes exact contact — what prismarine-physics leaves after
 * every horizontal collision — count as "against the wall".
 */
const SPRINT_WALL_MARGIN = 0.03

/**
 * A walking step is at most a diagonal; anything longer is a jump and must
 * not be re-steered along a wall.
 */
const WALK_STEP_REACH = 1.8

/** How far ahead the wall-slide probe looks — over one tick of sprinting. */
const SLIDE_PROBE = 0.35

/**
 * Sideways probe for a body FLUSH with a wall it is walking along. Slightly
 * more than SPRINT_WALL_MARGIN plus playerCollides' own inset, so the probe
 * fires on exactly the contact the sprint gate refuses.
 */
const SIDE_TOUCH = 0.06

/**
 * Consecutive airborne ticks before "not on the ground" is believed. Both
 * mineflayer (after any server position packet) and prismarine-physics (on a
 * post's edge) report onGround=false for a single tick while the body is
 * standing still; a jump or a fall shows in the velocity or the height on
 * that same tick, a flicker does not.
 */
const AIRBORNE_TICKS = 2

/**
 * Steering bias AWAY from a wall we are sliding along, as a fraction of the
 * along-wall component. Sliding with a heading exactly parallel is not enough:
 * whatever momentum the bot still carries into the wall gets clamped, and a
 * clamped position sits EXACTLY on the block face — the one value the server
 * disagrees with us about, because prismarine-physics reconstructs the body
 * from `minZ + halfWidth` while the server reconstructs it from the centre,
 * and the two round to opposite sides of the boundary (the 1.21 hitbox
 * precision class again — see geometry.ts). A hair of standoff keeps every
 * position we claim unambiguously outside the block.
 */
const WALL_STANDOFF = 0.25

/**
 * Above this, the next node is something to climb ONTO, not a wall to walk
 * around: a shelf, a step, a ladder exit. Steering away from it there is how a
 * wall-slide turns a one-block step-up into an unreachable node.
 */
const STEP_UP_MIN = 0.1
/**
 * Run-up line-up (the creep branch). A jump the planner flagged META_RUN is
 * given RUN_UP_ATTEMPTS lines-ups (any other parkour node one); each backs
 * off for at most RUN_UP_MAX_TICKS to RUN_UP_REAR blocks behind the take-off
 * cell's centre (a post: its own rear lip), then sprints in for at most
 * RUN_UP_MAX_TICKS more while the gates look for the take-off tick.
 */
const RUN_UP_ATTEMPTS = 2
const RUN_UP_MAX_TICKS = 12
const RUN_UP_REAR = 1.5
/**
 * Pure-pursuit distance for the run-up line-up: the body steers at a point
 * this far up the take-off CENTRE LINE from its own foot (mirrored through
 * the body when backing, since `back` moves away from the look point), so a
 * run-back and run-in converge onto the line the jump was priced on instead
 * of running parallel to it. Traced on the arena's basic3: the body lined up
 * 0.39 off the line and no run-in ever produced a take-off the rollouts
 * would sign, although the same jump is signed from on the line.
 */
const LINE_UP_AHEAD = 1.0
/**
 * Grounded ticks waiting at a lip with the line-ups exhausted and every gate
 * refusing before the take-off is given up: the landing cell is banned
 * (BAN_MS, BAN_WEIGHT) and the route re-solved round it. The futility timer
 * alone brought back the same plan and the same jump for the rest of the run
 * (basic3: three replans, 24 s against 12 on the plan that avoids it).
 */
const LIP_WAIT_TICKS = 30
const BAN_MS = 120_000
const BAN_WEIGHT = 200

/**
 * The creep sneaks only this close to its cap and WALKS the rest: sneaking
 * is 1.3 b/s against 4.3, and the edge guard is only needed near the lip. A
 * walking body a third of a block short of the cap cannot leave the block in
 * the one tick before sneak engages (0.22 per tick at most, the cap is 0.1
 * past the edge and the box half-width 0.3 beyond that).
 */
const CREEP_SNEAK_ZONE = 0.35

/**
 * How far inside a narrow support's physical overhang limit (half-width +
 * 0.30) the executor keeps the body's centre when creeping to, or backing to,
 * its lip. A sneaking tick moves at most ~0.05, so one tick without the
 * edge-guard (a server position packet clears onGround) cannot leave it.
 */
const NARROW_LIP_SAFETY = 0.08

/** Ground distance one sprinting tick covers, the sprint gate's look-ahead. */
const SPRINT_TICK = 0.3

/**
 * Arrival box height while sprint-hopping. The hop apex is 1.25 above the
 * take-off, so upstream's |dy| < 1 would fly the bot over a node without
 * consuming it — and a node left behind turns the bot round.
 */
const HOP_ARRIVE_DY = 1.45

/**
 * Cosine of the largest angle between the body's velocity and the commanded
 * heading at which a sprint-hop may still take off (20°). See the take-off
 * decision in the sprint branch: a hop into a sharper turn flies the OLD
 * heading and lands off the line. PF_HOP_TURN_DEG overrides it for A/B runs.
 */
const HOP_TURN_COS = Math.cos((Number(process.env.PF_HOP_TURN_DEG ?? 20) || 20) * Math.PI / 180)

/**
 * Parkour nodes this close ahead switch the corner cut off. A cut ends with
 * the body lined up on the CHORD, which is not the line the take-off wants;
 * measured on the two jump-dense routes, cutting into a jump made both of
 * them walk further and finish later (simple3 +0.32 s, basic1 +0.37 s).
 */
const CUT_JUMP_GUARD = Number(process.env.PF_CUT_JUMP_GUARD ?? 5) || 5

/**
 * Nodes scanned on the tick a cut is (re)picked. The line is then EXTENDED
 * by up to CUT_EXTEND nodes a tick while the body walks it, so the target
 * runs out to the visibility horizon within a few ticks without any one tick
 * paying for the whole scan. Extension only ever moves the aim further along
 * the same path, so the heading turns once, onto the line, and never flicks.
 */
const CUT_SCAN_INIT = 8
const CUT_EXTEND = 4

/**
 * Consecutive candidates whose chord the body does not fit down before a
 * scan stops. Visibility is not monotonic in path order: a corner can mask
 * the nearer node and clear the further one.
 */
const CUT_MISSES = 2

/**
 * Longest chord the cut steers down, in blocks. Past this the heading is
 * already the straight line to within a degree, and every re-validation
 * costs a sweep of this length per tick.
 */
const CUT_MAX_CHORD = 32

/**
 * Spacing of the synthetic nodes the physics gates are shown along a live
 * chord (gatePath in the tick loop): the block-a-node density the rollouts
 * and the hop score were tuned on, laid along the line actually being walked
 * instead of the zig-zag being skipped.
 */
const CUT_GATE_STEP = 1.0

/**
 * How far ahead of the body's foot on the cut LINE the steering point sits
 * (pure pursuit). The line is fixed when the target is picked and the anchor
 * slides along it; steering at a point 1.5 blocks up the line pulls a body
 * knocked sideways back onto it at a strong angle, where steering straight at
 * a target 30 blocks out barely turns for a block of error. That feedback is
 * what the physics rollouts assume — they re-aim at a node every block — and
 * without it a hop taken into a turn carried its old momentum 1.4 blocks off
 * the line and off a ledge (arena simple1, the first taut-cut run).
 */
const CUT_CARROT = Number(process.env.PF_CUT_CARROT ?? 1.5) || 1.5

/** Lateral error from the line past which the cut is dropped and re-picked from where the body is. */
const CUT_MAX_OFF = 0.6

/**
 * Ground required either side of a NEW line, in blocks, over its first
 * CUT_MARGIN_LEN blocks — OFF by default, kept as an A/B knob
 * (PF_CUT_SIDE_MARGIN / PF_CUT_MARGIN_LEN).
 *
 * It was the first answer to the simple1 fall (a hop into a turn drifting
 * off a one-wide ledge): refuse a line that skims an edge where the body is
 * still turning onto it. Measured on the arena it is the wrong answer. 2b2t
 * spawn is one-wide ledges and holes, and a half-block margin — even held
 * only for the first five blocks of a line and never on re-validation —
 * refused most lines: simple2 14.46 s against 13.87 s without it, simple3
 * 8.73 against 8.15, basic2 17.50 against 16.16. Pure pursuit (CUT_CARROT)
 * alone removed the fall (simple1 10.69 s, from 19.9 s falling and 11.08 s
 * on the old cut), because the fall was a steering problem, not a geometry
 * one. Applied, when on, at pick and extension only.
 */
const CUT_SIDE_MARGIN = Number(process.env.PF_CUT_SIDE_MARGIN ?? 0)
const CUT_MARGIN_LEN = Number(process.env.PF_CUT_MARGIN_LEN ?? 5) || 5

/**
 * A server correction that moved the body no further than this, with a cut
 * live, keeps the cut. Under a cut the body is legitimately off every node
 * (the chord runs across the zig-zag, up to a few blocks from its corner), so
 * the lagback handler's node-distance test would read a routine sub-block
 * correction as "snapped away" and replan the whole route.
 */
const CUT_LAG_NEAR = 2

/**
 * Under a live cut the wall-slide probes only the axes the chord actually
 * moves along: a component below this fraction of the heading is a line
 * running a few degrees off a wall, not into it.
 */
const SLIDE_AXIS_MIN = 0.2

/**
 * How far ABOVE a parkour node the body may be and still retire it in the
 * air. Roughly one tick of fall: at that height the landing is committed, so
 * holding the node buys nothing and costs the momentum the next move wants.
 */
const PARKOUR_LAND_DY = 0.5
/**
 * Fall (apex to landing, blocks) past which vanilla deals damage — and, with
 * the damage, sends the client an entity_velocity packet that overwrites its
 * motion with the server's: horizontal ZERO. Traced on the arena's
 * parkouradv1 (bench/arena/.run/scratch/exec3-*.jsonl): the bot landed a
 * 4.25-block drop on a fence post, the rollout approved the next take-off on
 * the landing tick with the landing speed, the packet arrived before the
 * next physics tick, and the jump left from rest and fell short. So after a
 * damaging landing the executor sits out the tick (plus the ping) the packet
 * needs, and decides from the velocity it actually has.
 */
const FALL_DAMAGE_DISTANCE = 3

/**
 * Air (of mineflayer's 0..20) below which the executor abandons the node and
 * surfaces. Vanilla's lung is 300 ticks and refills at 4 air-ticks per tick,
 * so a third of it is comfortably more than the time to swim up out of
 * anything the planner can legally route through.
 */
const AIR_RESERVE = 7

/** Movement below this, over a tick, is jitter rather than progress. */
const WEDGE_MOVE = 0.1

/**
 * Ticks of jitter-only movement before the executor stops believing its own
 * rollouts and starts trying escapes. Long enough to sit through a sharp turn
 * (bot.look interpolates) and a landing, short enough to save the run.
 */
const WEDGE_TICKS = 12

/**
 * Consecutive ticks one escape is given before the next is tried. An escape
 * that works stops being needed within a tick or two — the gates approve and
 * the recovery is not called again — and one that is not working has had its
 * chance.
 */
const ESCAPE_PATIENCE = 6

/**
 * Ticks an angled take-off is driven before the node is handed to the blind
 * escapes. A jump leaves the ground on the first tick, so anything past a
 * jump's worth of them means the angle was not the problem.
 */
const ANGLE_PATIENCE = 14

/**
 * Escapes, cheapest first. Cycled per node: an escape that did not help is
 * not the one tried next time the same node wedges. Each is simulated before
 * use (physics.canNudge), so none of them can be the fall it was avoiding.
 */
const ESCAPES: Array<{ back?: boolean, left?: boolean, right?: boolean, forward?: boolean, jump?: boolean }> = [
  { back: true },
  { left: true },
  { right: true }
]

const DEFAULT_OPTIONS: Required<Omit<PathfinderOptions, 'physicsFactory' | 'onNoPath'>> = {
  useWorkerThreads: true,
  workerEntryPath: '',
  maxSnapshotCells: 8_000_000,
  hitboxPrecisionFix: true
}

/**
 * Create the plugin function. `pathfinder` (the default instance) is what
 * `bot.loadPlugin(pathfinder)` takes, exactly like upstream.
 */
export function createPathfinder (options: PathfinderOptions = {}) {
  const opts = { ...DEFAULT_OPTIONS, ...options }

  return function inject (bot: Bot): void {
    const registry = bot.registry
    const waterType: number = registry.blocksByName.water.id
    const ladderId: number = registry.blocksByName.ladder.id
    const vineId: number = registry.blocksByName.vine.id
    const bubbleColumnId: number = registry.blocksByName.bubble_column ? registry.blocksByName.bubble_column.id : -1

    let stateMovements = new Movements(bot)
    let stateGoal: Goal | null = null
    let dynamicGoal = false
    let path: Move[] = []
    let pathUpdated = false
    // Last dispatched solve's inputs, for the onNoPath capture hook.
    let lastSolveStart: { x: number, y: number, z: number } = { x: 0, y: 0, z: 0 }
    /** Block the last solve was dispatched from — see `startSolve` and the
     *  exhausted-path recovery in the tick handler. */
    let lastSolveFrom: { x: number, y: number, z: number } | null = null
    let lastSolveGoal: unknown = null
    let lastSolveCfg: MovementsConfig | null = null
    let placing = false
    let placingBlock: { x: number, y: number, z: number, useOne?: boolean } | null = null
    let digging = false
    /** Sprint-hop latched at take-off, re-pressed on each landing (allowSprintHop). */
    let hopHold = false
    /** The node the corner cut is steering at, held until reached or invalid. */
    let cutTarget: Move | null = null
    /** Anchor of the cut line (slides along it each grounded tick); the line is cutFrom → cutTarget. */
    let cutFrom: Vec3 | null = null
    /** Body position at the last tick, for the lagback handler's cut-aware near test (CUT_LAG_NEAR). */
    let prevTickPos: Vec3 | null = null
    // Wedge recovery state — see the tick loop's recovery block.
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
    /** Cell the parkour corner-creep started from (see the creep branch). */
    let creepCell: { x: number, z: number } | null = null
    /** Slime-bounce phase, latched per node: has the rebound started? */
    let bounceNode: Move | null = null
    let bounceRisen = false
    /** Momentum chain, latched per node: has the re-jump left the stone? */
    let chainNode: Move | null = null
    let chainFired = false
    /** Run-up back-off, one attempt per parkour node (see the creep branch). */
    /** Run-up line-up state (see the creep branch): node, attempts used, phase (0 idle, 1 backing, 2 sprinting in), ticks in phase. */
    let runUpNode: Move | null = null
    let runUpAttempts = 0
    let runUpPhase = 0
    let runUpTicks = 0
    let runUpLastPos: Vec3 | null = null
    /** Consecutive grounded ticks spent waiting at a lip with nothing left to try (LIP_WAIT_TICKS). */
    let lipWaitTicks = 0
    /** Landing cells whose take-off could not be performed: "x,y,z" → expiry (performance.now ms). */
    const bannedCells = new Map<string, number>()
    /** The parkour node `creepCell` was latched for. */
    let creepNode: Move | null = null
    /** Highest point of the current flight, for the fall-damage settle (FALL_DAMAGE_DISTANCE). */
    let airPeakY = -Infinity
    let wasAirborne = false
    let landSettle = 0
    /** Consecutive ticks with onGround=false (see AIRBORNE_TICKS). */
    let airTicks = 0
    let prevTickY = 0
    /** Grounded ticks since the last touchdown (large while airborne); read by the arrival loop. */
    let landedTicksAgo = 99
    let lastTickGrounded = false
    /** The node most recently retired by the arrival loop — the take-off of the next parkour node. */
    let prevRetired: { x: number, y: number, z: number } | null = null
    /**
     * Decision trace (PF_EXEC_TRACE=<file prefix>): which branch of the tick
     * loop drove this tick. Written by a second physicsTick listener so every
     * early return is covered; off by default, costs nothing when unset.
     */
    let execBranch = 'idle'
    let execTick = 0
    const execTraceFile = process.env.PF_EXEC_TRACE ? `${process.env.PF_EXEC_TRACE}-${bot.username}.jsonl` : null
    const execTraceBuf: string[] = []
    let lastNodeTime = performance.now()
    /**
     * When a path node was last genuinely REACHED. `lastNodeTime` is also
     * pushed forward by server position corrections; this one never is, so it
     * can bound how long those corrections are allowed to excuse a lack of
     * progress. See the forcedMove handler.
     */
    let lastNodeArrival = performance.now()
    let goalSetTime = performance.now()
    let stopPathing = false
    /**
     * When the server last corrected our position. The action layer refuses
     * to send a click inside the correction window — a swing from a position
     * the server has already rejected is a wasted packet at best.
     */
    let lastForcedMoveAt = 0

    const physics = options.physicsFactory ? options.physicsFactory(bot) : new PhysicsSim(bot)
    const lockUseBlock = new Lock()
    // ── 1.21.x exact-boundary collision bug (hitboxPrecisionFix) ─────────
    //
    // prismarine-physics builds the body from playerHalfWidth 0.3 and
    // playerHeight 1.8, and every collision resolution leaves the box exactly
    // on a block boundary. On 1.21.x the server's own sweep computes exactly
    // 1.0 for such a move, calls it blocked, and teleports the client back —
    // silently, every tick, for as long as the bot keeps producing that
    // position. It is a physics bug, not a pathfinding one, but it presents
    // as a pathfinder that cannot climb: the bot welds itself to the face of
    // a one-block step and never gets off it.
    //
    // MEASURED on the arena's climb1 riser, walking jump, same controls, only
    // the starting clearance changed:
    //
    // | clearance | stock hitbox    | nudged hitbox |
    // | --------- | --------------- | ------------- |
    // | 0.01      | stuck, 19 corr. | CLIMBED, 0    |
    // | 0.15      | stuck, 17 corr. | CLIMBED, 0    |
    // | 0.20      | CLIMBED, 0      | CLIMBED, 0    |
    // | 0.30 spr. | stuck, 7 corr.  | CLIMBED, 0    |
    //
    // Nudging the dimensions by 1e-5 breaks the alignment and the whole class
    // goes away — 64 corrections across the probe instead of 174. Same fix as
    // PrismarineJS/mineflayer#3911 and mineflayer-pathfinder#364; applied
    // here because this package is what notices. Guarded on the exact stock
    // values so an application that already applies it is not doubled up.
    if (opts.hitboxPrecisionFix) {
      const ph = bot.physics as unknown as { playerHalfWidth?: number, playerHeight?: number }
      if (ph) {
        if (ph.playerHalfWidth === 0.3) ph.playerHalfWidth = 0.30001
        if (ph.playerHeight === 1.8) ph.playerHeight = 1.80001
      }
    }

    const workerHost = getSharedWorkerHost()
    if (opts.workerEntryPath) workerHost.setEntryPath(opts.workerEntryPath)
    // Pay the thread + wasm start-up while the bot is idle, not inside its
    // first goal. See SolverWorkerHost.prewarm.
    if (opts.useWorkerThreads) workerHost.prewarm()

    // ── async solve state ────────────────────────────────────────────────
    let solveGeneration = 0
    // PF_SOLVE_TIMING=1: per-solve latency breakdown on stdout (snapshot
    // build, engine time, wall from dispatch to path install, ticks to the
    // first controls). Diagnostic only; costs nothing when unset.
    const solveTimingOn = process.env.PF_SOLVE_TIMING === '1'
    let solveDispatchedAt = 0
    let goalSetAt = 0
    let firstDriveLogged = false
    /**
     * Eager start (improvement): a goal starts solving the moment it is set,
     * and the first path drives the bot the moment it lands, instead of each
     * waiting for the next physicsTick. mineflayer runs the simulation and
     * THEN emits physicsTick, so controls written between ticks are used by
     * the very next simulation — a path installed between ticks that waits
     * for the tick handler loses a whole tick, and setGoal waiting for a tick
     * to dispatch loses up to another. Upstream computes in-tick and walks in
     * the same tick; measured on the arena, our first move trailed upstream's
     * by 20-150 ms on every route despite faster solves. PF_NO_EAGER=1
     * restores the tick-aligned behaviour for A/B runs.
     */
    const eagerStart = process.env.PF_NO_EAGER !== '1'
    /** True while monitorMovement is running (reentrancy guard for the eager drive). */
    let inTick = false
    /** Wall time of the last tick handler, for the exec trace. */
    let lastTickMs = 0
    /** Diagonal squeeze on a refused flat step, before the jump gates (see the squeeze block). PF_NO_EARLY_SQUEEZE=1 for A/B. */
    const earlySqueeze = process.env.PF_NO_EARLY_SQUEEZE !== '1'
    let activeSolveCancel: (() => void) | null = null
    let activeSolveGeneration = -1
    let mainSolver: Solver | null = null
    let growFactor = 1
    let growAttempts = 0
    /** When the current growth sequence's first solve was dispatched. */
    let growStartedAt = 0
    let lastTouchedChunks: Set<string> | null = null

    // ── snapshot cache ───────────────────────────────────────────────────
    let cachedSnapshot: Snapshot | null = null
    let cachedLut: BlockLut | null = null
    let snapshotStale = true
    const pendingPatches: Array<{ x: number, y: number, z: number, stateId: number }> = []

    const pf: Record<string, unknown> = {}
    ;(bot as unknown as { pathfinder: typeof pf }).pathfinder = pf

    pf.thinkTimeout = 5000 // ms
    pf.tickTimeout = 40 // ms, amount of thinking per tick (parity; also worker partial cadence)
    pf.searchRadius = -1 // cost slack, -1: don't limit the search
    pf.enablePathShortcut = false
    pf.LOSWhenPlacingBlocks = true
    // Improvements, default off / additive:
    pf.stuckTimeout = -1 // ms without reaching the next node → path_stop (goto rejects PathStopped)
    pf.executionTimeout = -1 // ms per goal overall → path_stop
    pf.keepPathDuringRecompute = true // keep walking the valid prefix while replanning

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

    // ── absorbed wrapper scaffolding (additive; upstream has no equivalents) ──
    pf.isStuck = () => geometry.isStuck(bot)
    pf.canStandAt = (pos: Vec3) => geometry.canStandAt(bot, pos)
    pf.findEscape = (radius?: number) => geometry.findEscape(bot, radius)
    pf.findNearestOpenStandable = (radius?: number) => geometry.findNearestOpenStandable(bot, radius)
    pf.unstick = async (): Promise<boolean> => {
      // Stop driving first — an active goal's tick loop would fight the
      // escape controls every tick (the wrapper's unstick stopped first too).
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      return geometry.unstick(bot)
    }
    pf.centreInCell = (cell: Vec3, tolerance?: number) => geometry.centreInCell(bot, cell, tolerance)
    pf.jumpOnce = () => geometry.jumpOnce(bot)
    pf.settle = async (): Promise<boolean> => {
      // Come to a verified standstill: drop any goal (listeners see the
      // upstream goal_updated/GoalChanged contract), release controls, then
      // wait for drift to actually end; walk out if embedded.
      if (stateGoal || path.length > 0 || solveInFlight()) {
        (pf.setGoal as (g: Goal | null) => void)(null)
      }
      bot.clearControlStates()
      if (geometry.isStuck(bot)) await geometry.unstick(bot)
      return geometry.settle(bot)
    }

    // ── cooperative interrupts ───────────────────────────────────────────
    //
    // The hand-over point another plugin waits for. `isSafe` is the whole
    // contract: feet on something, nothing half-finished. A request that
    // arrives while the bot is standing on a block is granted on the same
    // tick, which is why "wait before jumping" needs no special case — the
    // executor yields before it ever reaches the take-off decision.
    const interrupts = new InterruptController(bot as unknown as { emit: (e: string, ...a: unknown[]) => boolean }, {
      isSafe: () => {
        if (digging || placing) return false
        const e = bot.entity as { onGround?: boolean, isInWater?: boolean }
        if (e.onGround === true || e.isInWater === true) return true
        // On a ladder or vine the bot is not "on the ground" but is not
        // falling either; stopping there is harmless.
        const feet = bot.blockAt(bot.entity.position, false) as { type: number } | null
        return feet !== null && (feet.type === ladderId || feet.type === vineId)
      },
      onPause: () => {
        fullStop()
        clearWedge()
      },
      onResume: (pausedMs: number) => {
        // Standing still because someone asked us to is not a stall. Push the
        // futility and stuck timers forward by exactly the pause, or a long
        // eat would look identical to a bot wedged in geometry.
        lastNodeTime += pausedMs
        lastNodeArrival += pausedMs
        goalSetTime += pausedMs
        clearWedge()
      }
    })

    pf.interrupt = async (reason: string, options?: InterruptOptions): Promise<InterruptHandle> =>
      await interrupts.acquire(reason, options)
    pf.withInterrupt = async <T>(reason: string, fn: () => Promise<T>, options?: InterruptOptions): Promise<T> =>
      await interrupts.run(reason, fn, options)

    /** What the executor is doing, for anything that needs to time around it. */
    function motionPhase (): MotionPhase {
      if (interrupts.paused) return 'paused'
      if (digging) return 'digging'
      if (placing) return 'interacting'
      if (path.length === 0) return 'idle'
      const e = bot.entity as { onGround?: boolean, isInWater?: boolean }
      if (e.isInWater === true) return 'swimming'
      if (e.onGround !== true) {
        const feet = bot.blockAt(bot.entity.position, false) as { type: number } | null
        if (feet && (feet.type === ladderId || feet.type === vineId)) return 'climbing'
        return 'airborne'
      }
      return 'walking'
    }

    pf.motion = {
      get phase (): MotionPhase { return motionPhase() },
      /**
       * True when taking the controls away right now would break the run —
       * mid-flight over a gap, or half-way through a dig or an interaction.
       * Callers that can wait should call `interrupt()` instead of polling
       * this: it does the waiting correctly.
       */
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

    // ── the interaction table ────────────────────────────────────────────
    //
    // Captured BEFORE any application wrapper monkey-patches bot.dig, so an
    // application whose own dig wrapper delegates here cannot recurse.
    const rawDig = (bot as unknown as DiggingBot).dig.bind(bot)
    const pacer = new Pacer()

    async function approachBlock (pos: Vec3, reach: number, signal?: AbortSignal): Promise<boolean> {
      if (inReach(bot as unknown as never, pos, reach)) return true
      if (signal?.aborted === true) return false
      // Release the action's interrupt for the walk — the executor cannot
      // drive while it is held, and re-take it once we have arrived.
      const session = activeSession
      if (session?.handle) {
        session.handle.release()
        session.handle = null
      }
      let arrived = false
      try {
        await gotoImpl(new GoalLookAtBlock(pos, bot.world as never, { reach }))
        arrived = true
      } catch {
        try {
          await gotoImpl(new GoalNear(pos.x, pos.y, pos.z, Math.max(1, Math.floor(reach) - 1)))
          arrived = inReach(bot as unknown as never, pos, reach)
        } catch {
          arrived = false
        }
      }
      if (session) {
        session.handle = await interrupts.acquire(session.reason, { timeout: 5000 })
      }
      return arrived
    }

    const actionCtx: ActionContext = {
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

    /**
     * Every world interaction the pathfinder performs goes through this
     * table, including the executor's own dig branch. Replace or wrap any
     * entry to change the behaviour everywhere at once.
     */
    const actions: ActionTable = createActionTable(actionCtx)
    pf.actions = actions

    // ── convenience methods ──────────────────────────────────────────────
    //
    // `goto` gets you somewhere; these get something DONE. Each one walks
    // into vanilla range only if it has to (a block already in reach is acted
    // on where the bot stands), then holds an interrupt for the interaction
    // itself so the tick loop is not fighting the hand.
    //
    // They are serialised against each other: one bot, one hand.
    interface ActionSession { reason: string, handle: InterruptHandle | null }
    let activeSession: ActionSession | null = null
    let actionChain: Promise<unknown> = Promise.resolve()

    async function runAction<T> (reason: string, fn: () => Promise<T>): Promise<T> {
      const start = async (): Promise<T> => {
        const session: ActionSession = { reason, handle: null }
        session.handle = await interrupts.acquire(reason, { timeout: 5000 })
        activeSession = session
        try {
          return await fn()
        } finally {
          activeSession = null
          session.handle?.release()
        }
      }
      const run = actionChain.then(start, start)
      actionChain = run.then(() => undefined, () => undefined)
      return await run
    }

    pf.dig = async (target: BlockTarget, options: DigOptions = {}): Promise<void> =>
      await runAction('pathfinder:dig', async () => {
        const pos = toVec3(target)
        if (options.approach !== false) await approachBlock(pos, options.reach ?? actions.config.reach, options.signal)
        return await actions.dig(resolveBlock(bot as unknown as never, target), options)
      })

    pf.place = async (target: BlockTarget, options: PlaceOptions = {}): Promise<void> =>
      await runAction('pathfinder:place', async () => {
        const pos = toVec3(target)
        if (options.approach !== false) await approachBlock(pos, options.reach ?? actions.config.reach, options.signal)
        return await actions.place(pos, options)
      })

    pf.open = async (target: BlockTarget, options: OpenOptions = {}): Promise<WindowLike> =>
      await runAction('pathfinder:open', async () => {
        const pos = toVec3(target)
        if (options.approach !== false) await approachBlock(pos, options.reach ?? actions.config.reach, options.signal)
        return await actions.open(resolveBlock(bot as unknown as never, target), options)
      })

    pf.activate = async (target: BlockTarget, options: ActivateOptions = {}): Promise<void> =>
      await runAction('pathfinder:activate', async () => {
        const pos = toVec3(target)
        if (options.approach !== false) await approachBlock(pos, options.reach ?? actions.config.reach, options.signal)
        return await actions.activate(resolveBlock(bot as unknown as never, target), options)
      })

    // ── snapshot management ──────────────────────────────────────────────

    function flushPatches (): void {
      // Never touch the SAB while a solve reads it; every solve-completion
      // boundary calls back here, so the queue drains promptly. Updates are
      // ALWAYS queued (never applied directly) so FIFO order guarantees the
      // newest write wins — a direct-write/queued-write interleave used to
      // let a stale queued value overwrite a newer one.
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

    function ensureSnapshot (start: Vec3, descriptor: GoalDescriptor | null, needStates: boolean): Snapshot {
      const lut = getLut(bot, stateMovements)
      const slack = pf.searchRadius as number
      const targets = descriptor ? descriptorTargets(descriptor) : []
      if (descriptor?.type === 'y') targets.push({ x: start.x, y: descriptor.y as number, z: start.z })

      const lutChanged = cachedLut !== lut
      let rebuild = snapshotStale || lutChanged || !cachedSnapshot
      if (!rebuild && cachedSnapshot) {
        // Box still adequate? Start + targets must sit inside with margin.
        const m = cachedSnapshot.meta
        const inside = (x: number, y: number, z: number): boolean =>
          x >= m.x0 + 8 && x <= m.x0 + m.w - 9 && y >= m.y0 + 4 && y <= m.y0 + m.h - 5 &&
          z >= m.z0 + 8 && z <= m.z0 + m.l - 9
        if (!inside(start.x, start.y, start.z)) rebuild = true
        for (const t of targets) {
          if (!inside(t.x, t.y ?? start.y, t.z)) rebuild = true
        }
        if (needStates && !cachedSnapshot.states) rebuild = true
      }

      if (rebuild) {
        const box: Box = computeBox(bot, start, targets, slack, growFactor, opts.maxSnapshotCells)
        cachedSnapshot = buildSnapshot(bot, lut, box, needStates, nextSnapshotGeneration())
        cachedLut = lut
        snapshotStale = false
        pendingPatches.length = 0
      } else {
        flushPatches()
      }
      return cachedSnapshot as Snapshot
    }

    // ── solve orchestration ──────────────────────────────────────────────

    function cancelActiveSolve (): void {
      solveGeneration++
      if (activeSolveCancel) {
        activeSolveCancel()
        activeSolveCancel = null
      }
      activeSolveGeneration = -1
      mainSolver = null
      // Solve is no longer in flight (a cancelled worker's result is
      // discarded by generation, so late SAB reads are harmless) — apply
      // any block updates that queued up during it.
      flushPatches()
    }

    function computeStartMove (startPos: Vec3): { x: number, y: number, z: number } {
      let p = startPos.floored()
      // A body overhanging its block's edge — crept to a lip, landed on a
      // corner — has its centre over the neighbouring column, and a solve
      // started there begins in mid-air over whatever lies below: the gap it
      // was about to jump, which the plan then drops into. Start from the
      // cell the body actually stands on: the one under its box with a floor.
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
      // Upstream quirk preserved: uses stateMovements.emptyBlocks regardless
      // of the movements passed to getPathFromTo.
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

    function contextFromRaw (raw: RawSolveResult): ComputedPathResult['context'] {
      const visitedChunks = new Set<string>()
      for (const [cx, cz] of raw.touchedChunks) visitedChunks.add(`${cx},${cz}`)
      return { visitedChunks }
    }

    function handleDriveResult (raw: RawSolveResult, generation: number, final: boolean): void {
      if (generation !== solveGeneration) return // stale (goal changed etc.)

      // Transparent snapshot growth: a boundary-limited noPath is retried
      // with a larger box before anything is emitted — within ONE think
      // budget for the whole sequence. A goal that is genuinely cut off
      // (the arena's parkouradv1 after a fall to the floor) used to grow
      // five times to the 8M-cell cap, ~1.5 s of exhaustive search each.
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
          // The retry emits nothing, so nothing downstream clears these — and
          // with pathUpdated still latched from an earlier solve the tick loop
          // takes neither the replan branch nor the walk branch, and the goal
          // sits with an empty path and no solve in flight.
          pathUpdated = false
          lastSolveFrom = null
          flushPatches()
          return // next physicsTick startSolve()s again with the bigger box
        }
      }

      // Terminal no-path (growth exhausted): hand the exact solver inputs to
      // the capture hook BEFORE queued patches mutate the snapshot.
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
          console.warn('[bulba-pathfinder] onNoPath hook threw:', error)
        }
      }

      if (final) {
        activeSolveCancel = null
        activeSolveGeneration = -1
        flushPatches()
      }

      const results = toResult(raw, contextFromRaw(raw))
      lastTouchedChunks = results.context?.visitedChunks ?? null
      const postT0 = solveTimingOn ? performance.now() : 0
      if (!inTick) physics.beginTick?.() // between ticks: never reuse the last tick's block cache
      results.path = postProcessPath(results.path)
      pathFromPlayer(results.path)
      if (solveTimingOn) {
        const now = performance.now()
        console.log(`[pf-timing] result ${raw.status} engine=${raw.engine ?? 'js'} final=${final} engineMs=${raw.time.toFixed(1)} visited=${raw.visitedNodes} nodes=${results.path.length} postMs=${(now - postT0).toFixed(2)} wallSinceDispatch=${(now - solveDispatchedAt).toFixed(1)} sinceGoal=${(now - goalSetAt).toFixed(1)}`)
      }
      bot.emit('path_update' as never, results as never)
      const wasEmpty = path.length === 0
      path = results.path
      // A fresh path's first jump takes off from the cell the solve started
      // in, so that is the "node just retired" the creep anchors on — the
      // body itself may be overhanging that cell's lip (a give-up leaves it
      // there), and anchoring on the column under its centre would creep it
      // forward into the gap.
      prevRetired = { x: lastSolveStart.x + 0.5, y: lastSolveStart.y, z: lastSolveStart.z + 0.5 }
      if (final) {
        pathUpdated = true
        // A freshly installed COMPLETE path deserves a full futility window:
        // the bot has not failed to reach anything on it yet. A streamed
        // PARTIAL does not — partials arrive every tick-slice while a long
        // search runs, so refreshing on those meant the 3.5 s check could
        // never expire on exactly the routes that think hardest.
        lastNodeTime = performance.now()
        // Growth budget is per solve, not per goal: a solve that succeeded
        // must not leave the next one starting from a 1.8^5 box.
        if (raw.status === 'success') {
          growFactor = 1
          growAttempts = 0
        }
      }
      // lastNodeArrival is deliberately NOT touched here. It means "when a
      // node was last genuinely reached", it is what bounds how long server
      // corrections may excuse a lack of progress, and a path_update is not
      // an arrival.
      //
      // The first path of a goal drives NOW (see eagerStart). Only when it
      // arrived between ticks — inside the tick the handler runs anyway — and
      // only for the empty→path transition, so a streamed partial that
      // replaces a path the bot is already walking changes nothing about
      // when controls are written.
      if (eagerStart && wasEmpty && path.length > 0 && !inTick && stateGoal !== null) {
        monitorMovement()
      }
    }

    /** Has the bot left the block the last solve was dispatched from? */
    function movedSinceLastSolve (): boolean {
      if (lastSolveFrom === null) return true
      const p = bot.entity.position.floored()
      return p.x !== lastSolveFrom.x || p.y !== lastSolveFrom.y || p.z !== lastSolveFrom.z
    }

    function startSolve (): void {
      if (!stateGoal || !stateMovements) return
      try {
        stateMovements.assertSupported()
      } catch (error) {
        // Post-hoc mutation (movements.canDig = true after setMovements)
        // must not throw inside the physicsTick handler: fail the goal
        // instead (goto rejects PathStopped) and say why, once per goal.
        console.error('[bulba-pathfinder] unsupported movements profile — stopping the goal:', (error as Error).message)
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
      lastSolveCfg = stateMovements.toConfig()
      const canUseWorker =
        opts.useWorkerThreads &&
        descriptor !== null &&
        stateMovements.exclusionAreasStep.length === 0 &&
        (!stateMovements.canDig || stateMovements.exclusionAreasBreak.length === 0) &&
        !workerHost.unavailable

      const needStates = (descriptor !== null && goalNeedsRaycast(descriptor)) || stateMovements.canDig
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
          cfg: stateMovements.toConfig(),
          goal: descriptor,
          start,
          timeout: pf.thinkTimeout as number,
          searchRadius: pf.searchRadius as number,
          sliceMs: Math.max(10, pf.tickTimeout as number),
          dig: digData,
          onPartial: (raw: RawSolveResult) => handleDriveResult(raw, generation, false)
        }
        // Mark in-flight synchronously so the next tick doesn't double-start.
        activeSolveGeneration = generation
        activeSolveCancel = () => {} // placeholder until the handle arrives
        workerHost.solve(req).then((handle) => {
          if (!handle) {
            // Worker unavailable → main-thread fallback (next tick).
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
        stateMovements.toConfig(),
        evaluator,
        start,
        { timeout: pf.thinkTimeout as number, searchRadius: pf.searchRadius as number },
        stepExclusion,
        digContextFor(stateMovements, snapshot, digData)
      )
      // First slice synchronously (upstream computes in-tick); subsequent
      // slices continue from monitorMovement.
      const raw = solver.compute(pf.tickTimeout as number)
      if (raw.status === 'partial') {
        mainSolver = solver
        activeSolveGeneration = generation
      }
      handleDriveResult(raw, generation, raw.status !== 'partial')
    }

    // ── path post-processing (upstream ports) ────────────────────────────

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

    function postProcessPath (thePath: Move[]): Move[] {
      for (let i = 0; i < thePath.length; i++) {
        const curPoint = thePath[i]
        if (curPoint.toBreak.length > 0 || curPoint.toPlace.length > 0) break
        const b = bot.blockAt(new Vec3(curPoint.x, curPoint.y, curPoint.z)) as BlockLike | null
        if (b && (b.type === waterType || b.type === bubbleColumnId ||
            ((b.type === ladderId || b.type === vineId) && i + 1 < thePath.length && thePath[i + 1].y < curPoint.y))) {
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
        } else {
          curPoint.x = Math.floor(curPoint.x) + 0.5
          curPoint.y = curPoint.y - 1
          curPoint.z = Math.floor(curPoint.z) + 0.5
        }
        // A momentum-chain stepping stone is landed on its far side: the
        // planner priced the re-jump with the creep credit of this support
        // (parkourEnvelope.ts), so that is where the body has to be.
        if (curPoint.aimDx !== 0 || curPoint.aimDz !== 0) {
          const half = support !== null && support.shapes.length > 0 ? CATCH_HALF[topCatchClass(support.shapes)] : 0.5
          const credit = Math.min(TAKEOFF_STAND, half + TAKEOFF_NARROW_MARGIN) * CHAIN_TAKEOFF_FRACTION
          const scale = credit / Math.max(Math.abs(curPoint.aimDx), Math.abs(curPoint.aimDz))
          curPoint.x += curPoint.aimDx * scale
          curPoint.z += curPoint.aimDz * scale
        }
      }

      if (!(pf.enablePathShortcut as boolean) || stateMovements.exclusionAreasStep.length !== 0 || thePath.length === 0) return thePath

      const newPath: Move[] = []
      let lastNode: { x: number, y: number, z: number } = bot.entity.position
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

    function pathFromPlayer (thePath: Move[]): void {
      if (thePath.length === 0) return
      let minI = 0
      let minDistance = 1000
      for (let i = 0; i < thePath.length; i++) {
        const node = thePath[i]
        if (node.toBreak.length !== 0 || node.toPlace.length !== 0) break
        const dist = bot.entity.position.distanceSquared(node)
        if (dist < minDistance) {
          minDistance = dist
          minI = i
        }
      }
      const n1 = thePath[minI]
      const dx = n1.x - bot.entity.position.x
      const dy = n1.y - bot.entity.position.y
      const dz = n1.z - bot.entity.position.z
      const reached = Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1
      if (minI + 1 < thePath.length && n1.toBreak.length === 0 && n1.toPlace.length === 0) {
        const n2 = thePath[minI + 1]
        const d2 = bot.entity.position.distanceSquared(n2)
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
            continue // upstream does NOT advance prevNode here
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

    /**
     * Is there a hole just past this landing, in the direction the body is
     * travelling? A cell it can move into with nothing to stand on — a wall
     * is not a hole, because a wall stops the overrun instead of swallowing
     * it. Sampled a block and two blocks out, which is as far as the rest of
     * a descent carries.
     */
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

    // ── stop / reset machinery (upstream ports + hardened cancel) ────────

    function fullStop (): void {
      hopHold = false
      cutTarget = null
      clearWedge()
      bot.clearControlStates()

      // Force horizontal velocity to 0 (otherwise inertia can move us too far)
      bot.entity.velocity.x = 0
      bot.entity.velocity.z = 0

      const blockX = Math.floor(bot.entity.position.x) + 0.5
      const blockZ = Math.floor(bot.entity.position.z) + 0.5

      if (Math.abs(bot.entity.position.x - blockX) > 0.2) { bot.entity.position.x = blockX }
      if (Math.abs(bot.entity.position.z - blockZ) > 0.2) { bot.entity.position.z = blockZ }
    }

    // ── wedge recovery (improvement) ─────────────────────────────────────

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

    /**
     * Drive an escape for ONE tick, or report that none is available.
     *
     * One tick at a time, re-decided every tick, because the amount of room
     * a wedge needs is tiny and the cost of taking more is not. With the
     * hitbox-precision fix in place a step-up needs about 0.01 blocks of
     * clearance; a fixed four-tick back-off gives 0.35 and then has to walk
     * all of it back, which on the arena's climb2 staircase showed up as a
     * 0.9-block sideways wobble on every single step and 14 blocks of extra
     * ground against upstream on the identical plan. The escape stops the
     * moment the gates can work with what it made.
     *
     * The escape is only swapped after ESCAPE_PATIENCE consecutive ticks of
     * it not helping, so a recovery does not flicker between directions.
     */
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
      // Boxed in on every side: nothing to do but let the futility timer
      // replan, and try a different escape first next time.
      nextEscape = (nextEscape + 1) % ESCAPES.length
      recoverTicks = 0
      wedgeTicks = 0
      return false
    }

    /**
     * The futility check, and it has to be reachable from every path through
     * the tick.
     *
     * Upstream's flat 3.5 s is a WALKING budget: swimming covers 2 blocks a
     * second against 4.3 walking, so a legitimate in-water crossing trips it
     * routinely — and the reset then releases jump and lets the bot sink for
     * the whole re-solve, which is worse than the stall it was called for.
     * Water gets the same distance, not the same time, and keeps swimming
     * while it thinks.
     *
     * Called from the end of the tick AND before every early return, because
     * the recovery branches return without falling through: an escape that
     * never works would otherwise hold the tick loop forever with the timer
     * that exists to break exactly that never once being read.
     */
    /**
     * A take-off the executor could not perform — a jump the planner priced
     * that no gate would sign from any line-up — is reported back to the
     * planner as a banned LANDING cell for BAN_MS, so the re-solve routes
     * round it instead of bringing the same jump back. The ban rides on the
     * snapshot's per-cell entity weights, which both solver cores read and
     * treat as impassable above 100; an exclusion area would have forced
     * the JS solver and switched the corner cut off for the whole route.
     */
    function banTakeoff (node: { x: number, y: number, z: number }): void {
      bannedCells.set(
        `${Math.floor(node.x)},${Math.floor(node.y + 0.001)},${Math.floor(node.z)}`,
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
      // Stuck at a take-off: without the ban the plan comes back unchanged
      // and the same jump is tried again for the rest of the run.
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

    /**
     * Turn to make the jump, before shuffling to make room for it.
     *
     * A player lining up an awkward hop does not stare at the block they
     * want — they turn a few degrees to clear whatever is in the way, and the
     * jump that was impossible head-on goes first try. That is the whole idea
     * behind Leg0shii's ParkourCalculatorMod, whose "angle solver" searches
     * yaw inputs for the ones that land a given jump; this is the same thing
     * at a hundredth the scope, run only on a node the executor is already
     * stuck on, and cached per node so the search is paid once.
     *
     * It is tried before the blind escapes because it is the only one that
     * costs nothing: a heading that lands the jump gets the bot to where it
     * was going, while a step back or a step sideways is ground given up and
     * walked again.
     */
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
        // Straight on already works, so this is not the problem — let the
        // escapes have it.
        if (headingBias === 0) headingBias = null
      }
      if (headingBias === null) return false
      // Bounded, because this branch returns without falling through to the
      // gates: an angle the server will not honour would otherwise be driven
      // forever. If the bot has not left the ground within a jump's worth of
      // ticks, the angle is not the answer — hand the node to the escapes.
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

    function internalStop (): void {
      stopPathing = false
      stateGoal = null
      path = []
      pathUpdated = false
      lastSolveFrom = null
      stopDiggingIfNeeded()
      placing = false
      placingBlock = null
      cancelActiveSolve()
      bot.emit('path_stop' as never)
      fullStop()
    }

    // Upstream detectDiggingStopped: clear the digging latch when an
    // interrupted dig settles.
    function detectDiggingStopped (): void {
      digging = false
      bot.removeAllListeners('diggingAborted' as never)
      bot.removeAllListeners('diggingCompleted' as never)
    }

    function stopDiggingIfNeeded (): void {
      if (!digging) return
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
      path = []
      cutTarget = null
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
      // Growth budget is per solve. Sharing one across a whole goal meant a
      // route that grew the box once kept the enlarged box (and the spent
      // attempts) for every later replan, so the last few solves of a long
      // run had no growth left when they needed it.
      growFactor = 1
      growAttempts = 0
      clearWedge()
      cancelActiveSolve()
      lockUseBlock.release()
      stateMovements.clearCollisionIndex()
      if (clearStates) bot.clearControlStates()
      if (stopPathing) internalStop()
    }

    /**
     * Improvement over upstream resetPath(reason, false): keep following the
     * still-valid path while the recompute runs, then splice over. Falls back
     * to the upstream hard reset when disabled or when there is nothing to keep.
     */
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

    // ── public API ───────────────────────────────────────────────────────

    pf.setGoal = (goal: Goal | null, dynamic = false) => {
      stateGoal = goal
      dynamicGoal = dynamic
      goalSetTime = performance.now()
      goalSetAt = goalSetTime
      firstDriveLogged = false
      growFactor = 1
      growAttempts = 0
      bot.emit('goal_updated' as never, goal as never, dynamic as never)
      resetPath('goal_updated')
      // Dispatch the solve now rather than on the next tick (eagerStart). The
      // tick handler's own guards (a goal already at its end, a solve already
      // in flight) apply unchanged; a bot whose entity is not in the world
      // yet keeps the tick-aligned path.
      if (eagerStart && stateGoal !== null && stateMovements !== undefined && !stopPathing &&
          bot.entity?.position !== undefined && Number.isFinite(bot.entity.position.x) &&
          !solveInFlight() && !stateGoal.isEnd(bot.entity.position.floored())) {
        startSolve()
      }
    }

    pf.setMovements = (movements: Movements) => {
      movements.assertSupported()
      stateMovements = movements
      snapshotStale = true
      // Build this profile's block table NOW, on the same reasoning as
      // SolverWorkerHost.prewarm: it is a fixed 47 ms pass over the whole
      // block registry, it is a pure function of (version, profile), and left
      // alone it is paid inside the FIRST goal — the one tick where the bot is
      // being asked to hurry. setMovements is where a bot is still idle. The
      // result is cached by profile fingerprint, so this is one-time per
      // profile and a no-op on every later call; failures are not fatal, the
      // solve would rebuild it anyway.
      try {
        const lut = getLut(bot, movements)
        // Warm buildSnapshot's loops while the bot is idle: a cold first
        // build ran 1.6-3x slower than a warm one (V8 tiers the row loops
        // up on use), and the first goal's build is the one that counts.
        // Three small boxes around the body; unloaded chunks cost nothing.
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
      // Improvement: stop() when idle doesn't latch a flag that would kill
      // the NEXT goal (upstream quirk); stopping an active goal still emits
      // path_stop (goto rejects 'PathStopped') within a tick.
      if (!stateGoal && path.length === 0 && !solveInFlight()) return
      stopPathing = true
    }

    pf.goto = (goal: Goal, options?: GotoOptions) => {
      return gotoImpl(goal, options)
    }

    function gotoImpl (goal: Goal, options: GotoOptions = {}): Promise<void> {
      // Port of upstream lib/goto.js — verbatim event contract and error
      // names; the wrapper string-matches these, keep them byte-identical.
      // The one addition is `bestEffort` (see GotoOptions in types.ts).
      const bestEffort = options.bestEffort === true
      const maxLegs = options.maxBestEffortLegs ?? 3
      let legs = 0
      return new Promise((resolve, reject) => {
        function makeError (name: string, message: string): Error {
          const err = new Error(message)
          err.name = name
          return err
        }

        function goalReached (): void {
          cleanup()
        }

        function noPathListener (results: ComputedPathResult): void {
          if (results.path.length === 0) {
            cleanup()
          } else if (results.status === 'noPath') {
            // BEST EFFORT: a noPath is not empty-handed. Solver.finish returns
            // the path to bestIdx — the closest node the search actually
            // reached — exactly as it does for a timeout. Rejecting here throws
            // that away, and every caller that wants "get as close as you can"
            // then re-derives it by hand.
            //
            // Instead let the drive walk this leg. When it runs out the tick
            // loop re-solves from the new block (see the movedSinceLastSolve
            // branch), and the solve that finds nothing better returns an EMPTY
            // path, which resolves above. So a best-effort goto ends with the
            // bot parked on the closest cell this profile could reach.
            //
            // Bounded, because convergence is not guaranteed: a shifting goal
            // or a moving world can hand back a fresh leg every time. Past the
            // cap we resolve where we stand rather than reject, so callers get
            // one outcome to handle instead of two.
            if (bestEffort && ++legs <= maxLegs) return
            cleanup(bestEffort ? undefined : makeError('NoPath', 'No path to the goal!'))
          } else if (results.status === 'timeout') {
            cleanup(makeError('Timeout', 'Took to long to decide path to goal!'))
          }
        }

        function goalChangedListener (newGoal: Goal): void {
          if (newGoal !== goal) {
            cleanup(makeError('GoalChanged', 'The goal was changed before it could be completed!'))
          }
        }

        function pathStopped (): void {
          cleanup(makeError('PathStopped', 'Path was stopped before it could be completed! Thus, the desired goal was not reached.'))
        }

        function cleanup (err?: Error): void {
          bot.removeListener('goal_reached' as never, goalReached as never)
          bot.removeListener('path_update' as never, noPathListener as never)
          bot.removeListener('goal_updated' as never, goalChangedListener as never)
          bot.removeListener('path_stop' as never, pathStopped as never)
          setTimeout(() => {
            if (err) reject(err)
            else resolve()
          }, 0)
        }

        bot.on('path_stop' as never, pathStopped as never)
        bot.on('goal_reached' as never, goalReached as never)
        bot.on('path_update' as never, noPathListener as never)
        bot.on('goal_updated' as never, goalChangedListener as never)
        ;(pf.setGoal as (g: Goal | null, dynamic?: boolean) => void)(goal)
      })
    }

    // Synchronous compute API (upstream parity — main-thread, tick-sliced).
    pf.getPathFromTo = function * (movements: Movements, startPos: Vec3, goal: Goal, options: {
      optimizePath?: boolean
      resetEntityIntersects?: boolean
      timeout?: number
      tickTimeout?: number
      searchRadius?: number
      startMove?: { x: number, y: number, z: number }
    } = {}): Generator<{ result: ComputedPathResult, astarContext: unknown }> {
      movements.assertSupported()
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

      // A private snapshot for this query (doesn't disturb the drive cache).
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
      const solver = new Solver(snap, movements.toConfig(), new GoalAdapter(goal), start, { timeout, searchRadius }, stepExclusion, queryDig)
      const astarContext = {
        get visitedChunks () { return solver.visitedChunks },
        compute: () => toResult(solver.compute(tickTimeout), null)
      }

      // NOTE: unlike drive solves, a read-only query must NOT touch
      // lastTouchedChunks (the drive loop's chunkColumnLoad replan state).
      let raw = solver.compute(tickTimeout)
      let result = toResult(raw, { visitedChunks: solver.visitedChunks })
      if (optimizePath) result.path = postProcessPath(result.path)
      yield { result, astarContext }
      while (result.status === 'partial') {
        raw = solver.compute(tickTimeout)
        result = toResult(raw, { visitedChunks: solver.visitedChunks })
        if (optimizePath) result.path = postProcessPath(result.path)
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

    // ── world-change invalidation ────────────────────────────────────────

    bot.on('blockUpdate' as never, ((oldBlock: BlockLike | null, newBlock: BlockLike | null) => {
      if (!oldBlock || !newBlock) return

      // Keep the snapshot current. Always queue-then-flush (FIFO) so a
      // newer write can never be overwritten by an older queued one.
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
        // Hard-clear when the change is imminent on our path; otherwise keep
        // walking the prefix while we replan.
        let imminent = false
        for (let i = 0; i < Math.min(3, path.length); i++) {
          const n = path[i]
          if (Math.abs(n.x - oldBlock.position.x) <= 1 && Math.abs(n.z - oldBlock.position.z) <= 1 &&
              Math.abs(n.y - oldBlock.position.y) <= 2) {
            imminent = true
            break
          }
        }
        if (imminent) resetPath('block_updated', false)
        else softReplan('block_updated')
      }
    }) as never)

    // Lagback / teleport reaction (improvement — upstream walks at a stale
    // node until the 3.5s futility timer). On a server position correction:
    // still near the path → splice to the closest node and keep walking;
    // snapped away → replan immediately from the corrected position.
    bot.on('forcedMove' as never, (() => {
      lastForcedMoveAt = performance.now()
      if (path.length === 0) return
      const at = bot.entity.position.floored()
      // Under a live cut the body is off every node by design, and the
      // skipped nodes retire on their own as it draws level with them, so a
      // small correction changes nothing: keep the line (CUT_LAG_NEAR).
      const nearCut = cutTarget !== null && prevTickPos !== null &&
        bot.entity.position.distanceTo(prevTickPos) <= CUT_LAG_NEAR
      if (nearCut || isPositionNearPath(at, path)) {
        if (!nearCut) pathFromPlayer(path)
        // A correction is not "stuck" — but only for as long as the bot is
        // otherwise making progress. A bot WEDGED in geometry gets corrected
        // every tick, and refreshing unconditionally meant the 3.5 s futility
        // timer was reset before it could ever expire: measured on 2b2t
        // spawn, 46 s motionless with 34 path nodes left and not one `stuck`
        // reset, in the same spot every run. Upstream has no such handler and
        // walks the same route without trouble. Past the grace window the
        // corrections stop excusing it and the futility check does its job.
        if (performance.now() - lastNodeArrival < FORCED_MOVE_GRACE_MS) {
          lastNodeTime = performance.now()
        }
      } else {
        resetPath('forced_move', false)
      }
    }) as never)

    bot.on('chunkColumnLoad' as never, ((chunk: Vec3) => {
      const cx = chunk.x >> 4
      const cz = chunk.z >> 4
      if (cachedSnapshot) {
        const m = cachedSnapshot.meta
        if ((cx << 4) + 15 >= m.x0 && (cx << 4) <= m.x0 + m.w - 1 &&
            (cz << 4) + 15 >= m.z0 && (cz << 4) <= m.z0 + m.l - 1) {
          snapshotStale = true
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

    // ── the tick loop (upstream monitorMovement port) ────────────────────

    function monitorMovement (): void {
      // Reentrancy guard: the eager first drive (handleDriveResult) may call
      // this between ticks, and a path_update listener may set a new goal
      // from inside it — never run two tick bodies nested.
      if (inTick) return
      inTick = true
      const t0 = performance.now()
      try {
        physics.beginTick?.()
        geometry.beginTick()
        monitorMovementInner()
      } finally {
        geometry.endTick()
        inTick = false
        lastTickMs = performance.now() - t0
      }
    }

    function monitorMovementInner (): void {
      // Improvement: a requested stop takes effect within one tick, even
      // mid-edge — controls are always released.
      if (stopPathing) {
        internalStop()
        return
      }

      // Cooperative interrupts. A stop always outranks a pause (above), but
      // everything else waits: while a handle is held the executor writes no
      // controls at all, and the path is left exactly as it was so the bot
      // carries on from the same node when the holder releases.
      if (interrupts.gate()) {
        execBranch = 'interrupt'
        // Standing still in water is not standing still — it is sinking, at
        // two blocks a second, for as long as the holder takes.
        if ((bot.entity as { isInWater?: boolean }).isInWater === true) {
          bot.setControlState('jump', true)
        }
        return
      }

      // Test freemotion
      const freeGoal = stateGoal as (Goal & { entity?: { position: Vec3 }, rangeSq?: number }) | null
      if (stateMovements && stateMovements.allowFreeMotion && freeGoal && freeGoal.entity) {
        const target = freeGoal.entity
        if (physics.canStraightLine([target.position])) {
          bot.lookAt(target.position.offset(0, 1.6, 0))
          // Upstream: `dist > rangeSq` — false when rangeSq is undefined, so
          // an entity goal without rangeSq STOPS at the entity (no ?? 0!).
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

      // Improvement knobs (default off).
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

      // Main-thread sliced solve continuation (upstream astartTimedout).
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
              bot.emit('goal_reached' as never, stateGoal as never)
              stateGoal = null
              fullStop()
              return
            }
            // Dynamic goal standing at its end: idle until hasChanged fires
            // (upstream else-if structure — no recompute here).
          } else if (!solveInFlight() && (!pathUpdated || movedSinceLastSolve())) {
            // Divergence from upstream, and the reason for it: upstream gates
            // this purely on `!pathUpdated`, which latches true as soon as any
            // final path is delivered. A path that is then walked to
            // exhaustion WITHOUT reaching the goal therefore leaves the bot
            // with nothing to walk, no recompute, and no escape — and because
            // `lastNodeTime` is refreshed above on every empty-path tick, the
            // 3.5 s futility timer can never fire either. The bot stands
            // still forever, waiting for a block change or a new goal.
            // Measured on 2b2t spawn: 83 s of a 97 s run motionless, and runs
            // that ended 29 blocks short of the goal having stopped trying.
            //
            // Re-solving when the bot is on a DIFFERENT block than the last
            // solve started from fixes it while staying bounded: a genuinely
            // unreachable goal still costs exactly one extra solve, because a
            // stationary bot never satisfies the condition again.
            startSolve()
          }
        }
      } else if (stateGoal && stateMovements && !pathUpdated && !solveInFlight()) {
        // Soft replan: keep walking the old prefix while recomputing.
        startSolve()
      }

      if (path.length === 0) {
        execBranch = solveInFlight() ? 'nopath-solving' : 'nopath'
        // Release what only the tick loop ever presses. `resetPath(reason,
        // false)` deliberately keeps the controls so the bot coasts through a
        // soft replan, which is right for forward/sprint — but a `back` or a
        // strafe left over from a wedge escape would then be held for the
        // whole recompute and walk the bot away from its own path.
        // Do not sink while thinking. A bot with no path sets no controls at
        // all, and in water that is not "standing still" — it is 2 blocks a
        // second downward, for as long as the solve takes (up to the whole
        // think budget). Every re-solve therefore started from a worse place
        // than the last one, which is how a stall in a pool turns into a
        // drowning.
        if ((bot.entity as { isInWater?: boolean }).isInWater === true) {
          bot.setControlState('jump', true)
        }
        if (recoverTicks > 0) {
          clearWedge()
          bot.setControlState('back', false)
          bot.setControlState('left', false)
          bot.setControlState('right', false)
        }
        return
      }

      let nextPoint: Move = path[0]
      const p = bot.entity.position
      if (solveTimingOn && !firstDriveLogged) {
        firstDriveLogged = true
        console.log(`[pf-timing] first drive tick sinceGoal=${(performance.now() - goalSetAt).toFixed(1)} inTick=${inTick}`)
      }

      // Handle digging (canDig solves only): stand still and hand the block
      // to the interaction table, which equips, aims, guards and verifies.
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
          // Through the action table, not bot.dig: the guards (vanilla range,
          // face agreement, grounded stance, mid-dig abort, verify-by-world)
          // are the same ones bot.pathfinder.dig() gets, and an application
          // that replaced `actions.dig` gets its own dig here too.
          //
          // `approach: false` — the executor is already standing on the node
          // the planner picked; walking somewhere else mid-path is the last
          // thing it should do.
          actions.dig(block as never, { approach: false, equipTool: true })
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

      // Door/fence-gate activation (activate — never place).
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
          // activateBlock settles async — if the goal/path changed meanwhile,
          // its callbacks must not touch the NEW path's state.
          const pathToken = path
          // Through the table too, so an application that wraps `activate`
          // sees the doors the executor opens as well as the ones it opens
          // itself. `settle: false` keeps this the same single click it has
          // always been — the bot is already facing the door it is walking
          // through, and re-aiming mid-path turns a door into a stall.
          actions.activate(block as never, { approach: false, retries: 1, settle: false }).then(() => {
            lockUseBlock.release()
            if (path !== pathToken) return
            placingBlock = nextPoint.toPlace.shift() ?? null
            if (!placingBlock) {
              placing = false
              lastNodeTime = performance.now()
            }
          }, (err: unknown) => {
            console.error(err)
            lockUseBlock.release()
            if (path !== pathToken) return
            resetPath('place_error')
          })
          return
        }
        // No other toPlace kind can exist in our paths; clear defensively.
        placing = false
        placingBlock = null
        return
      }

      // SWIMMING, not merely wet (improvement). Upstream keys its water
      // branch on `isInWater`, which mineflayer sets for a body touching
      // water anywhere — so one waterlogged step, a puddle, or a splash at
      // head height cancels sprint and holds jump for the whole crossing, and
      // the bot bobs across a ford it could have run through. What actually
      // distinguishes swimming from wading is whether the HEAD cell is water:
      // there the bot has to swim up, and below it, it can walk.
      const head = bot.blockAt(bot.entity.position.offset(0, 1, 0)) as { type: number } | null
      const swimming = (bot.entity as { isInWater?: boolean }).isInWater === true &&
        head !== null && (head.type === waterType || head.type === bubbleColumnId)

      let dx = nextPoint.x - p.x
      let dy = nextPoint.y - p.y
      let dz = nextPoint.z - p.z
      // Arrival. Two divergences from upstream's single shift, both forced by
      // the sprint-hop gait: a hop crosses a node at up to 1.25 blocks above
      // it — outside the |dy| < 1 box — and covers more than one node's worth
      // of ground per tick. A node left behind unconsumed is worse than a
      // wasted one: the executor turns the bot round and walks it back to
      // somewhere it has already flown over. So while hopping the box is as
      // tall as the hop, and EVERY node the body has passed is consumed, not
      // just the first. Walking, `arriveDy` is upstream's 1 and the loop can
      // only ever run once (two nodes inside one 0.7-wide box and within a
      // block of each other would have to be the same cell).
      const arriveDy = hopHold ? HOP_ARRIVE_DY : 1
      // Landed, by any of the means the planner counts as a landing (see
      // PhysicsSim.caught): read once for the arrival loop.
      const caughtNow = (() => {
        const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
        if (ent.onGround === true || ent.isInWater === true) return true
        const feet = bot.blockAt(p) as BlockLike | null
        return feet !== null && (feet.type === ladderId || feet.type === vineId)
      })()
      {
        const groundedNow = (bot.entity as { onGround?: boolean }).onGround === true
        if (!groundedNow) landedTicksAgo = 99
        else if (lastTickGrounded) landedTicksAgo = Math.min(99, landedTicksAgo + 1)
        else landedTicksAgo = 0
        lastTickGrounded = groundedNow
      }
      // A target from a path that has since been replaced, or one that has
      // become path[0] itself, is no line at all — and the retirement rule
      // below measures skipped nodes against it.
      if (cutTarget !== null && path.indexOf(cutTarget) < 1) cutTarget = null
      for (;;) {
        // Improvement: never finish a path mid-air. The arrival box has no
        // ground requirement, so a goal satisfied at jump apex would cut
        // sprint+forward and drop the bot short of its landing — keep flying
        // the final node until grounded (water and climbables count as landed).
        let airborneHold = false
        if (path.length === 1 && Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < 1) {
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
          if (ent.onGround !== true && ent.isInWater !== true) {
            const feet = bot.blockAt(p) as { type: number } | null
            airborneHold = feet === null || (feet.type !== ladderId && feet.type !== vineId)
          }
        }
        // A PARKOUR node is landed on, never flown over (improvement).
        //
        // The arrival box has no ground requirement, so a jump that passes
        // over its node on the way down satisfies it in mid-air — and the
        // executor then starts flying at the node AFTER it, which on the far
        // side of a gap means steering into the gap. On the arena's basic1 a
        // 4-block drop onto a 2-cell shelf was retired 0.9 blocks above the
        // shelf, and the bot spent the rest of its fall thrusting toward a
        // node across the chasm beyond it: 40 blocks down, one run in eight,
        // on either gait. `landsThere` already refuses to AUTHORISE a jump
        // that only passes through its node; this is the same rule on the
        // other side of the flight.
        //
        // Held back, the node stays the thing being steered at, so the bot
        // spends the descent aiming at the shelf — which is also the fastest
        // way to be standing on it.
        // The threshold is a LANDING's worth of height, not zero. Holding all
        // the way to touchdown also works and is safe, but it costs about a
        // tick per jump across the route book — while the node is held the
        // bot keeps steering at it, which means steering backwards once it is
        // underneath, and that sheds the momentum the next move wants. A body
        // within half a block of the node is committed to it; a body 0.9 up,
        // which is where the arena retired it, is not.
        //
        // And only where flying past would actually cost something. A landing
        // with more ground beyond it is one the bot can overrun harmlessly,
        // which is most of them — holding those too is safe but costs about a
        // tick a jump across the route book (basic2 +0.8 s, 15 jumps). So the
        // hold is spent on the case it was written for: a shelf with a hole
        // after it.
        // "Supported" has to mean what the extended repertoire means by it. A
        // gap-jump that CATCHES a ladder or a vine ends with the body neither
        // on the ground nor in water, so a hold that only accepts those two
        // can never release — the node sits there until the futility timer
        // gives up on a jump that in fact worked perfectly.
        if ((nextPoint as { parkour?: boolean }).parkour === true &&
            p.y - nextPoint.y > PARKOUR_LAND_DY) {
          const ent = bot.entity as { onGround?: boolean, isInWater?: boolean }
          const feet = bot.blockAt(p) as BlockLike | null
          const caught = ent.onGround === true || ent.isInWater === true ||
            (feet !== null && (feet.type === ladderId || feet.type === vineId))
          // A chain's stone is held to touchdown whatever lies beyond it: the
          // re-jump is pressed on the landing tick, and a stone retired in
          // mid-air is settled onto at a walk instead (the chain branch below).
          const stoneOfChain = path.length > 1 && (path[1] as Move).chain === true
          if (!caught && (stoneOfChain || holeBeyond(nextPoint, p))) break
        }
        if (airborneHold) break
        // Inside the box, or simply GONE BY. The corner cut does not steer
        // through every node centre by design, so a node the body has passed
        // is done even though it was never inside its 0.7-wide box; left in
        // place it turns the bot round to collect it, and the bot then cuts
        // forward again — the vibrating-on-flat-ground shuffle.
        //
        // "Passed" is measured along the CHORD the body is actually walking —
        // the line to the node the cut committed to — and only while a cut is
        // live. Two rejected alternatives, both of which strand nodes:
        //
        //   - the body's velocity: the wedge recovery drives BACKWARDS, which
        //     reads as "everything ahead of me is behind me".
        //   - the path's own next leg (path[0] -> path[1]): where that leg
        //     turns away from the chord, the window in which a node is both
        //     ahead-of-the-leg and inside the retirement disc can be empty. At
        //     a 45-degree zig with the node 0.64 off the chord there is no
        //     such tick at all, and at 0.6 off the window is 0.07 blocks —
        //     a quarter of a tick at sprint speed. Missing it strands the node
        //     permanently, because path.shift() is the only consumption site,
        //     and a stranded node is the bot shuffling on one spot until the
        //     futility timer replans. That is the glitch this whole feature
        //     was reported for.
        //
        // Projecting onto the chord cannot have an empty window: the body
        // advances along it monotonically. A skipped node is done the moment
        // the body draws LEVEL with it — behind the plane square to the chord
        // through the body — however far off the line it sits: the cut
        // committed to reaching the target directly, and a node behind that
        // plane can only be collected by turning round. (The old rule also
        // wanted the node within 0.9 of the body, which is why selection had
        // to keep every skipped node hugging the chord, and why a run four
        // across and thirteen along walked the zig-zag's 45° leg and then its
        // straight leg instead of the line between the ends.) Or the body is
        // inside the TARGET's own box with skipped nodes still queued — a
        // path that wiggled back on itself — and they are all behind it by
        // construction. Never the target itself, and never the last node:
        // the goal is always arrived at.
        let passed = false
        if (cutTarget !== null && cutTarget !== nextPoint && !swimming && path.length > 1 &&
            Math.abs(dy) < arriveDy &&
            (nextPoint as { parkour?: boolean }).parkour !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0) {
          // The line's own direction, so a body knocked off it does not tilt
          // the plane; body-to-target when no anchor is known.
          const from = cutFrom ?? p
          const cx = cutTarget.x - from.x
          const cz = cutTarget.z - from.z
          passed = dx * cx + dz * cz <= 0 ||
            (Math.abs(cutTarget.x - p.x) <= 0.35 && Math.abs(cutTarget.z - p.z) <= 0.35 &&
             Math.abs(cutTarget.y - p.y) < arriveDy)
        }
        // A parkour landing the body came down PAST (improvement,
        // allowLandingRetire). The arrival box is 0.35 wide; a running jump
        // lands up to a block beyond its node, which is exactly what
        // `landsThere` authorised. Outside the box the node is not retired,
        // so the bot turns round, walks back into it, and takes the next
        // jump from rest: on the arena's basic1 every 2-block hop of a chain
        // of four cost 8 ticks from landing to the next take-off instead of
        // 2. The rule is deliberately narrow: the body touched down THIS tick
        // or the last (a standing body beside the node is not a landing),
        // within a block, and beyond the node along the FLIGHT — from the
        // node it took off from, never the leg to the next node: on a
        // switchback that leg points back at the take-off and the test
        // retired the landing before the jump (the bot then walked at the
        // node after it, across the gap — a fall loop on a tree-crown climb
        // in production, 2026-08-26). Not for a momentum-chain stone, whose
        // re-jump is pressed from ON the stone.
        if (!passed && stateMovements.allowLandingRetire && !swimming && path.length > 1 &&
            caughtNow && landedTicksAgo <= 1 && prevRetired !== null && Math.abs(dy) < 1 &&
            (nextPoint as { parkour?: boolean }).parkour === true &&
            (path[1] as Move).chain !== true &&
            nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0 &&
            Math.hypot(dx, dz) <= 1.0) {
          const fx = nextPoint.x - prevRetired.x
          const fz = nextPoint.z - prevRetired.z
          passed = fx * fx + fz * fz > 1 && (p.x - nextPoint.x) * fx + (p.z - nextPoint.z) * fz > 0
        }
        if (!passed && !(Math.abs(dx) <= 0.35 && Math.abs(dz) <= 0.35 && Math.abs(dy) < arriveDy)) break

        // arrived at next point
        lastNodeTime = performance.now()
        lastNodeArrival = lastNodeTime
        prevRetired = { x: nextPoint.x, y: nextPoint.y, z: nextPoint.z }
        path.shift()
        // The target itself just retired: the nodes beyond it are not
        // "behind" anything, and the retirement rule above must not measure
        // them against a chord that no longer exists.
        if (cutTarget === nextPoint) cutTarget = null
        if (path.length === 0) { // done
          if (!dynamicGoal && stateGoal && (stateGoal.isEnd(p.floored()) || stateGoal.isEnd(p.floored().offset(0, 1, 0)))) {
            // A soft-replan solve may still be in flight (we kept walking the
            // old path and won the race) — cancel it, or its late result
            // would re-install a path with no goal set.
            cancelActiveSolve()
            bot.emit('goal_reached' as never, stateGoal as never)
            stateGoal = null
          }
          fullStop()
          return
        }
        // not done yet
        nextPoint = path[0]
        if (nextPoint.toBreak.length > 0 || nextPoint.toPlace.length > 0) {
          fullStop()
          return
        }
        dx = nextPoint.x - p.x
        dy = nextPoint.y - p.y
        dz = nextPoint.z - p.z
      }


      // A landing that hurt is a landing whose speed the server is about to
      // take away (FALL_DAMAGE_DISTANCE): no take-off decision on that tick.
      // Standing still for a tick costs nothing on a full block, and on a
      // post it is the only thing that does not walk off it.
      //
      // `airborne` is the tick's believed flight state (improvement): a lone
      // onGround=false with no upward velocity and no height change is a
      // flicker, not a flight. Traced on parkouradv1: the body landed on a
      // pot, onGround blinked off for one tick at 258.50 with vel.y −0.078,
      // the in-flight branch held forward+sprint for that tick, and the body
      // walked off the pot's 0.375 top before any gate could refuse it.
      let airborne = false
      {
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        if (!grounded && !swimming) {
          airTicks++
          airborne = airTicks >= AIRBORNE_TICKS || bot.entity.velocity.y > 0.05 || p.y < prevTickY - 0.02
        } else {
          airTicks = 0
        }
        prevTickY = p.y
        prevTickPos = p.clone()
        if (!grounded && !swimming) {
          wasAirborne = true
          if (p.y > airPeakY) airPeakY = p.y
        } else if (wasAirborne) {
          wasAirborne = false
          if (grounded && airPeakY - p.y > FALL_DAMAGE_DISTANCE) {
            // Two ticks at zero ping, not one: the server processes the
            // landing position on ITS next tick and the damage velocity
            // packet comes back a tick after that. Traced on parkouradv1's
            // fence-post landing: settle at tick N, packet at N+1 — and in
            // between the gates had approved a delayed jump and pressed
            // forward, which put the body on the post's edge with the sneak
            // guard off when the packet flipped onGround. Off it went.
            const ping = (bot as unknown as { player?: { ping?: number } }).player?.ping ?? 0
            landSettle = 2 + Math.min(4, Math.ceil(ping / 50))
          }
          airPeakY = -Infinity
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

      // Slime bounce (improvement, allowParkourExtended): this node is reached
      // by dropping onto the slime stand cell `via` and riding the rebound up
      // — the planner priced the whole arc as one edge (moveGen.slimeBounce).
      // Two phases, latched per node: aim at the slime's top centre until the
      // rebound has actually begun (feet at the slime, moving up), then aim
      // at the node. Walking, never sprinting — the drop has to land on ONE
      // block, and that is the whole point of "knowing where to land" — and
      // never sneaking, which cancels the bounce in vanilla. None of the
      // rollout gates below apply: a body in a rebound is not on the ground
      // to jump from, and the arc is the planner's envelope, not theirs.
      // Momentum chain (improvement, allowParkourExtended): the stepping
      // stone `via` was installed as its own parkour node ahead of this one
      // (Move.expandRaw) and has just been landed on and retired. The planner
      // priced this node on the J_CHAIN row — a re-jump on the LANDING TICK,
      // speed carried over — so there is exactly one right tick to press
      // jump, and the rollout gates (which model a jump from rest) would
      // refuse it. Press while grounded on the stone, aimed at the node,
      // sprint held; once the body has left the stone the ordinary in-flight
      // handling flies the arc. Not yet on the stone (retired mid-air over
      // it): settle onto it first, steering at its centre.
      const chainVia = (nextPoint as Move).via
      if ((nextPoint as Move).chain && chainVia !== null) {
        if (chainNode !== nextPoint) {
          chainNode = nextPoint
          chainFired = false
        }
        if (!chainFired) {
          cutTarget = null
          hopHold = false
          const ent = bot.entity as { onGround?: boolean }
          const onStone = Math.floor(p.x) === chainVia.x && Math.floor(p.z) === chainVia.z && p.y < chainVia.y + 1
          if (bot.entity.velocity.y > 0.3 && ent.onGround !== true) {
            chainFired = true
          } else if (ent.onGround === true && onStone) {
            execBranch = 'chain'
            // The landing speed is in the bot's state, so the ordinary
            // sprint-jump rollout predicts the chained flight honestly —
            // press only when it lands; otherwise this is an ordinary node
            // and the gates below decide (a refused re-jump is a stop, not
            // a fall).
            if (!physics.canSprintJump(path)) {
              chainFired = true
            } else {
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

      // Corner cut (improvement): walk the line, not the staircase.
      //
      // The planner routes cell centre to cell centre over eight directions,
      // so a run a few degrees off a cardinal comes back as an alternating
      // zig-zag — and a follower that steers at each centre in turn walks
      // every zig, and swings its heading ±45° at every one. Sprinting that
      // is merely wasteful; HOPPING it is expensive, because each arc is
      // committed at take-off and the yaw swing happens with only air control
      // to answer it. Measured on the arena's simple1, same plan and same
      // gait: 88.8 blocks walked for a 74.6-block line.
      //
      // So the executor pulls the string: it STEERS at the furthest node it
      // can reach in a straight line the body fits down — full hitbox sampled
      // every quarter block, floor under every sample — and the nodes in
      // between retire as the body goes by them (see the arrival loop). Only
      // across flat plain walking, and only what geometry can vouch for: a
      // rise, a parkour node, anything to break or place, or a profile with
      // exclusion zones stops the scan, because those carry planner reasoning
      // a swept box cannot re-derive.
      //
      // The nodes are STEERED PAST, not spliced out: each retires as the body
      // draws level with it (the arrival loop), so path[0] stays the first
      // node still ahead, and everything that reads path[0] — the lagback
      // recovery, the parkour approach, the trace — keeps its meaning.
      // Kept pointing at path[0] whatever the cut does below. The angle
      // solver's offset is measured from the heading to path[0], so adding it
      // to a heading aimed several nodes further on would fly the jump at
      // something nobody solved for.
      const nodeDx = dx
      const nodeDz = dz
      // Only where a swept hitbox is the WHOLE story. Exclusion areas and
      // entity avoidance are cost fields the planner paid to detour around,
      // and geometry cannot see either — a mob the profile is avoiding leaves
      // a one-cell detour that the cut would happily straighten right back
      // through it. Liquids likewise: the swept box reads a pond's bed as
      // floor, and the planner priced the swim.
      //
      // And never on the approach to a jump (CUT_JUMP_GUARD): the cut owns
      // the running and the node-by-node follower owns the jumps.
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
       * Can the body steer at path[k] from here? A rise, a jump or work to do
       * at the node ends a scan outright ('stop'): those carry planner
       * reasoning a swept box cannot re-derive, and nothing past them is one
       * line away either. A chord the body does not fit down is a 'miss', and
       * the scan looks a little further (CUT_MISSES). `ref` is the direction
       * that counts as forwards — never steer BACKWARDS, or the pick answers
       * with a node the body has already gone by (the shortest legal chord is
       * the one behind you) and the bot turns round, collects it, cuts forward
       * and turns round again.
       *
       * The skipped nodes are NOT required to hug the chord. They used to be
       * (within 0.55), because they retired only within 0.9 of the body; they
       * now retire as the body draws level with them, so the line may run
       * straight across a whole zig-zag — the diagonal leg and the straight
       * leg of an L become one chord between its ends.
       */
      const cuttableTo = (k: number, ox: number, oz: number, refDx: number, refDz: number, margin: boolean): 'ok' | 'miss' | 'stop' => {
        const n = path[k]
        if (n.toBreak.length > 0 || n.toPlace.length > 0) return 'stop'
        if (Math.abs(n.y - nextPoint.y) > 0.1) return 'stop'
        // A cut ends CUT_JUMP_GUARD nodes short of a jump, so that it ends by
        // REACHING a node on the path and the approach is walked on the
        // planner's own cells. Ending it only when the jump comes within the
        // guard of path[0] (the per-tick check above) left the body wherever
        // the taut line had it — off the node line by up to a couple of
        // blocks — at the moment node-by-node following took the take-off.
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
        // COMMIT to a target, and only ever trade it for one FURTHER along
        // the same path. Re-picking the furthest legal node every tick made
        // the heading flick between path[1] and path[4] as the line scan
        // flickered on and off near terrain — a wobble on the screen and a
        // waste of momentum. Extension has no such failure: the aim moves in
        // one direction, onto the straight line, and stops at the horizon.
        // The target is dropped only when it stops being legal (the chord
        // from where the body now is no longer fits), when the body has been
        // knocked too far off its line (CUT_MAX_OFF), or when it is reached.
        //
        // The line is FIXED: anchor → target, with the anchor sliding along
        // it to the body's foot each grounded tick, so the direction never
        // changes underneath the body and lateral error is a real quantity
        // the steering below can correct. Re-anchoring the chord at the body
        // every tick (the first taut cut did) has no memory of the line: a
        // body pushed sideways simply gets a new, rotated chord, and steering
        // straight at a far target never pulls it back.
        //
        // A body in the air keeps the target it took off with, unrevalidated.
        // Mid-arc a live re-pick swings the yaw with only air control (0.02
        // per tick) to answer it, and the bot lands off the line. This is the
        // same reasoning as biasFlight: a line committed to on the ground is
        // flown on the ground's terms.
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
        // Forwards, for a fresh pick, is the path's own next leg from here:
        // path[1] is always ahead of itself, and a node lying behind that
        // direction is one the body overshot and must go back for.
        if (cutTarget === null && !airborne) {
          cutTarget = scanCut(1, CUT_SCAN_INIT, p.x, p.z, path[1].x - p.x, path[1].z - p.z)
          if (cutTarget !== null) cutFrom = p.clone()
        }
      }

      // What the physics gates are asked about. Node by node, the path
      // itself. Under a live cut, the LINE being walked: synthetic nodes a
      // block apart up the line from the body's foot to the target, then the
      // real path beyond it. Every rollout — sprint, hop, grind, the jump
      // gates — then drives the heading the body is about to take rather than
      // the zig-zag it is skipping, and the node-count heuristics (HOP_LOOK,
      // the hop score, the rise scan) see the block-a-node density they were
      // tuned on. With the old bounded cut path[0] was always within half a
      // block of the chord, so asking about it was asking about the line; a
      // taut cut leaves path[0] up to several blocks off to the side, where a
      // rollout would authorise the wrong heading or refuse the right one.
      //
      // And the steering itself (pure pursuit, CUT_CARROT): aim at the point
      // on the line CUT_CARROT ahead of the body's foot, not at the target.
      // On the line the two headings are identical; off it, the carrot pulls
      // the body back at atan(error / CUT_CARROT), which is the same feedback
      // the rollouts model when they re-aim node by node.
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

      // Is the bot getting anywhere? Two triggers, because waiting is only
      // cheap when it is rare. The generic one is slow on purpose (a sharp
      // turn or a landing can hold the body still for a few ticks and neither
      // is a wedge). The fast one asks the physics whether walking at the node
      // from here gets anywhere at all: on a staircase the answer is no the
      // instant the bot lands against the next riser, and paying the full 12
      // ticks there cost ~1.4 s PER STEP on the arena's 46-block climb.
      const stillTicks = updateWedge(p)
      const wedged = stillTicks > WEDGE_TICKS ||
        (stillTicks >= 1 && (bot.entity as { onGround?: boolean }).onGround === true &&
         physics.isGrinding(gatePath))
      // A wedged body has just contradicted the swept line its target was
      // chosen on, and the recovery below reasons about path[0]: give it path[0].
      if (wedged && cutTarget !== null) {
        cutTarget = null
        gatePath = path
        dx = nodeDx
        dz = nodeDz
      }

      // Diagonal squeeze: go round the corner that is actually open
      // (improvement).
      //
      // A diagonal move is priced on the CHEAPER of its two corners — that is
      // upstream's rule and ours by parity — which is honest only if the
      // walker goes AROUND that corner. Aiming at the node centre instead
      // cuts across both, and a 0.6-wide body clips the blocked one: it never
      // arrives, the physics rollouts all refuse (rightly), and the executor
      // stands there. Measured on the arena's climb1 tower: 23 s motionless
      // one cell from the node, with the open corner beside it the whole
      // time, on a move the planner was right to think was walkable.
      //
      // Inserting the open corner turns the squeeze into two ordinary moves —
      // exactly the two the planner priced. It cannot recurse: the waypoint
      // is cardinal from here, and the node is cardinal from the waypoint.
      //
      // ONLY once the bot is actually stuck on the node, though. Most tight
      // diagonals are flown straight through perfectly well — the body clips
      // the corner, the physics resolves it per axis and the bot slides on —
      // and routing every one of them through a waypoint turns a straight
      // climb into a staircase of 1-block sidesteps. Measured on the arena's
      // climb2: a 0.9-block x zig-zag on every step, 127 blocks walked
      // against upstream's 105 on the same 62-node plan, and 4.4 s lost.
      //
      // One exception to "only once wedged" (improvement, earlySqueeze): a
      // FLAT diagonal walk step whose straight line the physics already
      // refuses, with exactly one corner open. Left to the gate cascade the
      // jump gates take it — a 12-tick arc for a 5-tick step — and the arc
      // overshoots the node, so the bot then turns round and walks back to
      // collect it, and takes the next jump from rest. Traced on the arena's
      // basic3 (-94,233,95 → -95,233,94, corner -95,233,95 solid): 22 ticks
      // for one block. A step UP is left to the jump gates as before; the
      // staircase wobble that ruled out pre-emptive waypoints came from
      // steps, and a flat squeeze the physics refuses head-on has no
      // straight alternative to wobble away from.
      let squeezed = false
      const walkNode = (nextPoint as { parkour?: boolean }).parkour !== true &&
        nextPoint.toBreak.length === 0 && nextPoint.toPlace.length === 0
      // Not under a live cut: its chord is verified, and a waypoint pushed in
      // front of path[0] would override the heading it is walking.
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
              clearWedge() // the waypoint IS the escape; do not also nudge
              path.unshift(waypoint)
              nextPoint = waypoint
              dx = nextPoint.x - p.x
              dy = nextPoint.y - p.y
              dz = nextPoint.z - p.z
            }
          }
        }
      }

      // Climb aid (improvement): with the next node directly ABOVE, the look
      // direction is degenerate — steer into an adjacent solid block instead
      // so 'forward' presses against it. Both vanilla and prismarine-physics
      // ascend climbables via horizontal collision, so without a wall to
      // press the bot would drift off a ladder/vine column.
      // Measured on the step to path[0]: a cut aimed several nodes on would
      // never read as the degenerate straight-up case, and the bot would
      // drift off the column instead of pressing into it.
      // Also while HANGING on a climbable with the next node higher and off
      // to the side — a spiral-ladder transfer (moveGen.climbTransfers): the
      // way round a pillar corner is up the current ladder to its top edge
      // (a ladder's collision top is standable) and a step from there, not a
      // diagonal drift out of the ladder cell while still low. Traced on
      // parkouradv1's column: the north→west transfer, flown from the top
      // edge, worked; the west→south one, steered diagonally from a low
      // catch, left the cell at 261.6 and fell.
      let climbing = false
      if (nextPoint.y > p.y + 0.1) {
        const feet = bot.blockAt(p) as BlockLike | null
        const hanging = (bot.entity as { onGround?: boolean }).onGround !== true
        const straightUp = Math.abs(nodeDx) < 0.2 && Math.abs(nodeDz) < 0.2
        if (feet && (feet.type === ladderId || feet.type === vineId) &&
            (straightUp || (hanging && nextPoint.y > p.y + 0.3))) {
          const fx = Math.floor(p.x)
          const fy = Math.floor(p.y + 0.001)
          const fz = Math.floor(p.z)
          for (const [ox, oz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
            const nb = bot.blockAt(new Vec3(fx + ox, fy, fz + oz)) as BlockLike | null
            if (nb && nb.boundingBox === 'block') {
              dx = ox
              dz = oz
              climbing = true
              break
            }
          }
        }
      }

      // Side-wall standoff (improvement). A body FLUSH with a wall it is
      // walking ALONG cannot sprint — the gate below refuses it, and rightly:
      // the server refuses those positions — and a heading at a node on the
      // same line never peels it off, because the node is 0.2 from the wall
      // and the heading's sideways component is a fraction of that. The
      // wall-slide further down only fires on a wall AHEAD. Traced on the
      // arena's tunnel1: the turn into a 2-high corridor left the body
      // against one wall and it walked 57 ticks of the corridor unsprinted;
      // climb2 the same on its landings. Steer away by the same standoff the
      // slide uses, on the chord being walked (a corner cut included), and
      // only where the ground continues that way. Before the sprint gate, so
      // its one-tick look-ahead probes the heading actually taken.
      if (!climbing && !swimming && (nextPoint as { parkour?: boolean }).parkour !== true &&
          dy <= STEP_UP_MIN && Math.hypot(dx, dz) > 0.15) {
        const floorAway = (ox: number, oz: number): boolean =>
          geometry.playerCollides(bot, p.x + ox, p.y - 0.55, p.z + oz)
        const negX = geometry.playerCollides(bot, p.x - SIDE_TOUCH, p.y, p.z)
        const posX = geometry.playerCollides(bot, p.x + SIDE_TOUCH, p.y, p.z)
        const negZ = geometry.playerCollides(bot, p.x, p.y, p.z - SIDE_TOUCH)
        const posZ = geometry.playerCollides(bot, p.x, p.y, p.z + SIDE_TOUCH)
        if (negX !== posX && Math.abs(dz) >= Math.abs(dx)) {
          const away = negX ? 1 : -1
          if (floorAway(away * 0.3, 0)) dx += away * Math.abs(dz) * WALL_STANDOFF
        } else if (negZ !== posZ && Math.abs(dx) >= Math.abs(dz)) {
          const away = negZ ? 1 : -1
          if (floorAway(0, away * 0.3)) dz += away * Math.abs(dx) * WALL_STANDOFF
        }
      }

      // Sprint gate. A vanilla client cancels sprinting the moment a
      // collision deflects it (LocalPlayer.aiStep). Holding sprint against a
      // wall is not cosmetic: the server REFUSES every position a sprinting
      // client claims there and teleports the bot back — measured, back to
      // back, on the same block: walking along the wall covered 7.1 blocks
      // with zero corrections, sprinting covered 0.0 with 26. Because the bot
      // cannot move it never stops touching the wall, so this is a livelock,
      // and the futility timer cannot break it either: every correction
      // re-arms the forced-move grace.
      //
      // The probe is geometric, and looks a sprinting tick AHEAD as well as at
      // the body, because reacting to isCollidedHorizontally is two ticks too
      // late: by then the server already has the bot down as sprinting, and
      // the touching position it refuses has already been sent. Reacting
      // instead of predicting was worth 7 corrections on this route; looking
      // ahead took it to none.
      const slideLen = Math.hypot(dx, dz)
      const againstWall = !climbing && (geometry.nearWall(bot, SPRINT_WALL_MARGIN) ||
        (slideLen > 0.01 && geometry.playerCollides(
          bot, p.x + dx / slideLen * SPRINT_TICK, p.y, p.z + dz / slideLen * SPRINT_TICK)))
      // The one exception is a parkour jump already in flight: it was
      // approved on a rollout that held sprint the whole way, and a corner
      // nick mid-arc is exactly what that model tolerates. Cutting sprint
      // there would land the bot short of a node the planner routed through.
      const flyingParkour = (nextPoint as { parkour?: boolean }).parkour === true &&
        (bot.entity as { onGround?: boolean }).onGround !== true
      const maySprint = stateMovements.allowSprinting && (flyingParkour || !againstWall)

      // Wall-slide steering. Aiming straight at the next node when a wall is
      // in the way makes the physics clamp the move, and the server refuses a
      // clamped position that also slid along the wall: on the same corner,
      // aiming diagonally into it produced 25 corrections and 0.01 blocks of
      // travel, while aiming along the wall produced none and 7.1 blocks. A
      // player walks along the wall and turns at the corner; so do we.
      //
      // Walking steps only. A jump is flown on the heading the physics
      // rollout approved, and re-steering it mid-flight would land the bot
      // somewhere the planner never routed through.
      //
      // Judged on the step being walked, never skipped because a cut is live:
      // with a cut live, `hypot(dx, dz)` is several blocks, and when this
      // block gated on that the heading stayed aimed diagonally into the
      // face, which is precisely the 25-corrections state it was written for.
      // A wall in the way also ENDS the cut: geometry has just contradicted
      // the swept line the target was chosen on.
      //
      // Under a cut the probes follow the CHORD's axes, not the step to
      // path[0] (which a taut cut leaves off to the side), and only the axes
      // the chord actually moves along (SLIDE_AXIS_MIN): a line running a few
      // degrees off a wall must not read as walking into it, or the cut
      // flickers off and on against every wall it passes.
      const slideLive = cutTarget !== null
      const slideDx = slideLive ? dx : nodeDx
      const slideDz = slideLive ? dz : nodeDz
      if (!climbing && (nextPoint as { parkour?: boolean }).parkour !== true &&
          dy <= STEP_UP_MIN && (slideLive || Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH)) {
        const slideLen = Math.hypot(slideDx, slideDz) || 1
        const sx = slideLive && Math.abs(slideDx) / slideLen < SLIDE_AXIS_MIN ? 0 : Math.sign(slideDx)
        const sz = slideLive && Math.abs(slideDz) / slideLen < SLIDE_AXIS_MIN ? 0 : Math.sign(slideDz)
        const xBlocked = sx !== 0 && geometry.playerCollides(bot, p.x + sx * SLIDE_PROBE, p.y, p.z)
        const zBlocked = sz !== 0 && geometry.playerCollides(bot, p.x, p.y, p.z + sz * SLIDE_PROBE)
        // The standoff is a fraction of the ALONG-wall component, and it
        // REPLACES the into-wall one. With nothing to slide along — a step
        // square-on to the face — that turns a heading of (0.02, 1.0) into
        // (0.02, -0.005) and hands Math.atan2 a direction made of float
        // noise, so the bot faces sideways at a wall instead of at its node.
        // There is no slide to steer there; leave the heading alone and let
        // the wedge recovery below deal with it if nothing moves.
        if (zBlocked || xBlocked) {
          cutTarget = null
          gatePath = path
          dx = nodeDx
          dz = nodeDz
        }
        // Only where the ground extends AWAY from the wall. The standoff
        // walked the bot off a ladder's 3/16 top edge on parkouradv1's
        // spiral column: the column blocked the step, the standoff steered
        // away from it, and away from it was air.
        const floorAway = (ox: number, oz: number): boolean =>
          geometry.playerCollides(bot, p.x + ox, p.y - 0.55, p.z + oz)
        if (zBlocked && !xBlocked && Math.abs(dx) >= 0.15 && floorAway(0, -sz * 0.3)) dz = -sz * Math.abs(dx) * WALL_STANDOFF
        else if (xBlocked && !zBlocked && Math.abs(dz) >= 0.15 && floorAway(-sx * 0.3, 0)) dx = -sx * Math.abs(dz) * WALL_STANDOFF
      }

      // Air, before anything else can spend it. Vanilla gives 300 ticks of it
      // and then 2 HP a second, and the planner has no idea — it will happily
      // route 40 blocks of submerged corridor, which at swim speed is 20 s
      // against a 15 s lung. Nothing downstream can rescue that, so the
      // executor watches its own breath: below a third of it, the goal stops
      // mattering until the bot has surfaced. Straight up is always the
      // fastest way out (3.5 blocks/s against 2 swimming sideways), and the
      // futility timer is held off because surfacing IS progress even though
      // no node is being reached. It outranks the wedge recovery: a bot both
      // stuck and out of air needs the air first.
      const air = (bot as { oxygenLevel?: number }).oxygenLevel
      if (swimming && air !== undefined && air <= AIR_RESERVE) {
        bot.clearControlStates()
        bot.setControlState('jump', true)
        lastNodeTime = performance.now()
        return
      }

      // ── when the plan and the world disagree ─────────────────────────────
      //
      // Every rollout above answers "does this work?" against
      // prismarine-physics, and prismarine-physics is not the authority. When
      // the two disagree the bot stands still holding a plan it believes in,
      // the futility timer replans, the identical plan comes back, and it
      // stands still again: measured on the arena's climb1 staircase, 55 s in
      // one spot across sixteen `stuck` resets, one cell from a node it was
      // right to want.
      //
      // So the recovery triggers on the SYMPTOM — the body has not moved —
      // rather than on any prediction about why, and then works down a ladder
      // of increasingly expensive answers: turn to make the jump, go round
      // the open corner, make room. Each is simulated before it is used, and
      // none of them run while the bot is making progress.
      //
      // A jump taken at an angle is flown at that angle: re-homing on the
      // node mid-arc would undo the very thing that made it possible.
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

      if (!squeezed && wedged) {
        if (driveAngledJump(nextPoint, nodeDx, nodeDz)) { execBranch = 'angled'; futile(swimming); return }
        if (driveRecovery(nextPoint)) { execBranch = 'recovery'; futile(swimming); return }
      }

      bot.look(Math.atan2(-dx, -dz), 0)
      bot.setControlState('forward', true)
      bot.setControlState('jump', false)
      // A flat walking step (no rise to jump for): the angled-walk branch
      // below may steer it round a clipped corner instead of jumping it.
      const flatWalkStep = walkNode && !swimming && !climbing && dy <= STEP_UP_MIN &&
        (cutTarget !== null || Math.hypot(nodeDx, nodeDz) <= WALK_STEP_REACH)
      let walkBias: number | null = null
      // Sneak is only ever engaged by the corner-creep branch below; every
      // other branch (including the jump itself) must take off un-sneaked.
      bot.setControlState('sneak', false)
      // Back/strafe are only ever engaged by the wedge recovery.
      bot.setControlState('back', false)
      bot.setControlState('left', false)
      bot.setControlState('right', false)


      if (swimming) {
        // Swim to the level the plan is on, and STAY there.
        //
        // Upstream holds jump for as long as the body is wet, which is a
        // sensible way to not drown and a terrible way to follow a path: the
        // search has no vertical water move at all (moveUp refuses a liquid
        // feet cell, moveDown refuses to go under), so every water node it
        // emits sits at one planned level — and an executor that swims up
        // unconditionally climbs out of its own plan. In a waterfall it rides
        // the column: on the arena's basic1 the bot floated up a stream it
        // was supposed to cross, replanned from higher each time, and never
        // arrived.
        //
        // The rule is the one already used for bubble columns, with the
        // threshold at the current level rather than above it: press up while
        // the node is at or above the body, let buoyancy go when the body has
        // drifted above it. That holds a surface swim level, climbs to a node
        // above, and sinks toward one below, with no separate state.
        hopHold = false
        const feet = bot.blockAt(bot.entity.position) as { type: number } | null
        const inColumn = feet !== null && feet.type === bubbleColumnId
        // A down-column does the vertical work itself, and swim-up in one is
        // strong enough to stall the descent outright — so a column only gets
        // help when the node is clearly above.
        bot.setControlState('jump', nextPoint.y > bot.entity.position.y + (inColumn ? 0.25 : -0.1))
        bot.setControlState('sprint', false)
        execBranch = 'swim'
      } else if (maySprint && physics.canStraightLine(gatePath, true)) {
        // Sprint-hop (improvement, allowSprintHop): the plain sprint upstream
        // uses here is the SLOWEST way a bot with a jump key crosses open
        // ground — 5.56 blocks/s against 6.97 hopping, measured on the arena
        // server, because the jump preserves the sprint boost ground friction
        // eats. The decision is re-taken on every take-off (and only there —
        // it is the one tick it can be acted on), from a rollout of both
        // gaits down this same path: the hop has to actually get further,
        // without losing height, or the bot keeps its feet.
        if ((bot.entity as { onGround?: boolean }).onGround === true) {
          // Never take off INTO a turn (HOP_TURN_MAX). A hop carries the
          // body's momentum for a whole arc with 0.02 a tick of air control,
          // so a take-off whose heading differs from the velocity lands off
          // the line by most of a block: traced on the arena's simple1, a
          // landing at full sprint onto a 45° line hopped at once, flew at
          // ~30°, came down 0.64 off the line in the gap between two cells of
          // a one-wide diagonal ledge and fell seven blocks. The rollout did
          // not catch it because its yaw is set instantly. Sprinting the turn
          // costs a few ticks — ground friction realigns the velocity in
          // three or four — and the hop resumes once aligned.
          const v = bot.entity.velocity
          const speed = Math.hypot(v.x, v.z)
          const head = Math.hypot(dx, dz)
          const turning = speed > 0.1 && head > 1e-6 &&
            (v.x * dx + v.z * dz) / (speed * head) < HOP_TURN_COS
          hopHold = stateMovements.allowSprintHop && !turning &&
            physics.sprintHopBetter(gatePath, stateMovements.allowLowCeilingHop)
        }
        // PRESS on the landing tick, never hold. prismarine-physics charges a
        // held jump key a 10-tick re-jump cooldown (`autojumpCooldown`) and
        // clears it the instant the key comes up, so a bonked arc — one that
        // lands after 5 ticks because the ceiling is low — spends the rest of
        // the cooldown on the ground, bleeding the sprint boost into friction.
        // Measured on flat stone with the same physics: under a 2-block roof,
        // sprint 5.59 b/s, hold-jump 6.50, press-on-landing 9.68 — faster than
        // bunny-hopping under open sky. Where the arc already outlasts the
        // cooldown the two cadences are identical tick for tick (7.05 both, 34
        // jumps each), so this is never the slower choice and is not optional.
        bot.setControlState('jump', hopHold && (bot.entity as { onGround?: boolean }).onGround === true)
        bot.setControlState('sprint', true)
        execBranch = hopHold ? 'hop' : 'sprint'
      } else if (flatWalkStep && (walkBias = physics.bestWalkHeading(gatePath, maySprint)) !== null) {
        // Angled walk (improvement): a flat step whose straight line grinds
        // on a corner is WALKED round it, before the jump gates get it. See
        // PhysicsSim.bestWalkHeading; the offset is re-solved every tick, so
        // the moment the straight line works again the branch above wins.
        hopHold = false
        // Off the line being walked: the chord under a live cut, else the step.
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
        // Improvement: creep to the takeoff CORNER before a standing parkour
        // jump. The sims above test jump-now from the CURRENT position; a
        // player walks to the lip first — for diagonal pillar hops all the
        // way to the corner overhang (the planner's per-axis flightNeeded
        // model, parkourEnvelope.ts, assumes the widest axis reaches
        // TAKEOFF_STAND). Sneak while creeping: the vanilla edge-guard makes
        // walking off the block physically impossible, so the overhang
        // stance is safe; the jump fires from another branch with sneak off.
        let creep = false
        // Take-off geometry, shared by the creep and the run-up below: the
        // support's half-width class, the creep cap along the flight line,
        // and how far along that line the body already is (from the cell the
        // approach started in). takeoffCap stays 0 when there is none.
        let takeoffHalf = 0.5
        let takeoffCap = 0
        let takeoffProj = 0
        /** Body offset from the take-off cell centre, and the per-axis limit a narrow support allows. */
        let takeoffPx = 0
        let takeoffPz = 0
        let takeoffLip = Infinity
        const parkourNode = (nextPoint as { parkour?: boolean }).parkour === true
        const grounded = (bot.entity as { onGround?: boolean }).onGround === true
        if (creepNode !== nextPoint) {
          creepNode = nextPoint
          creepCell = null
        }
        if (parkourNode && grounded) {
          const len = Math.sqrt(dx * dx + dz * dz)
          if (len > 0.01) {
            // Measured against the cell the creep STARTED in, not against
            // whatever cell the body is in now. The overhang stance puts the
            // hitbox centre past the lip by design, so once it crosses the
            // boundary `Math.floor(p)` names the NEXT cell, the offset flips
            // sign, and the bot reads as "not crept far enough" again — it
            // walks off the edge it was carefully standing on.
            //
            // And the cell is the TAKE-OFF cell — the node just retired — not
            // whatever cell the body is in when the creep begins. After a
            // run-in the body can already overhang the block's edge at that
            // moment, and anchoring on the neighbouring cell puts every offset
            // a block out: the sneak zone never arrives and the body walks off
            // the lip at speed. Traced on the arena's basic3: the creep began
            // at x −137.02 on the block −137..−136, anchored on −138, and the
            // bot fell into the gap it was meant to jump.
            if (creepCell === null) {
              const anchor = prevRetired !== null && Math.hypot(prevRetired.x - p.x, prevRetired.z - p.z) <= 1.2
                ? prevRetired
                : p
              creepCell = { x: Math.floor(anchor.x), z: Math.floor(anchor.z) }
            }
            const px = p.x - (creepCell.x + 0.5)
            const pz = p.z - (creepCell.z + 0.5)
            // On a narrow support (fence post, head, pot) the lip is closer:
            // the same reduced credit the planner used (parkourEnvelope.ts).
            // Without the cap the sneak guard stops the body at the post's
            // edge short of the full-block creep target, and the creep never
            // ends. The support is the block the feet rest on — probed at
            // feet − 0.2 like the physics does, then one lower, because a
            // fence's 1.5 top pokes into the cell above its own.
            let half = 0.5
            for (const oy of [-0.2, -1.2]) {
              const sup = bot.blockAt(new Vec3(creepCell.x, Math.floor(p.y + oy), creepCell.z)) as BlockLike | null
              if (sup !== null && sup.shapes.length > 0) {
                half = CATCH_HALF[topCatchClass(sup.shapes)]
                break
              }
            }
            takeoffHalf = half
            // Never past the support's PHYSICAL limit less a margin. The
            // planner's lip credit (TAKEOFF_NARROW_MARGIN, 0.28) is two
            // centimetres inside the 0.30 the hitbox can overhang — and a
            // fence post's is past it — which is one server position packet
            // from a fall: a packet clears onGround, the sneak edge-guard
            // needs onGround, and the unguarded tick walks off. Traced on
            // parkouradv1 (both a head and a post). The margin is more than
            // a sneaking tick moves, so a single unguarded tick stays on.
            const physicalLip = half + 0.3 - NARROW_LIP_SAFETY
            takeoffCap = Math.min(TAKEOFF_STAND, half + TAKEOFF_NARROW_MARGIN, physicalLip) * len / Math.max(Math.abs(dx), Math.abs(dz))
            takeoffProj = (px * dx + pz * dz) / len
            takeoffPx = px
            takeoffPz = pz
            creep = takeoffProj < takeoffCap
            // On a narrow support the limit is PER AXIS: the projection on a
            // mostly-z flight line read 0.29 while the body's x offset was
            // already 0.56 — past the head's 0.55 of support (parkouradv1);
            // the same for the back-off, which fell off the far side.
            if (half < 0.5) {
              takeoffLip = physicalLip
              if (Math.abs(px) >= physicalLip || Math.abs(pz) >= physicalLip) creep = false
            }
          }
        }
        // Sneak only near the cap (CREEP_SNEAK_ZONE); walk the rest of the
        // way — on a full block. A narrow support (head, pot, post) is
        // crossed entirely under sneak: its cap is under half a block from
        // its centre, and a walking tick there is the difference between
        // standing on it and falling off it (traced on parkouradv1).
        const narrowSupport = takeoffHalf < 0.5
        // Landing brake on a narrow support. A body that has just landed on
        // a post or a head with its flight speed still in it slides 2.2
        // times that speed before friction stops it — 0.57 blocks from a
        // sprint-jump, past the 0.425 a post supports. The planner meant
        // such a landing to re-jump on the landing tick (momentum), and
        // when the gates sign that the branches above fire it; when they do
        // not, the run-up used to start here with `back` UNDER SNEAK, which
        // is a third of a brake, and the body went off the far edge (arena
        // parkouradv1, landing at +0.11 on the post, gone by +0.43 three
        // ticks later). So: full `back`, no sneak, along the velocity, until
        // the speed is walking-slow; the line-up starts from a standing body.
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
        // Sneak near ANY edge of the take-off block, not only near the lip
        // on the flight line: a body left overhanging one edge (a given-up
        // jump, a corner landing) whose new flight line points elsewhere
        // reads as "not yet in the sneak zone" along that line and walks
        // off the edge it is already standing on.
        const nearEdge = Math.max(Math.abs(takeoffPx), Math.abs(takeoffPz)) >= 0.25
        const creepSneak = creep && (narrowSupport || nearEdge || takeoffProj >= takeoffCap - CREEP_SNEAK_ZONE)
        // Lip brake: a body at the lip with speed slides 2.2 times its
        // velocity before friction stops it — off the block, from a sprint.
        // Sneaking there costs nothing and the edge guard makes leaving the
        // block impossible; the jump itself fires from another branch with
        // sneak off.
        const atLip = stateMovements.allowRunUp && parkourNode && grounded && takeoffCap > 0 && takeoffProj >= takeoffCap - 0.5
        // A body already in the air keeps flying the heading it took off on.
        // Whatever the gates think from here, releasing forward mid-flight
        // throws away the air control the jump was approved with and lands
        // the bot short of a node the planner routed through — and there is
        // nothing else this branch could usefully do about a bot that is not
        // touching the ground.
        const inFlight = airborne
        // Caught on a ladder or vine mid-flight (the extended repertoire's
        // gap-jump into a climbable): a body on a climbable slides DOWN at
        // 0.15 a tick until it presses into the wall, and a catch planned at
        // the lowest ladder cell with the wall on the cell's far side slides
        // out of the bottom of it first — traced on parkouradv1's stair →
        // ladder (0,-1,5): in at 259.16, wall three ticks away, out at
        // 258.64. Sneaking on a climbable holds the height (vanilla and
        // prismarine-physics alike) and does not stop the climb: forward into
        // the wall still lifts the body.
        let caughtOnClimbable = false
        if (inFlight) {
          const feetBlock = bot.blockAt(p) as { type: number } | null
          caughtOnClimbable = feetBlock !== null && (feetBlock.type === ladderId || feetBlock.type === vineId)
        }

        // Riser standoff: take the pace back a step needs, and ONLY when the
        // step actually needs it.
        //
        // The server refuses every position where the body touches a face
        // while the feet are below its top, so a jump has to start far enough
        // back that the arc clears the lip first. MEASURED on one arena
        // riser, identical controls, only the starting clearance changed:
        // 0.00-0.15 gave 17-19 corrections and never climbed; 0.20 climbed
        // with none. The grind guard in physics.ts already draws that line —
        // it is why every gate above has just refused — so the trigger here
        // is simply "the gates said no to a step up", not a second clearance
        // constant of its own.
        //
        // That distinction is the whole difference between a bot that climbs
        // and one that looks like it is glitching. A walking jump started at
        // 0.2 lands on the next step at 0.2, so a clean staircase chains with
        // no correction at all; an explicit threshold set anywhere above that
        // landing clearance fires on EVERY step instead, and the bot visibly
        // shuffles backwards before each one.
        const stepBack = !creep && !inFlight && !climbing && !swimming &&
          dy > STEP_UP_MIN && (nextPoint as { parkour?: boolean }).parkour !== true &&
          physics.canNudge({ back: true }, 3)

        // Run-up line-up (improvement): a parkour jump no gate authorises
        // from where the body stands is flown the way a player flies it —
        // from the REAR of the support, sprinting across it and jumping at
        // the lip (the planner's run-length envelope, parkourEnvelope.ts
        // J_RUN: even 0.4 blocks of run adds half a block of flight, a post
        // gives 0.8, a full block 1.4). The planner says which jumps NEED
        // that (Move.run, META_RUN: the standing row cannot fly them); a jump
        // from rest at the corner can never make one of those, and creeping
        // there is how the arena's basic3 stood 2.5-3.2 s in front of a (4,4)
        // diagonal until the futility timer replanned. So: back off to the
        // rear of the run (a post: its rear lip, under sneak), then sprint
        // in while the ordinary gates look for the take-off tick — the
        // delayed-jump rollout (canStraightLine → canSprintJump(path, i))
        // takes over the moment a jump within its horizon lands, and this
        // code is not reached again. A line-up that reaches the lip without
        // one is over; run-flagged nodes get three, any other parkour node
        // one (the old single run-back), and after that the creep resumes
        // and the futility timer has the last word.
        //
        // The old run-back was armed ONCE per node, at whatever tick the node
        // became path[0] — usually the landing of the previous jump, where a
        // step back is refused (the body is still settling) or wasted (the
        // body is not yet in the take-off cell) — and then never again.
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
          // A narrow support gets ONE line-up and no sprint-in phase: its
          // run is the support itself (0.8 on a post), the delayed-jump
          // gates cover that from the rear lip, and a forward press there
          // is a fall. A full block gets RUN_UP_ATTEMPTS for a run-flagged
          // node, one otherwise (the old single run-back).
          const narrow = narrowSupport
          // Without allowRunUp this is the single run-back the executor
          // always had (phase 1 once, no sprint-in, no brake).
          const lineUp = stateMovements.allowRunUp
          const maxAttempts = narrow || !lineUp ? 1 : (nextPoint as Move).run ? RUN_UP_ATTEMPTS : 1
          if (runUpPhase === 0 && runUpAttempts < maxAttempts) {
            runUpAttempts++
            runUpPhase = 1
            runUpTicks = 0
            runUpLastPos = null
          }
          if (runUpPhase === 1) {
            // Backing: to the rear point, or as far as the physics says a
            // step back is safe, or until the body stops moving (a post's
            // edge guard, a wall).
            const rear = narrow ? -(takeoffHalf + 0.3 - NARROW_LIP_SAFETY) : -RUN_UP_REAR
            // A sneaking body moves ~0.06 a tick, a walking one 0.2: the
            // stall test is sized to the gait, or a narrow back-off ends
            // after two ticks and leaves half the run on the table.
            const stalled = runUpLastPos !== null && runUpTicks >= 2 &&
              p.distanceTo(runUpLastPos) < (narrow ? 0.02 : WEDGE_MOVE)
            runUpLastPos = p.clone()
            const axisOut = narrow && (Math.abs(takeoffPx) >= takeoffLip || Math.abs(takeoffPz) >= takeoffLip)
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
            // Sprinting in: well short of the lip (the gates' own horizon
            // is six ticks, and the brake below holds the body there) and
            // never onto ground the physics cannot find.
            if (runUpTicks < RUN_UP_MAX_TICKS && takeoffProj < takeoffCap - 0.35 &&
                physics.canNudge({ forward: true }, 3)) {
              runUpTicks++
              runIn = true
            } else {
              runUpPhase = 0 // this line-up is over: creep from here, or line up again
            }
          }
        }
        if (runBack || runIn) creep = false

        // Line up ON the flight line, not merely along it (LINE_UP_AHEAD).
        // The run-back and run-in steered straight at the node from wherever
        // the body stood, so a body that reached the take-off cell a third
        // of a block off the centre line backed off and ran in a third of a
        // block off it, and a lip take-off onto a one-block target is not
        // there. Steer at a point up the centre line from the body's foot
        // (pure pursuit); when backing, `back` moves away from the look
        // point, so look at the rear point's mirror image through the body
        // and the body converges as it backs.
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

        // A take-off nothing will sign. Line-ups exhausted, body at the lip,
        // every gate refusing, tick after tick: give the jump up now, ban
        // its landing cell and re-solve, rather than sit out the futility
        // timer for a replan that brings the same jump back (LIP_WAIT_TICKS).
        const attemptsDone = runUpAttempts >= (narrowSupport || !stateMovements.allowRunUp ? 1 : (nextPoint as Move).run ? RUN_UP_ATTEMPTS : 1)
        // A creep the edge guard is holding still counts as waiting: the lip
        // it wants is past the edge it is on.
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

        bot.setControlState('forward', creep || inFlight || runIn)
        bot.setControlState('back', stepBack || runBack)
        bot.setControlState('sneak', (creep && creepSneak) || caughtOnClimbable || runSneak || (atLip && !runIn))
        bot.setControlState('sprint', (flyingParkour && !caughtOnClimbable) || runIn)
        execBranch = caughtOnClimbable ? 'catch' : inFlight ? 'inflight' : runBack ? 'runback' : runIn ? 'runin' : creep ? 'creep' : stepBack ? 'stepback' : 'wait'
      }

      futile(swimming)
    }

    bot.on('physicsTick', monitorMovement)
    if (execTraceFile !== null) {
      // Server-side overwrites of the bot's own velocity/position, so a
      // trace can tell a physics decision from a packet that undid it.
      const client = (bot as unknown as { _client: { on: (ev: string, fn: (p: Record<string, unknown>) => void) => void } })._client
      for (const ev of ['entity_velocity', 'sync_entity_position', 'position']) {
        client.on(ev, (packet) => {
          if (ev !== 'position' && packet.entityId !== (bot.entity as { id?: number }).id) return
          execTraceBuf.push(JSON.stringify([execTick, 'packet', ev, packet.velocity ?? [packet.dx, packet.dy, packet.dz], packet.x ?? null, packet.y ?? null, packet.z ?? null, packet.flags ?? null]))
        })
      }
      bot.on('physicsTick', () => {
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
          n ? [+n.x.toFixed(1), +n.y.toFixed(1), +n.z.toFixed(1), (n as { parkour?: boolean }).parkour ? 1 : 0] : null,
          [+v.x.toFixed(3), +v.y.toFixed(3), +v.z.toFixed(3)],
          +lastTickMs.toFixed(2),
          execBranch === 'sprint' ? ((physics as { hopRefusal?: string }).hopRefusal ?? '') : ''
        ]))
        execBranch = 'idle'
        // Flush on the batch, and on every idle tick (a race's last ticks
        // were lost in the buffer when the process exited on arrival).
        if (execTraceBuf.length >= 40 || (execTraceBuf.length > 0 && path.length === 0)) {
          fs.appendFileSync(execTraceFile, execTraceBuf.join('\n') + '\n')
          execTraceBuf.length = 0
        }
      })
    }
  }
}

/** The plugin instance for `bot.loadPlugin(pathfinder)` — upstream-identical usage. */
export const pathfinder = createPathfinder()
