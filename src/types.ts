import type { Vec3 } from 'vec3'

/**
 * LUT flag bits, one byte per block state id.
 *
 * These mirror the exact pseudo-block classification mineflayer-pathfinder
 * computes per `getBlock()` probe (movements.js), precomputed once per
 * (minecraft version × movements profile) so a probe is a single typed-array
 * read instead of a world hash lookup + object allocation.
 */
// A plain const object rather than a `const enum` — const enums poison
// published .d.ts files for consumers compiling with isolatedModules.
export const LutFlags = {
  /**
   * Walk-through safe: bounding box 'empty' / climbable / carpet type, and
   * not a blocksToAvoid. NOTE: boundingBox is TYPE-level in prismarine-block
   * (an open fence gate still reports 'block'), and upstream's classification
   * inherits that — we mirror it exactly, quirks included.
   */
  SAFE: 1 << 0,
  /** Can stand on: bounding box 'block' and not a fence-like type (top > 1). */
  PHYSICAL: 1 << 1,
  LIQUID: 1 << 2,
  CLIMBABLE: 1 << 3,
  /**
   * Upstream `openable` (non-iron fence gates) AND this state has collision
   * shapes (i.e. the gate is closed): traversable via activate when
   * canOpenDoors. Upstream checks `openable && shapes.length !== 0` at move
   * time; we bake the shapes check per state.
   */
  OPENABLE_GATE: 1 << 4,
  /**
   * Improvement over upstream: a CLOSED non-iron door — traversable via
   * activate when canOpenDoors && canOpenRealDoors (upstream only ever opens
   * fence gates).
   */
  DOOR_CLOSED: 1 << 5,
  /**
   * Improvement over upstream: an OPEN non-iron door — treated as passable
   * when canOpenDoors && canOpenRealDoors (upstream blocks on it because the
   * type-level bounding box stays 'block').
   */
  DOOR_OPEN: 1 << 6,
  /**
   * Improvement over upstream: an OPEN non-iron fence gate (no collision
   * shapes) — passable when canOpenDoors && canOpenRealDoors. Upstream
   * blocks on it (type-level bbox again), making gates effectively
   * one-shot on modern versions.
   */
  GATE_OPEN: 1 << 7,
} as const

/**
 * Rare-block auxiliary classification, kept OUT of the main flags byte (all
 * 8 bits are taken) in an optional third grid that exists only when the
 * profile opts into the feature — zero cost and byte-identical behavior
 * otherwise. Currently: bubble columns (improvement over upstream, which
 * treats them as plain air).
 */
export const LutSpecial = {
  /** bubble_column[drag=false] (soul sand below) — pushes entities up. */
  BUBBLE_UP: 1,
  /** bubble_column[drag=true] (magma below) — drags entities down. */
  BUBBLE_DOWN: 2,
  /** vine — climbable only with an adjacent solid block to press against
   * (vanilla collision climb; prismarine-physics ascends only on
   * horizontal collision, so a free-hanging curtain is unclimbable). */
  VINE: 4,
  /** slime_block — a drop onto it rebounds (improvement, marked only when
   * the profile enables allowParkourExtended: slime-bounce moves). */
  SLIME: 8,
  /**
   * Bottom stair (improvement, allowParkourExtended): its lower slab
   * [0,0,0,1,0.5,1] is full-footprint whatever the facing, so a parkour
   * flight can catch the block half a block below its top and walk up the
   * step — half a block more usable flight than a full-block landing. Marked
   * only on straight/corner bottom stairs (top-half stairs stay full-top).
   */
  STAIR: 16
} as const

/**
 * Snapshot cell = 2 bytes: flags (LutFlags) + a packed height byte.
 *
 * Height byte layout: bits 0–5 = collision-top height in 1/32 blocks
 * (vanilla tops max out at 1.5 = 48, so 6 bits are exact), bits 6–7 = the
 * TOP-CATCH class — how wide the centered landing surface at the top of the
 * block is, from `topCatchClass` (shapes.ts): 0 = full/wide (≥0.4 half),
 * 1 = head/wall class (0.25), 2 = fence-post/pot class (0.125), 3 = too
 * small to stand on. Consumers must mask: height = (byte & 63) / 32.
 */
export interface SnapshotMeta {
  /** World-space minimum corner of the AABB (inclusive). */
  x0: number
  y0: number
  z0: number
  /** Grid dimensions. */
  w: number
  h: number
  l: number
  /** bot.game.minY — getLandingBlock stops scanning below this. */
  worldMinY: number
  /** PROCESS-globally unique generation (cache key across consumers). */
  generation: number
  /** Bumped on every in-place cell patch — (generation, patchCount)
   * uniquely identifies snapshot content for residency caches. */
  patchCount: number
}

export interface SnapshotBuffers {
  meta: SnapshotMeta
  /** flags[idx] — LutFlags byte per cell. SharedArrayBuffer-backed. */
  flags: Uint8Array
  /** heights[idx] — collision-top of the cell, in 1/32 blocks above the cell floor. */
  heights: Uint8Array
  /**
   * Sparse entity-intersection grid, mirroring Movements.entityIntersections:
   * pairs of (cellIdx, weight). Empty when allowEntityDetection is off.
   */
  entityIdx: Int32Array
  entityWeight: Int32Array
  /** Sparse exclusion-step weights (cellIdx, weight); empty unless exclusionAreasStep set. */
  exclusionIdx: Int32Array
  exclusionWeight: Int32Array
}

/** Serializable subset of a Movements profile — everything the solver core needs. */
export interface MovementsConfig {
  allowSprinting: boolean
  allowParkour: boolean
  /**
   * Improvement over upstream: extended parkour — sprint-jumps to diagonal
   * and long offsets, up (+1) and drop landings, and gap-jumps that catch a
   * ladder / water / bubble column (docs/ExtendedParkour.md). Default false;
   * needs allowParkour + allowSprinting.
   */
  allowParkourExtended: boolean
  canOpenDoors: boolean
  /** Improvement toggle: also open real (non-iron) doors, not just gates. Default true. */
  canOpenRealDoors: boolean
  maxDropDown: number
  infiniteLiquidDropdownDistance: boolean
  liquidCost: number
  entityCost: number
  /** Dig moves enabled (default false). Cost model is upstream-identical. */
  canDig: boolean
  digCost: number
  dontCreateFlow: boolean
  dontMineUnderFallingBlock: boolean
  /**
   * Improvement over upstream: ride bubble-column elevators (soul sand up,
   * magma down). Default false — upstream treats columns as plain air.
   */
  useBubbleColumns: boolean
  /** Cost per block of column ride (default 1 = admissible vs the |dy|
   * heuristic; the true ride is faster — ~0.31 up / ~0.72 down). */
  bubbleCost: number
  /**
   * Blocks of flight a planned parkour jump must keep short of the physics
   * limit (parkourEnvelope.ts ENVELOPE_SAFETY_MARGIN = 0.1 default). 0 plans
   * the frame-tight jumps a practised player makes; the executor's rollout
   * still gates every take-off. Optional for older serialized configs.
   */
  parkourSafetyMargin?: number
}

/** Per-state dig auxiliaries (only consulted when a block is unsafe). */
export const DigFlags = {
  /** gravityBlocks type (sand/gravel) — blocks below it can't be mined
   * under dontMineUnderFallingBlock. */
  CAN_FALL: 1,
  /** blocksCantBreak type (non-diggable + chest by default). */
  CANT_BREAK: 2
} as const

/**
 * Per-state dig tables, computed from the bot's CURRENT inventory/effects
 * (upstream re-evaluates bestHarvestTool per probe — same numbers, computed
 * once per solve instead of millions of times).
 */
export interface DigData {
  fingerprint: string
  /** labor[stateId] = 1 + 3 * digTime(bestTool) / 1000 — multiplied by
   * movements.digCost at move-generation time (upstream formula). */
  labor: Float32Array
  /** DigFlags byte per stateId. */
  flags: Uint8Array
}

export type SolveStatus = 'success' | 'partial' | 'timeout' | 'noPath'

/** One path step, wire format (before rehydration into Move objects). */
export interface RawPathNode {
  x: number
  y: number
  z: number
  cost: number
  parkour: boolean
  /** Cell to activate (fence gate / door) before entering this node, if any. */
  useOne: { x: number, y: number, z: number } | null
  /** Blocks to dig before entering this node (canDig solves only). */
  toBreak?: Array<{ x: number, y: number, z: number }>
  /**
   * Two-stage moves (allowParkourExtended). Slime bounce: drop onto the
   * slime STAND cell `via`, let the rebound carry the body up, land at this
   * node. Momentum chain (`chain`): jump onto the stepping stone `via` and
   * re-jump on the landing tick to reach this node. Absent otherwise.
   */
  via?: { x: number, y: number, z: number }
  chain?: boolean
}

export interface SolveResult {
  status: SolveStatus
  cost: number
  /** Milliseconds of think time for this compute (parity with upstream `time`). */
  time: number
  visitedNodes: number
  generatedNodes: number
  path: RawPathNode[]
  /** Packed (cx & 0xffff) | (cz << 16) chunk keys the search expanded into. */
  touchedChunks: number[]
  /**
   * True when the search pruned nodes at the snapshot boundary — the host
   * should retry with a larger snapshot before trusting a noPath.
   */
  boundaryLimited: boolean
}

export interface GoalDescriptor {
  type: string
  [key: string]: unknown
}

/** Minimal node shape our goals read — upstream passes Move (a Vec3 subclass). */
export interface XYZ {
  x: number
  y: number
  z: number
}

export interface PathfinderOptions {
  /**
   * Run the A* solver in a worker_thread (default true). Falls back to
   * main-thread tick-sliced solving (upstream-identical scheduling) when
   * worker startup fails or the goal isn't serializable (custom goal class).
   */
  useWorkerThreads?: boolean
  /** Override the worker entry file (tests / exotic bundlers). */
  workerEntryPath?: string
  /** Hard cap on snapshot cells before giving up growth (memory bound). */
  maxSnapshotCells?: number
  /**
   * Nudge prismarine-physics' player dimensions off their exact block
   * boundaries (0.3/1.8 → 0.30001/1.80001) on inject. Default TRUE.
   *
   * On 1.21.x a body resting exactly on a boundary makes the SERVER's
   * collision sweep compute exactly 1.0, call the move blocked, and teleport
   * the client back — silently, every tick, for as long as the bot keeps
   * producing that position. It is a physics bug, but it presents as a
   * pathfinder that cannot climb a one-block step. See the measurements at
   * the injection site. Turn it off if the application already applies the
   * same nudge (harmless either way — it is guarded on the stock values).
   */
  hitboxPrecisionFix?: boolean
  /**
   * Injectable movement-simulation backend (tests). Defaults to the
   * prismarine-physics-driven PhysicsSim, exactly like upstream.
   */
  physicsFactory?: (bot: unknown) => PhysicsLike
  /**
   * Called on every terminal no-path solve (after growth retries) with the
   * exact snapshot the solver saw — the capture side of the offline replay
   * loop for "the bot says No path in a room it should cross". The dump's
   * shape matches the upstream-mode PF_DUMP_SNAPSHOTS corpus lines.
   */
  onNoPath?: (dump: NoPathDump) => void
}

/** Payload of PathfinderOptions.onNoPath — serializable solver inputs. */
export interface NoPathDump {
  meta: SnapshotMeta
  flags: Uint8Array
  heights: Uint8Array
  start: XYZ
  /** serializeGoal() descriptor (JSON-able). */
  goal: unknown
  /** The solve's movements profile (Movements.toConfig()). */
  cfg: MovementsConfig
  visitedNodes: number
}

/** The sprint/jump decision surface the executor consumes. */
export interface PhysicsLike {
  canStraightLine (path: XYZ[], sprint?: boolean): boolean
  canSprintJump (path: XYZ[], jumpAfter?: number): boolean
  canWalkJump (path: XYZ[], jumpAfter?: number): boolean
  canStraightLineBetween (n1: Vec3, n2: Vec3): boolean
  /** Pressed into a face with no headway — the wedge the server refuses. */
  isGrinding (path: XYZ[], ticks?: number): boolean
  /** Is there ground behind to step back onto? */
  canBackOff (ticks?: number): boolean
  /** Would this control combination actually move the body, safely? */
  canNudge (control: { forward?: boolean, back?: boolean, left?: boolean, right?: boolean, jump?: boolean, sneak?: boolean }, ticks?: number): boolean
  /** A take-off heading offset that lands this jump, or null. */
  bestHeading (path: XYZ[], jump: boolean, sprint: boolean): number | null
  /** Does hopping while sprinting get further down this path, safely? */
  sprintHopBetter (path: XYZ[], lowCeilingHop?: boolean, horizon?: number): boolean
}

export type { Vec3 }
