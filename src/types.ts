import type { Vec3 } from 'vec3'

/** Block-state flags, precomputed per Minecraft version and movement profile
 * so collision probes read a byte without a world lookup or block allocation. */
// A const object keeps the published types compatible with isolatedModules.
export const LutFlags = {
  /** Walk-through safe: empty, climbable or carpet types not in blocksToAvoid.
   * prismarine-block's type-level boundingBox reports 'block' even for open gates. */
  SAFE: 1 << 0,
  /** Can stand on: bounding box 'block' and not a fence-like type (top > 1). */
  PHYSICAL: 1 << 1,
  LIQUID: 1 << 2,
  CLIMBABLE: 1 << 3,
  /** Closed non-iron fence gate with collision shapes; activation makes it
   * traversable when canOpenDoors is enabled. */
  OPENABLE_GATE: 1 << 4,
  /** Closed non-iron door; traversable by activation when canOpenDoors and canOpenRealDoors are enabled. */
  DOOR_CLOSED: 1 << 5,
  /** Open non-iron door; passable when canOpenDoors and canOpenRealDoors are enabled. */
  DOOR_OPEN: 1 << 6,
  /** Open non-iron fence gate without collision shapes; passable when both door options are enabled. */
  GATE_OPEN: 1 << 7,
} as const

/** Optional auxiliary classification for special blocks.
 * The main flags byte has no spare bits, so these values use a separate grid. */
export const LutSpecial = {
  /** bubble_column[drag=false] above soul sand pushes entities up. */
  BUBBLE_UP: 1,
  /** bubble_column[drag=true] above magma drags entities down. */
  BUBBLE_DOWN: 2,
  /** Vines need an adjacent solid block: prismarine-physics ascends only on
   * horizontal collision, so free-hanging vines cannot be climbed. */
  VINE: 4,
  /** Slime rebounds after a drop; marked only when allowParkourExtended is enabled. */
  SLIME: 8,
  /** Bottom stairs have a full-footprint lower slab regardless of facing,
   * so a jump can catch half a block below the top and step up. Requires allowParkourExtended. */
  STAIR: 16
  /** Bits 5-7 encode where narrow supports carry the body: an oriented edge panel,
   * an x/z line, or 0 for centred. These bits never overlap the special-block
   * values above, so equality tests on those values remain valid. */
} as const

/** Snapshot cells store a LutFlags byte and a packed height byte.
 * Height bits 0-5 encode 1/32 blocks: (byte & 63) / 32; bits 6-7 encode top-catch class.
 * Catch classes use half-widths: 0 = wide (>=0.4), 1 = head/wall (0.25),
 * 2 = post/pot (0.125), 3 = too small to stand on. */
export interface SnapshotMeta {
  /** World-space minimum corner of the AABB (inclusive). */
  x0: number
  y0: number
  z0: number
  /** Grid dimensions. */
  w: number
  h: number
  l: number
  /** World minimum Y; landing scans stop below this. */
  worldMinY: number
  /** Process-wide unique generation used as a cache key. */
  generation: number
  /** Incremented on each cell patch; (generation, patchCount) uniquely identifies
   * snapshot content for residency caches. */
  patchCount: number
}

export interface SnapshotBuffers {
  meta: SnapshotMeta
  /** LutFlags byte per cell, backed by SharedArrayBuffer. */
  flags: Uint8Array
  /** Packed height byte per cell: low six bits are collision height in 1/32 blocks. */
  heights: Uint8Array
  /** Sparse entity-intersection pairs (cellIdx, weight), matching Movements.entityIntersections.
   * Empty when allowEntityDetection is disabled. */
  entityIdx: Int32Array
  entityWeight: Int32Array
  /** Sparse exclusion-step weights (cellIdx, weight); empty unless exclusionAreasStep set. */
  exclusionIdx: Int32Array
  exclusionWeight: Int32Array
}

/** Serializable movement profile used by the solver. */
export interface MovementsConfig {
  /** Touchdown damage a hop search refuses, set under a health floor. Not read by the core search. */
  fallVeto?: number
  allowSprinting: boolean
  allowParkour: boolean
  /** Extended sprint-jumps to diagonal, long, raised and drop landings, including
   * ladder, water and bubble-column catches. Default false; requires allowParkour and allowSprinting. */
  allowParkourExtended: boolean
  canOpenDoors: boolean
  /** Open non-iron doors as well as gates when canOpenDoors is enabled. Default true. */
  canOpenRealDoors: boolean
  maxDropDown: number
  infiniteLiquidDropdownDistance: boolean
  liquidCost: number
  entityCost: number
  /** Enable digging moves. Default false. */
  canDig: boolean
  digCost: number
  dontCreateFlow: boolean
  dontMineUnderFallingBlock: boolean
  /** Ride bubble-column elevators: soul sand pushes up, magma pulls down. Default false. */
  useBubbleColumns: boolean
  /** Cost per block of bubble-column travel. Default 1 keeps the vertical
   * distance heuristic admissible. */
  bubbleCost: number
  /** Blocks of reach reserved below the physics limit. Default 0.1; 0 allows
   * frame-perfect jumps. The executor checks every takeoff with a physics rollout. */
  parkourSafetyMargin?: number
  /** Carry landing direction into search state to chain jumps on the landing tick.
   * Default false; requires allowParkourExtended. */
  allowParkourMomentum?: boolean
  /** Offer zero-margin jumps at a cost premium when the normal envelope has no jump.
   * Default true; requires allowParkourExtended. */
  allowParkourTight?: boolean
  /** Use physics-verified hops beyond the table's reach when search fails.
   * Default false; requires allowParkourExtended. */
  allowParkourPhysics?: boolean
}

/** Per-state dig auxiliaries (only consulted when a block is unsafe). */
export const DigFlags = {
  /** Falling blocks such as sand or gravel prevent mining below them when dontMineUnderFallingBlock is enabled. */
  CAN_FALL: 1,
  /** blocksCantBreak type (non-diggable + chest by default). */
  CANT_BREAK: 2
} as const

/** Per-state dig tables computed once per solve from the bot's current inventory and effects. */
export interface DigData {
  fingerprint: string
  /** labor[stateId] = 1 + 3 * digTime(bestTool) / 1000;
   * move generation multiplies it by movements.digCost. */
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
  /** Intermediate stand cell for a two-stage move when allowParkourExtended is enabled.
   * Slime moves rebound here; chain moves jump again on the landing tick. */
  via?: { x: number, y: number, z: number }
  chain?: boolean
  /** Jump requiring a run-up because standing reach is insufficient. */
  run?: boolean
  /** Aimed flight: bits 0-6 hold the 1-based AIM_VARIANTS index; bit 7 mirrors
   * the variant in world space. Variant offsets shift takeoff and landing from cell centres. */
  aim?: number
  /** Physics-verified control program replayed tick for tick from rest on the
   * previous node's support. Requires allowParkourPhysics. */
  program?: import('./hopOracle.js').HopProgram
}

export interface SolveResult {
  status: SolveStatus
  cost: number
  /** Solver computation time in milliseconds. */
  time: number
  visitedNodes: number
  generatedNodes: number
  path: RawPathNode[]
  /** Packed (cx & 0xffff) | (cz << 16) chunk keys the search expanded into. */
  touchedChunks: number[]
  /** True when search prunes nodes at the snapshot boundary; retry with a
   * larger snapshot before treating noPath as definitive. */
  boundaryLimited: boolean
}

export interface GoalDescriptor {
  type: string
  [key: string]: unknown
}

/** Minimal position shape consumed by goals. */
export interface XYZ {
  x: number
  y: number
  z: number
}

export interface PathfinderOptions {
  /** Run A* in a worker thread (default true). Falls back to tick-sliced solving
   * on the main thread if worker startup fails or a custom goal cannot be serialized. */
  useWorkerThreads?: boolean
  /** Worker solver core: Rust ('wasm', default) or the equivalent JavaScript implementation.
   * Both support physics hops; main-thread solves always use JavaScript. */
  core?: 'wasm' | 'js'
  /** Override the worker entry file (tests / exotic bundlers). */
  workerEntryPath?: string
  /** Hard cap on snapshot cells before giving up growth (memory bound). */
  maxSnapshotCells?: number
  /** Nudge stock player dimensions from 0.3/1.8 to 0.30001/1.80001 on injection
   * to avoid exact-boundary collision rounding and server corrections on 1.21.x. Default true. */
  hitboxPrecisionFix?: boolean
  /** Injectable movement simulator, defaulting to the prismarine-physics-backed PhysicsSim. */
  physicsFactory?: (bot: unknown) => PhysicsLike
  /** Called after growth retries on a terminal no-path solve with the exact solver snapshot. */
  onNoPath?: (dump: NoPathDump) => void
  /** Where warnings go (default console). The worker thread's own rare warnings still use console. */
  logger?: PathfinderLogger
}

export interface PathfinderLogger {
  warn: (message: string) => void
}

/** Per-call options for `bot.pathfinder.goto(goal, options)`. */
export interface GotoOptions {
  /** Resolve at the closest reachable cell when the goal is unreachable.
   * Default false rejects with NoPath, so callers do not silently accept a partial walk. */
  bestEffort?: boolean
  /** Maximum partial-path legs before best-effort walking resolves. Default 3;
   * the cap prevents endless retries when the closest reachable node keeps changing. */
  maxBestEffortLegs?: number
  /** Cancel the goal. `goto` rejects with the signal's reason and the bot stops. */
  signal?: AbortSignal
}

/** Options for `bot.pathfinder.computePath(goal, options)`. */
export interface ComputePathOptions {
  /** Where the path starts (default: the bot's position). */
  from?: Vec3
  /** The profile to plan with (default: the current one). */
  movements?: import('./movements.js').Movements
  /** Search budget in ms (default: thinkTimeout). */
  timeout?: number
  /** Extra cost allowed past the heuristic (default: searchRadius). */
  searchRadius?: number
  /** Cancel the search. The promise rejects with the signal's reason. */
  signal?: AbortSignal
}

/** Serializable solver inputs passed to PathfinderOptions.onNoPath. */
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
  /** Optional: the executor calls this once per tick so the physics may cache world reads within it. */
  beginTick? (): void
  canStraightLine (path: XYZ[], sprint?: boolean): boolean
  canSprintJump (path: XYZ[], jumpAfter?: number): boolean
  canWalkJump (path: XYZ[], jumpAfter?: number): boolean
  canStraightLineBetween (n1: Vec3, n2: Vec3): boolean
  /** Detect a body pressed against a face without making progress. */
  isGrinding (path: XYZ[], ticks?: number): boolean
  /** Check for safe ground behind the body. */
  canBackOff (ticks?: number): boolean
  /** Check whether the given controls move the body safely. */
  canNudge (control: { forward?: boolean, back?: boolean, left?: boolean, right?: boolean, jump?: boolean, sneak?: boolean }, ticks?: number): boolean
  /** Check whether holding the keys, then coasting to rest, stays supported; null uses current controls. */
  holdsOn? (keys: { forward?: boolean, back?: boolean, sneak?: boolean } | null, ticks?: number, keyTicks?: number, maxDrop?: number): boolean
  /** A take-off heading offset that lands this jump, or null. */
  bestHeading (path: XYZ[], jump: boolean, sprint: boolean): number | null
  /** Heading offset for walking around a clipped corner, or null. */
  bestWalkHeading (path: XYZ[], sprint: boolean): number | null
  /** Check whether sprint-hopping makes more progress safely than running. */
  sprintHopBetter (path: XYZ[], lowCeilingHop?: boolean, horizon?: number): boolean
  /** Check whether the kernel can walk from the current body state to the node. */
  walksTo? (node: XYZ): boolean
  /** Choose this tick's movement with the physics kernel when allowHopIntoJump is enabled. */
  gaitDrive? (path: ReadonlyArray<XYZ & { parkour?: boolean }>): GaitDrive
  /** Choose flight controls for landing on the node; null uses forward and sprint.
   * Without this method, jumps hold forward throughout. */
  airControl? (node: XYZ, engaged: boolean): { control: { forward: boolean, back: boolean, sprint: boolean } | null, engaged: boolean }
  /** Search up to budget scripted jumps from the current body state per call.
   * Returns undefined while searching or null when no candidate lands. */
  solveJump? (node: XYZ, budget: number): {
    yawA: number, yawB: number, turnTick: number, sprint: boolean, jumpAfter: number, ticks: number
    track: Array<{ x: number, y: number, z: number }>, miss: number
  } | null | undefined
  /** Reason sprintHopBetter refuses a hop; empty string means it accepts. */
  hopRefusal?: string
  /** Provide the kernel with block shape tables. */
  setShapeTables? (lut: import('./lut.js').BlockLut): void
  /** Movement-speed attribute without sprinting, used by the worker kernel. */
  walkSpeed? (): number
  /** Find a control program from the current body state to a planner node. */
  hopFrom? (cell: readonly [number, number, number], budgetTicks: number, land?: { x: number, y: number, z: number }, next?: readonly [number, number, number], beam?: boolean,
    seed?: ReadonlyArray<'kick' | 'pivot'>, seedFirst?: boolean): import('./hopOracle.js').HopProgram | null
  /** Find a live program past the gait's standing node to the next planned program's departure node. */
  skipAhead? (path: ReadonlyArray<XYZ & { parkour?: boolean }>, drive: GaitDrive): { program: import('./hopOracle.js').HopProgram, cut: number } | null
  /** Whether skipAhead uses search on its latest call. */
  skipSearched?: boolean
  /** Find a live program past a pillar-dodging jump at path[0] to path[1], using t0 as the comparison budget. */
  skipNeo? (path: ReadonlyArray<XYZ & { parkour?: boolean }>, t0: number, rest: boolean): { program: import('./hopOracle.js').HopProgram, cut: number } | null
  /** Whether the last hopLinedUp call lacks a supported departure or landing point. */
  hopNoStand?: boolean
  /** Find a live program onto an intermediate support, chosen for the following jump. */
  hopChain? (cell: readonly [number, number, number], next: readonly [number, number, number], budgetTicks: number, nextGo?: boolean): import('./hopOracle.js').HopProgram | null
  /** Check a lined-up jump from one node to another before the body arrives. */
  hopFromCell? (from: readonly [number, number, number], to: readonly [number, number, number], budgetTicks: number, beam?: boolean): import('./hopOracle.js').HopProgram | null
  /** Departure cell used by the last lined-up jump check. */
  linedFrom?: readonly [number, number, number] | null
  /** Find a live program past walking nodes to the first jump, compared with the gait's approach. */
  skipWalk? (path: ReadonlyArray<XYZ & { parkour?: boolean }>, allowed: (m: XYZ) => boolean): { program: import('./hopOracle.js').HopProgram, cut: number } | null
  /** Whether onGround remains set after the body leaves support; null without a kernel. */
  latchedNow? (): boolean | null
  /** Find a program from rest at a supported point in the body's current node. */
  hopLinedUp? (cell: readonly [number, number, number], budgetTicks: number): import('./hopOracle.js').HopProgram | null
  /** Find a short ground retreat followed by one jump from rest flush against an obstacle. */
  hopPrefix? (cell: readonly [number, number, number], land?: { x: number, y: number, z: number }, hint?: import('./hopOracle.js').HopProgram): import('./hopOracle.js').HopProgram | null
  /** Re-anchor a planned program at the current body state if it still lands. */
  hopAnchored? (cell: readonly [number, number, number], prog: import('./hopOracle.js').HopProgram): import('./hopOracle.js').HopProgram | null
  /** Detect whether the bot's physics preserves the faster vanilla jump rise and set the kernel rule. */
  probeVanillaJump? (): boolean
  /** Check whether a deep bounce still reaches its target and whether aborting lands in the pool. */
  deepVerdict? (prog: import('./hopOracle.js').HopProgram, t: number, fired: boolean, air: number, rebounds: number): { arrives: boolean, abortable: boolean } | null
  /** Moving line-up attempts start only near a safe glide end; moving.tried
   * records the attempt, and delay precedes the control presses. */
  hopLineUp? (prog: import('./hopOracle.js').HopProgram, budgetTicks: number, moving?: { reach: number, tried: boolean }): { yaws: number[], delay?: number, prog: import('./hopOracle.js').HopProgram } | null
  /** Diagnostic reason for the latest hopFrom or hopLinedUp result. */
  hopNote?: string
  /** True when hopLinedUp exhausts every program family without landing, rather than timing out. */
  hopExhausted?: boolean
  /** Last retired node, used to recognize landings in gait decisions. */
  gaitFrom?: { x: number, z: number } | null
  /** Whether the server counts the whole fall for fall damage. */
  wholeFall?: boolean
  /** Touchdown damage live hop searches refuse. */
  fallVeto?: number
  /** Enable the 45-degree strafe gait when Movements.allowStrafe is set. */
  gaitStrafe?: boolean
  /** Retire parkour nodes passed during landing when Movements.allowLandingRetire is set. */
  gaitLandRetire?: boolean
}

/** Kernel decision for this tick: replay flight headings for a hop, use yaw,
 * sprint and strafe for a run, or return none when no schedule is available.
 * For hops, strafe starts after the takeoff tick. */
export interface GaitDrive {
  kind: 'hop' | 'run' | 'none'
  flight: number[] | null
  strafe: boolean
  /** Flight tick at which forward and strafe are released; EASE_NEVER holds them throughout. */
  ease: number
  yaw: number
  sprint: boolean
  /** Kernel ticks to the window's goal using the faster hop or run schedule; Infinity if neither reaches it. */
  ticks?: number
  /** Path index where the window stops before a programmed jump from the preceding node; -1 if uncut. */
  cut?: number
  /** Path index of the window's goal; goalLast indicates whether the schedule stops there. */
  goal?: number
  goalLast?: boolean
  /** Number of kernel ticks simulated during the search. */
  work?: number
}

export type { Vec3 }
