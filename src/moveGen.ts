// Snapshot probes and preallocated neighbor arrays avoid allocations on walk-only expansions.
// Dig moves allocate only their block lists.
import { LutFlags, LutSpecial, DigFlags } from './types.js';
import type { MovementsConfig, SnapshotMeta, DigData } from './types.js';
import { getParkourExtTable, AIM_P0_MASK } from './parkourTable.js';
import type { ParkourExtEntry, ParkourExtTable } from './parkourTable.js';
import { J_RUN, J_LOW_RUN, runRow, J_CHAIN, CHAIN_MIN_COS, CHAIN_TAKEOFF_FRACTION, CHAIN_TURN_LOSS, LIP_PHASE, TAKEOFF_RUN, reachBucket, ENVELOPE_SAFETY_MARGIN, TAKEOFF_STAND, LAND_HALF, TAKEOFF_NARROW_MARGIN, LAND_NARROW_MARGIN, BOUNCE_APEX, BOUNCE_MAX_DROP, BOUNCE_MARGIN, BOUNCE_MARGIN_FAR, J_LID8_RUN, J_LID13_RUN, J_LID3_RUN, FLIGHT_ROW, FREE_APEX } from './parkourEnvelope.js';
import { CATCH_HALF, CARRY_SHIFT, carrySide, carryTouch, CARRY_W_NONE, CARRY_S_NONE, isLidByte } from './shapes.js';
/** Maximum number of lids checked for one candidate jump. */
const LID_MAX = 64;
/** Flight rows selected by the feet height at which the head hits a lid. */
function lidRows (cap: number): ReadonlyArray<readonly number[]> {
    return cap >= FREE_APEX ? J_RUN : cap >= 1.2 - 1e-9 ? J_LID3_RUN : cap >= 1.0125 - 1e-9 ? J_LID13_RUN : cap >= 0.7 - 1e-9 ? J_LID8_RUN : J_LOW_RUN;
}
/** cos(angle) gate as an exact integer test: dot > 0 and dot² ≥ c²·|u|²·|v|². */
const CHAIN_MIN_COS2 = CHAIN_MIN_COS * CHAIN_MIN_COS;
/** Momentum is MOM_NONE or 1 + a packed primitive flight direction (dx/g, dz/g).
 * Components lie in [-6, 6]; exact directions keep the re-jump cone consistent
 * with compound chains. The WASM core uses the same packing. */
export const MOM_NONE = 0;
/** Maximum drop retaining momentum: with the 1.25-block apex, this reaches
 * vanilla's 3-block fall-damage threshold. */
export const MOMENTUM_MAX_DROP = 1.75;
const MOM_RANGE = 6;
const MOM_SPAN = 2 * MOM_RANGE + 1;
export function momentumOf (dx: number, dz: number): number {
    let a = dx < 0 ? -dx : dx;
    let b = dz < 0 ? -dz : dz;
    while (b !== 0) {
        const t = a % b;
        a = b;
        b = t;
    }
    return 1 + (dx / a + MOM_RANGE) * MOM_SPAN + (dz / a + MOM_RANGE);
}
export function momentumDx (m: number): number {
    return Math.floor((m - 1) / MOM_SPAN) - MOM_RANGE;
}
export function momentumDz (m: number): number {
    return ((m - 1) % MOM_SPAN) - MOM_RANGE;
}
/** Required flight from distance `s` along the flight line, with per-axis
 * landing credits for narrow supports. */
function flightFrom2 (a: number, b: number, dist: number, s: number, lCredX: number, lCredZ: number): number {
    const dx = a - a / dist * s - lCredX;
    const dz = b - b / dist * s - lCredZ;
    const px = dx > 0 ? dx : 0;
    const pz = dz > 0 ? dz : 0;
    // Use sqrt rather than hypot so JavaScript and WASM agree on the last bit.
    return Math.sqrt(px * px + pz * pz);
}
export type StepExclusionFn = ((x: number, y: number, z: number) => number) | null
export type BreakExclusionFn = ((x: number, y: number, z: number) => number) | null

export interface SnapshotView {
  meta: SnapshotMeta
  flags: Uint8Array
  heights: Uint8Array
  entityIdx: Int32Array
  entityWeight: Int32Array
  /** Raw state IDs, required by the state-keyed dig tables when canDig is enabled. */
  states?: Uint16Array | null
  /** LutSpecial byte per cell, required for useBubbleColumns. */
  special?: Uint8Array | null
  /** Thin footprint byte per cell for allowParkourExtended. */
  thin?: Uint8Array | null
  /** Forbidden moves as (from cell index, to cell index) pairs.
   * The cells remain available to all other moves. */
  bannedEdges?: Int32Array | null
}

export interface DigContext {
  data: DigData
  states: Uint16Array
  breakExclusion: BreakExclusionFn
}

export const META_PARKOUR = 1;
export const META_USEONE = 2;
/** Slime-bounce move: the node is reached by dropping onto `via` (a slime
 * stand cell) and riding the rebound up. Always paired with META_PARKOUR. */
export const META_BOUNCE = 4;
/** Momentum-chain move: jump to `via` (a stepping stone) and re-jump on the
 * landing tick to reach the node. Always paired with META_PARKOUR. */
export const META_CHAIN = 8;
/** Extra flight required when a diagonal leaves an edge panel's empty side. */
const OFF_PANEL_MARGIN = 0.3;
/** Climbability uses the body centre, so a ladder catch needs the centre
 * inside the cell: 0.5 blocks from its middle, rather than LAND_HALF's 0.8. */
const LADDER_CATCH_OVERCREDIT = 0.3;
/** A ladder-top takeoff requires climbing out and resting on the 3/16 strip;
 * this adds the setup cost omitted by the climb moves. */
const LADDER_TOP_COST = 2;
/** A catch one block up tests climbability at the feet centre, so its
 * landing credit is the cell's half-width less a margin. */
const UP_CATCH_CREDIT = 0.45;
/** Thin footprint interval classes in sixteenths. */
const THIN_LO = [0, 0, 13, 7, 6, 5, 4, 0, 0, 0, 0, 7, 6, 5, 4, 0];
const THIN_HI = [16, 3, 16, 9, 10, 11, 12, 9, 10, 11, 12, 16, 16, 16, 16, 16];
/** Hitbox half-width a thin footprint is tested against (a centimetre of grace). */
const THIN_BODY_HALF = 0.29;
/** Whether segment P -> P+D enters the open rectangle [x0,x1] x [z0,z1]. */
function segHits (px: number, pz: number, dx: number, dz: number, x0: number, x1: number, z0: number, z1: number): boolean {
    let t0 = 0;
    let t1 = 1;
    if (dx === 0) {
        if (px <= x0 || px >= x1)
            return false;
    }
    else {
        let e = (x0 - px) / dx;
        let x = (x1 - px) / dx;
        if (e > x) {
            const sw = e;
            e = x;
            x = sw;
        }
        if (e > t0)
            t0 = e;
        if (x < t1)
            t1 = x;
    }
    if (dz === 0) {
        if (pz <= z0 || pz >= z1)
            return false;
    }
    else {
        let e = (z0 - pz) / dz;
        let x = (z1 - pz) / dz;
        if (e > x) {
            const sw = e;
            e = x;
            x = sw;
        }
        if (e > t0)
            t0 = e;
        if (x < t1)
            t1 = x;
    }
    return t1 - t0 > 1e-9;
}
/** Extra cost for a jump that requires the zero-margin envelope. */
const TIGHT_COST = 6;
/** Marks a jump requiring a run-up, lip takeoff or chain instead of a jump
 * from rest. Executor information only; always paired with META_PARKOUR. */
export const META_RUN = 16;
/** Marks an aimed flight line with offset takeoff or landing points.
 * outAim bits 0-6 hold the 1-based variant index; bit 7 marks a mirrored side.
 * One mirrored axis flips the perpendicular; two do not.
 * Executor information only; always paired with META_PARKOUR. */
export const META_AIM = 32;
/** Marks a drop into liquid deeper than maxDropDown with no return path.
 * Descendants inherit this flag and cannot be endpoints of partial or noPath
 * results, even when they are closer to the goal. */
export const META_DEEP = 64;
/** A drop past the fall budget onto a full top, landed with a water bucket placed just before
 * touchdown. Not inherited; always paired with META_DEEP. */
export const META_CLUTCH = 128;
/** A clutch drop only pays when the landing would hurt: support at least this far below the feet. */
const CLUTCH_MIN_DROP = 4;
const SAFE = LutFlags.SAFE;
const PHYSICAL = LutFlags.PHYSICAL;
const LIQUID = LutFlags.LIQUID;
const CLIMBABLE = LutFlags.CLIMBABLE;
const GATE = LutFlags.OPENABLE_GATE;
const DOOR_CLOSED = LutFlags.DOOR_CLOSED;
const DOOR_OPEN = LutFlags.DOOR_OPEN;
const GATE_OPEN = LutFlags.GATE_OPEN;
const PASSABLE_WHEN_OPEN = DOOR_OPEN | GATE_OPEN;
/** Small penalty beside fire, lava or cobwebs to prefer equally short paths
 * with clearance, since momentum through a turn can carry the body into them. */
const HAZARD_NEAR_COST = 0.01;
const CAN_FALL = DigFlags.CAN_FALL;
const CANT_BREAK = DigFlags.CANT_BREAK;
const BUBBLE_UP = LutSpecial.BUBBLE_UP;
const BUBBLE_DOWN = LutSpecial.BUBBLE_DOWN;
const SPECIAL_VINE = LutSpecial.VINE;
const SPECIAL_SLIME = LutSpecial.SLIME;
const SPECIAL_STAIR = LutSpecial.STAIR;
/** Bubble-only semantics (float support, fall catching, no-jump) must not
 * fire on VINE/SLIME-marked cells. */
const BUBBLE_MASK = BUBBLE_UP | BUBBLE_DOWN;
// Probe order: W, E, N, S, then NW, SW, NE, SE.
const CARDINAL_X = [-1, 1, 0, 0];
const CARDINAL_Z = [0, 0, -1, 1];
const DIAGONAL_X = [-1, -1, 1, 1];
const DIAGONAL_Z = [-1, 1, -1, 1];
const TRANSFER_X = [-1, 1, 0, 0, -1, -1, 1, 1];
const TRANSFER_Z = [0, 0, -1, 1, -1, 1, -1, 1];
/** Transfer cost per direction: octile + 0.5 pad (|dy| is added per move). */
const TRANSFER_COST = [1.5, 1.5, 1.5, 1.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5];
// A rebound drifts about one block by its apex and farther while descending.
const BOUNCE_X = [-1, 1, 0, 0, -1, -1, 1, 1, -2, 2, 0, 0];
const BOUNCE_Z = [0, 0, -1, 1, -1, 1, -1, 1, 0, 0, -2, 2];
const BOUNCE_OFFSETS = 12;
const BOUNCE_RING1 = 8;
/** Derive neighbor capacity from the moveset: typed-array overflow silently
 * drops writes while outCount continues, leaving invalid coordinates. */
function outCapacity (extended: boolean): number {
    // Cardinal parkour can emit three drop landings per direction,
    // adding 12 edges to the 20 base moves.
    const base = extended ? 20 : 32;
    if (!extended)
        return base;
    const t = getParkourExtTable();
    // Reserve 8 directions x 3 heights for climb transfers and five sets of
    // bounce targets. Sparse stepping-stone chains use a fixed allowance
    // rather than the product of all possible first and second jumps.
    const table = t.diag.length * 4 + t.cardX.length * 2 + t.cardZ.length * 2;
    return table + base + 24 + BOUNCE_OFFSETS * 5 + CHAIN_CAP;
}
/** Upper bound on momentum-chain edges per expansion. */
const CHAIN_CAP = 256;
export class MoveGen {
    /** Output arrays for one expansion, sized by outCapacity(). */
  readonly outIdx!: Int32Array
  readonly outX!: Int32Array
  readonly outY!: Int32Array
  readonly outZ!: Int32Array
  readonly outCost!: Float64Array
  readonly outMeta!: Uint8Array
    /** toBreak per neighbor: outBreaks[i] is null or an array of cell indices. */
  readonly outBreaks!: Array<number[] | null>
    /** Slime stand cell (index) a META_BOUNCE neighbour drops onto; -1 otherwise. */
  readonly outVia!: Int32Array
    /** AIM byte of an aimed parkour move (META_AIM), 0 otherwise. */
  readonly outAim!: Uint8Array
    /** Arrival momentum; MOM_NONE unless momentum is enabled for a support landing. */
  readonly outMom!: Uint8Array
  outCount = 0
    /** Slime stand cell for the next push. */
  private pendingVia = -1
    /** Aimed variant for the next push. */
  private pendingAim = 0
  /** Whether the head+1 lid lies inside the takeoff support panel's strip. */
  private lidHere = false
  /** Forbidden cell-index pairs, or null when none exist. */
  private readonly bannedEdges: Int32Array | null = null
  /** Current source cell of a banned edge, or -1 when no banned edge leaves it. */
  private bannedFrom = -1
    /** Arrival momentum for the next push. */
  private pendingMom = MOM_NONE
    /** The centre line's landing (node y, support y), reused by its aimed variants. */
    private extLandY = 0
    private extLandSupY = NaN
    private extLandLid1 = false
    /** Aimed variants to try after a corridor refusal: bit v selects variant v; 0 means none. */
  private extTryMask = 0
    /** Whether the candidate is a climbable catch one block above flight level. */
  private extUpCatch = false
    /** Whether extended parkour landings carry momentum. */
  readonly parkourMomentum!: boolean
    /** Momentum of the node being expanded. */
  private momIn = MOM_NONE
  private momDx = 0
  private momDz = 0
  private momD2 = 0
    /** A probe leaves the snapshot; expanding it may resolve a noPath result. */
  boundaryTouched = false
  private readonly flags!: Uint8Array
  private readonly heights!: Uint8Array
  private readonly x0!: number
  private readonly y0!: number
  private readonly z0!: number
  private readonly w!: number
  private readonly h!: number
  private readonly l!: number
  private readonly worldMinY!: number
  private readonly entityMap!: Map<number, number> | null
  private readonly stepExclusion!: StepExclusionFn
  private readonly allowParkour!: boolean
  private readonly allowSprinting!: boolean
    /** Extended parkour requires parkour and sprinting to be enabled. */
  private readonly parkourExtended!: boolean
    /** Generated offset/corridor table (null when the feature is off). */
  private readonly extTable!: ParkourExtTable | null
  private readonly canOpenDoors!: boolean
  private readonly doorMode!: boolean
  private readonly maxDropDown!: number
  /** Clutch drops: deepest node drop + 1 (support distance), the shallowest worth a bucket, and the extra cost. */
  private readonly clutchDropDown!: number
  private readonly clutchMin!: number
  private readonly clutchCost!: number
  /** Set by findLanding when its landing needs a clutch. */
  private landClutch = false
  private readonly infiniteLiquidDropdownDistance!: boolean
  private readonly liquidCost!: number
  private readonly entityCost!: number
  private readonly special!: Uint8Array | null
    /** Thin footprint grid (null without the extended repertoire). */
  private readonly thin!: Uint8Array | null
  private readonly bubbleCost!: number
    /** Added to every usable-flight row: ENVELOPE_SAFETY_MARGIN − the profile's
     * parkourSafetyMargin (0 at the default; positive plans tighter jumps). */
  private readonly marginCredit!: number
    /** Tight-pass margin credit; equal to marginCredit when allowParkourTight is disabled. */
  private readonly tightCredit!: number
  private readonly dig!: DigContext | null
  private readonly digCost!: number
  private readonly dontCreateFlow!: boolean
  private readonly dontMineUnderFallingBlock!: boolean
    /** Per-move toBreak scratch (cell indices); committed on push. */
  private moveBreaks: number[] = []
  constructor (snap: SnapshotView, cfg: MovementsConfig, stepExclusion: StepExclusionFn = null, dig: DigContext | null = null) {
        this.flags = snap.flags;
        this.heights = snap.heights;
        const m = snap.meta;
        this.x0 = m.x0;
        this.y0 = m.y0;
        this.z0 = m.z0;
        this.w = m.w;
        this.h = m.h;
        this.l = m.l;
        this.worldMinY = m.worldMinY;
        if (snap.entityIdx.length > 0) {
            const map = new Map();
            for (let i = 0; i < snap.entityIdx.length; i++) {
                map.set(snap.entityIdx[i], snap.entityWeight[i]);
            }
            this.entityMap = map;
        }
        else {
            this.entityMap = null;
        }
        if (snap.bannedEdges != null && snap.bannedEdges.length >= 2)
            this.bannedEdges = snap.bannedEdges;
        this.stepExclusion = stepExclusion;
        this.allowParkour = cfg.allowParkour;
        this.allowSprinting = cfg.allowSprinting;
        this.parkourExtended = cfg.allowParkour && cfg.allowSprinting && cfg.allowParkourExtended;
        this.extTable = this.parkourExtended ? getParkourExtTable() : null;
        const cap = outCapacity(this.parkourExtended);
        this.outIdx = new Int32Array(cap);
        this.outX = new Int32Array(cap);
        this.outY = new Int32Array(cap);
        this.outZ = new Int32Array(cap);
        this.outCost = new Float64Array(cap);
        this.outMeta = new Uint8Array(cap);
        this.outBreaks = new Array(cap).fill(null);
        this.outVia = new Int32Array(cap).fill(-1);
        this.outAim = new Uint8Array(cap);
        this.outMom = new Uint8Array(cap);
        this.parkourMomentum = this.parkourExtended && cfg.allowParkourMomentum === true;
        this.canOpenDoors = cfg.canOpenDoors;
        this.doorMode = cfg.canOpenDoors && cfg.canOpenRealDoors;
        this.maxDropDown = cfg.maxDropDown;
        this.clutchDropDown = cfg.clutchDropDown ?? 0;
        this.clutchMin = Math.max(cfg.maxDropDown, CLUTCH_MIN_DROP);
        this.clutchCost = cfg.clutchCost ?? 3;
        this.infiniteLiquidDropdownDistance = cfg.infiniteLiquidDropdownDistance;
        this.liquidCost = cfg.liquidCost;
        this.entityCost = cfg.entityCost;
        // The grid exists only when a feature marks special cells.
        this.special = snap.special ?? null;
        this.thin = snap.thin ?? null;
        this.bubbleCost = cfg.bubbleCost;
        this.marginCredit = ENVELOPE_SAFETY_MARGIN - (cfg.parkourSafetyMargin ?? ENVELOPE_SAFETY_MARGIN);
        this.tightCredit = cfg.allowParkourTight === false ? this.marginCredit : ENVELOPE_SAFETY_MARGIN;
        this.dig = cfg.canDig ? dig : null;
        this.digCost = cfg.digCost;
        this.dontCreateFlow = cfg.dontCreateFlow;
        this.dontMineUnderFallingBlock = cfg.dontMineUnderFallingBlock;
    }
    /** Cell index, or -1 outside the snapshot. */
  cellIndex (x: number, y: number, z: number): number {
        const lx = x - this.x0;
        const ly = y - this.y0;
        const lz = z - this.z0;
        if (lx < 0 || lx >= this.w || ly < 0 || ly >= this.h || lz < 0 || lz >= this.l) {
            this.boundaryTouched = true;
            return -1;
        }
        return (ly * this.l + lz) * this.w + lx;
    }
  flagsAt (x: number, y: number, z: number): number {
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? 0 : this.flags[idx];
    }
    /** Neighbor probe that leaves the boundary flag untouched because no move enters it. */
  private cellIndexQuiet (x: number, y: number, z: number): number {
        const lx = x - this.x0;
        const ly = y - this.y0;
        const lz = z - this.z0;
        if (lx < 0 || lx >= this.w || ly < 0 || ly >= this.h || lz < 0 || lz >= this.l) return -1;
        return (ly * this.l + lz) * this.w + lx;
    }
    /** Fire, lava and cobwebs have no collision box to deflect the body.
     * A diagonal can cut their corners by up to 0.21 blocks. */
  private hazardAt (x: number, y: number, z: number): boolean {
        const idx = this.cellIndexQuiet(x, y, z);
        if (idx < 0) return false;
        return (this.flags[idx] & (SAFE | PHYSICAL | PASSABLE_WHEN_OPEN)) === 0 && (this.heights[idx] & 63) === 0;
    }
    /** Whether any of the eight neighbors at feet or head height is a hazard. */
  private hazardNear (x: number, y: number, z: number, idx: number): boolean {
        let m = this.nearHazard;
        if (m === null) m = this.nearHazard = new Uint8Array(this.flags.length);
        const v = m[idx];
        if (v !== 0) return v === 2;
        let near = false;
        for (let dy = 0; dy <= 1 && !near; dy++) {
            for (let dz = -1; dz <= 1 && !near; dz++) {
                for (let dx = -1; dx <= 1; dx++) {
                    if ((dx !== 0 || dz !== 0) && this.hazardAt(x + dx, y + dy, z + dz)) { near = true; break; }
                }
            }
        }
        m[idx] = near ? 2 : 1;
        return near;
    }
    /** Absolute collision-top height; bits 6-7 of the height byte store the top-catch class. */
  heightAt (x: number, y: number, z: number): number {
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? y : y + (this.heights[idx] & 63) / 32;
    }
    /** Top-catch class: 0 for full/wide supports through 3 for unstandable blocks. */
  private catchAt (x: number, y: number, z: number): number {
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? 0 : this.heights[idx] >> 6;
    }
    /** Narrow-support position within its cell; 0 means centred. */
  private carryAt (x: number, y: number, z: number): number {
        return this.specialAt(x, y, z) >> CARRY_SHIFT;
    }
  /** Reject lava beside airborne landings: LAND_HALF allows the body to
     * land 0.3 blocks past the cell edge. Cells outside the snapshot do not
     * count as hazards here; this check applies only to jumps. */
  private lavaFlanked (tx: number, ny: number, tz: number): boolean {
        for (let dx = -1; dx <= 1; dx++) {
            for (let dz = -1; dz <= 1; dz++) {
                if (dx === 0 && dz === 0)
                    continue;
                for (let ly = ny; ly <= ny + 1; ly++) {
                    const idx = this.cellIndex(tx + dx, ly, tz + dz);
                    if (idx < 0)
                        continue;
                    const f = this.flags[idx];
                    if ((f & LIQUID) !== 0 && (f & SAFE) === 0)
                        return true;
                }
            }
        }
        return false;
    }
  /** Whether a cell is safe, including open doors and gates when enabled. */
  private isSafe (f: number): boolean {
        return (f & SAFE) !== 0 || (this.doorMode && (f & PASSABLE_WHEN_OPEN) !== 0);
    }
    /** Fence, wall and closed-gate tops support the feet 0.5 blocks into the
     * cell above, although their 1.5-block height excludes them from PHYSICAL.
     * hByte is the packed height byte. */
  private isTallStand (f: number, hByte: number): boolean {
        return (f & (SAFE | PHYSICAL | LIQUID)) === 0 && (hByte & 63) > 32;
    }
  private tallStandAt (x: number, y: number, z: number): boolean {
        const idx = this.cellIndex(x, y, z);
        return idx >= 0 && this.isTallStand(this.flags[idx], this.heights[idx]);
    }
    /** Thin floors support the feet inside their cell. Treating them as full
     * blocks places the landing one block above the floor. */
  private isThinFloor (f: number): boolean {
        return (f & (SAFE | PHYSICAL | CLIMBABLE)) === (SAFE | PHYSICAL);
    }
    /** LutSpecial byte, or 0 outside the snapshot or when disabled. */
  private specialAt (x: number, y: number, z: number): number {
        if (this.special === null)
            return 0;
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? 0 : this.special[idx];
    }
  private entitiesAt (x: number, y: number, z: number): number {
        if (this.entityMap === null)
            return 0;
        const idx = this.cellIndex(x, y, z);
        if (idx < 0)
            return 0;
        return this.entityMap.get(idx) ?? 0;
    }
  private exclusionAt (x: number, y: number, z: number): number {
        return this.stepExclusion === null ? 0 : this.stepExclusion(x, y, z);
    }
    /** Reset the break list before generating each move. */
  private beginMove (): void {
        if (this.moveBreaks.length > 0)
            this.moveBreaks = [];
    }
    /** Whether a block can be broken safely when canDig is enabled. */
  private safeToBreak (x: number, y: number, z: number, idx: number): boolean {
        const dig = this.dig!;
        if (this.dontCreateFlow) {
            // Liquids above or beside the block can flow into the opening.
            if ((this.flagsAt(x, y + 1, z) & LIQUID) !== 0)
                return false;
            if ((this.flagsAt(x - 1, y, z) & LIQUID) !== 0)
                return false;
            if ((this.flagsAt(x + 1, y, z) & LIQUID) !== 0)
                return false;
            if ((this.flagsAt(x, y, z - 1) & LIQUID) !== 0)
                return false;
            if ((this.flagsAt(x, y, z + 1) & LIQUID) !== 0)
                return false;
        }
        if (this.dontMineUnderFallingBlock) {
            const aboveIdx = this.cellIndex(x, y + 1, z);
            const aboveState = aboveIdx >= 0 ? dig.states[aboveIdx] : 0;
            if ((dig.data.flags[aboveState] & CAN_FALL) !== 0 || this.entitiesAt(x, y + 1, z) > 0) {
                return false;
            }
        }
        const state = dig.states[idx];
        if ((dig.data.flags[state] & CANT_BREAK) !== 0)
            return false;
        if (dig.breakExclusion !== null && dig.breakExclusion(x, y, z) >= 100)
            return false;
        return true;
    }
    /** Safe cells cost exclusion plus entity weight. Breakable unsafe cells
     * cost dig labor and enter the break list; other unsafe cells cost 100. */
  private safeOrBreak (x: number, y: number, z: number): number {
        let cost = this.exclusionAt(x, y, z);
        cost += this.entitiesAt(x, y, z) * this.entityCost;
        const idx = this.cellIndex(x, y, z);
        const f = idx < 0 ? 0 : this.flags[idx];
        if (this.isSafe(f))
            return cost;
        const dig = this.dig!;
        if (dig === null || idx < 0)
            return 100;
        if (!this.safeToBreak(x, y, z, idx))
            return 100;
        this.moveBreaks.push(idx);
        // Entity above a physical breakable block would fall into the hole.
        if ((f & PHYSICAL) !== 0)
            cost += this.entitiesAt(x, y + 1, z) * this.entityCost;
        cost += dig.data.labor[dig.states[idx]] * this.digCost;
        return cost;
    }
  /** Cached hazard proximity: 0 unknown, 1 clear, 2 beside a hazard. */
  private nearHazard: Uint8Array | null = null;
  private push (x: number, y: number, z: number, cost: number, meta: number): void {
        const mom = this.pendingMom;
        this.pendingMom = MOM_NONE;
        const idx = this.cellIndex(x, y, z);
        if (idx < 0)
            return; // Out-of-snapshot target; boundaryTouched is already set.
        if (this.bannedFrom >= 0 && this.edgeBanned(idx)) {
            // Discard pending metadata together with the banned edge.
            this.pendingVia = -1;
            this.pendingAim = 0;
            if (this.moveBreaks.length > 0)
                this.moveBreaks = [];
            return;
        }
        if (this.parkourExtended && this.hazardNear(x, y, z, idx))
            cost += HAZARD_NEAR_COST;
        const i = this.outCount++;
        this.outIdx[i] = idx;
        this.outX[i] = x;
        this.outY[i] = y;
        this.outZ[i] = z;
        this.outCost[i] = cost;
        this.outMeta[i] = meta;
        this.outVia[i] = this.pendingVia;
        this.pendingVia = -1;
        this.outAim[i] = this.pendingAim;
        this.pendingAim = 0;
        this.outMom[i] = mom;
        if (this.moveBreaks.length > 0) {
            this.outBreaks[i] = this.moveBreaks;
            this.moveBreaks = [];
        }
        else {
            this.outBreaks[i] = null;
        }
    }
  /** Whether the edge from bannedFrom to `to` is forbidden. */
  private edgeBanned (to: number): boolean {
        const e = this.bannedEdges as Int32Array;
        for (let k = 0; k + 1 < e.length; k += 2) {
            if (e[k] === this.bannedFrom && e[k + 1] === to)
                return true;
        }
        return false;
    }
  /** Fill the output arrays for (x, y, z). `mom` is the node's momentum
   * state, or MOM_NONE when momentum is disabled. */
  generate (x: number, y: number, z: number, mom: number = MOM_NONE): void {
        this.outCount = 0;
        this.bannedFrom = -1;
        if (this.bannedEdges !== null) {
            // Reading the source cell must not set the boundary flag.
            const lx = x - this.x0;
            const ly = y - this.y0;
            const lz = z - this.z0;
            if (lx >= 0 && lx < this.w && ly >= 0 && ly < this.h && lz >= 0 && lz < this.l) {
                const from = (ly * this.l + lz) * this.w + lx;
                for (let k = 0; k + 1 < this.bannedEdges.length; k += 2) {
                    if (this.bannedEdges[k] === from) {
                        this.bannedFrom = from;
                        break;
                    }
                }
            }
        }
        this.momIn = this.parkourMomentum ? mom : MOM_NONE;
        if (this.momIn !== MOM_NONE) {
            this.momDx = momentumDx(this.momIn);
            this.momDz = momentumDz(this.momIn);
            this.momD2 = this.momDx * this.momDx + this.momDz * this.momDz;
        }
        // Takeoff eligibility is node-invariant, so check it once per expansion.
        let ext = false;
        let jumps = false;
        let h0 = 0;
        let lowTakeoff = false;
        this.lidHere = false;
        if (this.parkourExtended) {
            // A blocked head+1 produces a bonked arc rather than forbidding jumps.
            // A body hanging on a ladder has no footing for the ground-based
            // envelope; it must climb, transfer or step off instead.
            const f0 = this.flagsAt(x, y, z);
            ext = (f0 & LIQUID) === 0 && (this.specialAt(x, y, z) & BUBBLE_MASK) === 0;
            jumps = ext && (f0 & CLIMBABLE) === 0;
            if (jumps) {
                h0 = this.heightAt(x, y - 1, z);
                lowTakeoff = !this.isSafe(this.flagsAt(x, y + 2, z));
                this.lidHere = lowTakeoff && this.lidInStrip(x, y, z);
            }
        }
        for (let i = 0; i < 4; i++) {
            const dx = CARDINAL_X[i];
            const dz = CARDINAL_Z[i];
            this.moveForward(x, y, z, dx, dz);
            this.moveJumpUp(x, y, z, dx, dz);
            this.moveDropDown(x, y, z, dx, dz);
            // The extended table replaces cardinal parkour: flat-cost jumps
            // undercut the octile heuristic and bypass reach and corridor checks.
            if (this.allowParkour && !ext)
                this.moveParkourForward(x, y, z, dx, dz);
            if (jumps) {
                const tab = this.extTable!;
                if (dx !== 0) {
                    for (const t of tab.cardX)
                        this.extTarget(x, y, z, dx, 1, h0, lowTakeoff, t);
                }
                else {
                    for (const t of tab.cardZ)
                        this.extTarget(x, y, z, 1, dz, h0, lowTakeoff, t);
                }
            }
        }
        for (let i = 0; i < 4; i++) {
            this.moveDiagonal(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i]);
            if (jumps) {
                for (const t of this.extTable!.diag) {
                    this.extTarget(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i], h0, lowTakeoff, t);
                }
            }
        }
        // With momentum in search state, the re-jump leaves the landing's own
        // state rather than becoming a compound edge.
        if (jumps && !this.parkourMomentum)
            this.momentumChains(x, y, z);
        this.moveDown(x, y, z);
        this.moveUp(x, y, z);
        if (this.special !== null) {
            this.moveBubbleUp(x, y, z);
            this.moveBubbleDown(x, y, z);
        }
        if (this.parkourExtended)
            this.climbTransfers(x, y, z);
    }
    /** Transfer between neighboring climbable cells, including pillar corners.
     * A diagonal needs one open corner column for the body to sweep through.
     * Octile distance + |dy| + 0.5 keeps the heuristic admissible. */
  private climbTransfers (x: number, y: number, z: number): void {
        const f0 = this.flagsAt(x, y, z);
        if ((f0 & CLIMBABLE) === 0 || !this.isSafe(f0) || !this.climbUsable(x, y, z))
            return;
        for (let i = 0; i < 8; i++) {
            const dx = TRANSFER_X[i];
            const dz = TRANSFER_Z[i];
            const tx = x + dx;
            const tz = z + dz;
            for (let dy = 1; dy >= -1; dy--) {
                this.beginMove();
                const ty = y + dy;
                const fT = this.flagsAt(tx, ty, tz);
                if ((fT & CLIMBABLE) === 0 || !this.isSafe(fT) || !this.climbUsable(tx, ty, tz))
                    continue;
                if (!this.isSafe(this.flagsAt(tx, ty + 1, tz)))
                    continue;
                // Climbing out of the top of this cell lifts the head a cell higher.
                if (dy === 1 && !this.isSafe(this.flagsAt(x, y + 2, z)))
                    continue;
                if (dx !== 0 && dz !== 0 &&
                    !this.cornerOpen(tx, z, y, dy) && !this.cornerOpen(x, tz, y, dy))
                    continue;
                let cost = TRANSFER_COST[i] + (dy < 0 ? -dy : dy);
                cost += this.exclusionAt(tx, ty, tz);
                cost += this.entitiesAt(tx, ty, tz) * this.entityCost;
                if (cost > 100)
                    continue;
                this.push(tx, ty, tz, cost, 0);
            }
        }
    }
    /** Both body cells of a corner column passable at the start level and,
     * for a rise or a drop, at the destination level too. */
  private cornerOpen (cx: number, cz: number, y: number, dy: number): boolean {
        if (!this.isSafe(this.flagsAt(cx, y, cz)) || !this.isSafe(this.flagsAt(cx, y + 1, cz)))
            return false;
        if (dy === 0)
            return true;
        return this.isSafe(this.flagsAt(cx, y + dy, cz)) && this.isSafe(this.flagsAt(cx, y + dy + 1, cz));
    }
    /** Connect a drop of at least two blocks to landings reachable by the slime
     * rebound, with the slime cell as `via` so search state needs no fall energy.
     * Outer-ring targets reserve more height for drift; octile distance + |dy|
     * + 1 keeps the heuristic admissible. */
  private slimeBounce (x: number, y: number, z: number, sx: number, sy: number, sz: number): void {
        const supIdx = this.cellIndex(sx, sy - 1, sz);
        if (supIdx < 0 || (this.special as Uint8Array)[supIdx] !== SPECIAL_SLIME)
            return;
        // Water over the slime damps the rebound.
        if ((this.flagsAt(sx, sy, sz) & LIQUID) !== 0)
            return;
        const d = y - sy;
        if (d < 2)
            return;
        const apex = BOUNCE_APEX[d > BOUNCE_MAX_DROP ? BOUNCE_MAX_DROP : d];
        const viaIdx = this.cellIndex(sx, sy, sz);
        for (let i = 0; i < BOUNCE_OFFSETS; i++) {
            const cx = sx + BOUNCE_X[i];
            const cz = sz + BOUNCE_Z[i];
            const maxTop = sy + apex - (i < BOUNCE_RING1 ? BOUNCE_MARGIN : BOUNCE_MARGIN_FAR);
            const lid = Math.floor(maxTop);
            if (lid < sy)
                continue;
            // Ring 2 sweeps through the cell between: it must be open over the
            // whole climb band.
            if (i >= BOUNCE_RING1) {
                const mx = sx + (BOUNCE_X[i] >> 1);
                const mz = sz + (BOUNCE_Z[i] >> 1);
                let open = true;
                for (let ly = sy + 1; ly <= lid + 1; ly++) {
                    if (!this.isSafe(this.flagsAt(mx, ly, mz))) {
                        open = false;
                        break;
                    }
                }
                if (!open)
                    continue;
            }
            // Scan the column down from the lid for the first support; every cell
            // above it must be passable (the body rises through them).
            let ly = lid;
            let nodeY = NaN;
            while (ly >= sy) {
                const idx = this.cellIndex(cx, ly, cz);
                if (idx < 0)
                    break;
                const f = this.flags[idx];
                if (this.isSafe(f)) {
                    ly--;
                    continue;
                }
                if ((f & PHYSICAL) !== 0 && !this.isThinFloor(f)) {
                    if (ly + (this.heights[idx] & 63) / 32 <= maxTop)
                        nodeY = ly + 1;
                }
                else if (this.isTallStand(f, this.heights[idx])) {
                    if (ly + (this.heights[idx] & 63) / 32 <= maxTop && this.isSafe(this.flagsAt(cx, ly + 3, cz)))
                        nodeY = ly + 1;
                }
                break;
            }
            if (Number.isNaN(nodeY) || nodeY <= sy)
                continue;
            if (!this.isSafe(this.flagsAt(cx, nodeY, cz)) || !this.isSafe(this.flagsAt(cx, nodeY + 1, cz)))
                continue;
            this.beginMove();
            const adx = cx - x;
            const adz = cz - z;
            const ax = adx < 0 ? -adx : adx;
            const az = adz < 0 ? -adz : adz;
            let cost = (ax > az ? ax - az : az - ax) + Math.SQRT2 * (ax < az ? ax : az) + (y - nodeY) + 1;
            cost += this.exclusionAt(cx, nodeY, cz);
            cost += this.entitiesAt(cx, nodeY, cz) * this.entityCost;
            if (cost > 100)
                continue;
            this.pendingVia = viaIdx;
            this.push(cx, nodeY, cz, cost, META_PARKOUR | META_BOUNCE);
        }
    }
    /** Ride up one block; the target is the current head cell, so only the
     * new head cell adds clearance cost. */
  private moveBubbleUp (x: number, y: number, z: number): void {
        this.beginMove();
        if (this.specialAt(x, y, z) !== BUBBLE_UP)
            return;
        let cost = this.bubbleCost;
        cost += this.safeOrBreak(x, y + 2, z);
        if (cost > 100)
            return;
        this.push(x, y + 1, z, cost, 0);
    }
    /** Sink one block in a downward bubble column. */
  private moveBubbleDown (x: number, y: number, z: number): void {
        this.beginMove();
        if (this.specialAt(x, y, z) !== BUBBLE_DOWN)
            return;
        let cost = this.bubbleCost;
        cost += this.safeOrBreak(x, y - 1, z);
        if (cost > 100)
            return;
        this.push(x, y - 1, z, cost, 0);
    }
  private moveForward (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        const fC = this.flagsAt(x + dx, y, z + dz);
        const fD = this.flagsAt(x + dx, y - 1, z + dz);
        let cost = 1; 
        cost += this.exclusionAt(x + dx, y, z + dz);
        // Without scaffolding, unsupported moves require water or bubble support.
        if ((fD & PHYSICAL) === 0 && (fC & LIQUID) === 0 &&
            (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) === 0) {
            if (!this.parkourExtended)
                return;
            if (this.tallStandAt(x + dx, y - 1, z + dz)) {
                // A fence or wall one cell below is within step height;
                // the feet rest 0.5 blocks into the target cell.
                if (this.heightAt(x + dx, y - 1, z + dz) - this.heightAt(x, y - 1, z) > 0.6)
                    return;
                if (!this.isSafe(this.flagsAt(x + dx, y + 2, z + dz)))
                    return;
            }
            else if ((fC & CLIMBABLE) === 0 || !this.isSafe(fC) || !this.climbUsable(x + dx, y, z + dz)) {
                return;
            }
        }
        else if (this.isThinFloor(fD)) {
            // A thin floor below needs a drop move; a same-level node would float.
            return;
        }
        const activatable = (fC & GATE) !== 0 || (this.doorMode && (fC & DOOR_CLOSED) !== 0);
        const throughClosedDoor = this.canOpenDoors && this.doorMode && (fC & DOOR_CLOSED) !== 0;
        // Opening a door's lower half also opens its upper half. In door mode,
        // the upper half must retain exclusion and entity costs without vetoing entry.
        const fB = this.flagsAt(x + dx, y + 1, z + dz);
        if (throughClosedDoor && (fB & DOOR_CLOSED) !== 0) {
            cost += this.exclusionAt(x + dx, y + 1, z + dz);
            cost += this.entitiesAt(x + dx, y + 1, z + dz) * this.entityCost;
        }
        else {
            cost += this.safeOrBreak(x + dx, y + 1, z + dz);
        }
        if (cost > 100)
            return;
        let meta = 0;
        if (this.canOpenDoors && activatable) {
            meta = META_USEONE;
        }
        else {
            cost += this.safeOrBreak(x + dx, y, z + dz); 
            if (cost > 100)
                return;
        }
        if ((this.flagsAt(x, y, z) & LIQUID) !== 0)
            cost += this.liquidCost;
        this.push(x + dx, y, z + dz, cost, meta);
    }
  private moveJumpUp (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        const fA = this.flagsAt(x, y + 2, z);
        const fH = this.flagsAt(x + dx, y + 2, z + dz);
        const fB = this.flagsAt(x + dx, y + 1, z + dz);
        const fC = this.flagsAt(x + dx, y, z + dz);
        let cost = 2; 
        // Entities above a removed support can fall into the jump path.
        if ((fA & PHYSICAL) !== 0 && this.entitiesAt(x, y + 3, z) > 0)
            return;
        if ((fH & PHYSICAL) !== 0 && this.entitiesAt(x + dx, y + 3, z + dz) > 0)
            return;
        if ((fB & PHYSICAL) !== 0 && (fH & PHYSICAL) === 0 && (fC & PHYSICAL) === 0 &&
            this.entitiesAt(x + dx, y + 2, z + dz) > 0)
            return;
        if ((fC & PHYSICAL) === 0) {
            // A fence or wall raises the feet 0.5 blocks into the node cell,
            // so the head needs clearance one cell higher.
            if (!this.parkourExtended || !this.tallStandAt(x + dx, y, z + dz))
                return;
            if (!this.isSafe(this.flagsAt(x + dx, y + 3, z + dz)))
                return;
        }
        else if (this.isThinFloor(fC)) {
            // A thin floor at feet level is walkable; jumping onto it would land in air.
            return;
        }
        else if (this.parkourExtended && (fC & CLIMBABLE) !== 0) {
            // A ladder is physical, but its 3/16 top edge is not a viable hop
            // target here. Enter the climbable cell instead.
            return;
        }
        const hC = this.heightAt(x + dx, y, z + dz);
        const h0 = this.heightAt(x, y - 1, z);
        if (hC - h0 > 1.2)
            return; 
        let latched = false;
        // Only the panel's open side permits this takeoff; leaving its end
        // would require walking into the target block first.
        if (!this.isSafe(fA) && this.parkourExtended && this.lidInStrip(x, y, z) && carrySide(this.carryAt(x, y - 1, z), dx, dz) === 1)
            latched = true; // The ground flag persists on the tick the body leaves the strip.
        else
            cost += this.safeOrBreak(x, y + 2, z); 
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + 2, z + dz); 
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + 1, z + dz); 
        if (cost > 100)
            return;
        this.push(x + dx, y + 1, z + dz, cost, latched ? META_PARKOUR : 0);
    }
    /** Scan down for a landing node: above a support or inside a catching cell.
     * Return NaN when no landing is available. */
  private findLanding (x: number, y: number, z: number, dx: number, dz: number): number {
        const lx = x + dx;
        const lz = z + dz;
        let ly = y - 2;
        this.landClutch = false;
        while (ly > this.worldMinY) {
            const idx = this.cellIndex(lx, ly, lz);
            if (idx < 0)
                return NaN; 
            const f = this.flags[idx];
            if ((f & LIQUID) !== 0 && this.isSafe(f))
                return ly;
            // Bubble columns catch falls like water.
            if (this.special !== null && (this.special[idx] & BUBBLE_MASK) !== 0)
                return ly;
            // A usable ladder or vine can catch the drop.
            if (this.parkourExtended && (f & CLIMBABLE) !== 0 && this.isSafe(f) && this.climbUsable(lx, ly, lz)) {
                if (y - ly <= this.maxDropDown)
                    return ly;
                return NaN;
            }
            // Thin floor (carpet class): feet land IN the cell, not on top of it.
            if (this.isThinFloor(f)) {
                if (y - ly <= this.maxDropDown)
                    return ly;
                return NaN;
            }
            if ((f & PHYSICAL) !== 0) {
                if (y - ly <= this.maxDropDown)
                    return ly + 1;
                // Too deep to walk off: a water bucket can still land it on a plain full top.
                if (this.clutchDropDown > 0 && y - ly > this.clutchMin && y - ly <= this.clutchDropDown + 1 && this.clutchFits(lx, ly, lz, idx)) {
                    this.landClutch = true;
                    return ly + 1;
                }
                return NaN;
            }
            // A fence or wall raises the feet and head 0.5 blocks into the cells above.
            if (this.parkourExtended && this.isTallStand(f, this.heights[idx])) {
                if (y - ly <= this.maxDropDown && this.isSafe(this.flagsAt(lx, ly + 3, lz)))
                    return ly + 1;
                return NaN;
            }
            if (!this.isSafe(f))
                return NaN;
            ly--;
        }
        return NaN;
    }
  /** A clutch landing: a full, plain top (no catch) with open air above and no lava beside. Water over slime still catches. */
  private clutchFits (lx: number, ly: number, lz: number, idx: number): boolean {
        const ht = this.heights[idx];
        if ((ht & 63) !== 32 || (ht >> 6) !== 0)
            return false;
        const above = this.flagsAt(lx, ly + 1, lz);
        if ((above & (LIQUID | CLIMBABLE)) !== 0 || !this.isSafe(above))
            return false;
        return !this.lavaFlanked(lx, ly + 1, lz);
    }
  /** A clutch drop starts from solid footing: the bucket is staged on the ground, and the fall counts from there. */
  private clutchSource (x: number, y: number, z: number): boolean {
        if ((this.flagsAt(x, y - 1, z) & PHYSICAL) === 0)
            return false;
        return (this.flagsAt(x, y, z) & (LIQUID | CLIMBABLE)) === 0;
    }
  private moveDropDown (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        let cost = 1;
        const landY = this.findLanding(x, y, z, dx, dz);
        if (Number.isNaN(landY))
            return;
        const clutch = this.landClutch;
        if (!clutch && !this.infiniteLiquidDropdownDistance && (y - landY) > this.maxDropDown)
            return;
        if (clutch && !this.clutchSource(x, y, z))
            return;
        cost += this.safeOrBreak(x + dx, y + 1, z + dz); 
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y, z + dz); 
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y - 1, z + dz); 
        if (cost > 100)
            return;
        if ((this.flagsAt(x + dx, y, z + dz) & LIQUID) !== 0)
            return; 
        cost += this.entitiesAt(x + dx, landY, z + dz) * this.entityCost;
        if (clutch) {
            // The fall itself, so a staircase wins unless the drop saves real distance (lib.rs move_drop_down).
            cost += y - landY;
            cost += this.clutchCost;
            this.push(x + dx, landY, z + dz, cost, META_CLUTCH | META_DEEP);
            return;
        }
        this.push(x + dx, landY, z + dz, cost, (y - landY) > this.maxDropDown ? META_DEEP : 0);
        if (this.parkourExtended && this.special !== null)
            this.slimeBounce(x, y, z, x + dx, landY, z + dz);
    }
  private moveDown (x: number, y: number, z: number): void {
        this.beginMove();
        // Can't descend against an up-column's push (ride edges handle columns).
        if (this.specialAt(x, y, z) === BUBBLE_UP)
            return;
        let cost = 1; 
        const landY = this.findLanding(x, y, z, 0, 0);
        if (Number.isNaN(landY))
            return;
        // Straight down is also the dig-down move: no clutch here.
        if (this.landClutch)
            return;
        cost += this.safeOrBreak(x, y - 1, z);
        if (cost > 100)
            return;
        if ((this.flagsAt(x, y, z) & LIQUID) !== 0)
            return; 
        cost += this.entitiesAt(x, landY, z) * this.entityCost;
        this.push(x, landY, z, cost, (y - landY) > this.maxDropDown ? META_DEEP : 0);
        if (this.parkourExtended && this.special !== null)
            this.slimeBounce(x, y, z, x, landY, z);
    }
  private moveUp (x: number, y: number, z: number): void {
        this.beginMove();
        const f1 = this.flagsAt(x, y, z);
        if ((f1 & LIQUID) !== 0)
            return;
        if (this.entitiesAt(x, y, z) > 0)
            return;
        let cost = 1; 
        cost += this.safeOrBreak(x, y + 2, z); 
        if (cost > 100)
            return;
        if ((f1 & CLIMBABLE) === 0) {
            // A jump reaches a climbable cell starting at head height,
            // allowing the climb to continue for the cost of a jump up.
            if (!this.parkourExtended)
                return;
            const fH = this.flagsAt(x, y + 1, z);
            if ((fH & CLIMBABLE) === 0 || !this.isSafe(fH) || !this.climbUsable(x, y + 1, z))
                return;
            this.push(x, y + 1, z, cost + 1, 0);
            return;
        }
        // Vines require an adjacent solid block for collision climbing;
        // a free-hanging curtain is passable but cannot be climbed.
        if (this.specialAt(x, y, z) === SPECIAL_VINE &&
            (this.flagsAt(x + 1, y, z) & PHYSICAL) === 0 &&
            (this.flagsAt(x - 1, y, z) & PHYSICAL) === 0 &&
            (this.flagsAt(x, y, z + 1) & PHYSICAL) === 0 &&
            (this.flagsAt(x, y, z - 1) & PHYSICAL) === 0)
            return;
        this.push(x, y + 1, z, cost, 0);
    }
  private moveDiagonal (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        let cost = Math.SQRT2; 
        const fC = this.flagsAt(x + dx, y, z + dz);
        // Ladder-top edges are not diagonal hop targets; enter or catch the cell instead.
        if (this.parkourExtended && (fC & (PHYSICAL | CLIMBABLE)) === (PHYSICAL | CLIMBABLE))
            return;
        // A thin floor at the target feet cell is same-level ground, not a +1 hop.
        const yo = (fC & PHYSICAL) !== 0 && !this.isThinFloor(fC) ? 1 : 0;
        // Hazards have no collision box to deflect a diagonal past their corners.
        // Check the body's full height, including the rise during a jump.
        if (this.parkourExtended) {
            for (let hy = y; hy <= y + yo + 1; hy++) {
                if (this.hazardAt(x, hy, z + dz) || this.hazardAt(x + dx, hy, z))
                    return;
            }
        }
        const h0 = this.heightAt(x, y - 1, z);
        // Each corner needs its own break list; keep the cheaper corner's digs.
        let cost1 = 0;
        cost1 += this.safeOrBreak(x, y + yo + 1, z + dz); 
        cost1 += this.safeOrBreak(x, y + yo, z + dz); 
        const hD1 = this.heightAt(x, y + yo - 1, z + dz);
        if (hD1 - h0 > 1.2)
            cost1 += this.safeOrBreak(x, y + yo - 1, z + dz); 
        const breaks1 = this.moveBreaks;
        this.moveBreaks = [];
        let cost2 = 0;
        cost2 += this.safeOrBreak(x + dx, y + yo + 1, z); 
        cost2 += this.safeOrBreak(x + dx, y + yo, z); 
        const hD2 = this.heightAt(x + dx, y + yo - 1, z);
        if (hD2 - h0 > 1.2)
            cost2 += this.safeOrBreak(x + dx, y + yo - 1, z); 
        const breaks2 = this.moveBreaks;
        // Keep the chosen corner's cost and break list together; ties select corner 2.
        if (cost1 < cost2) {
            cost += cost1;
            this.moveBreaks = breaks1;
        }
        else {
            cost += cost2;
        }
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + yo, z + dz);
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + yo + 1, z + dz);
        if (cost > 100)
            return;
        if ((this.flagsAt(x, y, z) & LIQUID) !== 0)
            cost += this.liquidCost;
        const fD = this.flagsAt(x + dx, y - 1, z + dz);
        if (yo === 1) { 
            const hC = this.heightAt(x + dx, y, z + dz);
            if (hC - h0 > 1.2)
                return; 
            cost += this.safeOrBreak(x, y + 2, z);
            if (cost > 100)
                return;
            cost += 1;
            this.push(x + dx, y + 1, z + dz, cost, 0);
        }
        else if (((fD & PHYSICAL) !== 0 && !this.isThinFloor(fD)) || (fC & LIQUID) !== 0 ||
            this.isThinFloor(fC) ||
            (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) !== 0) {
            // Edge supports form continuous ground only where their faces meet.
            // Disconnected ledges need a hop because the body loses support.
            let hop = false;
            if (this.parkourExtended) {
                const cA = this.carryAt(x, y - 1, z);
                const cB = cA === 0 ? 0 : this.carryAt(x + dx, y - 1, z + dz);
                hop = cB !== 0 && !carryTouch(cA, cB, dx, dz);
            }
            this.push(x + dx, y, z + dz, hop ? cost + 0.5 : cost, hop ? META_PARKOUR : 0);
        }
        else if ((this.flagsAt(x + dx, y - 2, z + dz) & PHYSICAL) !== 0 || (fD & LIQUID) !== 0) {
            if (!this.isSafe(fD))
                return; 
            cost += this.entitiesAt(x + dx, y - 1, z + dz) * this.entityCost;
            this.push(x + dx, y - 1, z + dz, cost, 0);
        }
    }
    // Gap jumps never dig through their flight path.
  private moveParkourForward (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        const h0 = this.heightAt(x, y - 1, z);
        const f1 = this.flagsAt(x + dx, y - 1, z + dz);
        const h1 = this.heightAt(x + dx, y - 1, z + dz);
        if (((f1 & PHYSICAL) !== 0 && h1 >= h0) ||
            !this.isSafe(this.flagsAt(x + dx, y, z + dz)) ||
            !this.isSafe(this.flagsAt(x + dx, y + 1, z + dz)))
            return;
        if ((this.flagsAt(x, y, z) & LIQUID) !== 0)
            return; 
        if ((this.specialAt(x, y, z) & BUBBLE_MASK) !== 0)
            return; 
        let cost = 1;
        cost += this.entitiesAt(x + dx, y, z + dz) * this.entityCost;
        let ceilingClear = this.isSafe(this.flagsAt(x, y + 2, z)) && this.isSafe(this.flagsAt(x + dx, y + 2, z + dz));
        let floorCleared = (this.flagsAt(x + dx, y - 2, z + dz) & PHYSICAL) === 0;
        const maxD = this.allowSprinting ? 4 : 2;
        for (let d = 2; d <= maxD; d++) {
            const dxx = dx * d;
            const dzz = dz * d;
            const fA = this.flagsAt(x + dxx, y + 2, z + dzz);
            const fB = this.flagsAt(x + dxx, y + 1, z + dzz);
            const fCd = this.flagsAt(x + dxx, y, z + dzz);
            const fDd = this.flagsAt(x + dxx, y - 1, z + dzz);
            if (this.isSafe(fCd))
                cost += this.entitiesAt(x + dxx, y, z + dzz) * this.entityCost;
            if (ceilingClear && this.isSafe(fB) && this.isSafe(fCd) && (fDd & PHYSICAL) !== 0) {
                cost += this.exclusionAt(x + dxx, y + 1, z + dzz);
                if (!this.lavaFlanked(x + dxx, y, z + dzz))
                    this.push(x + dxx, y, z + dzz, cost, META_PARKOUR);
                break;
            }
            else if (ceilingClear && this.isSafe(fB) && (fCd & PHYSICAL) !== 0) {
                if (this.isSafe(fA) && d !== 4) { 
                    cost += this.exclusionAt(x + dxx, y + 2, z + dzz);
                    const hC = this.heightAt(x + dxx, y, z + dzz);
                    if (hC - h0 > 1.2)
                        break; 
                    cost += this.entitiesAt(x + dxx, y + 1, z + dzz) * this.entityCost;
                    if (!this.lavaFlanked(x + dxx, y + 1, z + dzz))
                        this.push(x + dxx, y + 1, z + dzz, cost, META_PARKOUR);
                    break;
                }
            }
            else if ((ceilingClear || d === 2) && this.isSafe(fB) && this.isSafe(fCd) && this.isSafe(fDd) && floorCleared) {
                const fE = this.flagsAt(x + dxx, y - 2, z + dzz);
                if ((fE & PHYSICAL) !== 0) {
                    cost += this.exclusionAt(x + dxx, y - 1, z + dzz);
                    cost += this.entitiesAt(x + dxx, y - 1, z + dzz) * this.entityCost;
                    if (!this.lavaFlanked(x + dxx, y - 1, z + dzz))
                        this.push(x + dxx, y - 1, z + dzz, cost, META_PARKOUR);
                }
                floorCleared = floorCleared && (fE & PHYSICAL) === 0;
            }
            else if (!this.isSafe(fB) || !this.isSafe(fCd)) {
                break;
            }
            ceilingClear = ceilingClear && this.isSafe(fA);
        }
    }
    /** Chain jumps through stepping stones that have no adjacent run-up.
     * A landing-tick re-jump retains speed; only direction-aligned targets
     * beyond the stone's normal reach become compound edges with it as `via`.
     * Summing both edge costs keeps the heuristic admissible. */
  private momentumChains (x: number, y: number, z: number): void {
        const nOut = this.outCount;
        const tab = this.extTable!;
        for (let i = 0; i < nOut; i++) {
            if (this.outMeta[i] !== META_PARKOUR)
                continue;
            const bx = this.outX[i];
            const by = this.outY[i];
            const bz = this.outZ[i];
            // Catches have no support landing tick on which to re-jump.
            const fB = this.flagsAt(bx, by, bz);
            if ((fB & (LIQUID | CLIMBABLE)) !== 0 || this.isThinFloor(fB) ||
                (this.specialAt(bx, by, bz) & BUBBLE_MASK) !== 0)
                continue;
            const idxS = this.cellIndex(bx, by - 1, bz);
            if (idxS < 0)
                continue;
            const fS = this.flags[idxS];
            if ((fS & PHYSICAL) === 0 && !this.isTallStand(fS, this.heights[idxS]))
                continue;
            // Adjacent walkable ground already gives the stone running jumps.
            const hB = this.heightAt(bx, by - 1, bz);
            let runnable = false;
            for (let d = 0; d < 4; d++) {
                const nx = bx + CARDINAL_X[d];
                const nz = bz + CARDINAL_Z[d];
                if ((this.flagsAt(nx, by - 1, nz) & PHYSICAL) !== 0 && this.catchAt(nx, by - 1, nz) === 0 &&
                    this.isSafe(this.flagsAt(nx, by, nz)) && this.isSafe(this.flagsAt(nx, by + 1, nz))) {
                    const hN = this.heightAt(nx, by - 1, nz);
                    if (hN - hB <= 0.2 && hB - hN <= 0.2) {
                        runnable = true;
                        break;
                    }
                }
            }
            if (runnable)
                continue;
            const lowB = !this.isSafe(this.flagsAt(bx, by + 2, bz));
            if (lowB)
                continue; // No bonked chain row is available.
            const abx = bx - x;
            const abz = bz - z;
            const ab2 = abx * abx + abz * abz;
            const viaIdx = this.outIdx[i];
            const baseCost = this.outCost[i];
            for (let q = 0; q < 4; q++) {
                const sx = DIAGONAL_X[q];
                const sz = DIAGONAL_Z[q];
                for (const t of tab.diag) {
                    if (!this.chainAligned(abx, abz, ab2, t.tx * sx, t.tz * sz))
                        continue;
                    this.parkourExtTarget(bx, by, bz, sx, sz, hB, false, t, viaIdx, baseCost);
                }
            }
            for (let d = 0; d < 4; d++) {
                const dx = CARDINAL_X[d];
                const dz = CARDINAL_Z[d];
                const group = dx !== 0 ? tab.cardX : tab.cardZ;
                for (const t of group) {
                    const bcx = dx !== 0 ? t.tx * dx : 0;
                    const bcz = dx !== 0 ? 0 : t.tz * dz;
                    if (!this.chainAligned(abx, abz, ab2, bcx, bcz))
                        continue;
                    this.parkourExtTarget(bx, by, bz, dx !== 0 ? dx : 1, dx !== 0 ? 1 : dz, hB, false, t, viaIdx, baseCost);
                }
            }
            if (this.outCount - nOut >= CHAIN_CAP)
                return;
        }
    }
    /** The second hop continues the first: cos(angle) ≥ CHAIN_MIN_COS, in exact integer arithmetic. */
  private chainAligned (abx: number, abz: number, ab2: number, bcx: number, bcz: number): boolean {
        const dot = abx * bcx + abz * bcz;
        if (dot <= 0)
            return false;
        return dot * dot >= CHAIN_MIN_COS2 * ab2 * (bcx * bcx + bcz * bcz);
    }
  /** A lid inside the support panel's strip bonks a jump from the strip.
     * On the tick the body leaves it, vertical movement resolves first and
     * retains the ground flag; jumping then clears the lid. */
  private lidInStrip (x: number, y: number, z: number): boolean {
        const hb = this.thinShapeAt(x, y + 2, z);
        const sb = this.thinShapeAt(x, y - 1, z);
        if (hb === 0 || sb === 0)
            return false;
        const c = this.carryAt(x, y - 1, z);
        if (c < CARRY_W_NONE || c > CARRY_S_NONE)
            return false;
        if ((this.flagsAt(x, y + 2, z) & LIQUID) !== 0)
            return false;
        return THIN_LO[hb & 15] >= THIN_LO[sb & 15] && THIN_HI[hb & 15] <= THIN_HI[sb & 15] &&
            THIN_LO[hb >> 4] >= THIN_LO[sb >> 4] && THIN_HI[hb >> 4] <= THIN_HI[sb >> 4];
    }
    /** Whether a step leaves the panel's strip without entering level support
     * that would keep the body carried under the next cell's lid. */
  private leavesLid (x: number, y: number, z: number, dx: number, dz: number, h0: number): boolean {
        const c = this.carryAt(x, y - 1, z);
        if ((dx !== 0 && carrySide(c, dx, 0) === 1) || (dz !== 0 && carrySide(c, 0, dz) === 1))
            return true;
        if (dx !== 0 && this.levelSupport(x + dx, y - 1, z, h0))
            return false;
        if (dz !== 0 && this.levelSupport(x, y - 1, z + dz, h0))
            return false;
        if (dx !== 0 && dz !== 0 && this.levelSupport(x + dx, y - 1, z + dz, h0))
            return false;
        return dx !== 0 || dz !== 0;
    }
  private levelSupport (x: number, y: number, z: number, h0: number): boolean {
        return (this.flagsAt(x, y, z) & PHYSICAL) !== 0 && Math.abs(this.heightAt(x, y, z) - h0) < 0.07;
    }
  /** Floor beneath an obstructing thin shape is not a walkable route;
   * it must not veto a jump that passes beside the shape. */
  private thinStands (cx: number, y: number, cz: number): boolean {
        return (this.thinShapeAt(cx, y, cz) !== 0 && !this.isSafe(this.flagsAt(cx, y, cz))) ||
            (this.thinShapeAt(cx, y + 1, cz) !== 0 && !this.isSafe(this.flagsAt(cx, y + 1, cz)));
    }
    /** Footprint byte; a lid byte reads as 0, meaning a full-footprint block. */
  private thinShapeAt (cx: number, ly: number, cz: number): number {
        const b = this.thinAt(cx, ly, cz);
        return isLidByte(b) ? 0 : b;
    }
    /** Lid list of the candidate being evaluated: bonk height, path span. */
  private readonly lidHb = new Float64Array(LID_MAX);
  private readonly lidLo = new Float64Array(LID_MAX);
  private readonly lidHi = new Float64Array(LID_MAX);
  private spanLo = 0;
  private readonly grHb = new Float64Array(16);
  private readonly grLo = new Float64Array(16);
  private readonly grHi = new Float64Array(16);
  private spanHi = 0;
    /** Path interval where the hitbox overlaps a cell's thin footprint;
     * false when the path stays clear. */
  private thinSpan (cx: number, ly: number, cz: number, ax: number, az: number, t: ParkourExtEntry, sx: number, sz: number): boolean {
        const b = this.thinShapeAt(cx, ly, cz);
        let x0 = THIN_LO[b & 15] / 16;
        let x1 = THIN_HI[b & 15] / 16;
        let z0 = THIN_LO[b >> 4] / 16;
        let z1 = THIN_HI[b >> 4] / 16;
        if (sx < 0) { const lo = 1 - x1; x1 = 1 - x0; x0 = lo; }
        if (sz < 0) { const lo = 1 - z1; z1 = 1 - z0; z0 = lo; }
        x0 += ax - THIN_BODY_HALF; x1 += ax + THIN_BODY_HALF;
        z0 += az - THIN_BODY_HALF; z1 += az + THIN_BODY_HALF;
        const p = t.pts;
        let acc = 0;
        let lo = Infinity;
        let hi = -Infinity;
        for (let i = 0; i + 3 < p.length; i += 2) {
            const dx = p[i + 2] - p[i];
            const dz = p[i + 3] - p[i + 1];
            const len = Math.sqrt(dx * dx + dz * dz);
            let t0 = 0;
            let t1 = 1;
            let miss = false;
            if (dx === 0) { if (p[i] <= x0 || p[i] >= x1) miss = true; }
            else { let e = (x0 - p[i]) / dx; let q = (x1 - p[i]) / dx; if (e > q) { const w = e; e = q; q = w; } if (e > t0) t0 = e; if (q < t1) t1 = q; }
            if (dz === 0) { if (p[i + 1] <= z0 || p[i + 1] >= z1) miss = true; }
            else { let e = (z0 - p[i + 1]) / dz; let q = (z1 - p[i + 1]) / dz; if (e > q) { const w = e; e = q; q = w; } if (e > t0) t0 = e; if (q < t1) t1 = q; }
            if (!miss && t1 - t0 > 1e-9) {
                if (acc + t0 * len < lo) lo = acc + t0 * len;
                if (acc + t1 * len > hi) hi = acc + t1 * len;
            }
            acc += len;
        }
        this.spanLo = lo;
        this.spanHi = hi;
        return hi >= lo;
    }
    /** Walk an arc against lids; vertical motion resolves before horizontal.
     * A lid overhead caps the rise and resets vertical velocity. Entering a
     * lid's span above its bonk height is a side collision and returns NaN.
     * Return the cap, or Infinity with no bonk, stopping at touchdown. */
  private lidWalkWindow (n: number, r: number, s0: number, landRise: number, landIn: number): number {
        // Running takeoffs vary within a stride, so the whole phase window
        // must clear the lids; a standing takeoff has a fixed position.
        if (r === 0)
            return this.lidWalk(n, r, s0, landRise, landIn);
        let cap = Infinity;
        for (let k = -2; k <= 2; k++) {
            const c = this.lidWalk(n, r, s0 + k * 0.07, landRise, landIn);
            if (Number.isNaN(c))
                return NaN;
            if (c < cap)
                cap = c;
        }
        return cap;
    }
  private lidWalk (n: number, r: number, s0: number, landRise: number, landIn: number): number {
        const curve = FLIGHT_ROW[r];
        let capped = false;
        let cap = Infinity;
        let yy = 0;
        let vy = 0;
        let sPrev = s0;
        for (let i = 0; i < curve.length; i += 2) {
            let yn;
            if (!capped) {
                yn = curve[i + 1];
                let hbHere = Infinity;
                for (let j = 0; j < n; j++) {
                    if (this.lidLo[j] <= sPrev && sPrev <= this.lidHi[j] && this.lidHb[j] < hbHere)
                        hbHere = this.lidHb[j];
                }
                if (yn > hbHere) {
                    capped = true;
                    cap = hbHere;
                    yn = hbHere;
                    vy = 0;
                }
            }
            else {
                vy = (vy - 0.08) * 0.98;
                yn = yy + vy;
            }
            yy = yn;
            const sNext = s0 + curve[i];
            for (let j = 0; j < n; j++) {
                if (this.lidLo[j] > sPrev && this.lidLo[j] <= sNext && yy > this.lidHb[j])
                    return NaN;
            }
            sPrev = sNext;
            if (yy <= landRise && sNext >= landIn)
                break;
        }
        return cap;
    }
    /** Thin footprint byte of a cell (0 = solid, or no thin grid). */
  private thinAt (cx: number, ly: number, cz: number): number {
        if (this.thin === null)
            return 0;
        const idx = this.cellIndex(cx, ly, cz);
        return idx < 0 ? 0 : this.thin[idx];
    }
    /** A blocked corridor cell can contain a thin shape the hitbox misses.
     * Mirror its world footprint into table space before checking the path;
     * byte 0 represents a solid cell and always vetoes it. */
  private thinClear (cx: number, ly: number, cz: number, t: ParkourExtEntry, k: number, sx: number, sz: number): boolean {
        const b = this.thinShapeAt(cx, ly, cz);
        if (b === 0)
            return false;
        let x0 = THIN_LO[b & 15] / 16;
        let x1 = THIN_HI[b & 15] / 16;
        let z0 = THIN_LO[b >> 4] / 16;
        let z1 = THIN_HI[b >> 4] / 16;
        if (sx < 0) {
            const lo = 1 - x1;
            x1 = 1 - x0;
            x0 = lo;
        }
        if (sz < 0) {
            const lo = 1 - z1;
            z1 = 1 - z0;
            z0 = lo;
        }
        const ax = t.cells[k * 2];
        const az = t.cells[k * 2 + 1];
        x0 += ax - THIN_BODY_HALF;
        x1 += ax + THIN_BODY_HALF;
        z0 += az - THIN_BODY_HALF;
        z1 += az + THIN_BODY_HALF;
        const p = t.pts;
        for (let i = 0; i + 3 < p.length; i += 2) {
            if (segHits(p[i], p[i + 1], p[i + 2] - p[i], p[i + 3] - p[i + 1], x0, x1, z0, z1))
                return false;
        }
        return true;
    }
    /** Try shifted lines after a corridor refusal; a successful centre line
     * makes them redundant. Momentum chains cannot shift without a line-up.
     * A solid-cell refusal also excludes sibling lines sweeping that cell. */
  private extTarget (x: number, y: number, z: number, sx: number, sz: number, h0: number, lowTakeoff: boolean, t: ParkourExtEntry): void {
        this.extTargetLines(x, y, z, sx, sz, h0, lowTakeoff, t);
        // A head-height climbable catch is separate from the column's landing,
        // so a ladder above a floor remains a candidate.
        const tx = x + t.tx * sx;
        const tz = z + t.tz * sz;
        const fU = this.flagsAt(tx, y + 1, tz);
        if ((fU & CLIMBABLE) !== 0 && this.isSafe(fU) && this.climbUsable(tx, y + 1, tz)) {
            this.extUpCatch = true;
            this.parkourExtTarget(x, y, z, sx, sz, h0, lowTakeoff, t);
            this.extUpCatch = false;
        }
    }
  private extTargetLines (x: number, y: number, z: number, sx: number, sz: number, h0: number, lowTakeoff: boolean, t: ParkourExtEntry): void {
        this.extTryMask = 0;
        this.parkourExtTarget(x, y, z, sx, sz, h0, lowTakeoff, t);
        let mask = this.extTryMask;
        if (mask === 0)
            return;
        // An offset takeoff needs a full support top; narrow supports only
        // allow variants that keep the takeoff centred.
        if (this.catchAt(x, y - 1, z) !== 0 || this.carryAt(x, y - 1, z) !== 0) {
            mask &= AIM_P0_MASK;
            if (mask === 0)
                return;
        }
        const vs = t.variants;
        for (let v = 0; v < vs.length && mask !== 0; v++) {
            if ((mask >> v & 1) === 0)
                continue;
            this.extTryMask = -1;
            this.parkourExtTarget(x, y, z, sx, sz, h0, lowTakeoff, vs[v]);
            mask &= this.extTryMask;
        }
    }
  /** Evaluate one extended-parkour landing without digging.
   * With chainVia, use the landing-tick re-jump envelope, skip targets within
   * normal reach, and retain the first hop's cost and stepping stone. */
  private parkourExtTarget (x: number, y: number, z: number, sx: number, sz: number, h0: number, lowTakeoff: boolean, t: ParkourExtEntry, chainVia = -1, chainBase = 0): void {
        this.beginMove();
        const cells = t.cells;
        // Walkable ground on the first flight-line cell means there is no gap.
        const flx = x + cells[0] * sx;
        const flz = z + cells[1] * sz;
        // Floor under an obstruction is not walkable; a shifted jump can pass beside it.
        if ((this.flagsAt(flx, y - 1, flz) & PHYSICAL) !== 0 && this.heightAt(flx, y - 1, flz) >= h0 &&
            !this.thinStands(flx, y, flz) &&
            this.isSafe(this.flagsAt(flx, y, flz)) && this.isSafe(this.flagsAt(flx, y + 1, flz))) {
            // Every aimed line must also have a walkable route over this floor.
            if (t.aimIndex !== 0)
                this.extTryMask &= t.lineAvoid[0];
            return;
        }
        const tx = x + t.tx * sx;
        const tz = z + t.tz * sz;
        let nodeY;
        let cost = t.cost;
        // Support landings use the top-catch class; catches use cell-width credit.
        let landCatch = -1;
        // An overhang above a drop landing must clear the descending arc.
        let landLid1 = false;
        // Reach uses collision-top rise rather than node height: a fence top
        // to a head can be flat even when the destination node is a cell higher.
        let supY = NaN;
        const idxT = t.aimIndex !== 0 ? -1 : this.cellIndex(tx, y, tz);
        const fT = idxT < 0 ? 0 : this.flags[idxT];
        if (this.extUpCatch) {
            // A catch one block up needs air at flight level and headroom above
            // the climbable. A block or ladder at flight level is its own landing.
            if ((fT & (PHYSICAL | CLIMBABLE | LIQUID)) !== 0 || !this.isSafe(fT) ||
                (this.specialAt(tx, y, tz) & BUBBLE_MASK) !== 0 || !this.isSafe(this.flagsAt(tx, y + 2, tz)))
                return;
            // Entering across the ladder's 3/16 strip hits its end; catches
            // approach through the open face or along it.
            const pc = this.carryAt(tx, y + 1, tz);
            if (pc !== 0 && ((t.tx !== 0 && carrySide(pc, sx, 0) === 1) || (t.tz !== 0 && carrySide(pc, 0, sz) === 1)))
                return;
            // A missed catch must fall onto another planner landing.
            if (Number.isNaN(this.findExtLanding(tx, y, tz)))
                return;
            nodeY = y + 1;
            cost += 1;
        }
        else if (t.aimIndex !== 0) {
            // Aimed variants reuse the centre line's full-top landing.
            nodeY = this.extLandY;
            supY = this.extLandSupY;
            landLid1 = this.extLandLid1;
            // Aimed dives under a drop-landing overhang are outside the lid model.
            if (landLid1)
                return;
            landCatch = 0;
            if (nodeY > y)
                cost += nodeY - y;
        }
        else if ((fT & CLIMBABLE) !== 0 && this.isSafe(fT) && this.climbUsable(tx, y, tz)) {
            // Check ladders before PHYSICAL: catch their cells rather than land
            // on their thin tops. Up to two contiguous cells below flight level
            // add airtime and extend the catch's reach.
            if (!this.isSafe(this.flagsAt(tx, y + 1, tz)))
                return;
            nodeY = y;
            for (let ly = y - 1; ly >= y - 2; ly--) {
                const fL = this.flagsAt(tx, ly, tz);
                if ((fL & CLIMBABLE) === 0 || !this.isSafe(fL) || !this.climbUsable(tx, ly, tz))
                    break;
                nodeY = ly;
                cost += 1;
            }
        }
        else if (this.isThinFloor(fT)) {
            // Thin floors land the feet inside the cell, so this is a same-level jump.
            if (!this.isSafe(this.flagsAt(tx, y + 1, tz)))
                return;
            nodeY = y;
        }
        else if ((fT & PHYSICAL) !== 0) {
            if (this.heightAt(tx, y, tz) - h0 > 1.2)
                return; 
            if (!this.isSafe(this.flagsAt(tx, y + 1, tz)) || !this.isSafe(this.flagsAt(tx, y + 2, tz)))
                return;
            nodeY = y + 1;
            cost += 1;
            landCatch = this.heights[idxT] >> 6;
            supY = y;
        }
        else if (idxT >= 0 && this.isTallStand(fT, this.heights[idxT])) {
            // Fence and wall tops raise feet and head clearance by 0.5 blocks.
            // A low block on the post can raise the node without changing the
            // support height used for reach.
            if (this.heightAt(tx, y, tz) - h0 > 1.2)
                return;
            const f1 = this.flagsAt(tx, y + 1, tz);
            if (this.isSafe(f1)) {
                if (!this.isSafe(this.flagsAt(tx, y + 2, tz)) || !this.isSafe(this.flagsAt(tx, y + 3, tz)))
                    return;
                nodeY = y + 1;
                cost += 1;
            }
            else {
                const idx1 = this.cellIndex(tx, y + 1, tz);
                if (idx1 < 0 || (f1 & PHYSICAL) === 0 || (this.heights[idx1] & 63) > 16)
                    return;
                if (!this.isSafe(this.flagsAt(tx, y + 2, tz)) || !this.isSafe(this.flagsAt(tx, y + 3, tz)))
                    return;
                nodeY = y + 2;
                cost += 2;
            }
            landCatch = this.heights[idxT] >> 6;
            supY = y;
        }
        else {
            if (!this.isSafe(fT))
                return;
            landLid1 = !this.isSafe(this.flagsAt(tx, y + 1, tz));
            nodeY = this.findExtLanding(tx, y, tz);
            if (Number.isNaN(nodeY))
                return;
            // An overhang at landing height leaves no room to stand.
            if (landLid1 && nodeY >= y)
                return;
            const fN = this.flagsAt(tx, nodeY, tz);
            if ((fN & (LIQUID | CLIMBABLE)) === 0 && !this.isThinFloor(fN) &&
                (this.specialAt(tx, nodeY, tz) & BUBBLE_MASK) === 0) {
                landCatch = this.catchAt(tx, nodeY - 1, tz);
                supY = nodeY - 1;
            }
        }
        if (landCatch === 0 && t.aimIndex === 0) {
            this.extLandY = nodeY;
            this.extLandSupY = supY;
            this.extLandLid1 = landLid1;
        }
        // Body cells must be passable and walkable floor rules out a gap.
        // A head+1 obstruction flattens the arc rather than requiring clearance;
        // a lid inside the support strip is cleared by a latched takeoff.
        const latchedOff = this.lidHere && chainVia < 0 && this.leavesLid(x, y, z, t.tx > 0 ? sx : 0, t.tz > 0 ? sz : 0, h0);
        const bonk = lowTakeoff;
        let low = false;
        for (let k = 0; k * 2 < cells.length; k++) {
            const cx = x + cells[k * 2] * sx;
            const cz = z + cells[k * 2 + 1] * sz;
            for (let ly = y; ly <= y + 1; ly++) {
                if (this.isSafe(this.flagsAt(cx, ly, cz)) || this.thinClear(cx, ly, cz, t, k, sx, sz))
                    continue;
                // Any aimed line may avoid a thin shape; only lines missing the
                // entire cell can avoid a full cube. Shifted landings need a full top.
                if (landCatch === 0)
                    this.extTryMask = this.thinShapeAt(cx, ly, cz) !== 0 ? -1 : t.avoid[k];
                return;
            }
            if (k < t.nLine && (this.flagsAt(cx, y - 1, cz) & PHYSICAL) !== 0 &&
                this.heightAt(cx, y - 1, cz) >= h0 && !this.thinStands(cx, y, cz)) {
                if (t.aimIndex !== 0)
                    this.extTryMask &= t.lineAvoid[k];
                return; // Walkable floor rules out a gap.
            }
        }
        let nLid = 0;
        // Compare required flight with the landing bucket's reach at the
        // available takeoff speed. Bonked arcs use separate reach rows.
        // Interpolate fractional collision-top rises between neighboring rows.
        // Rises above +1 use the +1 row within the 1.2-block height limit.
        let dy;
        let frac = 0;
        let rise = 0;
        if (Number.isNaN(supY)) {
            dy = nodeY - y;
        }
        else {
            // Land on a bottom stair's full-footprint slab before walking up
            // its step; reach uses that half-block-lower landing height.
            const landTop = (this.specialAt(tx, supY, tz) & SPECIAL_STAIR) !== 0
                ? supY + 0.5
                : this.heightAt(tx, supY, tz);
            rise = landTop - h0;
            dy = Math.floor(rise);
            frac = rise - dy;
            if (dy >= 1) {
                dy = 1;
                frac = 0;
            }
        }
        const bucket = reachBucket(dy);
        // Run-up includes the support's width and walkable cells behind it.
        // Narrow supports shrink the per-axis takeoff and landing credits;
        // full-block ends reuse the table's precomputed flight requirement.
        const takeoffCatch = this.catchAt(x, y - 1, z);
        // A ladder's 3/16 top strip permits only a standing takeoff here: no
        // run-up, momentum lip or rise is supported by this envelope.
        const ladderTop = (this.flagsAt(x, y - 1, z) & CLIMBABLE) !== 0;
        if (ladderTop && nodeY > y)
            return;
        if (ladderTop)
            cost += LADDER_TOP_COST;
        // The tight pass restores the catch credit for a precise catch along the strip.
        let ladderCatchFix = false;
        const lCred = this.extUpCatch ? UP_CATCH_CREDIT : landCatch < 0 ? LAND_HALF : LAND_NARROW_MARGIN + CATCH_HALF[landCatch];
        const half = CATCH_HALF[takeoffCatch];
        // Edge strips and pane lines give direction-dependent carry credit.
        // Toward a strip's far side the body is already past centre; along a
        // line it retains full credit. Centred supports use their class credit.
        const major = t.tx > t.tz ? t.tx : t.tz;
        const majX = t.tx > t.tz;
        const tCarry = this.carryAt(x, y - 1, z);
        // Each offset endpoint needs a full support top; an unshifted endpoint
        // may still use a narrow support.
        if (t.pOff !== 0 && (takeoffCatch !== 0 || tCarry !== 0))
            return;
        if (t.qOff !== 0 && landCatch !== 0)
            return;
        const fwdX = majX ? sx : 0;
        const fwdZ = majX ? 0 : sz;
        const cFront = carrySide(tCarry, fwdX, fwdZ);
        const cBack = carrySide(tCarry, -fwdX, -fwdZ);
        // When the minor axis crosses an edge panel's empty side, the flight
        // crosses its 3/16 width and gets neither creep nor run-up credit.
        // A major-axis exit uses the ordinary ledge model; pure diagonals cross.
        const offPanel = tCarry !== 0 && t.tx !== 0 && t.tz !== 0 &&
            (carrySide(tCarry, majX ? 0 : sx, majX ? sz : 0) === 1 || (t.tx === t.tz && cFront === 1));
        const front = offPanel || cFront === 1 ? 0 : cFront === 2 ? TAKEOFF_STAND : Math.min(TAKEOFF_STAND, TAKEOFF_NARROW_MARGIN + half);
        const back = offPanel || cBack === 1 ? 0 : cBack === 2 ? 0.5 + TAKEOFF_NARROW_MARGIN : half + TAKEOFF_NARROW_MARGIN;
        /** Half-extent the lip take-off (allowParkourMomentum) leaves from, toward the target. */
        const lipHalf = offPanel || cFront === 1 ? -LAND_NARROW_MARGIN : cFront === 2 ? 0.5 : half;
        let run = ladderTop ? front : front + back;
        let lCredX = lCred;
        let lCredZ = lCred;
        if (landCatch > 0 && !Number.isNaN(supY)) {
            const lCarry = this.carryAt(tx, supY, tz);
            if (lCarry !== 0) {
                if (t.tx !== 0) {
                    const c = carrySide(lCarry, -sx, 0);
                    lCredX = c === 2 ? LAND_HALF : c === 1 ? 0 : lCred;
                }
                if (t.tz !== 0) {
                    const c = carrySide(lCarry, 0, -sz);
                    lCredZ = c === 2 ? LAND_HALF : c === 1 ? 0 : lCred;
                }
            }
        }
        const rx = x + t.runX * sx;
        const rz = z + t.runZ * sz;
        if (!offPanel && !ladderTop && this.isSafe(this.flagsAt(rx, y, rz)) && this.isSafe(this.flagsAt(rx, y + 1, rz))) {
            if ((this.flagsAt(rx, y - 1, rz) & PHYSICAL) !== 0 && this.catchAt(rx, y - 1, rz) === 0) {
                const hR = this.heightAt(rx, y - 1, rz);
                if (hR - h0 <= 0.2 && h0 - hR <= 0.6)
                    run += 1;
            }
            else if (this.isSafe(this.flagsAt(rx, y - 1, rz)) &&
                (this.flagsAt(rx, y - 2, rz) & PHYSICAL) !== 0 && this.catchAt(rx, y - 2, rz) === 0) {
                // Sprinting up a step preserves the run; a full-block rise requires a jump.
                const hR = this.heightAt(rx, y - 2, rz);
                if (h0 - hR <= 0.6)
                    run += 1;
            }
        }
        let row = runRow(run);
        if (bucket < 0)
            return; 
        let fn = takeoffCatch === 0 && landCatch <= 0 && !this.extUpCatch
            ? t.fnStand
            : flightFrom2(t.tx, t.tz, t.dist, front * t.dist / major, lCredX, lCredZ);
        // Leaving a panel's empty side starts against the wall without the
        // standing row's sprint impulse, so the jump needs extra clearance.
        if (offPanel)
            fn += OFF_PANEL_MARGIN;
        // A shifted line adds distance beyond the centre line's base cost.
        fn += t.extra;
        if (ladderTop) {
            if (offPanel) {
                // A diagonal ladder-top takeoff uses per-axis credit: along
                // the strip on one axis and pressed to the wall on the other.
                const cX = t.tx === 0 ? 0 : carrySide(tCarry, sx, 0) === 2 ? TAKEOFF_STAND : 0;
                const cZ = t.tz === 0 ? 0 : carrySide(tCarry, 0, sz) === 2 ? TAKEOFF_STAND : 0;
                const ex = t.tx - cX - lCredX;
                const ez = t.tz - cZ - lCredZ;
                fn = Math.sqrt((ex > 0 ? ex * ex : 0) + (ez > 0 ? ez * ez : 0)) + OFF_PANEL_MARGIN + t.extra;
            }
            else if (landCatch < 0 && (t.tx === 0 || t.tz === 0)) {
                fn += LADDER_CATCH_OVERCREDIT;
                ladderCatchFix = true;
            }
        }
        // If the default arc hits a lid side-on, try a slower takeoff that
        // passes underneath and still reaches the landing.
        let lidCap = Infinity;
        const s0 = front * t.dist / major;
        const landRise = Number.isNaN(supY) ? nodeY - y : rise;
        const landIn = t.headIn[(t.headCells.length >> 1) - 1];
        let nGr = 0;
        let nGrWalk = 0;
        {
            // Each lid caps the arc at its own bonk height: 0.2 for a full
            // block, 0.7 for a top slab, 1.0125 for a top trapdoor.
            // A thin lid left behind before takeoff does not cap the arc.
            let capT = Infinity;
            // A latched takeoff leaves the support strip's lid before jumping.
            if (!latchedOff && !this.isSafe(this.flagsAt(x, y + 2, z))) {
                const b = this.thinAt(x, y + 2, z);
                if (b !== 0 && !isLidByte(b)) {
                    if (takeoffCatch !== 0 || tCarry !== 0 || (this.thinSpan(x, y + 2, z, 0, 0, t, sx, sz) && this.spanHi >= (row > 0 ? Math.min(s0, TAKEOFF_RUN * t.dist / major) : s0)))
                        capT = y + 2 - 1.8 - h0;
                }
                else
                    capT = y + 2 + (b !== 0 ? (b & 15) / 16 : 0) - 1.8 - h0;
            }
            let capC = Infinity;
            for (let k = 0; k * 2 < cells.length; k++) {
                const cx = x + cells[k * 2] * sx;
                const cz = z + cells[k * 2 + 1] * sz;
                if (this.isSafe(this.flagsAt(cx, y + 2, cz)) || this.thinClear(cx, y + 2, cz, t, k, sx, sz))
                    continue;
                const b = this.thinAt(cx, y + 2, cz);
                const hb = y + 2 + (isLidByte(b) ? (b & 15) / 16 : 0) - 1.8 - h0;
                if (isLidByte(b)) {
                    // A lid outside part of the takeoff window can be hit side-on;
                    // walking the arc determines whether it clears.
                    const ax = cells[k * 2];
                    const az = cells[k * 2 + 1];
                    const hc = t.headCells;
                    let lo = -1e9;
                    let hi = 1e9;
                    for (let h = 0; h * 2 < hc.length; h++) {
                        if (hc[h * 2] === ax && hc[h * 2 + 1] === az) { lo = t.headIn[h]; hi = t.headOut[h]; break; }
                    }
                    if (lo > s0 - 0.14 && nGr < 16) {
                        this.grHb[nGr] = hb; this.grLo[nGr] = lo; this.grHi[nGr] = hi; nGr++;
                        continue;
                    }
                }
                if (hb < capC)
                    capC = hb;
            }
            let cap = capT < capC ? capT : capC;
            // Apply the corridor cap along the whole path, but the takeoff lid
            // only over its head span; the arc can rise past it after leaving.
            const seedLids = (): number => {
                let n = 0;
                if (capC < FREE_APEX) { this.lidHb[n] = capC; this.lidLo[n] = -1e9; this.lidHi[n] = 1e9; n++; }
                if (capT < FREE_APEX) { this.lidHb[n] = capT; this.lidLo[n] = t.headIn[0]; this.lidHi[n] = t.headOut[0]; n++; }
                return n;
            };
            let grBelow = false;
            for (let j = 0; j < nGr; j++) if (this.grHb[j] < capC && this.grHb[j] < FREE_APEX) grBelow = true;
            if (grBelow) {
                // A side collision falls back to the conservative 0.2-block cap.
                let n = seedLids();
                for (let j = 0; j < nGr; j++) if (this.grHb[j] < capC && this.grHb[j] < FREE_APEX) { this.lidHb[n] = this.grHb[j]; this.lidLo[n] = this.grLo[j]; this.lidHi[n] = this.grHi[j]; n++; }
                const c = this.lidWalkWindow(n, row, s0, Number.isNaN(supY) ? nodeY - y : rise, t.headIn[(t.headCells.length >> 1) - 1]);
                if (Number.isNaN(c)) {
                    const old = y + 2 - 1.8 - h0;
                    if (old < cap) cap = old;
                }
                else if (!landLid1) {
                    nLid = n;
                }
                else {
                    nGrWalk = n;
                }
            }
            lidCap = cap >= FREE_APEX ? Infinity : cap;
            low = lidCap !== Infinity;
            if (landLid1) {
                // A drop-landing overhang needs an arc check against its face;
                // a slower takeoff can pass beneath it.
                nLid = nGrWalk;
                if (nLid === 0)
                    nLid = seedLids();
                const nh = t.headCells.length >> 1;
                const b1 = this.thinAt(tx, y + 1, tz);
                let lo = t.headIn[nh - 1];
                let hi = t.headOut[nh - 1];
                let ok = true;
                if (b1 !== 0 && !isLidByte(b1)) {
                    if (this.thinSpan(tx, y + 1, tz, t.tx, t.tz, t, sx, sz)) { lo = this.spanLo; hi = this.spanHi; }
                    else ok = false;
                }
                if (ok) {
                    this.lidHb[nLid] = y + 1 + (isLidByte(b1) ? (b1 & 15) / 16 : 0) - 1.8 - h0;
                    this.lidLo[nLid] = lo;
                    this.lidHi[nLid] = hi;
                    nLid++;
                }
                else if (nGrWalk === 0) nLid = 0;
            }
        }
        // Arc walking cannot raise reach beyond the existing lid caps.
        const rowCap = lidCap;
        if (nLid > 0) {
            let found = false;
            for (let r = row; r >= 0; r--) {
                const cw = this.lidWalkWindow(nLid, r, s0, landRise, landIn);
                if (Number.isNaN(cw))
                    continue;
                const c = cw < rowCap ? cw : rowCap;
                if (r < row) {
                    const rr = lidRows(c)[r];
                    let u = rr[bucket];
                    if (frac > 0)
                        u = u + (rr[bucket - 1] - u) * frac;
                    if (fn > u + this.tightCredit)
                        continue;
                }
                row = r;
                lidCap = c;
                found = true;
                break;
            }
            if (!found) {
                if (landCatch === 0 && t.aimIndex === 0)
                    this.extTryMask = -1;
                return;
            }
            low = lidCap !== Infinity;
        }
        const rows = lidRows(lidCap)[row];
        // Interpolate from the lower landing's longer-reach row toward the higher landing.
        let usable = rows[bucket];
        if (frac > 0)
            usable = usable + (rows[bucket - 1] - usable) * frac;
        // Standing reach sets executor metadata without changing which moves are emitted.
        const standRows = lidRows(lidCap)[0];
        let usableStand = standRows[bucket];
        if (frac > 0)
            usableStand = usableStand + (standRows[bucket - 1] - usableStand) * frac;
        // When normal reach falls short, retry with no safety margin at all.
        // TIGHT_COST keeps precise jumps less attractive than comfortable ones.
        let mc = this.marginCredit;
        let tight = false;
        let feasible = false;
        let needsRunning = false;
        let runNeeded = false;
        let chained = false;
        let lipJump = false;
        for (;;) {
            const usableM = usable + mc;
            let ok = true;
            feasible = fn <= usableM;
            // Any run-up uses the running corridor arc.
            needsRunning = row >= 1;
            runNeeded = fn > usableStand + mc;
            chained = false;
            lipJump = false;
            if (chainVia < 0) {
                if (!feasible && this.parkourMomentum && !offPanel && !ladderTop && !this.extUpCatch) {
                    // A running lip takeoff gains overhang and tick-phase credit.
                    // Try it only beyond creep reach, using its later, lower arc.
                    const sLip = (lipHalf + LAND_NARROW_MARGIN + LIP_PHASE) * t.dist / major;
                    let usableLip = usableM;
                    // The later lip arc needs its own lid check.
                    if (nLid > 0) {
                        const cw = this.lidWalkWindow(nLid, row, sLip, landRise, landIn);
                        if (Number.isNaN(cw))
                            usableLip = -Infinity;
                        else {
                            const rr = lidRows(cw < rowCap ? cw : rowCap)[row];
                            usableLip = rr[bucket] + (frac > 0 ? (rr[bucket - 1] - rr[bucket]) * frac : 0) + mc;
                        }
                    }
                    if (flightFrom2(t.tx, t.tz, t.dist, sLip, lCredX, lCredZ) <= usableLip) {
                        feasible = true;
                        lipJump = true;
                    }
                }
                if (!feasible) {
                    // A direction-aligned landing-tick re-jump retains momentum
                    // from the far-side landing point, reduced by turn loss.
                    // Lids are excluded because there is no bonked chain row.
                    if (this.momIn === MOM_NONE || low || bonk || nLid > 0 || t.aimIndex !== 0 || offPanel || this.extUpCatch ||
                        !this.chainAligned(this.momDx, this.momDz, this.momD2, t.tx * sx, t.tz * sz)) {
                        ok = false;
                    }
                    else {
                        const sChain = front * CHAIN_TAKEOFF_FRACTION * t.dist / major;
                        const cosTurn = (this.momDx * (t.tx * sx) + this.momDz * (t.tz * sz)) / Math.sqrt(this.momD2 * (t.tx * t.tx + t.tz * t.tz));
                        if (flightFrom2(t.tx, t.tz, t.dist, sChain, lCredX, lCredZ) > J_CHAIN[bucket] - CHAIN_TURN_LOSS * (1 - cosTurn) + mc) {
                            ok = false;
                        }
                        else {
                            needsRunning = true;
                            chained = true;
                        }
                    }
                }
            }
            else {
                // Emit a chain only beyond the stone's normal reach. The first
                // hop lands on its far side, then re-jumps on the running arc;
                // lids are excluded because there is no bonked chain row.
                if (feasible || low || nLid > 0)
                    return;
                const sChain = front * CHAIN_TAKEOFF_FRACTION * t.dist / major;
                if (flightFrom2(t.tx, t.tz, t.dist, sChain, lCredX, lCredZ) > J_CHAIN[bucket] + mc)
                    ok = false;
                else
                    needsRunning = true;
            }
            if (ok)
                break;
            // A catch one block up uses only the normal creep/run envelope.
            if (tight || mc >= this.tightCredit || this.extUpCatch)
                return;
            tight = true;
            mc = this.tightCredit;
            if (ladderCatchFix)
                fn -= LADDER_CATCH_OVERCREDIT;
        }
        if (tight)
            cost += TIGHT_COST;
        // Check each cell against the lowest feet height on the chosen arc.
        // Blocks below that line only matter if a tall shape protrudes into it,
        // allowing drop jumps over corners near takeoff.
        const mf = lipJump
            ? (low ? t.mfLowLip : t.mfLip)
            : low
                ? (needsRunning ? t.mfLowRun : t.mfLowStand)
                : (needsRunning ? t.mfRun : t.mfStand);
        for (let k = 0; k * 2 < cells.length; k++) {
            const cx = x + cells[k * 2] * sx;
            const cz = z + cells[k * 2 + 1] * sz;
            // Probe one cell below the feet bound, including grazing tolerance:
            // fence tops reach 1.5 blocks above their cell floor.
            const lim = h0 + mf[k] + 0.05;
            const loLy = Math.floor(lim) - 1;
            for (let ly = y - 1; ly >= loLy; ly--) {
                // A refusal belongs to this arc; only the centre line selects
                // alternate variants, since their arcs differ.
                if (ly + 1 > lim) {
                    if (!this.isSafe(this.flagsAt(cx, ly, cz)) && !this.thinClear(cx, ly, cz, t, k, sx, sz)) {
                        if (landCatch === 0 && t.aimIndex === 0)
                            this.extTryMask = this.thinShapeAt(cx, ly, cz) !== 0 ? -1 : t.avoid[k];
                        return; 
                    }
                }
                else if (this.heightAt(cx, ly, cz) > lim && !this.thinClear(cx, ly, cz, t, k, sx, sz)) {
                    if (landCatch === 0 && t.aimIndex === 0)
                        this.extTryMask = this.thinShapeAt(cx, ly, cz) !== 0 ? -1 : t.avoid[k];
                    return; 
                }
            }
        }
        for (let k = 0; k < t.nLine; k++) {
            cost += this.entitiesAt(x + cells[k * 2] * sx, y, z + cells[k * 2 + 1] * sz) * this.entityCost;
        }
        cost += this.entitiesAt(tx, nodeY, tz) * this.entityCost;
        cost += this.exclusionAt(tx, nodeY, tz);
        if (cost > 100)
            return;
        // Only narrow support landings retain momentum; catches lack a re-jump
        // tick, and fall damage can zero horizontal velocity. Full supports
        // already have greater lip reach, so momentum would duplicate states.
        if (this.lavaFlanked(tx, nodeY, tz))
            return;
        if (this.parkourMomentum && landCatch >= 1 && rise >= -MOMENTUM_MAX_DROP)
            this.pendingMom = momentumOf(t.tx * sx, t.tz * sz);
        const runBit = (runNeeded || lipJump || tight) ? META_RUN : 0;
        if (chainVia >= 0) {
            this.pendingVia = chainVia;
            this.push(tx, nodeY, tz, chainBase + cost, META_PARKOUR | META_CHAIN | META_RUN);
        }
        else if (chained) {
            // This node is the stepping stone; the executor lands far-side and re-jumps.
            this.pendingVia = this.cellIndex(x, y, z);
            this.push(tx, nodeY, tz, cost, META_PARKOUR | META_CHAIN | META_RUN);
        }
        else {
            // Mirroring one axis flips the aimed line's perpendicular; two do not.
            if (t.aimIndex !== 0)
                this.pendingAim = t.aimIndex | (sx * sz < 0 ? 0x80 : 0);
            const shiftBit = t.aimIndex !== 0 ? META_AIM : 0;
            // Deep liquid catches have no return path and carry META_DEEP.
            this.push(tx, nodeY, tz, cost, META_PARKOUR | runBit | shiftBit | ((y - nodeY) > this.maxDropDown ? META_DEEP : 0));
        }
    }
    /** Scan the target column for support below flight level. Return the node
     * above a physical top or inside a catching cell, or NaN if none is in range. */
  private findExtLanding (tx: number, y: number, tz: number): number {
        let ly = y - 1;
        while (ly > this.worldMinY) {
            const idx = this.cellIndex(tx, ly, tz);
            if (idx < 0)
                return NaN;
            const f = this.flags[idx];
            if ((f & LIQUID) !== 0 && this.isSafe(f)) {
                if (!this.infiniteLiquidDropdownDistance && y - ly > this.maxDropDown)
                    return NaN;
                return ly;
            }
            if (this.special !== null && (this.special[idx] & BUBBLE_MASK) !== 0)
                return ly;
            if ((f & CLIMBABLE) !== 0 && this.climbUsable(tx, ly, tz)) {
                if (y - ly > this.maxDropDown)
                    return NaN;
                return ly;
            }
            // Thin floor (carpet class): feet land IN the cell, not on top of it.
            if (this.isThinFloor(f)) {
                if (y - ly > this.maxDropDown)
                    return NaN;
                return ly;
            }
            if ((f & PHYSICAL) !== 0) {
                if (y - (ly + 1) > this.maxDropDown)
                    return NaN;
                return ly + 1;
            }
            // Fence/wall top: a stand with the feet 0.5 into the cell above it.
            if (this.isTallStand(f, this.heights[idx])) {
                if (y - (ly + 1) > this.maxDropDown || !this.isSafe(this.flagsAt(tx, ly + 3, tz)))
                    return NaN;
                return ly + 1;
            }
            if (!this.isSafe(f))
                return NaN;
            ly--;
        }
        return NaN;
    }
    /** Vines require an adjacent solid wall for vanilla collision climbing. */
  private climbUsable (x: number, y: number, z: number): boolean {
        if (this.specialAt(x, y, z) !== SPECIAL_VINE)
            return true;
        return (this.flagsAt(x + 1, y, z) & PHYSICAL) !== 0 ||
            (this.flagsAt(x - 1, y, z) & PHYSICAL) !== 0 ||
            (this.flagsAt(x, y, z + 1) & PHYSICAL) !== 0 ||
            (this.flagsAt(x, y, z - 1) & PHYSICAL) !== 0;
    }
}
