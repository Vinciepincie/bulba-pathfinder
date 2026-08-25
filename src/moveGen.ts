// Neighbor generation: an exact port of mineflayer-pathfinder's
// Movements.getNeighbors() with the walk moveset plus (opt-in) the full dig
// cost model — canDig branches mirror upstream's safeToBreak/safeOrBreak
// including dontCreateFlow and dontMineUnderFallingBlock. Block PLACEMENT
// branches remain statically dead (no scaffolding) and are omitted. Costs,
// probe order, thresholds and quirks are mirrored verbatim; the documented
// improvement: real (non-iron) doors/open gates are traversable when
// canOpenDoors && canOpenRealDoors.
//
// All probes are typed-array reads against the snapshot; neighbors are
// written into preallocated arrays (including their world coordinates, so
// the solver never pays an index-decode per neighbor) — zero allocation per
// expansion on the walk-only path; dig moves allocate only their toBreak
// lists, like upstream.
import { LutFlags, LutSpecial, DigFlags } from './types.js';
import type { MovementsConfig, SnapshotMeta, DigData } from './types.js';
import { getParkourExtTable } from './parkourTable.js';
import type { ParkourExtEntry, ParkourExtTable } from './parkourTable.js';
import { J_RUN, J_LOW_RUN, runRow, J_CHAIN, CHAIN_MIN_COS, CHAIN_TAKEOFF_FRACTION, reachBucket, ENVELOPE_SAFETY_MARGIN, TAKEOFF_STAND, LAND_HALF, TAKEOFF_NARROW_MARGIN, LAND_NARROW_MARGIN, BOUNCE_APEX, BOUNCE_MAX_DROP, BOUNCE_MARGIN, BOUNCE_MARGIN_FAR } from './parkourEnvelope.js';
import { CATCH_HALF } from './shapes.js';
/** cos(angle) gate as an exact integer test: dot > 0 and dot² ≥ c²·|u|²·|v|². */
const CHAIN_MIN_COS2 = CHAIN_MIN_COS * CHAIN_MIN_COS;
/**
 * Flight needed for offset (a, b) ≥ 0 from a takeoff point `s` along the
 * flight line with per-axis landing credit `lCred` — parkourEnvelope's
 * flightNeeded/takeoff pair for arbitrary credits (the narrow-support case;
 * the wasm core mirrors this arithmetic op for op).
 */
function flightFrom (a: number, b: number, dist: number, s: number, lCred: number): number {
    const dx = a - a / dist * s - lCred;
    const dz = b - b / dist * s - lCred;
    const px = dx > 0 ? dx : 0;
    const pz = dz > 0 ? dz : 0;
    // sqrt of a sum, not hypot: correctly rounded everywhere, so JS and the
    // wasm core cannot disagree on the last bit.
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
  /** Raw state ids — required for canDig (dig tables key on state). */
  states?: Uint16Array | null
  /** LutSpecial byte per cell — required for useBubbleColumns. */
  special?: Uint8Array | null
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
const SAFE = LutFlags.SAFE;
const PHYSICAL = LutFlags.PHYSICAL;
const LIQUID = LutFlags.LIQUID;
const CLIMBABLE = LutFlags.CLIMBABLE;
const GATE = LutFlags.OPENABLE_GATE;
const DOOR_CLOSED = LutFlags.DOOR_CLOSED;
const DOOR_OPEN = LutFlags.DOOR_OPEN;
const GATE_OPEN = LutFlags.GATE_OPEN;
const PASSABLE_WHEN_OPEN = DOOR_OPEN | GATE_OPEN;
const CAN_FALL = DigFlags.CAN_FALL;
const CANT_BREAK = DigFlags.CANT_BREAK;
const BUBBLE_UP = LutSpecial.BUBBLE_UP;
const BUBBLE_DOWN = LutSpecial.BUBBLE_DOWN;
const SPECIAL_VINE = LutSpecial.VINE;
const SPECIAL_SLIME = LutSpecial.SLIME;
/** Bubble-only semantics (float support, fall catching, no-jump) must not
 * fire on VINE/SLIME-marked cells. */
const BUBBLE_MASK = BUBBLE_UP | BUBBLE_DOWN;
// Cardinal + diagonal probe tables, upstream order (W, E, N, S / NW, SW, NE, SE).
const CARDINAL_X = [-1, 1, 0, 0];
const CARDINAL_Z = [0, 0, -1, 1];
const DIAGONAL_X = [-1, -1, 1, 1];
const DIAGONAL_Z = [-1, 1, -1, 1];
// Climb-transfer directions: cardinals then diagonals (same order as above).
const TRANSFER_X = [-1, 1, 0, 0, -1, -1, 1, 1];
const TRANSFER_Z = [0, 0, -1, 1, -1, 1, -1, 1];
/** Transfer cost per direction: octile + 0.5 pad (|dy| is added per move). */
const TRANSFER_COST = [1.5, 1.5, 1.5, 1.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5, Math.SQRT2 + 0.5];
// Slime-bounce landing columns around the slime cell: the 8 neighbours,
// then the 4 cells two out along a cardinal (a rebound drifts ~1 block by
// the apex and a little more on the way down — parkourEnvelope.ts).
const BOUNCE_X = [-1, 1, 0, 0, -1, -1, 1, 1, -2, 2, 0, 0];
const BOUNCE_Z = [0, 0, -1, 1, -1, 1, -1, 1, 0, 0, -2, 2];
const BOUNCE_OFFSETS = 12;
const BOUNCE_RING1 = 8;
/**
 * Upper bound on neighbours from one expansion, derived rather than guessed.
 *
 * Base moveset: 4 cardinals x {forward, jumpUp, dropDown} + 4 diagonals +
 * moveDown + moveUp + the two bubble rides = 20. Extended parkour adds the
 * whole table: every diagonal entry in 4 quadrants, every pure-x entry in 2
 * directions, every pure-z entry in 2.
 *
 * The old fixed 160 was 8 short of today's table (32*4 + 5*2 + 5*2 + 20 =
 * 168). Overflowing a typed array does not throw — the writes are silently
 * dropped while `outCount` keeps counting — so the solver read undefined
 * coordinates for the last neighbours of a fully-loaded node and relaxed
 * them as NaN. Deriving the size means enlarging the table can never
 * reintroduce that.
 */
function outCapacity (extended: boolean): number {
    // Without the table, upstream's cardinal parkour runs instead, and its
    // "down" branch can push a landing at each of d = 2, 3 and 4 in the same
    // direction — 12 more than the 20 above.
    const base = extended ? 20 : 32;
    if (!extended)
        return base;
    const t = getParkourExtTable();
    // Extended also adds climb transfers (8 directions × 3 dy), slime bounces
    // (BOUNCE_OFFSETS per drop landing, 5 drop generators) and momentum
    // chains: per parkour landing, every table entry within the angle gate —
    // bounded by the full table again per landing, and the landings by the
    // table; the chain set is far sparser in practice (stepping stones only),
    // so a generous fixed allowance is kept rather than the product.
    const table = t.diag.length * 4 + t.cardX.length * 2 + t.cardZ.length * 2;
    return table + base + 24 + BOUNCE_OFFSETS * 5 + CHAIN_CAP;
}
/** Upper bound on momentum-chain edges per expansion (see outCapacity). */
const CHAIN_CAP = 256;
export class MoveGen {
    /** Output arrays for one expansion — see outCapacity(). */
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
  outCount = 0
    /** Set by slimeBounce for the push it is about to make. */
  private pendingVia = -1
    /** Set when any probe left the snapshot — a noPath may be growth-fixable. */
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
    /** allowParkourExtended && allowParkour && allowSprinting (all jumps need sprint). */
  private readonly parkourExtended!: boolean
    /** Generated offset/corridor table (null when the feature is off). */
  private readonly extTable!: ParkourExtTable | null
  private readonly canOpenDoors!: boolean
  private readonly doorMode!: boolean
  private readonly maxDropDown!: number
  private readonly infiniteLiquidDropdownDistance!: boolean
  private readonly liquidCost!: number
  private readonly entityCost!: number
    // ── bubble columns (null when useBubbleColumns is off — zero overhead) ──
  private readonly special!: Uint8Array | null
  private readonly bubbleCost!: number
    /** Added to every usable-flight row: ENVELOPE_SAFETY_MARGIN − the profile's
     * parkourSafetyMargin (0 at the default; positive plans tighter jumps). */
  private readonly marginCredit!: number
    // ── digging (null when canDig is off — the hot path never touches it) ──
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
        this.canOpenDoors = cfg.canOpenDoors;
        this.doorMode = cfg.canOpenDoors && cfg.canOpenRealDoors;
        this.maxDropDown = cfg.maxDropDown;
        this.infiniteLiquidDropdownDistance = cfg.infiniteLiquidDropdownDistance;
        this.liquidCost = cfg.liquidCost;
        this.entityCost = cfg.entityCost;
        // Presence of the grid is the signal — it exists only when a feature
        // (bubble columns, climbable vines) marked cells into it.
        this.special = snap.special ?? null;
        this.bubbleCost = cfg.bubbleCost;
        this.marginCredit = ENVELOPE_SAFETY_MARGIN - (cfg.parkourSafetyMargin ?? ENVELOPE_SAFETY_MARGIN);
        this.dig = cfg.canDig ? dig : null;
        this.digCost = cfg.digCost;
        this.dontCreateFlow = cfg.dontCreateFlow;
        this.dontMineUnderFallingBlock = cfg.dontMineUnderFallingBlock;
    }
    /** Cell index or -1 when out of the snapshot (upstream null-block). */
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
    /** Absolute collision-top height of the cell (upstream b.height). The
     * height byte is packed — bits 6–7 carry the top-catch class (types.ts). */
  heightAt (x: number, y: number, z: number): number {
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? y : y + (this.heights[idx] & 63) / 32;
    }
    /** topCatchClass of the cell's block (0 = full/wide … 3 = unstandable). */
  private catchAt (x: number, y: number, z: number): number {
        const idx = this.cellIndex(x, y, z);
        return idx < 0 ? 0 : this.heights[idx] >> 6;
    }
    /** Upstream block.safe, plus the open-door/open-gate improvement when enabled. */
  private isSafe (f: number): boolean {
        return (f & SAFE) !== 0 || (this.doorMode && (f & PASSABLE_WHEN_OPEN) !== 0);
    }
    /**
     * Tall stand (extended parkour): the fence/wall/closed-gate class —
     * excluded from PHYSICAL by upstream because a 1.5 top can't be stepped or
     * jumped onto from its own level, yet a real support: feet rest ON its
     * top, 0.5 into the cell above. `hByte` is the packed height byte.
     */
  private isTallStand (f: number, hByte: number): boolean {
        return (f & (SAFE | PHYSICAL | LIQUID)) === 0 && (hByte & 63) > 32;
    }
  private tallStandAt (x: number, y: number, z: number): boolean {
        const idx = this.cellIndex(x, y, z);
        return idx >= 0 && this.isTallStand(this.flags[idx], this.heights[idx]);
    }
    /**
     * Thin walk-in floor (carpet class): SAFE + PHYSICAL, not a climbable.
     * Feet stand IN this cell on its sub-block shape — never ON TOP of it as
     * a full block. Every "PHYSICAL ⇒ land at cell+1" site must branch here,
     * or carpeted floors produce landings one block up in the air.
     */
  private isThinFloor (f: number): boolean {
        return (f & (SAFE | PHYSICAL | CLIMBABLE)) === (SAFE | PHYSICAL);
    }
    /** LutSpecial byte of the cell (0 when the feature is off / out of box). */
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
    /** Reset the per-move toBreak scratch — every generator calls this first. */
  private beginMove (): void {
        if (this.moveBreaks.length > 0)
            this.moveBreaks = [];
    }
    /**
     * Upstream safeToBreak() against the snapshot (only reached when canDig).
     */
  private safeToBreak (x: number, y: number, z: number, idx: number): boolean {
        const dig = this.dig!;
        if (this.dontCreateFlow) {
            // false if next to liquid (above + 4 lateral), upstream order.
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
    /**
     * Upstream safeOrBreak(): exclusion + entity weight when the block is
     * safe; with canDig, unsafe-but-breakable blocks cost their dig labor and
     * are recorded in the per-move toBreak scratch; otherwise a flat 100.
     */
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
  private push (x: number, y: number, z: number, cost: number, meta: number): void {
        const idx = this.cellIndex(x, y, z);
        if (idx < 0)
            return; // target outside snapshot — boundaryTouched already set
        const i = this.outCount++;
        this.outIdx[i] = idx;
        this.outX[i] = x;
        this.outY[i] = y;
        this.outZ[i] = z;
        this.outCost[i] = cost;
        this.outMeta[i] = meta;
        this.outVia[i] = this.pendingVia;
        this.pendingVia = -1;
        if (this.moveBreaks.length > 0) {
            this.outBreaks[i] = this.moveBreaks;
            this.moveBreaks = [];
        }
        else {
            this.outBreaks[i] = null;
        }
    }
    /** Fills the out* arrays for the node at (x, y, z), upstream order. */
  generate (x: number, y: number, z: number): void {
        this.outCount = 0;
        // Extended-parkour takeoff gates are node-invariant — hoisted so a
        // non-jumpable node (in water, no headroom) pays 3 probes, not 40.
        let ext = false;
        let jumps = false;
        let h0 = 0;
        let lowTakeoff = false;
        if (this.parkourExtended) {
            // No y+2 requirement: a blocked head+1 at the takeoff just puts the
            // jump into the head-hitter class (bonked arc), like a real player
            // jumping inside a 2-high tunnel. `ext` supersedes upstream's
            // cardinal parkour at this node; `jumps` runs the table. They differ
            // on a climbable cell: a body hanging on a ladder has no footing to
            // jam from (the envelope was measured from solid ground), so it
            // climbs, transfers (climbTransfers) or steps off instead — and
            // upstream's flat-cost parkour must not fill that gap.
            const f0 = this.flagsAt(x, y, z);
            ext = (f0 & LIQUID) === 0 && (this.specialAt(x, y, z) & BUBBLE_MASK) === 0;
            jumps = ext && (f0 & CLIMBABLE) === 0;
            if (jumps) {
                h0 = this.heightAt(x, y - 1, z);
                lowTakeoff = !this.isSafe(this.flagsAt(x, y + 2, z));
            }
        }
        for (let i = 0; i < 4; i++) {
            const dx = CARDINAL_X[i];
            const dz = CARDINAL_Z[i];
            this.moveForward(x, y, z, dx, dz);
            this.moveJumpUp(x, y, z, dx, dz);
            this.moveDropDown(x, y, z, dx, dz);
            // Upstream's cardinal parkour is SUPERSEDED by the extended table, not
            // complemented by it. It charges a flat 1 for a jump covering up to four
            // blocks — less than a single walking step, which makes the octile
            // heuristic inadmissible and lets A* buy distance with jumps (2b2t
            // spawn: a plan costing 74.1 against upstream's 76.6 that was 3.5 blocks
            // LONGER to walk) — and it applies neither the reach envelope nor the
            // swept-corridor clearance, so it re-offers, at a cheaper price, exactly
            // the jumps parkourExtTarget just vetoed. Nor is coverage a reason to
            // keep it: over 120 seeded worlds and 108k parkour-bearing nodes it
            // reaches 22.5k targets the table does not, and a prismarine-physics
            // rollout — from the cell centre, the take-off corner, and one and two
            // blocks of run-up — can fly 1.1% of them. The rest are jumps the bot
            // cannot make, each one a planned stall. docs/ExtendedParkour.md, and
            // test/movegen.test.ts pins the supersession.
            if (this.allowParkour && !ext)
                this.moveParkourForward(x, y, z, dx, dz);
            if (jumps) {
                const tab = this.extTable!;
                if (dx !== 0) {
                    for (const t of tab.cardX)
                        this.parkourExtTarget(x, y, z, dx, 1, h0, lowTakeoff, t);
                }
                else {
                    for (const t of tab.cardZ)
                        this.parkourExtTarget(x, y, z, 1, dz, h0, lowTakeoff, t);
                }
            }
        }
        for (let i = 0; i < 4; i++) {
            this.moveDiagonal(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i]);
            if (jumps) {
                for (const t of this.extTable!.diag) {
                    this.parkourExtTarget(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i], h0, lowTakeoff, t);
                }
            }
        }
        if (jumps)
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
    /**
     * Improvement (allowParkourExtended): step from one climbable cell to an
     * adjacent one — sideways along a wall, or diagonally round a pillar
     * corner, one up, level or one down. This is how a spiral of ladders on a
     * 1x1 pillar is climbed: press into the corner at the top of one ladder,
     * drift round onto the next. A diagonal needs one open corner column to
     * sweep through (the other is usually the pillar itself). Cost is octile
     * + |dy| + 0.5, so the heuristic stays admissible. Not a parkour node —
     * the executor's physics rollouts drive it like any climb.
     */
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
    /**
     * Improvement (allowParkourExtended): slime bounce. A drop of `d ≥ 2` onto
     * the slime stand cell (sx, sy, sz) — just generated by the caller — is
     * ALSO an edge to every landing the rebound can reach: a support in one of
     * the 12 columns around the slime whose top sits above the slime and at or
     * under the bounce apex (BOUNCE_APEX, prismarine-physics-derived), with
     * the column and the head cells passable. Ring-1 targets get the apex
     * minus BOUNCE_MARGIN; ring-2 ones give up a block for the drift. The edge
     * is A → C with the slime as `via`, so the search never has to know how
     * much fall energy a node arrived with. Cost = octile + |dy| + 1 from the
     * takeoff, which keeps the heuristic admissible.
     */
  private slimeBounce (x: number, y: number, z: number, sx: number, sy: number, sz: number): void {
        const supIdx = this.cellIndex(sx, sy - 1, sz);
        if (supIdx < 0 || (this.special as Uint8Array)[supIdx] !== SPECIAL_SLIME)
            return;
        // A dry landing on the slime top — water over it damps the rebound.
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
            // Highest cell whose top could still be under the apex.
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
    /**
     * Improvement (useBubbleColumns): ride one block up an up-column. The
     * target cell (y+1) is the current head cell — already passable — so like
     * moveUp only the NEW head cell (y+2) is charged.
     */
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
    /** Improvement (useBubbleColumns): sink one block down a down-column. */
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
        let cost = 1; // move cost
        cost += this.exclusionAt(x + dx, y, z + dz);
        // Upstream's !blockD.physical && !blockC.liquid branch is pure block
        // placement — dead with zero scaffolding, so such a move is impossible.
        // Improvement (useBubbleColumns): a bubble cell floats you like water.
        if ((fD & PHYSICAL) === 0 && (fC & LIQUID) === 0 &&
            (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) === 0) {
            if (!this.parkourExtended)
                return;
            if (this.tallStandAt(x + dx, y - 1, z + dz)) {
                // Walk onto a fence/wall top sunk one below (top 0.5 above this
                // floor — within step height), feet then 0.5 into the target cell.
                if (this.heightAt(x + dx, y - 1, z + dz) - this.heightAt(x, y - 1, z) > 0.6)
                    return;
                if (!this.isSafe(this.flagsAt(x + dx, y + 2, z + dz)))
                    return;
            }
            else if ((fC & CLIMBABLE) === 0 || !this.isSafe(fC) || !this.climbUsable(x + dx, y, z + dz)) {
                return;
            }
            // else: step into a free-hanging ladder/vine cell — it catches.
        }
        else if (this.isThinFloor(fD)) {
            // A thin floor one below is a step DOWN into that cell — moveDropDown
            // produces the correct node; a same-level node here would float.
            return;
        }
        const activatable = (fC & GATE) !== 0 || (this.doorMode && (fC & DOOR_CLOSED) !== 0);
        const throughClosedDoor = this.canOpenDoors && this.doorMode && (fC & DOOR_CLOSED) !== 0;
        // blockB — the head cell. A closed door is TWO blocks tall: activating
        // the lower half opens both, so in door mode the closed UPPER half must
        // not veto the move (exclusion/entity weights still apply).
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
            cost += this.safeOrBreak(x + dx, y, z + dz); // blockC
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
        let cost = 2; // move cost (move+jump)
        // Entity-above checks, upstream verbatim (falling-entity safety).
        if ((fA & PHYSICAL) !== 0 && this.entitiesAt(x, y + 3, z) > 0)
            return;
        if ((fH & PHYSICAL) !== 0 && this.entitiesAt(x + dx, y + 3, z + dz) > 0)
            return;
        if ((fB & PHYSICAL) !== 0 && (fH & PHYSICAL) === 0 && (fC & PHYSICAL) === 0 &&
            this.entitiesAt(x + dx, y + 2, z + dz) > 0)
            return;
        // Upstream's !blockC.physical branch requires placing — impossible here.
        if ((fC & PHYSICAL) === 0) {
            // Extended: a fence/wall top is a stand too — feet land 0.5 into the
            // node cell, so the head needs the cell above the usual pair.
            if (!this.parkourExtended || !this.tallStandAt(x + dx, y, z + dz))
                return;
            if (!this.isSafe(this.flagsAt(x + dx, y + 3, z + dz)))
                return;
        }
        else if (this.isThinFloor(fC)) {
            // A thin floor at the target feet cell is same-level ground (moveForward
            // walks into it) — "jumping onto" it would land in the air above.
            return;
        }
        else if (this.parkourExtended && (fC & CLIMBABLE) !== 0) {
            // A ladder classifies as physical, so this would be a jump onto its
            // 3/16-deep top edge. The extended repertoire reaches a ladder by
            // stepping or jumping INTO its cell and climbing, never by hopping
            // between edges — a plan the executor cannot fly.
            return;
        }
        const hC = this.heightAt(x + dx, y, z + dz);
        const h0 = this.heightAt(x, y - 1, z);
        if (hC - h0 > 1.2)
            return; // Too high to jump
        cost += this.safeOrBreak(x, y + 2, z); // blockA
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + 2, z + dz); // blockH
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y + 1, z + dz); // blockB
        if (cost > 100)
            return;
        this.push(x + dx, y + 1, z + dz, cost, 0);
    }
    /**
     * Upstream getLandingBlock: scan down from (dx, -2, dz). Returns the
     * landing STAND cell y (already +1 above the physical block, or the liquid
     * cell itself), or NaN when there is no landing.
     */
  private findLanding (x: number, y: number, z: number, dx: number, dz: number): number {
        const lx = x + dx;
        const lz = z + dz;
        let ly = y - 2;
        while (ly > this.worldMinY) {
            const idx = this.cellIndex(lx, ly, lz);
            if (idx < 0)
                return NaN; // upstream null block → while condition fails → null
            const f = this.flags[idx];
            if ((f & LIQUID) !== 0 && this.isSafe(f))
                return ly;
            // Improvement (useBubbleColumns): a column catches the fall like water.
            if (this.special !== null && (this.special[idx] & BUBBLE_MASK) !== 0)
                return ly;
            // Extended: a ladder/vine below catches the drop (like findExtLanding).
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
                return NaN;
            }
            // Extended: a fence/wall top is a stand — feet 0.5 into the cell
            // above it, head reaching one cell further up.
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
  private moveDropDown (x: number, y: number, z: number, dx: number, dz: number): void {
        this.beginMove();
        let cost = 1; // move cost
        const landY = this.findLanding(x, y, z, dx, dz);
        if (Number.isNaN(landY))
            return;
        if (!this.infiniteLiquidDropdownDistance && (y - landY) > this.maxDropDown)
            return;
        cost += this.safeOrBreak(x + dx, y + 1, z + dz); // blockB
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y, z + dz); // blockC
        if (cost > 100)
            return;
        cost += this.safeOrBreak(x + dx, y - 1, z + dz); // blockD
        if (cost > 100)
            return;
        if ((this.flagsAt(x + dx, y, z + dz) & LIQUID) !== 0)
            return; // dont go underwater
        cost += this.entitiesAt(x + dx, landY, z + dz) * this.entityCost;
        this.push(x + dx, landY, z + dz, cost, 0);
        if (this.parkourExtended && this.special !== null)
            this.slimeBounce(x, y, z, x + dx, landY, z + dz);
    }
  private moveDown (x: number, y: number, z: number): void {
        this.beginMove();
        // Can't descend against an up-column's push (ride edges handle columns).
        if (this.specialAt(x, y, z) === BUBBLE_UP)
            return;
        let cost = 1; // move cost
        const landY = this.findLanding(x, y, z, 0, 0);
        if (Number.isNaN(landY))
            return;
        cost += this.safeOrBreak(x, y - 1, z); // block0 — with canDig: dig straight down
        if (cost > 100)
            return;
        if ((this.flagsAt(x, y, z) & LIQUID) !== 0)
            return; // dont go underwater
        cost += this.entitiesAt(x, landY, z) * this.entityCost;
        this.push(x, landY, z, cost, 0);
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
        let cost = 1; // move cost
        cost += this.safeOrBreak(x, y + 2, z); // block2
        if (cost > 100)
            return;
        // Upstream's non-climbable branch is 1x1 towering (placement) — dead.
        if ((f1 & CLIMBABLE) === 0)
            return;
        // Vines climb only with an adjacent solid block to press against —
        // vanilla's collision climb, and the only way prismarine-physics
        // ascends. A free-hanging curtain is passable but not climbable.
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
        let cost = Math.SQRT2; // move cost
        const fC = this.flagsAt(x + dx, y, z + dz);
        // Extended: never diagonally onto a ladder's top edge (see moveJumpUp);
        // ladders are entered cardinally, caught, or transferred to.
        if (this.parkourExtended && (fC & (PHYSICAL | CLIMBABLE)) === (PHYSICAL | CLIMBABLE))
            return;
        // A thin floor at the target feet cell is same-level ground, not a +1 hop.
        const yo = (fC & PHYSICAL) !== 0 && !this.isThinFloor(fC) ? 1 : 0;
        const h0 = this.heightAt(x, y - 1, z);
        // Two corner alternatives, each with its own toBreak set (upstream
        // toBreak1/toBreak2 — the cheaper corner's digs are kept).
        let cost1 = 0;
        cost1 += this.safeOrBreak(x, y + yo + 1, z + dz); // blockB1
        cost1 += this.safeOrBreak(x, y + yo, z + dz); // blockC1
        const hD1 = this.heightAt(x, y + yo - 1, z + dz);
        if (hD1 - h0 > 1.2)
            cost1 += this.safeOrBreak(x, y + yo - 1, z + dz); // blockD1
        const breaks1 = this.moveBreaks;
        this.moveBreaks = [];
        let cost2 = 0;
        cost2 += this.safeOrBreak(x + dx, y + yo + 1, z); // blockB2
        cost2 += this.safeOrBreak(x + dx, y + yo, z); // blockC2
        const hD2 = this.heightAt(x + dx, y + yo - 1, z);
        if (hD2 - h0 > 1.2)
            cost2 += this.safeOrBreak(x + dx, y + yo - 1, z); // blockD2
        const breaks2 = this.moveBreaks;
        // Keep the cheaper corner's cost AND digs (upstream tie → corner 2).
        if (cost1 < cost2) {
            cost += cost1;
            this.moveBreaks = breaks1;
        }
        else {
            cost += cost2;
            // this.moveBreaks already breaks2
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
        if (yo === 1) { // Case jump up by 1
            const hC = this.heightAt(x + dx, y, z + dz);
            if (hC - h0 > 1.2)
                return; // Too high to jump
            cost += this.safeOrBreak(x, y + 2, z);
            if (cost > 100)
                return;
            cost += 1;
            this.push(x + dx, y + 1, z + dz, cost, 0);
        }
        else if (((fD & PHYSICAL) !== 0 && !this.isThinFloor(fD)) || (fC & LIQUID) !== 0 ||
            this.isThinFloor(fC) ||
            (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) !== 0) {
            this.push(x + dx, y, z + dz, cost, 0);
        }
        else if ((this.flagsAt(x + dx, y - 2, z + dz) & PHYSICAL) !== 0 || (fD & LIQUID) !== 0) {
            if (!this.isSafe(fD))
                return; // don't self-immolate
            cost += this.entitiesAt(x + dx, y - 1, z + dz) * this.entityCost;
            this.push(x + dx, y - 1, z + dz, cost, 0);
        }
    }
    // Jump up, down or forward over a 1 block gap — upstream verbatim (never digs).
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
            return; // cant jump from water
        if ((this.specialAt(x, y, z) & BUBBLE_MASK) !== 0)
            return; // cant jump while floating in a column
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
                // Forward
                cost += this.exclusionAt(x + dxx, y + 1, z + dzz);
                this.push(x + dxx, y, z + dzz, cost, META_PARKOUR);
                break;
            }
            else if (ceilingClear && this.isSafe(fB) && (fCd & PHYSICAL) !== 0) {
                // Up
                if (this.isSafe(fA) && d !== 4) { // 4 forward 1 up fails often
                    cost += this.exclusionAt(x + dxx, y + 2, z + dzz);
                    const hC = this.heightAt(x + dxx, y, z + dzz);
                    if (hC - h0 > 1.2)
                        break; // Too high to jump
                    cost += this.entitiesAt(x + dxx, y + 1, z + dzz) * this.entityCost;
                    this.push(x + dxx, y + 1, z + dzz, cost, META_PARKOUR);
                    break;
                }
            }
            else if ((ceilingClear || d === 2) && this.isSafe(fB) && this.isSafe(fCd) && this.isSafe(fDd) && floorCleared) {
                // Down
                const fE = this.flagsAt(x + dxx, y - 2, z + dzz);
                if ((fE & PHYSICAL) !== 0) {
                    cost += this.exclusionAt(x + dxx, y - 1, z + dzz);
                    cost += this.entitiesAt(x + dxx, y - 1, z + dzz) * this.entityCost;
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
    /**
     * Improvement (allowParkourExtended): momentum chains. Every parkour
     * landing B this expansion just produced is also a possible RE-JUMP point:
     * a body that lands and presses jump on the same tick keeps its speed and
     * out-flies a running start (J_CHAIN). So for each such B that is a
     * stepping stone — a support with no walkable cell beside it to run from —
     * every table offset that continues the first jump's direction (angle
     * gate CHAIN_MIN_COS) is tried from B with the chain row, and the ones a
     * normal jump from B could not make become edges A → C with B as `via`.
     * Redundant chains are never emitted: if B's own expansion would reach C
     * (standing or running), this generates nothing. Cost is the sum of the
     * two edges, so the heuristic stays admissible. The executor flies A → B
     * as a parkour landing and presses jump toward C on the landing tick.
     */
  private momentumChains (x: number, y: number, z: number): void {
        const nOut = this.outCount;
        const tab = this.extTable!;
        for (let i = 0; i < nOut; i++) {
            if (this.outMeta[i] !== META_PARKOUR)
                continue;
            const bx = this.outX[i];
            const by = this.outY[i];
            const bz = this.outZ[i];
            // Landed ON a support: node above a physical top or a fence top —
            // never a ladder/water/thin-floor catch, which has no landing tick.
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
            // A stepping stone: nothing walkable beside it at its own level to
            // run from — otherwise B's expansion already has running jumps.
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
                continue; // no bonked-arc chain row measured
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
    /**
     * Improvement (allowParkourExtended): one candidate landing for one
     * extended-parkour offset — up (+1), same-level, drop (≤ maxDropDown), or a
     * ladder / water / bubble catch. No upstream equivalent (baritone and
     * azalea are cardinal-only, flat-only too). Rules: docs/ExtendedParkour.md.
     * Never digs; the executor's physics sim validates the jump at run time.
     *
     * With `chainVia ≥ 0` the takeoff is a momentum-chain re-jump from the
     * stepping stone (momentumChains): a landing a normal jump from here could
     * make is skipped (that edge exists from the stone's own expansion), the
     * rest are held to the J_CHAIN row from the landing point, and the push
     * carries `chainVia` and the first hop's cost.
     */
  private parkourExtTarget (x: number, y: number, z: number, sx: number, sz: number, h0: number, lowTakeoff: boolean, t: ParkourExtEntry, chainVia = -1, chainBase = 0): void {
        this.beginMove();
        const cells = t.cells;
        // Flat-ground fast-out: walkable floor on the first flight-line cell
        // means this isn't a gap (2 probes; kills almost every open-terrain node).
        const flx = x + cells[0] * sx;
        const flz = z + cells[1] * sz;
        if ((this.flagsAt(flx, y - 1, flz) & PHYSICAL) !== 0 && this.heightAt(flx, y - 1, flz) >= h0)
            return;
        const tx = x + t.tx * sx;
        const tz = z + t.tz * sz;
        const idxT = this.cellIndex(tx, y, tz);
        const fT = idxT < 0 ? 0 : this.flags[idxT];
        let nodeY;
        let cost = t.cost;
        // Top-catch class of the support the feet land ON (0 full … 3 post);
        // -1 for landings that ENTER a cell (ladder, water, thin floor) and get
        // the full cell-width credit.
        let landCatch = -1;
        // The cell whose collision top the feet land on (support landings): the
        // reach bucket is the REAL rise from the takeoff support to it, which
        // node cells do not carry — a fence stand (feet 0.5 into its node) to a
        // head (feet at its node's floor) is a flat jump one node cell up.
        let supY = NaN;
        if ((fT & CLIMBABLE) !== 0 && this.isSafe(fT) && this.climbUsable(tx, y, tz)) {
            // Grab a ladder/vine at flight level. Checked before PHYSICAL because
            // ladders classify as physical too (type-level bbox 'block') — and
            // grabbing one is real, landing on its thin collision top is not.
            // The arc may enter the column a cell lower where the ladder
            // continues: more airtime, so a longer reach — the catch is the
            // lowest contiguous ladder cell up to two below flight level.
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
            // Thin floor at flight level (carpeted landing): a same-level jump —
            // feet land IN the cell, exactly like the air-branch landing below.
            if (!this.isSafe(this.flagsAt(tx, y + 1, tz)))
                return;
            nodeY = y;
        }
        else if ((fT & PHYSICAL) !== 0) {
            // Up variant: the flight-level cell is the landing block, land on top.
            if (this.heightAt(tx, y, tz) - h0 > 1.2)
                return; // too high to jump
            if (!this.isSafe(this.flagsAt(tx, y + 1, tz)) || !this.isSafe(this.flagsAt(tx, y + 2, tz)))
                return;
            nodeY = y + 1;
            cost += 1;
            landCatch = this.heights[idxT] >> 6;
            supY = y;
        }
        else if (idxT >= 0 && this.isTallStand(fT, this.heights[idxT])) {
            // Fence/wall top at flight level: an up landing onto its 1.5 top —
            // feet 0.5 into the node cell, so the head needs one more cell. A pot
            // (or any low block) sitting on the post keeps the feet on the post's
            // tip but puts the node above the pot: the rise is still the tip's.
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
            if (!this.isSafe(fT) || !this.isSafe(this.flagsAt(tx, y + 1, tz)))
                return;
            nodeY = this.findExtLanding(tx, y, tz);
            if (Number.isNaN(nodeY))
                return;
            // Landed ON a support (node above it) rather than IN a catching cell?
            const fN = this.flagsAt(tx, nodeY, tz);
            if ((fN & (LIQUID | CLIMBABLE)) === 0 && !this.isThinFloor(fN) &&
                (this.specialAt(tx, nodeY, tz) & BUBBLE_MASK) === 0) {
                landCatch = this.catchAt(tx, nodeY - 1, tz);
                supY = nodeY - 1;
            }
        }
        // Corridor pass 1 — body cells at feet/head level must be passable, line
        // cells with walkable floor void the jump, and a blocked head+1 anywhere
        // (takeoff included) puts the jump into the head-hitter class: the arc
        // bonks at +0.2 and flies flattened, so y+2 clearance is NOT required.
        let low = lowTakeoff;
        for (let k = 0; k * 2 < cells.length; k++) {
            const cx = x + cells[k * 2] * sx;
            const cz = z + cells[k * 2 + 1] * sz;
            if (!this.isSafe(this.flagsAt(cx, y, cz)) ||
                !this.isSafe(this.flagsAt(cx, y + 1, cz)))
                return;
            if (!this.isSafe(this.flagsAt(cx, y + 2, cz)))
                low = true;
            if (k < t.nLine && (this.flagsAt(cx, y - 1, cz) & PHYSICAL) !== 0 &&
                this.heightAt(cx, y - 1, cz) >= h0)
                return; // walkable — not a gap
        }
        // Reach envelope: flight needed (per-axis corner credits, precomputed in
        // the table) vs usable flight for the landing bucket, at the speed the
        // takeoff supports — a run-up cell behind the flight line upgrades a
        // standing corner-creep jump to a running delayed jump. Head-hitter
        // jumps use the bonked-arc rows (+1 landings impossible there).
        // Real rise for support landings; node delta for catches. A fractional
        // rise (a post to a head, a head to a slab) interpolates between the two
        // integer rows — `frac` is how far from the lower-landing (longer-reach)
        // row toward the higher one; a rise above +1 keeps the +1 row (the 1.2
        // height check already gates it).
        let dy;
        let frac = 0;
        if (Number.isNaN(supY)) {
            dy = nodeY - y;
        }
        else {
            const rise = this.heightAt(tx, supY, tz) - h0;
            dy = Math.floor(rise);
            frac = rise - dy;
            if (dy >= 1) {
                dy = 1;
                frac = 0;
            }
        }
        const bucket = reachBucket(dy);
        // Reach is a function of the RUN available before the lip (J_RUN rows,
        // parkourEnvelope.ts): the support itself — rear overhang to the front
        // lip, 1.38 on a full block, 0.81 on a post — plus a block per walkable
        // cell behind the flight line, level or one step lower. Flight needed
        // is credited per axis at the lip; narrow supports (fence posts, heads,
        // pots) at either end shrink the credits and are recomputed here from
        // the table's offset, full-block ends take the precomputed value bit
        // for bit.
        const takeoffCatch = this.catchAt(x, y - 1, z);
        // A ladder's top edge is not a takeoff (its class says post-width, its
        // 3/16 depth says otherwise): jumps start from real stands only.
        if ((this.flagsAt(x, y - 1, z) & CLIMBABLE) !== 0)
            return;
        const lCred = landCatch < 0 ? LAND_HALF : LAND_NARROW_MARGIN + CATCH_HALF[landCatch];
        const half = CATCH_HALF[takeoffCatch];
        const front = Math.min(TAKEOFF_STAND, TAKEOFF_NARROW_MARGIN + half);
        let run = front + half + TAKEOFF_NARROW_MARGIN;
        const rx = x + t.runX * sx;
        const rz = z + t.runZ * sz;
        if (this.isSafe(this.flagsAt(rx, y, rz)) && this.isSafe(this.flagsAt(rx, y + 1, rz))) {
            if ((this.flagsAt(rx, y - 1, rz) & PHYSICAL) !== 0 && this.catchAt(rx, y - 1, rz) === 0) {
                // Level (or within step height) cell behind.
                const hR = this.heightAt(rx, y - 1, rz);
                if (hR - h0 <= 0.2 && h0 - hR <= 0.6)
                    run += 1;
            }
            else if (this.isSafe(this.flagsAt(rx, y - 1, rz)) &&
                (this.flagsAt(rx, y - 2, rz) & PHYSICAL) !== 0 && this.catchAt(rx, y - 2, rz) === 0) {
                // One step lower: sprinting up a step keeps the run.
                const hR = this.heightAt(rx, y - 2, rz);
                if (h0 - hR <= 1.05)
                    run += 1;
            }
        }
        const row = runRow(run);
        if (bucket < 0)
            return; // a rise above one block: no jump
        const rows = (low ? J_LOW_RUN : J_RUN)[row];
        // bucket = the row for the floor of the rise (the longer reach); the
        // next-higher landing is the row below it in the table.
        let usable = rows[bucket];
        if (frac > 0)
            usable = usable + (rows[bucket - 1] - usable) * frac;
        usable += this.marginCredit;
        const fn = takeoffCatch === 0 && landCatch <= 0
            ? t.fnStand
            : flightFrom(t.tx, t.tz, t.dist, front * t.dist / (t.tx > t.tz ? t.tx : t.tz), lCred);
        const feasible = fn <= usable;
        // Any run at all flies the running arc (the corridor curves).
        let needsRunning = row >= 1;
        if (chainVia < 0) {
            if (!feasible)
                return;
        }
        else {
            // A chain is only worth an edge where the stone's own jump falls
            // short. The executor lands the first hop on the FAR side of the stone
            // (the same creep credit a standing jump gets, so a post lands at its
            // tip's edge), and the re-jump flies the chain row from there — never
            // under a lid, on the running arc.
            if (feasible || low)
                return;
            const sChain = Math.min(TAKEOFF_STAND, TAKEOFF_NARROW_MARGIN + CATCH_HALF[takeoffCatch]) * CHAIN_TAKEOFF_FRACTION * t.dist / (t.tx > t.tz ? t.tx : t.tz);
            if (flightFrom(t.tx, t.tz, t.dist, sChain, lCred) > J_CHAIN[bucket] + this.marginCredit)
                return;
            needsRunning = true;
        }
        // Corridor pass 2 — per cell the flight-curve bound mf = the lowest the
        // feet can be over that cell (standing curve when a standing takeoff is
        // possible — it is the lower of the two; bonked curves for head-hitters).
        // Cells the body sweeps through must be passable; cells fully below the
        // feet line only veto when a tall shape (fence, wall) pokes up across
        // it. This is what lets a drop-jump fly over same-level corners near
        // takeoff (feet at apex there) instead of demanding full landing-depth
        // clearance everywhere.
        const mf = low
            ? (needsRunning ? t.mfLowRun : t.mfLowStand)
            : (needsRunning ? t.mfRun : t.mfStand);
        for (let k = 0; k * 2 < cells.length; k++) {
            const cx = x + cells[k * 2] * sx;
            const cz = z + cells[k * 2 + 1] * sz;
            // lim = lowest the feet get over this cell (+ grazing tolerance). One
            // cell below the body range is still probed: a fence top reaches 1.5
            // above its own cell.
            const lim = h0 + mf[k] + 0.05;
            const loLy = Math.floor(lim) - 1;
            for (let ly = y - 1; ly >= loLy; ly--) {
                if (ly + 1 > lim) {
                    if (!this.isSafe(this.flagsAt(cx, ly, cz)))
                        return; // body cell
                }
                else if (this.heightAt(cx, ly, cz) > lim) {
                    return; // pokes up into the flight path
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
        if (chainVia >= 0) {
            this.pendingVia = chainVia;
            this.push(tx, nodeY, tz, chainBase + cost, META_PARKOUR | META_CHAIN);
        }
        else {
            this.push(tx, nodeY, tz, cost, META_PARKOUR);
        }
    }
    /**
     * Landing scan for extended parkour: first support below flight level in
     * the target column — a physical top (node above it), or a water / bubble /
     * climbable catch (node at the cell itself). NaN when nothing in range.
     * Probe order mirrors findLanding, with the climbable case added.
     */
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
    /** Vanilla climbs vines only against a wall (mirrors moveUp's rule). */
  private climbUsable (x: number, y: number, z: number): boolean {
        if (this.specialAt(x, y, z) !== SPECIAL_VINE)
            return true;
        return (this.flagsAt(x + 1, y, z) & PHYSICAL) !== 0 ||
            (this.flagsAt(x - 1, y, z) & PHYSICAL) !== 0 ||
            (this.flagsAt(x, y, z + 1) & PHYSICAL) !== 0 ||
            (this.flagsAt(x, y, z - 1) & PHYSICAL) !== 0;
    }
}
