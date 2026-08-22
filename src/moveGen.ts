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
import { LutFlags, LutSpecial, DigFlags } from './types.js'
import type { MovementsConfig, SnapshotMeta, DigData } from './types.js'
import { getParkourExtTable } from './parkourTable.js'
import type { ParkourExtEntry, ParkourExtTable } from './parkourTable.js'
import { J_STANDING, J_RUNNING, J_LOW_STANDING, J_LOW_RUNNING, reachBucket } from './parkourEnvelope.js'

export const META_PARKOUR = 1
export const META_USEONE = 2

const SAFE = LutFlags.SAFE
const PHYSICAL = LutFlags.PHYSICAL
const LIQUID = LutFlags.LIQUID
const CLIMBABLE = LutFlags.CLIMBABLE
const GATE = LutFlags.OPENABLE_GATE
const DOOR_CLOSED = LutFlags.DOOR_CLOSED
const DOOR_OPEN = LutFlags.DOOR_OPEN
const GATE_OPEN = LutFlags.GATE_OPEN
const PASSABLE_WHEN_OPEN = DOOR_OPEN | GATE_OPEN

const CAN_FALL = DigFlags.CAN_FALL
const CANT_BREAK = DigFlags.CANT_BREAK

const BUBBLE_UP = LutSpecial.BUBBLE_UP
const BUBBLE_DOWN = LutSpecial.BUBBLE_DOWN
const SPECIAL_VINE = LutSpecial.VINE
/** Bubble-only semantics (float support, fall catching, no-jump) must not
 * fire on VINE-marked cells. */
const BUBBLE_MASK = BUBBLE_UP | BUBBLE_DOWN

// Cardinal + diagonal probe tables, upstream order (W, E, N, S / NW, SW, NE, SE).
const CARDINAL_X = [-1, 1, 0, 0]
const CARDINAL_Z = [0, 0, -1, 1]
const DIAGONAL_X = [-1, -1, 1, 1]
const DIAGONAL_Z = [-1, 1, -1, 1]

// ── extended parkour (improvement, allowParkourExtended) ───────────────────
// Sprint-jumps upstream never generates: diagonal and long offsets, up (+1)
// and drop landings, and gap-jumps that catch a ladder / water / bubble
// column. Nothing is hand-picked: parkourTable.ts enumerates every integer
// landing offset within the physics-derived reach envelope
// (parkourEnvelope.ts) and computes each offset's swept flight corridor;
// acceptance couples distance to landing height and run-up speed. The wasm
// core receives the same table with the solve parameters. Rules and
// derivations: docs/ExtendedParkour.md.

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
  const base = extended ? 20 : 32
  if (!extended) return base
  const t = getParkourExtTable()
  return t.diag.length * 4 + t.cardX.length * 2 + t.cardZ.length * 2 + base
}

export class MoveGen {
  /** Output arrays for one expansion — see outCapacity(). */
  readonly outIdx: Int32Array
  readonly outX: Int32Array
  readonly outY: Int32Array
  readonly outZ: Int32Array
  readonly outCost: Float64Array
  readonly outMeta: Uint8Array
  /** toBreak per neighbor: outBreaks[i] is null or an array of cell indices. */
  readonly outBreaks: Array<number[] | null>
  outCount = 0

  /** Set when any probe left the snapshot — a noPath may be growth-fixable. */
  boundaryTouched = false

  private readonly flags: Uint8Array
  private readonly heights: Uint8Array
  private readonly x0: number
  private readonly y0: number
  private readonly z0: number
  private readonly w: number
  private readonly h: number
  private readonly l: number
  private readonly worldMinY: number

  private readonly entityMap: Map<number, number> | null
  private readonly stepExclusion: StepExclusionFn

  private readonly allowParkour: boolean
  private readonly allowSprinting: boolean
  /** allowParkourExtended && allowParkour && allowSprinting (all jumps need sprint). */
  private readonly parkourExtended: boolean
  /** Generated offset/corridor table (null when the feature is off). */
  private readonly extTable: ParkourExtTable | null
  private readonly canOpenDoors: boolean
  private readonly doorMode: boolean
  private readonly maxDropDown: number
  private readonly infiniteLiquidDropdownDistance: boolean
  private readonly liquidCost: number
  private readonly entityCost: number

  // ── bubble columns (null when useBubbleColumns is off — zero overhead) ──
  private readonly special: Uint8Array | null
  private readonly bubbleCost: number

  // ── digging (null when canDig is off — the hot path never touches it) ──
  private readonly dig: DigContext | null
  private readonly digCost: number
  private readonly dontCreateFlow: boolean
  private readonly dontMineUnderFallingBlock: boolean
  /** Per-move toBreak scratch (cell indices); committed on push. */
  private moveBreaks: number[] = []

  constructor (snap: SnapshotView, cfg: MovementsConfig, stepExclusion: StepExclusionFn = null, dig: DigContext | null = null) {
    this.flags = snap.flags
    this.heights = snap.heights
    const m = snap.meta
    this.x0 = m.x0
    this.y0 = m.y0
    this.z0 = m.z0
    this.w = m.w
    this.h = m.h
    this.l = m.l
    this.worldMinY = m.worldMinY

    if (snap.entityIdx.length > 0) {
      const map = new Map<number, number>()
      for (let i = 0; i < snap.entityIdx.length; i++) {
        map.set(snap.entityIdx[i], snap.entityWeight[i])
      }
      this.entityMap = map
    } else {
      this.entityMap = null
    }
    this.stepExclusion = stepExclusion

    this.allowParkour = cfg.allowParkour
    this.allowSprinting = cfg.allowSprinting
    this.parkourExtended = cfg.allowParkour && cfg.allowSprinting && cfg.allowParkourExtended
    this.extTable = this.parkourExtended ? getParkourExtTable() : null
    const cap = outCapacity(this.parkourExtended)
    this.outIdx = new Int32Array(cap)
    this.outX = new Int32Array(cap)
    this.outY = new Int32Array(cap)
    this.outZ = new Int32Array(cap)
    this.outCost = new Float64Array(cap)
    this.outMeta = new Uint8Array(cap)
    this.outBreaks = new Array(cap).fill(null)
    this.canOpenDoors = cfg.canOpenDoors
    this.doorMode = cfg.canOpenDoors && cfg.canOpenRealDoors
    this.maxDropDown = cfg.maxDropDown
    this.infiniteLiquidDropdownDistance = cfg.infiniteLiquidDropdownDistance
    this.liquidCost = cfg.liquidCost
    this.entityCost = cfg.entityCost

    // Presence of the grid is the signal — it exists only when a feature
    // (bubble columns, climbable vines) marked cells into it.
    this.special = snap.special ?? null
    this.bubbleCost = cfg.bubbleCost

    this.dig = cfg.canDig ? dig : null
    this.digCost = cfg.digCost
    this.dontCreateFlow = cfg.dontCreateFlow
    this.dontMineUnderFallingBlock = cfg.dontMineUnderFallingBlock
  }

  /** Cell index or -1 when out of the snapshot (upstream null-block). */
  cellIndex (x: number, y: number, z: number): number {
    const lx = x - this.x0
    const ly = y - this.y0
    const lz = z - this.z0
    if (lx < 0 || lx >= this.w || ly < 0 || ly >= this.h || lz < 0 || lz >= this.l) {
      this.boundaryTouched = true
      return -1
    }
    return (ly * this.l + lz) * this.w + lx
  }

  flagsAt (x: number, y: number, z: number): number {
    const idx = this.cellIndex(x, y, z)
    return idx < 0 ? 0 : this.flags[idx]
  }

  /** Absolute collision-top height of the cell (upstream b.height). */
  heightAt (x: number, y: number, z: number): number {
    const idx = this.cellIndex(x, y, z)
    return idx < 0 ? y : y + this.heights[idx] / 32
  }

  /** Upstream block.safe, plus the open-door/open-gate improvement when enabled. */
  private isSafe (f: number): boolean {
    return (f & SAFE) !== 0 || (this.doorMode && (f & PASSABLE_WHEN_OPEN) !== 0)
  }

  /**
   * Thin walk-in floor (carpet class): SAFE + PHYSICAL, not a climbable.
   * Feet stand IN this cell on its sub-block shape — never ON TOP of it as
   * a full block. Every "PHYSICAL ⇒ land at cell+1" site must branch here,
   * or carpeted floors produce landings one block up in the air.
   */
  private isThinFloor (f: number): boolean {
    return (f & (SAFE | PHYSICAL | CLIMBABLE)) === (SAFE | PHYSICAL)
  }

  /** LutSpecial byte of the cell (0 when the feature is off / out of box). */
  private specialAt (x: number, y: number, z: number): number {
    if (this.special === null) return 0
    const idx = this.cellIndex(x, y, z)
    return idx < 0 ? 0 : this.special[idx]
  }

  private entitiesAt (x: number, y: number, z: number): number {
    if (this.entityMap === null) return 0
    const idx = this.cellIndex(x, y, z)
    if (idx < 0) return 0
    return this.entityMap.get(idx) ?? 0
  }

  private exclusionAt (x: number, y: number, z: number): number {
    return this.stepExclusion === null ? 0 : this.stepExclusion(x, y, z)
  }

  /** Reset the per-move toBreak scratch — every generator calls this first. */
  private beginMove (): void {
    if (this.moveBreaks.length > 0) this.moveBreaks = []
  }

  /**
   * Upstream safeToBreak() against the snapshot (only reached when canDig).
   */
  private safeToBreak (x: number, y: number, z: number, idx: number): boolean {
    const dig = this.dig as DigContext
    if (this.dontCreateFlow) {
      // false if next to liquid (above + 4 lateral), upstream order.
      if ((this.flagsAt(x, y + 1, z) & LIQUID) !== 0) return false
      if ((this.flagsAt(x - 1, y, z) & LIQUID) !== 0) return false
      if ((this.flagsAt(x + 1, y, z) & LIQUID) !== 0) return false
      if ((this.flagsAt(x, y, z - 1) & LIQUID) !== 0) return false
      if ((this.flagsAt(x, y, z + 1) & LIQUID) !== 0) return false
    }
    if (this.dontMineUnderFallingBlock) {
      const aboveIdx = this.cellIndex(x, y + 1, z)
      const aboveState = aboveIdx >= 0 ? dig.states[aboveIdx] : 0
      if ((dig.data.flags[aboveState] & CAN_FALL) !== 0 || this.entitiesAt(x, y + 1, z) > 0) {
        return false
      }
    }
    const state = dig.states[idx]
    if ((dig.data.flags[state] & CANT_BREAK) !== 0) return false
    if (dig.breakExclusion !== null && dig.breakExclusion(x, y, z) >= 100) return false
    return true
  }

  /**
   * Upstream safeOrBreak(): exclusion + entity weight when the block is
   * safe; with canDig, unsafe-but-breakable blocks cost their dig labor and
   * are recorded in the per-move toBreak scratch; otherwise a flat 100.
   */
  private safeOrBreak (x: number, y: number, z: number): number {
    let cost = this.exclusionAt(x, y, z)
    cost += this.entitiesAt(x, y, z) * this.entityCost
    const idx = this.cellIndex(x, y, z)
    const f = idx < 0 ? 0 : this.flags[idx]
    if (this.isSafe(f)) return cost

    const dig = this.dig
    if (dig === null || idx < 0) return 100
    if (!this.safeToBreak(x, y, z, idx)) return 100
    this.moveBreaks.push(idx)

    // Entity above a physical breakable block would fall into the hole.
    if ((f & PHYSICAL) !== 0) cost += this.entitiesAt(x, y + 1, z) * this.entityCost

    cost += dig.data.labor[dig.states[idx]] * this.digCost
    return cost
  }

  private push (x: number, y: number, z: number, cost: number, meta: number): void {
    const idx = this.cellIndex(x, y, z)
    if (idx < 0) return // target outside snapshot — boundaryTouched already set
    const i = this.outCount++
    this.outIdx[i] = idx
    this.outX[i] = x
    this.outY[i] = y
    this.outZ[i] = z
    this.outCost[i] = cost
    this.outMeta[i] = meta
    if (this.moveBreaks.length > 0) {
      this.outBreaks[i] = this.moveBreaks
      this.moveBreaks = []
    } else {
      this.outBreaks[i] = null
    }
  }

  /** Fills the out* arrays for the node at (x, y, z), upstream order. */
  generate (x: number, y: number, z: number): void {
    this.outCount = 0
    // Extended-parkour takeoff gates are node-invariant — hoisted so a
    // non-jumpable node (in water, no headroom) pays 3 probes, not 40.
    let ext = false
    let h0 = 0
    let lowTakeoff = false
    if (this.parkourExtended) {
      // No y+2 requirement: a blocked head+1 at the takeoff just puts the
      // jump into the head-hitter class (bonked arc), like a real player
      // jumping inside a 2-high tunnel.
      ext = (this.flagsAt(x, y, z) & LIQUID) === 0 &&
        (this.specialAt(x, y, z) & BUBBLE_MASK) === 0
      if (ext) {
        h0 = this.heightAt(x, y - 1, z)
        lowTakeoff = !this.isSafe(this.flagsAt(x, y + 2, z))
      }
    }
    for (let i = 0; i < 4; i++) {
      const dx = CARDINAL_X[i]
      const dz = CARDINAL_Z[i]
      this.moveForward(x, y, z, dx, dz)
      this.moveJumpUp(x, y, z, dx, dz)
      this.moveDropDown(x, y, z, dx, dz)
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
      if (this.allowParkour && !ext) this.moveParkourForward(x, y, z, dx, dz)
      if (ext) {
        const tab = this.extTable as ParkourExtTable
        if (dx !== 0) {
          for (const t of tab.cardX) this.parkourExtTarget(x, y, z, dx, 1, h0, lowTakeoff, t)
        } else {
          for (const t of tab.cardZ) this.parkourExtTarget(x, y, z, 1, dz, h0, lowTakeoff, t)
        }
      }
    }
    for (let i = 0; i < 4; i++) {
      this.moveDiagonal(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i])
      if (ext) {
        for (const t of (this.extTable as ParkourExtTable).diag) {
          this.parkourExtTarget(x, y, z, DIAGONAL_X[i], DIAGONAL_Z[i], h0, lowTakeoff, t)
        }
      }
    }
    this.moveDown(x, y, z)
    this.moveUp(x, y, z)
    if (this.special !== null) {
      this.moveBubbleUp(x, y, z)
      this.moveBubbleDown(x, y, z)
    }
  }

  /**
   * Improvement (useBubbleColumns): ride one block up an up-column. The
   * target cell (y+1) is the current head cell — already passable — so like
   * moveUp only the NEW head cell (y+2) is charged.
   */
  private moveBubbleUp (x: number, y: number, z: number): void {
    this.beginMove()
    if (this.specialAt(x, y, z) !== BUBBLE_UP) return
    let cost = this.bubbleCost
    cost += this.safeOrBreak(x, y + 2, z)
    if (cost > 100) return
    this.push(x, y + 1, z, cost, 0)
  }

  /** Improvement (useBubbleColumns): sink one block down a down-column. */
  private moveBubbleDown (x: number, y: number, z: number): void {
    this.beginMove()
    if (this.specialAt(x, y, z) !== BUBBLE_DOWN) return
    let cost = this.bubbleCost
    cost += this.safeOrBreak(x, y - 1, z)
    if (cost > 100) return
    this.push(x, y - 1, z, cost, 0)
  }

  private moveForward (x: number, y: number, z: number, dx: number, dz: number): void {
    this.beginMove()
    const fC = this.flagsAt(x + dx, y, z + dz)
    const fD = this.flagsAt(x + dx, y - 1, z + dz)

    let cost = 1 // move cost
    cost += this.exclusionAt(x + dx, y, z + dz)

    // Upstream's !blockD.physical && !blockC.liquid branch is pure block
    // placement — dead with zero scaffolding, so such a move is impossible.
    // Improvement (useBubbleColumns): a bubble cell floats you like water.
    if ((fD & PHYSICAL) === 0 && (fC & LIQUID) === 0 &&
        (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) === 0) return

    // A thin floor one below is a step DOWN into that cell — moveDropDown
    // produces the correct node; a same-level node here would float.
    if (this.isThinFloor(fD)) return

    const activatable = (fC & GATE) !== 0 || (this.doorMode && (fC & DOOR_CLOSED) !== 0)
    const throughClosedDoor = this.canOpenDoors && this.doorMode && (fC & DOOR_CLOSED) !== 0

    // blockB — the head cell. A closed door is TWO blocks tall: activating
    // the lower half opens both, so in door mode the closed UPPER half must
    // not veto the move (exclusion/entity weights still apply).
    const fB = this.flagsAt(x + dx, y + 1, z + dz)
    if (throughClosedDoor && (fB & DOOR_CLOSED) !== 0) {
      cost += this.exclusionAt(x + dx, y + 1, z + dz)
      cost += this.entitiesAt(x + dx, y + 1, z + dz) * this.entityCost
    } else {
      cost += this.safeOrBreak(x + dx, y + 1, z + dz)
    }
    if (cost > 100) return

    let meta = 0
    if (this.canOpenDoors && activatable) {
      meta = META_USEONE
    } else {
      cost += this.safeOrBreak(x + dx, y, z + dz) // blockC
      if (cost > 100) return
    }

    if ((this.flagsAt(x, y, z) & LIQUID) !== 0) cost += this.liquidCost

    this.push(x + dx, y, z + dz, cost, meta)
  }

  private moveJumpUp (x: number, y: number, z: number, dx: number, dz: number): void {
    this.beginMove()
    const fA = this.flagsAt(x, y + 2, z)
    const fH = this.flagsAt(x + dx, y + 2, z + dz)
    const fB = this.flagsAt(x + dx, y + 1, z + dz)
    const fC = this.flagsAt(x + dx, y, z + dz)

    let cost = 2 // move cost (move+jump)

    // Entity-above checks, upstream verbatim (falling-entity safety).
    if ((fA & PHYSICAL) !== 0 && this.entitiesAt(x, y + 3, z) > 0) return
    if ((fH & PHYSICAL) !== 0 && this.entitiesAt(x + dx, y + 3, z + dz) > 0) return
    if ((fB & PHYSICAL) !== 0 && (fH & PHYSICAL) === 0 && (fC & PHYSICAL) === 0 &&
        this.entitiesAt(x + dx, y + 2, z + dz) > 0) return

    // Upstream's !blockC.physical branch requires placing — impossible here.
    if ((fC & PHYSICAL) === 0) return
    // A thin floor at the target feet cell is same-level ground (moveForward
    // walks into it) — "jumping onto" it would land in the air above.
    if (this.isThinFloor(fC)) return

    const hC = this.heightAt(x + dx, y, z + dz)
    const h0 = this.heightAt(x, y - 1, z)
    if (hC - h0 > 1.2) return // Too high to jump

    cost += this.safeOrBreak(x, y + 2, z) // blockA
    if (cost > 100) return
    cost += this.safeOrBreak(x + dx, y + 2, z + dz) // blockH
    if (cost > 100) return
    cost += this.safeOrBreak(x + dx, y + 1, z + dz) // blockB
    if (cost > 100) return

    this.push(x + dx, y + 1, z + dz, cost, 0)
  }

  /**
   * Upstream getLandingBlock: scan down from (dx, -2, dz). Returns the
   * landing STAND cell y (already +1 above the physical block, or the liquid
   * cell itself), or NaN when there is no landing.
   */
  private findLanding (x: number, y: number, z: number, dx: number, dz: number): number {
    const lx = x + dx
    const lz = z + dz
    let ly = y - 2
    while (ly > this.worldMinY) {
      const idx = this.cellIndex(lx, ly, lz)
      if (idx < 0) return NaN // upstream null block → while condition fails → null
      const f = this.flags[idx]
      if ((f & LIQUID) !== 0 && this.isSafe(f)) return ly
      // Improvement (useBubbleColumns): a column catches the fall like water.
      if (this.special !== null && (this.special[idx] & BUBBLE_MASK) !== 0) return ly
      // Thin floor (carpet class): feet land IN the cell, not on top of it.
      if (this.isThinFloor(f)) {
        if (y - ly <= this.maxDropDown) return ly
        return NaN
      }
      if ((f & PHYSICAL) !== 0) {
        if (y - ly <= this.maxDropDown) return ly + 1
        return NaN
      }
      if (!this.isSafe(f)) return NaN
      ly--
    }
    return NaN
  }

  private moveDropDown (x: number, y: number, z: number, dx: number, dz: number): void {
    this.beginMove()
    let cost = 1 // move cost

    const landY = this.findLanding(x, y, z, dx, dz)
    if (Number.isNaN(landY)) return
    if (!this.infiniteLiquidDropdownDistance && (y - landY) > this.maxDropDown) return

    cost += this.safeOrBreak(x + dx, y + 1, z + dz) // blockB
    if (cost > 100) return
    cost += this.safeOrBreak(x + dx, y, z + dz) // blockC
    if (cost > 100) return
    cost += this.safeOrBreak(x + dx, y - 1, z + dz) // blockD
    if (cost > 100) return

    if ((this.flagsAt(x + dx, y, z + dz) & LIQUID) !== 0) return // dont go underwater

    cost += this.entitiesAt(x + dx, landY, z + dz) * this.entityCost

    this.push(x + dx, landY, z + dz, cost, 0)
  }

  private moveDown (x: number, y: number, z: number): void {
    this.beginMove()
    // Can't descend against an up-column's push (ride edges handle columns).
    if (this.specialAt(x, y, z) === BUBBLE_UP) return
    let cost = 1 // move cost

    const landY = this.findLanding(x, y, z, 0, 0)
    if (Number.isNaN(landY)) return

    cost += this.safeOrBreak(x, y - 1, z) // block0 — with canDig: dig straight down
    if (cost > 100) return

    if ((this.flagsAt(x, y, z) & LIQUID) !== 0) return // dont go underwater

    cost += this.entitiesAt(x, landY, z) * this.entityCost

    this.push(x, landY, z, cost, 0)
  }

  private moveUp (x: number, y: number, z: number): void {
    this.beginMove()
    const f1 = this.flagsAt(x, y, z)
    if ((f1 & LIQUID) !== 0) return
    if (this.entitiesAt(x, y, z) > 0) return

    let cost = 1 // move cost
    cost += this.safeOrBreak(x, y + 2, z) // block2
    if (cost > 100) return

    // Upstream's non-climbable branch is 1x1 towering (placement) — dead.
    if ((f1 & CLIMBABLE) === 0) return

    // Vines climb only with an adjacent solid block to press against —
    // vanilla's collision climb, and the only way prismarine-physics
    // ascends. A free-hanging curtain is passable but not climbable.
    if (this.specialAt(x, y, z) === SPECIAL_VINE &&
        (this.flagsAt(x + 1, y, z) & PHYSICAL) === 0 &&
        (this.flagsAt(x - 1, y, z) & PHYSICAL) === 0 &&
        (this.flagsAt(x, y, z + 1) & PHYSICAL) === 0 &&
        (this.flagsAt(x, y, z - 1) & PHYSICAL) === 0) return

    this.push(x, y + 1, z, cost, 0)
  }

  private moveDiagonal (x: number, y: number, z: number, dx: number, dz: number): void {
    this.beginMove()
    let cost = Math.SQRT2 // move cost

    const fC = this.flagsAt(x + dx, y, z + dz)
    // A thin floor at the target feet cell is same-level ground, not a +1 hop.
    const yo = (fC & PHYSICAL) !== 0 && !this.isThinFloor(fC) ? 1 : 0
    const h0 = this.heightAt(x, y - 1, z)

    // Two corner alternatives, each with its own toBreak set (upstream
    // toBreak1/toBreak2 — the cheaper corner's digs are kept).
    let cost1 = 0
    cost1 += this.safeOrBreak(x, y + yo + 1, z + dz) // blockB1
    cost1 += this.safeOrBreak(x, y + yo, z + dz) // blockC1
    const hD1 = this.heightAt(x, y + yo - 1, z + dz)
    if (hD1 - h0 > 1.2) cost1 += this.safeOrBreak(x, y + yo - 1, z + dz) // blockD1
    const breaks1 = this.moveBreaks
    this.moveBreaks = []

    let cost2 = 0
    cost2 += this.safeOrBreak(x + dx, y + yo + 1, z) // blockB2
    cost2 += this.safeOrBreak(x + dx, y + yo, z) // blockC2
    const hD2 = this.heightAt(x + dx, y + yo - 1, z)
    if (hD2 - h0 > 1.2) cost2 += this.safeOrBreak(x + dx, y + yo - 1, z) // blockD2
    const breaks2 = this.moveBreaks

    // Keep the cheaper corner's cost AND digs (upstream tie → corner 2).
    if (cost1 < cost2) {
      cost += cost1
      this.moveBreaks = breaks1
    } else {
      cost += cost2
      // this.moveBreaks already breaks2
    }
    if (cost > 100) return

    cost += this.safeOrBreak(x + dx, y + yo, z + dz)
    if (cost > 100) return
    cost += this.safeOrBreak(x + dx, y + yo + 1, z + dz)
    if (cost > 100) return

    if ((this.flagsAt(x, y, z) & LIQUID) !== 0) cost += this.liquidCost

    const fD = this.flagsAt(x + dx, y - 1, z + dz)
    if (yo === 1) { // Case jump up by 1
      const hC = this.heightAt(x + dx, y, z + dz)
      if (hC - h0 > 1.2) return // Too high to jump
      cost += this.safeOrBreak(x, y + 2, z)
      if (cost > 100) return
      cost += 1
      this.push(x + dx, y + 1, z + dz, cost, 0)
    } else if (((fD & PHYSICAL) !== 0 && !this.isThinFloor(fD)) || (fC & LIQUID) !== 0 ||
               this.isThinFloor(fC) ||
               (this.specialAt(x + dx, y, z + dz) & BUBBLE_MASK) !== 0) {
      this.push(x + dx, y, z + dz, cost, 0)
    } else if ((this.flagsAt(x + dx, y - 2, z + dz) & PHYSICAL) !== 0 || (fD & LIQUID) !== 0) {
      if (!this.isSafe(fD)) return // don't self-immolate
      cost += this.entitiesAt(x + dx, y - 1, z + dz) * this.entityCost
      this.push(x + dx, y - 1, z + dz, cost, 0)
    }
  }

  // Jump up, down or forward over a 1 block gap — upstream verbatim (never digs).
  private moveParkourForward (x: number, y: number, z: number, dx: number, dz: number): void {
    this.beginMove()
    const h0 = this.heightAt(x, y - 1, z)
    const f1 = this.flagsAt(x + dx, y - 1, z + dz)
    const h1 = this.heightAt(x + dx, y - 1, z + dz)
    if (((f1 & PHYSICAL) !== 0 && h1 >= h0) ||
        !this.isSafe(this.flagsAt(x + dx, y, z + dz)) ||
        !this.isSafe(this.flagsAt(x + dx, y + 1, z + dz))) return
    if ((this.flagsAt(x, y, z) & LIQUID) !== 0) return // cant jump from water
    if ((this.specialAt(x, y, z) & BUBBLE_MASK) !== 0) return // cant jump while floating in a column

    let cost = 1
    cost += this.entitiesAt(x + dx, y, z + dz) * this.entityCost

    let ceilingClear = this.isSafe(this.flagsAt(x, y + 2, z)) && this.isSafe(this.flagsAt(x + dx, y + 2, z + dz))
    let floorCleared = (this.flagsAt(x + dx, y - 2, z + dz) & PHYSICAL) === 0
    const maxD = this.allowSprinting ? 4 : 2

    for (let d = 2; d <= maxD; d++) {
      const dxx = dx * d
      const dzz = dz * d
      const fA = this.flagsAt(x + dxx, y + 2, z + dzz)
      const fB = this.flagsAt(x + dxx, y + 1, z + dzz)
      const fCd = this.flagsAt(x + dxx, y, z + dzz)
      const fDd = this.flagsAt(x + dxx, y - 1, z + dzz)

      if (this.isSafe(fCd)) cost += this.entitiesAt(x + dxx, y, z + dzz) * this.entityCost

      if (ceilingClear && this.isSafe(fB) && this.isSafe(fCd) && (fDd & PHYSICAL) !== 0) {
        // Forward
        cost += this.exclusionAt(x + dxx, y + 1, z + dzz)
        this.push(x + dxx, y, z + dzz, cost, META_PARKOUR)
        break
      } else if (ceilingClear && this.isSafe(fB) && (fCd & PHYSICAL) !== 0) {
        // Up
        if (this.isSafe(fA) && d !== 4) { // 4 forward 1 up fails often
          cost += this.exclusionAt(x + dxx, y + 2, z + dzz)
          const hC = this.heightAt(x + dxx, y, z + dzz)
          if (hC - h0 > 1.2) break // Too high to jump
          cost += this.entitiesAt(x + dxx, y + 1, z + dzz) * this.entityCost
          this.push(x + dxx, y + 1, z + dzz, cost, META_PARKOUR)
          break
        }
      } else if ((ceilingClear || d === 2) && this.isSafe(fB) && this.isSafe(fCd) && this.isSafe(fDd) && floorCleared) {
        // Down
        const fE = this.flagsAt(x + dxx, y - 2, z + dzz)
        if ((fE & PHYSICAL) !== 0) {
          cost += this.exclusionAt(x + dxx, y - 1, z + dzz)
          cost += this.entitiesAt(x + dxx, y - 1, z + dzz) * this.entityCost
          this.push(x + dxx, y - 1, z + dzz, cost, META_PARKOUR)
        }
        floorCleared = floorCleared && (fE & PHYSICAL) === 0
      } else if (!this.isSafe(fB) || !this.isSafe(fCd)) {
        break
      }

      ceilingClear = ceilingClear && this.isSafe(fA)
    }
  }

  /**
   * Improvement (allowParkourExtended): one candidate landing for one
   * extended-parkour offset — up (+1), same-level, drop (≤ maxDropDown), or a
   * ladder / water / bubble catch. No upstream equivalent (baritone and
   * azalea are cardinal-only, flat-only too). Rules: docs/ExtendedParkour.md.
   * Never digs; the executor's physics sim validates the jump at run time.
   */
  private parkourExtTarget (x: number, y: number, z: number, sx: number, sz: number, h0: number, lowTakeoff: boolean, t: ParkourExtEntry): void {
    this.beginMove()
    const cells = t.cells
    // Flat-ground fast-out: walkable floor on the first flight-line cell
    // means this isn't a gap (2 probes; kills almost every open-terrain node).
    const flx = x + cells[0] * sx
    const flz = z + cells[1] * sz
    if ((this.flagsAt(flx, y - 1, flz) & PHYSICAL) !== 0 && this.heightAt(flx, y - 1, flz) >= h0) return

    const tx = x + t.tx * sx
    const tz = z + t.tz * sz
    const fT = this.flagsAt(tx, y, tz)

    let nodeY: number
    let cost = t.cost
    if ((fT & CLIMBABLE) !== 0 && this.isSafe(fT) && this.climbUsable(tx, y, tz)) {
      // Grab a ladder/vine at flight level. Checked before PHYSICAL because
      // ladders classify as physical too (type-level bbox 'block') — and
      // grabbing one is real, landing on its thin collision top is not.
      if (!this.isSafe(this.flagsAt(tx, y + 1, tz))) return
      nodeY = y
    } else if (this.isThinFloor(fT)) {
      // Thin floor at flight level (carpeted landing): a same-level jump —
      // feet land IN the cell, exactly like the air-branch landing below.
      if (!this.isSafe(this.flagsAt(tx, y + 1, tz))) return
      nodeY = y
    } else if ((fT & PHYSICAL) !== 0) {
      // Up variant: the flight-level cell is the landing block, land on top.
      if (this.heightAt(tx, y, tz) - h0 > 1.2) return // too high to jump
      if (!this.isSafe(this.flagsAt(tx, y + 1, tz)) || !this.isSafe(this.flagsAt(tx, y + 2, tz))) return
      nodeY = y + 1
      cost += 1
    } else {
      if (!this.isSafe(fT) || !this.isSafe(this.flagsAt(tx, y + 1, tz))) return
      nodeY = this.findExtLanding(tx, y, tz)
      if (Number.isNaN(nodeY)) return
    }

    // Corridor pass 1 — body cells at feet/head level must be passable, line
    // cells with walkable floor void the jump, and a blocked head+1 anywhere
    // (takeoff included) puts the jump into the head-hitter class: the arc
    // bonks at +0.2 and flies flattened, so y+2 clearance is NOT required.
    let low = lowTakeoff
    for (let k = 0; k * 2 < cells.length; k++) {
      const cx = x + cells[k * 2] * sx
      const cz = z + cells[k * 2 + 1] * sz
      if (!this.isSafe(this.flagsAt(cx, y, cz)) ||
          !this.isSafe(this.flagsAt(cx, y + 1, cz))) return
      if (!this.isSafe(this.flagsAt(cx, y + 2, cz))) low = true
      if (k < t.nLine && (this.flagsAt(cx, y - 1, cz) & PHYSICAL) !== 0 &&
          this.heightAt(cx, y - 1, cz) >= h0) return // walkable — not a gap
    }

    // Reach envelope: flight needed (per-axis corner credits, precomputed in
    // the table) vs usable flight for the landing bucket, at the speed the
    // takeoff supports — a run-up cell behind the flight line upgrades a
    // standing corner-creep jump to a running delayed jump. Head-hitter
    // jumps use the bonked-arc rows (+1 landings impossible there).
    const bucket = reachBucket(nodeY - y)
    const jStand = low ? J_LOW_STANDING : J_STANDING
    const jRun = low ? J_LOW_RUNNING : J_RUNNING
    const needsRunning = t.fnStand > jStand[bucket]
    if (needsRunning) {
      if (t.fnRun > jRun[bucket]) return
      const rx = x + t.runX * sx
      const rz = z + t.runZ * sz
      if ((this.flagsAt(rx, y - 1, rz) & PHYSICAL) === 0) return
      const hR = this.heightAt(rx, y - 1, rz)
      if (hR - h0 > 0.2 || h0 - hR > 0.2) return
      if (!this.isSafe(this.flagsAt(rx, y, rz)) || !this.isSafe(this.flagsAt(rx, y + 1, rz))) return
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
      : (needsRunning ? t.mfRun : t.mfStand)
    for (let k = 0; k * 2 < cells.length; k++) {
      const cx = x + cells[k * 2] * sx
      const cz = z + cells[k * 2 + 1] * sz
      // lim = lowest the feet get over this cell (+ grazing tolerance). One
      // cell below the body range is still probed: a fence top reaches 1.5
      // above its own cell.
      const lim = h0 + mf[k] + 0.05
      const loLy = Math.floor(lim) - 1
      for (let ly = y - 1; ly >= loLy; ly--) {
        if (ly + 1 > lim) {
          if (!this.isSafe(this.flagsAt(cx, ly, cz))) return // body cell
        } else if (this.heightAt(cx, ly, cz) > lim) {
          return // pokes up into the flight path
        }
      }
    }

    for (let k = 0; k < t.nLine; k++) {
      cost += this.entitiesAt(x + cells[k * 2] * sx, y, z + cells[k * 2 + 1] * sz) * this.entityCost
    }
    cost += this.entitiesAt(tx, nodeY, tz) * this.entityCost
    cost += this.exclusionAt(tx, nodeY, tz)
    if (cost > 100) return
    this.push(tx, nodeY, tz, cost, META_PARKOUR)
  }

  /**
   * Landing scan for extended parkour: first support below flight level in
   * the target column — a physical top (node above it), or a water / bubble /
   * climbable catch (node at the cell itself). NaN when nothing in range.
   * Probe order mirrors findLanding, with the climbable case added.
   */
  private findExtLanding (tx: number, y: number, tz: number): number {
    let ly = y - 1
    while (ly > this.worldMinY) {
      const idx = this.cellIndex(tx, ly, tz)
      if (idx < 0) return NaN
      const f = this.flags[idx]
      if ((f & LIQUID) !== 0 && this.isSafe(f)) {
        if (!this.infiniteLiquidDropdownDistance && y - ly > this.maxDropDown) return NaN
        return ly
      }
      if (this.special !== null && (this.special[idx] & BUBBLE_MASK) !== 0) return ly
      if ((f & CLIMBABLE) !== 0 && this.climbUsable(tx, ly, tz)) {
        if (y - ly > this.maxDropDown) return NaN
        return ly
      }
      // Thin floor (carpet class): feet land IN the cell, not on top of it.
      if (this.isThinFloor(f)) {
        if (y - ly > this.maxDropDown) return NaN
        return ly
      }
      if ((f & PHYSICAL) !== 0) {
        if (y - (ly + 1) > this.maxDropDown) return NaN
        return ly + 1
      }
      if (!this.isSafe(f)) return NaN
      ly--
    }
    return NaN
  }

  /** Vanilla climbs vines only against a wall (mirrors moveUp's rule). */
  private climbUsable (x: number, y: number, z: number): boolean {
    if (this.specialAt(x, y, z) !== SPECIAL_VINE) return true
    return (this.flagsAt(x + 1, y, z) & PHYSICAL) !== 0 ||
      (this.flagsAt(x - 1, y, z) & PHYSICAL) !== 0 ||
      (this.flagsAt(x, y, z + 1) & PHYSICAL) !== 0 ||
      (this.flagsAt(x, y, z - 1) & PHYSICAL) !== 0
  }
}
