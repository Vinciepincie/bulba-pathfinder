// Fall recovery: counts the fall like the server does, notices a fall the executor did not plan,
// predicts where it ends, and (in 'recover' mode) steers and uses a breaker so the landing costs least.
// Design and limits: docs/api.md#fall-recovery.
import { Vec3 } from 'vec3'
import type { Item } from 'prismarine-item'
import type { Bot } from 'mineflayer'
import { FallPredictor, COAST } from './predict.js'
import type { AirProgram, Landing, LandingMedium } from './predict.js'
import { clutchOptions } from './clutch.js'
import type { ClutchOption, ClutchContext } from './clutch.js'
import { usableBreakers, handFor, inHand, defaultFallItems } from './items.js'
import type { FallItemRule, FallItemRef, FallItemKind } from './items.js'
import { readFallBody } from './model.js'
import type { FallBody } from './model.js'
import { newBody } from '../playerSim.js'
import type { SimBody, SimWorld } from '../playerSim.js'

export type FallRecoveryMode = 'off' | 'observe' | 'recover'

/** `bot.pathfinder.fallRecovery`: change its fields, the object itself stays. */
export interface FallRecoveryOptions {
  /**
   * 'observe' (default) only reports falls. 'recover' also steers in the air and uses breakers, taking
   * the controls from the path and from autoEat while it does; never from an exclusive interrupt of your own.
   */
  mode: FallRecoveryMode
  /** Which items break a fall (default `defaultFallItems`). Order matters only between rules matching the same item. */
  items: readonly FallItemRule[]
  /** The items recovery may spend, for bots that own only part of their inventory (default: all of it). */
  inventory?: (bot: Bot) => Iterable<Item | null | undefined>
  /** Take the water back after a bucket clutch (default true). */
  pickUpWater: boolean
  /** Hold a totem from the hotbar when no plan survives (default true, 'recover' only). */
  equipTotem: boolean
  /**
   * Pick the plan yourself (experimental). Called once when a plan is chosen: return one of the
   * options, `null` for no item and no steering, or `undefined` for the default choice.
   */
  choose?: (options: readonly FallChoice[], info: FallInfo) => FallChoice | null | undefined
}

export type FallCause = 'knockback' | 'explosion' | 'support_lost' | 'missed_landing' | 'airborne' | 'planned'

/** A way to end the fall. */
export interface FallChoice {
  kind: 'none' | 'steer' | 'clutch' | 'totem'
  /** The breaker used, for 'clutch'. */
  item: string | null
  breaker: FallItemKind | null
  /** Where the body is predicted to land with this choice. */
  landing: { x: number, y: number, z: number, medium: LandingMedium }
  /** Predicted health lost on landing, and health after. */
  loss: number
  healthAfter: number
  /** Ticks in which the item can still be used; 1 leaves no margin. */
  windowTicks: number
  /** Health this choice costs if the item use fails (for 'clutch'). */
  failLoss: number
}

export interface FallInfo {
  id: number
  cause: FallCause
  /** Fall counted so far, in blocks. */
  fallDistance: number
  position: Vec3
  /** Ticks until the predicted landing with the chosen plan. */
  ticksToImpact: number
  /** The landing if nothing is done. */
  unaided: FallChoice
  plan: FallChoice
  /** False in 'observe' mode: the plan is only what recovery would do. */
  executed: boolean
  health: number
  minHealth: number
  breachesMinHealth: boolean
  /**
   * The landing will kill: no known plan survives, or (in 'observe' mode) the fall as it is.
   * A later tick can still find a way out.
   */
  lethal: boolean
}

export interface FallOutcome {
  id: number
  cause: FallCause
  plan: FallChoice
  executed: boolean
  position: Vec3
  healthBefore: number
  health: number
  /** Health lost between the start of the fall and settling after the landing. */
  damage: number
  /** A breaker packet was sent. */
  clutched: boolean
  /** The water came back: true, false when it failed, null when nothing was to pick up. */
  pickedUp: boolean | null
  /** Why the episode ended early: 'death', 'teleport', 'disconnect', 'disposed' or 'mode_off'. */
  aborted: string | null
}

/** What the guard needs from the executor and the plugin. */
export interface GuardHost {
  bot: never
  options: FallRecoveryOptions
  tables: () => Omit<SimWorld, 'stateAt'> | null
  wholeFall: () => boolean
  minHealth: () => number
  /** Is the executor driving a path right now, where does the path land, and does its next drop clutch? */
  flight: () => { driving: boolean, floorY: number | null, clutch: { x: number, y: number, z: number } | null }
  /** An exclusive interrupt of the user's holds the controls (recovery stays hands-off). */
  userHolds: () => boolean
  /** Stop autoEat and executor interactions before the guard writes. */
  takeOver: () => void
  /** The guard owned the controls and has let go; `planned` for a planned clutch drop. */
  released: (planned: boolean, landed: boolean) => void
  warn: (msg: string) => void
}

/** Ticks a knockback keeps an airborne body suspect. */
const KNOCK_TICKS = 4
/** Kernel ticks per game tick for the plan search (about 1 to 2 ms). */
const SEARCH_BUDGET = 4000
/** Re-run the full plan search this often while steering; between, only the chosen plan is re-flown. */
const SEARCH_EVERY = 3
/** Headings tried when steering, and release ticks for each. */
const HEADINGS = 16
const RELEASES = [Infinity, 6, 3]
/** After landing: ticks to wait for the server's damage and block updates before finishing. */
const SETTLE_TICKS = 12
/** Ticks to wait for the water to be confirmed before giving the pickup up. */
const PICKUP_TICKS = 40
/** Ticks after landing before the water is taken back (the landing packet must be out first). */
const PICKUP_AFTER_TICKS = 2
/** Ticks to wait for a sent pickup to fill the bucket, and how many sends to try. */
const PICKUP_CONFIRM_TICKS = 8
const PICKUP_TRIES = 3
/** PF_FALL_TRACE=1 logs the guard's decisions to stdout. */
const TRACE = typeof process !== 'undefined' && process.env.PF_FALL_TRACE === '1'
function trace (msg: string): void { if (TRACE) console.log(`[fall] ${msg}`) }
/** Ticks a staged bucket waits for its drop before the staging is undone. */
const PREP_TICKS = 100
/** Chance a clutch with one usable tick works, and with more. */
const P_ONE_TICK = 0.85
const P_MULTI_TICK = 0.97

interface Episode {
  id: number
  cause: FallCause
  planned: boolean
  clutchTarget: { x: number, y: number, z: number } | null
  executed: boolean
  owning: boolean
  startHealth: number
  plan: FallChoice
  unaided: FallChoice
  program: AirProgram
  clutch: ClutchOption | null
  /** Packets sent: no second use of a bucket in one fall. */
  usedSlots: Set<number>
  /** Cells this fall put water or a block in. */
  placed: Array<{ x: number, y: number, z: number }>
  startTick: number
  clutched: boolean
  clutchedWater: { x: number, y: number, z: number } | null
  /** The slot the water came from: the pickup uses its empty bucket. */
  waterSlot: number
  /** Swap made to bring a main-inventory item into the hotbar: undone after landing. */
  swap: { slot: number, hotbar: number } | null
  prevQuickBar: number
  emittedStart: boolean
  emittedLethal: boolean
  lastSearch: number
  landedAt: number
  landPos: Vec3 | null
  pickup: 'none' | 'waiting' | 'sent' | 'done' | 'failed'
  pickupTicks: number
  pickupSentAt: number
  pickupTries: number
  chooseAsked: boolean
  /** What `choose` returned: undefined for the default. */
  userPick: FallChoice | null | undefined
}

/** Wire the guard to a bot. Returns its controls for the plugin. */
export function createFallGuard (host: GuardHost): FallGuard {
  return new FallGuard(host)
}

export class FallGuard {
  private readonly bot: BotLike
  private predictor: FallPredictor | null = null
  private predictorTables: unknown = null
  /** Fall distance as the server counts it, and whether a teleport made it uncertain. */
  fallDistance = 0
  private uncertain = false
  private prevY: number | null = null
  private prevGrounded = true
  /** Physics ticks seen; the 'move' fire checks it. */
  private ticks = 0
  private knockAt = -100
  private explosionAt = -100
  /** Take-off snapshot: was the executor driving, and how low could its flight land? */
  private takeoff: { driving: boolean, floorY: number | null, clutch: { x: number, y: number, z: number } | null, knocked: boolean } | null = null
  private episode: Episode | null = null
  private nextId = 1
  private armed: { tick: number, opt: ClutchOption, ep: Episode, x: number, y: number, z: number } | null = null
  private lastHealth = 20
  private disposed = false

  constructor (private readonly host: GuardHost) {
    this.bot = host.bot as unknown as BotLike
  }

  /** The guard drives the body: the executor and interrupt grants stand aside. */
  owns (): boolean {
    return this.episode?.owning === true
  }

  /** A fall episode is open (observed or recovering). */
  active (): boolean { return this.episode !== null }

  /** Cells the guard placed a breaker in this fall; block updates there are its own. */
  ownsCell (x: number, y: number, z: number): boolean {
    const ep = this.episode
    if (ep === null) return false
    for (const c of ep.placed) if (c.x === x && c.y === y && c.z === z) return true
    return false
  }

  /** Self velocity from the server: a knockback unless it only follows a damaging landing. */
  onVelocity (vx: number, vy: number, vz: number): void {
    if (Math.hypot(vx, vz) >= 0.05 || vy >= 0.05) this.knockAt = this.ticks
  }

  onExplosion (): void { this.explosionAt = this.ticks }

  /** A server correction: the server keeps the fall it counted; the jump of the correction itself is not a fall. */
  onForcedMove (): void {
    this.prevY = this.bot.entity?.position.y ?? null
    this.armed = null
  }

  onSpawn (): void {
    this.fallDistance = 0
    this.prevY = null
    this.armed = null
    // A body that spawns mid-air carries a fall the client never learns.
    this.uncertain = this.bot.entity !== undefined && !this.supported()
    if (this.episode !== null) this.abort('teleport')
  }

  /** Death, disconnect, dispose or mode change: end the episode and hand everything back. */
  abort (why: string): void {
    const ep = this.episode
    if (ep === null) return
    this.armed = null
    this.finish(ep, why)
  }

  dispose (): void {
    this.abort('disposed')
    this.disposed = true
  }

  /** Runs on each physicsTick before the executor. */
  tick (): void {
    if (this.disposed) return
    this.ticks++
    this.armed = null
    const e = this.bot.entity
    if (e === undefined || e === null) return
    const opts = this.host.options
    this.track()
    if (opts.mode === 'off') {
      if (this.episode !== null) this.abort('mode_off')
      return
    }
    const ep = this.episode
    if (ep === null) {
      // Staged for a drop that never came (the path changed): put things back.
      if (this.prep !== null && this.ticks - this.prep.at > PREP_TICKS && this.supported() && !this.host.userHolds()) this.undoPrep()
      this.maybeStart()
      return
    }
    this.step(ep)
  }

  private undoPrep (): void {
    const p = this.prep
    this.prep = null
    if (p === null) return
    const b = this.bot
    if (b.currentWindow !== null && b.currentWindow !== undefined) return
    try {
      // Only if the staged item is still where staging put it.
      const there = b.inventory?.slots?.[36 + (p.swap?.hotbar ?? 0)]?.name
      if (p.swap !== null && (there === p.item || there === 'bucket')) void b.clickWindow(p.swap.slot, p.swap.hotbar, 2).catch(() => {})
      if (b.quickBarSlot !== p.prevQuickBar) b.setQuickBarSlot(p.prevQuickBar)
    } catch {}
  }

  /** The 'move' event: the movement packet for this tick is out, so a clutch fired now is judged from it. */
  onMove (): void {
    const a = this.armed
    if (a === null || a.tick !== this.ticks) return
    const p = this.bot.entity?.position
    // Teleport and respawn echoes also emit 'move'; only the tick's own packet counts.
    if (p === undefined || Math.abs(p.x - a.x) > 1e-6 || Math.abs(p.y - a.y) > 1e-6 || Math.abs(p.z - a.z) > 1e-6) return
    this.armed = null
    this.fire(a.ep, a.opt)
  }

  // ---- fall counting ----

  private supported (): boolean {
    const e = this.bot.entity
    if (e.onGround === true || e.isInWater === true) return true
    const feet = this.bot.blockAt(e.position, false)
    const n = feet?.name ?? ''
    return n === 'ladder' || n === 'vine' || n === 'scaffolding' || n.endsWith('_vines') || n.endsWith('_vines_plant') || n === 'cobweb' || n === 'powder_snow'
  }

  private track (): void {
    const e = this.bot.entity
    const y = e.position.y
    const grounded = this.supported()
    if (typeof this.bot.health === 'number') this.lastHealth = this.bot.health
    if (grounded) {
      this.fallDistance = 0
      this.uncertain = false
    } else if (this.prevY !== null && y < this.prevY) {
      this.fallDistance += this.prevY - y
    }
    if (this.slowFalling()) this.fallDistance = 0
    // Take-off: remember whether the executor launched this flight.
    if (this.prevGrounded && !grounded) {
      const f = this.host.flight()
      this.takeoff = { driving: f.driving, floorY: f.floorY, clutch: f.clutch, knocked: this.ticks - this.knockAt <= KNOCK_TICKS }
    }
    if (grounded) this.takeoff = null
    this.prevGrounded = grounded
    this.prevY = y
  }

  private slowFalling (): boolean {
    const effects = this.bot.entity.effects ?? {}
    for (const ef of Object.values(effects)) {
      const n = (this.bot.registry?.effects?.[ef.id]?.name ?? '').toLowerCase().replace(/_/g, '')
      if (n === 'slowfalling' || n === 'levitation') return true
    }
    return false
  }

  // ---- detection ----

  /** Why this airborne body is falling unplanned, or null when the executor flies it on purpose. */
  private unplannedCause (): FallCause | null {
    const t = this.takeoff
    const recentKnock = this.ticks - this.knockAt <= KNOCK_TICKS
    if (this.ticks - this.explosionAt <= KNOCK_TICKS) return 'explosion'
    if (t === null) return recentKnock ? 'knockback' : 'airborne'
    if (t.knocked || (recentKnock && this.ticks - this.knockAt >= 0)) return 'knockback'
    if (!t.driving) return 'airborne'
    // A planned flight turns unplanned once the body is clearly below everything the path lands on.
    const y = this.bot.entity.position.y
    if (t.floorY !== null && y < t.floorY - 1.2 && this.bot.entity.velocity.y < 0) return 'missed_landing'
    return null
  }

  private maybeStart (): void {
    if (this.supported()) return
    const e = this.bot.entity
    const t = this.takeoff
    // A planned clutch drop: the executor flies it, the guard places the water. The clutch node may come up
    // only after take-off (a hop or a corner cut leaves earlier), so the path is asked every airborne tick.
    if (t !== null && t.driving && this.host.options.mode === 'recover' && this.unplannedCause() === null) {
      const target = t.clutch ?? this.host.flight().clutch
      if (target !== null) {
        this.begin('planned', target)
        return
      }
    }
    const cause = this.unplannedCause()
    if (cause === null) return
    // Rising after a knock: plan early, the ascent is time to stage an item.
    if (e.velocity.y >= 0 && cause !== 'knockback' && cause !== 'explosion') return
    if (!this.mightHurt()) return
    this.begin(cause, null)
  }

  /** Cheap pre-check before any rollout: is there enough air below to hurt? */
  private mightHurt (): boolean {
    if (this.uncertain) return true
    const e = this.bot.entity
    const safe = readFallBody(this.bot).safeFall
    const pr = this.ensurePredictor()
    if (pr === null) return false
    const x = Math.floor(e.position.x); const z = Math.floor(e.position.z)
    const y0 = Math.floor(e.position.y)
    const rise = Math.max(0, e.velocity.y) * 6
    for (let y = y0; y > y0 - 64; y--) {
      const s = pr.world.stateAt(x, y, z)
      if (s < 0) return true
      const top = pr.topOf(s)
      const n = pr.blockInfo(s)?.name ?? ''
      if (n === 'water' || n === 'cobweb' || n === 'powder_snow') return false
      if (top !== null) return this.fallDistance + rise + (e.position.y - (y + top)) > safe - 0.5
    }
    return true
  }

  private begin (cause: FallCause, clutchTarget: { x: number, y: number, z: number } | null): void {
    const placeholder: FallChoice = { kind: 'none', item: null, breaker: null, landing: { x: 0, y: 0, z: 0, medium: 'unknown' }, loss: NaN, healthAfter: NaN, windowTicks: 0, failLoss: NaN }
    const recover = this.host.options.mode === 'recover'
    // The controls are taken only once a prediction shows the landing hurts (act): a harmless knock
    // must not cancel eating or digging, nor reset the path.
    const owning = false
    const ep: Episode = {
      id: this.nextId++,
      cause,
      planned: cause === 'planned',
      clutchTarget,
      executed: recover && (cause !== 'planned' || !this.host.userHolds()),
      owning,
      startHealth: this.lastHealth,
      plan: placeholder,
      unaided: placeholder,
      program: COAST,
      clutch: null,
      usedSlots: new Set(),
      placed: [],
      startTick: this.ticks,
      clutched: false,
      clutchedWater: null,
      waterSlot: -1,
      // A planned drop staged its bucket before the fall: that staging is undone with the episode.
      swap: cause === 'planned' ? this.prep?.swap ?? null : null,
      prevQuickBar: cause === 'planned' && this.prep !== null ? this.prep.prevQuickBar : this.bot.quickBarSlot ?? 0,
      emittedStart: false,
      emittedLethal: false,
      lastSearch: -100,
      landedAt: -1,
      landPos: null,
      pickup: 'none',
      pickupTicks: 0,
      pickupSentAt: 0,
      pickupTries: 0,
      chooseAsked: false,
      userPick: undefined
    }
    this.episode = ep
    if (cause === 'planned') this.prep = null
    trace(`begin #${ep.id} ${cause} at ${this.bot.entity.position.toString()} v ${this.bot.entity.velocity.toString()} fd ${this.fallDistance.toFixed(2)}`)
    this.step(ep)
  }

  // ---- the episode ----

  private step (ep: Episode): void {
    const e = this.bot.entity
    if (!this.syncMode(ep)) return
    if (ep.planned && ep.landedAt < 0) this.checkPlanned(ep)
    if (ep.landedAt >= 0) {
      // Knocked off again while cleaning up: this fall is over, the next one starts fresh.
      if (!this.supported() && this.ticks - ep.landedAt > 1 && e.velocity.y < -0.1) {
        this.finish(ep, null)
        this.maybeStart()
        return
      }
      this.afterLanding(ep)
      return
    }
    if (this.supported()) {
      ep.landedAt = this.ticks
      ep.landPos = e.position.clone()
      trace(`landed #${ep.id} at ${e.position.toString()} water ${String(e.isInWater)} plan ${ep.plan.kind}`)
      if (ep.owning) this.releaseKeys()
      if (ep.clutchedWater !== null && this.host.options.pickUpWater && ep.executed) {
        ep.pickup = 'waiting'
        // A planned drop's path goes on from the water: hold the body until the bucket is back.
        if (!ep.owning && !this.host.userHolds()) {
          ep.owning = true
          this.host.takeOver()
          this.releaseKeys()
        }
      }
      this.afterLanding(ep)
      return
    }
    const pr = this.ensurePredictor()
    if (pr === null) return
    pr.ticksSpent = 0
    const body = this.bodyNow()
    const fb = readFallBody(this.bot)
    const whole = this.host.wholeFall()
    const sprintOk = (this.bot.food ?? 20) > 6
    // The fall counted so far is unknown after a correction or a mid-air spawn: assume the worst.
    const fall0 = this.uncertain ? Math.max(this.fallDistance, 256) : this.fallDistance
    const coast = pr.predict(body, fall0, COAST, fb, whole, sprintOk)
    ep.unaided = this.asChoice('none', coast, null, coast.loss)
    let refs = ep.executed ? this.refs(ep) : this.refs(null)
    // One swap per fall: once a main-inventory item came into the hotbar, others stay out of reach.
    if (ep.swap !== null) refs = refs.filter(r => inHand(r.slot))
    // Search the plans: every few ticks in full, otherwise re-fly the chosen program.
    let best: { choice: FallChoice, program: AirProgram, clutch: ClutchOption | null, landing: Landing } | null = null
    const full = this.ticks - ep.lastSearch >= SEARCH_EVERY || ep.plan.kind === 'none'
    const programs: AirProgram[] = [COAST]
    if (full || ep.program.yaw !== null) {
      if (ep.program.yaw !== null) programs.push({ yaw: ep.program.yaw, sprint: ep.program.sprint, releaseAt: Math.max(0, ep.program.releaseAt - 1) })
      if (full) {
        ep.lastSearch = this.ticks
        for (let h = 0; h < HEADINGS; h++) {
          for (const r of RELEASES) {
            for (const sprint of [true, false]) programs.push({ yaw: -Math.PI + (2 * Math.PI * h) / HEADINGS, sprint, releaseAt: r })
          }
        }
      }
    }
    const ctx = this.clutchContext(pr, fb, whole)
    const ranked: Array<{ choice: FallChoice, program: AirProgram, clutch: ClutchOption | null, landing: Landing }> = []
    for (const prog of programs) {
      if (pr.ticksSpent > SEARCH_BUDGET && best !== null) break
      const l = prog === COAST ? coast : pr.predict(body, fall0, prog, fb, whole, sprintOk)
      if (Number.isNaN(l.loss)) continue
      const candidates = this.candidates(ep, l, prog, refs, ctx)
      for (const c of candidates) {
        const entry = { ...c, landing: l }
        ranked.push(entry)
        if (best === null || score(c.choice) < score(best.choice)) best = entry
      }
    }
    // A planned drop with no way to place its water any more is an unplanned fall now.
    if (best === null && ep.planned && !ep.clutched) {
      this.demote(ep, 'missed_landing')
      return
    }
    // Once the water is down there is nothing left to place: keep flying the plan as it was.
    if (best === null && ep.clutched) {
      ep.clutch = null
      ep.program = { ...ep.program, releaseAt: Math.max(0, ep.program.releaseAt - 1) }
    } else if (best === null) {
      ep.plan = ep.unaided
      ep.program = COAST
      ep.clutch = null
    } else {
      let chosen = best
      const custom = this.host.options.choose
      if (custom !== undefined && !ep.chooseAsked) {
        ep.chooseAsked = true
        ep.userPick = this.ask(custom, ep, best)
      }
      // The user's answer holds for the whole fall: null means no item and no steering,
      // a choice keeps its kind and item while one is still possible.
      if (ep.userPick === null) chosen = { choice: ep.unaided, program: COAST, clutch: null, landing: coast }
      else if (ep.userPick !== undefined && (ep.userPick.kind !== best.choice.kind || ep.userPick.item !== best.choice.item)) {
        const want = ep.userPick
        const alt = ranked.find(c => c.choice.kind === want.kind && c.choice.item === want.item)
        if (alt !== undefined) chosen = alt
      }
      ep.plan = chosen.choice
      ep.program = chosen.program
      ep.clutch = chosen.clutch
    }
    this.report(ep, coast.tick)
    if (this.episode !== ep || !ep.executed) return
    this.act(ep, pr)
  }

  /**
   * Follow a mode change made mid-fall: 'observe' hands everything back and only reports,
   * 'recover' takes over an unplanned fall it was only watching. False when the episode ended.
   */
  private syncMode (ep: Episode): boolean {
    const recover = this.host.options.mode === 'recover'
    if (!recover && ep.executed) {
      ep.executed = false
      ep.clutch = null
      // The pickup is an item use too: it stops with the rest.
      if (ep.pickup === 'waiting' || ep.pickup === 'sent') ep.pickup = 'failed'
      if (ep.owning) {
        ep.owning = false
        this.releaseKeys()
        this.restoreInventory(ep)
        this.host.released(ep.planned, false)
      }
    } else if (recover && !ep.executed && ep.landedAt < 0 && !this.host.userHolds()) {
      // act() takes the controls if the landing hurts.
      ep.executed = true
    }
    return this.episode === ep
  }

  /**
   * A planned drop knocked, blown or flown off its landing becomes an unplanned fall:
   * steering, every breaker and the totem come back into play.
   */
  private checkPlanned (ep: Episode): void {
    // Touchdown clears the take-off record: the landing is handled next, it is no deviation.
    if (this.supported()) return
    const cause = this.unplannedCause()
    if (cause !== null) this.demote(ep, cause)
  }

  private demote (ep: Episode, cause: FallCause): void {
    trace(`planned #${ep.id} turned ${cause}`)
    ep.planned = false
    ep.clutchTarget = null
    ep.cause = cause
    ep.clutch = null
    ep.lastSearch = -100
    ep.executed = this.host.options.mode === 'recover'
    // Ownership follows once the landing is shown to hurt (act).
  }

  /** Each landing's options: as is, and with each breaker that fits it. */
  private candidates (ep: Episode, l: Landing, prog: AirProgram, refs: FallItemRef[], ctx: ClutchContext): Array<{ choice: FallChoice, program: AirProgram, clutch: ClutchOption | null }> {
    const out: Array<{ choice: FallChoice, program: AirProgram, clutch: ClutchOption | null }> = []
    const kind = prog.yaw === null ? 'none' : 'steer'
    if (!ep.planned) out.push({ choice: this.asChoice(kind, l, null, l.loss), program: prog, clutch: null })
    if (refs.length === 0) return out
    // A planned drop clutches only its own landing; the executor steers it there, so a coast that
    // drifts a little still lands on the planned top face.
    if (ep.planned) {
      const t = ep.clutchTarget
      if (t === null || l.medium !== 'ground' || Math.abs(l.y - (t.y + 1)) > 0.01) return out
    }
    for (const opt of clutchOptions(l, refs.filter(r => !ep.usedSlots.has(r.slot)), ctx)) {
      if (opt.fireTick < 0) continue
      const choice = this.asChoice('clutch', l, opt, l.loss)
      // Only worth it when it saves more health than the item costs.
      if (!ep.planned && !(l.loss - opt.loss > opt.ref.cost) && Number.isFinite(l.loss) && l.healthAfter > 0) continue
      out.push({ choice, program: prog, clutch: opt })
    }
    return out
  }

  private asChoice (kind: FallChoice['kind'], l: Landing, opt: ClutchOption | null, failLoss: number): FallChoice {
    return {
      kind,
      item: opt?.ref.item.name ?? null,
      breaker: opt?.kind ?? null,
      landing: { x: l.x, y: l.y, z: l.z, medium: l.medium },
      loss: opt !== null ? opt.loss : l.loss,
      healthAfter: opt !== null ? opt.healthAfter : l.healthAfter,
      windowTicks: opt?.windowTicks ?? 0,
      failLoss
    }
  }

  private ask (custom: NonNullable<FallRecoveryOptions['choose']>, ep: Episode, best: { choice: FallChoice }): FallChoice | null | undefined {
    try {
      const info = this.info(ep, 0)
      const r = custom([best.choice, ep.unaided], info)
      return r
    } catch (err) {
      this.host.warn(`fallRecovery.choose threw: ${String(err)}`)
      return undefined
    }
  }

  // ---- reporting ----

  private info (ep: Episode, ticks: number): FallInfo {
    const minHealth = this.host.minHealth()
    const health = this.lastHealth
    return {
      id: ep.id,
      cause: ep.cause,
      fallDistance: this.fallDistance,
      position: this.bot.entity.position.clone(),
      ticksToImpact: ticks,
      unaided: ep.unaided,
      plan: ep.plan,
      executed: ep.executed,
      health,
      minHealth,
      breachesMinHealth: minHealth > 0 && this.outcome(ep).healthAfter < minHealth,
      lethal: this.outcome(ep).healthAfter <= 0
    }
  }

  /** What the landing will be: the plan when recovery carries it out, the unaided fall when it only watches. */
  private outcome (ep: Episode): FallChoice {
    const allowed = ep.executed && (ep.owning || !this.host.userHolds())
    return allowed ? ep.plan : ep.unaided
  }

  private report (ep: Episode, coastTicks: number): void {
    const ticks = ep.clutch !== null ? Math.max(coastTicks, ep.clutch.fireTick) : coastTicks
    // An unknown landing (unloaded chunks below) is no verdict either way.
    const m = ep.unaided.landing.medium
    const harmful = ep.unaided.loss > 0 || m === 'lava' || m === 'fire'
    if (!ep.emittedStart && (harmful || ep.planned)) {
      // A planned clutch drop is no surprise: it reports only once it turns unplanned.
      if (!ep.planned) {
        ep.emittedStart = true
        this.emit('pathfinder:fall_start', this.info(ep, ticks))
      }
    }
    if (this.episode !== ep) return
    const lethal = this.outcome(ep).healthAfter <= 0
    if (lethal && !ep.emittedLethal) {
      ep.emittedLethal = true
      this.emit('pathfinder:fall_lethal', this.info(ep, ticks))
    }
  }

  private emit (event: string, payload: unknown): void {
    try {
      (this.bot as unknown as { emit: (e: string, p: unknown) => void }).emit(event, payload)
    } catch (err) {
      this.host.warn(`${event} listener threw: ${String(err)}`)
    }
  }

  // ---- acting ----

  private act (ep: Episode, pr: FallPredictor): void {
    const b = this.bot
    // Your exclusive interrupt owns the hands as well as the keys.
    if (!ep.owning && this.host.userHolds()) return
    // A planned drop is flown by the guard from take-off, so the water goes where the body really lands;
    // an unplanned fall is taken over once it is shown to hurt.
    if (!ep.owning) {
      const u = ep.unaided
      const hurts = u.loss > 0 || u.landing.medium === 'lava' || u.landing.medium === 'fire' || u.healthAfter <= 0 ||
        (Number.isNaN(u.loss) && ep.plan.kind !== 'none' && Number.isFinite(ep.plan.loss))
      if (!hurts && !ep.planned) return
      ep.owning = true
      this.host.takeOver()
    }
    if (ep.owning) {
      const p = ep.program
      if (p.yaw !== null) {
        const held = p.releaseAt > 0
        b.look(p.yaw, b.entity.pitch, true)
        b.setControlState('forward', held)
        b.setControlState('sprint', held && p.sprint && (b.food ?? 20) > 6)
      } else {
        b.setControlState('forward', false)
        b.setControlState('sprint', false)
      }
      for (const k of ['back', 'left', 'right', 'jump', 'sneak'] as const) b.setControlState(k, false)
    }
    const opt = ep.clutch
    if (opt !== null) {
      this.stage(ep, opt.ref)
      if (opt.fireTick === 0) {
        const pos = b.entity.position
        this.armed = { tick: this.ticks, opt, ep, x: pos.x, y: pos.y, z: pos.z }
      }
    } else if (ep.plan.healthAfter <= 0 && ep.owning && this.host.options.equipTotem) {
      // A lethal plan still steers for the least damage; the totem decides whether that is enough.
      this.holdTotem(ep)
    }
    void pr
  }

  /** Get the item into a hand now, so the fire tick only needs the use packet. */
  private stage (ep: Episode, ref: FallItemRef): void {
    const b = this.bot
    const h = handFor(ref.slot)
    if (h.hand === 1) return
    if (h.hotbar !== null) {
      if (b.quickBarSlot !== h.hotbar) b.setQuickBarSlot(h.hotbar)
      return
    }
    if (ep.swap !== null || inHand(ref.slot)) return
    // One number-key click moves it into the held hotbar slot.
    const hotbar = b.quickBarSlot ?? 0
    try {
      void b.clickWindow(ref.slot, hotbar, 2).catch(() => {})
      ep.swap = { slot: ref.slot, hotbar }
      ref.slot = 36 + hotbar
    } catch (err) {
      this.host.warn(`fall recovery could not stage ${ref.item.name}: ${String(err)}`)
    }
  }

  private fire (ep: Episode, opt: ClutchOption): void {
    if (this.episode !== ep || ep.usedSlots.has(opt.ref.slot)) return
    // A listener may have switched modes or taken the bot since the fire was armed.
    if (this.host.options.mode !== 'recover' || !ep.executed || (!ep.owning && this.host.userHolds())) return
    const b = this.bot
    const h = handFor(opt.ref.slot)
    // The slot must hold the item this option is for: a swap may not have landed yet.
    if (b.inventory?.slots?.[opt.ref.slot]?.name !== opt.ref.item.name) { trace(`fire #${ep.id} skipped: slot ${opt.ref.slot} holds ${String(b.inventory?.slots?.[opt.ref.slot]?.name)}`); return }
    try {
      if (h.hotbar !== null && b.quickBarSlot !== h.hotbar) b.setQuickBarSlot(h.hotbar)
      ep.usedSlots.add(opt.ref.slot)
      trace(`fire #${ep.id} ${opt.kind} ${opt.ref.item.name} slot ${opt.ref.slot} from ${b.entity.position.toString()} at ${opt.support.x},${opt.support.y},${opt.support.z} window ${opt.windowTicks}`)
      if (opt.kind === 'water') {
        const e = b.entity
        const yaw = e.yaw; const pitch = e.pitch
        e.yaw = opt.yaw
        e.pitch = opt.pitch
        try { b.activateItem(h.hand === 1) } finally { e.yaw = yaw; e.pitch = pitch }
        ep.clutchedWater = opt.cell
        ep.waterSlot = opt.ref.slot
        ep.placed.push(opt.cell)
      } else {
        b._client.write('block_place', {
          hand: h.hand, location: opt.support, direction: 1, cursorX: 0.5, cursorY: 1, cursorZ: 0.5, insideBlock: false, worldBorderHit: false, sequence: 0
        })
        b.swingArm?.(h.hand === 1 ? 'left' : 'right')
        // The client would otherwise fall through the block the server just placed.
        ep.placed.push(opt.cell)
        const state = this.stateOf(opt.kind === 'cobweb' ? 'cobweb' : opt.ref.item.name)
        if (state !== null && b._updateBlockState !== undefined) b._updateBlockState(new Vec3(opt.cell.x, opt.cell.y, opt.cell.z), state)
      }
      ep.clutched = true
    } catch (err) {
      this.host.warn(`fall recovery could not use ${opt.ref.item.name}: ${String(err)}`)
    }
  }

  /** Hold a totem: the offhand already counts, a hotbar one is selected, one in the inventory is swapped into the hand. */
  private holdTotem (ep: Episode): void {
    const b = this.bot
    const slots = b.inventory?.slots
    if (slots === undefined || slots[45]?.name === 'totem_of_undying') return
    // Only a totem recovery may spend: `inventory` limits it like any breaker.
    let allowed: Set<number> | null = null
    const inv = this.host.options.inventory
    if (inv !== undefined) {
      allowed = new Set()
      try { for (const it of inv(b as never)) if (it !== null && it !== undefined && typeof it.slot === 'number') allowed.add(it.slot) } catch { return }
    }
    const ok = (slot: number): boolean => slots[slot]?.name === 'totem_of_undying' && (allowed === null || allowed.has(slot))
    for (let i = 0; i < 9; i++) {
      if (ok(36 + i)) {
        if (b.quickBarSlot !== i) b.setQuickBarSlot(i)
        return
      }
    }
    if (ep.swap !== null || (b.currentWindow !== null && b.currentWindow !== undefined)) return
    for (let s = 9; s <= 35; s++) {
      if (!ok(s)) continue
      const hotbar = b.quickBarSlot ?? 0
      try {
        void b.clickWindow(s, hotbar, 2).catch(() => {})
        ep.swap = { slot: s, hotbar }
      } catch {}
      return
    }
  }

  private releaseKeys (): void {
    for (const k of ['forward', 'back', 'left', 'right', 'jump', 'sprint', 'sneak'] as const) this.bot.setControlState(k, false)
  }

  // ---- after landing ----

  private afterLanding (ep: Episode): void {
    if (ep.pickup === 'waiting' || ep.pickup === 'sent') {
      this.pickUp(ep)
      if (ep.pickup === 'waiting' || ep.pickup === 'sent') return
    }
    if (this.ticks - ep.landedAt < SETTLE_TICKS) return
    this.finish(ep, null)
  }

  /**
   * Take the water back once the server shows it placed (water in the cell, an empty bucket in the slot),
   * then wait for the bucket to fill again; a miss is retried a few times.
   */
  private pickUp (ep: Episode): void {
    const b = this.bot
    if (this.host.options.mode !== 'recover' || !ep.executed || (!ep.owning && this.host.userHolds())) { ep.pickup = 'failed'; return }
    ep.pickupTicks++
    const c = ep.clutchedWater
    const slot = ep.waterSlot
    if (c === null || slot < 0) { ep.pickup = 'failed'; return }
    const water = b.blockAt(new Vec3(c.x, c.y, c.z), false)?.name === 'water'
    const held = b.inventory?.slots?.[slot]?.name
    if (ep.pickup === 'sent') {
      // A fish bucket comes back as a plain water bucket.
      if (held === 'water_bucket') { ep.pickup = 'done'; trace('pickup confirmed'); return }
      if (this.ticks - ep.pickupSentAt < PICKUP_CONFIRM_TICKS) return
      if (ep.pickupTries >= PICKUP_TRIES) { ep.pickup = 'failed'; trace('pickup failed: no confirmation'); return }
      ep.pickup = 'waiting'
    }
    // Never on the landing tick: this tick's movement packet, the one that lands, has not gone out yet,
    // and the server would take the water away before it counts the landing.
    if (this.ticks - ep.landedAt < PICKUP_AFTER_TICKS) return
    if (!(water && held === 'bucket')) {
      if (ep.pickupTicks > PICKUP_TICKS) { ep.pickup = 'failed'; trace(`pickup gave up: water ${String(water)} held ${String(held)}`) }
      return
    }
    const h = handFor(slot)
    const e = b.entity
    const eye = e.position.offset(0, 1.62, 0)
    // The source's surface, under the eye where possible: an empty bucket's ray stops at source blocks.
    const tx = clamp(eye.x, c.x + 0.05, c.x + 0.95); const tz = clamp(eye.z, c.z + 0.05, c.z + 0.95)
    const dx = tx - eye.x; const dy = c.y + 0.5 - eye.y; const dz = tz - eye.z
    if (Math.hypot(dx, dy, dz) > 4.3) { ep.pickup = 'failed'; trace('pickup out of reach'); return }
    if (h.hotbar !== null && b.quickBarSlot !== h.hotbar) b.setQuickBarSlot(h.hotbar)
    const yaw = e.yaw; const pitch = e.pitch
    e.yaw = Math.atan2(-dx, -dz)
    e.pitch = Math.atan2(dy, Math.hypot(dx, dz))
    try { b.activateItem(h.hand === 1) } finally { e.yaw = yaw; e.pitch = pitch }
    ep.pickup = 'sent'
    ep.pickupSentAt = this.ticks
    ep.pickupTries++
    trace(`pickup sent (try ${ep.pickupTries}) at ${c.x},${c.y},${c.z} from ${eye.x.toFixed(2)},${eye.y.toFixed(2)},${eye.z.toFixed(2)}`)
  }

  private finish (ep: Episode, aborted: string | null): void {
    const b = this.bot
    if (this.episode !== ep) return
    this.episode = null
    this.armed = null
    // Local keys after a dispose (no listener releases them later); after a disconnect nothing is sent.
    if (ep.owning && aborted !== 'disconnect') {
      try { this.releaseKeys() } catch {}
    }
    if ((ep.owning || ep.planned) && aborted !== 'disposed' && aborted !== 'disconnect') this.restoreInventory(ep)
    const health = typeof b.health === 'number' ? b.health : this.lastHealth
    const outcome: FallOutcome = {
      id: ep.id,
      cause: ep.cause,
      plan: ep.plan,
      executed: ep.executed,
      position: b.entity?.position?.clone() ?? new Vec3(0, 0, 0),
      healthBefore: ep.startHealth,
      health,
      damage: Math.max(0, ep.startHealth - health),
      clutched: ep.clutched,
      pickedUp: ep.pickup === 'none' ? null : ep.pickup === 'done',
      aborted
    }
    // A death, disconnect or dispose has its own cleanup: no path reset or interrupt grants from here.
    if ((ep.owning || ep.planned) && (aborted === null || aborted === 'mode_off')) this.host.released(ep.planned, aborted === null)
    if (ep.emittedStart) this.emit('pathfinder:fall_finish', outcome)
  }

  private restoreInventory (ep: Episode): void {
    const b = this.bot
    const swap = ep.swap
    ep.swap = null
    try {
      // Player-inventory slot numbers mean other slots in an open container: leave the swap then.
      if (swap !== null && (b.currentWindow === null || b.currentWindow === undefined)) void b.clickWindow(swap.slot, swap.hotbar, 2).catch(() => {})
      if (b.quickBarSlot !== ep.prevQuickBar) b.setQuickBarSlot(ep.prevQuickBar)
    } catch {}
  }

  // ---- planned clutch drops ----

  /** A water bucket the planner may count on: 'recover' mode, a usable bucket, and water that stays. */
  clutchAvailable (): boolean {
    if (this.host.options.mode !== 'recover' || this.waterEvaporates()) return false
    return this.refs(null).some(r => r.kind === 'water')
  }

  /** Before walking off a clutch drop: get the bucket into a hand. 'staging' means wait a tick. */
  prepareClutch (): 'ready' | 'staging' | 'missing' {
    if (!this.clutchAvailable()) return 'missing'
    const b = this.bot
    const ref = this.refs(null).filter(r => r.kind === 'water').sort((x, y) => x.cost - y.cost)[0]
    const h = handFor(ref.slot)
    this.prep ??= { prevQuickBar: b.quickBarSlot ?? 0, swap: null, at: this.ticks, item: ref.item.name }
    this.prep.at = this.ticks
    if (h.hand === 1) return 'ready'
    if (h.hotbar !== null) {
      if (b.quickBarSlot !== h.hotbar) b.setQuickBarSlot(h.hotbar)
      return 'ready'
    }
    if (this.ticks - this.stagedAt < 20) return 'staging'
    this.stagedAt = this.ticks
    try {
      const hotbar = b.quickBarSlot ?? 0
      void b.clickWindow(ref.slot, hotbar, 2).catch(() => {})
      this.prep.swap ??= { slot: ref.slot, hotbar }
    } catch {
      return 'missing'
    }
    return 'staging'
  }

  private stagedAt = -100
  /** What staging a planned drop's bucket changed, undone if no drop follows (PREP_TICKS) or after it. */
  private prep: { prevQuickBar: number, swap: { slot: number, hotbar: number } | null, at: number, item: string } | null = null
  /** The guard tick the predictor's world memo was last reset on. */
  private memoTick = -1

  /** Does the live world still allow water on this support? A full cube that holds no water, air above. */
  clutchLandingOk (x: number, y: number, z: number): boolean {
    const pr = this.ensurePredictor()
    if (pr === null) return false
    const s = pr.world.stateAt(x, y, z)
    const inf = pr.blockInfo(s)
    if (inf === null || inf.waterloggable || !pr.fullCube(s)) return false
    const above = pr.blockInfo(pr.world.stateAt(x, y + 1, z))
    return above !== null && (above.name === 'air' || above.name === 'cave_air')
  }

  // ---- plumbing ----

  private refs (ep: Episode | null): FallItemRef[] {
    const o = this.host.options
    const all = usableBreakers(this.bot as never, o.items ?? defaultFallItems, o.inventory)
    if (ep?.planned === true) return all.filter(r => r.kind === 'water')
    return all
  }

  private clutchContext (pr: FallPredictor, body: FallBody, whole: boolean): ClutchContext {
    return {
      predictor: pr,
      body,
      whole,
      waterEvaporates: this.waterEvaporates(),
      interactable: interactable,
      stateOf: (name) => this.stateOf(name),
      sprintOk: (this.bot.food ?? 20) > 6,
      bodyFor: (slot) => slot === 45 ? body : { ...body, totemInHand: this.bot.inventory?.slots?.[45]?.name === 'totem_of_undying' }
    }
  }

  waterEvaporates (): boolean {
    const d = String(this.bot.game?.dimension ?? '')
    return d.includes('nether')
  }

  private stateOf (name: string): number | null {
    const blk = this.bot.registry?.blocksByName?.[name]
    return blk?.defaultState ?? blk?.minStateId ?? null
  }

  private bodyNow (): SimBody {
    const e = this.bot.entity
    const b = newBody(e.position.x, e.position.y, e.position.z)
    b.vx = e.velocity.x
    b.vy = e.velocity.y
    b.vz = e.velocity.z
    b.onGround = e.onGround === true
    return b
  }

  private ensurePredictor (): FallPredictor | null {
    const tables = this.host.tables()
    if (tables === null) return null
    if (this.predictor === null || this.predictorTables !== tables) {
      const bot = this.bot
      const reg = bot.registry
      const infoCache = new Map<number, { name: string, waterloggable: boolean, full: boolean, spikeUp?: boolean } | null>()
      const info = (s: number): { name: string, waterloggable: boolean, full: boolean, spikeUp?: boolean } | null => {
        let v = infoCache.get(s)
        if (v === undefined) {
          const blk = reg?.blocksByStateId?.[s]
          v = blk === undefined ? null : { name: blk.name, waterloggable: (blk.states ?? []).some(st => st.name === 'waterlogged'), full: blk.boundingBox === 'block' }
          if (v !== null && blk !== undefined && blk.name === 'pointed_dripstone') {
            const props = stateProps(blk, s)
            v.spikeUp = props.thickness === 'tip' && props.vertical_direction === 'up'
          }
          infoCache.set(s, v)
        }
        return v
      }
      const read = (x: number, y: number, z: number): number => {
        try {
          const col = bot.world.getColumn(x >> 4, z >> 4)
          if (col === null || col === undefined) return -1
          const v = col.getBlockStateId({ x: x & 15, y, z: z & 15 })
          return typeof v === 'number' && v >= 0 ? v : -1
        } catch { return -1 }
      }
      this.predictor = new FallPredictor(tables, read, info)
      this.predictorTables = tables
      const g = bot.game
      if (typeof g?.minY === 'number') this.predictor.world.minY = g.minY
      if (typeof g?.height === 'number' && typeof g?.minY === 'number') this.predictor.world.maxY = g.minY + g.height
    }
    const g = this.bot.game
    if (typeof g?.minY === 'number') {
      this.predictor.world.minY = g.minY
      if (typeof g.height === 'number') this.predictor.world.maxY = g.minY + g.height
    }
    const ph = this.bot.physics ?? {}
    this.predictor.setBody(ph.playerHalfWidth ?? 0.3, ph.playerHeight ?? 1.8, 0.1)
    // Blocks change between ticks: read the world afresh once per tick, keys anchored at the body.
    if (this.memoTick !== this.ticks) {
      this.memoTick = this.ticks
      const p = this.bot.entity?.position
      this.predictor.world.beginTick(p?.x ?? 0, p?.z ?? 0)
    }
    return this.predictor
  }
}

/** Lower is better: survival first, then the expected health cost with the chance the item fails. */
export function score (c: FallChoice): number {
  if (Number.isNaN(c.loss)) return 1e9
  const dies = !(c.healthAfter > 0)
  if (c.kind !== 'clutch') return (dies ? 1e6 : 0) + (Number.isFinite(c.loss) ? c.loss : 1e5)
  const p = c.windowTicks <= 1 ? P_ONE_TICK : P_MULTI_TICK
  const fail = Number.isFinite(c.failLoss) ? c.failLoss : 1e5
  return (dies ? 1e6 : 0) + p * c.loss + (1 - p) * fail
}

const INTERACTABLE = /(chest|barrel|furnace|smoker|crafting_table|anvil|door|trapdoor|fence_gate|button|lever|_bed$|shulker_box|hopper|dispenser|dropper|note_block|jukebox|enchanting_table|beacon|brewing_stand|cake|comparator|repeater|daylight_detector|lectern|loom|grindstone|stonecutter|cartography_table|smithing_table|bell|campfire|composter|cauldron|respawn_anchor|crafter|decorated_pot|chiseled_bookshelf|flower_pot|_sign$|command_block|structure_block|jigsaw|ender_chest|redstone_ore|dragon_egg|vault|trial_spawner)/
function interactable (name: string): boolean { return INTERACTABLE.test(name) }

/** A state's properties from minecraft-data's state list (mixed radix, the last property varying fastest). */
function stateProps (blk: { minStateId?: number, states?: Array<{ name: string, num_values?: number, values?: string[], type?: string }> }, state: number): Record<string, string> {
  const out: Record<string, string> = {}
  let idx = state - (blk.minStateId ?? state)
  const list = blk.states ?? []
  for (let i = list.length - 1; i >= 0; i--) {
    const st = list[i]
    const n = st.num_values ?? (st.values?.length ?? 2)
    const v = idx % n
    idx = Math.floor(idx / n)
    out[st.name] = st.values?.[v] ?? (st.type === 'bool' ? (v === 0 ? 'true' : 'false') : String(v))
  }
  return out
}
function clamp (v: number, lo: number, hi: number): number { return v < lo ? lo : v > hi ? hi : v }

interface BotLike {
  entity: {
    position: Vec3
    velocity: Vec3
    onGround?: boolean
    isInWater?: boolean
    yaw: number
    pitch: number
    effects?: Record<number, { id: number, amplifier: number }>
  }
  health?: number
  food?: number
  quickBarSlot: number
  game?: { dimension?: string, minY?: number, height?: number }
  registry?: {
    effects?: Record<number, { name?: string }>
    blocksByName?: Record<string, { defaultState?: number, minStateId?: number }>
    blocksByStateId?: Record<number, { name: string, minStateId?: number, states?: Array<{ name: string, num_values?: number, values?: string[], type?: string }>, boundingBox?: string }>
  }
  inventory?: { slots?: Array<Item | null> }
  currentWindow?: unknown
  physics?: { playerHalfWidth?: number, playerHeight?: number }
  world: { getColumn: (cx: number, cz: number) => { getBlockStateId: (p: { x: number, y: number, z: number }) => number } | null | undefined }
  blockAt: (p: Vec3, extra?: boolean) => { name?: string } | null
  look: (yaw: number, pitch: number, force?: boolean) => Promise<void> | void
  setControlState: (k: 'forward' | 'back' | 'left' | 'right' | 'jump' | 'sprint' | 'sneak', v: boolean) => void
  setQuickBarSlot: (i: number) => void
  clickWindow: (slot: number, button: number, mode: number) => Promise<void>
  activateItem: (offhand?: boolean) => void
  swingArm?: (arm?: 'left' | 'right') => void
  _client: { write: (name: string, data: unknown) => void }
  _updateBlockState?: (p: Vec3, state: number) => void
}
