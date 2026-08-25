// Derives the parkour reach envelope from prismarine-physics itself — the
// same engine (and the same held controls: forward+sprint+jump) the executor
// uses to gate every jump at run time. src/parkourEnvelope.ts holds a pasted
// copy of this derivation; envelope.test.ts re-runs it and asserts equality,
// so the constants can never silently drift from the real physics.
import { Vec3 } from 'vec3'
import { createRequire } from 'node:module'
import { VoxelWorld, mcData, Block, TEST_VERSION, STONE } from './voxelWorld.js'

const require = createRequire(import.meta.url)
/* eslint-disable @typescript-eslint/no-var-requires */
const { Physics, PlayerState } = require('prismarine-physics')
/* eslint-enable @typescript-eslint/no-var-requires */

/** Landing-height buckets, in blocks relative to takeoff: +1 up … -8 drop. */
export const ENVELOPE_DY_MAX_UP = 1
export const ENVELOPE_DY_MAX_DROP = 8
/** Run-up classes measured: 0 = standing start, 1 = one walkable cell behind,
 * 2 = two — collapsed to standing/running (speed saturates within a block). */
export const ENVELOPE_CLASSES = 3

/**
 * SAFETY_MARGIN keeps us from planning frame-perfect jumps the executor sim
 * then refuses (a stall costs 3.5s + replan). Take-off and landing credits
 * are PER-AXIS in the flightNeeded model (parkourEnvelope.ts) — the tables
 * here are pure simulated flight distances minus this margin.
 */
export const SAFETY_MARGIN = 0.1

function makePhysicsBot (world: VoxelWorld, pos: Vec3): Record<string, unknown> {
  return {
    version: TEST_VERSION,
    registry: mcData,
    entity: {
      position: pos,
      velocity: new Vec3(0, 0, 0),
      onGround: true,
      isInWater: false,
      isInLava: false,
      isInWeb: false,
      isCollidedHorizontally: false,
      isCollidedVertically: false,
      elytraFlying: false,
      attributes: {},
      effects: {},
      yaw: -Math.PI / 2, // face +x (atan2(-1, -0))
      pitch: 0
    },
    jumpTicks: 0,
    jumpQueued: false,
    fireworkRocketDuration: 0,
    inventory: { slots: [] }
  }
}

/**
 * Simulate one takeoff: run `runUp` blocks of sprint approach on a runway,
 * jump at the takeoff-cell center, record the airborne trajectory relative
 * to the takeoff center, then return horizontal reach per landing height:
 * result[0] = reach landing +1 up, result[k] = reach landing k-1 down
 * (k = 1 … ENVELOPE_DY_MAX_DROP + 1). Reach is the flown distance at the
 * last tick the feet were still at/above the landing plane (conservative).
 */
export function measureTakeoff (runUp: number, lowCeiling = false): number[] {
  // Runway at y = 0 (stand y = 1) long enough behind, void ahead and below.
  // lowCeiling: a solid lid 2 above the feet (the head-hitter class) — the
  // jump bonks at +0.2 rise and flies a flattened arc, exactly as
  // prismarine-physics resolves the collision.
  const world = new VoxelWorld({ x0: -8, y0: -12, z0: -3, x1: 12, y1: 8, z1: 3 })
  world.fill(-8, 0, -1, 0, 0, 1, STONE)
  if (lowCeiling) world.fill(-8, 3, -3, 12, 3, 3, STONE)

  const physics = Physics(mcData, null)
  // Standing measures the pure jam from the cell center — the corner-creep
  // takeoff credit lives in the per-axis flightNeeded model, not here.
  const startX = runUp === 0 ? 0.5 : 0.5 - runUp
  const bot = makePhysicsBot(world, new Vec3(startX, 1, 0.5))
  const controls = {
    forward: true,
    back: false,
    left: false,
    right: false,
    jump: runUp === 0, // standing start holds jump from tick 0
    sprint: true,
    sneak: false
  }
  const state = new PlayerState(bot, controls)
  const worldAdapter = {
    getBlock: (pos: Vec3) => {
      const stateId = world.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      if (stateId === null) return null
      const block = Block.fromStateId(stateId, 0) as { position?: Vec3 }
      block.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      return block
    }
  }

  // Reach at index 0 → dy = +1 (up); index k → dy = -(k-1) (flat / drops).
  const reach = new Array<number>(ENVELOPE_DY_MAX_DROP + 2).fill(0)
  let airborne = false
  for (let tick = 0; tick < 60; tick++) {
    if (state.pos.x >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, worldAdapter)
    // jumpTicks latches to 10 on the tick the jump actually fires.
    if (!airborne && (state.jumpTicks as number) > 0) airborne = true
    if (!airborne) continue
    const dx = state.pos.x - 0.5
    const dy = state.pos.y - 1
    // Up landing: last airborne tick the feet are at/above +1.
    if (dy >= 1) reach[0] = Math.max(reach[0], dx)
    // Flat and drop landings: feet at/above the plane.
    for (let k = 0; k <= ENVELOPE_DY_MAX_DROP; k++) {
      if (dy >= -k) reach[k + 1] = Math.max(reach[k + 1], dx)
    }
    if (dy < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return reach
}

/**
 * Usable flight distance (pure center travel minus SAFETY_MARGIN) per
 * landing dy, two rows: [0] standing start, [1] running (≥1 walkable cell
 * behind the takeoff — the per-bucket max of the 1- and 2-cell run-up
 * measurements, since a longer run-up can always jump from wherever the
 * shorter one would). Credits for takeoff/landing geometry are applied
 * per-offset by flightNeeded() in parkourEnvelope.ts.
 */
export function deriveJumpEnvelope (lowCeiling = false): [number[], number[]] {
  const rows = [0, 1, 2].map(cls => measureTakeoff(cls, lowCeiling).map(r => r - SAFETY_MARGIN))
  const running = rows[1].map((v, i) => Math.max(v, rows[2][i]))
  return [rows[0], running]
}

/**
 * Momentum chain: land from a sprint-jump and re-jump on the FIRST grounded
 * tick (jump released in the air, pressed on landing — the executor's
 * press-on-landing cadence). The landing speed of a full sprint-jump
 * carries into the takeoff, so the second jump out-flies even a running
 * start. Measured for the weakest realistic incoming speed — a STANDING
 * first jump (no run-up) over a 1-block gap — and reported as usable flight
 * per landing dy from the landing point (same buckets as the other rows,
 * minus SAFETY_MARGIN). Only meaningful when the second jump continues in
 * the first one's direction; the solver gates on that.
 */
export function measureChainTakeoff (): number[] {
  const world = new VoxelWorld({ x0: -10, y0: -12, z0: -3, x1: 30, y1: 8, z1: 3 })
  world.fill(-10, 0, -1, 0, 0, 1, STONE) // takeoff runway, stand y=1
  world.fill(2, 0, -1, 3, 0, 1, STONE) // 2-wide landing platform after a 1-block gap
  const physics = Physics(mcData, null)
  const bot = makePhysicsBot(world, new Vec3(0.5, 1, 0.5))
  const state = new PlayerState(bot, { forward: true, back: false, left: false, right: false, jump: true, sprint: true, sneak: false })
  const worldAdapter = {
    getBlock: (pos: Vec3) => {
      const stateId = world.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      if (stateId === null) return null
      const block = Block.fromStateId(stateId, 0) as { position?: Vec3 }
      block.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      return block
    }
  }
  const reach = new Array<number>(ENVELOPE_DY_MAX_DROP + 2).fill(0)
  let phase = 0 // 0 first flight, 1 landed (press jump), 2 second flight
  let landX = 0
  for (let tick = 0; tick < 120; tick++) {
    if (phase === 0 && (state.jumpTicks as number) > 0) state.control.jump = false
    physics.simulatePlayer(state, worldAdapter)
    if (phase === 0 && state.onGround === true && (state.pos.x as number) >= 2) {
      phase = 1
      landX = state.pos.x as number
      state.control.jump = true
    } else if (phase === 1 && (state.vel.y as number) > 0.3) {
      phase = 2
    } else if (phase === 2) {
      const dx = (state.pos.x as number) - landX
      const dy = (state.pos.y as number) - 1
      if (dy >= 1) reach[0] = Math.max(reach[0], dx)
      for (let k = 0; k <= ENVELOPE_DY_MAX_DROP; k++) {
        if (dy >= -k) reach[k + 1] = Math.max(reach[k + 1], dx)
      }
      if (dy < -(ENVELOPE_DY_MAX_DROP + 1)) break
    }
  }
  return reach.map(r => r - SAFETY_MARGIN)
}

/** Slime bounces measured up to this drop height (deeper clamps here). */
export const BOUNCE_MAX_DROP = 8

/**
 * Slime-bounce apex per drop height: index d (2 … BOUNCE_MAX_DROP) = how
 * high the feet rebound above the slime TOP after a free fall of d blocks
 * onto it, straight down, no controls (prismarine-physics reflects vel.y on
 * the contact tick; holding forward mid-air never changes vel.y, so the apex
 * is control-independent). Indices 0/1 are 0: a sub-2-block drop rebounds
 * less than a normal jump and is never worth planning around.
 * src/parkourEnvelope.ts holds a pasted copy; envelope.test.ts re-derives.
 */
export function measureSlimeBounce (): number[] {
  const slimeState = (mcData as { blocksByName: Record<string, { minStateId: number }> })
    .blocksByName.slime_block.minStateId
  const apexes = new Array<number>(BOUNCE_MAX_DROP + 1).fill(0)
  for (let d = 2; d <= BOUNCE_MAX_DROP; d++) {
    const world = new VoxelWorld({ x0: -4, y0: -2, z0: -4, x1: 4, y1: d + 8, z1: 4 })
    world.fill(-4, 0, -4, 4, 0, 4, slimeState)
    const physics = Physics(mcData, null)
    const bot = makePhysicsBot(world, new Vec3(0.5, 1 + d, 0.5))
    ;(bot.entity as { onGround: boolean }).onGround = false
    const controls = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false }
    const state = new PlayerState(bot, controls)
    const worldAdapter = {
      getBlock: (pos: Vec3) => {
        const stateId = world.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
        if (stateId === null) return null
        const block = Block.fromStateId(stateId, 0) as { position?: Vec3 }
        block.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
        return block
      }
    }
    let bounced = false
    let apex = 0
    for (let tick = 0; tick < 120; tick++) {
      physics.simulatePlayer(state, worldAdapter)
      if (!bounced && (state.vel.y as number) > 0.05) bounced = true
      if (bounced) {
        const y = (state.pos.y as number) - 1
        if (y > apex) apex = y
        if ((state.vel.y as number) < 0) break
      }
    }
    apexes[d] = apex
  }
  return apexes
}

/**
 * Airborne flight curve for one takeoff class: [dx0, dy0, dx1, dy1, …] per
 * tick, dx from the takeoff-cell center along the flight line, dy from the
 * takeoff walk level. The arc is target-independent (the executor holds the
 * same controls for every jump), so ONE curve per class bounds the feet
 * height at any horizontal progress — parkourTable.ts turns it into per-cell
 * corridor clearance floors. Standing uses the 1-cell run-up = false branch.
 */
export function measureFlightCurve (runUp: 0 | 1, lowCeiling = false): number[] {
  const world = new VoxelWorld({ x0: -8, y0: -12, z0: -3, x1: 12, y1: 8, z1: 3 })
  world.fill(-8, 0, -1, 0, 0, 1, STONE)
  if (lowCeiling) world.fill(-8, 3, -3, 12, 3, 3, STONE)

  const physics = Physics(mcData, null)
  const startX = runUp === 0 ? 0.5 : 0.5 - runUp
  const bot = makePhysicsBot(world, new Vec3(startX, 1, 0.5))
  const controls = {
    forward: true,
    back: false,
    left: false,
    right: false,
    jump: runUp === 0,
    sprint: true,
    sneak: false
  }
  const state = new PlayerState(bot, controls)
  const worldAdapter = {
    getBlock: (pos: Vec3) => {
      const stateId = world.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      if (stateId === null) return null
      const block = Block.fromStateId(stateId, 0) as { position?: Vec3 }
      block.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      return block
    }
  }

  const curve: number[] = []
  let airborne = false
  for (let tick = 0; tick < 60; tick++) {
    if (state.pos.x >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, worldAdapter)
    if (!airborne && (state.jumpTicks as number) > 0) airborne = true
    if (!airborne) continue
    curve.push(state.pos.x - 0.5, state.pos.y - 1)
    if (state.pos.y - 1 < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return curve
}

/**
 * Run lengths (blocks of sprint before the lip) the envelope is measured at.
 * 0 = a jam from rest; a fence post's ±0.405 support zone gives ~0.8, a full
 * block's ~1.4 (rear overhang 0.78 to the 0.6 creep point), each walkable
 * run-up cell behind adds 1; the rows saturate around 2.
 */
export const RUN_LENGTHS: readonly number[] = [0, 0.4, 0.8, 1.2, 1.6, 2, 3]

/**
 * Usable flight per landing dy after sprinting `runLength` blocks and
 * jumping at that point (the executor's delayed-jump rollout picks the
 * tick). Measured from the JUMP POINT, so the per-axis lip credit of the
 * takeoff support is applied by flightNeeded exactly as for the jam. Minus
 * SAFETY_MARGIN. `runLength` 0 reproduces measureTakeoff(0).
 */
export function measureRunTakeoff (runLength: number, lowCeiling = false): number[] {
  const world = new VoxelWorld({ x0: -8, y0: -12, z0: -3, x1: 12, y1: 8, z1: 3 })
  world.fill(-8, 0, -1, 0, 0, 1, STONE)
  if (lowCeiling) world.fill(-8, 3, -3, 12, 3, 3, STONE)
  const physics = Physics(mcData, null)
  const startX = 0.5 - runLength
  const bot = makePhysicsBot(world, new Vec3(startX, 1, 0.5))
  const state = new PlayerState(bot, { forward: true, back: false, left: false, right: false, jump: runLength === 0, sprint: true, sneak: false })
  const worldAdapter = {
    getBlock: (pos: Vec3) => {
      const stateId = world.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      if (stateId === null) return null
      const block = Block.fromStateId(stateId, 0) as { position?: Vec3 }
      block.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      return block
    }
  }
  const reach = new Array<number>(ENVELOPE_DY_MAX_DROP + 2).fill(0)
  let airborne = false
  let jumpX = 0.5
  for (let tick = 0; tick < 80; tick++) {
    if (!airborne && (state.pos.x as number) >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, worldAdapter)
    if (!airborne && (state.jumpTicks as number) > 0) {
      airborne = true
      // The jump fired on this tick from the position the previous tick left.
    }
    if (!airborne) { jumpX = state.pos.x as number; continue }
    const dx = (state.pos.x as number) - jumpX
    const dy = (state.pos.y as number) - 1
    if (dy >= 1) reach[0] = Math.max(reach[0], dx)
    for (let k = 0; k <= ENVELOPE_DY_MAX_DROP; k++) {
      if (dy >= -k) reach[k + 1] = Math.max(reach[k + 1], dx)
    }
    if (dy < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return reach.map(r => r - SAFETY_MARGIN)
}
