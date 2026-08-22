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
export const SAFETY_MARGIN = 0.2

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
