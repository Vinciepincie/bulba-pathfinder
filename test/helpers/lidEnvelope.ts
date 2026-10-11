// Lid-class envelope: the generator of test/helpers/jumpEnvelope.ts (measureRunTakeoff /
// measureFlightCurve) with the ceiling made a parameter — any block state at any height.
// lid = null: free; else { y, state } fills the layer y (stand level is y = 1).
import { Vec3 } from 'vec3'
import { createRequire } from 'node:module'
import { VoxelWorld, mcData, Block, STONE } from './voxelWorld.js'
import { RUN_LENGTHS, SAFETY_MARGIN, ENVELOPE_DY_MAX_DROP, measureRunTakeoff, measureFlightCurve } from './jumpEnvelope.js'
const require = createRequire(import.meta.url)
const { Physics, PlayerState } = require('prismarine-physics')

export interface Lid { y: number, state: number }

function stateWhere (name: string, want: Record<string, string>): number {
  const b = (mcData as any).blocksByName[name]
  for (let s = b.minStateId; s <= b.maxStateId; s++) {
    const p = (Block.fromStateId(s, 0) as any).getProperties()
    if (Object.entries(want).every(([k, v]) => String(p[k]) === v)) return s
  }
  throw new Error('no state ' + name + JSON.stringify(want))
}
export const TOP_SLAB = stateWhere('stone_slab', { type: 'top', waterlogged: 'false' })
export const TOP_TRAPDOOR = stateWhere('oak_trapdoor', { half: 'top', open: 'false', waterlogged: 'false', powered: 'false', facing: 'north' })

function bot (pos: Vec3): Record<string, unknown> {
  return {
    version: '1.21.1', registry: mcData,
    entity: { position: pos, velocity: new Vec3(0, 0, 0), onGround: true, isInWater: false, isInLava: false, isInWeb: false, isCollidedHorizontally: false, isCollidedVertically: false, elytraFlying: false, attributes: {}, effects: {}, yaw: -Math.PI / 2, pitch: 0 },
    jumpTicks: 0, jumpQueued: false, fireworkRocketDuration: 0, inventory: { slots: [] }
  }
}
function world (lid: Lid | null): { world: VoxelWorld, adapter: unknown } {
  const w = new VoxelWorld({ x0: -8, y0: -12, z0: -3, x1: 12, y1: 8, z1: 3 })
  w.fill(-8, 0, -1, 0, 0, 1, STONE)
  if (lid !== null) w.fill(-8, lid.y, -3, 12, lid.y, 3, lid.state)
  const adapter = {
    getBlock: (pos: Vec3) => {
      const s = w.stateAt(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      if (s === null) return null
      const blk = Block.fromStateId(s, 0) as { position?: Vec3 }
      blk.position = new Vec3(Math.floor(pos.x), Math.floor(pos.y), Math.floor(pos.z))
      return blk
    }
  }
  return { world: w, adapter }
}

/** measureRunTakeoff with a parametric lid (same loop, same margins). */
export function runTakeoffLid (runLength: number, lid: Lid | null): number[] {
  const { adapter } = world(lid)
  const physics = Physics(mcData, null)
  const state = new PlayerState(bot(new Vec3(0.5 - runLength, 1, 0.5)), { forward: true, back: false, left: false, right: false, jump: runLength === 0, sprint: true, sneak: false })
  const reach = new Array<number>(ENVELOPE_DY_MAX_DROP + 2).fill(0)
  let airborne = false
  let jumpX = 0.5
  for (let tick = 0; tick < 80; tick++) {
    if (!airborne && (state.pos.x as number) >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, adapter)
    if (!airborne && (state.jumpTicks as number) > 0) airborne = true
    if (!airborne) { jumpX = state.pos.x as number; continue }
    const dx = (state.pos.x as number) - jumpX
    const dy = (state.pos.y as number) - 1
    if (dy >= 1) reach[0] = Math.max(reach[0], dx)
    for (let k = 0; k <= ENVELOPE_DY_MAX_DROP; k++) if (dy >= -k) reach[k + 1] = Math.max(reach[k + 1], dx)
    if (dy < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return reach.map(r => r - SAFETY_MARGIN)
}

/** measureFlightCurve with a parametric lid. */
export function flightCurveLid (runUp: 0 | 1, lid: Lid | null): number[] {
  const { adapter } = world(lid)
  const physics = Physics(mcData, null)
  const state = new PlayerState(bot(new Vec3(runUp === 0 ? 0.5 : 0.5 - runUp, 1, 0.5)), { forward: true, back: false, left: false, right: false, jump: runUp === 0, sprint: true, sneak: false })
  const curve: number[] = []
  let airborne = false
  for (let tick = 0; tick < 60; tick++) {
    if (state.pos.x >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, adapter)
    if (!airborne && (state.jumpTicks as number) > 0) airborne = true
    if (!airborne) continue
    curve.push(state.pos.x - 0.5, state.pos.y - 1)
    if (state.pos.y - 1 < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return curve
}
export { RUN_LENGTHS, measureRunTakeoff, measureFlightCurve, STONE }

/** The trajectory measureRunTakeoff flies: [dx, dy] per airborne tick, dx from the JUMP POINT, dy from the walk level. */
export function runCurveLid (runLength: number, lid: Lid | null): number[] {
  const { adapter } = world(lid)
  const physics = Physics(mcData, null)
  const state = new PlayerState(bot(new Vec3(0.5 - runLength, 1, 0.5)), { forward: true, back: false, left: false, right: false, jump: runLength === 0, sprint: true, sneak: false })
  const curve: number[] = []
  let airborne = false
  let jumpX = 0.5
  for (let tick = 0; tick < 80; tick++) {
    if (!airborne && (state.pos.x as number) >= 0.5) state.control.jump = true
    physics.simulatePlayer(state, adapter)
    if (!airborne && (state.jumpTicks as number) > 0) airborne = true
    if (!airborne) { jumpX = state.pos.x as number; continue }
    curve.push((state.pos.x as number) - jumpX, (state.pos.y as number) - 1)
    if ((state.pos.y as number) - 1 < -(ENVELOPE_DY_MAX_DROP + 1)) break
  }
  return curve
}
