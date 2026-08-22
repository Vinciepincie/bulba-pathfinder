// A driveable fake mineflayer bot for executor-level tests: EventEmitter,
// control states, crude-but-honest voxel physics (walk toward yaw, gravity,
// 1-block step-up while jumping), and a physicsTick pump. Generalizes the
// repo's FarmPillar fake-physics harness (plan §5.3).
import { EventEmitter } from 'node:events'
import { Vec3 } from 'vec3'
import type { PhysicsLike, XYZ } from '../../src/types.js'
import { VoxelWorld, makeFakeBot, Block, AIR as AIR_STATE } from './voxelWorld.js'

const WALK_SPEED = 0.216 // blocks/tick (~4.317 m/s)
const SPRINT_SPEED = 0.28
const JUMP_VY = 0.42
const GRAVITY = 0.08

export interface DriveableBot extends EventEmitter {
  registry: unknown
  version: string
  game: { minY: number, height: number }
  world: unknown
  entity: {
    position: Vec3
    velocity: Vec3
    onGround: boolean
    effects: Record<string, unknown>
    height: number
    yaw: number
    isInWater?: boolean
  }
  entities: Record<string, unknown>
  inventory: { items: () => unknown[] }
  controlState: Record<string, boolean>
  blockAt: (pos: Vec3, extraInfos?: boolean) => unknown
  setControlState: (name: string, value: boolean) => void
  clearControlStates: () => void
  look: (yaw: number, pitch: number, force?: boolean) => Promise<void>
  lookAt: (pos: Vec3, force?: boolean) => Promise<void>
  activateBlock: (block: unknown) => Promise<void>
  loadPlugin: (plugin: (bot: unknown) => void) => void
  physics: { simulatePlayer: () => void }
  pathfinder?: Record<string, unknown>
  /** Test hooks */
  tick: () => void
  run: (ticks: number, everyMs?: number) => Promise<void>
  frozen: boolean
  activations: Vec3[]
  /** Every position bot.dig was called for. */
  digs: Vec3[]
}

function solidTop (world: VoxelWorld, x: number, y: number, z: number): number {
  const state = world.stateAt(x, y, z)
  if (state === null) return 1 // unloaded = solid
  const block = Block.fromStateId(state, 0) as { shapes: number[][], name: string }
  if (block.name.includes('door') || block.name.includes('gate')) {
    // treat like the real game: open door/gate panels don't block our crude
    // hitbox; closed ones do
    const props = (Block.fromStateId(state, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
    if (props.open === true || props.open === 'true') return 0
  }
  let top = 0
  for (const s of block.shapes ?? []) top = Math.max(top, s[4])
  return top
}

function blocked (world: VoxelWorld, x: number, feetY: number, z: number): boolean {
  // Body occupies feet + head cells; a collision top > 0.6 in the feet cell
  // or anything substantial in the head cell blocks.
  if (solidTop(world, x, feetY, z) > 0.6) return true
  if (solidTop(world, x, feetY + 1, z) > 0.11) return true
  return false
}

export function makeDriveableBot (world: VoxelWorld, startPos: Vec3): DriveableBot {
  const base = makeFakeBot(world, startPos.clone())
  const bot = new EventEmitter() as DriveableBot
  Object.assign(bot, base)
  bot.entity = {
    position: startPos.clone(),
    velocity: new Vec3(0, 0, 0),
    onGround: true,
    effects: {},
    height: 1.8,
    yaw: 0
  }
  bot.controlState = { forward: false, back: false, left: false, right: false, jump: false, sprint: false, sneak: false }
  bot.frozen = false
  bot.activations = []

  bot.setControlState = (name, value) => { bot.controlState[name] = value }
  bot.clearControlStates = () => {
    for (const k of Object.keys(bot.controlState)) bot.controlState[k] = false
  }
  bot.look = async (yaw) => { bot.entity.yaw = yaw }
  bot.lookAt = async (pos) => {
    const d = pos.minus(bot.entity.position)
    bot.entity.yaw = Math.atan2(-d.x, -d.z)
  }
  bot.activateBlock = async (block) => {
    const b = block as { position: Vec3, stateId?: number, name: string }
    bot.activations.push(b.position.clone())
    // Toggle a door/gate open (flip the `open` property state). Enough for
    // the executor's useOne flow: find a same-type state with open flipped.
    const state = world.stateAt(b.position.x, b.position.y, b.position.z)
    if (state === null) return
    const cur = Block.fromStateId(state, 0) as { type: number, getProperties: () => Record<string, unknown> }
    const props = cur.getProperties()
    const wantOpen = !(props.open === true || props.open === 'true')
    const blockType = (bot.registry as { blocksArray: Array<{ id: number, minStateId: number, maxStateId: number }> })
      .blocksArray.find(bb => bb.id === cur.type)
    if (!blockType) return
    for (let s = blockType.minStateId; s <= blockType.maxStateId; s++) {
      const cand = Block.fromStateId(s, 0) as { getProperties: () => Record<string, unknown> }
      const cp = cand.getProperties()
      const candOpen = cp.open === true || cp.open === 'true'
      if (candOpen === wantOpen &&
          cp.facing === props.facing && cp.half === props.half && cp.hinge === props.hinge) {
        world.set(b.position.x, b.position.y, b.position.z, s)
        return
      }
    }
  }
  bot.loadPlugin = (plugin) => { plugin(bot) }
  bot.physics = { simulatePlayer: () => {} }
  bot.pathfinder = undefined

  // Digging surface (canDig executor tests): resolves after a short delay,
  // clears the cell, fires diggingCompleted + the blockUpdate a real server
  // would send.
  ;(bot as unknown as { dig: (block: unknown, forceLook?: boolean) => Promise<void> }).dig =
    async (block: unknown) => {
      const b = block as { position: Vec3 }
      bot.digs.push(b.position.clone())
      await new Promise(resolve => setTimeout(resolve, 15))
      const oldBlock = bot.blockAt(b.position)
      world.set(b.position.x, b.position.y, b.position.z, AIR_STATE)
      const newBlock = bot.blockAt(b.position)
      bot.emit('diggingCompleted', newBlock)
      bot.emit('blockUpdate', oldBlock, newBlock)
    }
  ;(bot as unknown as { equip: (item: unknown, dest: string) => Promise<void> }).equip = async () => {}
  ;(bot as unknown as { stopDigging: () => void }).stopDigging = () => {}
  bot.digs = []

  let vy = 0
  bot.tick = () => {
    if (!bot.frozen) {
      const e = bot.entity
      const speed = bot.controlState.sprint ? SPRINT_SPEED : WALK_SPEED
      let dx = 0
      let dz = 0
      if (bot.controlState.forward) {
        dx = -Math.sin(e.yaw) * speed
        dz = -Math.cos(e.yaw) * speed
      }

      // Bubble-column / water sensing at the feet cell (crude mirror of
      // prismarine-physics: columns count as water; drag=false pushes up,
      // drag=true pulls down, and holding jump adds enough swim-up
      // acceleration to overcome the down-drag — the vanilla stall).
      // Like prismarine-physics, the whole bounding box feels the column —
      // sampling feet and feet+1 covers standing on soul sand (0.875 tall)
      // at the bottom of a column.
      const cxF = Math.floor(e.position.x)
      const cyF = Math.floor(e.position.y + 0.001)
      const czF = Math.floor(e.position.z)
      const classify = (cy: number): { dir: number, water: boolean } => {
        const state = world.stateAt(cxF, cy, czF)
        if (state === null) return { dir: 0, water: false }
        const b = Block.fromStateId(state, 0) as { name: string, getProperties: () => Record<string, unknown> }
        if (b.name === 'bubble_column') {
          const drag = b.getProperties().drag
          return { dir: (drag === true || drag === 'true') ? -1 : 1, water: true }
        }
        return { dir: 0, water: b.name === 'water' }
      }
      let cls = classify(cyF)
      if (cls.dir === 0 && !cls.water) cls = classify(cyF + 1)
      const bubbleDir = cls.dir
      e.isInWater = cls.water

      // Climbable sensing (ladder/vine at the feet cell). Like
      // prismarine-physics, ascent happens ONLY via horizontal collision
      // while pressing forward; otherwise the bot slides down slowly.
      let onClimbable = false
      {
        const s = world.stateAt(cxF, cyF, czF)
        if (s !== null) {
          const n = (Block.fromStateId(s, 0) as { name: string }).name
          onClimbable = n === 'ladder' || n === 'vine'
        }
      }

      // Jump.
      if (bot.controlState.jump && e.onGround && bubbleDir === 0) {
        vy = JUMP_VY
        e.onGround = false
      }

      // Horizontal move with per-axis voxel collision. The 0.3 leading edge
      // models the player's body half-width: like real physics, the CENTER
      // stops ~0.3 short of a wall face (so pressing into a wall keeps the
      // bot within the executor's 0.35 node-arrival radius).
      const feetY = Math.floor(e.position.y + 0.001)
      const lead = (v: number, from: number): number => v + Math.sign(v - from) * 0.3
      const tryAxis = (nx2: number, nz2: number): boolean =>
        !blocked(world, Math.floor(lead(nx2, e.position.x)), feetY, Math.floor(lead(nz2, e.position.z)))
      const nx = e.position.x + dx
      const nz = e.position.z + dz
      let collidedH = false
      if (tryAxis(nx, nz)) {
        e.position.x = nx
        e.position.z = nz
      } else if (tryAxis(nx, e.position.z)) {
        e.position.x = nx
        collidedH = true
      } else if (tryAxis(e.position.x, nz)) {
        e.position.z = nz
        collidedH = true
      } else {
        collidedH = dx !== 0 || dz !== 0
      }

      // Vertical.
      if (bubbleDir === 1) {
        vy = Math.min(0.5, vy + 0.1)
      } else if (bubbleDir === -1) {
        vy = Math.max(-0.3, vy - 0.05)
        if (bot.controlState.jump) vy += 0.08 // swim-up beats the down-drag
      } else if (onClimbable) {
        if (collidedH && bot.controlState.forward) vy = 0.2 // collision climb
        else vy = Math.max(-0.15, vy - GRAVITY) // slow ladder slide
      } else {
        vy -= GRAVITY
        vy *= 0.98
      }
      let ny = e.position.y + vy
      const cx = Math.floor(e.position.x)
      const cz = Math.floor(e.position.z)
      // Find support: the highest collision top at or below the feet.
      let support = -Infinity
      for (let sy = Math.floor(ny); sy >= world.box.y0 - 1; sy--) {
        const top = solidTop(world, cx, sy, cz)
        if (top > 0) {
          support = sy + top
          break
        }
      }
      if (vy <= 0 && ny <= support) {
        ny = support
        vy = 0
        e.onGround = true
      } else {
        e.onGround = false
      }
      e.position.y = ny
      e.velocity.set(dx, vy, dz)
    }
    bot.emit('physicsTick')
  }

  bot.run = async (ticks, everyMs = 2) => {
    for (let i = 0; i < ticks; i++) {
      bot.tick()
      await new Promise(resolve => setTimeout(resolve, everyMs))
    }
  }

  return bot
}

/**
 * Deterministic geometry-based physics stub for the executor's sprint/jump
 * decisions (replaces the prismarine-physics simulation in tests).
 */
export function makeFakePhysics (world: VoxelWorld, bot: DriveableBot): PhysicsLike {
  const flatTo = (target: XYZ): boolean => {
    const p = bot.entity.position
    // Straight-line OK when the next node isn't above us (no jump needed).
    return target.y <= Math.floor(p.y + 0.001)
  }
  return {
    canStraightLine: (path: XYZ[]) => flatTo(path[0]),
    canSprintJump: (path: XYZ[]) => !flatTo(path[0]),
    canWalkJump: (path: XYZ[]) => !flatTo(path[0]),
    canStraightLineBetween: () => false
  }
}
