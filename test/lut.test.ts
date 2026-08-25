// buildLut classification tests over the real 1.21.1 registry via the fake
// bot: flag bits per block type/state (upstream-quirk parity: carpets are
// SAFE+PHYSICAL, doors are PHYSICAL, fences are neither) and quantized
// collision-top heights.
import { expect } from 'chai'
import {
  VoxelWorld,
  makeFakeBot,
  makeOurMovements,
  lutFor,
  mcData,
  Block,
  AIR,
  STONE,
  WATER,
  LAVA,
  OAK_FENCE,
  LADDER,
  COBWEB
} from './helpers/voxelWorld.js'
import type { BlockLut } from '../src/lut.js'

// LutFlags is a `const enum` (src/types.ts) — inlined at compile time, so we
// re-declare the bit values here. These MUST match src/types.ts LutFlags.
const SAFE = 1
const PHYSICAL = 2
const LIQUID = 4
const CLIMBABLE = 8
const OPENABLE_GATE = 16
const DOOR_CLOSED = 32
const DOOR_OPEN = 64
const GATE_OPEN = 128

interface BlockTypeInfo {
  id: number
  name: string
  minStateId: number
  maxStateId: number
}

function typeOf (name: string): BlockTypeInfo {
  const b = (mcData.blocksByName as Record<string, BlockTypeInfo | undefined>)[name]
  if (!b) throw new Error(`block type not in 1.21.1 registry: ${name}`)
  return b
}

function statesOf (name: string): number[] {
  const t = typeOf(name)
  const out: number[] = []
  for (let s = t.minStateId; s <= t.maxStateId; s++) out.push(s)
  return out
}

describe('buildLut', () => {
  let lut: BlockLut

  before(() => {
    const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 4, z1: 2 })
    world.fill(-2, 0, -2, 2, 0, 2, STONE)
    const bot = makeFakeBot(world)
    const movements = makeOurMovements(bot)
    lut = lutFor(bot, movements)
  })

  describe('basic type flags', () => {
    it('stone is PHYSICAL and not SAFE', () => {
      expect(lut.flags[STONE] & PHYSICAL).to.not.equal(0)
      expect(lut.flags[STONE] & SAFE).to.equal(0)
    })

    it('air is SAFE and not PHYSICAL', () => {
      expect(lut.flags[AIR] & SAFE).to.not.equal(0)
      expect(lut.flags[AIR] & PHYSICAL).to.equal(0)
    })

    it('water is SAFE and LIQUID', () => {
      expect(lut.flags[WATER] & SAFE).to.not.equal(0)
      expect(lut.flags[WATER] & LIQUID).to.not.equal(0)
    })

    it('lava is LIQUID but not SAFE (blocksToAvoid)', () => {
      expect(lut.flags[LAVA] & LIQUID).to.not.equal(0)
      expect(lut.flags[LAVA] & SAFE).to.equal(0)
    })

    it('cobweb is not SAFE (blocksToAvoid)', () => {
      expect(lut.flags[COBWEB] & SAFE).to.equal(0)
    })

    it('ladder is CLIMBABLE and SAFE', () => {
      expect(lut.flags[LADDER] & CLIMBABLE).to.not.equal(0)
      expect(lut.flags[LADDER] & SAFE).to.not.equal(0)
    })

    it('oak_fence is neither SAFE nor PHYSICAL (fence set strips PHYSICAL)', () => {
      expect(lut.flags[OAK_FENCE] & SAFE).to.equal(0)
      expect(lut.flags[OAK_FENCE] & PHYSICAL).to.equal(0)
    })

    it('white_carpet is SAFE and PHYSICAL (upstream carpet quirk)', () => {
      const carpet = typeOf('white_carpet').minStateId
      expect(lut.flags[carpet] & SAFE).to.not.equal(0)
      expect(lut.flags[carpet] & PHYSICAL).to.not.equal(0)
    })
  })

  describe('fence gates', () => {
    it('oak_fence_gate: states with shapes get OPENABLE_GATE, shapeless (open) states get GATE_OPEN', () => {
      const states = statesOf('oak_fence_gate')
      expect(states.length).to.be.greaterThan(1)
      let sawClosed = 0
      let sawOpen = 0
      for (const stateId of states) {
        const block = Block.fromStateId(stateId, 0) as {
          shapes: number[][]
          getProperties: () => Record<string, unknown>
        }
        const hasShapes = (block.shapes ?? []).length !== 0
        const flagsByte = lut.flags[stateId]
        if (hasShapes) {
          sawClosed++
          expect(flagsByte & OPENABLE_GATE, `state ${stateId} (closed) missing OPENABLE_GATE`).to.not.equal(0)
          expect(flagsByte & GATE_OPEN, `state ${stateId} (closed) must not have GATE_OPEN`).to.equal(0)
        } else {
          sawOpen++
          const props = block.getProperties()
          const open = props.open === true || props.open === 'true'
          expect(open, `shapeless gate state ${stateId} should be an open state`).to.equal(true)
          expect(flagsByte & GATE_OPEN, `state ${stateId} (open) missing GATE_OPEN`).to.not.equal(0)
          expect(flagsByte & OPENABLE_GATE, `state ${stateId} (open) must not have OPENABLE_GATE`).to.equal(0)
        }
      }
      expect(sawClosed).to.be.greaterThan(0)
      expect(sawOpen).to.be.greaterThan(0)
    })
  })

  describe('doors', () => {
    it('oak_door: closed states get DOOR_CLOSED, open states get DOOR_OPEN', () => {
      const states = statesOf('oak_door')
      let sawClosed = 0
      let sawOpen = 0
      for (const stateId of states) {
        const block = Block.fromStateId(stateId, 0) as { getProperties: () => Record<string, unknown> }
        const props = block.getProperties()
        const open = props.open === true || props.open === 'true'
        const flagsByte = lut.flags[stateId]
        if (open) {
          sawOpen++
          expect(flagsByte & DOOR_OPEN, `state ${stateId} (open) missing DOOR_OPEN`).to.not.equal(0)
          expect(flagsByte & DOOR_CLOSED, `state ${stateId} (open) must not have DOOR_CLOSED`).to.equal(0)
        } else {
          sawClosed++
          expect(flagsByte & DOOR_CLOSED, `state ${stateId} (closed) missing DOOR_CLOSED`).to.not.equal(0)
          expect(flagsByte & DOOR_OPEN, `state ${stateId} (closed) must not have DOOR_OPEN`).to.equal(0)
        }
      }
      expect(sawClosed).to.be.greaterThan(0)
      expect(sawOpen).to.be.greaterThan(0)
    })

    it('oak_door: every state is PHYSICAL (upstream type-level boundingBox quirk)', () => {
      for (const stateId of statesOf('oak_door')) {
        expect(lut.flags[stateId] & PHYSICAL, `state ${stateId} not PHYSICAL`).to.not.equal(0)
      }
    })

    it('iron_door: no DOOR_CLOSED/DOOR_OPEN flags on any state', () => {
      for (const stateId of statesOf('iron_door')) {
        expect(lut.flags[stateId] & (DOOR_CLOSED | DOOR_OPEN), `state ${stateId} has a door flag`).to.equal(0)
      }
    })
  })

  describe('heights (collision top in 1/32 blocks, bits 0–5; top-catch class in bits 6–7)', () => {
    const height = (stateId: number): number => lut.heights[stateId] & 63
    const catchClass = (stateId: number): number => lut.heights[stateId] >> 6

    it('stone_slab bottom state → 16 (0.5 * 32), full-width catch', () => {
      const bottomStates = statesOf('stone_slab').filter(stateId => {
        const props = (Block.fromStateId(stateId, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
        return props.type === 'bottom'
      })
      expect(bottomStates.length).to.be.greaterThan(0)
      for (const stateId of bottomStates) {
        expect(height(stateId), `state ${stateId}`).to.equal(16)
        expect(catchClass(stateId), `state ${stateId}`).to.equal(0)
      }
    })

    it('oak_fence → 48 (1.5 * 32), post-class catch on every state (arms or not)', () => {
      expect(height(OAK_FENCE)).to.equal(48)
      for (const stateId of statesOf('oak_fence')) {
        expect(height(stateId), `state ${stateId}`).to.equal(48)
        expect(catchClass(stateId), `state ${stateId}`).to.equal(3)
      }
    })

    it('creeper_head → 16 (0.5 * 32), head-class catch', () => {
      const head = typeOf('creeper_head').minStateId
      expect(height(head)).to.equal(16)
      expect(catchClass(head)).to.equal(1)
    })

    it('bottom stairs are marked STAIR in the special grid, top-half stairs are not', () => {
      // The extended-parkour LUT (allowParkourExtended) carries the special
      // grid; the walk-only lut used elsewhere in this file does not, so build
      // one here.
      const world = new VoxelWorld({ x0: -2, y0: 0, z0: -2, x1: 2, y1: 4, z1: 2 })
      world.fill(-2, 0, -2, 2, 0, 2, STONE)
      const bot = makeFakeBot(world)
      const m = makeOurMovements(bot, { allowParkourExtended: true })
      const extLut = lutFor(bot, m)
      const STAIR = 16
      let bottom = 0
      let top = 0
      for (const stateId of statesOf('oak_stairs')) {
        const props = (Block.fromStateId(stateId, 0) as { getProperties: () => Record<string, unknown> }).getProperties()
        if (props.waterlogged === true || props.waterlogged === 'true') continue
        const marked = (extLut.special![stateId] & STAIR) !== 0
        if (props.half === 'bottom') { bottom++; expect(marked, `bottom ${stateId}`).to.equal(true) } else { top++; expect(marked, `top ${stateId}`).to.equal(false) }
      }
      expect(bottom).to.be.greaterThan(0)
      expect(top).to.be.greaterThan(0)
    })

    it('soul_sand → 28 (0.875 * 32)', () => {
      const soulSand = typeOf('soul_sand').minStateId
      expect(height(soulSand)).to.equal(28)
      expect(catchClass(soulSand)).to.equal(0)
    })

    it('air → 0', () => {
      expect(lut.heights[AIR]).to.equal(0)
    })
  })
})
