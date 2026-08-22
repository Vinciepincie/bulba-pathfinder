// fastEvaluator must be formula-identical to the goal classes it shortcuts.
import { expect } from 'chai'
import { Vec3 } from 'vec3'
import { fastEvaluator } from '../src/fastEvaluator.js'
import { serializeGoal } from '../src/goalSerde.js'
import { GoalAdapter } from '../src/goalAdapter.js'
import { GoalBlock, GoalNear, GoalXZ, GoalNearXZ, GoalY, GoalGetToBlock, GoalFollow } from '../src/goals.js'
import type { Goal } from '../src/goals.js'

function mulberry32 (seed: number): () => number {
  let a = seed >>> 0
  return function () {
    a |= 0
    a = (a + 0x6D2B79F5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

describe('fastEvaluator', () => {
  const goals: Array<[string, Goal]> = [
    ['GoalBlock', new GoalBlock(7, -3, 11)],
    ['GoalNear', new GoalNear(-5, 4, 9, 3)],
    ['GoalXZ', new GoalXZ(13, -8)],
    ['GoalNearXZ', new GoalNearXZ(-2, 6, 4)],
    ['GoalY', new GoalY(12)],
    ['GoalGetToBlock', new GoalGetToBlock(3, 2, -7)],
    ['GoalFollow', new GoalFollow({ position: new Vec3(4.7, 1.2, -3.9) }, 2)]
  ]

  for (const [name, goal] of goals) {
    it(`${name}: identical heuristic + isEnd on 500 random nodes`, () => {
      const descriptor = serializeGoal(goal)
      expect(descriptor).to.not.equal(null)
      const fast = fastEvaluator(descriptor!)
      expect(fast, `${name} should have a fast evaluator`).to.not.equal(null)
      const adapter = new GoalAdapter(goal)
      const rand = mulberry32(777)
      for (let i = 0; i < 500; i++) {
        const x = Math.floor(rand() * 81) - 40
        const y = Math.floor(rand() * 81) - 40
        const z = Math.floor(rand() * 81) - 40
        expect(fast!.heuristic(x, y, z)).to.equal(adapter.heuristic(x, y, z), `${name} heuristic at ${x},${y},${z}`)
        expect(fast!.isEnd(x, y, z)).to.equal(adapter.isEnd(x, y, z), `${name} isEnd at ${x},${y},${z}`)
      }
    })
  }

  it('returns null for goal types that need world access or composition', () => {
    expect(fastEvaluator({ type: 'lookAt', x: 0, y: 0, z: 0, reach: 4.5, entityHeight: 1.6 })).to.equal(null)
    expect(fastEvaluator({ type: 'invert', goal: { type: 'block', x: 0, y: 0, z: 0 } })).to.equal(null)
    expect(fastEvaluator({ type: 'compositeAny', goals: [] })).to.equal(null)
  })
})
