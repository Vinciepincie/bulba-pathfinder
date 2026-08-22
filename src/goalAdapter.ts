// Bridges a Goal object to the solver's (x, y, z) evaluator interface with a
// single reused Vec3 scratch node — goals see a real Vec3 (upstream passes
// Move, a Vec3 subclass) but the hot path allocates nothing.
import { Vec3 } from 'vec3'
import type { Goal } from './goals.js'
import type { GoalEvaluator } from './solver.js'

export class GoalAdapter implements GoalEvaluator {
  private readonly scratch = new Vec3(0, 0, 0)
  private readonly goal: Goal

  constructor (goal: Goal) {
    this.goal = goal
  }

  heuristic (x: number, y: number, z: number): number {
    const s = this.scratch
    s.x = x
    s.y = y
    s.z = z
    return this.goal.heuristic(s)
  }

  isEnd (x: number, y: number, z: number): boolean {
    const s = this.scratch
    s.x = x
    s.y = y
    s.z = z
    return this.goal.isEnd(s)
  }
}
