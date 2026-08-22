// Monomorphic goal evaluators for the hot solver loop. GoalAdapter routes
// every heuristic/isEnd through a Vec3 scratch + a virtual goal method —
// fine for custom goals, wasteful for the built-in coordinate goals whose
// math is a handful of flops. These closures capture the constants directly;
// identical formulas to goals.ts (which stay the source of truth for the
// public API). Returns null for goal types that need world access
// (lookAt/placeBlock) or arbitrary composition — those keep the adapter.
import type { GoalDescriptor } from './types.js'
import type { GoalEvaluator } from './solver.js'

const SQRT2 = Math.SQRT2

function octile (dx: number, dz: number): number {
  const adx = Math.abs(dx)
  const adz = Math.abs(dz)
  return Math.abs(adx - adz) + Math.min(adx, adz) * SQRT2
}

export function fastEvaluator (d: GoalDescriptor): GoalEvaluator | null {
  switch (d.type) {
    case 'block': {
      const gx = d.x as number
      const gy = d.y as number
      const gz = d.z as number
      return {
        heuristic: (x, y, z) => octile(gx - x, gz - z) + Math.abs(gy - y),
        isEnd: (x, y, z) => x === gx && y === gy && z === gz
      }
    }
    case 'near':
    case 'follow': {
      const gx = d.x as number
      const gy = d.y as number
      const gz = d.z as number
      const rangeSq = d.rangeSq as number
      return {
        heuristic: (x, y, z) => octile(gx - x, gz - z) + Math.abs(gy - y),
        isEnd: (x, y, z) => {
          const dx = gx - x
          const dy = gy - y
          const dz = gz - z
          return dx * dx + dy * dy + dz * dz <= rangeSq
        }
      }
    }
    case 'xz': {
      const gx = d.x as number
      const gz = d.z as number
      return {
        heuristic: (x, _y, z) => octile(gx - x, gz - z),
        isEnd: (x, _y, z) => x === gx && z === gz
      }
    }
    case 'nearxz': {
      const gx = d.x as number
      const gz = d.z as number
      const rangeSq = d.rangeSq as number
      return {
        heuristic: (x, _y, z) => octile(gx - x, gz - z),
        isEnd: (x, _y, z) => {
          const dx = gx - x
          const dz = gz - z
          return dx * dx + dz * dz <= rangeSq
        }
      }
    }
    case 'y': {
      const gy = d.y as number
      return {
        heuristic: (_x, y, _z) => Math.abs(gy - y),
        isEnd: (_x, y, _z) => y === gy
      }
    }
    case 'getToBlock': {
      const gx = d.x as number
      const gy = d.y as number
      const gz = d.z as number
      return {
        heuristic: (x, y, z) => {
          const dy = y - gy
          return octile(x - gx, z - gz) + Math.abs(dy < 0 ? dy + 1 : dy)
        },
        isEnd: (x, y, z) => {
          const dy = y - gy
          return Math.abs(x - gx) + Math.abs(dy < 0 ? dy + 1 : dy) + Math.abs(z - gz) === 1
        }
      }
    }
    default:
      return null
  }
}
