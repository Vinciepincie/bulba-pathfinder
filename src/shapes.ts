// Port of mineflayer-pathfinder/lib/shapes.js (MIT), verbatim math.
import { Vec3 } from 'vec3'

export function getShapeFaceCenters (shapes: number[][], direction: Vec3, half: 'top' | 'bottom' | null = null): Vec3[] {
  const faces: Vec3[] = []
  for (const shape of shapes) {
    const halfsize = new Vec3(shape[3] - shape[0], shape[4] - shape[1], shape[5] - shape[2]).scale(0.5)
    let center = new Vec3(shape[0] + shape[3], shape[1] + shape[4], shape[2] + shape[5]).scale(0.5)
    center = center.offset(halfsize.x * direction.x, halfsize.y * direction.y, halfsize.z * direction.z)

    if (half === 'top' && center.y <= 0.5) {
      if (Math.abs(direction.y) === 0) center.y += halfsize.y - 0.001
      if (center.y <= 0.5) continue
    } else if (half === 'bottom' && center.y >= 0.5) {
      if (Math.abs(direction.y) === 0) center.y -= halfsize.y - 0.001
      if (center.y >= 0.5) continue
    }

    faces.push(center)
  }
  return faces
}
