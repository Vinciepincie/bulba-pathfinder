// Minimal WebAssembly ambient types (we compile with lib:ES2022 only — no
// DOM lib — and only need this narrow surface).
declare namespace WebAssembly {
  interface Memory {
    readonly buffer: ArrayBuffer
  }
  interface Instance {
    readonly exports: unknown
  }
  function instantiate (bytes: ArrayBufferView | ArrayBuffer, imports?: object): Promise<{ instance: Instance }>
}

// Minimal ambient types for prismarine-physics (ships no declarations).
declare module 'prismarine-physics' {
  import type { Vec3 } from 'vec3'

  export interface SimControl {
    forward: boolean
    back: boolean
    left: boolean
    right: boolean
    jump: boolean
    sprint: boolean
    sneak: boolean
  }

  export class PlayerState {
    constructor (bot: unknown, control: SimControl)
    pos: Vec3
    vel: Vec3
    yaw: number
    control: SimControl
    onGround: boolean
    isInWater: boolean
    isInLava: boolean
    isCollidedHorizontally: boolean
    isCollidedVertically: boolean
  }
}
