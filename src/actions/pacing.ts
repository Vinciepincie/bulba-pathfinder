// Breaks and placements share a cooldown because servers can count both as block actions.
// Vanilla applies a five-tick destroyDelay between held-button breaks.
import type { PacingProfile } from './types.js'

interface ProfileSpec {
  cooldownMinTicks: number
  cooldownJitterTicks: number
  actionsPerBreather: number
  breatherMinTicks: number
  breatherJitterTicks: number
}

const PROFILES: Record<Exclude<PacingProfile, 'none'>, ProfileSpec> = {
  default: {
    cooldownMinTicks: 12,
    cooldownJitterTicks: 6,
    actionsPerBreather: 4,
    breatherMinTicks: 30,
    breatherJitterTicks: 30
  },
  drill: {
    // Stay above vanilla's five-tick destroyDelay.
    cooldownMinTicks: 6,
    cooldownJitterTicks: 2,
    actionsPerBreather: 10,
    breatherMinTicks: 10,
    breatherJitterTicks: 8
  }
}

/** Per-bot pacing state. One burst counter shared by digs and placements. */
export class Pacer {
  private actionsThisBurst = 0
  private lastActionAt = 0
  private lastTool: string | null = null

  /** ms since the last paced action left the client. */
  msSinceLastAction (): number {
    if (this.lastActionAt === 0) return Number.POSITIVE_INFINITY
    return Date.now() - this.lastActionAt
  }

  /** The item in hand for the last paced action, if it was recorded. */
  get heldForLastAction (): string | null {
    return this.lastTool
  }

  /** Called immediately before the packet goes out. */
  noteAction (tool: string | null = null): void {
    this.lastActionAt = Date.now()
    this.lastTool = tool
  }

  /** Reset the burst counter when switching tasks. */
  reset (): void {
    this.actionsThisBurst = 0
  }

  /** Wait after a confirmed break or placement to preserve action spacing. */
  async cooldown (profile: PacingProfile, waitTicks: (ticks: number) => Promise<void>): Promise<void> {
    if (profile === 'none') return
    const spec = PROFILES[profile]
    this.actionsThisBurst++

    let ticks = spec.cooldownMinTicks + Math.floor(Math.random() * (spec.cooldownJitterTicks + 1))
    if (this.actionsThisBurst % spec.actionsPerBreather === 0) {
      ticks += spec.breatherMinTicks + Math.floor(Math.random() * (spec.breatherJitterTicks + 1))
    }
    await waitTicks(ticks)
  }
}

/** Scale client dig timing for servers below 20 TPS, where breaks take longer in wall-clock time. */
export function tpsScale (tps: number | null | undefined): number {
  if (typeof tps !== 'number' || !isFinite(tps) || tps <= 0) return 1
  return Math.min(4, Math.max(1, 20 / Math.max(2, tps)))
}
