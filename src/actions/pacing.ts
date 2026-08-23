// Break/place pacing.
//
// A bot that breaks blocks the instant each dig resolves produces a perfectly
// regular packet train at a rate no human hand makes. Server-side FastBreak /
// FastPlace checks look at exactly that: the interval between consecutive
// block actions, and how little it varies. Vanilla itself enforces a 5-tick
// `destroyDelay` between breaks for a held mouse button, so anything faster is
// already outside what a client produces.
//
// So breaks and placements share ONE jittered cooldown window, and every
// fourth-or-so action takes a longer breather. Sharing the window matters:
// alternating break/place at full speed is the same packet rate as breaking at
// full speed, and a checker that counts block actions does not care which kind
// they were.
//
// Profiles:
//   'none'    — no spacing at all. Correct for single interactions and for
//               servers you control.
//   'default' — conservative, for sustained work next to other players.
//   'drill'   — fast but still above vanilla's floor, for bulk clearing.
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
    // Strictly above vanilla's 5-tick destroyDelay — fast, never impossible.
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

  /** Reset the burst counter — e.g. when the bot switches to another task. */
  reset (): void {
    this.actionsThisBurst = 0
  }

  /**
   * Wait out the cooldown owed after the action that just completed. Call it
   * AFTER the break/placement lands, so the wait overlaps nothing.
   */
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

/**
 * Scale a dig-time estimate for how far behind the server is. mineflayer's
 * dig timer is derived from client-side block hardness; on a server running
 * below 20 TPS the break takes proportionally longer in wall-clock, and a
 * timer that fires early makes the bot think the dig failed.
 */
export function tpsScale (tps: number | null | undefined): number {
  if (typeof tps !== 'number' || !isFinite(tps) || tps <= 0) return 1
  return Math.min(4, Math.max(1, 20 / Math.max(2, tps)))
}
