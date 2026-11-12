// A small host interface keeps actions replaceable without depending on the whole plugin.
import type { Vec3 } from 'vec3'
import { Pacer } from './pacing.js'
import type { ActionConfig, BlockLike } from './types.js'
import type { BotLike } from './reach.js'

export interface DiggingBot extends BotLike {
  dig: (block: unknown, forceLook?: unknown, digFace?: unknown) => Promise<void>
  stopDigging?: () => void
  placeBlock: (referenceBlock: unknown, faceVector: Vec3) => Promise<void>
  activateBlock: (block: unknown, direction?: Vec3, cursorPos?: Vec3) => Promise<void>
  openBlock?: (block: unknown, direction?: Vec3, cursorPos?: Vec3) => Promise<unknown>
  equip: (item: unknown, destination?: string) => Promise<void>
  swingArm?: (hand?: string) => void
  closeWindow?: (window: unknown) => void
  currentWindow?: unknown
  heldItem?: { type: number, name?: string } | null
  inventory: { items: () => Array<{ type: number, name?: string, nbt?: unknown }>, emptySlotCount?: () => number }
  on: (event: string, listener: (...args: unknown[]) => void) => unknown
  removeListener: (event: string, listener: (...args: unknown[]) => void) => unknown
  once?: (event: string, listener: (...args: unknown[]) => void) => unknown
  _client?: { write: (name: string, params: Record<string, unknown>) => void }
}

export interface ActionContext {
  bot: DiggingBot
  config: ActionConfig
  pacer: Pacer
  /** Original bot.dig captured at injection; bypasses application overrides that could recurse. */
  rawDig: (block: unknown, forceLook?: unknown, digFace?: unknown) => Promise<void>
  /** ms since the server last corrected the bot's position, or Infinity. */
  msSinceForcedMove: () => number
  /** Server TPS estimate, or null when unknown. */
  serverTps: () => number | null
  /** Fastest tool in the inventory for this block, or null. */
  bestHarvestTool: (block: BlockLike) => unknown
  /** Approach an interactable position using the plugin's goal; false when no path exists. */
  approach: (pos: Vec3, reach: number, signal?: AbortSignal) => Promise<boolean>
  /** Equip through the action table, so an override of `actions.equip` covers every action. */
  equip: (item: unknown, destination?: string) => Promise<void>
}
