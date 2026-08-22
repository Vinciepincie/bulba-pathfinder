// Shared paths / defaults for the real-world benchmark arena.
//
// Everything generated (Paper jar, trimmed world, server configs, results)
// lives under `.run/` and is gitignored: the arena is reproducible from the
// scripts plus a world download, never committed.
import { dirname, join, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

export const ARENA_DIR = dirname(fileURLToPath(import.meta.url))
export const RUN_DIR = process.env.ARENA_RUN_DIR
  ? resolve(process.env.ARENA_RUN_DIR)
  : join(ARENA_DIR, '.run')

export const WORLD_NAME = 'arena'
export const WORLD_DIR = join(RUN_DIR, WORLD_NAME)
export const RESULTS_DIR = join(RUN_DIR, 'results')
export const ROUTES_FILE = join(ARENA_DIR, 'routes.json')

/**
 * Server version. 1.21.11 is the newest protocol mineflayer speaks
 * (minecraft-data 3.111.0) and is what the production bot runs, so the bots
 * connect natively — no translation layer between the thing under test and
 * the server. Human spectators on a newer client join through ViaVersion.
 */
export const MC_VERSION = process.env.ARENA_MC_VERSION ?? '1.21.11'

/**
 * Minecraft version the Paper jar is fetched for. Kept separate from
 * MC_VERSION (the protocol the bots speak) because Paper is only used for the
 * one-time DataFixer upgrade, and its build numbering is its own.
 */
export const PAPER_VERSION = process.env.ARENA_PAPER_VERSION ?? '1.21.11'
export const PORT = Number(process.env.ARENA_PORT ?? 25599)
export const HOST = process.env.ARENA_HOST ?? '127.0.0.1'

/** Blocks from 0,0 kept by the trimmer (the arena half-extent). */
export const ARENA_RADIUS = Number(process.env.ARENA_RADIUS ?? 750)

export const BOT_UPSTREAM = 'PF_Upstream'
export const BOT_BULBA = 'PF_Bulba'
export const BOT_BULBA_WASM = 'PF_Bulba_wasm'
export const BOT_REFEREE = 'PF_Referee'
export const BOT_NAMES = [BOT_UPSTREAM, BOT_BULBA, BOT_BULBA_WASM, BOT_REFEREE]

export const JAR_PATH = join(RUN_DIR, `paper-${PAPER_VERSION}.jar`)
export const SERVER_HEAP = process.env.ARENA_HEAP ?? '4G'
/** How far the server streams chunks — must comfortably cover a whole route. */
export const VIEW_DISTANCE = Number(process.env.ARENA_VIEW_DISTANCE ?? 16)

/**
 * Which server implementation to run: `pumpkin` (Rust, boots in
 * milliseconds) or `paper` (Java). Paper is still needed once for
 * `arena:server --upgrade`, which uses Minecraft's DataFixerUpper.
 */
export const SERVER_KIND = (process.env.ARENA_SERVER ?? 'paper') as 'pumpkin' | 'paper'
