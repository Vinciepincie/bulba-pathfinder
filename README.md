Vibe coded slop. WIP not recommended for production yet.
---
# @bulba/pathfinder

A drop-in replacement for [`mineflayer-pathfinder`](https://github.com/PrismarineJS/mineflayer-pathfinder):
**same API, same outcome, no compromises — only improvements.**
The A* solver runs over a typed-array world snapshot in a `worker_thread` —
with a Rust/wasm core for the hot goal types and the JS solver as the
always-available reference implementation — so pathfinding compute is orders
of magnitude faster than upstream **and can never block your bot's event
loop**: the failure class where an unreachable goal freezes the process for
minutes is structurally impossible here.

Digging is **opt-in** (`movements.canDig = true`; default **off**, unlike
upstream) with the exact upstream cost model when enabled. Block placement
is never performed. With digging off, the only block interaction the planner
ever performs is activating a door or fence gate on the path.

```js
import { pathfinder, Movements, goals } from '@bulba/pathfinder' // named ESM imports…
// const { pathfinder, Movements, goals } = require('@bulba/pathfinder') // …or CJS
// import pkg from '@bulba/pathfinder'  // …or the upstream default-import style

bot.loadPlugin(pathfinder)
const movements = new Movements(bot)
bot.pathfinder.setMovements(movements)
await bot.pathfinder.goto(new goals.GoalNear(x, y, z, 1))
```

## API compatibility

The full upstream surface is provided with identical semantics:

- **`bot.pathfinder`**: `goto(goal)`, `setGoal(goal, dynamic?)`, `setMovements(m)`,
  `stop()`, `isMoving()`, `isMining()`, `isBuilding()`, `bestHarvestTool(block)`,
  `getPathTo(...)`, `getPathFromTo(...)` (synchronous generator, upstream-identical),
  `thinkTimeout`, `tickTimeout`, `searchRadius` (cost-slack semantics),
  `enablePathShortcut`, `LOSWhenPlacingBlocks`, and the `goal` / `movements` getters.
- **`goto()` rejections** use the verbatim upstream error names and messages:
  `NoPath` / `Timeout` / `GoalChanged` / `PathStopped` — code that string-matches
  them keeps working byte-for-byte.
- **Events on the bot**: `path_update` (`{ status, cost, time, visitedNodes,
  generatedNodes, path, context }`), `goal_reached`, `path_reset`, `goal_updated`,
  `path_stop` — same payload shapes, same ordering rules. Like upstream, the
  emitted `path` array is the live path and drains as the bot walks.
- **`Movements`**: every upstream field exists with the same meaning
  (`allowSprinting`, `allowParkour`, `canOpenDoors`, `maxDropDown`, `liquidCost`,
  `entityCost`, `blocksToAvoid`, `climbables`, `carpets`, `fences`, exclusion
  areas, entity avoidance, …). The cost model — move costs, probe order,
  thresholds, even upstream's classification quirks — is mirrored exactly, and a
  differential fuzz suite runs upstream as the oracle to prove it.
- **`goals`**: `Goal`, `GoalBlock`, `GoalNear`, `GoalXZ`, `GoalNearXZ`, `GoalY`,
  `GoalGetToBlock`, `GoalLookAtBlock`, `GoalBreakBlock`, `GoalCompositeAny`,
  `GoalCompositeAll`, `GoalInvert`, `GoalFollow`, `GoalPlaceBlock` — identical
  heuristics and end conditions. Custom goal classes work too (they solve on the
  main thread, tick-sliced exactly like upstream).
- **Path following** is a port of upstream's tick loop, driven by the same
  prismarine-physics simulations for sprint/jump decisions — movement on the
  server is indistinguishable (see *Walking the server accepts* below for the
  three places it is deliberately more careful).

### Deliberate divergences (all walk-only-safety or strict improvements)

| Divergence | Why |
|---|---|
| `Movements.canDig` defaults to **false** (upstream: true) | A pathfinder that silently digs is how bots get banned — digging is a deliberate opt-in. When enabled, the cost model (dig times, `dontCreateFlow`, `dontMineUnderFallingBlock`, tool selection) is upstream-identical and differentially fuzz-tested. |
| `allow1by1towers`/`scafoldingBlocks` are inert (warned once, ignored) | No placement moves exist — digging yes, placing no. |
| `isBuilding()` is only `true` while activating a door/gate | No placement executor branch. |
| `Movements.getBlock` / `safeToBreak` / `safeOrBreak` are provided for API compatibility, but the per-move generators (`getNeighbors`, `getMoveForward`, …, `getLandingBlock`) are **not** — the solver's move generation runs against the snapshot, not live blocks | Niche internals; open an issue if a real consumer needs them. |

## Improvements over upstream

- **Never blocks the event loop.** Solves run in one shared `worker_thread`
  per process against a `SharedArrayBuffer` snapshot; cancellation is a shared
  flag checked between expansion batches, so a stale solve dies mid-flight
  without thread churn. If worker startup fails (exotic bundler, permissions),
  everything transparently falls back to main-thread tick-sliced solving —
  which is still upstream-identical in scheduling, just much faster per slice.
- **The 1.21.x hitbox-precision fix, applied on inject.** prismarine-physics
  builds the body from `playerHalfWidth` 0.3 and `playerHeight` 1.8, and every
  collision resolution leaves it exactly on a block boundary. On 1.21.x the
  server's own sweep then computes exactly 1.0 for the next move, calls it
  blocked, and teleports the client back — silently, every tick, for as long
  as the bot keeps producing that position. It is a physics bug
  ([mineflayer#3911](https://github.com/PrismarineJS/mineflayer/issues/3911)),
  but it presents as a pathfinder that cannot climb a one-block step. Measured
  on the arena's climb1 riser, walking jump, only the starting clearance
  changed:

  | clearance | stock 0.3 / 1.8 | nudged 0.30001 / 1.80001 |
  |---|---|---|
  | 0.01 | stuck, 19 corrections | **climbed, 0** |
  | 0.15 | stuck, 17 corrections | **climbed, 0** |
  | 0.20 | climbed, 0 | climbed, 0 |
  | 0.30, sprinting | stuck, 7 corrections | **climbed, 0** |

  Nudging both dimensions by 1e-5 breaks the alignment and the whole class
  goes away. On the arena's 46-block cobble climb that is 50.4 s → 24.9 s, and
  two routes that never finished at all now do. Guarded on the exact stock
  values (an application that already applies the nudge is not doubled up) and
  switchable with `hitboxPrecisionFix: false`. `geometry.ts`'s body probes read
  the same dimensions back off `bot.physics`, so they never disagree with the
  engine they are predicting.
- **Walking the server accepts.** Even with that fixed, prismarine-physics
  will produce positions a real server refuses, and a refused position is a
  teleport back — every tick, for as long as the bot keeps producing it, which
  is a livelock the futility timer cannot break because each correction looks
  like progress. These rules keep the bot out of that state, all of them
  measured against a real 1.21.11 server (`bench/arena`, which counts
  corrections per engine):
  - a jump is only taken if **one** jump plus the run-in reaches the node.
    Holding jump for the whole rollout — upstream's model — lets a take-off
    that lands short bounce on and satisfy the node from a cell the planner
    never routed through; on 2b2t spawn that put the bot in a 1-block pit two
    cells short of its landing, where it sat for the rest of the run.
  - **sprint is dropped against a wall**, the way a vanilla client drops it on
    any collision that deflects it. Measured back to back on one block:
    walking along the wall covered 7.1 blocks with zero corrections, sprinting
    covered 0.0 with 26.
  - a walking step whose straight line runs into a wall **steers along the
    wall** with a hair of standoff, instead of grinding on the block face the
    physics would clamp it onto. Aiming into the corner: 25 corrections and
    0.01 blocks. Aiming along it: none, and 7.1 blocks. (With nothing to slide
    along — a step square-on to the face — the steer is skipped rather than
    replacing the heading with float noise.)
  - a **wedge recovery** that triggers on the symptom rather than on a
    prediction. Every gate above answers "does this work?" against
    prismarine-physics, and prismarine-physics is not the authority; when the
    two disagree the bot stands still with a plan it believes in, the futility
    timer replans, the plan comes back identical, and it stands still again.
    So when the body has stopped moving the executor stops trusting its
    rollouts and tries the escapes a player would — a step back, a step
    sideways — each simulated first so it cannot become the fall it was
    avoiding, and cycled per node so an escape that did not help is not the
    one tried next time.
  - a **diagonal squeeze** goes round the corner that is open. A diagonal move
    is priced on the cheaper of its two corners (upstream's rule, and ours by
    parity), which is honest only if the walker goes around that corner;
    aiming at the node centre cuts across both and a 0.6-wide body clips the
    blocked one. The open corner is inserted as a waypoint and the squeeze
    becomes the two ordinary moves the planner actually priced.
- **Much faster compute.** Block classification is precomputed into a
  per-blockstate LUT; the world is snapshotted into flat typed arrays; the A*
  core uses packed integer node ids, epoch-stamped g/parent tables and a
  typed-array binary heap — zero allocation per expanded node. Coordinate
  goals get monomorphic evaluators (no per-node virtual dispatch).
- **Rust/wasm core for the hot path.** Coordinate goals (Block/Near/XZ/
  NearXZ/Y/GetToBlock/Follow, plus `GoalCompositeAny` over any mix of them —
  i.e. nearly every real solve) run on a dependency-free Rust A* compiled to
  wasm (~50 KB, embedded — consumers never need a Rust toolchain). The core
  keeps state resident between solves: snapshot grids stay in wasm memory
  keyed by `(generation, patchCount)` and are only re-copied when a block
  actually changed, dig tables upload once per inventory/effects
  fingerprint, and the A* arena is epoch-stamped so repeat solves skip the
  clear entirely. The JS solver is the permanent reference implementation
  and fallback (raycast goals, other composites, exclusion areas, wasm
  unavailable, `PF_NO_WASM=1`), and a differential suite pins the wasm core
  to **bit-identical statuses, costs, paths and visited counts** — including
  residency invalidation (patched snapshots, alternating snapshots,
  dig-table swaps). `npm run build:wasm` regenerates it (needs
  `rustup target add wasm32-unknown-unknown`).
- **Dig tables cross the worker boundary once.** The worker caches dig
  tables by fingerprint (host mirrors the eviction), so repeat `canDig`
  solves stop paying the ~120 KB per-solve table copy.
- **Full digging support, opt-in.** `movements.canDig = true` enables the
  complete upstream dig cost model — best-tool dig times, `dontCreateFlow`,
  `dontMineUnderFallingBlock`, `blocksCantBreak`, dig-aware executor with
  tool equip — but computes dig tables once per (inventory × effects)
  instead of upstream's per-probe nbt parsing. Off by default.
- **Bubble-column elevators, opt-in.** `movements.useBubbleColumns = true`
  teaches the planner the modern-base elevators upstream can't see: soul
  sand columns ride up, magma columns ride down (`movements.bubbleCost` per
  block, default 1), columns catch falls like water (unlimited-height drops
  into a column are legal), floating in a column supports horizontal moves,
  and parkour is refused mid-column. The executor suppresses its usual
  in-water jump while descending so the down-drag isn't fought
  (prismarine-physics simulates both column types natively). Runs in the
  wasm core too — bit-identical to the JS reference. Off by default: with
  the flag off, columns classify exactly as upstream (plain passable air)
  and zero new code runs. Note upstream walks onto magma blocks (damage
  included) and so do we — `movements.blocksToAvoid.add(magmaId)` opts out.
- **Extended parkour, opt-in.** `movements.allowParkourExtended = true`
  (plus `allowParkour` + `allowSprinting`) teaches the planner every
  straightforward sprint-jump a vanilla player can make and upstream cannot:
  **diagonal and long offsets** ((2,1), (2,2), (3,1), (3,2) per quadrant —
  upstream, baritone and azalea are all cardinal-only), **up (+1) and drop
  landings** (to `maxDropDown`), and gap-jumps that **catch a ladder, water
  or a bubble column** mid-column (cardinal offsets get these landing types
  too). Clearance is derived from the actual 0.6-wide hitbox swept along
  the flight line (docs/ExtendedParkour.md), costs sit above both the
  octile heuristic and the walking cost so jumps only appear where walking
  fails, and the executor's live physics simulation still gates every jump
  at run time. Zero-allocation and aggressively early-outed (a flat-ground
  node pays ~2 probes per offset); runs in the wasm core too —
  bit-identical to the JS reference. Off by default: these are moves
  upstream cannot generate, so parity solves must not see them.
- **Vine climbing, auto-enabled on 1.16+.** Upstream ships
  `climbables.add(vine)` commented out; we enable it where vanilla makes
  vines unconditionally climbable (the 1.16 `climbable` tag) — but the
  planner only climbs vine cells with an **adjacent solid block to press
  against**, because vanilla and prismarine-physics ascend climbables via
  horizontal collision: a free-hanging curtain is passable, never climbable
  (and a vine capped by its source block is exited sideways below it, never
  bonked into). The executor steers into the backing wall when the next
  node is directly above, making ladder/vine ascents deterministic. Opt out
  with `movements.climbables.delete(vineId)`. Below 1.16 the upstream
  behavior is kept. Deliberately ladder+vine only: nether/cave vines are in
  the vanilla tag but prismarine-physics cannot climb them.

`npm run bench` (identical worlds/profiles, N=40, Node 22):

| scenario | upstream | ours (warm, JS) | worker e2e (wasm) |
|---|---|---|---|
| short walk (8 blocks) | 0.76 ms | 0.16 ms | 0.13 ms |
| long walk (48 blocks, obstacles) | 19 ms | 0.42 ms | 0.36 ms |
| walled goal → NoPath (radius 64) | 255 ms | 16 ms | 9.1 ms |
| walled goal → NoPath (unbounded, 160×160) | 1588 ms | 32 ms | 14.2 ms |
| serpentine maze | 111 ms | 2.5 ms | 1.4 ms |
| parkour gauntlet (11× 2-gap jumps over void) | 1.9 ms | 0.06 ms | 0.11 ms |
| anarchy spawn (97×97 lavacasts/junk, parkour canyon crossings) | 66 ms | 1.4 ms | 0.89 ms |
| dig tunnel (canDig) | 36 ms | 0.76 ms | 0.51 ms |

The table measures steady state (3 unsampled warmup iterations per cell) —
what a long-lived bot pays per solve. One-time process costs (worker thread
spawn, V8 JIT warmup, wasm instantiation, first LUT transfer) are excluded;
`PF_BENCH_WARMUP=0` includes them, which adds ~3 ms amortized across
whichever scenario runs first and nothing after that. The worker round-trip
itself costs ~0.1 ms and happens off the bot's event loop.

Those are synthetic worlds. `bench/arena/` races both engines through **real
2b2t spawn** on a real Paper server — two bots, two processes, one start
block, released together, with human spectators teleported along to watch.
See [bench/arena/README.md](bench/arena/README.md).
- **Better routes.** Upstream never reopens closed nodes; with parkour edges
  its heuristic is inadmissible and it locks in suboptimal routes. This
  solver reopens on a strictly better g, removing most of that: across the
  500-case differential sweep, costs match upstream exactly in ~96% of
  reachable cases and ours is cheaper in most of the rest (both engines share
  the same first-goal-pop approximation, so a rare few-percent loss remains
  possible — outliers are pinned against Dijkstra ground truth in CI).
- **Walks while thinking.** Long solves stream partial paths (upstream's
  `partial` status semantics) and `keepPathDuringRecompute` (default on) keeps
  following the still-valid prefix while a recompute (moved goal, changed
  world) runs — no stop-start stutter on dynamic goals.
- **Doors actually work.** With `canOpenDoors`, closed non-iron doors are
  opened via activate and open doors/gates are walked through. (Upstream's
  door handling never worked on modern versions: the type-level bounding box
  makes it treat open doors — and open fence gates — as walls. Set
  `movements.canOpenRealDoors = false` for strict upstream reachability.)
- **`stop()` you can trust.** Takes effect within one physics tick, always
  releases every control state, and an idle `stop()` no longer poisons the
  next goal (upstream latches a flag that kills the next `setGoal`).
- **Opt-in watchdogs** (both default off): `bot.pathfinder.stuckTimeout` (ms
  without reaching the next path node → `path_stop`, so `goto` rejects
  `PathStopped`) and `bot.pathfinder.executionTimeout` (wall-clock cap per
  goal). These replace the defensive scaffolding wrappers usually build.
- **Body-geometry toolkit** (absorbed from production wrapper code, additive
  API): `bot.pathfinder.isStuck()`, `canStandAt(pos)`, `findEscape()`,
  `findNearestOpenStandable()`, `unstick()` (walk-only escape when embedded in
  a block), `centreInCell(cell)`, `settle()` (verified standstill), and
  `jumpOnce()`. All shape-aware (real collision AABBs — carpets, slabs, stairs)
  with epsilon comparisons (never trust exact boundary equality).
- **Snapshot freshness without rebuilds.** Block updates patch single snapshot
  cells in place; chunk loads and profile changes invalidate it. A `noPath`
  that pruned at the snapshot boundary transparently retries with a larger
  box before being reported.
- **ESM-first dual package.** Real named exports for ESM, working `require()`
  for CJS, types included.

## Options

```js
import { createPathfinder } from '@bulba/pathfinder'
bot.loadPlugin(createPathfinder({
  useWorkerThreads: true,   // default; false = always main-thread sliced
  maxSnapshotCells: 8_000_000, // snapshot memory cap (× 2 bytes)
  hitboxPrecisionFix: true, // default; see the 1.21.x note above
}))
```

Runtime knobs on `bot.pathfinder` (beyond the upstream trio):
`stuckTimeout` (default −1 = off), `executionTimeout` (−1 = off),
`keepPathDuringRecompute` (default true).

Opt-in `Movements` flags (both default **false**, so the walking outcome
matches upstream until you ask for more): `allowParkourExtended` (the full
sprint-jump repertoire, `docs/ExtendedParkour.md`) and `allowSprintHop`
(hold jump while sprinting across open ground — 6.97 blocks/s against 5.56,
measured on the arena, taken only where a rollout of both gaits down the same
path says the hop gets further without losing height).

## Scope

Water walking, ladders, doors/gates, sprinting, parkour, drop-downs, entity
avoidance and (opt-in) digging are all supported — the full upstream moveset
except block placement, which is permanently out of scope; the package
writes zero place packets by construction, and zero dig packets unless
`canDig` is explicitly enabled.

## Testing

- **Unit floor**: heap, LUT classification, each move generator against
  hand-built scenes, snapshot round-trips.
- **Differential fuzz**: seeded random voxel worlds solved by upstream
  mineflayer-pathfinder (the oracle) and this solver on identical geometry —
  reachability must match exactly, our cost must never exceed upstream's, and
  every path is validated by an independent walkability checker.
- **Executor tests**: the full plugin driving a fake bot through fake voxel
  physics — the `goto` promise contract, events, stop semantics, replanning
  around world changes.
- **Worker round-trip**: real worker_thread solves, cancellation, partial
  streaming (run `npm run build` first).
- **canDig differential**: exact dig-cost parity scenes vs upstream
  (dontCreateFlow/dontMineUnderFallingBlock refusals included), randomized
  dig fuzz with edge-walk arbitration, executor dig-through-wall test.
- **wasm ↔ JS differential**: the Rust core must match the JS reference
  bit-for-bit on statuses, costs, paths, toBreak lists and visited counts.

```sh
npm test          # everything
npm run test:fuzz # widen the fuzz sweep
```

## License

MIT. Portions are derived from
[mineflayer-pathfinder](https://github.com/PrismarineJS/mineflayer-pathfinder)
(MIT) — the cost model, goal classes and tick-loop semantics are intentional
ports to guarantee behavioral parity. Design informed by
[azalea-rs](https://github.com/azalea-rs/azalea) (moves/costs split,
recompute-while-walking) and [Baritone](https://github.com/cabaletta/baritone)
(cost-model ancestry).
