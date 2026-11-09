# @bulba/pathfinder

Pathfinding for [mineflayer](https://github.com/PrismarineJS/mineflayer) bots. Its navigation API follows
[mineflayer-pathfinder](https://github.com/PrismarineJS/mineflayer-pathfinder), so most navigation code moves over by
changing the import ([differences](#differences-from-mineflayer-pathfinder)), and it adds:

- **Search off your event loop.** For the built-in goals, the search runs on a worker thread against a snapshot of the
  world, on a Rust/WebAssembly core with a JavaScript fallback. An unreachable goal costs the worker some time, not your
  tick loop. [Some setups](#when-the-search-runs-on-your-thread) search on your thread instead, in time slices.
- **Parkour checked against the game's physics** (opt-in). Jumps are planned and flown with an exact simulation of the
  player, including diagonal and long jumps, ladder catches, slime bounces, momentum chains and sprint-hopping.
- **Interruptible movement.** Other plugins can borrow the bot at a safe moment, and the path carries on from the same
  node afterwards, so auto-eat stops happening in mid-jump.
- **Guarded interactions.** `dig`, `place`, `open` and `activate` walk into range, aim first, and refuse clicks that the
  server would drop. `dig` and `place` judge the result by the world, after the server has had time to answer.
- **Hooks for your own packages.** Cancellation, typed events, overridable actions, and planning without moving.

> **Status: 0.x.** The API can still change between minor versions. It has been raced on a real server only on Minecraft
> 1.21.11. See [Compatibility](#compatibility).

## Install

```sh
npm install @bulba/pathfinder mineflayer
```

Node 22 or newer (mineflayer needs it), mineflayer 4.37 or newer. There is no native build: the WebAssembly core ships
inside the package.

## Quick start

In an `.mjs` file, or a project with `"type": "module"`:

```js
import { createBot } from 'mineflayer'
import { pathfinder, Movements, goals } from '@bulba/pathfinder'

const bot = createBot({ host: 'localhost', username: 'bot' })
bot.loadPlugin(pathfinder)
bot.once('spawn', () => bot.pathfinder.setMovements(new Movements(bot)))

// Say "come" in chat.
bot.on('chat', async (username, message) => {
  if (message !== 'come') return
  const player = bot.players[username]?.entity
  if (!player) return bot.chat('I cannot see you')
  const { x, y, z } = player.position
  try {
    await bot.pathfinder.goto(new goals.GoalNear(x, y, z, 1))
    bot.chat('here')
  } catch (err) {
    bot.chat(`could not get there: ${err.name}`)
  }
})
```

The bot only knows the chunks the server has sent it, so a goal far outside them cannot be found until it walks closer.

CommonJS works too: `const { pathfinder, Movements, goals } = require('@bulba/pathfinder')`. TypeScript gets
`bot.pathfinder` and the events on `bot.on` as soon as the package is imported.

## Movement profiles

`Movements` says what the bot is allowed to do. Fresh out of the constructor it walks, sprints and jumps like
mineflayer-pathfinder, apart from not digging. Turn the parkour engine on with a preset:

```js
const movements = new Movements(bot).applyPreset('parkour')
bot.pathfinder.setMovements(movements)
```

| Preset | What the bot does |
|---|---|
| `default` | Walks, sprints and makes the basic straight jumps. |
| `parkour` | Everything `default` does, plus physics-verified jumps of every kind, ladder and water catches, and the fastest gait it can find, including sprint-hopping. |

A preset only sets the movement engine's `allow*` flags. `canDig`, `maxDropDown`, the block sets and everything else stay
as they were.

The settings you are most likely to change:

| Setting | Default | Meaning |
|---|---|---|
| `canDig` | `false` | Allow paths through blocks, using the best tool. Off by default, unlike mineflayer-pathfinder. |
| `allowSprinting` | `true` | Sprint where it is safe. |
| `allowParkour` | `true` | Basic gap jumps. |
| `canOpenDoors` | `false` | Walk through doors and fence gates, opening them. |
| `maxDropDown` | `4` | Longest drop a walk or a grid jump may choose, unless it lands in liquid. As in mineflayer-pathfinder, a walk-off counts down to the supporting block, so `4` means falling 3 blocks. An extended jump counts to the feet, so `4` means landing 4 lower. The physics-verified hops of the `parkour` preset are bounded by fall damage instead, so they can drop further, for example onto slime. |
| `infiniteLiquidDropdownDistance` | `true` | Any height is fine into water. |
| `blocksToAvoid` | fire, cobweb, lava | Block ids a path never enters. A `Set`. |
| `blocksCantBreak` | chest, unbreakable blocks | Block ids a digging path leaves alone. A `Set`. |
| `exclusionAreasStep`, `exclusionAreasBreak` | `[]` | Functions from a block to an extra cost. See [below](#when-the-search-runs-on-your-thread). |
| `useBubbleColumns` | `false` | Ride soul-sand and magma bubble columns. |

The individual `allow*` flags behind the `parkour` preset are documented on the
[`Movements` class](src/movements.ts).

## Going places

```js
await bot.pathfinder.goto(goal)                // resolves on arrival
bot.pathfinder.setGoal(goal, true)             // dynamic: follow it as it moves
bot.pathfinder.stop()                          // the goal ends on the next tick; a body in flight still lands
```

`stop()` ends the goal and emits `path_stop` on the next tick. If the bot is in the air it keeps its controls until it
lands, so `path_stop` does not mean it has landed.

Goals, from `goals`:

| Goal | Arrives when the bot |
|---|---|
| `GoalBlock(x, y, z)` | Stands in that block. |
| `GoalNear(x, y, z, range)` | Is within `range` of it. |
| `GoalXZ(x, z)`, `GoalNearXZ(x, z, range)` | Reaches that column, or comes within `range` of it. |
| `GoalY(y)` | Is at that height. |
| `GoalGetToBlock(x, y, z)` | Stands next to it. |
| `GoalLookAtBlock(pos, bot.world, { reach })` | Can see a face of it within reach. |
| `GoalBreakBlock(x, y, z, bot, { reach })` | Can break it from there. |
| `GoalPlaceBlock(pos, bot.world, options)` | Can place a block there. |
| `GoalFollow(entity, range)` | Is within `range` of the entity. Use it with `setGoal(goal, true)`. |
| `GoalInvert(goal)` | Has left the other goal. |
| `GoalCompositeAny(goals)`, `GoalCompositeAll(goals)` | Meets any, or all, of them. |

A subclass of `Goal` works as well.

### Options

```js
await bot.pathfinder.goto(goal, {
  signal: AbortSignal.timeout(30_000), // cancel from outside, here with a deadline
  bestEffort: true                     // an unreachable goal: walk as close as possible and resolve there
})
```

### Errors

`goto` rejects with an error you can tell apart by `name`. Every rejection also ends the goal, so the bot does not keep
walking behind your back:

| `err.name` | Meaning |
|---|---|
| `NoPath` | No route within the world the bot has loaded. Unloaded chunks count as blocked. With `bestEffort`, `goto` resolves instead, without arriving: check where the bot ended up. |
| `Timeout` | No path was found within `thinkTimeout`. `bestEffort` does not cover this. |
| `GoalChanged` | Another goal replaced this one. That goal stays. |
| `PathStopped` | The goal was stopped. `err.reason` says why: `stop` (also `stuckTimeout`, `executionTimeout` and a goal that became invalid, such as a `GoalFollow` whose entity left), `low_health`, `death`, `disconnected`, `disposed` or `unsupported`. |
| `AbortError` | The `signal` fired. A signal aborted with an `Error` as its reason rejects with that error, so `AbortSignal.timeout()` rejects with a `TimeoutError`. |

If the bot is in the air when a `goto` fails, the goal ends as it lands.

A goal set with `setGoal` has no promise. It stays set after a failed search, and the bot walks the partial path it found.
For a long trip, that is the way to go: the goal is solved again as chunks arrive. A `GoalFollow` emits no `goal_reached`
when the bot comes within range, because the entity can move again.

### Events

`goal_reached`, `goal_updated`, `path_update`, `path_reset` and `path_stop` match mineflayer-pathfinder. `path_stop` also
carries the reason: the ones above, plus `aborted`, `no_path` and `timeout` when a `goto` ended that way. Action and
interrupt events are prefixed `pathfinder:`. The full list with payloads is [`PathfinderEvents`](src/api.ts).

### Timeouts

| On `bot.pathfinder` | Default | Meaning |
|---|---|---|
| `thinkTimeout` | `5000` | Search budget per solve, ms. With the `parkour` preset, a failed search can spend about as long again looking for hops, and longer while it keeps making progress. Use a `signal` for a hard deadline. |
| `tickTimeout` | `40` | Search per tick on the main thread, ms. |
| `searchRadius` | `-1` | Extra cost allowed past the heuristic. |
| `stuckTimeout` | `-1` | Ms without reaching a node before the goal stops. |
| `executionTimeout` | `-1` | Ms a goal may run before it stops. |
| `keepPathDuringRecompute` | `true` | Keep walking the valid part of a path while a replan runs. |

### When the search runs on your thread

The worker cannot call your code, so the search runs on your thread, in `tickTimeout` slices, when the goal is a custom
`Goal` subclass, when `exclusionAreasStep` is set, when `canDig` and `exclusionAreasBreak` are both set, or when worker
threads are off or the worker has died. Hop discovery only runs on the worker, so this search finds fewer parkour routes.

Taking the world snapshot always happens on your thread. It is bounded by `maxSnapshotCells`.

## Staying alive

```js
bot.pathfinder.minHealth = 8
bot.on('pathfinder:low_health', (health) => retreatOrEat(health)) // yours to write
```

With `minHealth` set, two things happen:

- Below the threshold the goal stops once the body is on the ground, in water or on a ladder, never mid-flight.
  `goto` rejects with `PathStopped` and `reason: 'low_health'`. Healing and retreating are up to you, and a new goal set
  while still below the threshold stops the same way.
- Each fall a path chooses is limited to what the spare health can pay for at the moment the path is planned.

This is a floor for the bot's own choices, not a guarantee. Several falls on one path can add up, because the stop only
comes after the landing that crosses the threshold. It also cannot stop a missed jump, a fall you did not plan, or a player
hitting the bot.

It is off by default (`0`). `stopOnDeath` (default `true`) ends the goal when the bot dies, so it does not walk back to
it after the respawn. Set it to `false` for the mineflayer-pathfinder behaviour.

## Planning without moving

```js
const result = await bot.pathfinder.computePath(goal, { timeout: 2000 })
if (result.status === 'success') console.log(`${result.path.length} nodes, cost ${result.cost}`)
```

`computePath` leaves the current goal and path alone. It searches on the worker when
[it can](#when-the-search-runs-on-your-thread). Options: `from`, `movements`, `timeout`, `searchRadius`, `signal`. Status
is `success`, `noPath` or `timeout`, with the path to the closest cell reached when it is not `success`. `getPathTo` and
`getPathFromTo` also exist for compatibility, but they always block your thread.

## Interactions

`goto` gets the bot somewhere. These get something done, and walk only when the block is out of reach:

```js
await bot.pathfinder.dig(blockPosition)
await bot.pathfinder.place(emptyPosition, { item: 'cobblestone' })
const window = await bot.pathfinder.open(chestPosition)   // any window, not only containers
await bot.pathfinder.activate(leverPosition)              // lever, button, door
```

A position is a `Vec3`, a `{ x, y, z }` or a block.

They run one at a time and hold the bot while they act. Walking into range replaces the current goal, so a `goto` in
progress rejects with `GoalChanged`. Pass `{ approach: false }` to get `OutOfReach` instead of a walk. Each takes a
`signal`, and `stop()` does not end them.

An action called from inside another action, or inside your own `withInterrupt`, runs at once rather than queueing behind
the hold its caller already has. It cannot walk there, so pass `approach: false` or expect `OutOfReach`. Holding a handle
from `interrupt()` instead, call `actions.dig` and the rest directly: they take no hold of their own.

All four check range and aim before the click. Beyond that:

| | Line of sight | Standing still | After a position correction | Afterwards |
|---|---|---|---|---|
| `dig` | Needs the cursor on a face | Refuses while airborne | Refuses during `forcedMoveGrace` | The block must be gone a few ticks later. |
| `place` | To the support's face | Waits up to a second, then clicks anyway | Refuses during `forcedMoveGrace` | The block, or the one the item makes, must be there. |
| `open` | When a face can be picked | Not checked | Waits 8 ticks, then clicks | A window must open. |
| `activate` | When a face can be picked | Not checked | Not checked | None: it resolves once the click is sent. Check the block yourself. |

Doors that a path walks through are opened without aiming.

`place` prefers a support it can reach and see. For a shulker box it also skips supports whose opposite side is blocked,
so the box can open. `requireOpenable: true` asks the same of any block, `false` turns it off.

Their failures have a stable `name`: `OutOfReach`, `Occluded`, `NoTarget`, `DigFailed`, `PlaceFailed`, `OpenFailed`,
`ActivateFailed`, `MissingItem`, `Unreachable`, `ActionAborted`. `err.refused === true` means the action stopped short
and repositioning and retrying is reasonable. An error from mineflayer itself or from an override of yours can also come
through unchanged. `ActionAborted` covers a `signal`, a walk that something else ended, a disconnect and `dispose()`.

Defaults live on `bot.pathfinder.actions.config`:

| Field | Default | Meaning |
|---|---|---|
| `reach` | `4.5` | Interaction range. |
| `retries` | `3` | Attempts for `dig`, `place` and `activate`. `open` ignores it and escalates through four steps instead; its own `retries` option caps them. |
| `timeout` | `30000` | Ms until `dig`, `place` and `open` stop starting new attempts, counted from when the action gets its turn, after any walk. `activate` has none. |
| `pacing` | `'default'` | Spacing of breaks and placements: `'none'`, `'default'` or `'drill'`. Use `'none'` on a server you control. |
| `windowTimeout` | `3000` | Ms to wait for a window. |
| `forcedMoveGrace` | `800` | Ms after a position correction during which `dig` and `place` refuse to click. `open` waits 8 ticks instead; `activate` does not check. |

Change fields of `actions.config`; the actions keep that object, so replacing it has no effect.

**Paths place no blocks.** The planner digs when `canDig` is on and opens doors, but it never builds. `place()` puts
the block you ask for where you ask for it. `allow1by1towers` and `scafoldingBlocks` are accepted and ignored.

## Borrowing the bot

```js
await bot.pathfinder.withInterrupt('totem-swap', () => bot.equip(totem, 'off-hand'))
```

The request waits for a safe moment: on the ground, in water or on a ladder, not mid-flight or mid-dig. The path is not
reset, no `goto` rejects, and the walk carries on from the same node on release. Requests are exclusive and first come,
first served. `{ exclusive: false }` is for an observer that only needs the bot to stand still.

`{ timeout }` (default 5 s) is how long to wait for a safe moment. **When it runs out the request is granted anyway**, even
in mid-flight. `{ timeout: 0 }` waits for as long as it takes. `{ force: true }` takes the next tick without waiting. No
option overrides another exclusive holder, so the total wait can be longer than the timeout. The actions above ask with
the 5 s default. A dig or door the path had started is abandoned when a request is granted in its middle.

While an exclusive request is held, every control is yours: the executor releases its keys at the handover and writes
none until the release, not even jump in water. With no goal at all, the executor leaves the controls alone, apart from
braking the drift of its own last stop until the bot is still or something else presses a movement key.

`bot.pathfinder.motion` tells you what the executor is doing: `phase`, `critical`, `jumpPending`, `paused`, `holders`,
and `node`, the next node of the path. `isMoving()` is true while a path is being followed, including while paused.
`isMining()` and `isBuilding()` cover only the path's own digs and doors.

### Auto-eat

An adapter for [mineflayer-auto-eat](https://github.com/linkle69/mineflayer-auto-eat) (`npm install mineflayer-auto-eat`)
drives eating through this protocol:

```js
import { pathfinder, autoEatIntegration } from '@bulba/pathfinder'
import { loader as autoEat } from 'mineflayer-auto-eat'

bot.loadPlugin(pathfinder)
bot.loadPlugin(autoEat)
bot.loadPlugin(autoEatIntegration({ startAt: 16 }))
```

It waits up to `waitForSafe` (default 10 s) for a safe moment, then eats anyway. Pass `waitForSafe: 0` to wait for as long
as it takes. `bot.pathfinderAutoEat` can `suspend()`, `resume()` and `eatNow()`.

## Building on it

Keep your package a consumer of an existing `bot.pathfinder` rather than a fork of it:

| You want to | Use |
|---|---|
| Change or observe how blocks are broken, placed, opened or equipped | `actions.wrap` |
| Decide where the bot may walk or dig | `Movements` sets and `exclusionAreas*` |
| Define a new kind of destination | A `Goal` subclass |
| Check a route before committing | `computePath` |
| Take the bot for a moment | `interrupt` / `withInterrupt` |
| React to what happens | The events and `motion` |
| Cancel what you started | `signal` on `goto`, `computePath`, `dig`, `place`, `open` and `activate` |
| Unload cleanly | `dispose()` |

`actions.wrap` keeps the built-in guards and returns a function that removes the wrapper, in any order:

```js
const unwrap = bot.pathfinder.actions.wrap('dig', (next) => async (block, options) => {
  if (protectedRegion.contains(block.position)) throw new Error('not here')
  return await next(block, options)
})
```

Everything goes through this table, including the digs and doors of a path being walked. Assigning
`actions.dig = fn` replaces the entry outright.

The solver, snapshot and worker host are available from `@bulba/pathfinder/internals`. They carry no stability promise.

## Plugin options

Load a configured instance instead of `pathfinder`, not as well as it:

```js
import { createPathfinder } from '@bulba/pathfinder'

bot.loadPlugin(createPathfinder({
  useWorkerThreads: true,      // false: always solve on the main thread, in time slices
  core: 'wasm',                // 'js': the JavaScript core, on the same worker
  maxSnapshotCells: 8_000_000, // world snapshot size limit, at 2 to 6 bytes a cell
  hitboxPrecisionFix: true,    // see below
  logger: console,             // where warnings go: any object with warn(message)
  onNoPath: (dump) => {}       // receives the solver's inputs when a goal has no path
}))
```

`hitboxPrecisionFix` nudges the player's collision box by 1e-5 so that it does not sit exactly on a block boundary, where
1.21 servers reject the next move and teleport the bot back
([mineflayer#3911](https://github.com/PrismarineJS/mineflayer/issues/3911)).

All bots in a process share one worker thread. It never keeps the process alive.

## The physics patch

Two things in the installed prismarine-physics differ from the game: it does not climb nether and cave vines, and from
1.21 a jump should keep a faster upward speed (a jump held on a slime rebound keeps the rebound). Installing the package
changes no other package. It works without the patch: it detects what your copy of the physics does, leaves those vines
out of its plans, and simulates the jump rule your bot actually runs.

To get the exact behaviour, apply the patch yourself. It only edits prismarine-physics in your `node_modules`, and it is
safe to run again:

```sh
npm exec -- bulba-pathfinder-patch-physics
```

An install can replace the patched file, so to keep it, run it after every install:

```json
{ "scripts": { "postinstall": "bulba-pathfinder-patch-physics" } }
```

## Compatibility

| | |
|---|---|
| Node | 22 or newer, tested on 22 and 24 |
| mineflayer | 4.37 or newer within major version 4 |
| Live server | Paper 1.21.11: the 60-route benchmark below |
| Unit tests | World data from Minecraft 1.8.9 to 1.21.1, mostly 1.21.1 |

Other versions should work wherever mineflayer does, but they have not been raced on a real server. Reports are welcome.

### Differences from mineflayer-pathfinder

| Difference | Why |
|---|---|
| `canDig` defaults to `false` | A pathfinder that digs without being asked surprises people. |
| No placement moves: `allow1by1towers` and `scafoldingBlocks` are ignored | The planner does not build. |
| `isBuilding()` is only true while opening a door or gate | Same reason. |
| The goal ends when the bot dies | Set `stopOnDeath = false` for the old behaviour. |
| A failed `goto` ends its goal | Upstream rejects and keeps walking the partial path. |
| An unreachable goal rejects `NoPath` even when the path is empty | Upstream resolves, as if the bot had arrived. |
| `stop()` while idle does nothing | Upstream latches a flag that kills your next goal. |
| With no goal, the executor leaves the controls alone | Upstream does too; here it is a promise, so other plugins can drive. |
| `goal_reached` fires after the goal is cleared | A listener can set the next goal without it being wiped. |
| `GoalFollow` ends when its entity is no longer valid | Upstream keeps walking to where it was. |
| `GoalLookAtBlock` measures reach from the eye | Upstream measures from the feet, and misses blocks above the bot. |
| Nether and cave vines are climbed if your physics can | See [the physics patch](#the-physics-patch). |
| `getPathFromTo` needs a real start position | Upstream also accepts `null`. |
| Per-move generators such as `getMoveForward` do not exist | Moves are generated against the snapshot, on the worker. |

Costs for the shared moves follow the same model, and a differential test runs mineflayer-pathfinder as an oracle.

Uninstall mineflayer-pathfinder when you move over. Both packages declare `bot.pathfinder`, so with both in a TypeScript
program the type you get depends on which one loads first.

### Limits

- Search has no breath budget or flowing-water model, so it can ask for a swim that has to be abandoned part way.
- The planner has no vertical move in water and no block placement.
- The physics kernel is calibrated to prismarine-physics 1.11. Other versions may fall out of step with it.

## Development

```sh
npm install          # also builds dist/, which the worker tests need
npm test             # applies the physics patch to this checkout's node_modules first
npm run test:pack    # packs the tarball and uses it from empty ESM, CJS and TS projects
npm run build:wasm   # rebuilds the Rust core (rustup target add wasm32-unknown-unknown)
```

[`bench/arena`](bench/arena/README.md) races engines on a real Paper server: 60 routes including hard parkour courses, of
which the `parkour` preset finished 59 when 0.1.0 was prepared. Design notes are in [`docs/`](docs/).

Environment variables starting with `PF_` are switches and traces for that benchmark. They are not supported and can
change or disappear in any release.

## License

MIT. The cost model, goal classes and tick-loop semantics are ported from
[mineflayer-pathfinder](https://github.com/PrismarineJS/mineflayer-pathfinder) (MIT). The design also draws on
[azalea](https://github.com/azalea-rs/azalea) and [Baritone](https://github.com/cabaletta/baritone).
