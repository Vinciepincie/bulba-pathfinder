# API reference

- [Loading the plugin](#loading-the-plugin)
- [bot.pathfinder](#botpathfinder): [properties](#properties), [methods](#methods), [errors](#errors), [events](#events)
- [Movements](#movements)
- [Goals](#goals)
- [Actions](#actions)
- [Interrupts](#interrupts)
- [Auto-eat](#auto-eat)
- [Where the search runs](#where-the-search-runs)
- [The physics patch](#the-physics-patch)
- [Differences from mineflayer-pathfinder](#differences-from-mineflayer-pathfinder)
- [Limits](#limits)

Terms used below: the **planner** searches for a path on a copy of the nearby world (the **snapshot**). The
**executor** walks that path, tick by tick, steering, sprinting and jumping. A **node** is one step of the path.

## Loading the plugin

### pathfinder

The plugin, ready to load: `bot.loadPlugin(pathfinder)`. It adds `bot.pathfinder`.

### createPathfinder([options])

Returns a configured plugin. Load it instead of `pathfinder`, not as well as it.

```js
bot.loadPlugin(createPathfinder({ useWorkerThreads: true }))
```

- `useWorkerThreads` (default `true`): search on a worker thread. `false` searches on the main thread, in time slices.
- `core` (default `'wasm'`): the worker's search core, `'wasm'` (Rust) or `'js'`. Both give the same paths.
- `maxSnapshotCells` (default `8_000_000`): the largest snapshot, in cells. A cell costs 2 to 6 bytes.
- `hitboxPrecisionFix` (default `true`): nudge the player's collision box by 1e-5, so it never rests exactly on a
  block boundary. 1.21 servers reject the next move from there and teleport the bot back
  ([mineflayer#3911](https://github.com/PrismarineJS/mineflayer/issues/3911)).
- `logger` (default `console`): where warnings go. Any object with `warn(message)`.
- `onNoPath(dump)`: called with the planner's inputs when a goal has no path, for offline debugging.

All bots in a process share one worker thread. It does not keep the process alive.

### Imports

```js
import { pathfinder, createPathfinder, Movements, goals, autoEatIntegration } from '@bulba/pathfinder'
const { pathfinder, Movements, goals } = require('@bulba/pathfinder')
```

Importing the package also types `bot.pathfinder` and its events on `bot.on` in TypeScript.

The solver, snapshot and worker host are exported from `@bulba/pathfinder/internals`. They carry no stability promise.
Environment variables starting with `PF_` are benchmark switches and traces, unsupported, and can change in any release.

## bot.pathfinder

### Properties

#### bot.pathfinder.goal

The current goal, or `null`. Read-only.

#### bot.pathfinder.movements

The current `Movements`. Read-only: change it with `setMovements`.

#### bot.pathfinder.thinkTimeout

Search budget per path, in ms. Default `5000`. With the `parkour` preset, a search that fails can spend about as long
again looking for jumps, and longer while it keeps finding them. Pass a `signal` to `goto` for a hard deadline.

#### bot.pathfinder.tickTimeout

How long a main-thread search may run per tick, in ms. Default `40`.

#### bot.pathfinder.searchRadius

How much further than the straight-line estimate a path may wander, as extra cost. Default `-1`, no limit.

#### bot.pathfinder.stuckTimeout

Ms without reaching the next node before the goal stops. Default `-1`, off.

#### bot.pathfinder.executionTimeout

Ms a goal may run before it stops. Default `-1`, off.

#### bot.pathfinder.keepPathDuringRecompute

Keep walking the part of the path that is still valid while a new one is searched. Default `true`.

#### bot.pathfinder.minHealth

The health to keep, from 0 to 20. Default `0`, off. With it set:

- Below it, the goal stops as soon as the bot is on the ground, in water or on a ladder, never in mid-air. `goto` rejects
  with `PathStopped`, `reason: 'low_health'`, and `pathfinder:low_health` is emitted. A new goal set while still below
  it stops the same way.
- Each fall a path chooses is limited to what the health above the floor can pay for, at the moment the path is
  planned.

It limits the bot's own choices. Several falls on one path can add up before the stop, and it cannot prevent a missed
jump or damage from elsewhere.

#### bot.pathfinder.stopOnDeath

End the goal when the bot dies, so it does not walk back after respawning. Default `true`. mineflayer-pathfinder keeps
the goal.

#### bot.pathfinder.motion

What the executor is doing now:

- `phase`: `'idle'`, `'walking'`, `'airborne'`, `'climbing'`, `'swimming'`, `'digging'`, `'interacting'` or `'paused'`.
- `critical`: taking the controls now would break a jump, a dig or an interaction.
- `jumpPending`: the next node is a jump that has not started.
- `paused`: an interrupt holds the bot.
- `holders`: the reasons of the interrupts held.
- `node`: the next node of the path, or `null`. Read it, do not change it.

#### bot.pathfinder.actions

The action table. See [Actions](#actions).

#### bot.pathfinder.enablePathShortcut

Merge straight runs of path nodes, as in mineflayer-pathfinder. Default `false`. Has no effect with
`exclusionAreasStep` set.

#### bot.pathfinder.LOSWhenPlacingBlocks

Accepted for compatibility with mineflayer-pathfinder. It has no effect.

### Methods

#### bot.pathfinder.goto(goal, [options])

Walk to `goal`. Returns a promise that resolves on arrival and rejects with one of the [errors](#errors). A rejection
also ends the goal. If the bot is in the air at that moment, the goal ends once it lands.

- `signal`: an `AbortSignal` that cancels the goal. `goto` rejects with the signal's reason when that is an `Error`,
  else with an `AbortError`.
- `bestEffort` (default `false`): when there is no path, walk as close as possible and resolve there. Check where the
  bot ended up. It does not cover `Timeout`.
- `maxBestEffortLegs` (default `3`): how many partial walks `bestEffort` makes before resolving.

#### bot.pathfinder.setGoal(goal, [dynamic])

Set a goal without waiting for it, or `null` to clear it. With `dynamic` (default `false`) the goal stays after the bot
reaches it, so a moving goal such as `GoalFollow` keeps being followed. Either kind is planned again when it moves.
A goal set this way stays set after a failed search, and the bot walks the partial path it found. For a
long trip that is useful: the goal is planned again as new chunks arrive.

#### bot.pathfinder.setMovements(movements)

Use a different [`Movements`](#movements). The current path is planned again.

#### bot.pathfinder.stop()

End the goal on the next tick and emit `path_stop`. A bot in the air keeps its controls until it lands. With no goal it
does nothing.

#### bot.pathfinder.computePath(goal, [options])

Plan a path without moving or touching the current goal. Returns a promise of a result with `status` (`'success'`,
`'noPath'` or `'timeout'`), `path`, `cost`, `time`, `visitedNodes` and `generatedNodes`. When the status is not
`success`, `path` leads to the closest point found.

- `from`: where the path starts. Default: the bot's position.
- `movements`: the `Movements` to plan with. Default: the current one.
- `timeout`: search budget in ms. Default: `thinkTimeout`.
- `searchRadius`: default `searchRadius`.
- `signal`: cancels the search.

#### bot.pathfinder.getPathTo(movements, goal, [timeout]), bot.pathfinder.getPathFromTo(movements, startPos, goal, [options])

The mineflayer-pathfinder ways to plan without moving. They search on the main thread and block it. Prefer
`computePath`.

#### bot.pathfinder.dig(target, [options])

Walk into range if needed, then break the block. See [Actions](#actions). `target` is a block, a `Vec3` or `{ x, y, z }`.
Given a block, it digs that block, and refuses with `NoTarget` if something else has taken its place.

#### bot.pathfinder.place(position, [options])

Walk into range if needed, then place `options.item` (a name, an id or an inventory item) at `position`.

#### bot.pathfinder.open(target, [options])

Walk into range if needed, then open the block's window: a chest, a furnace, a crafting table, any window. Resolves with
the window.

#### bot.pathfinder.activate(target, [options])

Walk into range if needed, then right-click the block: a lever, a button, a door.

#### bot.pathfinder.interrupt(reason, [options]), bot.pathfinder.withInterrupt(reason, fn, [options])

Take the bot for a moment. See [Interrupts](#interrupts).

#### bot.pathfinder.dispose()

Stop any goal, end pending actions and interrupts, remove every listener and take `bot.pathfinder` off the bot.

#### bot.pathfinder.isMoving(), isMining(), isBuilding()

`isMoving()` is true while a path is being followed, including while paused. `isMining()` and `isBuilding()` cover only
the path's own digs and doors.

#### bot.pathfinder.bestHarvestTool(block)

The inventory item that breaks `block` fastest, which can be any item, or `null` with an empty inventory. An empty
hand is not compared.

#### Recovery helpers

- `isStuck()`: the bot's hitbox is inside a solid block.
- `unstick()`: walk and jump to the nearest cell it can stand in. Clears the goal first.
- `settle()`: clear the goal and controls, unstick if needed, and wait until the bot stops drifting.
- `canStandAt(pos)`, `findEscape([radius])`, `findNearestOpenStandable([radius])`: find cells to stand in.
- `centreInCell(cell, [tolerance])`: centre the bot in a cell it has reached.
- `jumpOnce()`: jump once.

### Errors

`goto` rejects with an error whose `name` tells the cause:

| `err.name` | Meaning |
|---|---|
| `NoPath` | No route through the world the bot has loaded. Chunks it has not received count as blocked. |
| `Timeout` | No path was found within `thinkTimeout`. |
| `GoalChanged` | Another goal replaced this one, or `setGoal(null)` cleared it. A replacing goal stays. |
| `PathStopped` | The goal was stopped. `err.reason` says why. |
| `AbortError` | The `signal` fired with no `Error` as its reason. With one, `goto` rejects with that error, so `AbortSignal.timeout()` gives a `TimeoutError`. |

`PathStopped` reasons, also passed to the `path_stop` event:

| `reason` | Cause |
|---|---|
| `stop` | `stop()`, `stuckTimeout`, `executionTimeout`, or a goal that became invalid, such as a `GoalFollow` whose entity left |
| `low_health` | Health fell below `minHealth` |
| `death` | The bot died, with `stopOnDeath` on |
| `disconnected` | The connection ended |
| `disposed` | `dispose()` |
| `unsupported` | The `Movements` cannot be planned with |

`path_stop` has three more: `aborted` (a `goto` signal), and `no_path` and `timeout` (a `goto` that failed with
`NoPath` or `Timeout`). The same outcome is spelled `NoPath` as an error name, `noPath` as a `computePath` status, and
`no_path` as a stop reason, following the conventions of each.

Actions have their own errors, listed under [Actions](#actions).

### Events

Emitted on the bot. `goal_reached`, `goal_updated`, `path_update`, `path_reset` and `path_stop` match
mineflayer-pathfinder.

| Event | Arguments |
|---|---|
| `goal_reached` | `goal`. Emitted after the goal is cleared, so a listener can set the next one. A goal set with `setGoal(goal, true)` never emits it. |
| `goal_updated` | `goal`, `dynamic` |
| `path_update` | A result, as from `computePath`, with `status` also `'partial'` while a search is still running |
| `path_reset` | `reason`: why the path was dropped and planned again, such as `'block_updated'` or `'stuck'` |
| `path_stop` | `reason`, see above |
| `pathfinder:low_health` | `health`, `minHealth` |
| `pathfinder:interrupt_requested` | `reason` |
| `pathfinder:paused` | `reasons` |
| `pathfinder:resumed` | `reason`, `pausedMs` |
| `pathfinder:dig_start` | `block` |
| `pathfinder:dig_finish` | `position` |
| `pathfinder:dig_aborted` | `position`, `why` |
| `pathfinder:dig_error` | `error`, a dig the path started that failed |
| `pathfinder:place_start`, `place_finish` | `position`, and the `item` or the placed block |
| `pathfinder:open_start`, `open_finish`, `open_retry` | `position`, and the window or the retry step and message |
| `pathfinder:activate` | `position`, `block` |
| `pathfinder:activate_error` | `error`, a door the path opened that did not respond |
| `pathfinder:autoeat_start`, `autoeat_finish`, `autoeat_failed` | With the auto-eat adapter loaded. `autoeat_failed` passes the error. |

## Movements

What the bot may do. A new `Movements(bot)` walks, sprints and makes basic jumps like mineflayer-pathfinder, but does
not dig.

### movements.applyPreset(name)

Set the movement flags of a preset, and return the `Movements`. Other settings are left alone.

| Preset | |
|---|---|
| `'default'` | Walking, sprinting and basic straight jumps. |
| `'parkour'` | Adds every kind of jump, checked by simulating the player's physics: diagonal, long, up and down jumps, catches on ladders and in water, slime bounces, chained jumps, sprint-hopping, and run-ups for long gaps. |

### Settings

| Setting | Default | Meaning |
|---|---|---|
| `canDig` | `false` | Paths may break blocks, with the best tool. |
| `allowSprinting` | `true` | Sprint where it is safe. |
| `allowParkour` | `true` | Basic gap jumps. |
| `canOpenDoors` | `false` | Paths may go through doors and fence gates, opening them. |
| `maxDropDown` | `4` | Longest drop a path may choose, unless it lands in water. As in mineflayer-pathfinder, a walk-off counts down to the block it lands on, so `4` means falling 3 blocks. A jump counts from feet to feet, so `4` allows landing 4 lower. Only the `parkour` preset's physics-checked jumps are bounded by fall damage instead. |
| `infiniteLiquidDropdownDistance` | `true` | Any drop into water is fine. |
| `blocksToAvoid` | fire, cobweb, lava | A `Set` of block ids a path never enters. |
| `blocksCantBreak` | chest and unbreakable blocks | A `Set` of block ids a digging path leaves alone. |
| `exclusionAreasStep`, `exclusionAreasBreak` | `[]` | Functions from a block to an extra cost for stepping on it or breaking it. A break cost of `100` or more forbids the break. `exclusionAreasStep` makes the search run on the main thread, and so does `exclusionAreasBreak` when `canDig` is on. |
| `useBubbleColumns` | `false` | Ride bubble columns up and down. |
| `parkourSafetyMargin` | `0.1` | Blocks of jump reach held back for timing and lag. |
| `digCost`, `liquidCost`, `entityCost` | `1` | Extra cost of digging, swimming and walking through entities. |
| `allowEntityDetection` | `true` | Make cells other entities stand in cost `entityCost` more per entity. |
| `allowCornerCut` | `true` | Steer straight across corners instead of along each node. |
| `allowStopSnap` | `true` | On stopping, cancel drift and centre the bot in its block. |

The flags behind the `parkour` preset, all off by default: `allowParkourExtended`, `allowParkourMomentum`,
`allowParkourPhysics`, `allowSprintHop`, `allowLowCeilingHop`, `allowRunUp`, `allowLandingRetire`, `allowHopIntoJump`,
`allowStrafe`. `allowParkourTight` (on) offers frame-perfect jumps at a higher cost. Each is described in
[src/movements.ts](../src/movements.ts).

## Goals

From `goals`:

| Goal | Reached when the bot |
|---|---|
| `GoalBlock(x, y, z)` | Stands in that block. |
| `GoalNear(x, y, z, range)` | Is within `range` of it. |
| `GoalXZ(x, z)`, `GoalNearXZ(x, z, range)` | Reaches that column, or comes within `range` of it. |
| `GoalY(y)` | Is at that height. |
| `GoalGetToBlock(x, y, z)` | Stands next to it. |
| `GoalLookAtBlock(pos, bot.world, { reach })` | Can see a face of it within reach. |
| `GoalBreakBlock(x, y, z, bot, { reach })` | Can break it from there. |
| `GoalPlaceBlock(pos, bot.world, options)` | Can place a block there. |
| `GoalFollow(entity, range)` | Is within `range` of the entity. Use with `setGoal(goal, true)`. It ends when the entity is no longer valid. |
| `GoalInvert(goal)` | Has left the other goal. |
| `GoalCompositeAny(goals)`, `GoalCompositeAll(goals)` | Meets any, or all, of them. |

A subclass of `Goal` works too, but searches on the main thread: see [Where the search runs](#where-the-search-runs).

## Actions

`bot.pathfinder.dig`, `place`, `open` and `activate` queue one at a time, take the bot with an
[interrupt](#interrupts) while they act, and walk into range first. Walking replaces the current goal, so a `goto` in
progress rejects with `GoalChanged`.

Options common to all four:

- `approach` (default `true`): walk into range. `false` throws `OutOfReach` instead.
- `reach`, `retries`, `timeout`: defaults from `actions.config`. For `open`, `retries` caps the escalation steps,
  four by default. `activate` has no `timeout`.
- `signal`: cancels the action. `stop()` only cancels it during its walk into range, with `ActionAborted`.

`dig` also takes `equipTool` (default `true`), `requireGrounded` (default `true`) and `pacing`. `place` takes `item`,
`faces`, `sneak`, `requireOpenable`, `requireGrounded`, `settleTicks` and `pacing`. `open` takes `windowTimeout`,
`closeStale` and `reuse`. `activate` takes `sneak` and `settle`.

What each checks:

| | Aim | Standing still | After a server position correction | Afterwards |
|---|---|---|---|---|
| `dig` | The cursor must be on a face | Waits up to a second, then refuses while airborne | Refuses during `forcedMoveGrace` | The block must be gone a few ticks later |
| `place` | Line of sight to the supporting face | Waits up to a second | Refuses during `forcedMoveGrace` | The block, or the one the item makes, must be there |
| `open` | Best effort | Not checked | Waits 8 ticks | A window must open |
| `activate` | Best effort | Not checked | Not checked | Nothing: check the block yourself |

`place` prefers a support it can reach and see. For a shulker box it also skips supports whose opposite side is
blocked, so the box can still open: `requireOpenable` turns this on or off for any block. Items that place a block of
another name, such as a torch on a wall, redstone or seeds, are recognised.

### Action errors

| `err.name` | Meaning |
|---|---|
| `OutOfReach` | Too far, with `approach: false`. |
| `Unreachable` | No path to a spot within reach. |
| `Occluded` | No clear line to a face. |
| `NoTarget` | The chunk is not loaded, or the cell holds another block than the one asked for. A `dig` whose block is already gone resolves. |
| `MissingItem` | Nothing to place. |
| `DigFailed`, `PlaceFailed`, `OpenFailed`, `ActivateFailed` | Every attempt failed, or the `timeout` ran out. `place` also fails at once when the cell holds another block, the bot stands in it, or no face can support it. |
| `ActionAborted` | The `signal` fired, the walk was ended by something else, the bot disconnected, or `dispose()`. |

`err.refused === true` means the action stopped before acting, and moving and retrying is reasonable. Errors from
mineflayer itself, or from your own overrides, come through unchanged.

### actions.config

Change its fields. The actions keep this object, so replacing it has no effect.

| Field | Default | Meaning |
|---|---|---|
| `reach` | `4.5` | Interaction range. |
| `retries` | `3` | Attempts for `dig`, `place` and `activate`. `open` escalates through up to four steps: as is, re-aim, a jump, a walk closer. |
| `timeout` | `30000` | Ms after which `dig`, `place` and `open` start no new attempt, counted from when they start acting, after their turn and their walk into range. `0` removes the cap. |
| `pacing` | `'default'` | Spacing between breaks and placements: `'none'`, `'default'` or `'drill'`. Use `'none'` on a server you control. |
| `windowTimeout` | `3000` | Ms to wait for a window. |
| `forcedMoveGrace` | `800` | Ms after a position correction during which `dig` and `place` refuse to click. |

### actions.wrap(name, wrapper)

Layer your own code over `'dig'`, `'place'`, `'open'`, `'activate'` or `'equip'`, keeping the built-in checks. Returns a
function that removes the layer, in any order. The path's own digs and doors go through the same table.

```js
const unwrap = bot.pathfinder.actions.wrap('dig', (next) => async (block, options) => {
  if (protectedRegion.contains(block.position)) throw new Error('not here') // your own check runs first
  return await next(block, options) // then the normal dig
})

unwrap() // removes this layer again
```

Assigning `actions.dig = fn` replaces the entry outright.

### Calling actions from your own code

An action called inside another action, or inside your own `withInterrupt`, runs at once instead of queueing behind the
hold its caller already has. Inside another action it can still walk into range: the outer hold is released for the
walk and taken back after it. Inside your own `withInterrupt` it cannot, so expect `OutOfReach` when the block is too
far. Holding a handle
from `interrupt()` instead, call `bot.pathfinder.actions.dig` and the others directly: they take no hold of their own.

## Interrupts

```js
await bot.pathfinder.withInterrupt('totem', () => bot.equip(totem, 'off-hand'))

const handle = await bot.pathfinder.interrupt('pvp', { force: true })
// ... drive the bot yourself ...
handle.release()
```

A request waits for a safe moment: on the ground, in water or on a ladder, not mid-jump or mid-dig. The path is kept,
no `goto` rejects, and the walk resumes from the same node on release. Requests are exclusive and served in order.

- `timeout` (default `5000`): how long to wait for a safe moment. **When it runs out, the request is granted anyway**,
  even mid-jump. `0` waits as long as it takes.
- `force`: take the bot on the next tick, without waiting.
- `exclusive` (default `true`): `false` is for an observer that only needs the bot to stand still, and can share.
- `signal`: cancels a request still waiting.

No option overrides another exclusive holder, so the wait can be longer than `timeout`. A dig or door the path had
started is abandoned when a request is granted in its middle.

While an exclusive request is held, every control is yours: the executor releases its keys and writes none until the
release, not even jump in water. With no goal at all, the executor also leaves the controls alone, apart from braking
its own last stop until the bot is still or something else presses a movement key.

## Auto-eat

An adapter for [mineflayer-auto-eat](https://github.com/linkle69/mineflayer-auto-eat), which eats through an
interrupt so it never happens mid-jump:

```js
import { pathfinder, autoEatIntegration } from '@bulba/pathfinder'
import { loader as autoEat } from 'mineflayer-auto-eat'

bot.loadPlugin(pathfinder)
bot.loadPlugin(autoEat)
bot.loadPlugin(autoEatIntegration({ startAt: 16 }))
```

Options: `startAt` (food level, default `16`), `healthBelow` (also eat at or below this health, default `0`, off),
`checkEveryTicks` (default `20`), `waitForSafe` (ms to wait for a safe moment before eating anyway, default `10000`,
`0` waits as long as it takes), `skipWhenWindowOpen` (default `true`), `eatOptions`, `reason`.

`bot.pathfinderAutoEat` has `eatNow()`, `suspend()`, `resume()` and `detach()`.

## Where the search runs

The search runs on the worker thread unless it has to call your code, which the worker cannot. It runs on the main
thread, in `tickTimeout` slices, for a custom `Goal` subclass, with `exclusionAreasStep`, with `canDig` and
`exclusionAreasBreak` together, with `useWorkerThreads: false`, or after the worker has failed. That search finds fewer
parkour routes: the physics-checked jumps are only searched for on the worker.

Taking the snapshot of the world always happens on the main thread. Its size is bounded by `maxSnapshotCells`.

## The physics patch

The prismarine-physics that mineflayer uses differs from the game in two places: it does not climb nether and cave
vines, and from 1.21 a jump should keep its faster upward speed, so that a jump held on a slime rebound keeps the
rebound. The pathfinder detects what your copy does, leaves those vines out of its paths and simulates the jump the bot
will actually make. Installing the package changes nothing in other packages.

`npx bulba-pathfinder-patch-physics` edits prismarine-physics in your `node_modules` to match the game. It is safe to run
again. An install can replace the file, so add it as a `postinstall` script to keep it:

```json
{ "scripts": { "postinstall": "bulba-pathfinder-patch-physics" } }
```

## Differences from mineflayer-pathfinder

| Here | mineflayer-pathfinder |
|---|---|
| `canDig` defaults to `false` | Digs by default |
| No block placement in paths: `allow1by1towers` and `scafoldingBlocks` are ignored, and `isBuilding()` is only true at doors | Builds towers and bridges |
| A failed `goto` ends its goal | Rejects and keeps walking the partial path |
| An unreachable goal rejects `NoPath` even when the path is empty | Resolves, as if the bot had arrived |
| The goal ends when the bot dies, unless `stopOnDeath = false` | Walks back after respawning |
| `stop()` while idle does nothing | Latches a flag that stops your next goal |
| `goal_reached` fires after the goal is cleared | Fires before, so a listener's next goal can be wiped |
| `GoalFollow` ends when its entity is no longer valid | Keeps walking to where it was |
| `GoalLookAtBlock` measures reach from the eye | Measures from the feet, and misses blocks above the bot |
| Nether and cave vines are climbed when the physics can | Not climbed |
| `getPathFromTo` needs a real start position | Also accepts `null` |
| No per-move generators such as `getMoveForward` | Available |

Costs for the moves both have follow the same model, and the tests check paths against mineflayer-pathfinder's.

Uninstall mineflayer-pathfinder when you switch: both declare `bot.pathfinder`, and with both in a TypeScript program
the type depends on which loads first.

## Limits

- No breath budget or flowing-water model, so a planned swim can be abandoned part way.
- No vertical moves in water, and no block placement in paths.
- The physics simulation is calibrated to prismarine-physics 1.11.
- Raced live only on Paper 1.21.11. The unit tests use world data from Minecraft 1.8.9 to 1.21.1.
