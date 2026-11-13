# Changelog

This project follows [semantic versioning](https://semver.org/). Before 1.0, a minor version can break the API.
`bulba-pathfinder/internals` and `PF_*` environment variables carry no stability promise at any version.

## 0.2.0 (2026-11-13)

### Added

- Fall recovery (`bot.pathfinder.fallRecovery`). The pathfinder counts falls the way the server does and notices a
  fall the path did not plan (knockback, explosions, a floor broken away, or falling with no goal).
  - Events `pathfinder:fall_start`, `pathfinder:fall_lethal` and `pathfinder:fall_finish`.
  - In `'recover'` mode it steers in the air toward the cheapest landing and uses a fall breaker just before landing:
    water or fish buckets, cobweb or hay bales, then takes the water back.
  - Items are chosen with `items` rules (name, id or a function) and limited with an `inventory` function.
- `movements.maxClutchDropDown`: paths may plan drops deeper than `maxDropDown` and land them with a water bucket. Off
  by default; needs `'recover'` mode and a usable bucket.
- `motion.phase` can be `'recovering'`; `path_reset` can carry `'fall'`, `'clutch_missing'` and `'clutch_unfit'`.

The default mode, `'observe'`, only emits events: paths are walked as before.

## 0.1.0 (2026-11-13)

First public release.

### Added

- `Movements.applyPreset('parkour')`: turns on the full physics-verified parkour engine in one call.
- `goto(goal, { signal })`: cancel a goal from outside.
- `computePath(goal, options)`: plan on the worker without moving the bot or touching its goal.
- `minHealth`: stop a goal below a health threshold, and limit each planned fall to what health can pay for.
- `stopOnDeath`, plus an end to goals when the connection drops, so `goto` can no longer hang on a dead bot.
- `dispose()`: take the plugin off a live bot.
- `actions.wrap(name, wrapper)`: composable action overrides that can be removed in any order.
- `PathStopped` errors and `path_stop` events carry a `reason`.
- `logger` plugin option.
- TypeScript types for `bot.pathfinder` and the events on `bot.on`.

### Changed

- The solver, snapshot and worker building blocks moved from the package root to `bulba-pathfinder/internals`.
- Installing no longer edits prismarine-physics. The patch is now an explicit, optional command,
  `bulba-pathfinder-patch-physics`, and an unpatched install leaves nether and cave vines out of its plans.
- Node 22 or newer, mineflayer 4.37 or newer.
- A `goto` that rejects with `NoPath` or `Timeout`, or a best-effort `goto` that resolves short of its goal, now ends the
  goal. It used to keep walking the partial path.
- An unreachable goal with an empty path now rejects with `NoPath` or `Timeout`. It used to resolve as if it had arrived.
- A walk that an action started and something else ended fails the action with `ActionAborted`. It used to start a second
  walk.
- While an exclusive interrupt is held the executor writes no controls at all, and with no goal it writes none either,
  apart from braking its own last stop.
- `goal_reached` is emitted after the goal is cleared, so a listener can set the next one.
- `GoalFollow` ends when its entity is no longer valid. `GoalLookAtBlock` measures reach from the eye.
- `place` only keeps the opposite side of its support clear for shulker boxes, unless told otherwise, and accepts a
  block that differs from the item's name when the item places it (a torch on a wall, redstone, seeds).
- An action called inside another action or a `withInterrupt` runs inline instead of waiting forever for its turn.
- `actions.config` is read-only as a property: change its fields.

### Fixed

- An interrupted dig no longer removes other plugins' `diggingAborted` and `diggingCompleted` listeners.
- `dig` treats an unloaded chunk as "unknown" instead of as "already broken".
- `dig` and `place` equip through `actions.equip`, so an override of it covers them too.
- An idle worker thread no longer holds the process open.
- Actions and `goto` calls can no longer hang after a disconnect or `dispose()`.
- CommonJS TypeScript consumers get CommonJS declarations.
- A dig that takes longer than about 6 seconds can finish; its timeout used to fire first.
- `dig` waits for the server's answer before judging the result, so a refused break is reported as `DigFailed`.
- The path's own digs stop when the goal does, even while still aiming.
- A worker that dies during a cancelled solve can no longer crash the process, and neither can an interrupt timer firing
  before login.
- TypeScript: `pf.dig` and the other actions accept prismarine-block blocks, `autoEatIntegration()` can be loaded,
  exclusion callbacks receive a typed `Block`, and `bot.pathfinderAutoEat` is declared.
- A block from a second copy of vec3 is still accepted as a target. An option passed as `undefined` keeps its default.
- The physics patch replaces the file instead of writing into it, so a pnpm store shared by other projects is left alone.
