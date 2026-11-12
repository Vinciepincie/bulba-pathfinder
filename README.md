# bulba-pathfinder

[![npm](https://img.shields.io/npm/v/bulba-pathfinder)](https://www.npmjs.com/package/bulba-pathfinder)
[![CI](https://github.com/Vinciepincie/bulba-pathfinder/actions/workflows/ci.yml/badge.svg)](https://github.com/Vinciepincie/bulba-pathfinder/actions/workflows/ci.yml)
[![license](https://img.shields.io/npm/l/bulba-pathfinder)](https://github.com/Vinciepincie/bulba-pathfinder/blob/main/LICENSE)

Pathfinding for [mineflayer](https://github.com/PrismarineJS/mineflayer) bots, with the same API as
[mineflayer-pathfinder](https://github.com/PrismarineJS/mineflayer-pathfinder). Most code moves over by changing the
import; [a few defaults differ](#moving-from-mineflayer-pathfinder).

- **Searches off the main thread.** For the built-in goals, the search runs on a worker thread, in WebAssembly, so a
  long or hopeless search does not stall your bot.
- **Parkour.** Diagonal and long jumps, landing on ladders and in water, slime bounces and sprint-hopping, each
  simulated with the player's physics before the bot commits to it.
- **Pausable.** Your code, or another plugin, can take the bot with `withInterrupt`. It waits for the bot to land first,
  and the walk resumes where it stopped. An adapter for mineflayer-auto-eat is included.
- **Dig, place and open in one call.** `dig`, `place`, `open` and `activate` walk into range first. `dig` and `place`
  aim at a face the server accepts, and check that the block really changed.

> **0.x:** the API can still change between minor versions. Tested live on Paper 1.21.11; the unit tests cover
> Minecraft 1.8.9 to 1.21.1.

## Install

```sh
npm install bulba-pathfinder mineflayer
```

Node 22 or newer, and mineflayer 4.37 or a later 4.x.

## Example

Save as `bot.mjs` and run `node bot.mjs`:

```js
import { createBot } from 'mineflayer'
import { pathfinder, Movements, goals } from 'bulba-pathfinder'

const bot = createBot({ host: 'localhost', username: 'bot' })
bot.loadPlugin(pathfinder)

bot.once('spawn', () => {
  // 'parkour' adds diagonal, long and physics-checked jumps. Without it the bot walks,
  // sprints and makes straight gap jumps, like mineflayer-pathfinder.
  bot.pathfinder.setMovements(new Movements(bot).applyPreset('parkour'))
})

bot.on('chat', async (username, message) => {
  const player = bot.players[username]?.entity
  if (!player) return

  if (message === 'come') {
    const { x, y, z } = player.position
    try {
      await bot.pathfinder.goto(new goals.GoalNear(x, y, z, 1))
      bot.chat('here')
    } catch (err) {
      if (err.name === 'NoPath' || err.name === 'Timeout') bot.chat("can't get there")
    }
  }

  if (message === 'follow') bot.pathfinder.setGoal(new goals.GoalFollow(player, 2), true)
  if (message === 'stop') bot.pathfinder.stop()
})
```

CommonJS works too, with `require('bulba-pathfinder')`, and TypeScript types are included.

## Moving from mineflayer-pathfinder

Change the import and uninstall mineflayer-pathfinder. A few things behave differently:

| | mineflayer-pathfinder | bulba-pathfinder |
|---|---|---|
| Breaking blocks | On by default | Off by default. Set `movements.canDig = true` to turn it on. |
| A `goto` with no path | Rejects, but the bot keeps walking toward the goal | Rejects, and the bot stops |
| When the bot dies | It walks back to the goal after respawning | The goal ends. Set `bot.pathfinder.stopOnDeath = false` to keep it. |
| Placing blocks (towers, bridges) | Yes | Never. `allow1by1towers` and `scafoldingBlocks` are ignored. |

Plugins that bring their own mineflayer-pathfinder, such as mineflayer-pvp and mineflayer-collectblock, do not work
with it yet: they pass it mineflayer-pathfinder's own `Movements`.

The [full list](https://github.com/Vinciepincie/bulba-pathfinder/blob/main/docs/api.md#differences-from-mineflayer-pathfinder)
is in the API reference.

## More than walking

Stop whenever you like, from any event, and hear how it ended:

```js
bot.on('health', () => { if (bot.health < 6) bot.pathfinder.stop() })
bot.on('path_stop', (reason) => bot.chat(`stopped: ${reason}`))
```

Break, place and use blocks, walking into range first:

```js
await bot.pathfinder.dig(block)
await bot.pathfinder.place(position, { item: 'cobblestone' })
const chest = await bot.pathfinder.open(chestBlock)
```

Set a health floor. Below it the goal stops once the bot is on the ground, in water or on a ladder, and each fall a
path plans is limited to the health above the floor:

```js
bot.pathfinder.minHealth = 8
bot.on('pathfinder:low_health', () => { /* eat first: a goal set below the floor stops too */ })
```

Pause the walk to run your own code. The first argument is a label. It waits for the bot to land, then the walk
resumes:

```js
await bot.pathfinder.withInterrupt('swap-totem', () => bot.equip(totem, 'off-hand'))
```

Plan a route without moving:

```js
const { status, path } = await bot.pathfinder.computePath(goal)
```

Change how blocks are broken, including by the path's own digs:

```js
bot.pathfinder.actions.wrap('dig', (next) => async (block, options) => {
  if (isProtected(block.position)) throw new Error('not this one') // your own check runs first
  return await next(block, options) // then the normal dig
})
```

## Documentation

| | |
|---|---|
| [API reference](https://github.com/Vinciepincie/bulba-pathfinder/blob/main/docs/api.md) | Every method, option, goal, event and error |
| [Changelog](https://github.com/Vinciepincie/bulba-pathfinder/blob/main/CHANGELOG.md) | What changed in each release |
| [Design notes](https://github.com/Vinciepincie/bulba-pathfinder/tree/main/docs) | How the parkour engine works |
| [Benchmark](https://github.com/Vinciepincie/bulba-pathfinder/blob/main/bench/arena/README.md) | 60 routes on a real Paper server, including hard parkour courses. The `parkour` preset finishes 59 of them. |

## Optional physics patch

The prismarine-physics that mineflayer uses differs from the game in two places: it cannot climb nether and cave vines,
and a 1.21 jump off a slime rebound loses the rebound. Without the patch, paths leave those vines out and plan jumps for
the physics your bot actually has. To make prismarine-physics match the game in both places:

```sh
npx bulba-pathfinder-patch-physics
```

It edits prismarine-physics in your `node_modules`. Add `"postinstall": "bulba-pathfinder-patch-physics"` to your
scripts to keep it applied after every install.

## Development

```sh
npm install
npm test
npm run test:pack   # checks the packed tarball from ESM, CommonJS and TypeScript projects
```

To release, bump the version with `npm version`, push, and publish a GitHub release named after the tag, such as
`v0.1.0`. The release workflow publishes it to npm.

## License

MIT. Goals, movement costs and the plugin API are ported from
[mineflayer-pathfinder](https://github.com/PrismarineJS/mineflayer-pathfinder). The design also draws on
[azalea](https://github.com/azalea-rs/azalea) and [Baritone](https://github.com/cabaletta/baritone).
