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
upstream) with the exact upstream cost model when enabled. The **planner**
never generates placement moves: with digging off, the only block interaction
it performs on its own is activating a door or fence gate on the path.

What the bot does *deliberately* is separate from what the planner does on the
way, and that half is a first-class API: [`dig` / `place` / `open` /
`activate`](#interactions-dig-place-open-activate) are convenience methods
that walk into vanilla range only if they have to, guard every click the way
the server judges it, and retry. They run through one **overridable table**
that the executor's own digs and doors go through too, and hand off to other
plugins through a [cooperative interrupt
protocol](#interrupts-borrowing-the-bot-mid-path) — so auto-eat stops
happening in mid-jump.

```js
import { pathfinder, Movements, goals } from '@bulba/pathfinder' // named ESM imports…
// const { pathfinder, Movements, goals } = require('@bulba/pathfinder') // …or CJS
// import pkg from '@bulba/pathfinder'  // …or the upstream default-import style

bot.loadPlugin(pathfinder)
const movements = new Movements(bot)
bot.pathfinder.setMovements(movements)
await bot.pathfinder.goto(new goals.GoalNear(x, y, z, 1))

// …and the interaction half
await bot.pathfinder.dig(new Vec3(x, y, z))
const chest = await bot.pathfinder.open(chestBlock)
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

## Interactions: `dig`, `place`, `open`, `activate`

`goto` gets the bot somewhere. These get something **done** — and they walk
into range only when they have to, so a block already within vanilla reach is
acted on from where the bot stands, with no goal set and no path computed.

```js
await bot.pathfinder.dig(vec3OrBlock)                      // break it
await bot.pathfinder.place(vec3, { item: 'red_shulker_box' })
const window = await bot.pathfinder.open(vec3OrBlock)      // any window, not just containers
await bot.pathfinder.activate(vec3OrBlock)                 // lever, button, door
```

They are serialised against each other and hold an exclusive
[interrupt](#interrupts-borrowing-the-bot-mid-path) for the interaction
itself, so the tick loop is never fighting the hand — and neither is anything
else that plays by the same protocol.

**`open` is not a container API.** `bot.openContainer` asserts the window
against a chest/shulker/barrel allowlist and throws on anything else, which
makes it useless for a crafting table, an enchanting table, an anvil, a
furnace or a beacon. `pathfinder.open()` returns whatever window the block
opens — mineflayer's `Window`, already extended with
`close`/`deposit`/`withdraw`.

### What the built-ins guard against

Every one of these is a real failure that presents to the application as "the
bot just stood there", because the server's answer to a bad click is silence:

| Guard | What it stops |
|---|---|
| **Vanilla range**, eye to the *closest point of the cell* | Measuring centre-to-centre thinks it can reach ~0.87 further than the server does on a diagonal. The click is dropped, silently. |
| **Face agreement** | The clicked face is the one the bot's own raycast reports — the face the server derives from the same rotation. A geometric guess only survives point-blank, and only with `allowGeometric`. |
| **Aim settle** | Rotation reaches the server in a position packet. Clicking on the same tick as the look sends the click with the *previous* rotation attached. |
| **Grounded stance** | A swing sent while falling or sliding is sent from a position the server has not accepted yet. |
| **Correction window** | Nothing is clicked within `forcedMoveGrace` ms of a lagback. |
| **Decay guard** | Leaves and falling blocks change under a slow aim; the block is re-read before the swing. |
| **Mid-dig abort** | A lagback, the footing decaying away, or being pushed cancels the dig instead of mining a block the bot can no longer see. |
| **Verify by the world** | `bot.dig` resolves on its own timer and `bot.placeBlock` *rejects* on a laggy server after the placement landed. Neither promise is evidence; the block is. |
| **Sneak on placement** | Right-clicking a chest/hopper/barrel/crafting table with a block in hand **opens it** instead of placing. Vanilla's answer is to sneak, and so is ours — without it a shulker station on a hopper is unplaceable. |
| **Shulker openability** | A shulker opens along the face it was placed against. Faces whose opposite side is obstructed are skipped, so the bot cannot brick its own storage. Turn off with `requireOpenable: false`. |
| **Bounded `open`** | `bot.openBlock` is `activateBlock` plus a bare `once(bot, 'windowOpen')` with **no timeout**: a dropped click hangs the caller forever, and the abandoned listener later steals a window opened by something else. Ours times out, says whether the server refused the click or never saw it, and removes the listener. |

Failures throw with stable `name`s — `OutOfReach`, `Occluded`, `NoTarget`,
`DigFailed`, `PlaceFailed`, `OpenFailed`, `MissingItem`, `Unreachable`,
`ActionAborted` — matched the way the upstream `NoPath`/`Timeout` names are.
`err.refused === true` means the action stopped short and a reposition-and-
retry is reasonable; anything else is structural.

### Pacing

Breaks and placements share one jittered cooldown window, because a checker
counting block actions does not care which kind they were. Vanilla itself
enforces a 5-tick `destroyDelay` between breaks, so anything faster is outside
what a client produces.

```js
bot.pathfinder.actions.config.pacing = 'default' // ~0.6–0.9s, breather every 4th (the default)
bot.pathfinder.actions.config.pacing = 'drill'   // fast, still above vanilla's floor
bot.pathfinder.actions.config.pacing = 'none'    // no spacing at all
```

`'none'` is the right choice on a server you control, and for one-off
interactions. The setting also applies to the executor's own `canDig` breaks.

### Overriding

Every world interaction the pathfinder performs — including the executor's own
`canDig` dig branch and the doors it opens while walking — goes through one
table, so replacing an entry replaces it **everywhere**:

```js
bot.pathfinder.actions.dig      // (block, options) => Promise<void>
bot.pathfinder.actions.place    // (vec3, options)  => Promise<void>
bot.pathfinder.actions.open     // (block, options) => Promise<Window>
bot.pathfinder.actions.activate // (block, options) => Promise<void>
bot.pathfinder.actions.equip    // (item, dest)     => Promise<void>
bot.pathfinder.actions.config   // reach, retries, timeout, pacing, windowTimeout, forcedMoveGrace
```

Wrap rather than replace, and you keep the guards:

```js
const inner = bot.pathfinder.actions.dig
bot.pathfinder.actions.dig = async (block, options) => {
  if (protectedRegion.contains(block.position)) throw new Error('not there')
  metrics.digs++
  return await inner(block, options)
}
```

> **If your application monkey-patches `bot.dig`**, stop: the pathfinder
> captures `bot.dig` at inject time and calls that, so a wrapper which
> delegates back into the pathfinder cannot recurse — but a global patch also
> applies to every other consumer of the bot and cannot be scoped, ordered or
> removed. `actions.dig` is the same power without those problems.

Events, all namespaced so they cannot collide with upstream's:
`pathfinder:dig_start`, `dig_finish`, `dig_aborted`, `dig_error`,
`place_start`, `place_finish`, `open_start`, `open_finish`, `open_retry`,
`activate`.

## Interrupts: borrowing the bot mid-path

Every mineflayer stack hits this eventually. A second plugin — auto-eat, a
totem swapper, an armour manager — wants the bot's hands, has no idea what the
pathfinder is doing, and acts whenever its own trigger fires. If that moment
is the tick the executor committed to a sprint-jump, the bot eats in mid-air:
item use and sprinting cancel each other in vanilla, the held item is swapped
away from the tool the next node needs, and the bite lands the bot somewhere
the planner never routed through. The pathfinder then replans from wherever it
fell — which is how "the bot randomly walks back on itself" happens.

The fix is not to teach the pathfinder about eating. It is to let borrowers
**ask**, and to answer only where handing over is harmless:

```js
const handle = await bot.pathfinder.interrupt('totem-swap')
try {
  await bot.equip(totem, 'off-hand')
} finally {
  handle.release()
}

// or, releasing for you even if the body throws:
await bot.pathfinder.withInterrupt('totem-swap', () => bot.equip(totem, 'off-hand'))
```

The path is **not** reset while a handle is held: no `path_reset`, no
re-solve, no `goto()` rejection. The bot stands still on its own path and
carries on from the same node. From the caller's point of view the walk just
took a second longer — and the stuck and execution timers are pushed forward
by exactly the pause, so a long hold is never mistaken for a stall.

- Granted on the ground, in water, or on a climbable — never mid-flight, never
  mid-dig, never mid-interaction. **"Wait before jumping" needs no special
  case**: a request arriving while the bot is standing is granted before the
  executor reaches the take-off decision.
- **Exclusive by default.** A bot has one pair of hands: while
  `pathfinder.dig()` holds them, an auto-eat request queues behind it instead
  of swapping the pickaxe out from under a swing. Pass `{ exclusive: false }`
  for a passive observer that only wants the bot to stand still.
- FIFO with head-of-line blocking, so a stream of observers cannot starve an
  exclusive request.
- `{ timeout }` (default 5 s) bounds the wait for a safe moment — a genuinely
  wedged bot still gets to eat. It does **not** override exclusivity.
- `pf.stop()` always outranks a pause.

`bot.pathfinder.motion` is the read-only view for anything that needs to time
around the executor: `phase` (`idle` / `walking` / `airborne` / `climbing` /
`swimming` / `digging` / `interacting` / `paused`), `critical`, `jumpPending`,
`paused`, `holders`, `node`. Events: `pathfinder:interrupt_requested`,
`pathfinder:paused`, `pathfinder:resumed`.

### Auto-eat

A ready-made adapter for
[`mineflayer-auto-eat`](https://github.com/linkle69/mineflayer-auto-eat). It
turns the plugin's own `physicsTick` eating off and drives it through the
interrupt protocol instead — the bot lands, stops on its path, eats, and walks
on from the same node.

```js
import { pathfinder, autoEatIntegration } from '@bulba/pathfinder'
import { loader as autoEat } from 'mineflayer-auto-eat'

bot.loadPlugin(pathfinder)
bot.loadPlugin(autoEat)
bot.loadPlugin(autoEatIntegration({ startAt: 16 }))
```

Options: `startAt` (food level, default 16), `healthBelow` (eat when hurt too,
default off), `checkEveryTicks` (20), `waitForSafe` (10 s before eating
anyway), `skipWhenWindowOpen` (true), `eatOptions`, `reason`. It is defensive
about the plugin's own shape — 3.x has shipped both `enableAuto`/`disableAuto`
and `enable`/`disable`, and both `opts` and `options`.

`bot.pathfinderAutoEat` gives you `eatNow()`, `suspend()` / `resume()`
(counted, for a caller that needs the hand), `eating`, `suspended` and
`detach()`.

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
- **The sprint-hop gait** (opt-in, `Movements.allowSprintHop`). Hopping while
  sprinting is how a player crosses open ground, and it is not a small
  difference — the jump keeps the sprint boost that ground friction eats:

  | gait | open sky | under a 2-high roof |
  |---|---|---|
  | walk | 4.30 | 4.30 |
  | sprint | 5.59 | 5.59 |
  | sprint + **hold** jump | 7.05 | 6.50 |
  | sprint + **press on landing** | **7.05** | **9.68** |

  The cadence is the interesting half. prismarine-physics charges a held jump
  key a 10-tick re-jump cooldown (`autojumpCooldown`) and clears it the moment
  the key comes up, so a bonked arc — one that lands after five ticks because
  the ceiling is low — spends the rest of the cooldown standing still, bleeding
  the boost into friction. Pressing on each landing instead makes a 2-block
  roof the **fastest ground in the game**, quicker than bunny-hopping under
  open sky, and where the arc already outlasts the cooldown the two cadences
  are identical tick for tick (34 jumps each). So it is never the slower
  choice, and it is not behind a flag.

  There is exactly one shape of ground where holding wins, and the gait never
  meets it: a staircase with treads wide enough that re-pressing on the
  landing fires the next jump straight into the riser. On 3-block treads that
  is 60.8 blocks per 200 ticks held against 41.2 pressed — but the gait
  refuses to leave the ground into rising terrain at all (a rise is the jump
  gates' business), so the cadence is never asked the question there. A
  rollout that picks between the two cadences was built and measured, and it
  never once chose to hold; that is why there is one cadence rather than a
  choice. Worth knowing if the rise veto is ever relaxed.

  `Movements.allowLowCeilingHop` (opt-in, needs `allowSprintHop`) is what lets
  the bot go and find that ground. It changes only how far ahead a DROP in the
  ceiling vetoes a take-off: off, that is the whole comparison horizon, which
  is safe and also switches the gait off for a passage that is mostly 2-high,
  because every 3-high pocket has a low section within six nodes; on, the veto
  spans the arc's own footprint, so the bot sprints the last step into a low
  section and hops the moment it is under it — and still never takes off from
  high ground into a ceiling it would hit side-on, which is the case that jams.

  Nothing hard-codes any of that, though — both
  gaits are driven down the SAME path for the same horizon and the faster one
  wins, so the bot declines the hop from a standstill (no boost to keep yet)
  and takes it once it is moving, all from the same comparison. It is refused
  outright when the rollout would leave the ground the path stays on, when the
  ground rises, on the last stretch before the goal, when the roof drops
  within a hop's reach (the body is already up at 1.25 when the low section
  arrives, which jams rather than bonks), and when there is water or lava in
  the airspace three blocks up — which the planner never looks at, because
  walking never goes there. Worth 0.5–1.9 s per route on the arena's flat
  routes; `bulba-nohop` races the same engine with it off, so the number is
  measured rather than argued.
- **Ice, slime and potions are already in the answer, because the gates are
  rollouts.** Nothing in the executor knows what ice is. Every gate simulates
  the LIVE `PlayerState`, so block slipperiness and the server's
  `movementSpeed` attribute are inputs to the comparison rather than special
  cases — and they point opposite ways, which is why it matters that it is
  measured and not assumed:

  | surface | walk | sprint | sprint-hop |
  |---|---|---|---|
  | stone | 4.30 | 5.60 | 7.07 |
  | ice / packed ice | 4.10 | 5.33 | **9.11** |
  | blue ice | 4.31 | 5.60 | **9.19** |
  | slime | 3.20 | 4.16 | 7.89 |
  | stone, Speed II | 6.03 | **7.83** | 7.58 |

  Ice is *slower* to walk and sprint on and 29% faster to hop on, so the bot
  hops there without being told to. Under Speed II plain sprinting overtakes
  the hop, so it stops hopping — again without being told. Two gaps are worth
  knowing about: the SEARCH prices every surface the same (it will not seek
  out an ice highway or route around slime), and prismarine-physics gates its
  soul-sand/honey slowdown on a `velocityBlocksOnTop` feature whose version
  list stops at 1.20, so on 1.21 the rollouts believe soul sand is ordinary
  ground.
- **Corner cutting** (`Movements.allowCornerCut`, on). The planner routes cell
  centre to cell centre over eight directions, so a run a few degrees off a
  cardinal comes back as an alternating zig-zag — and a follower that steers
  at each centre walks every zig and swings its heading ±45° at every one. The
  executor instead pulls the string taut: it steers along a straight line to
  the furthest node the body fits down (full hitbox sampled every quarter
  block, floor under every sample, no liquid or avoided block in any cell),
  up to 32 blocks out, and the nodes it skips retire as the body draws level
  with them rather than by being stood on. A run four blocks across and
  thirteen along is planned as a 45° leg and a straight leg; the bot walks
  the line between the two ends. The plan is untouched, so turning it off
  restores node-by-node following exactly. Measured on the arena against the
  bounded cut it replaced: simple2 14.5 → 13.9 s, simple3 8.9 → 8.2 s, basic2
  17.1 → 16.2 s, tunnel1 12.0 → 10.4 s, nothing slower.

  The line is *tracked*, not merely aimed at. It is fixed when picked, its
  anchor slides along it under the body, and the bot steers at a point 1.5
  blocks up the line from its own foot (pure pursuit), so a body knocked
  sideways is pulled back at a strong angle. Steering straight at a node 30
  blocks out has almost no such feedback, and the first taut cut showed why
  that matters: a hop taken into a turn carried the old heading's momentum
  1.4 blocks off the line and off a one-wide ledge (simple1, 19.9 s and a
  7-block fall). Pure pursuit alone removed it (10.7 s). A side-margin floor
  check was tried for the same fall and measured as the wrong tool on this
  terrain — half a block of required ground refused most lines and gave back
  every gain — so it is off, kept as `PF_CUT_SIDE_MARGIN` for A/B runs. A cut
  also ends five nodes short of any jump, so it ends by reaching a node on
  the path and the take-off is approached on the planner's own cells.

  Pure pursuit alone was not enough, either: the same corner still fell one
  run in two, because the drift happens in the AIR. The bot landed at the
  corner at full sprint, picked the 45° line and hopped on that very tick,
  and a hop carries the old heading's momentum for a whole arc with 0.02 a
  tick of air control — it flew at 30°, came down 0.64 off the line in the
  gap between two cells of the ledge, and the rollout had not seen it because
  its yaw is set instantly. So a sprint-hop never takes off into a turn: with
  the commanded heading more than 20° off the body's velocity the bot sprints
  the turn (ground friction realigns it in three or four ticks) and hops once
  aligned. Nine of nine clean runs since, at 10.6–10.7 s against 11.1 s on the
  bounded cut. `PF_HOP_TURN_DEG` overrides the angle for A/B runs.
- **Take-offs that cannot be made are given up, not retried forever.** On
  the arena's `basic3` the planner priced a 4×4 diagonal jump from a cell
  hemmed in by a two-high pillar; the run-up could not line up on the flight
  line, no rollout ever signed a take-off, the futility timer replanned and
  the same jump came straight back — three replans and a fall, 24 s against
  12 s on the plan that avoids it. Four things changed. The run-up now lines
  up *on* the take-off centre line (pure pursuit while backing off and
  running in), which is where the frame-tight lip take-off exists; the creep
  anchors on the take-off node rather than on whatever cell the body is in
  (after a run-in it can already be over the lip, and the offsets were a
  block out); it sneaks near any edge of the block, not only near the lip on
  the flight line; and a lip the executor has waited at for 1.5 s with every
  line-up spent is given up: the landing cell is banned for two minutes as a
  wall-weight in the snapshot both solver cores read, and the route is
  re-solved round it. The re-solve also starts from the block the body stands
  on rather than the column under its centre, so a body overhanging a lip is
  not planned as if it were already in the gap. `basic3` now arrives in 15.7 s
  on the plan that used to stall, every time.
- **Landings on posts are braked.** A body landing on a fence post or a head
  with its flight speed still in it slides 0.57 blocks before friction stops
  it, past the 0.425 a post supports. The planner meant such a landing to
  re-jump on the landing tick, and when the rollouts sign that it does; when
  they do not, the run-up used to begin with `back` under sneak — a third of a
  brake — and the body went off the far edge (`parkouradv1`, one run in two
  before, three in three after the taut cut moved the landing point). Now the
  body brakes with full `back` and no sneak until it is walking-slow, then
  lines up. A damaging landing also sits out two ticks rather than one, since
  the server's velocity packet arrives a full server tick after the landing
  it answers. `parkouradv1` arrives three in three at 17.0–17.2 s.

  How the line is found: on the tick a cut is picked the executor scans eight
  nodes ahead, and then *extends* the target by up to four nodes every
  grounded tick while walking, so the aim reaches the visibility horizon
  within a few ticks without any one tick paying for a long scan. A target is
  only ever traded for one further along the same path, which is why the
  heading turns once, onto the line, instead of flickering as the swept-box
  test wobbles near terrain. A body in the air keeps the target it took off
  with, because a re-pick mid-hop swings the yaw with only air control to
  answer it. The cut never runs into a jump — it leaves the body lined up on
  the chord rather than on the take-off, which cost 0.3 s a route on the
  jump-dense ones.

  What the physics gates see: under a live cut the sprint, hop, grind and jump
  rollouts are driven down the *chord* (synthetic nodes a block apart, then
  the real path beyond the target) rather than at `path[0]`, which a taut cut
  leaves several blocks off to the side. "Gone by" is measured on the chord
  the body is walking, never along the path's own next leg (where the window
  can be empty outright) and never against velocity (which the wedge recovery
  reverses). A server correction of under two blocks with a cut live keeps
  the cut; the node-distance test the lagback handler uses would read the
  body's legitimate offset from the zig-zag as "snapped away" and replan.
- **Parkour landings are landed on.** The arrival box has no ground
  requirement, so a jump that passes over its node on the way down satisfies
  it in mid-air — and the
  executor then starts steering at the node AFTER it, which past a gap means
  steering into the gap. On the arena's `basic1`, a 4-block drop onto a 2-cell
  shelf was retired 0.9 blocks above the shelf and the bot spent the rest of
  its fall thrusting at a node across the chasm beyond: a 40-block fall, about
  one run in eight, on either gait. A parkour node is now held until the body
  is supported — on the ground, in water, or caught on a ladder or vine, which
  is how the extended repertoire's gap-jumps end — and only where there is
  actually a hole past the landing, since a landing with more ground beyond it
  can be overrun harmlessly and holding those too costs a tick a jump. Worth
  0.8 s on `basic1` on top of removing the fall.
- **Parkour take-offs are decided on the ground.** Asked while the bot is
  still falling out of the previous jump, the rollout has to guess the speed
  it will land with, and a marginal jump is decided entirely by that number.
  On the arena's `basic1` — a 5-block drop-jump immediately followed by a
  4-block flat one across a waterfall chasm — that guess was right about two
  runs in three and cost a 40-block fall the rest of the time. Waiting costs
  nothing: a jump can only start from the ground anyway, the gates re-run
  every tick, and the in-flight branch keeps the current jump flying. 5/5
  clean afterwards, from 4/5 with a death.
- **Turning to make a jump, before shuffling to make room for it.** When a
  node cannot be reached, the executor searches take-off headings (±30°, 0
  first, cached per node) for one that lands it — the idea behind
  [ParkourCalculatorMod](https://github.com/Leg0shii/ParkourCalculatorMod)'s
  angle solver, at a hundredth the scope. Only then does it fall back to
  stepping back or sideways, one tick at a time and each simulated first,
  because a heading that works costs nothing while ground given up has to be
  walked again. A fixed four-tick back-off instead of per-tick nudges was
  worth a 0.9-block sideways wobble on every step of a staircase — 14 blocks
  of extra ground and 3.5 s on one arena route.
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

Interaction defaults live on `bot.pathfinder.actions.config` and apply to the
next action — the convenience methods and the executor's own digs and doors
alike. Per-call options override them:

```js
bot.pathfinder.actions.config = {
  reach: 4.5,             // vanilla block-interaction range
  retries: 3,             // attempts per action
  timeout: 30_000,        // overall deadline per action, ms (0 = none)
  pacing: 'default',      // 'none' | 'default' | 'drill'
  windowTimeout: 3000,    // ms to wait for windowOpen
  forcedMoveGrace: 800,   // ms after a lagback before anything is clicked
}
```

Opt-in `Movements` flags (all default **false**, so the walking outcome
matches upstream until you ask for more):

| flag | what it buys |
|---|---|
| `allowParkourExtended` | the full sprint-jump repertoire (`docs/ExtendedParkour.md`) |
| `allowParkourMomentum` | needs `allowParkourExtended`; momentum-aware search — a landing on a narrow support carries its speed into the search node, so chained re-jumps of any length plan as states, and running take-offs are credited from the lip (`docs/VelocityInSearch.md`) |
| `allowSprintHop` | the hopping gait — 7.05 blocks/s against 5.59 sprinting, taken only where a rollout of both gaits down the same path says the hop gets further without losing height |
| `allowLowCeilingHop` | needs `allowSprintHop`; hop through low headroom instead of round it — 9.68 blocks/s under a 2-block roof |

`allowCornerCut` defaults to **true**: it changes only what the executor
steers at, never the plan, and node-by-node following is restored exactly by
turning it off.

`allowLowCeilingHop` is **not** a cadence switch — it does not choose between
holding the jump key and re-pressing it. The gait always re-presses, with the
flag off as well as on; what the flag changes is only how far ahead a drop in
the ceiling vetoes a take-off. See the sprint-hop section above.

## Scope

Water walking, ladders, doors/gates, sprinting, parkour, drop-downs, entity
avoidance and (opt-in) digging are all supported — the full upstream moveset
except block placement, which is permanently out of scope **for the planner**:
no move generator emits a placement, so no route can depend on the bot
building one, and the solver writes zero place packets by construction. Dig
packets are written only when `canDig` is explicitly enabled.

Deliberate, caller-driven interaction is a different thing and is in scope:
`pathfinder.place()` places exactly the block you asked for, where you asked
for it, and nothing plans around it.

**Water**, specifically, is where upstream's executor and its own planner
disagree, and the executor is fixed here rather than the search:

- The swim branch is keyed on the **head** cell, not on `isInWater`. Upstream
  bobs across a one-deep ford with sprint cancelled because the body is merely
  touching water; wading and swimming are different things.
- Jump is held only while the node is **at or above** the body. Upstream holds
  it for as long as the bot is wet, which climbs a column the search never
  routed through — there is no vertical water move in the move generator at
  all (`moveUp` refuses a liquid feet cell, `moveDown` refuses to go under),
  so every water node sits at one planned level. On the arena's `basic1` that
  was the difference between crossing a waterfall and riding it upward.
- The bot **surfaces for air** below a third of its lungs, and does not sink
  while it thinks: a bot with no path sets no controls, which in water is two
  blocks a second downward for the whole re-solve, so every retry started
  deeper than the last. The futility budget is 8 s in water rather than 3.5,
  because swimming covers 2 blocks a second against 4.3 walking and a reset
  that releases jump is worse than the stall it was called for.

What is **not** modelled, and the executor only survives rather than solves:

- No breath budget and no flowing-water direction in the search, so it can
  still ask for a swim the bot has to abandon part-way.
- **No fall-risk model, and it cannot live in the executor.** Nothing anywhere
  consults `bot.health`: `maxDropDown` bounds a drop the planner *chose*, not
  the void a jump crosses. Refusing a fatal-miss take-off was tried and
  measured WORSE — on terrain like 2b2t spawn nearly every jump is over a
  void, so the veto refuses the route the planner insists on, the bot has no
  alternative to fall back to, and it wanders off an edge instead (arena
  `basic1`: 5/5 clean without it, 0/2 with it). It belongs in the search,
  where a fatal-miss edge can be priced against a safer line, not in the
  executor, which has only "yes" and "stand still".
- **No vertical water move at all.** `moveUp` refuses a liquid feet cell and
  `moveDown` refuses to go under, so the planner cannot express "ride this
  column down and climb out lower" — the obvious line for a player at a
  waterfall, and one the bot therefore never takes.

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
