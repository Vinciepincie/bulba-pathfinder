# Arena: real-world pathfinder benchmark

`bench/bench.ts` measures the solver on synthetic voxel worlds. This measures
the whole thing — solver, executor, and physics — against terrain nobody
designed to be walkable: **2b2t spawn**, on a real Paper server, with
`@bulba/pathfinder` and upstream `mineflayer-pathfinder` racing the same
routes side by side.

Two bots, two Node processes, one start block, released together. You can join
and watch.

## Requirements

- Java 21+ (Paper 1.21.11)
- Node 18+
- A copy of 2b2t spawn. Any World Downloader archive works: point the trimmer
  at the `.zip` directly, or at an unpacked save folder.
- ~1.5 GB free disk (trimmed world 78 MB, Paper + upgraded world the rest)

## Quick start

```bash
npm run build                                   # the race loads dist/, not src/
npm run arena:world  -- --src "path/to/2b2t_org.zip"
npm run arena:server -- --upgrade --via
npm run arena:race   -- --wait-for 1
```

1. **`arena:world`** keeps every chunk within `|x|,|z| <= 750` of spawn and
   drops the rest: 8836 chunks, 78 MB, out of a 6.8 GB download. Chunk
   payloads are copied byte-for-byte, so a 1.12 archive stays a 1.12 world.
   Reads ZIP64 archives directly, so there is no 7 GB unpack step.
2. **`arena:server`** fetches Paper (checksum-verified), writes the server
   config, and `--upgrade` converts all 8836 chunks to 1.21.11 once, so no
   run pays DataFixer cost mid-benchmark. `--via` adds ViaVersion so you can
   spectate from a newer client.
3. **`arena:race`** boots the server, connects the bots, and runs the route
   book. `--wait-for 1` holds until you have joined.

To race a single pair of coordinates without touching the route book:

```bash
npm run arena:race -- --from 91,145,150 --to 19,132,176 --wait-for 1
```

Add `--save-route r02` to keep it.

Everything generated lives in `bench/arena/.run/` (gitignored). Only
`routes.json` is committed.

## No plugin required

Everything the benchmark needs from the server is a vanilla command run by an
opped referee bot: teleporting racers and spectators, turning off collision
between the two racers, freezing time, weather, fire and mobs, and drawing the
world border at the trimmed edge. A Paper plugin would add a Java toolchain to
the setup and buy nothing.

## What a run measures

**By default a route is one step.** The bots race, and the planning cost is
read out of that same run (the `1st solve` column). Nothing is solved before
the start.

`1st solve` is each engine's own `time` field, and the two do not cover the
same window: upstream's runs from the start of the search, the wasm core's
starts inside the worker, after the snapshot is built and shipped. `1st move`
is the wall clock from launch to the bot leaving the start block, so it counts
everything either engine does first, whoever does it and wherever.

**Planning phase** (`--mode both`, no movement) — each engine plans from the
start block N times: best/median/worst solve time, nodes visited, path
length, path cost.

It runs **after** the race, deliberately. Timing a warm solve means driving
the live path, which warms the custom engine's snapshot cache; measuring
first would hand it a pre-loaded first solve in the race it never earned,
while upstream has no cache to warm. That is a free head start on the wall
clock, so every engine starts the route cold.

Reported **cold and warm**. Cold builds a fresh world snapshot per solve (what
the synchronous API does); warm drives the live path, which reuses the cached
snapshot a running bot actually has. The custom engine copies a box of terrain
into flat arrays before searching — that is what makes each expansion cheap
enough to run in a worker — and on a short search you pay for the whole box
and use almost none of it. On one 106-node route the cold figures were
upstream 27.1 ms against 34.9 ms here, while the same search warm took 3 ms.
Upstream builds no snapshot, so its two columns should agree; that agreement
is the control.

Both phases drive the **synchronous** API for the cold column, which is the
JavaScript solver in every engine, so `bulba` and `bulba-wasm` produce
identical cold numbers (that they match to the node is the differential suite
doing its job). The wasm core only runs on the live async path, so the warm
column and the race's `1st solve` are what separate them.

**Race phase** — both bots are teleported onto the same start block facing
the goal, held through a countdown, then released in the same instant:

| metric | meaning |
| --- | --- |
| `outcome` | `arrived`, `no-path`, `think-timeout`, `timeout`, `died`, `gave-up` |
| `wall` | the headline: start to arrival |
| `1st solve` | how long the first complete plan took, as the engine reports it |
| `1st move` | launch to the bot actually leaving the start block |
| `solves` / `replans` | how often the engine had to think again mid-walk |
| `visited` | total nodes expanded across every solve |
| `blocks` | distance actually travelled |
| `jumps` | takeoffs, a proxy for how much parkour was attempted |
| `dmg` | fall damage taken |
| `left` | distance still to the goal when the run ended |

Results are printed as tables and written to `.run/results/arena-*.json`.

## Auto-debug

A route that ends `timeout, 29.2 blocks left` is a score, not a bug report. So
when a route goes badly the arena stops and collects the evidence before
anything moves the bots again, and writes it to
`.run/debug/<stamp>-<route>-a<attempt>/NOTES.md`.

It triggers (`--debug auto`, the default) when any of these is true:

- one of our engines did not arrive,
- one of our engines stalled for 3 s or more, even on a route it won,
- upstream arrived where we did not, or beat us by more than `--faster-by`
  (10% by default).

`--debug always` writes a bundle for every route, `--debug off` writes none,
and `!debug` in the interactive session writes one for the last race on
demand.

**Why it runs inside the route.** The two facts that actually explain a loss
have a short shelf life. The path the executor was still holding is gone the
moment the goal is cleared, and the blocks around the failure can only be read
from the racer that failed — the referee sits at the finish on a 2-chunk view
distance and has never loaded them. Both are captured in place, at the stall.

A bundle contains:

| | |
| --- | --- |
| why it was flagged | the verdict lines, worst engine first |
| what each engine did | one row per engine: solve, first move, plan size, blocks walked, jumps, replans, worst stall |
| plan vs practice | where the two engines' *plans* first differ, how far apart their *lines* ever got, and whether we left upstream's corridor at all |
| where it got stuck | every stall over 2.5 s: the position, how long, how many nodes were still queued, and the next nodes the executor was trying to reach with their parkour flags and costs |
| the blocks it was stuck against | an ASCII slice per Y layer around the stall with the bot and the path nodes marked, plus full state (`properties`, collision `shapes`) for the feet/under/head blocks and each node ahead |
| where to look | source files chosen from the failure's shape, not a generic checklist |
| reproduce | two ready-to-run commands: the whole route, and just the segment it failed on |

`bundle.json` next to it carries the raw `RouteReport`: the per-tick trace,
every `path_update`, reset reasons, stalls and the terrain probe.

Read `shapes` rather than `boundingBox` when a bot comes to rest at a
fractional Y — `boundingBox` reports `block` for carpets, slabs and snow
layers alike, so only the collision shapes explain it.

### The loop

The arena server is the slow part, so keep it up and iterate against it:

```sh
npm run arena:race -- --routes simple2 --timeout 25 --keep   # race, flag, bundle
# read .run/debug/<stamp>-simple2-a1/NOTES.md, change src/, then:
npm run build                                                # the racers load dist/
npm run arena:race -- --attach --routes simple2 --timeout 25 --keep
```

`--debug-cmd` hooks an agent into that loop: it runs the given command with
the bundle's `NOTES.md` path appended, e.g. `--debug-cmd "claude -p"`. It is
off by default, and the racers keep running while it works — a fix still needs
`npm run build` and a re-race before it is in play.

### Route history

Every flagged run also appends to `history.json`, next to the route book:
verdict, both engines' numbers, whether the plans matched, and where we got
stuck. It is committed rather than left under the gitignored `.run/`, because
the signal worth having is "this route has wedged in the same spot four runs
running", and that only exists across runs.

## How the comparison is kept fair

- **One process per racer.** Upstream solves on the main thread in per-tick
  slices; the custom engine solves in a worker. In a shared process the
  upstream slices would stall the other bot's packet handling, and the
  wall-clock number would measure the harness.
- **The built package.** The custom engine is loaded from `dist/`, exactly
  what production runs (worker thread, wasm core). Loading `src/` through tsx
  silently falls back to main-thread solving.
- **Identical movement rules**: no digging, no block placement, no scaffolding,
  same `maxDropDown`, same think timeout. The single deliberate difference is
  `allowParkourExtended`, which is the feature under test — `--parity` turns
  it off for an apples-to-apples run.
- **Identical start state**: same block, same facing, hunger topped up with a
  saturation effect (sprint-jumps need food > 6, and a long benchmark would
  otherwise starve the bots into a walk halfway through).
- **A frozen world**: no daylight cycle, weather, fire tick, mob spawning,
  random ticks or entity drift between runs.
- **No shoving**: each racer is on its own `collisionRule never` team, so
  sharing a start block costs nothing.
- **Endpoints snapped once, centrally.** Coordinates get written down two ways
  (the block you stand *in* and the block you stand *on*), so each endpoint is
  nudged to the nearest block with a floor and two blocks of headroom. One
  racer computes the correction and both are given the same answer, so the two
  engines are never asked to solve slightly different problems. The authored
  and resolved coordinates are both in the JSON.
- **Arrival is measured, not reported.** `outcome` comes from where the bot
  actually ended up, never from whether `goto()` resolved. Upstream's `goto`
  resolves *successfully* on any `path_update` carrying an empty path, and its
  `noPath` results carry `path: []` — so an unreachable goal reports "arrived"
  in 90 ms without the bot moving. Scoring the promise would hand upstream a
  win for giving up fastest. When the two disagree the run records both
  (`outcome` vs `promiseOutcome`).
- **Everyone launches on the same instant.** The go messages are sent in
  order and each racer is its own process with its own event loop, so "start
  when you get this" gave whoever was told first a visible head start off the
  line. Each racer now waits for a shared wall-clock instant instead, and the
  spread between their actual launches is measured and reported as
  `start skew` — if it ever exceeds a tick, the wall times are suspect.
- **Chunks land before anyone races.** Racers are parked in spectator above
  the start while the area streams in, and the hold only lifts once no chunk
  has arrived for 2 s. Both engines drop their path on every chunk that lands,
  so starting early measures chunk delivery, not pathfinding.

## Adding routes

2b2t spawn is mostly lava, void and unclimbable towers, so a lot of
good-looking endpoints are genuinely unreachable on foot. Scout first:

```bash
npm run arena:probe -- --from 91,145,150 --min 40 --max 150
npm run arena:probe -- --from 91,145,150 --save p    # save the reachable ones
```

The probe samples destinations on rings around the start, finds a standable
block in each column, and asks the custom engine whether it can get there. It
is a fast oracle for this: on a genuinely unreachable goal it returns
`noPath` in a fraction of a second where upstream just burns its whole think
budget.

It is a scouting tool, not a route source. The 39 probe-generated routes it
had filled `routes.json` with were all spokes off the same start block and
told us little; the book is hand-authored now.

### Defining routes with signs

The nicest way: put the route in the world itself.

```bash
npm run arena:edit          # same as: npm run arena:race -- --edit
```

You get creative mode and a stack of signs. Write on one:

```
!PF
Basic 1 with waterfall
Start
parkour-simple
```

and on another at the destination the same thing with `Finish`. Line 1 is the
marker, line 2 is the route name, line 3 is the role, line 4 is an optional
scenario. **Everything is case insensitive**, so `!pf` / `START` / `finish`
all work, and a Start and Finish pair up by name regardless of capitalisation.

Then, all from chat:

| | |
| --- | --- |
| `!scan` | re-read every `!PF` sign in the world |
| `!save` | write the sign routes into `routes.json` |
| `!race <name\|id>` | run that route right now, everyone watching |
| `!race all` | run the whole book |
| `!list` · `!tp <id>` · `!drop <id>` | manage saved routes |
| `!stop` | end the session |

`!race` looks a route up by name or id, in `routes.json` first and then in the
world, so you can place two signs and race them without saving anything.

The same command set is available after a normal run with `--keep`, so you can
watch a batch finish and then re-run a single route on the spot.

A route defined this way is visible in-world, survives restarts, and shows
what it was meant to test. Scanning reads the region files rather than
sweeping bots around — 8836 chunks in about a second — so it covers the whole
arena, not just one view distance. `!scan` flushes the world first so freshly
placed signs are included.

To race the signs directly without saving them:

```bash
npm run arena:race -- --signs                              # all of them
npm run arena:race -- --signs --routes "Basic 1 with waterfall"   # just one
```

`--routes` matches a route's id *or* its name, case insensitively.

### Defining routes by hand

Edit `routes.json`:

```json
{ "id": "r02", "name": "lavacast hop", "scenario": "walk",
  "start": [91, 145, 150], "end": [32, 184, 121] }
```

Coordinates are **feet blocks** — the F3 "Block" readout where you are
standing. Scenarios: `walk`, `parkour-simple`, `parkour-advanced`, `mixed`.

`worldDigest` in `routes.json` is the digest `arena:world` prints. Two people
with the same digest trimmed the same terrain and can compare numbers.

## Watching

Join `127.0.0.1:25599` with any username (offline mode). You spawn as a
spectator, and the referee teleports you to each start block before the
countdown, so you see both bots leave together. Race results are also
announced in chat.

The referee itself waits at the **finish**, floating two blocks above the
destination and facing back down the route. It marks where the bots are
headed, and gives you a second vantage point to teleport to mid-race if you
would rather watch them arrive than watch them leave.

Spectating from a client newer than 1.21.11 needs the `--via` step. The bots
always speak 1.21.11 natively — the thing under test never runs through a
translation layer.

## Flags

`arena:race`:

| flag | default | |
| --- | --- | --- |
| `--edit` | off | author routes in-world with signs instead of racing |
| `--signs` | off | race the `!PF` signs in the world, not `routes.json` |
| `--engines upstream,bulba-wasm` | all three | pick which engines race |
| `--from x,y,z --to x,y,z` | — | race one ad-hoc route, no route book needed |
| `--save-route <id>` | — | also append that ad-hoc route to `routes.json` |
| `--routes r01,r02` | all | pick routes by id |
| `--scenario walk` | all | pick routes by scenario |
| `--mode race|both|solve` | `race` | `both` adds a separate planning phase, run after the race |
| `--repeat N` | 1 | attempts per route |
| `--solve-repeats N` | 5 | planning-phase samples |
| `--timeout S` | 120 | seconds before a race is abandoned |
| `--think MS` | 5000 | per-solve think budget |
| `--tolerance N` | 0 | goal radius; 0 means the exact block |
| `--max-drop N` | 4 | `maxDropDown` for both engines |
| `--countdown S` | 3 | seconds held before "GO" |
| `--wait-for N` | 0 | hold until N spectators have joined |
| `--parity` | off | disable extended parkour (upstream-equivalent rules) |
| `--attach` | off | use a server that is already running |
| `--keep` | off | leave the server up after the run |
| `--debug auto\|off\|always` | `auto` | write a debug bundle for a flagged route |
| `--faster-by N` | 0.10 | upstream lead that counts as a loss for us |
| `--debug-cmd "<cmd>"` | — | run this with the bundle's `NOTES.md` appended |

`arena:world`: `--src`, `--out`, `--radius`, `--spawn x,y,z`, `--force`.
`arena:server`: `--upgrade`, `--via`.
`arena:probe`: `--from x,y,z`, `--min`, `--max`, `--rings`, `--spokes`,
`--save <id prefix>`, `--attach`.
`arena:edit` is just `arena:race -- --edit`.

Environment: `ARENA_PORT`, `ARENA_HOST`, `ARENA_MC_VERSION`, `ARENA_RADIUS`,
`ARENA_VIEW_DISTANCE`, `ARENA_HEAP`, `ARENA_RUN_DIR`.

## Caveats

- Both bots share one server, so a server hitch hits both. `/tick query` is
  sampled per route and recorded as `tickMs`; treat a run with a bad tick time
  as suspect.
- Wall-clock includes execution luck (a missed jump costs a replan). Use
  `--repeat` for anything you intend to quote.
- Digging is off by design. Routes that need a block broken are unreachable
  for both engines, which is a valid `no-path` result, not a bug.
- If every engine reports `no-path`, the goal block probably is not
  stand-able. Retry with `--tolerance 1`.
