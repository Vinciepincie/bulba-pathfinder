# Physics-verified hops

`Movements.allowParkourPhysics` enables simulated hop discovery when the
extended table search cannot complete a route. It defaults to false and
requires `allowParkourExtended`. The same physics kernel also supports live
take-off searches and the optional local gait scheduler.

The table in [ExtendedParkour.md](ExtendedParkour.md) is cheap enough for
ordinary A* expansion, but a fixed set of flight lines cannot describe every
neo, thin landing, rebound or ice chain. Here a candidate edge is a control
program flown tick by tick, including the velocity it brings to its landing.

## The movement kernel

[src/playerSim.ts](../src/playerSim.ts) implements the normal movement branch
of prismarine-physics: collision, step-up, sneak edge guarding, ladders,
slipperiness and slime rebound. It uses unit-vector headings and reusable
bodies to avoid trigonometry and allocation in the inner loop. Slipperiness
uses Float64 storage; rounding a value such as blue ice's 0.989 to Float32
changes the trajectory.

The kernel reads collision boxes from the snapshot grid. It caches repeated
cell-range queries and, on a fixed grid, occupancy of 4-by-4-by-4 bricks so
empty regions and dry regions can be skipped. Live grids carry invalidation
stamps because the world can change. A cached rollout must describe one
consistent world, not a mixture of old and new blocks.

Liquids, fire and cobwebs stop a rollout. The kernel does not simulate
swimming or web slowdown, and treating a body frozen at liquid contact as a
stable landing would accept lava. Elytra, levitation and slow falling are
outside this kernel's movement model.

The kernel matches the bot's jump rule. A rule that preserves
`max(jumpPower, upwardVelocity)` lets a grounded slime rebound keep its rise
while a sprint jump adds horizontal boost; a rule that replaces upward
velocity suppresses that rebound. `PhysicsSim.probeVanillaJump` probes the
bot's physics and selects the matching kernel rule, including in the worker.
Ordinary jumps are unaffected by this distinction.

## Control programs

[src/hopOracle.ts](../src/hopOracle.ts) searches `HopProgram`s. A phased
program gives a ground heading and jump time, a first air heading and turn
time, then a heading to touchdown. A heading is either a fixed direction or
a point re-aimed at each tick. Programs can also carry rebound headings,
an exact live starting state, or a per-tick control sequence from the beam.

The families cover different ways of using the same physics:

- `straight`, `run-turn` and `air-turn` vary approach, jump timing and the
  change of direction in flight.
- `yaw` flies out along a pillar's face and back behind it for a neo.
- `kick` separates the run direction from the facing at take-off. The
  sprint-jump boost follows the facing, which matters on narrow strips.
- `pivot` runs along a world axis and redirects that boost more sharply,
  then turns in flight. It is reserved for narrow landings. The 3/16
  trapdoor strips on `mcc-7-1` illustrate this geometry.
- `leave` jumps on the first tick that begins with `onGround` still set but
  no support under the box. This event survives small changes in starting
  position better than a fixed jump tick, especially when leaving a strip
  under a lid.
- `bounce` aims first at slime, then at the landing or another rebound.

Programs share simulated prefixes. A family branches from a common run at
each candidate jump tick, then from a common air phase at each turn tick.
Re-flying those prefixes for every combination wastes most of the work.
Final robustness checks still replay each accepted program from its start.

For a collision-free final phase, touchdown is affine in its fixed heading:
`C + R * direction`. Intersecting that circle with the landing area gives
useful headings directly instead of searching a large angular grid. The
oracle also prunes starts whose available airtime, speed and acceleration
cannot reach the destination. Such ballistic pruning is not applied where
a rebound or climbable can change the flight.

## What counts as a landing

`stands()` finds supported resting positions using drop probes and collision
face edges. A regular grid alone misses thin ledges and the slivers beside
posts. Starting the probes at the support's own height also avoids embedding
a body in a low ceiling before the simulation begins.

A landing is supported overlap, not simply a centre inside a cell. The body
must overlap the column by at least 0.1 blocks, so its centre can be up to
0.2 outside. Support can come from the column's boxes or an eligible nearby
box within step height, with footing under the body. This matches landings
that hang over a strip's edge or rest partly on the trapdoor beside it.

An ordinary accepted program must stay on its landing for eight ticks with
forward, sprint and jump released and sneak held. Planned programs use starts
nudged 0.03 blocks in each axis direction; live, beam and precise programs use
0.01. Goal-side bridges use the stricter 0.06 nudge. For a live body flush
against a wall, a nudge embedded in that wall can be omitted if the original
body is clear; the check never omits both directions of an axis.

Some legs deliberately touch and go. Their touchdown must support the next
leg rather than stop the body. On narrow momentum chains, stopping can
remove the only viable continuation. Two-leg selection therefore judges the
touchdown by the time to it plus the next flight, rather than choosing the
first leg in isolation. General next-leg lookahead is restricted to landings
with little forward speed, where a pinned body and a moving body can have
similar arrival times but very different continuations.

## Slime and climbables

Sneak on slime touchdown suppresses rebound and makes an ordinary rest
possible. A planned bounce leaves sneak released. Holding sneak throughout
the air would sacrifice steering, so landing control applies it on the
crossing tick. The oracle can chain rebounds when one cannot reach the
landing. `mcc-3-1` has a drop onto slime whose rebound threads an offset slot,
continues across slime and reaches a ladder.

A deep bounce searches slime pools more than 14 and up to 72 blocks below
take-off. Start nudges alone do not protect a long fall with a narrow final
catch. `commitGate` additionally checks an interior pool landing and tests
velocity perturbations through stretches where aborting no longer reaches
the pool, with position perturbations at the start of each such stretch.

During a deep replay, `deepVerdict` simulates the remaining program from the
live body every tick. If it no longer arrives while still grounded, the
executor brakes and replans. In flight, it switches to the pool's interior
basin while that abort remains reachable; otherwise it continues the
committed flight. The replay is held across path updates and corrections so
a rebound contact does not accidentally hand control to another path. The
snapshot must include the pool, so the host probes for slime beneath the
initial search box and can extend its lower bound. `opk2-2` illustrates why
a pool far below the ordinary search region needs this handling.

A hanging ladder or vine is a different starting state from a stand on its
top edge. Climbable programs start held against the climbable, then push off;
horizontal speed is clamped to 0.15 blocks per tick while on it. A catch
below the target can climb on to the required height. Live catch controls
hold height and oppose drift immediately, rather than allowing ordinary
air steering to slide the body out of the cell. A proposed outward lean on
a hanging traverse is unsuitable: wall contact is what keeps the body on
the climbable under a beam.

## Beam search

When the phased families find no program, `beamSearch` searches per-tick
headings, jumps and coasts. It keeps an ensemble of the body and nudged
copies under identical controls. A search of the nominal body alone favours
fragile lines that fail after tiny position changes.

An airborne state's landing disc bounds where its momentum and remaining
air control can put it. The disc is judged before the vertical crossing
move: collision resolves Y at the tick's initial horizontal position.
Judging only the end-of-tick position overestimates what can carry the body.
Ground states retain a share of the beam so early jumps do not crowd out
later, faster take-offs.

For an obstructed flight, `DetourField` ranks states by a route around the
collision geometry. A straight-line distance or landing disc alone rewards
states headed into the obstacle. This matters for the two panes across
`opk-2`'s one-wide strip. When following a planner corridor, the beam advances
through its waypoints and allows touch-and-go legs. The ice chains on
`mcc-7-2` start from the last firm support and carry speed across the pads;
restarting from rest on each ice pad loses the speed needed for later legs.

## Stitching hops into a route

[src/hopPipeline.ts](../src/hopPipeline.ts) runs in the worker with either
the JavaScript or Rust cell solver. After an incomplete search, it tries
simulated edges and follows each landing with another cell search. A useful
hop can initially lead away from the goal, so its value is what the
follow-up opens, not just its landing's heuristic.

Stitching is best-first and retains alternatives. A greedy choice can land
on a promising platform with no continuation and discard the actual route.
Cheap families are explored before expensive yaw, pivot and beam searches;
completed routes are compared using program ticks and the remaining planner
cost. Searches making new progress receive bounded extra time.

Take-offs include the partial path's tail, reached high vantages, ledges
above slime, and nearby reached frontier cells overlooking new ground.
Searching only the tail misses a usable ledge beside or above it. A backward
closure over planner edges from the goal identifies goal-side landings;
these can be worth an initial retreat that an ordinary heuristic limit
would reject. `opk2-1`'s drop under a corridor wall is such a route.

A cell can contain several disconnected resting places. A pane rail's top
and the floor on either side share one planner node but cannot be joined
by a walk. Region stitching keeps the exact landing place and searches its
walk-connected stands instead of pretending every place in the cell is
interchangeable. `mcc-6-1`'s hoops and thin rails require these region chains.
Live re-searches and replans preserve that required resting region too.

Before stitching a planner jump onto or off a narrow strip, `verifiedPath`
tries it in physics. An exhausted search can ban that directed edge and
re-solve without removing the whole cell. A spent budget is not evidence
that the jump is impossible. Streaming exposes verified prefixes and stable
stitched landings while work continues, preserving the branch already flown.
The host cuts partial paths before one-way drops and jumps onto ice, where
a tentative route can commit the body before a continuation exists.

## Live searches and replay

[src/physics.ts](../src/physics.ts) and [src/plugin.ts](../src/plugin.ts)
try a program from the exact live body before braking and lining up. A
planned program is re-anchored and robustly re-flown from the current state;
when it no longer works, a live search can find a replacement. Kick and
pivot plans seed live searches with their own families on a separate work
allowance. Seeded pivots also participate in skip-ahead searches and replay
directly from the searched state. They are not confined to worker planning.

If re-anchoring fails, ordinary line-up approaches a stand. Precision
programs can need a start accurate to about 0.01 blocks, finer than one
walking or sneaking press. `lineUp` searches short presses and their coast
to rest, then re-anchors the flight. Moving line-up can use the body's glide
instead of stopping it first. A short, supported ground prefix can also
create clearance for a jump from a rest flush against a wall.

Replay reproduces the executor's look rounding in 0.15-degree steps. A
kernel-selected flight uses its verified controls through touchdown;
independent air steering would change the flight being validated. Path
updates retain a running program and take up the new path from its landing.

Live searches are principally bounded by kernel work, with a wall-clock
safety cap. A clock-only budget makes the same take-off succeed or fail with
machine load. Only exhausted family searches support a negative verdict;
beam timeouts and missing stand models do not prove impossibility. Repeating
the same search from an unchanged rest adds no evidence.

`holdsOn` simulates both a line-up or creep input and its coast. Sneak alone
is insufficient on a 3/16 panel because the edge guard checks supporting
cells, not the panel's exact footprint. Skip-ahead compares one verified
flight with the planned approach and leg, including damaging-landing cost;
it is not enough for a shortcut merely to land somewhere sooner.

## Local gait scheduling

`allowHopIntoJump`, requiring `allowSprintHop`, lets
[src/gaitSearch.ts](../src/gaitSearch.ts) choose between running this tick
and starting a hop. It searches short schedules of runs and jumps through
the existing path, then replays the selected ground tick or flight. It
handles both optional speed hops and jumps required by gaps or rises.

The scheduler uses the executor's wall steering, node retirement and
supported-landing rules. Its horizon ends at path limits and before programs,
neos or chains requiring other control. A program's take-off must be stood
on, rather than retired during an overhead flight. An eased flight releases
forward before touchdown when full travel would overshoot the landing or
leave too little room for the next riser.

Bounds on remaining time, repeated-ground-state detection and a hop-limited
horizon keep the search small. Dominance along straight runs is useful;
merging bodies merely a stride apart before a turn is not, because their
turning opportunities differ. Corner-rounding and direct wall-following
policies use a time margin and stickiness to avoid switching between
incompatible schedules. Hazards count as obstacles even without collision
boxes. Slippery surfaces need a higher speed bound than ordinary ground,
especially for tapped head-hitter hops.

`allowStrafe` uses a 45-degree look offset with forward and strafe keys to
produce input of magnitude 1 rather than forward-only 0.98. It is omitted
on the jump tick because the sprint boost follows the look direction.
The plain gait remains an alternative when strafe gives no competitive run.

A gait search with no schedule is not a veto on the legacy jump gates. Its
failure can concern the horizon's later turns or a landing type outside that
search, rather than the next take-off. Likewise, allowing arbitrary turning
hops can produce a locally quick landing with no continuation beyond the
horizon. More permissive local motion does not automatically improve a route.

## Damaging landings

[src/fallDamage.ts](../src/fallDamage.ts) separates the safe-fall distance
from damage rounding. For the 1.21.5+ rule it counts the landing tick and
rounds down; the older rule excludes that tick and rounds up. Hay and honey
reduce damage, beds reduce effective fall distance, and a non-sneaking slime
bounce avoids damage. Sneaking onto slime takes the fall instead, so deep
pool arrivals must not blindly suppress their rebound.

A damaging landing can be followed by a server velocity packet that removes
horizontal speed. The executor waits for that update before deciding the
next take-off, with a bounded timeout, and the gait charges for damage and
lost speed. A teleport resets fall accounting. The oracle rejects programs
whose touchdown costs five health points or more; deep bounces additionally
use their commitment and live-abort checks.
