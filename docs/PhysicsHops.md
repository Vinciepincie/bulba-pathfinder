# Physics-verified hops (`allowParkourPhysics`)

Improvement, opt-in (needs `allowParkourExtended`). The parkour table
(`parkourTable.ts`, [ExtendedParkour.md](ExtendedParkour.md)) plans a jump from
a handful of numbers per cell and a fixed set of flight lines. That covers
ordinary parkour. It cannot express a flight through a 2/16 pane post, a jump
that turns round an L-wall, a neo off a one-wide stair, a drop onto slime that
rebounds onto a ladder, or a chain of re-jumps that carries speed across ice.
Here the question "can a body get from this stand to that one" is answered by
the physics itself: flown, tick for tick, in an exact copy of the engine the
bot moves with.

## The kernel (`src/playerSim.ts`)

An allocation-free port of prismarine-physics' normal movement branch
(collision, step-up, sneak edge guard, ladders, slime rebound, liquids as
"stop"). It is **bit-identical**: `test/playerSim.test.ts` flies 60 random
worlds of awkward shapes (slabs, stairs, fences, panes, trapdoors, walls,
heads, pots, ladders, ice, slime, snow) with random controls against
prismarine itself and asserts exact equality of every position and velocity,
over the stateAt path and the inline grid path alike. The heading is a unit
vector, not a yaw, so there is no trigonometry in the loop. Traps found on the
way: slipperiness must be Float64 (blue ice 0.989 in f32 diverges by 1.8e-9);
waterlogged states are water; a body dropped *into* a pane falls through it.

Cost: about 0.5 µs a tick, reading the snapshot's state grid inline
(`SimWorld.grid`).

## Control programs and the oracle (`src/hopOracle.ts`)

A **program** is what the executor replays: a run heading until the jump
fires (pressed on tick `jumpAt`), a first air heading for `turnAir` air ticks
(or until a slime rebound), a second heading to touchdown; forward held,
sprint as the program says. Headings aim at a point (re-aimed every tick) or
along a fixed direction. A program is accepted only if it lands on its node
and *stays* (eight ticks of letting go), and does so from starts nudged
`ROBUST_NUDGE` (0.03) either way on each axis.

Families, simplest first: `straight`, `run-turn`, `air-turn`, `yaw` (the neo:
out along the pillar's face, back in behind it), `bounce` (at a slime top
until the rebound, then at the landing).

**The search is a tree.** Every program of a family shares its run-up with
every program that jumps later, and its first air phase with every program
that turns later. `advance()` is the one flight loop, stepping between marks
(a ground tick, an air tick, a rebound); the search branches from copies of
the flight at each mark. The kernel is deterministic, so a branch is the same
flight as one flown from scratch, and `robust()` re-flies every pick from
scratch anyway. On mcc-6-2's four neos this took the compile from 970k kernel
ticks (1.6 s) to 51k (0.1 s).

**The second heading is aimed, not gridded.** After the turn, a fixed-heading
air flight is affine in the heading: the feet come down (at the tick the
vertical motion alone decides) at `C + R·u`, and a body let go there comes to
rest at `C' + R'·u`. So the headings that land are arcs of a circle, found in
O(1) (the circle meets the column only if R lies between its nearest and
farthest distance) and swept only over the column's angular window. Per arc
it tries the touchdown furthest inside the column (a neo's landing arc hugs
the column's edge: that is the robust one), the rest point nearest the
centre, and two spread across the arc (an obstacle cuts into an arc from one
side). The oracle returns the **quickest** robust program, not the first.

**A take-off out of reach is not flown** (`canReach`). The most ground a
flight covers before it has fallen past the landing's level is the speed it
carries plus its air acceleration, summed over the ticks the fall takes:
nothing in the way adds to either. A take-off (and, later, a first air phase)
the landing column lies beyond is dropped with its whole subtree of turns.
Sound, so the programs found are the same (`PF_REACH_PRUNE=0` flies
everything for A/B); not asked where slime could throw the body back up or
off a climbable. A ladder catch has no ballistic pre-screen, and there it is
the difference between a search and a timeout: mcc-4-3's jump to a ladder from
a moving body took 262k kernel ticks (129 ms, against the executor's 40 ms)
and takes 6k (9 ms).

`stands()` finds where a body can rest on a node: nine drop tests, from just
above the feet cell's own boxes (from any higher, a ceiling two blocks up
starts the body embedded: 6-1's pane rails had no stands at all), and more
off the edges of the carrying cells' own collision boxes, by the overlap a
landing needs either way: a thin ledge — the 3/16 top edge of an open
trapdoor, which spiral3-b lands on beside a daylight detector that leaves a
window a few hundredths wide — meets no point of the grid.

**A landing is the body supported by its node's block**, not its centre
inside the block (`LAND_OVERLAP`): the box over the column by a tenth at
least, the centre up to 0.2 outside. The planner's shifted lines land that
way by design (Move.takeoff), and 8-1's strip past a two-high post has no
other landing at all: the flight must pass the post with the box clear of it
and come down hanging over the strip's edge. The aim sweeps the widened
column, and the executor retires a program's node by the same test.

**Slime.** A landing on a slime top is made with sneak pressed on the
touchdown tick (`sneakLanding`: in the air, coming down, the feet reaching the
top this tick, the box over the column) — without it prismarine throws the
body back up. The executor applies the same rule to every touchdown on slime
while a path runs (not a planned bounce's `via`), and holds sneak while
standing on slime before a jump: a body let go on slime rebounds off its own
weight every other tick, which drops the sneak edge guard. The bounce family
aims its first phase at the slime's centre and 0.3 short of it, walking too
from a live state. A landing no single rebound reaches is tried as a CHAIN
(`HopProgram.bounces`): at each rebound, on to the next slime tops under the
rebound's apex, up to two after the first; nudged twins are held to the first
rebound, past it the replay is the kernel's own flight.

**Off a climbable** (`HopProgram.climb`). A ladder cell's pads are a body
hanging on it, pressed against the ladder's face, held (sneak, no keys) at two
heights. The run is the push off it (the ladder clamps horizontal speed to
0.15), jump held climbs, and the program has fired once the body is off the
climbable and airborne. The executor climbs to the take-off, holds, and
searches live from the held body. A ladder's top edge is an ordinary stand:
the Ten Ways slime room's way starts by walking off it.

**Vantage take-offs** (`hopPipeline.vantages`). The partial path ends where
the heuristic bottoms out — on the floor under a goal on a ledge, where no hop
rises far enough. Discovery also takes off from the highest reached cell of
each column within 16 of the goal, at least 3 over the partial path's end
(four of them), and a winning hop from one is stitched in behind a search to
it.

## The beam (`HopOracle.beamSearch`)

For the jumps no family flies, a beam over per-tick controls: each tick a
heading off the bearing to the next waypoint (or one of the four world axes:
a run along a rail holds one exactly), jumping or not while grounded, or a
coast. Programs are the controls themselves (`HopProgram.seq`).

- **Ensemble.** Every state is five bodies (the body and four nudged
  `LIVE_NUDGE` = 0.01 either way) on the same controls; a state lives while
  all of them do. A beam over the body alone finds a precise jump's one
  fragile line first and nothing else.
- **The landing disc** ranks and prunes airborne states: with forward held on
  any mix of headings, the body comes down within R of its drift point. It is
  judged **a step early**: prismarine moves Y first, so what carries the body
  is whatever is under it when the crossing tick *begins*. The end-of-tick
  disc is a quarter block optimistic, and that is exactly the margin by which
  doomed early jumps would fill the beam.
- **Touch and go.** A landing short of the node starts the next leg instead
  of ending the flight, and the last leg may be a slide onto the node (ice). A
  short leg is ranked, not cut, if something within three blocks under the
  disc's nearest point can carry it (`footing`); a body falling below the
  landing with no slime under it is dead. Nudged twins are held only to the
  end of the first real leg: over several bounces two bodies a hundredth
  apart part ways as any two players would, and past it the replay is exact.
- **Ranking** is an optimistic time to the landing: distance at sprint-jump
  speed, plus the ticks a grounded body still needs to reach sprint speed at
  its ground's acceleration (ice is slow to get going). Grounded states are
  deduplicated by position and speed, and a share of the beam is always kept
  for them, or the first jumps off crowd out every later one.
- **Corridor.** Given the planner's partial path, the beam follows it
  waypoint by waypoint (a chain carries its momentum along the pads the
  planner found).

Synthetic results (`test/hopOracle.test.ts`): a three-block gap in 17 ticks, a
neo round a pillar in 18, a stepping-stone chain no single program can make in
32.

## Where it runs

**Planning** (`src/hopPipeline.ts`, the worker). When a search fails (no path,
or out of think time; on the first box of a solve even when it touched the
box's edge, unless the partial path ends against the box's own faces — then
the box was too small and growth comes first: climb1's goal is 46 blocks up,
and the discovery used to stand 1.6 s at the edge of a box a growth fills in
200 ms), `discover()` looks for hops from the tail of the best partial
path: simple families for every candidate, the yaw family for the twelve most
promising, the beam for the four most promising. Each candidate is judged by
what it opens (a follow-up search from its landing, capped at a share of the
budget); the winner is stitched in and the round repeats. The result is a
whole path to the goal or nothing.

**Executing** (`plugin.ts`, the program branch and the take-off search). The
exact replay is only possible because the kernel models the executor exactly,
including mineflayer's `bot.look`, which turns the yaw in whole 0.15° steps
from where it is (`LOOK_STEP`): a live program flies the yaws the replay will.

- A node no gate flies gets a **live** program from the body exactly as it
  is (`PhysicsSim.hopFrom`, over a flat copy of the live world), replayed
  from the very tick it was found: a neo on arrival, and any other parkour
  node on the first tick every gate refuses it (the entry of the line-up
  block), once per node. On arrival, moving: a take-off with no stop at all.
  Failing that, braked, from rest; then a
  program from a stand point with a line-up (`hopLinedUp`, simple families
  first: the yaw family costs five times as much, and put first it spent the
  whole budget on 6-2's third neo while the air-turn that flies it takes
  18 ms); the old scripted search last.
- **A verdict, not a timeout.** When every family has run to the end from the
  take-off cell and nothing lands (`hopExhausted`), a table jump is given up
  at once — its landing banned, a re-solve — instead of after the old search
  and the run-ups reach the same conclusion (8-1: 78 ticks of them). The beam
  may still be running out of time; it does not count. Nor does a landing or
  a take-off the model has no stand for (`HopOracle.noVerdict`): spiral3-b's
  stair landing has none, and a give-up there cost a re-solve for a jump the
  gates fly.
- A **planned** program is tried live first: as the node comes up, and at each
  rest near its start, re-anchored at the exact state (`HopOracle.anchored`),
  else searched from there. The line-up is only the fallback, and it presses
  only while the predicted rest point improves (one sneak press moves it
  ~0.065, three times the tolerance: the old rule circled the point for 147
  ticks). A planned beam program is never replayed from a line-up, only
  re-derived.

`PF_TAKEOFF_KERNEL=0` turns the executor's kernel search off for A/B runs;
`PF_HOP_DEBUG=1` and `PF_BEAM_DEBUG=1` print the pipeline's rounds and the
beam's layers.

## The gait (`allowHopIntoJump`, `src/gaitSearch.ts`)

Separate, but on the same kernel: on a grounded tick, hop now or keep the
feet down? A hop is the faster gait and it is ballistic, so the answer turns
on what its arc comes down in. `gaitSearch` asks it of the SCHEDULES of hops
the body could fly from here: up to three ticks on the ground before each
hop, up to four hops, then its feet, depth first, hops first, bounded by the
best so far. The hop is taken when a schedule that starts with it reaches the
goal and none that keeps its feet this tick does as well (toward a jump) or a
tick better (over open ground, where which schedule is first past a node
eight blocks on is half a tick of noise).

The goal is the ground's own (`PhysicsSim.gaitVerdict`): the landing of the
first jump ahead — a parkour node that is one (not ground the plan happens to
jump: `kWalkable`), or a rise too high to step — else a node nine on, else
the path's end, arrived at on the ground. The old guards ("a rise ahead", "a
jump ahead", "too little path", "the roof drops") are gone with the kernel
on: each was a fixed refusal where the search has an exact answer.

Every schedule is flown the way the executor would fly it:

- **Run ticks steer as the executor walks** (`src/wallSteer.ts`, one module
  for both): off a wall it runs along, along one in its way, no sprint
  against one.
- **The take-off at a jump is the executor's gate** (`kJumpLands`): the first
  tick `canSprintJump` would take it, its graze rule (a flight touching a wall
  must land with the heading 0.02 rad either way) and `maySprint` before it.
  A jump onto the path's LAST node is timed to standing in its box, not to
  landing within a block of it.
- **A node is passed as the executor retires it**: in the air low over a
  parkour node, or down within a block and past it. A landing short of the
  node is a take-off for it still (tenways-headhitters: counted as the jump
  landed, the gate then jumped from the lip into the next head-hitter, 12
  ticks a platform).
- **A landing is a body carried by a block** (`PlayerSim.carried`). The tick
  a body lands it moves down before it moves on, so it can touch down on the
  edge it is leaving and end the tick past it, with the ground flag and
  nothing underneath. The gates' own `caught` asks the same now: a jump that
  "landed" 0.03 past a slime top was approved and fell five blocks.
- **A landing that hurts loses its speed** (below): the body stands a tick and
  starts from rest.

The hop the search chose is replayed as flown: the look of every tick from
the take-off to the touchdown (`GaitResult.flight`, the `khop` branch). A
search that runs out of work before the feet had their say gives no verdict,
and the feet stay down: a hop is thrown for its whole arc, a tick on the
ground is asked again the next.

**The 45° strafe** (`allowStrafe`, `kStrafe`). Forward alone is an input 0.98
long; forward and a strafe key are normalised to 1. With the look an eighth
of a turn off the line and the strafe key held the push is along the line
and 2% stronger, on the ground and in the air: 5.58 to 5.69 b/s sprinting,
6.97 to 7.04 hopping (kernel, flat stone). Never on a jump's own tick: a
sprint jump's boost goes where the body LOOKS. The kernel's schedules run and
fly with it, the flight's looks carry it, and the executor's sprinting ticks
hold it; the key is released at the top of the next tick, whichever branch
that tick takes.

## Landings that hurt (`src/fallDamage.ts`)

A landing the server hurts is followed by an `entity_velocity` packet that
zeroes the body's horizontal speed. Read from the 1.21.11 server: every
tick's descent counts, the landing's too, and the damage is rounded DOWN, so
it takes four blocks, not three (a jump off a ledge two up, 3.25, is free;
before 1.21.5 the landing tick is left out and any excess rounds up). Hay and
honey keep a fifth, a bed halves the fall, slime spares a body that bounces,
water and a ladder take the fall with them. The executor counts the fall the
same way and, after a landing that hurts, decides nothing until the packet is
in (at best before the next tick, four ticks late when the server stalls;
eight ticks is when it stops waiting). A packet after a landing only the
older rule hurts switches the rule for the session.

From 1.21.9 that packet carries blocks a tick and mineflayer (4.37) still
scales it by 1/8000, which leaves the body no downward speed and a tick off
the ground after every hurt; the executor applies it as sent.

## Measured (arena, 2026-09-22)

Full route book, previous commit's build against this one, back to back on
one server (`bench/arena`, the results files of 2026-09-22): the 32 routes
both finish sum 2.6 s less, with basic3, paradise3-l49, spiral3-c and
spiral3-d each 0.7-1.1 s faster and no route slower beyond noise. Advanced parkour 14 → 17 of 28 in a sweep (mcc-3-2, 6-2 and
8-1 newly arrive), 18 in targeted runs: mcc-6-3 arrives raced after 6-2a, and
fails in a sweep because its first search runs over columns the server has
sent empty (present, unfilled). A wait for absent columns (`CHUNK_WAIT_MS`)
cannot see that, and held climb1's first search 1.3 s in every sweep: it is
off. The solver cores agree: the JS core in the worker
(`PF_NO_WASM=1`) and the Rust core take the same paths with the same node
counts, within 0.1 s, the Rust search 3-10x faster where the search itself
dominates.

Known limits: mcc-1-1 is impossible in the arena's copy (its command blocks
were stripped: the gap needs 10.3 blocks of carry, a best jump gives about
8); the ice routes (7-1, 7-2) need a chain that keeps its heading and speed
through zig-zagging pads, which the beam does not find yet; 3-1 (a drop onto
slime whose rebound threads two offset slots onto a ladder) and 6-1 (neos off
one-wide stairs round floating pillars onto 2/16 rails) are not solved.
Nothing takes off from a climbable: the Ten Ways slime way starts by jumping
off the west ladder onto a slime block (`tenways-slime-first`, `-bounce`),
and the oracle's pads, discovery's sources and the planner all leave a
ladder cell out. A flat walk node a block away can still get a sprint jump
(climb2's diagonal corner: the landing only settles back, 25 ticks on the
raised block beside it, the same in the previous build).
