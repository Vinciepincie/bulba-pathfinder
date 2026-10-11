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
(`SimWorld.grid`). Most of a tick is the box gather (getSurroundingBBs: 12.7
cells for 3.4 boxes, a quarter of them none) and the liquid test (4.5 cells,
every tick), so both read as little as they can, results unchanged to the bit
(`test/playerSim.test.ts` checks a static grid, a lazy window forgotten as
cells change, and the plain world against each other, boxes in order): a
range inside the grid is indexed straight, no bounds test a cell; the last
two gathers are kept by cell range (38% asked again: the next tick's move,
a move after the step-up's query) and the liquid answer likewise; and a
solve's grid, which is fixed for its life (the oracle's stand caches assume
it too), is read by 4³ bricks marked solid and wet as first asked, so a
gather skips the layers of empty bricks a fall's tall range crosses and a
body over dry bricks touches no liquid without a cell read. Warm pipelines:
mcc-3-1 266 to 178 ms, opk2-1 513 to 459, mcc-7-2 886 to 676, the same
kernel ticks.

**The jump rule follows the bot's own physics.** Vanilla's jumpFromGround
(1.21.11, LivingEntity and Paper's mapped copy alike) sets the rise to
max(jumpPower, vy): a jump held on the grounded tick of a slime rebound keeps
the rebound and adds the sprint boost. Stock prismarine sets 0.42 and kills
it. `scripts/patchPhysics.mjs` patches the installed prismarine to vanilla
from 1.21 (the version read; older ones keep the stock rule), and a patch
whose anchor is gone is a warning, never a failed install.
`PhysicsSim.probeVanillaJump` asks the bot's physics which rule it has (one
tick, grounded and rising at 1.0 with jump held, every effect, cobweb and
liquid cleared) and sets the kernel's (`setVanillaJump`), in the main thread
and, per solve, in the worker; a probe that throws is asked again. The
server never re-simulates a bounce, so either rule passes its checks; only
the kernel and the bot's prismarine must agree. No ordinary landing changes
(a jump from the ground starts below 0.42); only the rebound-jump branch
below depends on it.

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
flight covers before it has fallen past the landing's level, by a step at
most, is the speed it carries plus its air acceleration, summed over the
ticks the fall takes: nothing in the way adds to either. (The step: a body
coming down beside the column a step under its top steps up onto it. Held at
the level itself, the test cut every hop onto mcc-8-2's pipe, which is
reached off the 3/16 panel beside it and up the pipe's 0.19: the live search
found none, the bot stood nine seconds for the one its gait flew.) A take-off (and, later, a first air phase)
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
Carried by the column's own boxes - or, centred over the column, by a
neighbour's box no more than a step above the node's own stands
(`carriedAt`): mcc-8-2's strip of 3/16 panels runs beside a closed trapdoor
0.19 higher, and a hop onto the strip comes down on the trapdoor by box
overlap, centred over the strip. The executor's arrival box already counted
it (progLanded); the oracle did not, and found no landing there at all.

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

**A landing that hurts is no landing** (`FALL_VETO`). The oracle has no other
fall model: a flight keeps the highest point since its last support, and a
touchdown costing five points or more (`fallDamagePoints`, the whole fall,
sneaking as the program presses it, so onto slime the whole of it) fails it.
The candidate rings reach six below a take-off, four points at most with the
jump's rise; only the deep ring below reaches further.

**Deep bounces** (`HopProgram.deep`). A slime pool more than 14 below the
take-off (to 72: `deepSlimeTops`, apart from the shallow tops the chains use)
is a fall any miss of which kills. Its classes come after every shallow and
chain class; its tops are those within the rebound's reach of the landing
(`deepReach`, 2.3 more where the rebound jump is allowed). discover()'s deep
ring (the landings within that reach, per pool level: each column's nearest
top once, then the reach per landing height) is flown by the `deep` family
alone, the deep bounce classes: no other family's landing 15-72 down is one. After the
rebound each second heading is flown with and without the rebound jump
(vanilla rule only). Caught on a climbable in the target's column below it,
the body climbs on: jump held, steered at the column's centre, no sprint (a
free-hanging vine curtain has no wall to press into; its own keys there slide
it off). Robustness is not enough: a program nudge-robust at its start can
have a committed window a tenth of a block wide. So `robust()` also runs
`commitGate`: the rebound must come down a block and a half inside the pool,
and every tick off the ground from which the abort no longer lands in the
pool (a committed stretch; abortability comes and goes, so every such tick,
not a suffix) must still arrive with its velocity kicked 0.05 either way on
any axis, and each stretch's first tick with the body moved half a block
either way on x or z (scanned, not bisected: the margin has holes). Not half
a block at every tick: the abortable ticks are re-flown live from the body as
it is, so only what arises inside a stretch is left, and a teleport a few
ticks before a one-wide catch misses by construction; that rule passed no
program at all on opk2-2. The abort is the executor's own: steer at the
nearest of the pool's inside tops (`HopProgram.basin`), no sprint, jump or
sneak.

Live, every tick of a deep program flies the rest of it in the kernel from
the body as it is (`deepVerdict`). On the arch, a flight that no longer
arrives is no take-off: braked on the edge guard (at the lip at a sprint, a
hard reset letting go of every key coasts it off) and planned again from
there. Off the ground (a walk-off fires 3-4 ticks after leaving the arch),
one that no longer arrives switches to the **basin** while the abort still
lands inside the pool, steered from that very tick: at its inside tops until
still, then a replan (a give-up in the pool, not a death); else it flies
on. The path is held from the replay's first tick to the end of the basin
(`deepHeld`): results, block updates and server corrections leave it and
the verdict decides. The rebound ticks are on the ground, so the mid-jump
rule does not protect them, and a new Move for the flight's node would end
the replay. A deep catch counts at the target's own height, not a block
under it: the climb-on takes it there.

The pool must be in the box to be flown. Arena opk2-2's is 61 below its arch
and 8 under the box a solve plans in, and the first search dies on the arch
without touching the box's edge, so growth never fires. So once per goal the
host probes the live world under the first box (`slimeUnderBox`: chunks whose
sections in that band have no slime in their palette are skipped whole) for
slime that is the first collision going down its column from the course's
low point, within 72 of its top, and the goal's boxes reach two below the
lowest such top (`computeBox` floorY; a cached box that does not is
rebuilt). Probed again when a chunk arrives in the box or the movements
change, the lowest floor kept. It fires on opk2-1, 2-2 and 2-3 (one pool),
free on the two that never use it (identical kernel ticks offline).

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
- **Bookkeeping.** A body already down (or a twin let go) is never stepped
  again, so a child shares it instead of copying it; and the merge classes
  are integer tuples hashed into a map (collisions compared exactly), not
  strings built per child. The same programs: mcc-7-2's carry chain search
  2.7 to 1.2 s offline. A live take-off search never re-finds a chain (a beam
  program longer than the live beam's 80 ticks and farther than one flight,
  `futileChain`), so the executor no longer runs one (200-450 ms of the main
  thread each time, always none).

Synthetic results (`test/hopOracle.test.ts`): a three-block gap in 17 ticks, a
neo round a pillar in 18, a stepping-stone chain no single program can make in
32.

## Where it runs

**Planning** (`src/hopPipeline.ts`, the worker — on either core:
`createPathfinder({ core: 'js' })` runs the JavaScript reference solver there
instead of the wasm core, the arena's `bulba` engine). When a search fails (no path,
or out of think time; on the first box of a solve even when it touched the
box's edge, unless the partial path ends against the box's own faces — then
the box was too small and growth comes first: climb1's goal is 46 blocks up,
and the discovery used to stand 1.6 s at the edge of a box a growth fills in
200 ms), `discover()` looks for hops from the tail of the best partial
path: simple families for every candidate, the yaw family for the twelve most
promising, the beam for the four most promising. Each candidate is judged by
what it opens (a follow-up search from its landing, capped at a share of the
budget). The result is a whole path to the goal or nothing. Of the hops
whose follow-ups reach the goal, the quickest whole path wins (`stitchTime`:
its programs' ticks at 3.6 a unit of planner cost, the planner's cost for
the rest), not the first found: arena opk2-2 flew a 119-tick catch with
111-tick ones beside it, every success after the first dropped as "the
same ground" (they all end at the goal).

The stitching is **best-first** (`stitchHops`, `StitchState`): every hop
whose follow-up gets strictly closer is kept, the closest expanded first
(the old greedy pick), the rest held in reserve — on mcc-4-2 the hop onto a
slime balloon (follow-up best h 22.4) beat the hop onto the next tower's
ladder (26.2), opened nothing, and the greedy pipeline returned null. Each
state expands in levels, every state's cheap ones before any one's dear:
0 — vantage take-offs (reached high ground within 16 of the goal, and of the
partial path's END: a partial that bottoms out on a ladder under its tower
top) and drop-edge take-offs, the tail only when those find nothing; 1 — the
tail, if level 0 skipped it; 2 — frontier take-offs (below), simple
families; 3 — the yaw family, from the tail and then the frontier; 4 — the
beam. `reached`
answers for a state's whole search only: a search to one cell (the way to an
off-path take-off) invalidates it.

**Frontier take-offs** (`frontierSources`, `HopOracle.opens`): reached,
standable cells within 8 of the tail that look out on unreached ground a hop
could land on (no more than MAX_REGRESS further from the goal), lowest
heuristic first, a column or two levels apart from the tail and from each
other, at most four. The partial path follows the heuristic to where it
bottomed out; the way on may leave from a ledge beside it. On opk2-5 the
partial ends on a chain two blocks under the grate pillar (810,233,-395)
whose 4-block jump under a shelf the cell planner refuses; from the pillar
the pipeline stitches six straight programs to the finish (601 ms offline;
it found nothing before). A wider ring (landings up to 8 further from the
goal, for opk2-1's slot drop under a wall that winds away first) was tried
and left out: the yaw budget went on cave drops first, and the hop is a
wall-scrape landing (1 of 213 brute-force landings clean).

**Drop-edge take-offs** (`dropTakeoffs`, `HopOracle.openDrop`, families
`'bounce'`): ledges over the slime the tail sees, reached but off the path —
the highest reached cell of each column round the slime tops with a drop
beside it, nearest the slime first, each flown by the bounce family alone to
its first edge. On mcc-3-1 the partial path ends behind a pane window; the
only ledge the 9-block drop onto the slime leaves from is three columns
along, and the rebound goes through a 1-wide slot, onto the next slime row
and under the second barrier to the ladder. The bounce family's tops are
those within BOUNCE_REACH of the landing OR within what the rebound off them
physically throws a body (`reboundReach`: a nine-block drop throws it more
than nine) — a widening only, the fixed ring's chains still flown after.
They wait while the vantages get somewhere (`StitchState.dropsOnly`): a
level 0 whose vantage hops found edges defers its drop take-offs to a state
of its own at its level and score, flown once no level-0 state that got
closer is left. mcc-4-2 spent 35-50% of its pipeline on drops while its
vantage hops got closer, and no drop state is on its final path: 3.97 to
1.00 s offline, the same path.

**Stands beside a post** (`HopOracle.stands`): rest points beside the boxes
in the node's feet and head cells, dropped from the support's own top — a
fence post on a plank strip leaves a sliver (the cell's corners, for a post
with arms) the body stands on hanging over the strip's edge, and every point
of the grid above is inside the post (opk-1: no stand on its first strip, so
no hop to it). And a landing that slid into liquid does not `stay`: the
kernel freezes a body at liquid contact, so it would "stay" in the lava
beside the ledge it came down on.

**The goal side** (`goalSide`, levels 2-3). A backward closure over the
cell planner's own edges from the goal cell (`edgesFrom`, the worker's
MoveGen; `nodeLike`), never through a cell the failed search reached, at
most 4000 cells generated in 15% of the budget. A hop that lands on the goal
side completes the route, so those landings are worth any regress (exempt
from MAX_REGRESS), tried nearest first and from every take-off (not deduped
across sources: which take-off flies one is the question), at most 8 per
take-off. On opk2-1 the way on is a slot drop under a corridor wall, 4.8
further from the finish than the moss stone it leaves from: ranked by the
heuristic it was never tried, and a cave below was tried first. A goal-side
bridge is held to a stricter nudge (`SIDE_NUDGE` 0.06): the ones it opens
are over voids - opk2-1's holds by an air-turn 13t, a marginal 6-block ice
jump on mcc-7-2 (14 brute-force landings) does not.

**Places, not cells** (region chains: `HopOracle.regions`, `StitchState.rest`).
A thin block in the cell under a node (a pane rail, a post) makes the node
several places the planner reads as one: the rail's top, and the floor either
side of it a block lower (`underStands`, filed under the node by nodeY, kept
only where no other column carries the body), which no straight walk joins.
mcc-6-1's hoops are a line of them over a void: the planner's node is the
pane's top, and the pipeline returned null. A hop whose rest is beside such a
block (`besideRest`: below the block's top, so a bottom trapdoor's top is
still the planner's own place) gets no planner follow-up: it is held to its
place (`HopProgram.land`, re-verified) and the stitching goes on from that
rest in region mode (`discoverFromRest`: take-offs from the rest and the
stands it walks to, landings in each place of every node in range, the rest's
own node's other places too). Region states have one level (simple
families), up to 20 rounds, and may land up to REGION_SLACK further from the
goal onto a place not landed on before: a pane crossed inside a hoop gets no
closer. mcc-6-1: 12 programs, 141 ticks. The ordinary discover() does not
target under places: tried, the path was the same and opk2-5's discovery
flew 81% more. A region round keeps REGION_EDGES (2) edges, not EDGES (6):
best-first follows the best, and under a race's load the rest made mcc-6-1
5.2-6.1 s against 2.8-3.0 s for the same 12 programs (the ordinary rounds
keep 6: at 3, mcc-7-1 and 7-2 lost their paths).

**A pipeline still getting closer gets more time** (PROGRESS_*): each hop
that lands nearer the goal than any before (by 0.5) pushes the deadline to
at least 2.5 s from then, 6 s past the first at most; one that is not stops
on time (an impossible route still gives up at 5 s). Under the race's load,
mcc-6-1's twelve rounds of steady progress ran out two short of the goal.

The executor holds the hop to its place too. Its live re-searches pass
`land` (`hopFrom`), since a live program to the nearest place strands the
next take-off behind the pane. The landing counts only at the place's level.
A region program lost on the ground replans from where the body is
(`'prog-lost'`), at most PROG_LOST_TRIES (2) times from one rest. The worker
gets the body's exact feet (`SolveMessage.startPos`, on the ground only), so
a replan beside a rail starts in region mode from that rest. A rest whose
column the start cell does not mean gets a region chain too, after the
ordinary pipeline (tier 1): two opk-1 in-route rests go from null to a
program each.

**Carry chains over ice** (`HopOracle.chainStart`, `chain`, level 0). A
partial path that ends on ice (slip 0.9 and up) is bridged from its last
firm node, first, before vantages and the tail: the beam from there along the
partial path (its corridor), its depth the corridor's length at chain speed
with CHAIN_SLACK 1.4 to spare, one edge, CHAIN_SHARE 0.85 of what is left.
Speed carried across ice is never stopped and taken up again: on mcc-7-2
nothing lands from rest on the last pads, and a 107-tick chain from the stone
before the first crosses all six (about 0.67M kernel ticks, 2-3 s).

**Pivot jumps** (family `'pivot'`, level 3, after the yaw and kick families
land nothing; never live). A run along a world axis toward the landing (along
a narrow take-off's length), the jump pressed facing 0.3-1.2 rad off the
bearing and at least PIVOT_MIN off the run (the sprint-jump boost goes along
the facing: replayed as a kick), then turned. Narrow landings alone (stand
points spanning under PIVOT_NARROW 0.25 on one axis), two per source:
mcc-7-1's 3/16 trapdoor strips need a jump some 80 degrees off the run, which
no other family flies. In the live search it would cost 0.5M ticks a strip
and starve the beam's verdict, so it stays in the pipeline.

**The failed search's partial is walked while the pipeline runs**
(`streamablePrefix`): up to, not including, the earliest point of the walk
that any level-0 off-path take-off (a vantage, a drop edge) is nearest to,
the start itself counting - never past a take-off the pipeline may choose -
the whole partial when there is none. The bot otherwise stood for the whole
pipeline (mcc-3-2: 14 ticks; mcc-4-2: 75); the host's partial rules
(one-way drops, ice, never empty) still hold. The single nearest (node,
candidate) pair cut mcc-3-1 at a drop by the partial's far end: the bot
walked 6-14 ticks west and back to the drop behind its start that the
pipeline chose (take-off at tick 31-37 against 12 unstreamed).

**A box grown for an earlier goal is not inherited** (`ensureSnapshot`,
`INHERITED_BOX_RATIO` 4). mcc-6-1's twice-grown 237x120x206 box carried
through 6-2 into 6-3, where a follow-up search from a landing dropped to the
ground and flooded it: the first path came after 2.56 s instead of ~0.1 s.
And the pipeline's per-state re-solve and its search to an off-path take-off
get a share of what is left, as a follow-up does, not all of it.

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
- **Precision plans** (a beam chain, a pivot: `HopProgram.seq`, `precise`)
  land only from within about a hundredth of their start, finer than a
  walking line-up places a body (one sneaking press moves the rest ~0.06).
  The walking line-up brings the body to rest within PROG_EXACT_TOL 0.05; at
  each rest within PROG_EXACT_REACH 0.11 the plan is re-anchored, else the
  kernel's **exact line-up** (`HopOracle.lineUp`) finds one or two one-tick
  sneaking presses, turned in whole LOOK_STEPs as bot.look turns, whose
  coasted rest is within 0.01 of the start and re-anchors the plan (about
  22k kernel ticks, 6 game ticks). The executor replays the presses
  (`'prog-press'`), lets go to rest, and re-anchors there. Two line-ups a
  node; a precise plan is spent at a rest within tolerance it has tried.
- **A start across a gap** is never walked to: a program whose start is more
  than PROG_FAR_START 2 away and the kernel does not walk to (`walksTo`)
  replans from where the body is (`'prog-far'`), at most PROG_LOST_TRIES
  times from one rest; past that it holds on the edge guard until the
  futility clock bans the take-off. Nearer, where the kernel does not walk to
  a start at the body's level, the whole line-up sneaks.
- **A ladder catch below a climb** (the planner's catch a block under a
  top, then the climb out of the same column): the take-off search tries a
  landing on the climb node's top first, the catch after. The catch then
  costs a block of climbing the top landing skips (arena tenways-ladder: 12
  ticks).
- **A settle owed to a landing long past is spent** (HURT_WAIT_TICKS + 8
  ticks): one carried over a path's end and a think into the next goal held
  the bot 8 ticks at its start (arena opk-9).
- **A slip is no flight.** Where no gate flies a take-off, a body in the air
  keeps the keys of the flight it took; but one that only slipped a step down
  off its support (falling, under 0.1 b/t across) presses neither forward
  nor sprint and lands sneaking, the edge guard on. Pressed, they carried a
  body off arena mcc-8-2's 3/16 plate band, 24 blocks down, on all three
  engines.

A live search is stopped by its WORK, not by the clock (`HopOracle.tickLimit`:
kernel ticks, 2 500 for each millisecond of the budget it is given). By the
clock the same search flew 30 000 ticks or 110 000 in its 40 ms, depending
on what else the machine was doing, and the one program that lands opk-6's
second jump was found or missed by a few flights: missed, the bot stood
twelve seconds re-solving, one run in three.

`PF_TAKEOFF_KERNEL=0` turns the executor's kernel search off for A/B runs;
`PF_HOP_DEBUG=1` and `PF_BEAM_DEBUG=1` print the pipeline's rounds and the
beam's layers.

## The gait (`allowHopIntoJump`, `src/gaitSearch.ts`)

Separate, but on the same kernel: what a grounded body does THIS tick. The
kernel decides it (`PhysicsSim.gaitDrive`, the executor's first question on
the ground): a jump, handed over as the flight that was judged and replayed
look for look (`khop`); or a tick on the ground, taken as the kernel ran it
(`krun`); or nothing — it has no way there from here, and the old gates and
the oracle decide. In the arena it drives five grounded ticks in six.

`gaitSearch` asks the question of the SCHEDULES the body could fly from here:
ticks on the ground, a jump, ticks on the ground, a jump. Two kinds of jump:

- **a hop of the gait**, over ground that runs on: up to three ticks after a
  landing, up to four in a schedule;
- **the jump the path asks for** — a gap, a rise over a step: one for each,
  on any tick (the lip is where it is), three in a schedule. Over a gap it is
  taken only if it comes down the same from a hair either side (the nudge
  test, as the oracle's programs): a schedule that keeps its feet for a
  take-off a tick on is no schedule if that take-off is one in a hundred.

The jump is taken when a schedule that starts with it reaches the goal and
none that keeps its feet this tick does as well (with a jump ahead) or a tick
better (over open ground, where which schedule is first past a node eight
blocks on is half a tick of noise). The goal is the node ten blocks down the
path, or the path's end (arrived at on the ground), or the landing of the
third jump the path asks for, or the node before one the search has no
business with (a planned program, a chain, unloaded world).

Every schedule is flown the way the executor would fly it:

- **Run ticks steer as the executor walks** (`src/wallSteer.ts`, one module
  for both): off a wall it runs along, along one in its way, no sprint
  against one.
- **A node is passed as the executor retires it**: in its box; in the air low
  over a parkour node; down within a block of one and past it. A landing
  short of a parkour node is a take-off for it still.
- **A landing is a body carried by a block** (`PlayerSim.carried`). The tick
  a body lands it moves down before it moves on, so it can touch down on the
  edge it is leaving and end the tick past it, with the ground flag and
  nothing underneath.
- **A schedule ends where the body could stop**: braked from its last
  landing it stays carried. A post reached at a hop's speed is slid off the
  far side, and is no arrival.
- **A landing that hurts loses its speed** (below), **slime is the oracle's**
  (the kernel's bounce is no landing), and no hop of the gait takes off into
  a turn of more than 45°, no jump the path asks for into one of more than
  20°. Into a turn a jump flies the old heading; the kernel flies it exactly,
  so the angles are a cut on the search's work: a flight thrown across the
  line is one more to refute. At 45° the gait took 73 ticks off 21 routes
  offline for a quarter more work; at 90° and 180° less, for more.
- **A landing that hurts costs what the health is worth** (HURT_COST, six
  ticks): a schedule comes down hurt only where that saves it as much. Timed
  alone, a hop off a ledge into a fall that hurts beat running off it by a
  tick.
- **Walked off a ledge, the body steers on down the path** (`fallAim`, in
  the kernel and the executor's `kfall` branch): once the node below is
  behind its motion it steers at the next on the same floor, not back at the
  node. Steering at it all the way, it came down at a standstill on top of it.
- **A jump for a landing or the path's last node is eased where it would fly
  past** (`GaitResult.ease`): forward let go at the latest tick that brings
  it down short of the far side, found by bisection on bare flights, and
  replayed so. Thrown at full stretch onto mcc-6-2a's last node the body came
  down past it and walked back, three ticks in every run.

**The node a planned program takes off from is stood on.** The gait's goal
is cut at a program, chain or via node (the oracle's flight, not a hop), and
the node before it is `last` for the search — arrived at on the ground, in
its cell, stopped — and retired by the executor on the ground only
(`stoodOn`). Flown over, it was retired at the first tick of a hop thrown at
it, the program's line-up took the keys in the air, and the body came down
past the one-wide block (arena mcc-6-3, a fall).

**Measured and left out: a take-off the kernel refused barred from the
legacy gates.** A 4.3-block jump on mcc-4-2 came down 1.2 short and the body
fell thirty blocks, after the kernel had found no schedule there; barring the
gates after such a refusal looked right. (Re-read later from the trace: that
flight was cut by a path installed mid-air that retargeted a node behind the
body and dropped sprint for two ticks — the gate's jump itself lands from
1514 take-offs. The bot's other mcc-4-2 falls were slides down a ladder with
no path and no keys, fixed by `fullStop`'s climbable hold.) Three versions, three ways wrong: `none` is a verdict
on the way to the horizon's goal, not on the next jump (basic1's chasm jump
lands, +4.5 s a run); a flight cut in liquid says nothing (that jump is swum
through a waterfall); and the kernel's landing is a body on a block, so
every catch — a ladder, a thin floor — reads as a miss (mcc-4-1, 5-2, 7-3
went from arriving to failing). The kernel's "no schedule" is not the
kernel's "cannot": the gates keep their take-offs.

**Measured and left out: a main-thread solver slice beside the worker.** The
worker's first path lands two or three ticks after the goal is set (the
solve, two message hops, the tick they fall into), where the main-thread
solver had its first slice inside the goal call: +0.06 s a goal on the
arena's JavaScript engines, +0.3 cold (A-B-B-A on ten routes, same-build
spread 0.03). One tickTimeout slice on the main thread was tried three ways.
Ahead of the dispatch, driving its partial: the wasm engine gained 0.1-0.26 s
a route, and parkouradv1 went from 3 of 4 arrivals to 0 of 4 — a 40 ms
JavaScript search is far shallower than the worker's first partial, and its
guess sent the bot the wrong first way. Ahead of the dispatch, whole paths
only: neutral, the dispatch waited 40 ms for nothing on every goal the slice
could not finish. Beside the dispatch, whole paths only, the worker cancelled
on success: no gain anywhere (the worker answers an easy goal before the
slice is done) and the tick loop still blocked for the slice. The latency is
the worker's to shave, not the main thread's to pre-empt.

**A streamed partial path is progress too.** A long search streams a partial
every slice, each installed trimmed to the body (`pathFromPlayer`), so the
node the body was running at is gone before the arrival loop retires it; and
partials do not restart the futility clock (a search that thinks hardest
would never be found stuck). Running flat out on partials for 3.5 s the body
was declared stuck at its take-off tick, every key released in the jump, and
it came down short of the landing and off the course (arena mcc-3-2, 210
blocks). A partial that drops the old head with the body within 1.5 of it
counts the node reached (`PARTIAL_PROGRESS_REACH`).

**A streamed partial never takes a one-way drop.** The first partial of a
long search heads for its best guess so far; on parkouradv1 that is the floor
four blocks below the start pillar, and the bot took off on it in the ticks
before the whole path (onto the fence post) arrived — 13 of 21 traced runs
ended on the floor, no-path. A partial is cut at its first drop of more than
`PARTIAL_ONE_WAY_DROP` 1.25 (a drop along the last final path's own edges
stands); the bot waits at the edge for the final path instead. And a partial
with nothing in it (its best node is the start) is not emitted at all: an
empty `path_update` reads as "arrived" to `goto()`, which on four advanced
routes resolved the race 1.6 s in, metres from the goal — `goto` itself now
ignores an empty PARTIAL as well.

**Never let go, never stand in lava.** `fullStop` holds sneak on a
climbable (released when a path is installed): with no keys a body on a
ladder slides down at 0.15 a tick, and a path that ended at a catch let the
bot slide off the bottom rung (mcc-4-2). A body touching lava or fire is
walked to the nearest point clear of it before any other branch
(`hazardEscape`; the futility clock is held): a take-off search from a body
already in liquid flies nothing, and opk-9's bot stood in the lava for the
64 ticks of the re-solve. On ice the line-up neither runs back nor steps
back nor runs up, and holds sneak (the creep's edge guard lapsed and slid
off mcc-7-2's pad B, 21 blocks). A mid-flight ladder catch is steered into
the wall against its along-wall drift (mcc-4-1 slid out of the cell's edge).
A neo's search from the braked rest gets `TAKEOFF_REST_TICKS` (twice the moving
budget; opk-6's needed 102k kernel ticks against 100k), and the program
branch does not repeat it from the same spot.

**A leave program** (`HopProgram.jumpOnLeave`, family `leave`, last in
`all`): the jump is pressed on the latched tick - the first that starts on
the ground with nothing under the box - not on a tick count, which a start a
hundredth along the run moves. The trapdoor shaft's jump-ups (a lid over
the strip, ExtendedParkour.md) land from 420 of 441 nudged starts this way
against none by tick count. Live take-off searches only (hopFrom), after
every other family found nothing; replayed, the jump waits for
`latchedNow()`, and without a kernel to tell that tick the program is
spent, never replayed blind.

**Caught on a ladder in the air: the catch controls from the first tick.**
A body airborne with its feet in a ladder or vine, its node (or the next) in
that column at or above the feet's cell, sneaks, presses into the wall and
steers against the drift - before the air control, the kernel hop's tape and
the gates, which flew it one or two ticks without sneak (mcc-4-1: 282.50 to
282.05 at vy -0.225; parkouradv1 caught 0.01 over the ladder's foot). The
gait's catch schedules climb so.

**A kernel hop is flown as taped, not air-controlled.** The narrow-landing
air control (`PhysicsSim.airControl`) steers a body in the air toward a
parkour node; a hop the kernel chose (`gaitDrive`) was flown through its
landing, eased or not, and is not re-steered. Air control first, mcc-8-3's
trapdoor tower held forward through a hop eased from its first air tick
onto a 3/16 panel top; the head caught the panel above (the body above
289.2 under a panel at 291), the push was lost and the body fell 200 blocks.
In ABBA12's 30 routes no kernel hop was ever air-controlled (0 of 314).

**No search twice from the same spot.** A program the lined search installs
skips the program branch's first try (it was the same search); the gate
fallback's search at rest marks the rest searched, as the neo branch does.

**Never onto ice from a partial, never stop in the air.** A streamed partial
is cut before a jump that lands on ice, as at a one-way drop: mcc-7-2's long
think carried the bot from pad to pad over a 21-block drop. And a body in
the air when its path goes (a reset, a stop, a new goal) keeps the flight's
forward and sprint to touchdown (`coastHold`): `fullStop` released the keys,
zeroed the horizontal speed and snapped the position mid-air, and a goal
reset off an ice pad dropped the bot onto the obsidian.

**A take-off search is bounded by its work, not the clock.** The budgets
are kernel ticks (`TAKEOFF_*_TICKS`); the clock is only a safety cap at
300 ticks a ms (`LIVE_FLOOR_TICKS_PER_MS`). As a 160 ms clock the verdict
followed the host: on a busy machine the main thread flew 300-900 ticks a
ms, and tenways-slime-way's 6-tick program (93.8k ticks) was cut at 90k
and the bot ran back and waited. A long tick costs no ticks - mineflayer's
physics timer catches the backlog up four ticks a callback, decision for
decision - so a search is never sliced across ticks. And the programs a
search found are still verified past its budget (`QR_GRACE_TICKS` 8k):
dropped unverified, opk-6's yaw 15t (found at 102k of 100k) came and went
with where the expiry check fell.

**A take-off is ranked by its landing and the next leg** (`hopFrom` next,
`nextLegPick`). Where the node after is a jump too, the robust programs
within a tick of the quickest are collected, and from each distinct
touchdown the next leg is searched (the simple families, 10k kernel ticks
each, 40k in all, at most four touchdowns); the one with the least total
wins, and the quickest stands unless another's total is strictly less.
Arena opk-1's first arm: the quickest 9t lands pinned against the fence post
(velocity 0) and the next leg from rest takes 15t; a 9t landing still
moving goes on in 9 (24 to 18, the same search work give or take 12k).
Only where the quickest comes down all but stopped (under 0.12 a tick
across, `NEXT_PINNED`): a landing that keeps its speed goes on as quick as
any, and ranking every take-off cost a fifth to a half more search work.

**The window ends before a neo** too (`Move.wOff`): the gait flew at it
straight and came back 'none' or spent, 10-30k work a tick on opk-2 and
mcc-8-1; the take-off node is stood on and the neo branch takes off from it
as it always did. Tick-neutral (within 3), a tenth of the work; reverted for
a day, mcc-8-1 lost 0.23 s and nothing gained (tenways-ladder's loss was
the catch below the top: see the catch-or-top take-off).

**Ladder catches are landings** (`GaitGoal.catchAt`, `Search.climb`). Where
the plain window has no schedule and a jump ahead catches a ladder or vine
(not scaffolding: a sneaking body slides down it), the window is cut up the
caught column and searched again: a flight caught on the climbable in the
node's column is a landing, climbed on as the executor's catch branch climbs
(sneak, into the wall, against the drift) to the goal's level - lost where it
leaves the climbable short of it, and every nudged flight must catch the
same column and climb too. Positive verdicts only: a rule that refused gate
jumps broke mcc-4-1/5-2/7-3 before. The legacy gate caught mcc-4-1's ladders
on the first tick it could, 0.44 above their foot; the kernel's catches are
1.9-2.6 above it (driveSim: about -40 ticks on the route). The cut window's
best schedule may land ON the catch node (the stand over a ladder's top), and
that is a verdict too. Refused (catch verdicts only), tenways-ladder's last
ladder top had no other way to its goal: the live search caught lower, and
from there nothing reached the next node (seven give-ups, a timeout). On
mcc-5-3 the same kind of verdict costs 3 ticks (283 to 286): the next jump has
no schedule and the gate takes it 44 degrees into the turn.

**A creep or a wait never coasts off its support** (`holdSupport`,
`PhysicsSim.holdsOn`). Prismarine's sneak edge guard asks only whether the
moved box's cells hold a block, so over an open trapdoor's cell a body coasts
off its 3/16 panel sneaking. The executor asks the physics instead: where this
tick's keys and the coast after them (20 ticks, 2.2 times the speed on
ordinary ground) leave the support or drop it more than a step, the first of
let go / a sneaking press back or forward / a walking one that keeps it on
replaces the keys. Waiting and creeping branches and the idle no-path tick
only (a run-back's next tick presses on). Arena mcc-8-2 P3, after a give-up:
the creep let go 0.035 inside the panel's edge at 0.024 a tick and the coast
took the box off - ABBA10 B1, all three engines dead; the survivors rested
1.75 cm inside it.

**What the executor retires**, mirrored in `Search.pass`:

- a node whose box the body is in (or low over, for a parkour node);
- a parkour node the body came down past, within a block — or within 2.5 on
  the node's own platform, ground the kernel walks back over (`walksTo`);
- a node the body is DOWN on the leg past (`onLegPast`): on the planner's
  own line to the next node, within a box's half-width, level with the node
  or beyond. The line of a jump too: a body that came down past its take-off
  node, toward the gap, takes the jump from there (spiral3-d walked back to
  the node's box and turned round again, nine ticks). On the ground only:
  retired from under a jump up to it, a node had the bot steer at the next
  one from mid-air and fall eight blocks (parkouradv1);
- the path's last node in its own cell: a goal is a cell, and the box round
  a node off its centre reaches into the next.

And a kernel flight ends no cut: a wall probe at jump height, against an
overhang the line on the ground never touches, dropped the cut and left
path[0] a node the body had run by — simple2 turned round for it and retired
seven nodes at once.


The search is depth first, jumps first, and exact where the path turns,
drops or ends. Three cuts keep it to some hundreds of kernel ticks (0.6 ms a
tick of the bot's on average, 16 ms at the worst):

- **The bound.** A schedule is dropped once it cannot beat the best so far
  going on at the fastest any gait goes — through the boxes of the nodes
  left, in turn, so round the corners — and a body with no hop left in its
  schedule at the speed a runner settles to.
- **Dominance.** A ground state is not flown from twice (which is also how a
  body run against a wall or down a hole stops being flown), nor where a
  state explored before it was further down a STRAIGHT run no later, by more
  than the speed between them could close. Not short of a turn or the path's
  end: there the faster body is the one that overshoots, and a stride's
  difference in where the turn is met is ticks.
- **The horizon in hops.** Under a low roof a hop is three ticks and a block
  and a half; four of them leave most of ten blocks to be run, the same run
  behind every schedule. The goal is no further than the chain of hops comes
  down, and three blocks: what the last landing shows in.

A search that runs out of work with a jump in hand takes it: the likeliest
refutations (the same jump a tick or two later) are the first flown.

The nudge test on a gap jump (four flights) is paid at a schedule's end, as it
is about to be the best, not at every landing explored: on a course with
three gaps to time it was most of the work, and a jump found not to hold
voids its subtree (the states explored under it count for nothing in the
dominance). Measured offline: a tenth less work for the same ticks.

The executor asks the kernel BEFORE its line-up and its wedge recovery. Those
are for a body no schedule moves, and each walked one off the line it was on:
a landing shifted two ticks sideways before a jump the kernel had from where
it came down (mcc-6-2a), a body creeping along a riser to its take-off walked
back by the recovery every time it got there (parkouradv1, 70 ticks). The
slow wedge trigger stays: a schedule the world does not follow moves nothing.

**A second ground policy that rounds corners** (`Search.roundMode`,
`GaitPick`; `PF_GAIT_ROUND=0` bisects). Each decision searches twice: the
plain policy, whose run ticks walk the wall they meet (`sideStandoff`,
`wallSlide`, no sprint against it), and one whose run tick, where the
straight step to its node has a box in the way at body height within 2.5,
steers at that box's nearest grown corner instead — kept only where that
corner lies within 0.5 of the leg. The rounding policy is taken where it is 2
ticks sooner, and kept once taken until the plain one is 2 ticks sooner. On
opk-4's diagonal pillar squeezes the plain run was refuted (the standoff
pushed the body off its support) and the gait threw 12-tick hops pinned to
the pillar face. Offline over 24 routes (driveSim) the guarded pair took 56
ticks off; unguarded — no margin, no stickiness, no corridor — it fell on
basic3 and stalled on spiral3-a, the policies flapping into states neither
schedules.
A box no higher than the node's feet is the riser the path climbs, not a
box in the way: rounding the step it was to jump onto cost spiral3-c 13 ticks
(a hop away from the step, then back) and climb2 and opk-7 a few each.

**The 45° strafe** (`allowStrafe`, `kStrafe`). Forward alone is an input 0.98
long; forward and a strafe key are normalised to 1. With the look an eighth
of a turn off the line and the strafe key held the push is along the line
and 2% stronger, on the ground and in the air: 5.58 to 5.69 b/s sprinting,
6.97 to 7.04 hopping (kernel, flat stone). Never on a jump's own tick: a
sprint jump's boost goes where the body LOOKS. The kernel's schedules run and
fly with it, the flight's looks carry it, and the executor's sprinting ticks
hold it; the key is released at the top of the next tick, whichever branch
that tick takes.

**The strafe is an option, never a restriction** (`gaitSearch`): where the
strafed search finds a hop but no run, the search without the strafe is
asked too and taken when more than a tick sooner (a strafed search with no
schedule at all leaves the tick to the gates: asked there, the plain hop put
mcc-5-2 0.09 off its line and a momentum chain onto a bar arm was lost). From spiral3-c's riser leg the strafed
search, run to exhaustion, found no run better than r67.8 where the plain one
found r48.9, and hopped away from the node and back (+8 ticks; -11 on the
segment in driveSim). Asked on every tick it took -17 for half again the work.

**An overshoot is eased** where the flight retired its parkour node in the
air and came down more than a block past it (`EASE_PAST`): the eased flights
are tried for it as for a node not yet passed. opk-9's stair of 2-deep steps
was hopped to the far edge of each and the next riser bonked (-5 ticks
offline; it fired on no other route).

**Measured and left out.** Rounds of schedules by the ticks they wait
(limited discrepancy): re-flying the earlier rounds cost three times what the
order saved. A state "on another's heels" (same velocity, under a stride
behind) as dominated: at a corner the stride is the difference between
rounding it and running into it. A jump taken from beside its take-off node,
the node left behind: a few ticks on the plain routes, forty lost on each of
two courses to landings a jump on could not be thrown from. Jumps the path
asks for into a turn of 45° (or any): spiral3-a and spiral3-d gained a dozen
ticks each, and basic3 and opk-7 came down in states the kernel had no
schedule from — the fourth lip, past the horizon, infeasible from where the
third's turning jump landed. A flight that turns in the air past its apex
toward the node after the landing: 2-4 ticks on the zig-zag courses, a third
more work, and two routes left without a verdict the same way. A hop that
scraped something on the way (a log's corner) flown again aimed past either
side of its node: twice the work for 5 ticks on opk2-3, and a course fell.

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

Sneaking refuses slime's bounce, and with it slime's spare: a body sneaking
onto slime takes the whole fall. The executor's onto-slime rule (sneak on the
touchdown tick) therefore stands down, letting go of the sneak, when that
landing would hurt and cost 8 points or more, or all but 2 of the bot's
health (`fallDamagePoints`): a deep bounce's basin, a miss into a pool far
below.

From 1.21.9 that packet carries blocks a tick and mineflayer (4.37) still
scales it by 1/8000, which leaves the body no downward speed and a tick off
the ground after every hurt; the executor applies it as sent.

## Measured (arena, 2026-10-02: the gait from the kernel)

One server, the builds minutes apart, executor ticks from the traces (first
tick with a path to the last): the 37 routes of the book all arrive, and sum
8 613 ticks against 9 053 for the build before the kernel's gait (a4c6d8e):
22 s less, no route more than 3 ticks slower. Against the first kernel build
(0bf64fc: the verdict only, the gates' flights) 318 ticks less, with its
three slow routes gone (mcc-5-2, a fall in one run of two; mcc-8-1 +29;
spiral3-c +25). The largest: climb2 -50, paradise3-l49 -37, parkouradv1 -31,
mcc-5-1 -29, tunnel1 -28, mcc-3-2 -23, mcc-2-1 -22.

Heppe's speedrun stages (routes opk-*, timed from leaving the start box to
entering the finish, against the time the map asks): opk-6 139 of 139 and
opk-8 68 of 68 pass, opk-3 is a tick short (110 of 109); opk-2 (90 of 67),
opk-4 (171 of 147) and opk-7 (195 of 126) are the planner's routes, not the
gait's: the fast lines there are jumps the cell planner does not know.

The executor's tick costs 0.6 ms on average (p99 5 ms); a jump decision
1.4 ms on average, 16 ms where three jumps in a row are timed.

The retirement rules, the eased flights and the fall steering (the same day,
later): 8 613 ticks again over the 37, all 43 arriving; what moved is the
turning round — four ground reversals across the book where there were
sixteen (simple2 -8, spiral3-d -9, mcc-6-2a -5, basic2 -4, mcc-1-2 -4; mcc-2-3
+10, pyramid-l85 +8, mcc-3-3 +5). The bot's tick costs 0.8 ms on average, a
jump decision 2.5 ms, 24 ms at the 99th percentile.

2026-10-03: the gait's hops into a turn of up to 45° (A/B on one server, 45°
against 20°: 8 590 against 8 619 — climb2 -16, spiral3-c -12, basic2 -10,
mcc-2-3 -7; spiral3-d +27 from a search that ran out of work and took the
first hop it had), then the nudge test paid at a schedule's end (spiral3-d
back to 250) and the vines of the climbable tag: 8 574 ticks over the 37,
all 43 arriving, 0.7 ms a tick. The second speedrun map's stage 3 (opk2-3)
arrives in 195 ticks of its 170; stages 1, 2 and 5 have no plan yet (the
planner's reach from their starts is 42-105 cells: a 4-block jump under a
trapdoor header, a 60-block drop onto slime and a bounce back up, header
jumps under a two-high roof and a 2+1 neo are not moves it has).

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
A flat walk node a block away can still get a sprint jump
(climb2's diagonal corner: the landing only settles back, 25 ticks on the
raised block beside it, the same in the previous build).
