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

**Fire and cobwebs stop the model** (`SIM_HAZARD`, `KERNEL_HAZARD`). A body
whose box (a thousandth in) touches a fire's or a soul fire's cell, or a
cobweb's, stops as a liquid stops it (`inLiquid`): fire has no box, but the
executor walks a body touching one out before anything else (plugin.ts
`hazardEscape`), and this model does not slow a body in a web. So no gait
schedule, program or skip flies through either. Read by 4³ bricks (a third
bit beside solid and wet) and by cell range as the liquid test is; no
measurable cost. Arena mcc-8-3: the stair landing came down 0.04 into a
fire's cell, the escape walked it 0.05 out, and with the asked-jump cap at 45
degrees the gait and the escape traded 11 ticks of it. The planner's half of
it is the diagonal rule (`moveGen.ts hazardAt`, below). PF_KERNEL_HAZARD=0
for A/B.

**No diagonal past a hazard's corner** (extended profile, `MoveGen.hazardAt`
and its Rust mirror). A diagonal keeps the cheaper of its two corner columns
(upstream); past a solid corner the box slides off it, but past a cell with
no box that is never entered (fire, lava, a cobweb: `blocksToAvoid`, not a
fence or an open door) the body cuts it by up to 0.21. From the feet to the
head over a jump up, either corner such a cell: no diagonal. Two of the 60
arena plans changed, mcc-8-1 and mcc-8-3, each to a line of the same cost
round its fire.

**A cell beside a hazard costs a little more** (extended, `HAZARD_NEAR_COST`
0.01, `MoveGen.hazardNear` and its Rust mirror, kept per cell a solve). A
secondary criterion: of two lines that cost the same, the one a block off a
fire, lava or a cobweb wins; no detour is taken for it (a hundred cells beside
a hazard to outweigh one block - at 0.25 a cell a long lava-side corridor
bought one). The gait turns into a node from
its 0.35 box and carries its speed through the turn, so a line turning beside
a fire nicks it: on mcc-8-3 every schedule from the stair foot grazed the
fire's cell by hundredths, the kernel (rightly) refused them all, and the
gates walked in. Its neighbour test indexes quietly (`cellIndexQuiet`: no
boundary flag for a cell no move goes to). Changes only mcc-8-1's and mcc-8-3's
plans; mcc-8-3 21.8-22.0 to 21.5 s. On mcc-8-1 the reshuffled tie meets more
of the aimed neos no flight makes, which the neo lookahead (below) catches.

**The gait rounds a hazard as it rounds a box** (`Search.roundCorner`,
`hazardCells`): the rounding policy's first box in the way may be a hazard's
cell, at any height the body spans, gathered once a search.

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
coming down beside the column a step under its top steps up onto it - only
where a box a step under the landing stands in or beside its column, and
never for a catch, which holds the body in its column; elsewhere the level
is the landing's own and the pruning unchanged. Held at the level itself, the
test cut every hop onto mcc-8-2's pipe, which is reached off the 3/16 panel
beside it and up the pipe's 0.19: the live search found none, the bot stood
nine seconds for the one its gait flew.) A take-off (and, later, a first air phase)
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
(`carriedAt`), with the column's own footing under the box a step down at
most (`footingUnder`; a rest on the take-off's own lip, centred past a gap
from the node's footing, is none): mcc-8-2's strip of 3/16 panels runs beside
a closed trapdoor 0.19 higher, and a hop onto the strip comes down on the
trapdoor by box overlap, centred over the strip. The executor's arrival box
already counted it (progLanded); the oracle did not, and found no landing
there at all.

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
- **The way round** (`DetourField`, `BEAM_DETOUR`). A beam to the landing
  alone ranks a body by the way round what stands between it and the landing
  column, not the straight line through it. A grid of 1/8-block points round
  the pads and the column: a point is free when a body centred there clears
  every collision box at some height from the lower of the pads and the
  landing to the higher plus a block (a one-high step or a slab is jumped, a
  two-high pane or a fence is not) - every half block, and the levels the
  pads and the landing stand at (a landing under a head-hitter is free at its
  floor alone); its detour is the octile geodesic over free points to the
  column less the octile line (a point the grid cannot reach: 6, demoted,
  never dropped). A body in the air whose straight line is blocked ranks by
  that way, not by its landing disc, which is blind to the obstacle it is
  about to meet. With no point blocked, or none of the column free, the field
  is null and the beam ranks as it always has. Arena opk-2's two panes across a one-wide
  strip: the straight-line beam flew 707k kernel ticks and found nothing; with
  the field it flies round both panes in 17 ticks from the arrival (375k
  ticks), 18 from the braked rest (597k).
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

**The planner's strip jumps are flown before they are stitched**
(`verifiedPath`). A planner jump onto or off a strip - the take-off's or the
landing's stands span less than 0.35 along the jump, where the planner prices
a run-up or a landing the body has no room for - is flown: from the known
rest when it is a path's first move (the body's own at a replan, a program's
landing), then from the take-off's stand points on the grid of turns, as the
executor searches a take-off it is stuck at, each on the executor's own
budget. One that the families searched to their end (or, where they ran out,
the beam) cannot land is banned as an EDGE (`SnapshotView.bannedEdges`:
both cores skip that one move, its cells stay open to every other) and the
path searched again; a budget spent is no verdict, as the executor bans only
an exhausted search. Momentum chains and slime bounces fly off the jump
before them and are not asked. Applied to the first search, every follow-up
that could be kept, every re-solve and every way to an off-path take-off; at
most 8 bans a path, 16 a pipeline. Arena mcc-8-2: the planner's diagonal
jumps off P3's 3/16 panel and off the trapdoor beside it onto the strip by
the pipe fly from nowhere - the executor gave up and re-planned three times,
3 s each, a strip cell a time (its own ban makes the CELL impassable). Seven
bans (about 1.5 s of the pipeline) and the pipeline stitches the run-up east
and a 20-tick straight program onto the pipe: 22.5 s to 15.6 s, no replans.
If nothing stitches, the worker posts the verified first search (its bans
kept), not the original.

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

**And its stitched legs, as they are found** (`STREAM_STITCH`,
`HopPipelineContext.onStitched`). Past 150 ms of a pipeline still running, a
state pushed closer to the goal than any streamed before is posted as a
partial (its stitched legs, to the last one's landing: the follow-up's tail is
no leg, and a later state may branch off before its end), only on along the
branch already streamed: the bot may be on its legs, and another branch is the
whole path's to bring. The legs are verified programs onto landings a body stops on;
the host's cuts (a one-way drop, ice) still hold, so a leg down a drop waits
for the whole path. Only the round's best kept state is streamed, and only when it is the one the
search goes on from next (the first kept past the delay could be one a better
sibling leaves). On the host a program in replay keeps its flight across a
path update, a line-up for the same program keeps its line-up, and every path
of a solve must hold the leg flown since it started (a leg the pipeline
streamed in THIS solve: its nodes are tagged with the solve's generation as
they come in, before a carried program lands on one; the executor's own live
program on a partial is no such leg - held to it, mcc-8-2's final was refused
and re-solved for 27 ticks - and neither is an old solve's): a partial without it is
not installed, a whole path is dropped - in the air the flight is kept alone
and the solve made from its landing, on the ground the path cut before its next
unflown leg. A cancelled pipeline gives up at once (`PF_PIPE_ABORT`: the
oracle's searches end as if out of time); it used to run to its deadline with
the next solve queued behind it in the worker.
mcc-8-2: its first legs were known 0.3 s in, the whole path 3.4 s in, the bot
ran the first search's prefix down a side line, stood 29 ticks at its end and
walked 15 back; streamed, the kick took off at tick 59, not 85.

**A path come in mid-flight without the landing is taken up from the landing**
(`FLIGHT_TRIM`), not from the body in the air: the node nearest a body still
over its take-off may lie behind where it comes down (mcc-8-2, a streamed path
from a re-verified first search 3.5 blocks back from a live program's landing:
70 ticks stood at it, then the futility reset).

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
  (the kernel's bounce is no landing), and no hop of the gait, nor a jump the
  path asks for, takes off into a turn of more than 45° (the asked jumps were
  held to 20° until 2026-10-29: two same-host A/Bs -1.2 to -3.9 s). Into a
  turn a jump flies the old heading; the kernel flies it exactly, so the
  angles are a cut on the search's work: a flight thrown across the line is
  one more to refute. At 45° the gait took 73 ticks off 21 routes offline for
  a quarter more work; at 90° and 180° less, for more. Being a cut, it gives
  way where it leaves nothing (`GAIT_ROOT_UNCUT`): a root whose jump it cut
  and whose feet have no schedule either flies the jump after all (mcc-5-2,
  down on a skull against a wall, the velocity 7° one way and the next node 40°
  the other: 47° was 'none' and four ticks of the gates, the hop from there as
  quick as any).
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
In ABBA12's 30 routes no kernel hop was ever air-controlled (0 of 314). The
flag ends on any grounded tick, whichever branch takes it: a branch that
handled a touchdown before the replay's own check left it set, and a later
flight replayed the old tape without air control.

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
The speed is the one along the way to the next node (`NEXT_ALONG`): opk-2's
second neo came down at 0.122 a tick, sideways, 0.057 of it toward the
finish, and ran four ticks to get going.

**A live program slower than the plan is not taken on the way to it**
(`PROG_LIVE_PLAN`). As a planned program's node comes up, a live search from
the moving body runs first; what it finds is taken only if it is no slower
than the plan with its line-up (the walk to the start at 0.1 a tick, 4 ticks
to settle, 2 more). opk2-2's deep bounce re-anchored on arrival in 111 ticks,
or where it did not, the search's quickest was a 131-tick bounce, taken,
where the line-up was 6 ticks to the planned 111. At the start (lined up),
whatever the search finds is taken as before.

**Skip-ahead** (`PhysicsSim.skipAhead`, plugin.ts `SKIP_AHEAD`). Where the
gait's window stops at a node a planned program leaves from (it is stood on:
the program takes off from rest there), a live program from the body as it
is straight to that program's node, when it beats the gait's ticks to the
stood node plus the least the plan's leg can take (`legFloor`: a standing
jump's rise in the kernel under the lowest roof on the way - at the stood node
and every half block on where a body fits, since a roof over the gap cuts the
arc where the take-off has open sky - then the fall through free air, or a
walk-off where the node is lower at all, and the ground
between at most 0.5 a tick past 1.8 free; a floor, never an estimate, so a
skip is taken only when the plan could not be quicker). Families only, on
100k kernel ticks, one robust flight that comes down on the node itself
(`HopOracle.singleFlight`: a touch-and-go program holds its nudged twins only
to their first touchdown), installed as the node's own live program and flown
whole; the nodes before it retire at the take-off. Tried on the touchdown
tick and the one after and at a fresh rest, three searches a node at most;
one that never takes off is undone (nodes and plan back, braked on the edge
guard) and its node not tried again; a path installed in flight carries it
onto the new path's landing node. opk2-1: from the moving body on the 969
post the yaw family flies the cave moss in 10 ticks, where the bot hopped to
the 967 post, lined up and flew the planned 12: the map timer 91 to 82.

**Skip-ahead at a neo** (`PhysicsSim.skipNeo`, plugin.ts `SKIP_NEO`). With a
neo's own live program found (on the arrival, `t0` ticks, or from the braked
rest), a live program past it straight to the node after, when it beats `t0`
plus the leg floor from the neo's cell centre; the beam too, with its detour
field (500k kernel ticks on the arrival, 800k from the rest: a long tick
costs no ticks). Installed, carried and undone as the skip-ahead's. opk-2's
double neo flew 18 + 13 ticks with a landing between; one flight round both
panes takes 17.

**Skip-ahead over walk nodes** (`PhysicsSim.skipWalk`, plugin.ts
`SKIP_WALK`). On a touchdown or a fresh rest, where the path walks aside to
its next jump (the walk at least `SKIP_WALK_DETOUR` 0.5 longer than the
straight line, the jump at most 3 nodes on): the gait's own time to be down
on that node - its search with the node as the path's end, the phase that
keeps its feet searched for its best (`GaitGoal.best`), not stopped at its
first - against one live flight straight there, families only, one flight
robust to the node itself. Not onto a momentum chain's stepping stone (the
re-jump is the chain's), and no skip of any kind onto a node a climbable
carries or catches (a ladder or vine in its cell or under it: the gait's catch
window flies those; a skip landed on a ladder's plate on parkouradv1, was
retired in the air, and the catch branch lost the body). opk2-1's ladder top
walks a block aside to jump the +1 diagonal: 19 ticks; one flight, 10.

**A skip's landing that hurts is charged as the gait charges one**
(`HopOracle.landingHurts`, `HURT_TICKS`: the stand, the health, the speed),
or a slower flight with the same fall would beat the gait's charged time. A
slime landing is sneaked onto, as the replay does, and takes the whole fall;
the executor's own fall rule (`wholeFall`) applies.

**A neo a few nodes on is judged before the body gets there** (`NEO_AHEAD`,
`PhysicsSim.hopFromCell`). Once a node, on a gait tick, the first neo within
3 nodes gets the lined-up verdict from its take-off node's own stands: the
simple and yaw families, then the beam. Every family flown out with nothing
landing is the verdict the arrival would reach after braking and searching
from the moving body and from rest; reached early, the neo is banned and the
plan replanned while the body walks on (`softReplan`). mcc-8-1's aimed
four-block neos past a two-high post on a one-wide strip (the planner sweeps
an aimed neo at a half-width of 0.2, a 0.1 corner allowance kept for the
neos that slide along a pillar) cost 10-15 ticks each at the take-off; with
the lookahead mcc-8-1 8.75 to 8.55 s (0 replans) with the near-hazard cost at
0.01 a cell; mcc-8-3 21.2.

**A planned chain is flown by the kernel, picked for the leg after it**
(`CHAIN_KERNEL`, `HopOracle.hopChain`). The gait's window stops before a chain
node and the gates flew it, the first tick a jump landed, blind to the next
leg. mcc-5-2's chain runs skull, a 0.125 iron-bar arm, skull: the long jump off
the bar needs one more ground tick on it, and the body keeps `onGround` a tick
past the bar's edge (vertical collision resolves at the old x/z), so the last
position over it must be past z -290.8625 - a quarter of the tick phases. Any
upstream change that moved the touchdown 0.09 on (the 45° asked cap did) left
no flight at all, and 73 ticks went on brake, creep, wait, the lined and the
old search, recovery and the 3.5 s futility reset (7.9 to 12.1 s). The pick:
touch and go onto the stone (`goLanding`: the touchdown over it is the landing,
no stop asked - a stone that thin stops nothing at the speed the next jump
needs), programs within 2 ticks of each family's quickest, the least total of
the leg and the next leg's search from its touchdown, the yaw family where no
simple touchdown has one, all inside the caller's budget. Traced: `chain
run-turn 13t` then `straight 16t` = 29 ticks, chosen, not lucky. At rest on a
chain's stone with no flight (`CHAIN_LOST`) the carried speed and the plan's
edge are gone together: a replan from there at once, no ban (a start has no
momentum, so the planner cannot bring the chain back from the rest).
Only mcc-5-2's first plan holds chains, so the reach is small. Measured and not
built: the same two-leg pick on every live take-off - replayed over every one
of the 60-route trace, mcc-6-3 3 ticks, mcc-8-2 1, opk2-1 -1, all else 0.

**A line-up never slides the body off its take-off** (`PROG_LINEUP_BRAKE`,
`PhysicsSim.holdsOn` with `keyTicks`). Where the line-up's keys and the coast
after them would leave the support, the body is braked against its velocity
(sneak off) for as long as the brake keeps it on. Sneak is no guard on a
panel: prismarine's edge guard asks whether the moved box's cells hold a
block. mcc-8-2 with streamed legs came down on a 3/16 open-trapdoor panel at
0.25 b/t with +0.026 sideways: every sneaking line-up and coast slid off in two
ticks (a 200-block fall); three ticks of brake along the velocity hold it.

**A stuck take-off is given up on the lined search's verdict, from beside it
too** (`LINED_GIVEUP`). Every family flown out with nothing landing gave the
take-off up only from the plan's own take-off cell; a run-back leaves the body
a cell off it, with the old search's 36 ticks and the futility clock to follow
(tenways-ladder, five of them). Within 1.5 blocks of the node it left, the
take-off cell's own verdict is asked and given up on, or its program flown; off
it, a replan from there. A search cut by its budget is no verdict: the old ways
go on as before. The old search (`solveJump`, whose grid is the yaw family's:
877 ticks in 42 traced races found nothing) runs only where the lined one gave
none, and the lined search's beam gets a budget of its own (150k: the shared
one left it a layer at most in 30 of 88 traced searches).

**On the way to a planned program's start, the plan from the moving body**
(`PROG_MOVE_ANCHOR`). Each grounded tick within 2 blocks of the start the
plan is re-anchored at the body as it is (`HopOracle.anchored`: one robust
flight, under a ms) and taken when it flies no slower than after the rest of
its line-up (the PROG_LIVE_PLAN estimate). Line-ups in the arena's traces:
mcc-8-2 42 ticks, mcc-4-2 23, mcc-6-1 12, mcc-7-1/7-2 10, opk-6 9; with it
mcc-4-2 -18, mcc-6-1 -6, opk-6 -6, mcc-8-2 -6 on the legs. A beam program
re-anchored or found live is spent past its own verified landing, not at its
controls' end: the oracle holds the last control on, and the replay does too.
Same-host A/B of the moving anchor and the walk skip together (2026-10-28,
31 routes): wasm -3.90 s, JS -3.33, nohop -1.91; mcc-4-2 -0.64, basic3 -0.57,
parkouradv1 -0.52, mcc-8-2 -0.41, opk2-1 -0.39 (map timer 82 to 75), mcc-8-1
-0.37, mcc-2-3 -0.30, opk-6 -0.30; nothing slower beyond its noise.

**A kick or pivot plan is searched live in its own family** (`SEED_FAMILY`,
`HopOracle.hopFrom` seed, `physics.seedFamilies`). The kick and the pivot are
the planner's families only; a body come down beside such a plan's line found
nothing in the live families' whole budget (every `from none` of the
2026-11-04 trace was a budget cut, 101-140k of 100k) and stopped, lined up,
pressed. At a program node whose plan is a kick or pivot the live search tries
that family first, then the other, on 150k of their own; at a neo the kick
after the live families find nothing. Offline at the traced states: a pivot
13t from mcc-7-1's landing (36 ticks to touchdown flown), a pivot 23t and a
pivot 17t on mcc-8-2 (29 and 28), a kick 17t from mcc-6-2's neo (24). The
skip-ahead takes a kick or pivot target too, seeded, and prices the plan's leg
no lower than its program's ticks (`SKIP_PLAN_LEG`: the floor alone priced
mcc-7-1's 18t pivot plus its 14-tick exact line-up as a 4-block jump, and the
13t skip lost). A live pivot replays from the state it was found from, so its
`precise` (an exact line-up) never comes into it. The cost: a seeded search
that finds nothing takes 300-400 ms of the main thread instead of 70-90 (no
ticks: mineflayer catches the backlog up).

**A partial path run out is coasted off, not stopped dead** (`PARTIAL_END_COAST`).
`fullStop()` zeroes the velocity and snaps the body to its block's centre:
right at the goal, wrong at the end of a partial whose solve is still running.
mcc-7-2's whole path came 1.1-1.5 s in; past ~1.4 s the bot ran out of the
first partial (cut before the ice), was snapped 0.57 off its line at 0.17 b/t,
and the program's line-up from that dead rest failed its anchor and pressed
(+6 ticks). Now the keys are let go and the body coasts to rest, where the
coast with no keys is verified to stay on the support at its level
(`holdsOn` with `maxDrop` 0.01: no step down off the node the next path leaves
from) and no fire, lava or cobweb lies in the 3x3 columns round it (`holdsOn`
sees neither: a coast into them still "holds"); else the stop as before.

**A live start flush against a wall keeps only the nudges that fit**
(`LIVE_NUDGE_SAFE`, `HopOracle.requiredNudges`). `robust()` flies a live
program from its start and four copies 0.01 off it; a body pressed flat on a
wall had one copy 0.01 INSIDE it, which no kernel clamps against the box it
already overlaps, so it flew a path the body cannot and failed every
candidate. A copy is dropped only when it collides and the start does not,
never both of an axis, and the beam builds the same ensemble. mcc-6-2's neo,
landed flat on its pillar (z = -364 + 0.30001): no live family in 289k ticks
before, run-turn 16t in 2k with it.

**Speed tricks, measured in the kernel** (bench `speedTricks.ts`): head-hitter
hops TAPPED (jump released in the air: the held jump re-arms only every 10
ticks) 0.487 b/t on stone against 0.355 sprint-jumping in the open, 0.967 on
ice, 0.995 on blue ice; held, no gain at all. The gait taps (a flight presses
jump on its first tick only) and its pruning bound now lifts to 1.4 where a
slippery floor is in reach (`GAIT_VMAX_SLIP`; the fixed 0.5 lost half a tick on
ice under a ceiling). The kernel jumps off honey at 0.4 where prismarine does
(`SIM_HONEY`: the cell under the floored feet, so a honey STACK, not one block).

**A kick plan's lane, pursued** (`LANE_KICK`, `classes(..., hint)`): from a body
come down 0.2-0.4 beside a kick plan's run lane (mcc-8-2's 3/16 panels), a
run parallel to the lane keeps the miss; classes that steer at a point 1.5-2
blocks past the plan's own take-off on its lane, the plan's kick and its two
neighbours, every jump tick, on 40k of their own before the seed families.
**A precise plan lined up from the moving body** (`MOVING_LINEUP`,
`HopOracle.movingLineUp`): the glide of the moving body to its rest first
(its anchor, if it holds, is the line-up: let go), else one or two presses
from the moving body aimed from that glide's end, every candidate coasted to
rest and anchored there, the prefix held from the nudged bodies too; once per
approach, after the live search and the moving anchor. mcc-7-2's 107t chain
anchored at t33, not t35-41. Same-host A/B of these two, the wall-safe
nudges and the slippery bound (2026-11-06, all 60): -1.54 s; mcc-7-2 -0.71,
mcc-8-2 -0.63 (stand-still 25-79 ticks -> 4), mcc-6-1 -0.21.

**A teleport across a stale gap is a teleport** (`forcedAtTick`): the fall test
skips a move over ticks it did not see, and a server position in that gap was
counted as a fall: opk-8 raced after mcc-7-2 stood 8 ticks settling a
"61-block fall" at its start.

Same-host session A/B of the seed families, the plan-leg price and the coast
(2026-11-05, all 60 routes, ABBA, quiet CPU; the coast's hazard and level
guards came after it): 59 arrive in both arms, -2.02 s in all; mcc-7-1 -1.14
(t -66: the skip's pivot 13t from the landing, 23 ticks on the leg),
mcc-6-2 -0.35 (t -26: the neo's kick 17t), mcc-7-2 -0.34, mcc-4-2 -0.24;
nothing slower beyond its noise (opk2-2 +0.14 and opk2-1 +0.11 one run each,
basic1 +0.11 at t 1.6).

**A riser's eased flights** (gaitSearch.ts `RISER_EASE`). A jump the path asks
for that scrapes a riser on its first three rising ticks has the flight it
took off from flown eased too, forward let go up to six ticks before its
touchdown: a shorter, slower landing that leaves room to clear the riser.
opk-2's hop onto the slab came down 0.17 short of the stair, the jump up
bonked it (0.27 to 0) and crawled on from 0.13; eased 6, the jump clears it
at 0.26. Only for jumps at depth 1 at most (nested, the eases multiplied the
work past the budget), on 0.4 of the work left (spent on them, opk2-1's run
phase had no answer and hopped into a header), and the eased landings are not
dominated by the un-eased flight's faster bodies ahead (before a rise the
faster body is the one that bonks; that rule for every rise gained nothing and
blew the work on the ten-ways courses). Same-host A/B with the along-track
next-leg gate: -2.55 s over the 31-route book (basic3 -1.27, climb2 -0.54,
opk-2 -0.38, mcc-2-3 -0.33), no route slower beyond its noise.

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
older rule hurts switches the rule for the session. A teleport (the body
moved further than a tick can) ends the fall and any wait owed to the last
landing: the server put the body anew (arena opk-9 stood 7 ticks at its start
for opk-7's hurt landing).

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

Known limits: mcc-1-1 is impossible as built (the gap needs 10.3 blocks of
carry, a best jump gives about 8; the original map has the same gap, and its
command blocks only do particles, sounds and checkpoints); the ice routes (7-1, 7-2) need a chain that keeps its heading and speed
through zig-zagging pads, which the beam does not find yet; 3-1 (a drop onto
slime whose rebound threads two offset slots onto a ladder) and 6-1 (neos off
one-wide stairs round floating pillars onto 2/16 rails) are not solved.
A flat walk node a block away can still get a sprint jump
(climb2's diagonal corner: the landing only settles back, 25 ticks on the
raised block beside it, the same in the previous build).

## Batch of 2026-11-08

All-60 ABBA t42 against every switch below off: 613.87 -> 609.15 s, 59 arrive in
every pass. Each is `PF_<NAME>=0` to switch off.

- `CLIMB_HOLD`: a body on a climbable with its next node above holds jump from the
  first tick, sneak only while falling (it stops the first 0.15 slide; rising it only
  cuts the steering to 30%). Executor and `Search.climb`/`climbsUp` alike. Yields to a
  verified flight passing through unless this column holds the next, the just retired
  or the following node. The keys are the hold's to release (`releaseClimbHold`).
- `GAIT_REUSE`: the rounding policy's Search is skipped when the plain one met no
  corner it would round: the same tree, decision-identical.
- `GAIT_GROUND_PASS`: the gait passes a parkour node from high over it where ground goes
  on beyond it, as the executor retires it (mcc-7-1). Costs mcc-6-3 about 0.15 s: a run
  now wins a marginal choice over a hop (gaitSearch `bound`, any gain with a jump ahead).
- `LIVE_PREFIX`: up to 4 ground ticks, then one jump, from a flush rest (mcc-6-2), 80k
  ticks a search. `PREFIX_RETRY_CAP`: one retry per node after a correction, 8 per goal.
- `BONK_FIRE`: a jump is a take-off when the kernel re-armed its cooldown, bonked or not;
  the oracle could never fly a take-off under a 2-high ceiling. `FLIGHT_CAP`: 600 ticks
  for any flight, oracle and replay (touch-and-go legs restart their own limits).
- `REGION_SETTLE`: a region program down off its place's level settles there as
  `stays` verified, no keys, sneaking; its `ticks` run to that arrival, so a 3t bonk
  onto a stair top (11 to its place) no longer beats a 7t walk-off (mcc-6-1).
- Safety: `REBOUND_CAP` (a boxed-in rebound-jump flight looped for ever), `TOUCH_WET`
  (a touchdown taken without a stay could end in lava), `FORCED_IDLE_REPLAN`,
  `FORCED_RELEASE` (a correction off the path let go of the keys), `LOST_PER_GOAL`.

Tried and removed: an outward lean to make the hanging traverse let go above its node.
The rounded heading's wall contact is what keeps mcc-4-3 hanging under its beam; without
it the body fell 40 blocks. Do not add it back.

Not done: `work/codex/lostfresh.patch` (a fresh solve after a lost program; the arena
cannot exercise it). mcc-8-2 stays bimodal, 7.6 or 8.75 s: a flight just past a node's
box does not retire it, slows towards it, and the planned jump no longer fits.
