# Extended parkour (`allowParkourExtended`)

Improvement over upstream mineflayer-pathfinder, whose parkour is
cardinal-only and same-level-only — as are baritone's and azalea's (checked
2026-08-19). With the flag on (plus `allowParkour` + `allowSprinting`), the
solver generates every straightforward sprint-jump a vanilla player can make.
Nothing is hand-picked: the move set is **computed** from the game's actual
jump physics.

## The reach envelope (`src/parkourEnvelope.ts`)

`test/helpers/jumpEnvelope.ts` simulates the exact jump the executor performs
— prismarine-physics, holding forward+sprint+jump — and measures horizontal
reach per landing height (+1 up, 0, −1 … −8 down) for a **standing** start
and a **running** start (≥1 walkable cell behind the takeoff; speed saturates
within a block, measured). The resulting table is pasted as constants;
`envelope.test.ts` re-derives it on every run and asserts equality, so the
constants can never drift from the physics. Values include the near-edge
landing credit (+0.8: a block can be caught by its edge and walked to center)
minus a 0.3 safety margin against frame-perfect plans the executor sim would
refuse (a refusal = 3.5s stall + replan).

Standing jumps are measured **from the toe**: when no jump validates yet, the
executor creeps up to 0.15 blocks toward the gap (hitbox still on the block)
and keeps re-testing — a player walking to the lip before a standing jump.
Without the creep the bot froze at the node center and standing reach ended
at 3.12, refusing the "simple" (3,1) diagonal (the prod 19:25 noPath).

Anchors, all live-verified: standing (toed) reach spans the (2,2) pillar hop
and (3,1), but not (3,2); running reach covers the classic 4-block jump;
drops extend reach monotonically (more airtime = more distance — a −4 drop
carries a running jump ~5.7 blocks). The executor's jump simulation budget is
45 ticks (upstream: 20) so deep-drop landings plus their run-in fit.

## The generated table (`src/parkourTable.ts`)

Offsets are enumerated, not listed: every integer landing (a, b) with
2 ≤ distance ≤ 5.66 (matching the deepest default-profile drop). The cost is
`max(dist + 0.5, octile(a, b))`: euclidean plus a tie-breaking pad, floored at
the solver's own heuristic. The pad alone is not quite enough at the
enumeration limit — the octile-minus-euclidean deficit peaks at ~0.082 × major,
so (2,6), (3,6), (6,2) and (6,3) came out **below** the heuristic by up to
0.035 and made A* inadmissible. Flooring those four is preferable to lowering
`MAX_OFFSET_MAJOR`, which would delete real reach. For each offset the
**swept corridor** is computed by clipping the flight segment against each
cell rect Minkowski-inflated by the hitbox half-width **minus a 0.15
corner-nick tolerance**: cells the center line crosses are *line* cells
(walkable floor there voids the jump — it isn't a gap), edge-clipped cells
are *corner* cells. The run-up cell is the backward continuation of the
flight line.

The nick tolerance matters: inflating by `0.3 − 0.15` is exactly "the hitbox
penetrates the cell by ≥ 0.15 on BOTH axes", so a flight that merely clips a
block's corner by a few centimetres is no longer treated as a wall. Vanilla
resolves collisions per axis — a corner nick costs a few centimetres of
travel on one axis and the player slides on (and aims a hair off-line
anyway), while any real cut through a block still exceeds the tolerance and
vetoes. This is what allows diagonal jumps that pass a pillar standing
right beside the landing block (the prod capture of 2026-08-20). The
executor's per-tick physics rollout is the backstop: it simulates the actual
graze before committing the jump.

Each corridor cell also carries **mf**, the minimum feet height (relative to
the takeoff walk level) while the hitbox overlaps that cell, evaluated on the
**flight curve** — the tick-by-tick prismarine-physics arc for the takeoff
class (`FLIGHT_STANDING` / `FLIGHT_RUNNING` in `parkourEnvelope.ts`; the arc
is target-independent because the executor holds the same controls for every
jump, and it is unimodal, so the minimum over a swept interval is at an
endpoint). Corridor rule per cell: body cells (those the vertical range
`[mf, +head]` passes through) must be passable, and a cell below the body
vetoes only when its collision top pokes above `mf` (+0.05 grazing
tolerance, one extra cell probed because fences reach 1.5). Consequences:
same-level corners near takeoff never veto a drop-jump (the feet are at apex
over them), and a fence under the apex is legitimately cleared, while the
same fence right after takeoff still vetoes.

The table is built **once, in JS only**, and shipped to the Rust/wasm core
inside the solve parameters (~4 KB) — one implementation of the geometry,
nothing to keep bit-identical by hand (the mf floats are consumed, never
recomputed, by Rust). The differential suite still pins the two engines to
identical outputs.

## Landing resolution, per offset

At most one landing per offset — whatever the target column offers first:

- **catch a climbable at flight level** (checked before PHYSICAL: ladders
  classify as bbox-'block' physical, and grabbing one is real while standing
  on its thin collision top is not; vines only count with a wall behind,
  mirroring `moveUp`),
- **thin floor at flight level** (carpet class: SAFE+PHYSICAL, not
  climbable): a **same-level** landing — the feet land IN that cell on its
  sub-block shape, exactly like the air-branch landing below. Treating it as
  the up variant would put the node one cell up in the air (the prod
  carpeted-storage-room "No path", 2026-08-20 — same rule applied in
  `findLanding`/`findExtLanding`/`moveDiagonal`/`moveForward`/`moveJumpUp`),
- **up (+1)**: the flight-level cell is itself the landing block (rise ≤ 1.2),
- otherwise scan the column downward: a **physical top** lands on it, a
  **thin floor** lands in it, shallow **water**
  (`infiniteLiquidDropdownDistance` honored), a **bubble column** (unlimited —
  columns catch like water) or a lower **climbable** catches; depth is capped
  by `maxDropDown`, and the corridor's per-cell mf bound covers the arc
  dipping under takeoff level near the target.

The landing then passes the envelope: `flightNeeded ≤ J[speed][dyBucket]`,
where `flightNeeded` is precomputed per offset with PER-AXIS takeoff/landing
credits (parkourEnvelope.ts): the 0.6-wide hitbox takes off from the block's
corner (standing: the executor sneaks to the corner overhang, widest axis at
TAKEOFF_STAND = 0.6 — a 0.2 support patch under the box, which is what keeps
the executor robust to server corrections on full blocks; running: the rollout delays the jump,
TAKEOFF_RUN = 0.4 along the line) and catches the landing block's near
corner (LAND_HALF = 0.8 per axis). A diagonal (3,3) hop therefore needs ~2.26 blocks of flight, not
the 4.24 center-to-center distance — this is what makes 1x1 pillar courses
jammable from a standstill, exactly like real players (mcpk.wiki's per-axis
formula; validated against its tick math: our standing flat J is the wiki's
2.6234 jam figure). Speed upgrades from standing to running only if the
run-up cell is walkable at takeoff level. Drop-jumps reach farther than flat
jumps and up-jumps shorter, exactly like the real game. The corridor's mf
curves are looked up shifted by the takeoff offset, so cells behind the
launch point (crossed walking, at ground level) keep their full poke veto.

**Head-hitters**: a blocked head+1 anywhere over the corridor (takeoff
included) no longer vetoes — it selects the bonked-arc class: J_LOW rows and
FLIGHT_LOW curves simulated with a solid lid 2 above the feet (the arc bonks
at +0.2, prismarine-physics collision). +1 landings are impossible there
(sentinel row), flat hops keep ~0.86 usable flight and drops most of their
reach — 2-high tunnel gap-hops and drops off ledges under an overhang, like
real players. The executor needs no change: it holds jump and the per-tick
rollout simulates the bonk against the real world. Neos and momentum chains
remain out of scope — the executor only holds jump+sprint+forward.

## Run length, not "standing" vs "running" (`J_RUN`)

The envelope's first model had two takeoff classes: a jam from rest at the
lip, and a running jump with a walkable cell behind. A recorded human run
of the arena's parkouradv1 showed that nobody jumps from rest: the player
sprints across whatever support there is — from its rear overhang to its
front lip — and jumps at the edge, and the physics brute force reproduced
every "impossible" hop of that course exactly that way. Even 0.4 blocks of
run adds half a block of flight; a fence post's ±0.405 support zone gives
0.81 of run, a head 1.06, a full block 1.38, and each walkable cell behind
the flight line — level or one step lower — adds one; reach saturates by
about 2. So the rows are now per **run length before the lip**
(`RUN_LENGTHS` = 0 … 3, `J_RUN` / `J_LOW_RUN`, derived by
`measureRunTakeoff`, pinned by the envelope test), the solver sums the
support's run and the run-up cell, takes the longest measured row not above
it, and credits the takeoff at the lip; `J_STANDING` is row 0 and
`J_RUNNING` is what row 2+ used to be. The reach bucket is the REAL rise
between support tops — a fence stand (feet 0.5 into its node) to a head is a
flat jump one node cell up — with fractional rises interpolated between the
integer rows. The executor matches: a parkour jump no gate authorises from
where the body stands makes it back off to the rear of the support (once
per node, each step validated by a rollout) and sprint in, the delayed-jump
rollout picking the tick.

The rows carry a 0.1-block safety margin (`ENVELOPE_SAFETY_MARGIN`);
`Movements.parkourSafetyMargin` re-tunes it, and the arena's `--risky`
sets 0 to plan the frame-tight jumps a practised player makes. What stays
out of the model at any margin: run-ups from a cell BESIDE the takeoff (a
90° turn at the lip), landing on a stair's low step for extra airtime, and
jumps with a brute-force window under 1% — on parkouradv1 that is the
stair-peak (0,5), the 5-fence → 2-fence (3,−3,5) and the pillar-top →
beam (1,5).

## Narrow supports: fence tops, heads, pots (`topCatchClass`)

Upstream classifies fences and walls (collision top > 1) as *not physical*
— right for walking, wrong for landing: a fence post is a real stand, feet
0.5 into the cell above it. With the flag on, the **tall-stand** class
(not passable, not physical, top > 1) is a landing everywhere a physical top
is: drop scans, the extended landing scan, the flight-level up-variant, and
`moveJumpUp` (rise ≤ 1.2 from the takeoff support, so a post is jumpable
only from another post or a half-step; the head needs the cell above the
usual pair). A post sunk one below the floor (top 0.5 above it) is simply
walked onto.

What a narrow top costs is **catch credit**. The height byte carries a
2-bit `topCatchClass` (shapes.ts): the largest centered square inside any
single top face within step height of the collision top — 0 full/wide
(slabs, stairs, chests), 1 the 0.25-half class (heads, wall posts), 2 the
0.125-half class (fence posts, flower pots), 3 nothing (a ladder's 3/16
edge). One face at a time, not the union: a fence with arms projects to a
cross whose bounding box fills the cell. The envelope credits generalise
from the full-block constants: takeoff = min(0.6, half + 0.28) — on a post
the creep goes to two centimetres inside the overhang limit half + 0.3 (there
is no other takeoff on a 1x1 top; "jump exactly from the corner"), a full
block keeps the 0.6 patch (crediting its lip re-planned the route book over
marginal jumps the executor then fell off) and landing = half + 0.3 (near-edge catch; 0.5 + 0.3 is the 0.8
above).
Full-block ends still take the precomputed `fnStand`/`fnRun` bit for bit;
a narrow end recomputes `flightNeeded` in the solver with the reduced
credits (sqrt of a sum, never `hypot`, so the wasm core rounds identically).
Running takeoffs need a class-0 takeoff and run-up cell — nothing narrower
hosts a run-up. The executor caps its corner creep with the same credit,
read from the live block's shapes: without it the sneak guard parks the body
at a post's edge short of the full-block creep target and the jump never
fires.

Two consequences worth knowing when authoring a course: a post-to-post
(3,1)+1 needs 2.26 blocks of flight against 1.69 usable from a standstill —
legal between full blocks, on posts only as a momentum chain — and a
(2,2)+1 from a post onto a head needs 1.60. `hopMath.ts` in the arena
scratch folder prints the numbers.

## Ladders: entry, catch, transfer — never the top edge

A ladder cell is entered cardinally from an adjacent stand (`moveForward`
with no floor under the target — the ladder catches), caught by a jump at
or below flight level (already), caught by a plain drop (`findLanding`),
and **transferred** from: `climbTransfers` steps from one climbable cell to
an adjacent one — along a wall or diagonally round a pillar corner — one
up, level or one down, with one open corner column to sweep through. That
is how a spiral of ladders on a 1x1 pillar is climbed: press into the corner
at the top of one ladder, drift round onto the next (prismarine-physics
climbs on held jump, and the executor's ordinary rollouts drive it). Cost
octile + |dy| + 0.5, admissible.

No extended jump is generated FROM a climbable cell (a hanging body has no
footing to jam from; upstream's flat-cost cardinal parkour stays superseded
there too), and nothing jumps ONTO a ladder's top edge (`moveJumpUp` and the
diagonal): a ladder classifies as physical, so a +1 hop between the top
edges of two ladders priced like a block hop — and a plan the executor
cannot fly. Climbing over the lip at the top of a ladder (`moveUp` to the
cell above, then off it) is unchanged.

## Momentum chains (`META_CHAIN`, `via`)

A body that lands from a sprint-jump and presses jump again on the landing
tick keeps its speed, and prismarine-physics says that takeoff out-flies
even a running start: `J_CHAIN` (measureChainTakeoff, pinned by
`envelope.test.ts`) gives 2.59 / 3.50 / 4.10 / 4.40 blocks of usable flight
for +1 / flat / −1 / −2 landings against 2.48 / 3.33 / 3.89 / 4.18 running
and 1.69 / 2.42 / 2.93 / 3.19 standing. This is how a player crosses a
course of 1x1 posts where no stand offers a run-up — and why a post-to-post
(3,1)+1 that no standing jump reaches (2.40 needed) is a real jump when
the post is entered at speed.

A* has no speed in its state, so the chain is a compound edge: for every
parkour landing B an expansion produces that is a **stepping stone** — a
support with no walkable class-0 cell beside it at its own level (any such
neighbour means B's own expansion has running jumps) and no lid over it —
every table offset that **continues the first hop's direction** (cos ≥
`CHAIN_MIN_COS` = 0.85, about 32°, in exact integer arithmetic — a 45° turn
lost enough momentum on the arena's simple2 to miss a (3,3)) is tried from B with the
chain row, and the ones B's own standing/running jump could not make become
edges A → C with B as `via`, priced at the sum of the two hops. Landings
that ENTER a cell (ladder, water, thin floor) have no landing tick and are
never chained; nothing is chained under a lid (no bonked chain row was
measured). The re-jump is credited HALF the stone's standing creep credit
(`CHAIN_TAKEOFF_FRACTION`) — the executor lands the first hop that far past
the stone's centre (`Move.aimDx/aimDz`, scaled by the support's catch class
in `postProcessPath`), which keeps the landing a hitbox inside the edge:
aiming at the lip itself put the body five centimetres from the edge at
speed and it slid off — and the corridor uses the running arc.

The executor expands the raw node into the stone (a plain parkour landing)
followed by the chain node (`Move.expandRaw`). The stone is flown and
landed like any other parkour node — its hole-beyond hold retires it on the
touchdown tick — and the chain node's handler then presses jump toward C on
that same grounded tick with sprint held, but only if the ordinary
sprint-jump rollout, run from that landing state (so with the landing speed
in it), says the chained flight lands; a refused re-jump leaves the node to
the normal gates, a stop rather than a fall. Once the body has left the
stone the ordinary in-flight handling flies the arc.

Measured on the arena's parkouradv1 course, chained corner jumps reproduce
the human line through the first fence cluster and across the stair peaks
in simulation; four 5-centre hops there still fall 0.7–1.2 blocks short of
the engine even chained, which is either an engine/vanilla gap or a
technique not yet modelled (a recorded human run decides).

## Slime bounce (`META_BOUNCE`, `via`)

A drop of `d ≥ 2` onto a slime block is also an edge to every landing the
rebound reaches: `BOUNCE_APEX[d]` (prismarine-physics, `measureSlimeBounce`
in the envelope helper, pinned by `envelope.test.ts`) is how high the feet
come back above the slime after a straight free fall of `d` — 1.30 at 2,
2.10 at 3, 2.55 at 4, up to 5.16 at 8 — and a support in one of the 12
columns around the slime with its top above the slime and under the apex
(ring 1: minus 0.2; ring 2 along a cardinal: minus a whole block for the
drift) is a landing, if its column and head cells are passable. The edge is
takeoff → landing with the slime stand cell as `via`, so the search never
has to know how much fall energy a node arrived with; cost octile + |dy| + 1
from the takeoff. Slime cells are marked into the special grid only when
the flag is on (`LutSpecial.SLIME`); the JS worker passes `via` through and
the wasm result blob carries it after the meta byte.

The executor drives a bounce node in two latched phases: aim at the slime's
top centre until the rebound has begun (feet at the slime, moving up), then
at the node — walking, never sprinting (the drop has to land on one block)
and never sneaking (which cancels the bounce). Landing on slime as a plain
stepping stone needs nothing: it is a full block, the physics rollouts see
the micro-bounces, and `onGround` is true on the contact tick.

Not yet modelled: **takeoffs from slime** (slipperiness 0.8 cuts ground
acceleration to ~0.42× stone's, so a standing jump off a slime block flies
short of the stone-measured envelope — the same is true of ice). Until the
envelope carries a slippery-takeoff class, do not author long standing
jumps that start on slime.

## What the executor adds on top

The table decides what is *reachable*; getting there is the executor's job,
and on real terrain a planned jump can still refuse to happen. Three
behaviours cover that, in the order they are tried, and all three exist
because a route on 2b2t spawn stopped on them:

1. **Turn to make it.** The take-off heading is searched over ±30° (0 first,
   cached per node) for one that lands the jump — the idea behind
   [ParkourCalculatorMod](https://github.com/Leg0shii/ParkourCalculatorMod)'s
   angle solver. A heading that works costs nothing; ground given up has to
   be walked again, so this comes first. Bounded: if the body has not left the
   ground within a jump's worth of ticks, the angle was not the problem.
2. **Go round the open corner.** A diagonal is priced on the cheaper of its
   two corners, which is only honest if the walker goes *around* that corner;
   the open one is inserted as a waypoint. Only once wedged on the node —
   doing it pre-emptively turns a straight climb into a staircase of 1-block
   sidesteps (measured: +14 blocks and +3.5 s on one route).
3. **Make room.** A step back or sideways, each simulated first so it cannot
   become the fall it was avoiding, ONE tick at a time. A fixed four-tick
   back-off overshoots by an order of magnitude — the clearance a step-up
   actually needs is ~0.01 blocks once the hitbox-precision fix is in
   (README) — and shows up as a visible wobble before every jump.

None of these run while the bot is making progress.

## Efficiency

Zero allocation per expansion; all probes are typed-array reads against the
snapshot. Flag off: no new code runs. Flag on: takeoff gates (liquid, bubble,
y+2 headroom) are hoisted per node, and each offset opens with a 2-probe
fast-out (walkable floor on the first line cell — kills virtually every
open-terrain candidate), so flat ground costs ~2 probes × ~27 offsets per
direction group; the full corridor checks only run at real gap edges.
Benchmarks are unchanged with the flag off and the differential fuzz runs at
the same speed with it on.

## Parity

- Default **off**. The upstream-oracle fuzz (`fuzz.test.ts`) and the prod
  shadow profile must never enable it — upstream cannot generate these moves,
  so reachability would diverge (same policy as `useBubbleColumns`).
- JS and Rust consume the same uploaded table at the same points in move
  generation (cardinal loop after `moveParkourForward`, diagonal loop after
  `moveDiagonal`) with identical probe order and float arithmetic — pinned by
  the wasm↔JS differential in `wasm.test.ts`, including 20 fuzz worlds with
  the flag on.
- Rust adds no exclusion-area cost: exclusion areas force the JS solver path
  (existing divergence, same as every other move).
- With the flag on the table **supersedes** `moveParkourForward` rather than
  coexisting with it: that generator is skipped entirely (JS and Rust alike).
  It charges a flat 1 for a jump covering up to four blocks — cheaper than a
  single walking step — so beside the table it wins every tie, which means it
  re-offers at a lower price exactly the jumps the corridor and envelope just
  vetoed, and lets A* buy distance with jumps (2b2t spawn: a plan costing 74.1
  against upstream's 76.6 that was 3.5 blocks *longer* to walk, and lost the
  race it should have won). The flat cost also makes the octile heuristic
  inadmissible (h = 4 over a cost-1 edge), so the search is not optimal under
  its own model either.

  Coverage is not a reason to keep it. Over 120 seeded worlds and 108k
  parkour-bearing nodes it reaches 22.5k targets the table does not, and a
  prismarine-physics rollout of a sample of those — tried from the cell
  centre, from the take-off corner, and with one and two blocks of run-up —
  can fly **1.1%** of them. The rest are jumps the bot cannot make, each one a
  planned stall. With the flag off it is still the bit-exact upstream port and
  the differential fuzz pins it. `test/movegen.test.ts` pins the supersession.
