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
TAKEOFF_STAND = 0.6; running: the rollout delays the jump, TAKEOFF_RUN = 0.4
along the line) and catches the landing block's near corner (LAND_HALF = 0.8
per axis). A diagonal (3,3) hop therefore needs ~2.26 blocks of flight, not
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
