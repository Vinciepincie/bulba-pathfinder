# Extended parkour

`Movements.allowParkourExtended` enables table-based diagonal, long, raised
and drop jumps, including ladder and liquid catches. It requires
`allowParkour` and `allowSprinting` and defaults to false. The table replaces
the upstream cardinal parkour generator while enabled.

The planner uses measured reach and swept collision corridors to find
candidate jumps cheaply. The executor then checks take-offs against live
physics. [VelocityInSearch.md](VelocityInSearch.md) explains carried momentum;
[PhysicsHops.md](PhysicsHops.md) explains the simulated control programs used
when the table cannot describe a route.

## Reach and run-up

[src/parkourEnvelope.ts](../src/parkourEnvelope.ts) stores horizontal reach
and flight curves measured with prismarine-physics. Reach is travel from
the jump point, separate from the support and landing credits. Height
buckets cover +1, level, and drops through -8 blocks. Deeper drops use the
last bucket conservatively; `maxDropDown` separately limits landing depth.

`J_RUN` measures runs of 0, 0.4, 0.8, 1.2, 1.6, 2 and 3 blocks before
take-off. Even a short run matters: at level height, 0.4 blocks of run raises
the tabulated reach from about 2.52 to 3.07 blocks. Further gains diminish:
the 0.8-block row reaches about 3.19 and the 3-block row about 3.24. This is
why a narrow support can provide useful acceleration without a full runway.
The planner includes the support's usable width and an eligible cell behind
it, including a step up within 0.6 blocks, then chooses the longest measured
run no greater than the available distance.

The rows subtract `ENVELOPE_SAFETY_MARGIN = 0.1` blocks for timing
uncertainty. `Movements.parkourSafetyMargin` adjusts that reserve.
`allowParkourTight`, true by default, permits a fallback with the reserve
restored at an extra cost of 6, so comfortable jumps remain preferable.
These are planning margins, not a promise that every planned jump executes.

Reach uses the actual rise between collision tops, interpolating fractional
heights between rows. A fence top can therefore be level with a destination
whose planner node is a cell higher. A bottom stair is caught on its
full-footprint slab at half-block height before the body walks up its step;
the lower landing gives the flight more airtime.

## Support and catch geometry

The player is 0.6 blocks wide. On a full support, standing take-off credit
is 0.6 blocks from the centre on the widest axis. That leaves a 0.2-block
support patch under the hitbox. The separate running credit is 0.4 along the
flight line. A full landing supplies 0.8 blocks of catch credit per axis:
half a block plus the hitbox radius. A diagonal jump must apply these credits
on each axis, rather than subtracting them once from centre-to-centre distance.

[src/shapes.ts](../src/shapes.ts) classifies narrow supports with
`topCatchClass`. Its four credited half-widths are 0.5, 0.25, 0.1875 and
0.125 blocks, for wide tops, heads or wall posts, pots, and fence posts or
narrower shapes. Take-off credit is `min(0.6, halfWidth + 0.28)` and landing
credit is `halfWidth + 0.3`. The take-off stays 0.02 inside the overhang limit.
The executor reads the live support faces to bound its creep.

Classification uses the union of faces within step height of the top.
Classifying boxes separately loses the full top of an upside-down stair,
whose boxes tile the cell. Fence arms do not turn a post into a full top:
their union still leaves only a narrow centred square.

`carryCode` adds direction-dependent credit for edge panels and lines, such
as open trapdoors, ladder strips and panes. A centred width alone would
credit space on the empty side and lose usable space along the strip.
Diagonal walks between panels require a common supported body position;
otherwise the move is a hop. Crossing a panel's narrow width also removes
run-up and creep credit and adds a reach reserve.

## Generated corridors and costs

[src/parkourTable.ts](../src/parkourTable.ts) enumerates integer offsets with
components up to `MAX_OFFSET_MAJOR = 6`, excluding adjacent moves and offsets
beyond the deepest running reach after support credits. The base cost is
`max(distance + 0.5, octileDistance)`. The pad prevents jumps replacing
comparable walks for free; the octile floor keeps an edge from costing less
than the heuristic's distance over it.

A minimum cost based on a whole jump's airtime is unsuitable for short hops.
It prices every hop as though the body stops to line up and encourages long
walking detours. A sprint-hop carries speed through the arc; alignment costs
belong to the alignment, not to every jump.

Each entry stores cells swept by the flight and minimum feet heights over
those cells. Cells on the flight line are distinguished from clipped corner
cells: walkable floor along the line makes a jump unnecessary. Obstacles
below the body matter only if their tops protrude into the arc, with a small
grazing allowance. This lets a dropping jump clear a nearby corner or fence
under its apex without treating the same obstacle near take-off as clear.

The centred sweep uses the 0.3-block hitbox radius minus a 0.15 corner-nick
tolerance. Vanilla resolves collisions per axis, so a shallow graze can slide
past a corner. Aimed sweeps use a smaller 0.1 tolerance. Head sweeps use a
0.29 radius because vertical collision resolves before horizontal movement.
The executor simulates the actual contact before committing a take-off.

JavaScript builds and caches the geometry once. The WebAssembly host uploads
the serialized table through `ext_begin`, writes at `ext_ptr`, then calls
`ext_commit`; subsequent solves use the resident table.
[rust/src/lib.rs](../rust/src/lib.rs) describes it as approximately 0.7 MB
serialized. Rust consumes the shared geometry and clearance bounds rather
than reconstructing them. Older cores without the residency exports use the
host's per-solve compatibility upload.

## Landing selection

[src/moveGen.ts](../src/moveGen.ts) resolves the target column before checking
reach. A climbable at flight level is a catch, not a landing on its thin
collision top. A carpet-like thin floor holds feet inside its cell rather
than one cell above it. A physical block or tall fence/wall top can be a
raised landing if its rise is at most 1.2 blocks and headroom permits it.
Otherwise the scan descends to a physical top, thin floor, usable climbable,
water or bubble column. Drop limits apply, with the configured liquid-depth
exception and bubble-column handling.

The extended lookup table rejects big dripleaf as a dependable support,
removes support from states with no collision boxes, and marks powder snow
unsafe. A block type's broad classification alone cannot describe these
states: a leaf can tilt away, an open gate carries nothing, and powder snow
sinks the body.

Climbable catches and support landings need different credits. In particular,
a ladder catch needs the body's centre inside the ladder cell. A catch one
cell above take-off uses a reduced 0.45 credit, must approach through an open
side, and requires another planner landing below it if the catch is missed.

The extended profile rejects airborne landings flanked by lava at feet or
head height. Near-edge catch credit would otherwise admit landings whose
hitboxes overlap the lava. Hazard corner checks and a small neighbouring
hazard cost also keep equivalent paths farther from fire, lava and cobwebs.

## Ceilings and late take-offs

A ceiling changes the arc instead of simply forbidding the jump. A full
ceiling two blocks above the feet clips the rise at 0.2 blocks; separate
`J_LOW_RUN` rows retain short level hops and longer drops but reject +1
landings. Graded rows model top slabs, top trapdoors and higher full ceilings
at their own collision heights.

`lidWalk` checks the arc tick by tick over each ceiling span. A lid left
behind before the body rises into it does not cap the whole flight; entering
a lid side-on is different from bonking underneath it. A slower take-off can
sometimes pass below an overhang above a drop landing. Aimed dives under
that landing overhang are excluded from this table model. The shelf and
trapdoor geometry on `opk2-5` illustrates why one blanket low-ceiling class
is insufficient.

A lid confined to an edge support's strip can be left before the jump fires.
Vertical collision sets `onGround` before horizontal motion, so the flag can
remain set for the first tick without support. `leavesLid` recognises these
late take-offs. The physics oracle's `leave` program fires on that event,
rather than relying on a start-sensitive tick count. Momentum re-jumps under
lids remain excluded because the table has no measured bonked chain row.

## Aimed lines and neos

A blocked centre line can have a clear route beside it. `AIM_VARIANTS` has
24 entries represented as `[pOff, qOff, wOff, wFrac]`: lateral take-off and
landing offsets, an optional waypoint offset, and its fraction along the
jump. Equal endpoint offsets produce parallel lines; unequal offsets aim
across the gap. A waypoint bends the route beside a pillar and back behind
it, describing a neo. These variants are tried when corridor geometry blocks
the centre line, with extra cost for alignment and steering.

`avoid` and `lineAvoid` masks prune variants as obstacles and floor cells are
encountered. An endpoint moved off-centre needs a full support top. The
checks are per endpoint, and `AIM_P0_MASK` restricts narrow take-offs to
variants with `pOff = 0`.

`META_AIM` marks the move. Its separate aim byte holds a one-based variant
index in bits 0-6 and the mirrored side in bit 7. `Move.fromRaw` decodes it
into `pOff`, `qOff` and `wOff`; path post-processing offsets the landing aim
and records a take-off aim in `Move.takeoff` for straight lines. A neo uses
a searched run and turn script instead of a fixed take-off point. The
variants describe candidate geometry, while physics validation decides
whether the controls can fly it.

## Ladders and slime

Ladders support cardinal entry, airborne and drop catches, and transfers
between adjacent climbable cells, including around a pillar. A hanging body
has no standing jump footing. The top strip of a ladder is a separate,
restricted take-off: standing credit along the strip, no runway or momentum
lip credit, no raised landing, and an added cost for climbing out and settling.
A ladder beginning at head height can be entered by jumping up into it.

A slime bounce is a compound edge with the slime stand stored as `via`.
Measured rebound apex depends on drop height: about 1.30 blocks after a
2-block fall and 5.16 after an 8-block fall. The table reserves height for
horizontal drift and considers nearby higher supports. The executor aims at
the slime until rebound begins, then at the destination, without sneak,
which would cancel the bounce. More complex and deep rebounds use the
physics programs described in [PhysicsHops.md](PhysicsHops.md).

## Chaining and execution

A landing-tick re-jump retains more speed than a fresh run on a small support.
Without momentum state, `momentumChains` represents two hops as a compound
edge through a stepping stone, with both costs summed and the stone in
`via`. The continuation must satisfy `CHAIN_MIN_COS = 0.70`, roughly a
45-degree turn. Catches into water, thin floors or climbables are not ordinary
grounded re-jumps. Aiming halfway through the stone's standing creep credit
leaves room to land without sliding off its far edge.

`allowParkourMomentum` carries landing direction in the search instead,
allowing longer chains and additional lip take-offs. Its state, turn-loss
model and conservative drop cutoff are described in
[VelocityInSearch.md](VelocityInSearch.md).

Live rollout gates remain necessary because the table approximates support,
speed and collision geometry. Their jump simulations allow 45 ticks so deep
drop landings and the run-in fit. When a jump does not validate, the executor
can search headings, creep within the support, or, with `allowRunUp`, back
off and build speed. Recovery steps are simulated individually: a fixed
multi-tick retreat can overshoot the small clearance a jump actually needs.
The executor refuses take-offs it cannot validate; world changes and server
corrections can still invalidate a flight after launch.
