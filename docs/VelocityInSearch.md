# Momentum in the search

`Movements.allowParkourMomentum` adds landing direction to extended-parkour
search state. It defaults to false and requires `allowParkourExtended`.
The implementation is in [src/moveGen.ts](../src/moveGen.ts),
[src/solver.ts](../src/solver.ts) and [rust/src/lib.rs](../rust/src/lib.rs).

## Why direction matters

A cell alone cannot distinguish a body resting on a post from one landing
there at speed. The support's own run-up is enough to model many ordinary
jumps, but a re-jump on the first grounded tick can carry farther. On a
course of posts, heads and pots, stopping at every landing can remove the
only way to reach the next support.

The search records momentum only for parkour landings on narrow supports
(`topCatchClass >= 1`). It stores the exact primitive horizontal direction,
reducing the flight offset by its greatest common divisor, rather than
rounding it to eight compass bins. A chain's turn test therefore uses the
actual incoming and outgoing directions.

Walking does not create momentum states. Full-width landings do not carry
them either: their own run-up and lip take-off already cover the useful
reach, while extra direction states would multiply expansions without
opening useful moves. Run reach also saturates quickly: the flat `J_RUN`
row gains only about 0.05 blocks between 0.8 and 3 blocks of run.

## Re-jump reach

From a momentum state, the planner first tries the support's ordinary reach.
If that is insufficient, it can use `J_CHAIN` from a far-side landing point.
The continuation must satisfy `CHAIN_MIN_COS = 0.70`, roughly a 45-degree
turn, and its usable reach is reduced by
`CHAIN_TURN_LOSS * (1 - cosTurn)`, with `CHAIN_TURN_LOSS = 0.8`. Turning
retains less useful speed than continuing straight; applying the full
straight-chain row throughout the cone overstates reach.

The re-jump fallback excludes aimed lines, lids, panel-width exits and the
special catch one cell above take-off. Catches do not retain direction for
another re-jump. The stone aim uses half its standing creep credit, leaving
room to land without sliding off its edge. Direction can persist across successive
eligible landings, so this is not limited to the two-hop compound edges used
when momentum state is disabled.

`MOMENTUM_MAX_DROP = 1.75` conservatively limits the downward support-to-support
rise that retains direction. A jump's roughly 1.25-block apex adds descent
before landing. Damaging landings can remove speed through a server velocity
update, so the planner does not rely on long-drop momentum. Actual damage
and packet waiting are handled separately by the executor, as described in
[PhysicsHops.md](PhysicsHops.md).

## Sparse states and path representation

A search node is `(cell, momentum)`, with zero meaning no momentum. Ordinary
states occupy the cell-indexed part of the reusable, epoch-stamped arrays.
Directional states are created on demand in a secondary region linked per
cell (`slotFor` in JavaScript, `slot_for` in Rust). This avoids allocating a
full cell-by-direction grid. With the flag disabled, the search uses only
the ordinary cell states.

Momentum stays internal to the solver. An outgoing re-jump uses the existing
chain metadata and sets `via` to the current stepping stone. `Move.expandRawPath`
aims that existing stone toward its far side instead of inserting it twice.

## Lip take-offs

The flag also enables a later running take-off when ordinary creep credit
falls short. Vertical collision sets `onGround` before horizontal movement,
so a jump can still fire on the first tick after the body leaves support.
The planner credits the support's overhang limit plus `LIP_PHASE = 0.14`,
the midpoint of a 0.28-block stride window. Dedicated `mfLip` clearance
bounds cover the later, lower rising arc; a lip jump also receives its own
ceiling check.

This is distinct from landing momentum. The `(0,5)` stair hop on
`parkouradv1` combines a late take-off with a catch on the destination
stair's low slab. Treating it as a full-height landing or attributing all
its reach to incoming speed obscures the geometry that makes it possible.

## Live validation

The executor checks take-offs against live physics and refuses those it
cannot validate. It tests a chained re-jump from the actual landing state,
including its velocity, rather than assuming the search's direction label
supplies enough speed. Run-up and timing retries can recover a refused
take-off. A validated launch can still be disrupted by world changes or
server corrections; the model does not guarantee that a flight cannot fall.
