# Velocity-in-search: momentum-aware A* for frame-tight parkour

**Status: SHIPPED 2026-08-26** — `Movements.allowParkourMomentum` (opt-in, needs
`allowParkourExtended`), JS + Rust/wasm mirrored, commits `86db699` (search
state), `01caca6` (lip take-offs + chain turn loss), `72a3eeb` (executor).
The arena's `parkouradv1` now completes end-to-end with `--momentum --risky`
(6 of 7 runs, ~18 s, 20 jumps, 0 replans); it was 0 of N. The plan below is
kept as written; this section is what the measurements actually said, and
where the build departs from the plan because of them.

## What was built, and why it differs from the plan

**Only landing momentum is a search state, and only on narrow supports.**
The plan's `DIR_d` for a *walked* sprint would have multiplied flat-terrain
states for nothing: the `J_RUN` rows saturate by ~0.8 blocks of run, so
every support's own run already earns within 0.05 of the top row. The
physically significant momentum is the landing-tick re-jump (`J_CHAIN`,
+0.36 over `J_RUN`), so a node is `(cell, momentum)` where momentum is the
exact primitive flight direction of the parkour landing that reached it
(`momentumOf`, 13×13 packed, never a bin — the re-jump cone is the same
integer test the compound chains use). That makes `momentumChains`' two-hop
compound edge a special case: chains of any length, from any narrow
support, no `CHAIN_CAP`. Full blocks carry none — their lip take-off (below)
out-reaches a landing-tick re-jump in every bucket, and giving them a state
was 1.1–5× the route book's visited counts for no edge the cell lacked;
narrow-only, the counts are within 5% of the flag-off numbers.

**Sparse secondary arena, not `slot = cell × M`.** Momentum states exist only
where a landing put one, so they live in a secondary region of the same
epoch-stamped arrays behind a per-cell list (`solver.ts slotFor`, mirrored
in `lib.rs slot_for`). The cell region `[0, n)` is the whole search with the
flag off — byte-identical, proved on the full suite and all 9 route-book
routes JS+wasm before/after.

**Wire format unchanged.** A chain node's `via` is the node before it;
`Move.expandRawPath` aims that stone far-side instead of duplicating it.

**Lip take-offs (the thing the stair actually needed).** prismarine-physics
(and vanilla) resolves the y axis, which sets `onGround`, before a tick's
horizontal move, so the flag lags a tick and a jump input on the first
airborne tick still fires — the late jump. Traced on the course's stair
(`bench/arena/.run/scratch/trace.ts`): grounded at 0.78 past centre, flagged
at 1.06, jumped there, landed. A running body therefore leaves from the
support's overhang limit `half + 0.3` plus a tick phase in `[0, 0.28)`,
while the planner credited the creep point 0.6. Under the flag a jump the
creep credit refuses is re-judged from the lip at the median phase
(`LIP_PHASE`), bounded by a lip corridor (`mfLip` = the running corridor
floored by the later, lower rising arc) shipped with the table; every edge
the plain model emits keeps its arc. This, not momentum, is what planned
the stair `(0,5)` (5 − 0.94 − 0.8 = 3.26 against 3.50) and the fence→fence
`(3,−3,5)`; the human's take-off speed at the stair was 0.26 — sub-sprint
after a 90° turn, exactly what the stair's own run gives.

**Chain turn loss at the executor's cadence.** The 0°→60° table below was
measured with the re-jump pressed on the *second* grounded tick; the
executor (and `J_CHAIN`'s own derivation) press on the first. Re-measured
at that cadence (`turnChain1.ts`): 0° 3.70, 30° 3.60, 45° 3.47, 60° 3.30
flat — a straight line, 0.8 blocks per unit cos (`CHAIN_TURN_LOSS`). The
45° gate at the full row over-credited turned chains by 0.23.

**Corrections to the ground truth quoted below.** The "0.7 conservative"
stair figure compared a from-centre measurement with a from-the-jump-point
row (0.11 apples to apples). The fence→fence "0.43 miss" came from feet
heights one block off (the analyzer reports the support cell; the stand
node is one above it) — with the right heights the brute force lands it, as
it lands `(0,5)` and `(−1,4)`. The finish beam `(1,5)` is *not* a course
block short: from the column's rear edge the run-back gives a consistent
phase and the beam is landed every run with `--risky`. It is, though, the
one hop outside the default margin, and a run-up block does not move it
(the run rows saturate; +0.02). The course now has one block at
`-111,263,135` extending the beam toward the column, so the last hop is a
`(1,4)` and the whole route plans and arrives at the default margin —
2/2 live, 17.7 s, first solves 21–72 ms, `--risky` no longer needed.

**Two things the live server taught (executor, all traced tick by tick with
`PF_EXEC_TRACE`, which now records the velocity and the server's own-entity
packets):**

- A landing that hurts loses its speed. Past three blocks of fall the server
  answers the damage with an `entity_velocity` packet that zeroes the
  client's horizontal motion on the landing tick. The rollout had approved
  the next take-off on the landing speed; the jump left from rest and fell
  short. The executor sits out that tick plus the ping after a damaging
  landing, and the planner gives such landings no momentum
  (`MOMENTUM_MAX_DROP` = 1.75 of rise: the 1.25 apex plus that is the
  3-block threshold).
- Ladders: caught mid-flight the body slides down 0.15 a tick and a catch at
  the lowest cell with the wall on the far side slid out before touching it
  — sneak holds the height and the wall press still climbs. A spiral
  transfer goes *over the top*: climb to the ladder's standable top edge,
  step from there (the recorded human does exactly this); and the
  wall-slide standoff must never steer toward air — it walked the bot off a
  3/16 top edge.

**Residual.** One run in seven stalled on a pot after the `(5,−2)` drop
happened not to hurt (fall right at 3.0): no settle, the body sprinted off
the 0.375-wide support with its landing speed. Pre-existing pot take-off
fragility; the fix is an executor rule for narrow supports, not a planner
one. The JS engine (the wasm fallback) also walks an interim *partial* path
off the 1-wide start platform when its first solve is slow — partials that
drop are a hazard independent of this feature.

**Cost.** Route book offline, flag on vs off: identical plans (two a little
cheaper: basic2 104.71→104.39, tunnel1 60.23→59.87), visited within 5%,
JS and wasm identical on every solve. `parkouradv1` with `--risky`: 2960
visited, 42 ms in wasm. Live, the whole book with `--momentum` at the
default margin (`bulba-wasm` vs `upstream`, 2026-08-26): 9/10 arrived (the
tenth is parkouradv1, whose finish needs `--risky`), 8 wins, 0 deaths, 0
replans on every route, mean 14.84 s against upstream's 18.28 s, first
solves 2–93 ms; faster than upstream on 8 of 9, climb2 within 0.7 s.

---

## The original plan (2026-08-25)

**Status:** design + implementation plan, not started. Hand this to a Claude
Code session (`fable`) to execute. Everything here is decided; the physics is
already measured (don't re-derive it). Work the milestones in order — each one
ends at a committable, differential-verified state.

## Why

The solver's node is `(x, y, z)`. A move's feasibility is a pure function of
geometry, so the planner has no idea whether the body arrives at a block
**sprinting** or standing still. Real parkour depends on incoming velocity: a
long jump needs a run-up, and a human carries sprint momentum *through turns*
and *across several hops*. The current run-length model
(`docs/ExtendedParkour.md`, `J_RUN`) approximates this by probing straight-
behind run-up cells only — it cannot express "arrives here already sprinting
toward +z."

Concretely, on the arena's `parkouradv1` the bots clear the fence/pot/head
cluster and stop at the `(0,5)` stair hop. That hop (and the two after it) is a
3-hop momentum build finished with a **lip take-off + ~90° turn** — the human
creeps to the stair's far edge and turns a +x sprint into a +z jump ("very low
tolerance, jump exactly from the corner"). The physics *can* do it (the
brute-forcer `tasHop.ts` lands it in dozens of takeoffs); the analytic planner
cannot, because A\* has no velocity in its state.

**Definition of done:** the planner emits the momentum-dependent jumps a human
makes, the executor flies them, and `parkouradv1` completes end-to-end at the
tight margin (or, honestly, every hop that is physically possible — the final
beam is a genuine 5-flat gap with no run-up and stays out of scope unless the
course gains a block). No regression: the route book stays 9/9 with unchanged
walls/visited/timing when the feature is **off**, and the wasm↔JS differential
stays bit-identical throughout.

## The design (decided)

### Node = `(cell, momentum)`

Add a small **momentum** dimension to the search node. Recommended first
encoding — deliberately coarse, because sprint speed saturates within ~1-2
blocks so "how much run-up" barely matters, only direction does:

```
momentum ∈ { NONE } ∪ { DIR_0 … DIR_7 }        // 9 states
```

- `NONE` — arrived walking / from a standstill / after a hard turn. The
  base moveset behaves exactly as today.
- `DIR_d` — arrived **sprint-saturated moving in direction d** (the 8
  cardinal+diagonal move directions, same order as `CARDINAL_*/DIAGONAL_*` in
  `moveGen.ts`).

Start with 9 states. Only refine to `(dir × level)` (reuse `RUN_LENGTHS`
granularity) at M4 if measurement shows the 2-value speed model rejects real
jumps — it probably won't, because saturation is fast.

*(Built: the exact primitive landing direction, landing-only, narrow
supports only — see the status section.)*

### Momentum transitions (in move generation)

Each generated neighbour carries an **outgoing** momentum, a pure function of
the move type and the node's incoming momentum:

| move | outgoing momentum |
| --- | --- |
| straight walk/step continuing direction `d` (prev move also `d`) | `DIR_d` (builds/keeps sprint) |
| walk that turns (incoming `DIR_e`, e≠d) | `NONE` if the turn > ~60°, else `DIR_d` (a gentle turn keeps sprint) |
| walk from `NONE` | `NONE` (one cell isn't enough to saturate — but a 2nd straight cell promotes to `DIR_d`; see note) |
| jump / parkour landing in direction `d` | `DIR_d` (the landing carries speed; this is the chain premise) |
| drop / climb / bubble / ladder-transfer | `NONE` (vertical or slow) |

Note on saturation: with the 9-state model, treat any straight motion of ≥1
cell that was *entered* with momentum, or the 2nd consecutive straight cell, as
`DIR_d`. The exact promotion rule is a one-integer state machine; keep it in a
single helper `momentumAfter(inMom, moveDir, moveKind)` mirrored JS↔Rust.

*(Built: only the "parkour landing" row, and only when the landing neither
hurts nor is a full block.)*

### Momentum-gated moves

Moves that need a run-up become available only from momentum-bearing nodes,
with the reach picked by the incoming momentum's alignment:

- The extended-parkour long jumps (`parkourExtTarget`) already compute
  `usable` from `J_RUN[runRow]`. Replace the straight-behind run-up probe with:
  **if incoming momentum is `DIR_d` and the jump direction is within the turn
  cone of `d`, use the momentum run-level** (saturated → the top `J_RUN` row),
  reduced by the measured turn factor for the angle (table below). Otherwise
  fall back to the current geometric run-up probe.
- This makes the momentum chain (`momentumChains`) a *special case* of the
  general rule and lets it fire across arbitrary approaches, not just 2-hop
  `META_PARKOUR → stepping-stone` sequences. Keep `momentumChains` working as a
  fallback until the general rule subsumes it, then delete it.

*(Built: the gated row is `J_CHAIN` less the turn loss, from the landing
state; `momentumChains` stays as the flag-off model.)*

### Turn factor (measured — do NOT re-derive)

Chain/momentum reach vs turn angle between incoming and outgoing direction,
landing-tick re-jump, flat bucket (subtract for other buckets proportionally):

| turn | cos | flat reach |
| --- | --- | --- |
| 0° | 1.00 | 3.90 |
| 30° | 0.87 | 3.81 |
| 45° | 0.71 | 3.70 |
| 60° | 0.50 | 3.56 |

Cutoff: reach must stay ≥ the row it is credited against. Beyond ~60° the
incoming momentum no longer helps — treat as `NONE`. `CHAIN_MIN_COS = 0.70`
(currently committed) is the 45° line and a safe start.

*(This table is the second-grounded-tick cadence; the executor's is one
tick earlier and loses 0.8 per unit cos — see the status section.)*

### Wire format: minimal-to-none

The **output path is still a sequence of cells.** The executor can re-derive
"this take-off needs momentum, don't decelerate into it" from the path shape
(the run-up cells precede the jump) plus the existing `parkour` flag. So:

- **Do not** add momentum to `RawPathNode` / the wasm result blob unless M3
  proves the executor needs an explicit hint. If it does, add one bit
  (`carriedMomentum`) exactly like the existing `chain`/`via` fields
  (`types.ts` `RawPathNode`, `move.ts` `Move`, `wasmSolver.ts readResult`,
  `lib.rs serialize_result`).
- The momentum dimension is **internal to the solver**. This is what keeps the
  scope tractable.

### Arena indexing (the core change)

Today the solver arenas (`g/parent/meta/stamp/closed/breaks` in `solver.ts`
`Arena`; the `Vec<>`s in `lib.rs SolverState`) are indexed by snapshot cell
index. Momentum multiplies the index:

```
slot = cellIndex * M + momentum          // M = 9 (or 1 when the flag is off)
```

- With the flag **off**, `M = 1`, `momentum = 0` always → byte-identical to
  today (this is milestone M0's proof).
- With it on, the arenas are `M×` larger. Grow-only + epoch-stamped, so only
  touched slots are written; memory grows but isn't zeroed per solve. At
  `M = 9` on a large snapshot this is tens of MB — acceptable. Cap per-cell
  states (keep only the best `g` per `(cell, momentum)`) — the epoch stamp
  already does this.
- The start node seeds `momentum = NONE`. `isEnd` ignores momentum (any
  momentum at the goal cell is a goal).
- The heap, decode, and chunk-touch logic key on `slot`; decode `cell` from
  `slot / M` for coordinates.

*(Built as a sparse secondary region instead — memory proportional to the
landings touched, not to `M ×` the snapshot.)*

### Executor

The plan already routes the run-up cells, so the body walks them. The change is
to **not stop or turn before a momentum jump**, and to take off from the lip:

- The sprint gate and corner-cut already keep sprint on straight runs. Add:
  when the next node is a `parkour` take-off that the plan reached with
  momentum, keep `forward+sprint` held through the approach and take off at the
  lip (reuse the creep-to-lip logic in `plugin.ts`, but running, not sneaking).
- The live `canSprintJump` rollout still gates every take-off against real
  physics, so a mis-estimated momentum jump **degrades to a refused take-off
  (stall → replan), never a fall.** That is the safety net that makes the whole
  feature low-risk to fly.

## Milestones (each ends committable + differential-green)

- **M0 — plumbing, flag off, prove identity.** Add `M`, the `slot` indexing,
  the `momentum` field (always 0), a new `MovementsConfig.allowParkourMomentum`
  (default false) + `lutFingerprint`/`toConfig` wiring. JS + Rust. Run the full
  suite: **the wasm↔JS differential and route book must be byte-identical to
  `d0b7257`.** This proves the encoding change is inert when off.

- **M1 — JS momentum, flag on.** `momentumAfter()` transitions, momentum-gated
  long-jump reach with the turn factor, in `moveGen.ts` only (Rust still M0).
  Offline verify with `bench/arena/.run/scratch/solveCourse.ts --roofreach`:
  the stair `(0,5)` and stair→fence `(-1,4)` become reachable. Verify the route
  book is unchanged with the flag **off**, and no worse **on** (`--from`/
  `--chain` spot-checks). Commit.

- **M2 — Rust mirror + differential.** Port `momentumAfter` and the gated moves
  to `lib.rs` op-for-op. Extend `test/wasm.test.ts genWorld` with
  momentum-bearing layouts (straight sprint lanes into gaps, turn-jumps) and
  assert identical paths/costs/visited. This is the correctness gate — do not
  proceed until green. Commit.

- **M3 — executor.** Momentum-aware take-off in `plugin.ts` (hold sprint into
  the jump, lip take-off). Arena-race `parkouradv1` (`--attach`, the user's
  `--keep` host on `127.0.0.1:25599`). Iterate against `tasHop.ts` /
  `analyzeRun2.ts` if a specific hop won't fly. Commit.

- **M4 — cost + tune.** Measure search cost with the flag on (route book visited
  counts must stay put; the course no-path search will grow — bound it with the
  per-cell momentum cap and, if needed, refine `NONE`-vs-directional promotion
  so flat terrain stays single-momentum). Confirm **never slower than upstream**
  on the route book. Refine to `(dir × level)` only if M1/M3 showed the 2-value
  speed model rejecting real jumps. Commit + update `docs/ExtendedParkour.md`.

## Risks & mitigations

- **State blowup → slower search.** Flag-gated (off = zero change); `NONE`
  default keeps flat terrain single-momentum; per-cell cap; measured at M4.
- **JS/Rust divergence.** M0 proves inertness; M2 re-pins with momentum fuzz
  worlds. Never merge a milestone with a red differential.
- **Executor can't reproduce planned momentum.** The rollout gate degrades a
  bad take-off to stall+replan, not a fall; the run-up path is already routed.
- **Heuristic admissibility.** Momentum-gated edges keep `cost ≥ octile` like
  every existing edge; the chain cost model is the template.

## Ground truth already measured (reuse, don't redo)

- Sprint saturates within ~1-2 blocks (hence the coarse speed model).
- Turn-reach table above (0-60°).
- Stair take-off with a one-lower run-up: flat-top **3.94**, low-step **4.23**
  (vs the flat-runway `J_RUN` row's 3.24 — the model is ~0.7 conservative for
  step-up take-offs; momentum-in-search is what fixes this generally).
- `parkouradv1` truth: stair `(0,5)` and stair→fence `(-1,4)` land in the
  brute-forcer; `fence→fence (3,-3,5)` is near the physics limit (0.43 miss);
  the finish beam `(1,5)` is a 5-flat gap with no run-up (needs a course block).
- Tools: `bench/arena/.run/scratch/solveCourse.ts` (`--chain`, `--from`,
  `--roofreach N`, `--risky`, `--wasm`, and now `--momentum`, `--mom dx,dz`),
  `tasHop.ts` (physics brute-force, `--chain`, `--runup`, `--start`),
  `trace.ts` (tick trace of one take-off), `execSim.ts` (the plugin's own
  rollout gates driving a body along a node path), `recordPlayer.ts` →
  `analyzeRun2.ts` (human capture, geometry-grounded hop detection), the
  derivations in `test/helpers/jumpEnvelope.ts` (`measureRunTakeoff`,
  `measureChainTakeoff`, `turnChain.ts`, `stairReach.ts`).

## Hard-won operational notes (this repo)

- **NEVER `git checkout`/`git restore` a tracked source file with uncommitted
  work** — it silently discards it (this cost a full recovery once). Commit
  before any git surgery. If a reconstruction is ever needed, the compiled
  `dist/esm/*.js` preserves comments (`removeComments: false`) and is an exact
  image of the source; the wasm↔JS differential proves a reconstruction correct.
- **Do not run subagents in this repo** (one reverted uncommitted work).
- The arena `--keep` host holds the world; `--attach` to it, don't boot a
  second server. It restarts often ("Server closed" kicks helper bots).
- `git commit` here: use `-F <file>` or a heredoc (PowerShell quoting splits
  `-m` with embedded quotes); the tree uses CRLF (expect LF→CRLF warnings).
- Build order after a change: `npm run build:wasm` (needs cargo + wasm32
  target) → `npx tsc --noEmit` → `npm run build` → `npx mocha`.
- `PF_EXEC_TRACE=<prefix> npm run arena:race -- ...` writes
  `<prefix>-<bot>.jsonl`: per tick position, ground flag, branch, controls,
  path length, next node, velocity, plus every own-entity
  `entity_velocity`/`sync_entity_position`/`position` packet. This is the
  tool that found both server-side surprises above.
