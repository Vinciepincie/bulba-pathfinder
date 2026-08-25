# Velocity-in-search: momentum-aware A* for frame-tight parkour

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
  `--roofreach N`, `--risky`, `--wasm`), `tasHop.ts` (physics brute-force,
  `--chain`, `--runup`, `--start`), `recordPlayer.ts` → `analyzeRun2.ts` (human
  capture, geometry-grounded hop detection), the derivations in
  `test/helpers/jumpEnvelope.ts` (`measureRunTakeoff`, `measureChainTakeoff`,
  `turnChain.ts`, `stairReach.ts`).

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
