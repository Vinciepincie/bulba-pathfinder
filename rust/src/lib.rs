//! wasm A* core for @bulba/pathfinder.
//!
//! An op-for-op port of src/solver.ts + src/moveGen.ts (which remain the
//! REFERENCE implementation — differential tests in test/wasm.test.ts pin
//! this port to identical statuses, costs and paths). Covers coordinate
//! goals — including GoalCompositeAny of coordinate goals — over the walk +
//! door + canDig moveset; raycast goals, other composites and exclusion-area
//! callbacks stay on the JS solver.
//!
//! All f64 arithmetic mirrors the JS evaluation order so IEEE-754 results
//! are bit-identical — tie-breaking, costs and paths must not diverge.
//!
//! Residency model (all state persists across solves in this instance):
//!   - snapshot buffers: uploaded once per (generation, patchCount) via
//!     snap_begin → write into snap_*_ptr() → snap_set_meta; the host skips
//!     the upload entirely when its key matches.
//!   - dig tables: dig_begin → dig_*_ptr(), re-uploaded per fingerprint.
//!   - solver arena: grow-only, epoch-stamped — no per-solve alloc/zeroing.
//! Interface: plain `extern "C"` exports, zero dependencies, no bindgen.

#![allow(clippy::too_many_arguments)]
// Single-threaded wasm: the static-mut state is only ever touched from the
// one worker thread that owns this instance.
#![allow(static_mut_refs)]

use core::f64::consts::SQRT_2;

// ── LutFlags (must match src/types.ts) ─────────────────────────────────────
const SAFE: u8 = 1;
const PHYSICAL: u8 = 2;
const LIQUID: u8 = 4;
const CLIMBABLE: u8 = 8;
const GATE: u8 = 16;
const DOOR_CLOSED: u8 = 32;
const DOOR_OPEN: u8 = 64;
const GATE_OPEN: u8 = 128;
const PASSABLE_WHEN_OPEN: u8 = DOOR_OPEN | GATE_OPEN;

// LutSpecial bytes (bubble columns, vines, slime — see types.ts).
const BUBBLE_UP: u8 = 1;
const BUBBLE_DOWN: u8 = 2;
const SPECIAL_VINE: u8 = 4;
const SPECIAL_SLIME: u8 = 8;
const SPECIAL_STAIR: u8 = 16;
// Bubble-only semantics must not fire on VINE/SLIME-marked cells.
const BUBBLE_MASK: u8 = BUBBLE_UP | BUBBLE_DOWN;

// DigFlags (must match src/types.ts)
const CAN_FALL: u8 = 1;
const CANT_BREAK: u8 = 2;

const META_PARKOUR: u8 = 1;
const META_USEONE: u8 = 2;
const META_BOUNCE: u8 = 4;
const META_CHAIN: u8 = 8;

const CARDINAL: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
const DIAGONAL: [(i32, i32); 4] = [(-1, -1), (-1, 1), (1, -1), (1, 1)];

// Climb-transfer directions + per-direction cost (mirror of moveGen.ts).
const TRANSFER: [(i32, i32); 8] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)];
const TRANSFER_COST: [f64; 8] = [1.5, 1.5, 1.5, 1.5, SQRT_2 + 0.5, SQRT_2 + 0.5, SQRT_2 + 0.5, SQRT_2 + 0.5];

// Slime-bounce landing columns (mirror of moveGen.ts BOUNCE_X/Z).
const BOUNCE_OFF: [(i32, i32); 12] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1), (-2, 0), (2, 0), (0, -2), (0, 2)];
const BOUNCE_RING1: usize = 8;

// Narrow-support credits and the slime-bounce envelope: pasted from
// src/parkourEnvelope.ts / shapes.ts (envelope.test.ts pins the JS side to
// the physics; wasm.test.ts pins this side to the JS side).
const CATCH_HALF: [f64; 4] = [0.5, 0.25, 0.1875, 0.125];
/// Run lengths the J_RUN rows were measured at (parkourEnvelope.ts RUN_LENGTHS).
const RUN_LENGTHS: [f64; 7] = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0, 3.0];

/// Mirror of parkourEnvelope.ts runRow: longest measured run ≤ run_length.
#[inline]
fn run_row(run_length: f64) -> usize {
    let mut row = 0;
    for i in 1..RUN_LENGTHS.len() {
        if RUN_LENGTHS[i] <= run_length {
            row = i;
        }
    }
    row
}
const LAND_HALF: f64 = 0.8;
const TAKEOFF_STAND: f64 = 0.6;
const TAKEOFF_NARROW_MARGIN: f64 = 0.28;
const LAND_NARROW_MARGIN: f64 = 0.3;
const BOUNCE_APEX: [f64; 9] = [
    0.0, 0.0, 1.299059403294517, 2.1016917686844407, 2.5491276710482,
    3.5285290916089673, 4.052437529400663, 4.592855224215983, 5.161928468689879,
];
const BOUNCE_MAX_DROP: i32 = 8;
const BOUNCE_MARGIN: f64 = 0.2;
const BOUNCE_MARGIN_FAR: f64 = 1.0;
/// Offset of the momentum-chain row (J_CHAIN, 10 buckets) inside ext_reach —
/// uploaded with the table (serializeParkourTable), never pasted here: a
/// pasted copy once sat 0.1 below the pinned JS row and diverged the engines.
const CHAIN_ROW: usize = 140;
const CHAIN_MIN_COS: f64 = 0.70;
const CHAIN_TAKEOFF_FRACTION: f64 = 0.5;
/// Chain turn loss per unit cos (parkourEnvelope.ts CHAIN_TURN_LOSS).
const CHAIN_TURN_LOSS: f64 = 0.8;
/// Median tick phase of a lip take-off (parkourEnvelope.ts LIP_PHASE = LIP_STRIDE / 2).
const LIP_PHASE: f64 = 0.28 / 2.0;
/// Deepest landing rise that still carries momentum (moveGen.ts MOMENTUM_MAX_DROP).
const MOMENTUM_MAX_DROP: f64 = 1.75;
const CHAIN_MIN_COS2: f64 = CHAIN_MIN_COS * CHAIN_MIN_COS;
const CHAIN_CAP: usize = 256;

// Momentum state (mirror of moveGen.ts momentumOf / momentumDx / momentumDz):
// 0 = none, else 1 + packed primitive landing direction, components in [-6, 6].
const MOM_NONE: u8 = 0;
const MOM_RANGE: i32 = 6;
const MOM_SPAN: i32 = 2 * MOM_RANGE + 1;

#[inline]
fn momentum_of(dx: i32, dz: i32) -> u8 {
    let mut a = dx.abs();
    let mut b = dz.abs();
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    (1 + (dx / a + MOM_RANGE) * MOM_SPAN + (dz / a + MOM_RANGE)) as u8
}

#[inline]
fn momentum_dx(m: u8) -> i32 {
    (m as i32 - 1) / MOM_SPAN - MOM_RANGE
}

#[inline]
fn momentum_dz(m: u8) -> i32 {
    (m as i32 - 1) % MOM_SPAN - MOM_RANGE
}

/// Secondary (momentum) slots reserved beyond the cell region per solve
/// (mirror of solver.ts momentumReserve).
#[inline]
fn momentum_reserve(n: usize) -> usize {
    if n >> 3 > 4096 { n >> 3 } else { 4096 }
}

/// Mirror of moveGen.ts chainAligned: cos(angle) ≥ CHAIN_MIN_COS, exact.
#[inline]
fn chain_aligned(abx: i32, abz: i32, ab2: i32, bcx: i32, bcz: i32) -> bool {
    let dot = abx * bcx + abz * bcz;
    if dot <= 0 {
        return false;
    }
    (dot * dot) as f64 >= CHAIN_MIN_COS2 * ab2 as f64 * (bcx * bcx + bcz * bcz) as f64
}

/// Mirror of moveGen.ts flightFrom: per-axis credited flight for offset
/// (a, b) from takeoff point `s` along the line — sqrt of a sum, never
/// hypot, so both engines round identically.
#[inline]
fn flight_from(a: f64, b: f64, dist: f64, s: f64, l_cred: f64) -> f64 {
    let dx = a - a / dist * s - l_cred;
    let dz = b - b / dist * s - l_cred;
    let px = if dx > 0.0 { dx } else { 0.0 };
    let pz = if dz > 0.0 { dz } else { 0.0 };
    (px * px + pz * pz).sqrt()
}

/// Extended-parkour offset, received from JS with the solve params — the
/// table (offsets, swept-corridor cells, reach envelope) is GENERATED once in
/// parkourTable.ts / parkourEnvelope.ts; this core never re-derives geometry.
#[derive(Clone, Copy, Default)]
struct ExtEntry {
    tx: i32,
    tz: i32,
    n_line: usize,
    n_cells: usize,
    /// Pair offset into ext_cells (index = cells_off * 2 + k * 2).
    cells_off: usize,
    run_x: i32,
    run_z: i32,
    dist: f64,
    cost: f64,
    /// Flight needed from the standing / running takeoff (per-axis corner
    /// credits, precomputed by parkourTable.ts) — compared against the
    /// uploaded J_STANDING / J_RUNNING rows per landing bucket.
    fn_stand: f64,
    fn_run: f64,
}

// ── goal ───────────────────────────────────────────────────────────────────
#[derive(Clone, Copy)]
struct GoalSpec {
    kind: u8, // 0 block, 1 near/follow, 2 xz, 3 nearxz, 4 y, 5 getToBlock
    gx: f64,
    gy: f64,
    gz: f64,
    range_sq: f64,
}

#[inline]
fn octile(dx: f64, dz: f64) -> f64 {
    let adx = dx.abs();
    let adz = dz.abs();
    (adx - adz).abs() + adx.min(adz) * SQRT_2
}

impl GoalSpec {
    #[inline]
    fn heuristic(&self, x: i32, y: i32, z: i32) -> f64 {
        let xf = x as f64;
        let yf = y as f64;
        let zf = z as f64;
        match self.kind {
            0 | 1 => octile(self.gx - xf, self.gz - zf) + (self.gy - yf).abs(),
            2 | 3 => octile(self.gx - xf, self.gz - zf),
            4 => (self.gy - yf).abs(),
            5 => {
                let dy = yf - self.gy;
                let dyk = if dy < 0.0 { dy + 1.0 } else { dy };
                octile(xf - self.gx, zf - self.gz) + dyk.abs()
            }
            _ => 0.0,
        }
    }

    #[inline]
    fn is_end(&self, x: i32, y: i32, z: i32) -> bool {
        let xf = x as f64;
        let yf = y as f64;
        let zf = z as f64;
        match self.kind {
            0 => xf == self.gx && yf == self.gy && zf == self.gz,
            1 => {
                let dx = self.gx - xf;
                let dy = self.gy - yf;
                let dz = self.gz - zf;
                dx * dx + dy * dy + dz * dz <= self.range_sq
            }
            2 => xf == self.gx && zf == self.gz,
            3 => {
                let dx = self.gx - xf;
                let dz = self.gz - zf;
                dx * dx + dz * dz <= self.range_sq
            }
            4 => yf == self.gy,
            5 => {
                let dy = yf - self.gy;
                let dyk = if dy < 0.0 { dy + 1.0 } else { dy };
                (xf - self.gx).abs() + dyk.abs() + (zf - self.gz).abs() == 1.0
            }
            _ => true,
        }
    }
}

/// GoalCompositeAny semantics for >1 spec: heuristic = min, isEnd = any —
/// same iteration order and fold as the JS class.
struct MultiGoal {
    specs: Vec<GoalSpec>,
}

impl MultiGoal {
    #[inline]
    fn heuristic(&self, x: i32, y: i32, z: i32) -> f64 {
        if self.specs.len() == 1 {
            return self.specs[0].heuristic(x, y, z);
        }
        let mut min = f64::MAX;
        for s in &self.specs {
            let h = s.heuristic(x, y, z);
            if h < min {
                min = h;
            }
        }
        min
    }

    #[inline]
    fn is_end(&self, x: i32, y: i32, z: i32) -> bool {
        if self.specs.len() == 1 {
            return self.specs[0].is_end(x, y, z);
        }
        for s in &self.specs {
            if s.is_end(x, y, z) {
                return true;
            }
        }
        false
    }
}

// ── config ────────────────────────────────────────────────────────────────
#[derive(Clone, Copy, Default)]
struct Config {
    allow_sprinting: bool,
    allow_parkour: bool,
    allow_parkour_extended: bool,
    can_open_doors: bool,
    door_mode: bool,
    infinite_liquid_dropdown: bool,
    can_dig: bool,
    dont_create_flow: bool,
    dont_mine_under_falling: bool,
    use_bubble: bool,
    allow_parkour_momentum: bool,
    max_drop_down: i32,
    liquid_cost: f64,
    entity_cost: f64,
    dig_cost: f64,
    bubble_cost: f64,
    /// ENVELOPE_SAFETY_MARGIN − parkourSafetyMargin (moveGen.ts marginCredit).
    margin_credit: f64,
}

// ── heap (mirror of src/heap.ts) ──────────────────────────────────────────
struct MinHeap {
    nodes: Vec<i32>,
    fs: Vec<f64>,
    n: usize,
}

impl MinHeap {
    fn new(cap: usize) -> Self {
        MinHeap { nodes: vec![0; cap], fs: vec![0.0; cap], n: 0 }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.n == 0
    }

    fn clear(&mut self) {
        self.n = 0;
    }

    fn push(&mut self, node: i32, f: f64) {
        if self.n == self.nodes.len() {
            let cap = self.nodes.len() * 2;
            self.nodes.resize(cap, 0);
            self.fs.resize(cap, 0.0);
        }
        let mut i = self.n;
        self.n += 1;
        while i > 0 {
            let parent = (i - 1) >> 1;
            if self.fs[parent] <= f {
                break;
            }
            self.nodes[i] = self.nodes[parent];
            self.fs[i] = self.fs[parent];
            i = parent;
        }
        self.nodes[i] = node;
        self.fs[i] = f;
    }

    fn pop(&mut self) -> i32 {
        let top = self.nodes[0];
        self.n -= 1;
        let n = self.n;
        if n > 0 {
            let node = self.nodes[n];
            let f = self.fs[n];
            let mut i = 0usize;
            let half = n >> 1;
            while i < half {
                let mut child = (i << 1) + 1;
                let right = child + 1;
                if right < n && self.fs[right] < self.fs[child] {
                    child = right;
                }
                if self.fs[child] >= f {
                    break;
                }
                self.nodes[i] = self.nodes[child];
                self.fs[i] = self.fs[child];
                i = child;
            }
            self.nodes[i] = node;
            self.fs[i] = f;
        }
        top
    }
}

// ── neighbor output (mirror of MoveGen out*) ──────────────────────────────
// Capacity, mirroring moveGen.ts outCapacity(): the base moveset can push 20
// (4 cardinals x forward/jumpUp/dropDown, 4 diagonals, down, up, two bubble
// rides), upstream's cardinal parkour up to 12 more when the table is off,
// and the extended table one per entry per applicable direction — 32*4 + 5*2
// + 5*2 = 148 today, so 168 in the worst case; extended parkour also adds
// up to 24 climb transfers and 12 slime-bounce landings per drop generator
// (5 of them), 252 in all, plus up to CHAIN_CAP momentum chains. In Rust an
// overflow is an index panic inside the wasm core (the solve traps and the
// host falls back to main-thread JS) rather than the silently dropped write
// the JS typed arrays give. 576 leaves the table room to grow.
const OUT_CAP: usize = 576;

struct NeighborOut {
    idx: [i32; OUT_CAP],
    x: [i32; OUT_CAP],
    y: [i32; OUT_CAP],
    z: [i32; OUT_CAP],
    cost: [f64; OUT_CAP],
    meta: [u8; OUT_CAP],
    breaks: [Option<Vec<i32>>; OUT_CAP],
    /// Slime stand cell of a META_BOUNCE neighbour, -1 otherwise.
    via: [i32; OUT_CAP],
    /// Momentum the neighbour arrives with (mirror of MoveGen.outMom).
    mom: [u8; OUT_CAP],
    count: usize,
}

// ── the persistent instance state ─────────────────────────────────────────
struct SolverState {
    // snapshot residency (uploaded via snap_begin/snap_set_meta)
    flags: Vec<u8>,
    heights: Vec<u8>,
    states: Vec<u16>,
    special: Vec<u8>,
    x0: i32,
    y0: i32,
    z0: i32,
    w: i32,
    h: i32,
    l: i32,
    world_min_y: i32,

    // dig residency (uploaded via dig_begin)
    dig_labor: Vec<f32>,
    dig_flags: Vec<u8>,

    // per-solve inputs
    entity_keys: Vec<i32>,
    entity_weights: Vec<i32>,
    cfg: Config,
    goal: MultiGoal,
    max_cost: f64,

    // extended-parkour table (per-solve upload; see ExtEntry)
    ext_entries: Vec<ExtEntry>,
    ext_cells: Vec<i32>,
    /// Per-corridor-cell min feet height: six blocks (standing, running,
    /// low-standing, low-running, lip, low-lip), each ext_mf_block long,
    /// indexed by block·ext_mf_block + cells_off + k.
    ext_mf: Vec<f64>,
    ext_mf_block: usize,
    /// J_RUN (7 run rows × 10 buckets), J_LOW_RUN, then J_CHAIN (CHAIN_ROW)
    /// — usable flight per run length / landing dy (parkourEnvelope.ts).
    ext_reach: [f64; 150],
    ext_diag_n: usize,
    ext_card_x_n: usize,
    ext_card_z_n: usize,

    // persistent epoch-stamped arena (grow-only, never zeroed per solve).
    // Slots [0, n_cells) are cells at momentum NONE; momentum states live in
    // the secondary region [n_cells, n_cells + sec_count) of the same
    // arrays (mirror of solver.ts Arena / slotFor).
    epoch: u32,
    g: Vec<f64>,
    parent: Vec<i32>,
    meta: Vec<u8>,
    stamp: Vec<u32>,
    closed: Vec<u8>,
    breaks: Vec<Option<Vec<i32>>>,
    /// Slime stand cell per META_BOUNCE node (-1 otherwise); stamped like g.
    vias: Vec<i32>,
    heap: MinHeap,
    n_cells: usize,
    momentum: bool,
    sec_count: usize,
    /// Per cell: first secondary entry, trusted only when it reads back the cell.
    mom_head: Vec<i32>,
    sec_cell: Vec<i32>,
    sec_dir: Vec<u8>,
    sec_next: Vec<i32>,

    out: NeighborOut,
    move_breaks: Vec<i32>,
    /// Set by slime_bounce for the push it is about to make.
    pending_via: i32,
    /// Set by parkour_ext_target for the landing it is about to push.
    pending_mom: u8,
    /// Momentum of the node being expanded (mirror of MoveGen.momIn).
    mom_in: u8,
    mom_dx: i32,
    mom_dz: i32,
    mom_d2: i32,

    // per-solve search state
    best_idx: i32,
    best_h: f64,
    visited: u32,
    open_count: u32,
    boundary_touched: bool,
    chunk_set: Vec<u64>, // open-addressed set of packed (cx,cz), 0 = empty slot
    chunk_list: Vec<(i32, i32)>,
    done: bool,
    done_status: i32,
    result: Vec<u8>,
}

impl SolverState {
    fn new() -> Self {
        SolverState {
            flags: Vec::new(),
            heights: Vec::new(),
            states: Vec::new(),
            special: Vec::new(),
            x0: 0,
            y0: 0,
            z0: 0,
            w: 0,
            h: 0,
            l: 0,
            world_min_y: 0,
            dig_labor: Vec::new(),
            dig_flags: Vec::new(),
            entity_keys: Vec::new(),
            entity_weights: Vec::new(),
            cfg: Config::default(),
            goal: MultiGoal { specs: Vec::new() },
            max_cost: -1.0,
            ext_entries: Vec::new(),
            ext_cells: Vec::new(),
            ext_mf: Vec::new(),
            ext_mf_block: 0,
            ext_reach: [0.0; 150],
            ext_diag_n: 0,
            ext_card_x_n: 0,
            ext_card_z_n: 0,
            epoch: 0,
            g: Vec::new(),
            parent: Vec::new(),
            meta: Vec::new(),
            stamp: Vec::new(),
            closed: Vec::new(),
            breaks: Vec::new(),
            vias: Vec::new(),
            heap: MinHeap::new(4096),
            n_cells: 0,
            momentum: false,
            sec_count: 0,
            mom_head: Vec::new(),
            sec_cell: Vec::new(),
            sec_dir: Vec::new(),
            sec_next: Vec::new(),
            out: NeighborOut {
                idx: [0; OUT_CAP],
                x: [0; OUT_CAP],
                y: [0; OUT_CAP],
                z: [0; OUT_CAP],
                cost: [0.0; OUT_CAP],
                meta: [0; OUT_CAP],
                breaks: [const { None }; OUT_CAP],
                via: [-1; OUT_CAP],
                mom: [0; OUT_CAP],
                count: 0,
            },
            move_breaks: Vec::new(),
            pending_via: -1,
            pending_mom: MOM_NONE,
            mom_in: MOM_NONE,
            mom_dx: 0,
            mom_dz: 0,
            mom_d2: 0,
            best_idx: -1,
            best_h: f64::INFINITY,
            visited: 0,
            open_count: 0,
            boundary_touched: false,
            chunk_set: vec![0; 4096],
            chunk_list: Vec::new(),
            done: false,
            done_status: 2,
            result: Vec::new(),
        }
    }

    /// Grow-only arena sizing + epoch bump (mirror of the JS arena pool).
    fn arena_prepare(&mut self, n: usize, momentum: bool) {
        let need = if momentum { n + momentum_reserve(n) } else { n };
        if self.g.len() < need {
            self.grow_slots(need);
        }
        if momentum && self.mom_head.len() < n {
            self.mom_head.resize(n, -1);
        }
        self.n_cells = n;
        self.momentum = momentum;
        self.sec_count = 0;
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            // wraparound: fresh stamps
            self.stamp.fill(0);
            self.epoch = 1;
        }
    }

    /// Slot arrays to at least `need`, contents (stamps included) preserved.
    fn grow_slots(&mut self, need: usize) {
        self.g.resize(need, 0.0);
        self.parent.resize(need, -1);
        self.meta.resize(need, 0);
        self.stamp.resize(need, 0);
        self.closed.resize(need, 0);
        self.breaks.resize_with(need, || None);
        self.vias.resize(need, -1);
    }

    #[inline]
    fn cell_of(&self, slot: i32) -> i32 {
        let s = slot as usize;
        if s < self.n_cells { slot } else { self.sec_cell[s - self.n_cells] }
    }

    #[inline]
    fn mom_of(&self, slot: i32) -> u8 {
        let s = slot as usize;
        if s < self.n_cells { MOM_NONE } else { self.sec_dir[s - self.n_cells] }
    }

    /// Mirror of solver.ts slotFor: the cell at NONE, else the matching
    /// secondary entry, created on first sight (per-cell list off mom_head).
    fn slot_for(&mut self, cell: i32, mom: u8) -> i32 {
        if mom == MOM_NONE {
            return cell;
        }
        let c = cell as usize;
        let mut k = self.mom_head[c];
        let mut first = -1;
        if k >= 0 && (k as usize) < self.sec_count && self.sec_cell[k as usize] == cell {
            first = k;
            loop {
                if self.sec_dir[k as usize] == mom {
                    return (self.n_cells + k as usize) as i32;
                }
                let nx = self.sec_next[k as usize];
                if nx < 0 {
                    break;
                }
                k = nx;
            }
        }
        let idx = self.sec_count;
        self.sec_count += 1;
        if self.n_cells + idx >= self.g.len() {
            let need = (self.n_cells + idx + 1) * 3 / 2;
            self.grow_slots(need);
        }
        if idx >= self.sec_cell.len() {
            let cap = (idx + 1) * 3 / 2;
            self.sec_cell.resize(cap, 0);
            self.sec_dir.resize(cap, 0);
            self.sec_next.resize(cap, -1);
        }
        self.sec_cell[idx] = cell;
        self.sec_dir[idx] = mom;
        self.sec_next[idx] = first;
        self.mom_head[c] = idx as i32;
        (self.n_cells + idx) as i32
    }
}

static mut STATE: Option<SolverState> = None;

unsafe fn state() -> &'static mut SolverState {
    if STATE.is_none() {
        STATE = Some(SolverState::new());
    }
    STATE.as_mut().unwrap()
}

// ── allocation exports (for per-solve param/entity blobs) ─────────────────
#[no_mangle]
pub extern "C" fn wasm_alloc(size: usize) -> *mut u8 {
    let mut v = Vec::<u8>::with_capacity(size.max(1));
    let ptr = v.as_mut_ptr();
    core::mem::forget(v);
    ptr
}

#[no_mangle]
pub unsafe extern "C" fn wasm_free(ptr: *mut u8, size: usize) {
    drop(Vec::from_raw_parts(ptr, 0, size.max(1)));
}

// ── snapshot residency exports ────────────────────────────────────────────
#[no_mangle]
pub unsafe extern "C" fn snap_begin(n: u32, states_len: u32, special_len: u32) {
    let st = state();
    st.flags.resize(n as usize, 0);
    st.heights.resize(n as usize, 0);
    st.states.resize(states_len as usize, 0);
    st.special.resize(special_len as usize, 0);
}

#[no_mangle]
pub unsafe extern "C" fn snap_flags_ptr() -> *mut u8 {
    state().flags.as_mut_ptr()
}

#[no_mangle]
pub unsafe extern "C" fn snap_heights_ptr() -> *mut u8 {
    state().heights.as_mut_ptr()
}

#[no_mangle]
pub unsafe extern "C" fn snap_states_ptr() -> *mut u8 {
    state().states.as_mut_ptr() as *mut u8
}

#[no_mangle]
pub unsafe extern "C" fn snap_special_ptr() -> *mut u8 {
    state().special.as_mut_ptr()
}

#[no_mangle]
pub unsafe extern "C" fn snap_set_meta(x0: i32, y0: i32, z0: i32, w: i32, h: i32, l: i32, world_min_y: i32) {
    let st = state();
    st.x0 = x0;
    st.y0 = y0;
    st.z0 = z0;
    st.w = w;
    st.h = h;
    st.l = l;
    st.world_min_y = world_min_y;
}

// ── dig residency exports ─────────────────────────────────────────────────
#[no_mangle]
pub unsafe extern "C" fn dig_begin(len: u32) {
    let st = state();
    st.dig_labor.resize(len as usize, 0.0);
    st.dig_flags.resize(len as usize, 0);
}

#[no_mangle]
pub unsafe extern "C" fn dig_labor_ptr() -> *mut u8 {
    state().dig_labor.as_mut_ptr() as *mut u8
}

#[no_mangle]
pub unsafe extern "C" fn dig_flags_ptr() -> *mut u8 {
    state().dig_flags.as_mut_ptr()
}

// ── solver impl ───────────────────────────────────────────────────────────
impl SolverState {
    #[inline]
    fn cell_index(&mut self, x: i32, y: i32, z: i32) -> i32 {
        let lx = x - self.x0;
        let ly = y - self.y0;
        let lz = z - self.z0;
        if lx < 0 || lx >= self.w || ly < 0 || ly >= self.h || lz < 0 || lz >= self.l {
            self.boundary_touched = true;
            return -1;
        }
        (ly * self.l + lz) * self.w + lx
    }

    #[inline]
    fn flags_at(&mut self, x: i32, y: i32, z: i32) -> u8 {
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            0
        } else {
            self.flags[idx as usize]
        }
    }

    /// The height byte is packed — bits 6–7 carry the top-catch class.
    #[inline]
    fn height_at(&mut self, x: i32, y: i32, z: i32) -> f64 {
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            y as f64
        } else {
            y as f64 + (self.heights[idx as usize] & 63) as f64 / 32.0
        }
    }

    /// topCatchClass of the cell's block (mirror of moveGen.ts catchAt).
    #[inline]
    fn catch_at(&mut self, x: i32, y: i32, z: i32) -> usize {
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            0
        } else {
            (self.heights[idx as usize] >> 6) as usize
        }
    }

    #[inline]
    fn is_safe(&self, f: u8) -> bool {
        (f & SAFE) != 0 || (self.cfg.door_mode && (f & PASSABLE_WHEN_OPEN) != 0)
    }

    /// Fence/wall/closed-gate class stand (mirror of moveGen.ts isTallStand).
    #[inline]
    fn is_tall_stand(&self, f: u8, h_byte: u8) -> bool {
        (f & (SAFE | PHYSICAL | LIQUID)) == 0 && (h_byte & 63) > 32
    }

    #[inline]
    fn tall_stand_at(&mut self, x: i32, y: i32, z: i32) -> bool {
        let idx = self.cell_index(x, y, z);
        idx >= 0 && self.is_tall_stand(self.flags[idx as usize], self.heights[idx as usize])
    }

    #[inline]
    fn ext_on(&self) -> bool {
        self.cfg.allow_parkour && self.cfg.allow_sprinting && self.cfg.allow_parkour_extended
    }

    /// Thin walk-in floor (carpet class): SAFE + PHYSICAL, not a climbable —
    /// feet stand IN this cell, never on top of it (mirrors JS isThinFloor).
    #[inline]
    fn is_thin_floor(&self, f: u8) -> bool {
        (f & (SAFE | PHYSICAL | CLIMBABLE)) == (SAFE | PHYSICAL)
    }

    /// LutSpecial byte of the cell (0 when no feature is on / out of box).
    /// Mirrors JS specialAt exactly, including the gate-before-probe order.
    #[inline]
    fn special_at(&mut self, x: i32, y: i32, z: i32) -> u8 {
        if self.special.is_empty() {
            return 0;
        }
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            0
        } else {
            self.special[idx as usize]
        }
    }

    #[inline]
    fn entities_at(&mut self, x: i32, y: i32, z: i32) -> f64 {
        if self.entity_keys.is_empty() {
            return 0.0;
        }
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            return 0.0;
        }
        // Entity lists are tiny (a handful of mobs); linear scan.
        for (i, k) in self.entity_keys.iter().enumerate() {
            if *k == idx {
                return self.entity_weights[i] as f64;
            }
        }
        0.0
    }

    fn safe_to_break(&mut self, x: i32, y: i32, z: i32, idx: i32) -> bool {
        if self.cfg.dont_create_flow {
            if (self.flags_at(x, y + 1, z) & LIQUID) != 0 {
                return false;
            }
            if (self.flags_at(x - 1, y, z) & LIQUID) != 0 {
                return false;
            }
            if (self.flags_at(x + 1, y, z) & LIQUID) != 0 {
                return false;
            }
            if (self.flags_at(x, y, z - 1) & LIQUID) != 0 {
                return false;
            }
            if (self.flags_at(x, y, z + 1) & LIQUID) != 0 {
                return false;
            }
        }
        if self.cfg.dont_mine_under_falling {
            let above_idx = self.cell_index(x, y + 1, z);
            let above_state = if above_idx >= 0 { self.states[above_idx as usize] } else { 0 };
            if (self.dig_flags[above_state as usize] & CAN_FALL) != 0 || self.entities_at(x, y + 1, z) > 0.0 {
                return false;
            }
        }
        let state_id = self.states[idx as usize];
        (self.dig_flags[state_id as usize] & CANT_BREAK) == 0
    }

    fn safe_or_break(&mut self, x: i32, y: i32, z: i32) -> f64 {
        // exclusion areas force the JS path — contribution here is 0.
        let mut cost = self.entities_at(x, y, z) * self.cfg.entity_cost;
        let idx = self.cell_index(x, y, z);
        let f = if idx < 0 { 0 } else { self.flags[idx as usize] };
        if self.is_safe(f) {
            return cost;
        }
        if !self.cfg.can_dig || idx < 0 {
            return 100.0;
        }
        if !self.safe_to_break(x, y, z, idx) {
            return 100.0;
        }
        self.move_breaks.push(idx);
        if (f & PHYSICAL) != 0 {
            cost += self.entities_at(x, y + 1, z) * self.cfg.entity_cost;
        }
        cost += self.dig_labor[self.states[idx as usize] as usize] as f64 * self.cfg.dig_cost;
        cost
    }

    #[inline]
    fn begin_move(&mut self) {
        if !self.move_breaks.is_empty() {
            self.move_breaks = Vec::new();
        }
    }

    fn push_out(&mut self, x: i32, y: i32, z: i32, cost: f64, meta: u8) {
        let mom = self.pending_mom;
        self.pending_mom = MOM_NONE;
        let idx = self.cell_index(x, y, z);
        if idx < 0 {
            return;
        }
        let i = self.out.count;
        self.out.count += 1;
        self.out.idx[i] = idx;
        self.out.x[i] = x;
        self.out.y[i] = y;
        self.out.z[i] = z;
        self.out.cost[i] = cost;
        self.out.meta[i] = meta;
        self.out.via[i] = self.pending_via;
        self.pending_via = -1;
        self.out.mom[i] = mom;
        self.out.breaks[i] = if self.move_breaks.is_empty() {
            None
        } else {
            Some(core::mem::take(&mut self.move_breaks))
        };
    }

    fn move_forward(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) {
        self.begin_move();
        let f_c = self.flags_at(x + dx, y, z + dz);
        let f_d = self.flags_at(x + dx, y - 1, z + dz);

        let mut cost = 1.0;

        // Improvement (useBubbleColumns): a bubble cell floats you like water.
        if (f_d & PHYSICAL) == 0 && (f_c & LIQUID) == 0 && (self.special_at(x + dx, y, z + dz) & BUBBLE_MASK) == 0 {
            if !self.ext_on() {
                return;
            }
            if self.tall_stand_at(x + dx, y - 1, z + dz) {
                // Walk onto a fence/wall top sunk one below (mirror of moveGen.ts).
                if self.height_at(x + dx, y - 1, z + dz) - self.height_at(x, y - 1, z) > 0.6 {
                    return;
                }
                let f_up = self.flags_at(x + dx, y + 2, z + dz);
                if !self.is_safe(f_up) {
                    return;
                }
            } else if (f_c & CLIMBABLE) == 0 || !self.is_safe(f_c) || !self.climb_usable(x + dx, y, z + dz) {
                return;
            }
            // else: step into a free-hanging ladder/vine cell — it catches.
        } else if self.is_thin_floor(f_d) {
            // A thin floor one below is a step DOWN into that cell — move_drop_down
            // produces the correct node; a same-level node here would float.
            return;
        }

        let activatable = (f_c & GATE) != 0 || (self.cfg.door_mode && (f_c & DOOR_CLOSED) != 0);
        let through_closed_door = self.cfg.can_open_doors && self.cfg.door_mode && (f_c & DOOR_CLOSED) != 0;

        let f_b = self.flags_at(x + dx, y + 1, z + dz);
        if through_closed_door && (f_b & DOOR_CLOSED) != 0 {
            cost += self.entities_at(x + dx, y + 1, z + dz) * self.cfg.entity_cost;
        } else {
            cost += self.safe_or_break(x + dx, y + 1, z + dz);
        }
        if cost > 100.0 {
            return;
        }

        let mut meta = 0u8;
        if self.cfg.can_open_doors && activatable {
            meta = META_USEONE;
        } else {
            cost += self.safe_or_break(x + dx, y, z + dz);
            if cost > 100.0 {
                return;
            }
        }

        if (self.flags_at(x, y, z) & LIQUID) != 0 {
            cost += self.cfg.liquid_cost;
        }

        self.push_out(x + dx, y, z + dz, cost, meta);
    }

    fn move_jump_up(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) {
        self.begin_move();
        let f_a = self.flags_at(x, y + 2, z);
        let f_h = self.flags_at(x + dx, y + 2, z + dz);
        let f_b = self.flags_at(x + dx, y + 1, z + dz);
        let f_c = self.flags_at(x + dx, y, z + dz);

        let mut cost = 2.0;

        if (f_a & PHYSICAL) != 0 && self.entities_at(x, y + 3, z) > 0.0 {
            return;
        }
        if (f_h & PHYSICAL) != 0 && self.entities_at(x + dx, y + 3, z + dz) > 0.0 {
            return;
        }
        if (f_b & PHYSICAL) != 0 && (f_h & PHYSICAL) == 0 && (f_c & PHYSICAL) == 0
            && self.entities_at(x + dx, y + 2, z + dz) > 0.0
        {
            return;
        }

        if (f_c & PHYSICAL) == 0 {
            // Extended: a fence/wall top is a stand too (mirror of moveGen.ts).
            if !self.ext_on() || !self.tall_stand_at(x + dx, y, z + dz) {
                return;
            }
            let f3 = self.flags_at(x + dx, y + 3, z + dz);
            if !self.is_safe(f3) {
                return;
            }
        } else if self.is_thin_floor(f_c) {
            // A thin floor at the target feet cell is same-level ground (move_forward
            // walks into it) — "jumping onto" it would land in the air above.
            return;
        } else if self.ext_on() && (f_c & CLIMBABLE) != 0 {
            // No jumps onto a ladder's top edge (mirror of moveGen.ts).
            return;
        }

        let h_c = self.height_at(x + dx, y, z + dz);
        let h_0 = self.height_at(x, y - 1, z);
        if h_c - h_0 > 1.2 {
            return;
        }

        cost += self.safe_or_break(x, y + 2, z);
        if cost > 100.0 {
            return;
        }
        cost += self.safe_or_break(x + dx, y + 2, z + dz);
        if cost > 100.0 {
            return;
        }
        cost += self.safe_or_break(x + dx, y + 1, z + dz);
        if cost > 100.0 {
            return;
        }

        self.push_out(x + dx, y + 1, z + dz, cost, 0);
    }

    /// Returns landing stand-cell y, or i32::MIN when none.
    fn find_landing(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) -> i32 {
        let lx = x + dx;
        let lz = z + dz;
        let mut ly = y - 2;
        while ly > self.world_min_y {
            let idx = self.cell_index(lx, ly, lz);
            if idx < 0 {
                return i32::MIN;
            }
            let f = self.flags[idx as usize];
            if (f & LIQUID) != 0 && self.is_safe(f) {
                return ly;
            }
            // Improvement (useBubbleColumns): a column catches the fall like water.
            if !self.special.is_empty() && (self.special[idx as usize] & BUBBLE_MASK) != 0 {
                return ly;
            }
            // Extended: a ladder/vine below catches the drop.
            if self.ext_on() && (f & CLIMBABLE) != 0 && self.is_safe(f) && self.climb_usable(lx, ly, lz) {
                if y - ly <= self.cfg.max_drop_down {
                    return ly;
                }
                return i32::MIN;
            }
            // Thin floor (carpet class): feet land IN the cell, not on top.
            if self.is_thin_floor(f) {
                if y - ly <= self.cfg.max_drop_down {
                    return ly;
                }
                return i32::MIN;
            }
            if (f & PHYSICAL) != 0 {
                if y - ly <= self.cfg.max_drop_down {
                    return ly + 1;
                }
                return i32::MIN;
            }
            // Extended: a fence/wall top is a stand (feet 0.5 into the cell above).
            if self.ext_on() && self.is_tall_stand(f, self.heights[idx as usize]) {
                let f3 = self.flags_at(lx, ly + 3, lz);
                if y - ly <= self.cfg.max_drop_down && self.is_safe(f3) {
                    return ly + 1;
                }
                return i32::MIN;
            }
            if !self.is_safe(f) {
                return i32::MIN;
            }
            ly -= 1;
        }
        i32::MIN
    }

    fn move_drop_down(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) {
        self.begin_move();
        let mut cost = 1.0;

        let land_y = self.find_landing(x, y, z, dx, dz);
        if land_y == i32::MIN {
            return;
        }
        if !self.cfg.infinite_liquid_dropdown && (y - land_y) > self.cfg.max_drop_down {
            return;
        }

        cost += self.safe_or_break(x + dx, y + 1, z + dz);
        if cost > 100.0 {
            return;
        }
        cost += self.safe_or_break(x + dx, y, z + dz);
        if cost > 100.0 {
            return;
        }
        cost += self.safe_or_break(x + dx, y - 1, z + dz);
        if cost > 100.0 {
            return;
        }

        if (self.flags_at(x + dx, y, z + dz) & LIQUID) != 0 {
            return;
        }

        cost += self.entities_at(x + dx, land_y, z + dz) * self.cfg.entity_cost;

        self.push_out(x + dx, land_y, z + dz, cost, 0);
        if self.ext_on() && !self.special.is_empty() {
            self.slime_bounce(x, y, z, x + dx, land_y, z + dz);
        }
    }

    fn move_down(&mut self, x: i32, y: i32, z: i32) {
        self.begin_move();
        // Can't descend against an up-column's push (ride edges handle columns).
        if self.special_at(x, y, z) == BUBBLE_UP {
            return;
        }
        let mut cost = 1.0;

        let land_y = self.find_landing(x, y, z, 0, 0);
        if land_y == i32::MIN {
            return;
        }

        cost += self.safe_or_break(x, y - 1, z);
        if cost > 100.0 {
            return;
        }

        if (self.flags_at(x, y, z) & LIQUID) != 0 {
            return;
        }

        cost += self.entities_at(x, land_y, z) * self.cfg.entity_cost;

        self.push_out(x, land_y, z, cost, 0);
        if self.ext_on() && !self.special.is_empty() {
            self.slime_bounce(x, y, z, x, land_y, z);
        }
    }

    fn move_up(&mut self, x: i32, y: i32, z: i32) {
        self.begin_move();
        let f1 = self.flags_at(x, y, z);
        if (f1 & LIQUID) != 0 {
            return;
        }
        if self.entities_at(x, y, z) > 0.0 {
            return;
        }

        let mut cost = 1.0;
        cost += self.safe_or_break(x, y + 2, z);
        if cost > 100.0 {
            return;
        }

        if (f1 & CLIMBABLE) == 0 {
            return;
        }

        // Vines climb only with an adjacent solid block to press against.
        if self.special_at(x, y, z) == SPECIAL_VINE
            && (self.flags_at(x + 1, y, z) & PHYSICAL) == 0
            && (self.flags_at(x - 1, y, z) & PHYSICAL) == 0
            && (self.flags_at(x, y, z + 1) & PHYSICAL) == 0
            && (self.flags_at(x, y, z - 1) & PHYSICAL) == 0
        {
            return;
        }

        self.push_out(x, y + 1, z, cost, 0);
    }

    fn move_diagonal(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) {
        self.begin_move();
        let mut cost = SQRT_2;

        let f_c = self.flags_at(x + dx, y, z + dz);
        // Extended: never diagonally onto a ladder's top edge (mirror of moveGen.ts).
        if self.ext_on() && (f_c & (PHYSICAL | CLIMBABLE)) == (PHYSICAL | CLIMBABLE) {
            return;
        }
        // A thin floor at the target feet cell is same-level ground, not a +1 hop.
        let yo: i32 = if (f_c & PHYSICAL) != 0 && !self.is_thin_floor(f_c) { 1 } else { 0 };
        let h_0 = self.height_at(x, y - 1, z);

        let mut cost1 = 0.0;
        cost1 += self.safe_or_break(x, y + yo + 1, z + dz);
        cost1 += self.safe_or_break(x, y + yo, z + dz);
        let h_d1 = self.height_at(x, y + yo - 1, z + dz);
        if h_d1 - h_0 > 1.2 {
            cost1 += self.safe_or_break(x, y + yo - 1, z + dz);
        }
        let breaks1 = core::mem::take(&mut self.move_breaks);

        let mut cost2 = 0.0;
        cost2 += self.safe_or_break(x + dx, y + yo + 1, z);
        cost2 += self.safe_or_break(x + dx, y + yo, z);
        let h_d2 = self.height_at(x + dx, y + yo - 1, z);
        if h_d2 - h_0 > 1.2 {
            cost2 += self.safe_or_break(x + dx, y + yo - 1, z);
        }

        if cost1 < cost2 {
            cost += cost1;
            self.move_breaks = breaks1;
        } else {
            cost += cost2;
            // move_breaks already holds corner 2's digs
        }
        if cost > 100.0 {
            return;
        }

        cost += self.safe_or_break(x + dx, y + yo, z + dz);
        if cost > 100.0 {
            return;
        }
        cost += self.safe_or_break(x + dx, y + yo + 1, z + dz);
        if cost > 100.0 {
            return;
        }

        if (self.flags_at(x, y, z) & LIQUID) != 0 {
            cost += self.cfg.liquid_cost;
        }

        let f_d = self.flags_at(x + dx, y - 1, z + dz);
        if yo == 1 {
            let h_c = self.height_at(x + dx, y, z + dz);
            if h_c - h_0 > 1.2 {
                return;
            }
            cost += self.safe_or_break(x, y + 2, z);
            if cost > 100.0 {
                return;
            }
            cost += 1.0;
            self.push_out(x + dx, y + 1, z + dz, cost, 0);
        } else if ((f_d & PHYSICAL) != 0 && !self.is_thin_floor(f_d))
            || (f_c & LIQUID) != 0
            || self.is_thin_floor(f_c)
            || (self.special_at(x + dx, y, z + dz) & BUBBLE_MASK) != 0
        {
            self.push_out(x + dx, y, z + dz, cost, 0);
        } else if (self.flags_at(x + dx, y - 2, z + dz) & PHYSICAL) != 0 || (f_d & LIQUID) != 0 {
            if !self.is_safe(f_d) {
                return;
            }
            cost += self.entities_at(x + dx, y - 1, z + dz) * self.cfg.entity_cost;
            self.push_out(x + dx, y - 1, z + dz, cost, 0);
        }
    }

    fn move_parkour_forward(&mut self, x: i32, y: i32, z: i32, dx: i32, dz: i32) {
        self.begin_move();
        let h_0 = self.height_at(x, y - 1, z);
        let f1 = self.flags_at(x + dx, y - 1, z + dz);
        let h_1 = self.height_at(x + dx, y - 1, z + dz);
        let fwd0 = self.flags_at(x + dx, y, z + dz);
        let fwd1 = self.flags_at(x + dx, y + 1, z + dz);
        if ((f1 & PHYSICAL) != 0 && h_1 >= h_0) || !self.is_safe(fwd0) || !self.is_safe(fwd1) {
            return;
        }
        if (self.flags_at(x, y, z) & LIQUID) != 0 {
            return;
        }
        if (self.special_at(x, y, z) & BUBBLE_MASK) != 0 {
            return; // cant jump while floating in a column
        }

        let mut cost = 1.0;
        cost += self.entities_at(x + dx, y, z + dz) * self.cfg.entity_cost;

        let c0 = self.flags_at(x, y + 2, z);
        let c1 = self.flags_at(x + dx, y + 2, z + dz);
        let mut ceiling_clear = self.is_safe(c0) && self.is_safe(c1);
        let below2 = self.flags_at(x + dx, y - 2, z + dz);
        let mut floor_cleared = (below2 & PHYSICAL) == 0;
        let max_d = if self.cfg.allow_sprinting { 4 } else { 2 };

        let mut d = 2;
        while d <= max_d {
            let dxx = dx * d;
            let dzz = dz * d;
            let f_a = self.flags_at(x + dxx, y + 2, z + dzz);
            let f_b = self.flags_at(x + dxx, y + 1, z + dzz);
            let f_cd = self.flags_at(x + dxx, y, z + dzz);
            let f_dd = self.flags_at(x + dxx, y - 1, z + dzz);

            if self.is_safe(f_cd) {
                cost += self.entities_at(x + dxx, y, z + dzz) * self.cfg.entity_cost;
            }

            if ceiling_clear && self.is_safe(f_b) && self.is_safe(f_cd) && (f_dd & PHYSICAL) != 0 {
                self.push_out(x + dxx, y, z + dzz, cost, META_PARKOUR);
                break;
            } else if ceiling_clear && self.is_safe(f_b) && (f_cd & PHYSICAL) != 0 {
                if self.is_safe(f_a) && d != 4 {
                    let h_c = self.height_at(x + dxx, y, z + dzz);
                    if h_c - h_0 > 1.2 {
                        break;
                    }
                    cost += self.entities_at(x + dxx, y + 1, z + dzz) * self.cfg.entity_cost;
                    self.push_out(x + dxx, y + 1, z + dzz, cost, META_PARKOUR);
                    break;
                }
            } else if (ceiling_clear || d == 2)
                && self.is_safe(f_b)
                && self.is_safe(f_cd)
                && self.is_safe(f_dd)
                && floor_cleared
            {
                let f_e = self.flags_at(x + dxx, y - 2, z + dzz);
                if (f_e & PHYSICAL) != 0 {
                    cost += self.entities_at(x + dxx, y - 1, z + dzz) * self.cfg.entity_cost;
                    self.push_out(x + dxx, y - 1, z + dzz, cost, META_PARKOUR);
                }
                floor_cleared = floor_cleared && (f_e & PHYSICAL) == 0;
            } else if !self.is_safe(f_b) || !self.is_safe(f_cd) {
                break;
            }

            ceiling_clear = ceiling_clear && self.is_safe(f_a);
            d += 1;
        }
    }

    /// Mirror of moveGen.ts momentumChains: re-jump edges through the
    /// parkour landings this expansion just produced (stepping stones only).
    fn momentum_chains(&mut self, x: i32, y: i32, z: i32) {
        let n_out = self.out.count;
        for i in 0..n_out {
            if self.out.meta[i] != META_PARKOUR {
                continue;
            }
            let bx = self.out.x[i];
            let by = self.out.y[i];
            let bz = self.out.z[i];
            let f_b = self.flags_at(bx, by, bz);
            if (f_b & (LIQUID | CLIMBABLE)) != 0 || self.is_thin_floor(f_b) || (self.special_at(bx, by, bz) & BUBBLE_MASK) != 0 {
                continue;
            }
            let idx_s = self.cell_index(bx, by - 1, bz);
            if idx_s < 0 {
                continue;
            }
            let f_s = self.flags[idx_s as usize];
            if (f_s & PHYSICAL) == 0 && !self.is_tall_stand(f_s, self.heights[idx_s as usize]) {
                continue;
            }
            let h_b = self.height_at(bx, by - 1, bz);
            let mut runnable = false;
            for d in 0..4 {
                let (ddx, ddz) = CARDINAL[d];
                let nx = bx + ddx;
                let nz = bz + ddz;
                let fn0 = self.flags_at(nx, by - 1, nz);
                if (fn0 & PHYSICAL) != 0 && self.catch_at(nx, by - 1, nz) == 0 {
                    let a0 = self.flags_at(nx, by, nz);
                    let a1 = self.flags_at(nx, by + 1, nz);
                    if self.is_safe(a0) && self.is_safe(a1) {
                        let h_n = self.height_at(nx, by - 1, nz);
                        if h_n - h_b <= 0.2 && h_b - h_n <= 0.2 {
                            runnable = true;
                            break;
                        }
                    }
                }
            }
            if runnable {
                continue;
            }
            let lid = self.flags_at(bx, by + 2, bz);
            if !self.is_safe(lid) {
                continue;
            }
            let abx = bx - x;
            let abz = bz - z;
            let ab2 = abx * abx + abz * abz;
            let via_idx = self.out.idx[i];
            let base_cost = self.out.cost[i];
            for q in 0..4 {
                let (sx, sz) = DIAGONAL[q];
                for k in 0..self.ext_diag_n {
                    let e = self.ext_entries[k];
                    if !chain_aligned(abx, abz, ab2, e.tx * sx, e.tz * sz) {
                        continue;
                    }
                    self.parkour_ext_target(bx, by, bz, sx, sz, h_b, false, e, via_idx, base_cost);
                }
            }
            for d in 0..4 {
                let (dx, dz) = CARDINAL[d];
                if dx != 0 {
                    for k in 0..self.ext_card_x_n {
                        let e = self.ext_entries[self.ext_diag_n + k];
                        if !chain_aligned(abx, abz, ab2, e.tx * dx, 0) {
                            continue;
                        }
                        self.parkour_ext_target(bx, by, bz, dx, 1, h_b, false, e, via_idx, base_cost);
                    }
                } else {
                    for k in 0..self.ext_card_z_n {
                        let e = self.ext_entries[self.ext_diag_n + self.ext_card_x_n + k];
                        if !chain_aligned(abx, abz, ab2, 0, e.tz * dz) {
                            continue;
                        }
                        self.parkour_ext_target(bx, by, bz, 1, dz, h_b, false, e, via_idx, base_cost);
                    }
                }
            }
            if self.out.count - n_out >= CHAIN_CAP {
                return;
            }
        }
    }

    /// Improvement (allowParkourExtended), mirror of moveGen.ts
    /// parkourExtTarget — rules in docs/ExtendedParkour.md. `chain_via ≥ 0`
    /// is the momentum-chain re-jump variant (see momentum_chains).
    fn parkour_ext_target(&mut self, x: i32, y: i32, z: i32, sx: i32, sz: i32, h_0: f64, low_takeoff: bool, e: ExtEntry, chain_via: i32, chain_base: f64) {
        self.begin_move();
        let cbase = e.cells_off * 2;
        // Flat-ground fast-out: walkable floor on the first flight-line cell.
        let flx = x + self.ext_cells[cbase] * sx;
        let flz = z + self.ext_cells[cbase + 1] * sz;
        if (self.flags_at(flx, y - 1, flz) & PHYSICAL) != 0 && self.height_at(flx, y - 1, flz) >= h_0 {
            return;
        }

        let tx = x + e.tx * sx;
        let tz = z + e.tz * sz;
        let idx_t = self.cell_index(tx, y, tz);
        let f_t = if idx_t < 0 { 0 } else { self.flags[idx_t as usize] };

        let mut node_y: i32;
        let mut cost = e.cost;
        // Top-catch class of the landing support; -1 = enters a cell (full credit).
        let mut land_catch: i32 = -1;
        // Cell whose top the feet land on (support landings): the reach
        // bucket is the real rise to it. i32::MIN = a catch (node delta).
        let mut sup_y: i32 = i32::MIN;
        if (f_t & CLIMBABLE) != 0 && self.is_safe(f_t) && self.climb_usable(tx, y, tz) {
            // Grab a ladder/vine at flight level — before PHYSICAL, because
            // ladders classify as physical too; the catch is the lowest
            // contiguous ladder cell up to two below (see the JS reference).
            let t1 = self.flags_at(tx, y + 1, tz);
            if !self.is_safe(t1) {
                return;
            }
            node_y = y;
            let mut ly = y - 1;
            while ly >= y - 2 {
                let f_l = self.flags_at(tx, ly, tz);
                if (f_l & CLIMBABLE) == 0 || !self.is_safe(f_l) || !self.climb_usable(tx, ly, tz) {
                    break;
                }
                node_y = ly;
                cost += 1.0;
                ly -= 1;
            }
        } else if self.is_thin_floor(f_t) {
            // Thin floor at flight level (carpeted landing): a same-level
            // jump — feet land IN the cell, like the air-branch landing.
            let t1 = self.flags_at(tx, y + 1, tz);
            if !self.is_safe(t1) {
                return;
            }
            node_y = y;
        } else if (f_t & PHYSICAL) != 0 {
            // Up variant: the flight-level cell is the landing block.
            if self.height_at(tx, y, tz) - h_0 > 1.2 {
                return; // too high to jump
            }
            let t1 = self.flags_at(tx, y + 1, tz);
            let t2 = self.flags_at(tx, y + 2, tz);
            if !self.is_safe(t1) || !self.is_safe(t2) {
                return;
            }
            node_y = y + 1;
            cost += 1.0;
            land_catch = (self.heights[idx_t as usize] >> 6) as i32;
            sup_y = y;
        } else if idx_t >= 0 && self.is_tall_stand(f_t, self.heights[idx_t as usize]) {
            // Fence/wall top at flight level: up landing onto its 1.5 top;
            // a pot on the post puts the node above the pot (mirror of moveGen.ts).
            if self.height_at(tx, y, tz) - h_0 > 1.2 {
                return;
            }
            let t1 = self.flags_at(tx, y + 1, tz);
            if self.is_safe(t1) {
                let t2 = self.flags_at(tx, y + 2, tz);
                let t3 = self.flags_at(tx, y + 3, tz);
                if !self.is_safe(t2) || !self.is_safe(t3) {
                    return;
                }
                node_y = y + 1;
                cost += 1.0;
            } else {
                let idx1 = self.cell_index(tx, y + 1, tz);
                if idx1 < 0 || (t1 & PHYSICAL) == 0 || (self.heights[idx1 as usize] & 63) > 16 {
                    return;
                }
                let t2 = self.flags_at(tx, y + 2, tz);
                let t3 = self.flags_at(tx, y + 3, tz);
                if !self.is_safe(t2) || !self.is_safe(t3) {
                    return;
                }
                node_y = y + 2;
                cost += 2.0;
            }
            land_catch = (self.heights[idx_t as usize] >> 6) as i32;
            sup_y = y;
        } else {
            let t1 = self.flags_at(tx, y + 1, tz);
            if !self.is_safe(f_t) || !self.is_safe(t1) {
                return;
            }
            match self.find_ext_landing(tx, y, tz) {
                Some(ly) => node_y = ly,
                None => return,
            }
            // Landed ON a support rather than IN a catching cell?
            let f_n = self.flags_at(tx, node_y, tz);
            if (f_n & (LIQUID | CLIMBABLE)) == 0
                && !self.is_thin_floor(f_n)
                && (self.special_at(tx, node_y, tz) & BUBBLE_MASK) == 0
            {
                land_catch = self.catch_at(tx, node_y - 1, tz) as i32;
                sup_y = node_y - 1;
            }
        }

        // Corridor pass 1 — body cells passable, line-cell walkable rule,
        // and any blocked head+1 (takeoff included) selects the head-hitter
        // class (mirror of moveGen.ts).
        let mut low = low_takeoff;
        for k in 0..e.n_cells {
            let c_ax = self.ext_cells[cbase + k * 2];
            let c_az = self.ext_cells[cbase + k * 2 + 1];
            let cx = x + c_ax * sx;
            let cz = z + c_az * sz;
            let c0 = self.flags_at(cx, y, cz);
            let c1 = self.flags_at(cx, y + 1, cz);
            if !self.is_safe(c0) || !self.is_safe(c1) {
                return;
            }
            let c2 = self.flags_at(cx, y + 2, cz);
            if !self.is_safe(c2) {
                low = true;
            }
            if k < e.n_line {
                let fd = self.flags_at(cx, y - 1, cz);
                if (fd & PHYSICAL) != 0 && self.height_at(cx, y - 1, cz) >= h_0 {
                    return; // walkable — not a gap
                }
            }
        }

        // Reach envelope: flight needed vs usable flight for the landing
        // bucket (mirror of moveGen.ts — the J_LOW rows when a lid is over
        // the corridor).
        // Real rise for support landings, fractional rises interpolated
        // between the integer rows (mirror of moveGen.ts).
        let dy: i32;
        let mut frac = 0.0;
        let mut rise = 0.0;
        if sup_y == i32::MIN {
            dy = node_y - y;
        } else {
            // A bottom stair is landed on its 0.5 slab (walk up the step):
            // half a block lower than the top, half a block more flight.
            let land_top = if (self.special_at(tx, sup_y, tz) & SPECIAL_STAIR) != 0 {
                sup_y as f64 + 0.5
            } else {
                self.height_at(tx, sup_y, tz)
            };
            rise = land_top - h_0;
            let mut d = rise.floor() as i32;
            frac = rise - d as f64;
            if d >= 1 {
                d = 1;
                frac = 0.0;
            }
            dy = d;
        }
        let bucket_i = 1 - dy;
        if bucket_i < 0 {
            return; // a rise above one block: no jump
        }
        let bucket = if bucket_i >= 10 { 9 } else { bucket_i as usize };
        // Run-length model (mirror of moveGen.ts): reach from the run
        // available before the lip — the support itself plus a walkable
        // cell behind, level or one step lower.
        let takeoff_catch = self.catch_at(x, y - 1, z);
        if (self.flags_at(x, y - 1, z) & CLIMBABLE) != 0 {
            return; // a ladder's top edge is not a takeoff (mirror of moveGen.ts)
        }
        let l_cred = if land_catch < 0 { LAND_HALF } else { LAND_NARROW_MARGIN + CATCH_HALF[land_catch as usize] };
        let half = CATCH_HALF[takeoff_catch];
        let front = (TAKEOFF_NARROW_MARGIN + half).min(TAKEOFF_STAND);
        let mut run = front + half + TAKEOFF_NARROW_MARGIN;
        let rx = x + e.run_x * sx;
        let rz = z + e.run_z * sz;
        let r0 = self.flags_at(rx, y, rz);
        let r1 = self.flags_at(rx, y + 1, rz);
        if self.is_safe(r0) && self.is_safe(r1) {
            let f_r = self.flags_at(rx, y - 1, rz);
            if (f_r & PHYSICAL) != 0 && self.catch_at(rx, y - 1, rz) == 0 {
                let h_r = self.height_at(rx, y - 1, rz);
                if h_r - h_0 <= 0.2 && h_0 - h_r <= 0.6 {
                    run += 1.0;
                }
            } else if self.is_safe(f_r) {
                let f_r2 = self.flags_at(rx, y - 2, rz);
                if (f_r2 & PHYSICAL) != 0 && self.catch_at(rx, y - 2, rz) == 0 {
                    let h_r = self.height_at(rx, y - 2, rz);
                    if h_0 - h_r <= 1.05 {
                        run += 1.0;
                    }
                }
            }
        }
        let row = run_row(run);
        let base = (if low { 70 } else { 0 }) + row * 10;
        let mut usable = self.ext_reach[base + bucket];
        if frac > 0.0 {
            usable = usable + (self.ext_reach[base + bucket - 1] - usable) * frac;
        }
        usable += self.cfg.margin_credit;
        let fn_needed = if takeoff_catch == 0 && land_catch <= 0 {
            e.fn_stand
        } else {
            let a = e.tx as f64;
            let b = e.tz as f64;
            flight_from(a, b, e.dist, front * e.dist / (if a > b { a } else { b }), l_cred)
        };
        let mut feasible = fn_needed <= usable;
        let mut needs_running = row >= 1;
        let mut chained = false;
        let mut lip_jump = false;
        if chain_via < 0 {
            if !feasible && self.momentum {
                // Lip take-off (mirror of moveGen.ts): half + 0.3 past centre,
                // tried only where the creep credit falls short.
                let a = e.tx as f64;
                let b = e.tz as f64;
                let s_lip = (half + LAND_NARROW_MARGIN + LIP_PHASE) * e.dist / (if a > b { a } else { b });
                if flight_from(a, b, e.dist, s_lip, l_cred) <= usable {
                    feasible = true;
                    lip_jump = true;
                }
            }
            if !feasible {
                // Momentum chain from the landing's own state (mirror of
                // moveGen.ts): continue the incoming flight within the cone,
                // chain row from the far-side landing point less the turn
                // loss, never under a lid.
                if self.mom_in == MOM_NONE || low
                    || !chain_aligned(self.mom_dx, self.mom_dz, self.mom_d2, e.tx * sx, e.tz * sz)
                {
                    return;
                }
                let a = e.tx as f64;
                let b = e.tz as f64;
                let s_chain = (TAKEOFF_NARROW_MARGIN + CATCH_HALF[takeoff_catch]).min(TAKEOFF_STAND) * CHAIN_TAKEOFF_FRACTION * e.dist / (if a > b { a } else { b });
                let cos_turn = (self.mom_dx * (e.tx * sx) + self.mom_dz * (e.tz * sz)) as f64
                    / ((self.mom_d2 * (e.tx * e.tx + e.tz * e.tz)) as f64).sqrt();
                if flight_from(a, b, e.dist, s_chain, l_cred) > self.ext_reach[CHAIN_ROW + bucket] - CHAIN_TURN_LOSS * (1.0 - cos_turn) + self.cfg.margin_credit {
                    return;
                }
                needs_running = true;
                chained = true;
            }
        } else {
            // Chain variant (mirror of moveGen.ts): only where the stone's
            // own jump falls short, from the landing point, never under a lid.
            if feasible || low {
                return;
            }
            let a = e.tx as f64;
            let b = e.tz as f64;
            let s_chain = (TAKEOFF_NARROW_MARGIN + CATCH_HALF[takeoff_catch]).min(TAKEOFF_STAND) * CHAIN_TAKEOFF_FRACTION * e.dist / (if a > b { a } else { b });
            if flight_from(a, b, e.dist, s_chain, l_cred) > self.ext_reach[CHAIN_ROW + bucket] + self.cfg.margin_credit {
                return;
            }
            needs_running = true;
        }

        // Corridor pass 2: per-cell flight-curve bound mf = the lowest the
        // feet can be over that cell (see the JS reference for the rules).
        let mf_base = (if lip_jump {
            if low { 5usize } else { 4 }
        } else {
            (if low { 2usize } else { 0 }) + (if needs_running { 1 } else { 0 })
        }) * self.ext_mf_block;
        for k in 0..e.n_cells {
            let c_ax = self.ext_cells[cbase + k * 2];
            let c_az = self.ext_cells[cbase + k * 2 + 1];
            let cx = x + c_ax * sx;
            let cz = z + c_az * sz;
            let mf = self.ext_mf[mf_base + e.cells_off + k];
            let lim = h_0 + mf + 0.05;
            let lo_ly = (lim.floor() as i32) - 1;
            let mut ly = y - 1;
            while ly >= lo_ly {
                if (ly + 1) as f64 > lim {
                    let fd = self.flags_at(cx, ly, cz);
                    if !self.is_safe(fd) {
                        return; // body cell
                    }
                } else if self.height_at(cx, ly, cz) > lim {
                    return; // pokes up into the flight path
                }
                ly -= 1;
            }
        }

        for k in 0..e.n_line {
            let c_ax = self.ext_cells[cbase + k * 2];
            let c_az = self.ext_cells[cbase + k * 2 + 1];
            cost += self.entities_at(x + c_ax * sx, y, z + c_az * sz) * self.cfg.entity_cost;
        }
        cost += self.entities_at(tx, node_y, tz) * self.cfg.entity_cost;
        // exclusion areas force the JS path — contribution here is 0.
        if cost > 100.0 {
            return;
        }
        // A narrow support landing that does not hurt carries its flight
        // direction (mirror of moveGen.ts: a full block's lip out-reaches a
        // re-jump, a damaging fall's velocity packet zeroes the motion).
        if self.momentum && land_catch >= 1 && rise >= -MOMENTUM_MAX_DROP {
            self.pending_mom = momentum_of(e.tx * sx, e.tz * sz);
        }
        if chain_via >= 0 {
            self.pending_via = chain_via;
            self.push_out(tx, node_y, tz, chain_base + cost, META_PARKOUR | META_CHAIN);
        } else if chained {
            self.pending_via = self.cell_index(x, y, z);
            self.push_out(tx, node_y, tz, cost, META_PARKOUR | META_CHAIN);
        } else {
            self.push_out(tx, node_y, tz, cost, META_PARKOUR);
        }
    }

    /// Mirror of moveGen.ts findExtLanding (findLanding order + climbables).
    fn find_ext_landing(&mut self, tx: i32, y: i32, tz: i32) -> Option<i32> {
        let mut ly = y - 1;
        while ly > self.world_min_y {
            let idx = self.cell_index(tx, ly, tz);
            if idx < 0 {
                return None;
            }
            let f = self.flags[idx as usize];
            if (f & LIQUID) != 0 && self.is_safe(f) {
                if !self.cfg.infinite_liquid_dropdown && y - ly > self.cfg.max_drop_down {
                    return None;
                }
                return Some(ly);
            }
            if !self.special.is_empty() && (self.special[idx as usize] & BUBBLE_MASK) != 0 {
                return Some(ly);
            }
            if (f & CLIMBABLE) != 0 && self.climb_usable(tx, ly, tz) {
                if y - ly > self.cfg.max_drop_down {
                    return None;
                }
                return Some(ly);
            }
            // Thin floor (carpet class): feet land IN the cell, not on top.
            if self.is_thin_floor(f) {
                if y - ly > self.cfg.max_drop_down {
                    return None;
                }
                return Some(ly);
            }
            if (f & PHYSICAL) != 0 {
                if y - (ly + 1) > self.cfg.max_drop_down {
                    return None;
                }
                return Some(ly + 1);
            }
            // Fence/wall top: a stand with the feet 0.5 into the cell above it.
            if self.is_tall_stand(f, self.heights[idx as usize]) {
                let f3 = self.flags_at(tx, ly + 3, tz);
                if y - (ly + 1) > self.cfg.max_drop_down || !self.is_safe(f3) {
                    return None;
                }
                return Some(ly + 1);
            }
            if !self.is_safe(f) {
                return None;
            }
            ly -= 1;
        }
        None
    }

    /// Mirror of moveGen.ts climbTransfers: step between adjacent climbable
    /// cells (round a pillar corner, along a wall), one up / level / one down.
    fn climb_transfers(&mut self, x: i32, y: i32, z: i32) {
        let f0 = self.flags_at(x, y, z);
        if (f0 & CLIMBABLE) == 0 || !self.is_safe(f0) || !self.climb_usable(x, y, z) {
            return;
        }
        for i in 0..8 {
            let (dx, dz) = TRANSFER[i];
            let tx = x + dx;
            let tz = z + dz;
            let mut dy = 1;
            while dy >= -1 {
                self.begin_move();
                let ty = y + dy;
                let f_t = self.flags_at(tx, ty, tz);
                if (f_t & CLIMBABLE) == 0 || !self.is_safe(f_t) || !self.climb_usable(tx, ty, tz) {
                    dy -= 1;
                    continue;
                }
                let head = self.flags_at(tx, ty + 1, tz);
                if !self.is_safe(head) {
                    dy -= 1;
                    continue;
                }
                if dy == 1 {
                    let above = self.flags_at(x, y + 2, z);
                    if !self.is_safe(above) {
                        dy -= 1;
                        continue;
                    }
                }
                if dx != 0 && dz != 0 && !self.corner_open(tx, z, y, dy) && !self.corner_open(x, tz, y, dy) {
                    dy -= 1;
                    continue;
                }
                let mut cost = TRANSFER_COST[i] + (if dy < 0 { -dy } else { dy }) as f64;
                cost += self.entities_at(tx, ty, tz) * self.cfg.entity_cost;
                if cost > 100.0 {
                    dy -= 1;
                    continue;
                }
                self.push_out(tx, ty, tz, cost, 0);
                dy -= 1;
            }
        }
    }

    /// Mirror of moveGen.ts cornerOpen.
    fn corner_open(&mut self, cx: i32, cz: i32, y: i32, dy: i32) -> bool {
        let a = self.flags_at(cx, y, cz);
        let b = self.flags_at(cx, y + 1, cz);
        if !self.is_safe(a) || !self.is_safe(b) {
            return false;
        }
        if dy == 0 {
            return true;
        }
        let c = self.flags_at(cx, y + dy, cz);
        let d = self.flags_at(cx, y + dy + 1, cz);
        self.is_safe(c) && self.is_safe(d)
    }

    /// Mirror of moveGen.ts slimeBounce: a drop onto the slime stand cell
    /// (sx, sy, sz) is also an edge to every landing its rebound reaches.
    fn slime_bounce(&mut self, x: i32, y: i32, z: i32, sx: i32, sy: i32, sz: i32) {
        let sup_idx = self.cell_index(sx, sy - 1, sz);
        if sup_idx < 0 || self.special[sup_idx as usize] != SPECIAL_SLIME {
            return;
        }
        if (self.flags_at(sx, sy, sz) & LIQUID) != 0 {
            return;
        }
        let d = y - sy;
        if d < 2 {
            return;
        }
        let apex = BOUNCE_APEX[(if d > BOUNCE_MAX_DROP { BOUNCE_MAX_DROP } else { d }) as usize];
        let via_idx = self.cell_index(sx, sy, sz);
        for i in 0..BOUNCE_OFF.len() {
            let (ox, oz) = BOUNCE_OFF[i];
            let cx = sx + ox;
            let cz = sz + oz;
            let max_top = sy as f64 + apex - (if i < BOUNCE_RING1 { BOUNCE_MARGIN } else { BOUNCE_MARGIN_FAR });
            let lid = max_top.floor() as i32;
            if lid < sy {
                continue;
            }
            if i >= BOUNCE_RING1 {
                let mx = sx + (ox >> 1);
                let mz = sz + (oz >> 1);
                let mut open = true;
                let mut ly = sy + 1;
                while ly <= lid + 1 {
                    let f = self.flags_at(mx, ly, mz);
                    if !self.is_safe(f) {
                        open = false;
                        break;
                    }
                    ly += 1;
                }
                if !open {
                    continue;
                }
            }
            let mut ly = lid;
            let mut node_y = i32::MIN;
            while ly >= sy {
                let idx = self.cell_index(cx, ly, cz);
                if idx < 0 {
                    break;
                }
                let f = self.flags[idx as usize];
                if self.is_safe(f) {
                    ly -= 1;
                    continue;
                }
                let h = self.heights[idx as usize];
                if (f & PHYSICAL) != 0 && !self.is_thin_floor(f) {
                    if ly as f64 + (h & 63) as f64 / 32.0 <= max_top {
                        node_y = ly + 1;
                    }
                } else if self.is_tall_stand(f, h) {
                    let f3 = self.flags_at(cx, ly + 3, cz);
                    if ly as f64 + (h & 63) as f64 / 32.0 <= max_top && self.is_safe(f3) {
                        node_y = ly + 1;
                    }
                }
                break;
            }
            if node_y == i32::MIN || node_y <= sy {
                continue;
            }
            let n0 = self.flags_at(cx, node_y, cz);
            let n1 = self.flags_at(cx, node_y + 1, cz);
            if !self.is_safe(n0) || !self.is_safe(n1) {
                continue;
            }
            self.begin_move();
            let adx = cx - x;
            let adz = cz - z;
            let ax = if adx < 0 { -adx } else { adx };
            let az = if adz < 0 { -adz } else { adz };
            let mut cost = (if ax > az { ax - az } else { az - ax }) as f64
                + SQRT_2 * (if ax < az { ax } else { az }) as f64
                + (y - node_y) as f64
                + 1.0;
            cost += self.entities_at(cx, node_y, cz) * self.cfg.entity_cost;
            if cost > 100.0 {
                continue;
            }
            self.pending_via = via_idx;
            self.push_out(cx, node_y, cz, cost, META_PARKOUR | META_BOUNCE);
        }
    }

    /// Mirror of moveGen.ts climbUsable (moveUp's vine-wall rule).
    fn climb_usable(&mut self, x: i32, y: i32, z: i32) -> bool {
        if self.special_at(x, y, z) != SPECIAL_VINE {
            return true;
        }
        (self.flags_at(x + 1, y, z) & PHYSICAL) != 0
            || (self.flags_at(x - 1, y, z) & PHYSICAL) != 0
            || (self.flags_at(x, y, z + 1) & PHYSICAL) != 0
            || (self.flags_at(x, y, z - 1) & PHYSICAL) != 0
    }

    /// Improvement (useBubbleColumns): ride one block up an up-column. The
    /// target cell (y+1) is the current head cell — already passable — so
    /// like move_up only the NEW head cell (y+2) is charged.
    fn move_bubble_up(&mut self, x: i32, y: i32, z: i32) {
        self.begin_move();
        if self.special_at(x, y, z) != BUBBLE_UP {
            return;
        }
        let mut cost = self.cfg.bubble_cost;
        cost += self.safe_or_break(x, y + 2, z);
        if cost > 100.0 {
            return;
        }
        self.push_out(x, y + 1, z, cost, 0);
    }

    /// Improvement (useBubbleColumns): sink one block down a down-column.
    fn move_bubble_down(&mut self, x: i32, y: i32, z: i32) {
        self.begin_move();
        if self.special_at(x, y, z) != BUBBLE_DOWN {
            return;
        }
        let mut cost = self.cfg.bubble_cost;
        cost += self.safe_or_break(x, y - 1, z);
        if cost > 100.0 {
            return;
        }
        self.push_out(x, y - 1, z, cost, 0);
    }

    fn generate(&mut self, x: i32, y: i32, z: i32, mom: u8) {
        self.out.count = 0;
        self.mom_in = if self.momentum { mom } else { MOM_NONE };
        if self.mom_in != MOM_NONE {
            self.mom_dx = momentum_dx(self.mom_in);
            self.mom_dz = momentum_dz(self.mom_in);
            self.mom_d2 = self.mom_dx * self.mom_dx + self.mom_dz * self.mom_dz;
        }
        // Extended-parkour takeoff gates are node-invariant — hoisted (mirror
        // of moveGen.ts generate()).
        let mut ext = false;
        let mut jumps = false;
        let mut h_0 = 0.0;
        let mut low_takeoff = false;
        if self.ext_on() {
            // No y+2 requirement: a blocked head+1 at the takeoff selects the
            // head-hitter class instead. `ext` supersedes upstream's cardinal
            // parkour, `jumps` runs the table — not from a climbable cell
            // (mirror of moveGen.ts).
            let f0 = self.flags_at(x, y, z);
            ext = (f0 & LIQUID) == 0 && (self.special_at(x, y, z) & BUBBLE_MASK) == 0;
            jumps = ext && (f0 & CLIMBABLE) == 0;
            if jumps {
                h_0 = self.height_at(x, y - 1, z);
                let head = self.flags_at(x, y + 2, z);
                low_takeoff = !self.is_safe(head);
            }
        }
        for i in 0..4 {
            let (dx, dz) = CARDINAL[i];
            self.move_forward(x, y, z, dx, dz);
            self.move_jump_up(x, y, z, dx, dz);
            self.move_drop_down(x, y, z, dx, dz);
            // Superseded by the extended table when that is on — see the note
            // in moveGen.ts generate(): flat-1 pricing, no reach envelope and
            // no corridor check, for extra targets that are 98.9% unflyable.
            if self.cfg.allow_parkour && !ext {
                self.move_parkour_forward(x, y, z, dx, dz);
            }
            if jumps {
                if dx != 0 {
                    for k in 0..self.ext_card_x_n {
                        let e = self.ext_entries[self.ext_diag_n + k];
                        self.parkour_ext_target(x, y, z, dx, 1, h_0, low_takeoff, e, -1, 0.0);
                    }
                } else {
                    for k in 0..self.ext_card_z_n {
                        let e = self.ext_entries[self.ext_diag_n + self.ext_card_x_n + k];
                        self.parkour_ext_target(x, y, z, 1, dz, h_0, low_takeoff, e, -1, 0.0);
                    }
                }
            }
        }
        for i in 0..4 {
            let (dx, dz) = DIAGONAL[i];
            self.move_diagonal(x, y, z, dx, dz);
            if jumps {
                for k in 0..self.ext_diag_n {
                    let e = self.ext_entries[k];
                    self.parkour_ext_target(x, y, z, dx, dz, h_0, low_takeoff, e, -1, 0.0);
                }
            }
        }
        if jumps && !self.momentum {
            self.momentum_chains(x, y, z);
        }
        self.move_down(x, y, z);
        self.move_up(x, y, z);
        if !self.special.is_empty() {
            self.move_bubble_up(x, y, z);
            self.move_bubble_down(x, y, z);
        }
        if self.ext_on() {
            self.climb_transfers(x, y, z);
        }
    }

    #[inline]
    fn decode_x(&self, idx: i32) -> i32 {
        idx % self.w + self.x0
    }

    #[inline]
    fn decode_z(&self, idx: i32) -> i32 {
        (idx / self.w) % self.l + self.z0
    }

    #[inline]
    fn decode_y(&self, idx: i32) -> i32 {
        idx / (self.w * self.l) + self.y0
    }

    fn touch_chunk(&mut self, x: i32, z: i32) {
        let cx = x >> 4;
        let cz = z >> 4;
        let key = (((cx as u32) as u64) << 32 | ((cz as u32) as u64)) + 1; // +1: 0 marks empty
        // Load factor < 1/2 keeps linear probing terminating and fast.
        if self.chunk_list.len() * 2 >= self.chunk_set.len() {
            let cap = self.chunk_set.len() * 2;
            let mask = cap - 1;
            let mut fresh = vec![0u64; cap];
            for &(ocx, ocz) in &self.chunk_list {
                let okey = (((ocx as u32) as u64) << 32 | ((ocz as u32) as u64)) + 1;
                let mut slot = ((okey.wrapping_mul(0x9E3779B97F4A7C15)) >> 48) as usize & mask;
                while fresh[slot] != 0 {
                    slot = (slot + 1) & mask;
                }
                fresh[slot] = okey;
            }
            self.chunk_set = fresh;
        }
        let mask = self.chunk_set.len() - 1;
        let mut slot = ((key.wrapping_mul(0x9E3779B97F4A7C15)) >> 48) as usize & mask;
        loop {
            let cur = self.chunk_set[slot];
            if cur == key {
                return;
            }
            if cur == 0 {
                self.chunk_set[slot] = key;
                self.chunk_list.push((cx, cz));
                return;
            }
            slot = (slot + 1) & mask;
        }
    }

    /// Runs up to max_expansions node expansions.
    /// Returns 0 = budget exhausted, 1 = success (goal node in best_idx), 2 = noPath.
    fn run(&mut self, max_expansions: u32) -> i32 {
        let mut remaining = max_expansions;
        let epoch = self.epoch;
        while !self.heap.is_empty() {
            if remaining == 0 {
                return 0;
            }
            remaining -= 1;

            let idx = self.heap.pop();
            let ui = idx as usize;
            if self.stamp[ui] != epoch || self.closed[ui] != 0 {
                continue; // stale duplicate (untouched or closed)
            }

            let cell = self.cell_of(idx);
            let x = self.decode_x(cell);
            let y = self.decode_y(cell);
            let z = self.decode_z(cell);

            if self.goal.is_end(x, y, z) {
                self.best_idx = idx;
                self.done = true;
                self.done_status = 1;
                return 1;
            }

            self.closed[ui] = 1;
            self.visited += 1;
            self.open_count -= 1;
            self.touch_chunk(x, z);

            let mom = self.mom_of(idx);
            self.generate(x, y, z, mom);
            let g = self.g[ui];

            for i in 0..self.out.count {
                let n_idx = self.slot_for(self.out.idx[i], self.out.mom[i]);
                let ni = n_idx as usize;
                let touched = self.stamp[ni] == epoch;

                let g2 = g + self.out.cost[i];
                let h = self.goal.heuristic(self.out.x[i], self.out.y[i], self.out.z[i]);
                if self.max_cost > 0.0 && g2 + h > self.max_cost {
                    continue;
                }

                if touched {
                    if self.closed[ni] != 0 {
                        if self.g[ni] <= g2 {
                            continue;
                        }
                        self.closed[ni] = 0; // reopen
                        self.visited -= 1;
                        self.open_count += 1;
                    } else if self.g[ni] < g2 {
                        continue;
                    }
                } else {
                    self.stamp[ni] = epoch;
                    self.closed[ni] = 0;
                }

                self.g[ni] = g2;
                self.parent[ni] = idx;
                self.meta[ni] = self.out.meta[i];
                self.vias[ni] = self.out.via[i];
                match self.out.breaks[i].take() {
                    Some(b) => self.breaks[ni] = Some(b),
                    None => {
                        if self.breaks[ni].is_some() {
                            self.breaks[ni] = None;
                        }
                    }
                }
                if h < self.best_h {
                    self.best_h = h;
                    self.best_idx = n_idx;
                }
                if !touched {
                    self.open_count += 1;
                }
                self.heap.push(n_idx, g2 + h);
            }
        }
        self.done = true;
        self.done_status = 2;
        2
    }

    fn serialize_result(&mut self, status: u8) {
        let mut out = Vec::with_capacity(4096);
        out.push(status);
        out.push(self.boundary_touched as u8);
        let cost = if self.best_idx >= 0 && self.stamp[self.best_idx as usize] == self.epoch {
            self.g[self.best_idx as usize]
        } else {
            0.0
        };
        out.extend_from_slice(&cost.to_le_bytes());
        out.extend_from_slice(&self.visited.to_le_bytes());
        out.extend_from_slice(&(self.visited + self.open_count).to_le_bytes());

        out.extend_from_slice(&(self.chunk_list.len() as u32).to_le_bytes());
        for (cx, cz) in &self.chunk_list {
            out.extend_from_slice(&cx.to_le_bytes());
            out.extend_from_slice(&cz.to_le_bytes());
        }

        // Path reconstruction (reverse then flip), mirroring makeResult.
        let mut nodes: Vec<i32> = Vec::new();
        if self.best_idx >= 0 {
            let mut cur = self.best_idx;
            while cur >= 0 && self.stamp[cur as usize] == self.epoch && self.parent[cur as usize] >= 0 {
                nodes.push(cur);
                cur = self.parent[cur as usize];
            }
            nodes.reverse();
        }
        out.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
        for &n in &nodes {
            let ni = n as usize;
            let cell = self.cell_of(n);
            out.extend_from_slice(&self.decode_x(cell).to_le_bytes());
            out.extend_from_slice(&self.decode_y(cell).to_le_bytes());
            out.extend_from_slice(&self.decode_z(cell).to_le_bytes());
            let parent = self.parent[ni];
            let edge = self.g[ni] - if parent >= 0 { self.g[parent as usize] } else { 0.0 };
            out.extend_from_slice(&edge.to_le_bytes());
            out.push(self.meta[ni]);
            if (self.meta[ni] & (META_BOUNCE | META_CHAIN)) != 0 {
                // The via cell follows the meta byte (wasmSolver.ts readResult).
                let via = self.vias[ni];
                out.extend_from_slice(&self.decode_x(via).to_le_bytes());
                out.extend_from_slice(&self.decode_y(via).to_le_bytes());
                out.extend_from_slice(&self.decode_z(via).to_le_bytes());
            }
            match &self.breaks[ni] {
                Some(b) => {
                    out.extend_from_slice(&(b.len() as u16).to_le_bytes());
                    for &cell in b {
                        out.extend_from_slice(&self.decode_x(cell).to_le_bytes());
                        out.extend_from_slice(&self.decode_y(cell).to_le_bytes());
                        out.extend_from_slice(&self.decode_z(cell).to_le_bytes());
                    }
                }
                None => out.extend_from_slice(&0u16.to_le_bytes()),
            }
        }
        self.result = out;
    }
}

// ── param parsing ─────────────────────────────────────────────────────────
unsafe fn read_i32(ptr: *const u8, off: &mut usize) -> i32 {
    let v = i32::from_le_bytes(core::slice::from_raw_parts(ptr.add(*off), 4).try_into().unwrap());
    *off += 4;
    v
}

unsafe fn read_u32(ptr: *const u8, off: &mut usize) -> u32 {
    let v = u32::from_le_bytes(core::slice::from_raw_parts(ptr.add(*off), 4).try_into().unwrap());
    *off += 4;
    v
}

unsafe fn read_f64(ptr: *const u8, off: &mut usize) -> f64 {
    let v = f64::from_le_bytes(core::slice::from_raw_parts(ptr.add(*off), 8).try_into().unwrap());
    *off += 8;
    v
}

/// Params blob layout — see wasmSolver.ts (packParams). Snapshot + dig
/// tables come from residency (snap_*/dig_* uploads). Returns 0 ok.
#[no_mangle]
pub unsafe extern "C" fn solve_init(params: *const u8) -> i32 {
    let st = state();

    let mut off = 0usize;
    let sx = read_i32(params, &mut off);
    let sy = read_i32(params, &mut off);
    let sz = read_i32(params, &mut off);

    let goal_count = read_i32(params, &mut off) as usize;
    if goal_count == 0 || goal_count > 4096 {
        return 1;
    }
    let mut specs = Vec::with_capacity(goal_count);
    for _ in 0..goal_count {
        let kind = read_i32(params, &mut off) as u8;
        let gx = read_f64(params, &mut off);
        let gy = read_f64(params, &mut off);
        let gz = read_f64(params, &mut off);
        let range_sq = read_f64(params, &mut off);
        specs.push(GoalSpec { kind, gx, gy, gz, range_sq });
    }

    let cfg_bits = read_i32(params, &mut off) as u32;
    let max_drop_down = read_i32(params, &mut off);
    let liquid_cost = read_f64(params, &mut off);
    let entity_cost = read_f64(params, &mut off);
    let dig_cost = read_f64(params, &mut off);
    let bubble_cost = read_f64(params, &mut off);
    let margin_credit = read_f64(params, &mut off);
    let search_radius = read_f64(params, &mut off);

    let entity_ptr = read_u32(params, &mut off) as *const u8;
    let entity_count = read_u32(params, &mut off) as usize;
    let ext_ptr = read_u32(params, &mut off) as *const u8;
    let ext_len = read_u32(params, &mut off) as usize;

    st.cfg = Config {
        allow_sprinting: cfg_bits & 1 != 0,
        allow_parkour: cfg_bits & 2 != 0,
        can_open_doors: cfg_bits & 4 != 0,
        door_mode: cfg_bits & 8 != 0,
        infinite_liquid_dropdown: cfg_bits & 16 != 0,
        can_dig: cfg_bits & 32 != 0,
        dont_create_flow: cfg_bits & 64 != 0,
        dont_mine_under_falling: cfg_bits & 128 != 0,
        use_bubble: cfg_bits & 256 != 0,
        allow_parkour_extended: cfg_bits & 512 != 0,
        allow_parkour_momentum: cfg_bits & 1024 != 0,
        max_drop_down,
        liquid_cost,
        entity_cost,
        dig_cost,
        bubble_cost,
        margin_credit,
    };

    let n = (st.w * st.h * st.l) as usize;
    if n == 0 || st.flags.len() != n {
        return 2; // snapshot not uploaded / meta mismatch
    }
    if st.cfg.can_dig && (st.states.len() != n || st.dig_labor.is_empty()) {
        return 3;
    }
    if st.cfg.use_bubble && st.special.len() != n {
        return 4; // special grid not uploaded with the snapshot
    }
    if !st.special.is_empty() && st.special.len() != n {
        return 4; // stale special grid from a different snapshot
    }

    st.entity_keys.clear();
    st.entity_weights.clear();
    if entity_count > 0 {
        let ents = core::slice::from_raw_parts(entity_ptr as *const i32, entity_count * 2);
        for i in 0..entity_count {
            st.entity_keys.push(ents[i * 2]);
            st.entity_weights.push(ents[i * 2 + 1]);
        }
    }

    // Extended-parkour table (layout: serializeParkourTable in parkourTable.ts).
    st.ext_entries.clear();
    st.ext_cells.clear();
    st.ext_mf.clear();
    st.ext_mf_block = 0;
    st.ext_diag_n = 0;
    st.ext_card_x_n = 0;
    st.ext_card_z_n = 0;
    let ext_on = st.cfg.allow_parkour && st.cfg.allow_sprinting && st.cfg.allow_parkour_extended;
    if ext_on {
        if ext_len == 0 {
            return 5; // flag set but no table — would silently diverge from JS
        }
        let mut eo = 0usize;
        st.ext_diag_n = read_i32(ext_ptr, &mut eo) as usize;
        st.ext_card_x_n = read_i32(ext_ptr, &mut eo) as usize;
        st.ext_card_z_n = read_i32(ext_ptr, &mut eo) as usize;
        let cells_len = read_i32(ext_ptr, &mut eo) as usize;
        let n_total = st.ext_diag_n + st.ext_card_x_n + st.ext_card_z_n;
        if n_total == 0 || n_total > 256 || cells_len > 8192 {
            return 5;
        }
        for _ in 0..n_total {
            let tx = read_i32(ext_ptr, &mut eo);
            let tz = read_i32(ext_ptr, &mut eo);
            let n_line = read_i32(ext_ptr, &mut eo) as usize;
            let n_cells = read_i32(ext_ptr, &mut eo) as usize;
            let cells_off = read_i32(ext_ptr, &mut eo) as usize;
            let run_x = read_i32(ext_ptr, &mut eo);
            let run_z = read_i32(ext_ptr, &mut eo);
            st.ext_entries.push(ExtEntry {
                tx, tz, n_line, n_cells, cells_off, run_x, run_z,
                dist: 0.0, cost: 0.0, fn_stand: 0.0, fn_run: 0.0,
            });
        }
        for _ in 0..cells_len {
            st.ext_cells.push(read_i32(ext_ptr, &mut eo));
        }
        eo = (eo + 7) & !7; // f64 section is 8-aligned in the blob
        for i in 0..150 {
            st.ext_reach[i] = read_f64(ext_ptr, &mut eo);
        }
        for i in 0..n_total {
            st.ext_entries[i].dist = read_f64(ext_ptr, &mut eo);
            st.ext_entries[i].cost = read_f64(ext_ptr, &mut eo);
            st.ext_entries[i].fn_stand = read_f64(ext_ptr, &mut eo);
            st.ext_entries[i].fn_run = read_f64(ext_ptr, &mut eo);
        }
        // Per-cell min-feet arrays: standing, running, low-standing,
        // low-running, lip, low-lip blocks.
        st.ext_mf_block = cells_len / 2;
        for _ in 0..cells_len * 3 {
            st.ext_mf.push(read_f64(ext_ptr, &mut eo));
        }
        if eo != ext_len {
            return 5; // layout mismatch between serializer and parser
        }
    }

    st.goal = MultiGoal { specs };
    let momentum = ext_on && st.cfg.allow_parkour_momentum;
    st.arena_prepare(n, momentum);
    st.heap.clear();
    st.chunk_set.fill(0);
    st.chunk_list.clear();
    st.visited = 0;
    st.open_count = 0;
    st.boundary_touched = false;
    st.done = false;
    st.done_status = 2;
    st.result.clear();

    let s_idx = st.cell_index(sx, sy, sz);
    let h0 = st.goal.heuristic(sx, sy, sz);
    st.max_cost = if search_radius < 0.0 { -1.0 } else { h0 + search_radius };
    st.best_h = h0;
    st.best_idx = s_idx;
    if s_idx >= 0 {
        let ui = s_idx as usize;
        let epoch = st.epoch;
        st.stamp[ui] = epoch;
        st.closed[ui] = 0;
        st.g[ui] = 0.0;
        st.parent[ui] = -1;
        st.meta[ui] = 0;
        st.vias[ui] = -1;
        if st.breaks[ui].is_some() {
            st.breaks[ui] = None;
        }
        st.heap.push(s_idx, h0);
        st.open_count = 1;
    } else {
        st.done = true;
    }

    0
}

/// 0 = budget exhausted (call again), 1 = success, 2 = noPath.
#[no_mangle]
pub unsafe extern "C" fn solve_run(max_expansions: u32) -> i32 {
    let st = state();
    if st.done {
        return st.done_status;
    }
    st.run(max_expansions)
}

/// Serialize the result with the given status code
/// (0 success, 1 partial, 2 timeout, 3 noPath). Returns result length.
#[no_mangle]
pub unsafe extern "C" fn finalize(status: u8) -> u32 {
    let st = state();
    st.serialize_result(status);
    st.result.len() as u32
}

#[no_mangle]
pub unsafe extern "C" fn result_ptr() -> *const u8 {
    state().result.as_ptr()
}
