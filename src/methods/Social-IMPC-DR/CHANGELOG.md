# Changelog — Social-IMPC-DR (Landing Pad Extension)

## Original Codebase

The Social-IMPC-DR method provides **MPC-based multi-agent navigation**.
Each drone solves a convex optimization problem (via cvxpy) every timestep to
compute the best acceleration that moves it toward its goal while avoiding
collisions with other agents.

### Original files used by our project

| File | Role |
|---|---|
| `uav.py` | Drone agent class — position, velocity, dynamics matrices, MPC state |
| `run.py` | Runs the MPC solver per drone each step (`run_cvxp`), applies control |
| `avoid.py` | Builds pairwise collision avoidance constraints (`GET_cons`) |
| `SET.py` | Global simulation parameters (timestep, horizon, agent count, etc.) |
| `test.py` | Main simulation loop: init drones → run MPC each step → collect data |
| `others.py` | Helper functions for data collection and obstacle lists |
| `dynamic.py` | Dynamics utilities |
| `app2_standardized.py` | Interactive UI: pick scenario, run simulation, generate HTML animation |
| `plot.py` / `plot_standardized.py` | Visualization and animation generation |
| `src/utils.py` | `StandardizedEnvironment` — environment configs, positions, obstacle layouts |

### What the original code does

- Multiple drones navigate from **different starts** to **different goals**
- Collision avoidance via MPC constraints (no central coordinator)
- Supports doorway, hallway, intersection scenarios
- Agents with index ≥ 2 were hardcoded as **stationary obstacles** (Umax/Vmax = 0)

### What the original code does NOT do

- No shared goal / bottleneck concept
- No turn-taking or mutual exclusion
- No notion of cargo, priority, or medical urgency
- Cannot handle all drones converging on one point

---

## Phase 1 — Baseline Bottleneck Simulation

Introduces a **single landing pad** that all drones share, with a **mutual
exclusion constraint** — only one drone may approach the pad at a time.

### New files

| File | Purpose |
|---|---|
| `landing_pad.py` | `LandingPadController` class — Phase 1 controller |
| `docs/PROJECT_GUIDELINE.md` | Full project description with 5-phase roadmap |

### `LandingPadController` methods

| Method | What it does |
|---|---|
| `select_active_drone()` | Picks the **closest** active drone to proceed; all others yield |
| `freeze_yielding()` | Zeros velocity for yielding drones (no MPC call) |
| `cleanup_landed()` | Teleports landed drones to (100, 100) so they don't block the pad |
| `reset_mpc()` | Warm-start reset when a drone transitions from yielding → active |
| `update_idle_positions()` | Appends position history for drones that skipped MPC (keeps animation aligned) |
| `step_update()` | Per-step hook (no-op in Phase 1; overridden in Phase 2) |

### Changes to original files

#### `src/utils.py`
- Added `landing_pad` environment type with shared goal at `[0, 0]`
- Added `get_landing_pad_obstacles()` — wall perimeter with approach gaps
- Added `LANDING_PAD_CENTER`, `LANDING_PAD_RADIUS` class attributes
- Added `landing_pad` entry in `get_standard_agent_positions()`
- Added landing-pad marker drawing (yellow circle + red cross) in visualization

#### `SET.py`
- Added `ENV_TYPE` parameter and `env_type` global

#### `app2_standardized.py`
- Added `landing_pad` scenario branch in the interactive menu
- Fixed legend to only show moving agents (not wall agents)

#### `avoid.py`
- `GET_cons()` accepts `wall_collision_multiplier` and `env_type` parameters
- Stationary robot detection: applies larger safety margin around walls

#### `run.py`
- Reads `env_type` and `wall_collision_multiplier` from `SET` and forwards to `GET_cons()`

#### `test.py`
- `PLAN()` accepts `num_moving_drones`, `wall_collision_multiplier`, `verbose`, `env_type`
- Imports and instantiates `LandingPadController` for landing pad scenarios
- Delegates yielding, cleanup, MPC reset, and position bookkeeping to controller
- Goal threshold relaxed to 0.05 for landing pad (0.02 for others)
- Early `break` when all moving drones reach their goals

### Bugs fixed during Phase 1

| # | Symptom | Root cause | Fix |
|---|---------|-----------|-----|
| 1 | Both drones stuck, never reaching pad | 4× `r_min` inflation in `avoid.py` made MPC infeasible | Removed inflation; use turn-based yielding instead |
| 2 | Drone 1 blocked by landed Drone 0 at (0,0) | No mechanism to clear landed drones | Teleport landed drones to (100, 100) |
| 3 | Drone 1 oscillates ~0.2 from goal | Stale MPC warm-start after long hold; ghost obstacle from stale `pre_traj` | Skip MPC for yielding drones; reset warm-start on release; update `pre_traj` on teleport |
| 4 | Animation shows both drones moving simultaneously | `post_processing()` only runs inside MPC — skipped drones had shorter position arrays | Manually append position for skipped drones |

### Phase 1 results
- 2 drones, 1 landing pad
- Drone 0 lands at step 51, Drone 1 lands at step 102
- 100% success rate

---

## Phase 2 — Dynamic Priority Integration

Replaces Phase 1's "closest drone first" policy with a **priority scoring
system** based on medical cargo criticality. A farther drone carrying a
critical organ now lands before a closer drone carrying routine equipment.

### New files

| File | Purpose |
|---|---|
| `priority.py` | `priority_score()` function — computes a [0, 1] score from 4 factors |
| `priority_manager.py` | `PriorityManager(LandingPadController)` — Phase 2 controller |

### `priority.py` details

| Component | Values |
|---|---|
| Cargo weights | organ (1.0), blood_product (0.7), medication (0.4), equipment (0.1) |
| Acuity scores | critical (1.0), urgent (0.6), routine (0.2) |
| Factor weights | cargo 35%, expiry 30%, distance 15%, acuity 20% |
| Functions | `priority_score()` → float [0, 1]; `rank_drones()` → sorted list |

### `PriorityManager` (extends `LandingPadController`)

| Override | What it does |
|---|---|
| `select_active_drone()` | Highest `priority_score()` proceeds instead of closest drone. Falls back to Phase 1 distance-based if no cargo config is set. |
| `step_update()` | Decrements `time_to_expiry` by 1 each step, making urgency increase over time |

### Changes to original files

#### `uav.py`
- `__init__()` accepts optional `cargo_type`, `time_to_expiry`, `patient_acuity` (backward-compatible defaults: equipment / 300.0 / routine)

#### `test.py`
- `PLAN()` accepts `cargo_configs` kwarg
- `initialize()` forwards cargo configs to `uav()` constructor
- Instantiates `PriorityManager` when cargo configs are provided

#### `app2_standardized.py`
- Added interactive cargo priority configuration prompt for landing pad
- Default Demo 2 configs: drone 0 = organ/critical/60s, drone 1 = equipment/routine/200s, drone 2 = medication/urgent/150s

#### `SET.py`
- `initialize_set()` accepts `NUM_MOVING_DRONES` parameter
- Stores `num_moving_drones` global for use by `run.py`

#### `run.py`
- Replaced hardcoded `if 2 <= agent.index <= 21` with `wall_start = SET.num_moving_drones`
- Agents with index ≥ `num_moving_drones` are treated as stationary obstacles
- **Bug fix:** Drone 2+ previously always got zero velocity due to the hardcoded threshold

### Phase 2 results
- 3 drones, mixed cargo, 1 landing pad
- Drone 0 (organ/critical) lands at step 36 — highest priority despite same distance
- Drone 2 (medication/urgent) lands at step 67
- Drone 1 (equipment/routine) lands at step 103
- 100% success rate, priority ordering correct

---

## Integration Prep — Hooks for Future Phases

Two forward-looking changes to smooth integration for Phases 3–5.  All
existing behavior is unchanged — the hooks are no-ops until overridden.

### `negotiation_hook()` on `LandingPadController`

A new virtual method called **before** `select_active_drone()` each step:

```python
def negotiation_hook(self, agent_list, active_drones, step):
    """Return None to proceed normally, or a result dict to override."""
    return None
```

A Phase 4 subclass can override this to run pairwise negotiation or
LLM-based arbitration **without touching** `select_active_drone()`.
If it returns a result dict the normal scoring is skipped entirely.

### `select_active_drone()` returns a metadata dict

The old return format was `(allowed_idx, yielding_drones)`.
The new format is a dict so future phases can inspect and log decisions:

```python
{
    "allowed":  int | None,     # index of the drone that may proceed
    "yielding": set[int],       # indices of drones that must hold
    "method":   str,            # "distance" | "priority" | custom
    "scores":   dict[int, float] # per-drone score used for the decision
}
```

Phase 4 can compare its own LLM-based decision against `scores`;
Phase 5 can feed the scores into a lookahead scheduler.

---

## Integration Guide for Future Phases

All future phases extend the controller hierarchy.
Create a subclass of `PriorityManager` (or `LandingPadController`) and
override the relevant hook methods — no changes to `test.py` or the MPC
solver are needed.

### Phase 3 — Holding Orbit Patterns

**Goal:** Replace "freeze in place" with smooth orbit paths while yielding.

| What to do | How |
|---|---|
| Subclass `LandingPadController` (or `PriorityManager`) | `class OrbitController(PriorityManager)` |
| Override `freeze_yielding()` | Instead of zeroing velocity, inject a circular orbit reference around the drone's current position |
| Optionally override `step_update()` | Advance the orbit phase angle each step |

`freeze_yielding()` receives the full `agent_list` and the set of yielding
indices, so the orbit controller has everything it needs.

### Phase 4 — LLM-Based Negotiation / Pairwise Arbitration

**Goal:** Replace the rule-based scoring with pairwise negotiation between
drones (or an LLM mediator).

| What to do | How |
|---|---|
| Subclass `PriorityManager` | `class NegotiationController(PriorityManager)` |
| Override `negotiation_hook()` | Run pairwise comparisons or LLM calls; return the full result dict to override normal selection |
| Use `scores` from the result dict | Log or compare rule-based vs LLM decisions for evaluation |

The `negotiation_hook()` receives `agent_list`, `active_drones`, and `step`.
If it returns a result dict, `select_active_drone()` is bypassed entirely.
Return `None` to fall through to the normal priority scoring.

### Phase 5 — Multi-Step Lookahead Scheduling

**Goal:** Schedule landing order over a planning horizon, not just one step.

| What to do | How |
|---|---|
| Subclass `PriorityManager` | `class SchedulerController(PriorityManager)` |
| Override `select_active_drone()` | Use `rank_drones()` from `priority.py` plus a planning-horizon optimizer to decide the sequence. Return the result dict with `method="scheduler"`. |
| Use `step_update()` | Track schedule progress, re-plan when drones land or new information arrives |

`rank_drones()` in `priority.py` returns a sorted list of
`(index, score)` tuples — a ready-made input for a scheduling algorithm.
