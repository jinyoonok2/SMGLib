# Priority-Aware Multi-Drone Landing Pad Coordination

A fleet of hospital drones carrying time-sensitive medical cargo must share a
**single landing pad**. Only one drone can land at a time, creating a bottleneck.
This project extends the **Social-IMPC-DR** (Infinite-Horizon Model Predictive
Control with Deadlock Resolution) framework to evaluate priority-based
coordination strategies that optimize patient welfare over simple first-come
first-served policies.

---

## Quick Start

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib
```

**Run Phase 1** (2 drones, closest-first):
```bash
python app2_standardized.py landing_pad configs/phase1_landing_pad.json
```

**Run Phase 2** (3 drones, priority-based):
```bash
python app2_standardized.py landing_pad configs/phase2_landing_pad.json
```

**Interactive mode** (manual parameter entry):
```bash
python app2_standardized.py landing_pad
```

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Project Architecture

```
configs/
  phase1_landing_pad.json ──┐
  phase2_landing_pad.json ──┤    Scenario configs: drone positions,
  priority_config.json ─────┤    simulation params, cargo assignments,
                            │    and priority scoring parameters.
                            ▼
app2_standardized.py        ← ENTRY POINT
  │                           Loads scenario config OR prompts user.
  │
  ├── Calls PLAN() ────────► test.py (simulation engine)
  │                            │
  │     Receives configs      │
  │     from caller.          │
  │                            ├── Creates drones ────► uav.py
  │                            │                         Position, velocity,
  │                            │                         cargo, expiry, acuity.
  │                            │
  │                            ├── Creates policy ────► LandingPadController
  │                            │      │                  or PriorityManager
  │                            │      │
  │                            │      └── Uses ───────► priority.py
  │                            │                         Scoring math.
  │                            │
  │                            └── Runs physics ──────► run.py (MPC solver)
  │                                                      Unchanged from
  │                                                      original codebase.
  │
  └── Generates animation ──► .gif / .html / .avi
```

---

## Phase 1 — Baseline Bottleneck (Completed)

**Goal:** Establish the landing pad environment with a mutual exclusion
constraint — only one drone may approach the pad at a time.

**Policy:** The **closest drone** to the pad proceeds; all others freeze in place.

**Controller:** `LandingPadController` in `landing_pad.py`

| Method | What it does |
|---|---|
| `select_active_drone()` | Picks closest active drone; others yield |
| `freeze_yielding()` | Zeros velocity for waiting drones |
| `cleanup_landed()` | Removes landed drones from the pad |
| `reset_mpc()` | Warm-start reset when a drone transitions from yielding → active |
| `update_idle_positions()` | Keeps animation aligned for frozen drones |

**Result:** 2 drones, 1 pad. Drone 0 lands at step 51, Drone 1 at step 102.
100% success rate.

**Metrics (Phase 1):**

| Metric | Value |
|---|---|
| Total delivery time | 102 steps |
| Pad idle time | 0 steps (Drone 1 starts immediately after Drone 0 lands) |
| Collision events | 0 |
| Success rate | 100% |

![Phase 1 — 2 drones, closest-first](../../../logs/Social-IMPC-DR/animations/landing_pad_2agents.gif)

---

## Phase 2 — Dynamic Priority Integration (Completed)

**Goal:** Replace closest-first with a **priority scoring system** based on
medical cargo criticality.

**Policy:** The drone with the **highest priority score** proceeds. A farther
drone carrying a critical organ lands before a closer drone with routine
equipment.

**Controller:** `PriorityManager(LandingPadController)` in `priority_manager.py`

**Scoring** (`priority.py`):

| Factor | Weight | Values |
|---|---|---|
| Cargo type | 35% | organ (1.0), blood_product (0.7), medication (0.4), equipment (0.1) |
| Time to expiry | 30% | Lower remaining time → higher urgency |
| Distance to pad | 15% | Closer → higher score |
| Patient acuity | 20% | critical (1.0), urgent (0.6), routine (0.2) |

Output: `priority_score()` → float in [0, 1]

**Result:** 3 drones, mixed cargo, 1 pad.
- Drone 0 (organ/critical) lands at step 36 — highest priority
- Drone 2 (medication/urgent) lands at step 67
- Drone 1 (equipment/routine) lands at step 103
- 100% success, correct priority ordering.

**Metrics (Phase 2):**

| Metric | Value |
|---|---|
| Total delivery time | 103 steps |
| Priority inversions | 0 (correct ordering: organ → medication → equipment) |
| Weighted delivery delay | Organ delivered first despite being farther — delay minimized for highest-criticality cargo |
| Collision events | 0 |
| Success rate | 100% |

![Phase 2 — 3 drones, priority-based](../../../logs/Social-IMPC-DR/animations/landing_pad_3agents.gif)

---

## Scenario Configuration

Each phase is driven by a JSON config file in `configs/`. Example
(`phase2_landing_pad.json`):

```json
{
    "env_type": "landing_pad",
    "verbose": true,
    "num_moving_drones": 3,
    "min_radius": 0.5,
    "wall_collision_multiplier": 2.0,
    "epsilon": 0.1,
    "step_size": 0.1,
    "k_value": 10,
    "max_steps": 200,
    "use_priority": true,
    "drones": [
        {"start": [0.0, 3.0],  "goal": [0.0, 0.0], "cargo_type": "organ",      "time_to_expiry": 60.0,  "patient_acuity": "critical"},
        {"start": [0.0, -3.0], "goal": [0.0, 0.0], "cargo_type": "equipment",  "time_to_expiry": 200.0, "patient_acuity": "routine"},
        {"start": [-2.5, 0.0], "goal": [0.0, 0.0], "cargo_type": "medication", "time_to_expiry": 150.0, "patient_acuity": "urgent"}
    ]
}
```

| Field | Purpose |
|---|---|
| `env_type` | Environment layout (`landing_pad`, `doorway`, `hallway`, `intersection`) |
| `num_moving_drones` | Number of active drones |
| `use_priority` | `false` → Phase 1 (closest-first), `true` → Phase 2 (priority-based) |
| `drones` | Per-drone start/goal positions and cargo attributes |
| `min_radius` | Minimum safe distance between agents |
| `wall_collision_multiplier` | Safety margin multiplier for wall agents |
| `max_steps` | Simulation time limit |

---

## Priority Configuration

Priority scoring parameters are defined in `configs/priority_config.json`:

```json
{
    "factor_weights": {
        "cargo":    0.35,
        "expiry":   0.30,
        "distance": 0.15,
        "acuity":   0.20
    },
    "cargo_weights": {
        "organ":         1.0,
        "blood_product": 0.7,
        "medication":    0.4,
        "equipment":     0.1
    },
    "acuity_scores": {
        "critical": 1.0,
        "urgent":   0.6,
        "routine":  0.2
    },
    "max_distance": 10.0,
    "max_expiry": 300.0
}
```

| Field | Purpose |
|---|---|
| `factor_weights` | How much each factor contributes to the final score (must sum to 1.0) |
| `cargo_weights` | Categorical score per cargo type (higher = more urgent) |
| `acuity_scores` | Categorical score per patient acuity level |
| `max_distance` | Normalisation cap for the distance component |
| `max_expiry` | Normalisation cap for the expiry component |

This file is **required** — `priority.py` will print an error and exit if it is
missing. Scenario configs define *what* is being simulated; priority config
defines *how* the scoring system evaluates drones.

---

## Policy Class Hierarchy

```
LandingPadController              ← Phase 1: closest drone first
    │
    └── PriorityManager           ← Phase 2: highest priority score first
            │
            └── (Phase 3+)        ← Override hooks for new behavior
```

Each subclass only overrides the methods it changes. `test.py` calls the same
interface regardless of which controller is active. Two hooks exist for future
phases:

- `negotiation_hook()` — called before `select_active_drone()`. Return `None`
  for normal behavior, or a result dict to override. (Phase 4 target)
- `step_update()` — called each simulation step. Phase 2 uses it to decrement
  `time_to_expiry`. (Extensible for Phase 3+)

---

## Next Phases (This is just for an example, you can build it in a direction you want)

### Phase 3 — Holding Patterns

**Goal:** Yielding drones fly circular orbits instead of freezing in place.

**What to implement:**
- Subclass `PriorityManager` → `OrbitController`
- Override `freeze_yielding()` to compute circular orbit positions
- Add orbit parameters (radius, speed) to the scenario config
- VIP drones fly direct; low-priority drones enter holding pattern when yielding

### Phase 4 — Pre-Contact Negotiation

**Goal:** When two drones are near the pad simultaneously, they exchange status
and negotiate who yields.

**What to implement:**
- Subclass `PriorityManager` → `NegotiationController`
- Override `negotiation_hook()` to trigger pairwise arbitration
- Two modes: **rule-based** (deterministic score comparison) vs **LLM-enhanced**
  (format drone status as prompt, LLM returns yield decision)
- Run same scenario with both modes, compare outcomes

### Phase 5 — Lookahead Scheduling & Global Evaluation

**Goal:** Plan the full landing queue over a time horizon instead of greedy
per-step decisions. This phase also owns the **global evaluation metrics**
for the entire project.

**What to implement:**
- Subclass `PriorityManager` → `SchedulerController`
- Override `select_active_drone()` to use `rank_drones()` + time-horizon optimizer
- Implement data logging and reporting for the metrics below

**Evaluation metrics (tracked across all phases):**

| Metric | Description | Phases |
|---|---|---|
| Pad utilization | Fraction of time the pad is actively in use vs. idle | All |
| Priority inversion rate | How often a lower-priority drone lands before a higher-priority one | 2+ |
| Total weighted delivery delay | Sum of (priority_weight × delivery_delay) across all drones | 2+ |
| Patient welfare proxy | Mathematical score based on delivery times and cargo criticality | 2+ |
| VIP delay | Additional time a VIP drone waits due to conflicts | 3+ |
| Decision latency | Time to resolve negotiation between drones | 4+ |
| Global weighted delay vs. greedy | Comparison of lookahead scheduler against greedy baseline | 5 |

> **Note:** Phases 1–2 report basic metrics inline (delivery time, priority
> inversions, success rate). The full metrics dashboard is a Phase 5
> deliverable — there is no separate "evaluation phase."

---

## Future Extensions

### Extension A — Round-Trip Scenarios (Phase 6 candidate)

**Goal:** Drones don't just arrive — they land, unload, and depart back to
their origin, turning the pad into a **two-way bottleneck** in a continuous
delivery loop.

**Why it matters:** The current roadmap (Phases 1–5) treats the landing pad as
a finish line. Round trips test **sustained throughput** where the pad is a
temporary stop in a Source → Hospital → Source cycle.

**Feasibility:** High. The codebase already supports mid-simulation goal changes
via `uav.change_target()`. No MPC or physics changes are needed.

**What to implement:**
- Per-drone trip-state FSM: `INBOUND → LANDED → UNLOADING → OUTBOUND → DONE`
- Override `cleanup_landed()` to call `change_target(return_point)` instead of
  teleporting the drone off-screen
- Add `round_trip: true` flag and return waypoints to the scenario config
- Manage both arrival and departure queues at the pad

### Extension B — Continuous Speed Control (Phase 3/5 integration)

**Goal:** Instead of freezing yielding drones completely, modulate their flight
speed so they arrive exactly when the pad clears — replacing discrete Go/Stop
decisions with continuous velocity optimization.

**Why it matters:** In real robotics, stopping completely is energy-inefficient.
A drone told to fly at 50% speed two miles out can arrive precisely when the pad
is free, eliminating idle hover time.

**Relationship to existing phases:**
- **Phase 3** proposes circular holding orbits. Speed control is the alternative:
  slow approach instead of loitering.
- **Phase 5** proposes lookahead scheduling. Predictive speed tuning is its
  advanced form — the `SchedulerController` computes the optimal `speed_factor`
  per drone so that arrivals are sequenced without stopping.

**Feasibility:** Two tiers.

| Tier | Description | Effort |
|---|---|---|
| **A — Vmax scaling** | Set `agent.Vmax = 1.0 * speed_factor` per drone. MPC naturally plans a slower trajectory. Override `freeze_yielding()` to reduce speed instead of zeroing velocity. | Low (~30 lines) |
| **B — Predictive arrival** | `SchedulerController` estimates pad clearance time, computes required speed factor per drone so `ETA == pad_clear_time`. Re-solves every N steps. | Medium-high (~400-500 lines) |

> **Note:** `Vmax` is a ceiling, not a target — the MPC may fly slower than the
> cap due to collision avoidance or terminal constraints. Tier B would need
> iterative tuning or a two-layer optimizer (scheduler sets Vmax, MPC executes).

---

## File Reference

| File | Status | Purpose |
|---|---|---|
| `app2_standardized.py` | Modified | Entry point — config-file mode or interactive mode |
| `test.py` | Modified | Simulation engine — delegates to controllers |
| `uav.py` | Modified | Drone data object — added cargo/priority attributes |
| `run.py` | Modified | MPC solver — fixed hardcoded index threshold |
| `avoid.py` | Modified | Collision constraints — wall multiplier support |
| `SET.py` | Modified | Global params — added `num_moving_drones`, `ENV_TYPE` |
| `landing_pad.py` | New | Phase 1 controller — mutual exclusion, yielding |
| `priority.py` | New | Scoring math — `priority_score()`, `rank_drones()` |
| `priority_manager.py` | New | Phase 2 controller — priority-based policy |
| `test_phase2.py` | New | Standalone Phase 2 test script |
| `configs/*.json` | New | Scenario configs and priority scoring parameters |
| `src/utils.py` | Modified | Added `landing_pad` environment type |

---

## Environment Setup

```bash
conda create -n smglib python=3.10
conda activate smglib
pip install -r requirements.txt   # cvxpy, numpy, matplotlib, scipy
```
