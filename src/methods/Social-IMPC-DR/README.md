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
python app2_standardized.py landing_pad scenarios/phase1_landing_pad.json
```

**Run Phase 2** (3 drones, priority-based):
```bash
python app2_standardized.py landing_pad scenarios/phase2_landing_pad.json
```

**Interactive mode** (manual parameter entry):
```bash
python app2_standardized.py landing_pad
```

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Project Architecture

```
scenarios/
  phase1_landing_pad.json ──┐
  phase2_landing_pad.json ──┤    Scenario configs: drone positions,
                            │    simulation params, cargo assignments.
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

![Phase 2 — 3 drones, priority-based](../../../logs/Social-IMPC-DR/animations/landing_pad_3agents.gif)

---

## Scenario Configuration

Each phase is driven by a JSON config file in `scenarios/`. Example
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

## Next Phases (For Teammates)

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

### Phase 5 — Lookahead Scheduling

**Goal:** Plan the full landing queue over a time horizon instead of greedy
per-step decisions.

**What to implement:**
- Subclass `PriorityManager` → `SchedulerController`
- Override `select_active_drone()` to use `rank_drones()` + time-horizon optimizer
- Track metrics: pad utilization, priority inversion rate, total weighted
  delivery delay, patient welfare proxy

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
| `scenarios/*.json` | New | Scenario configs for automated runs |
| `src/utils.py` | Modified | Added `landing_pad` environment type |

---

## Environment Setup

```bash
conda create -n smglib python=3.10
conda activate smglib
pip install -r requirements.txt   # cvxpy, numpy, matplotlib, scipy
```
