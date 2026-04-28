# Social-IMPC-DR — Track 1 (Policy/Yield)

This branch implements **single-winner landing-pad coordination** for medical
drones sharing one central pad. Drones do not all fly together; instead,
exactly one inbound drone holds the pad while the rest yield based on a
selected policy. Other behaviour (priority scoring, holding orbits,
ETA/expiry negotiation, round-trip trips) is composed on top via peer
plugins.

The trajectory-planner formulation (simultaneous arrivals, speed scheduling)
is **not** included here — it lives in `research/track-trajectory-planner`.

---

## 1. Setup

### 1.1 Prerequisites

- Python 3.10
- Conda (Miniconda/Anaconda)

### 1.2 Environment

```bash
conda create -n smglib python=3.10
conda activate smglib
```

### 1.3 Install dependencies

```bash
cd src/methods/Social-IMPC-DR
pip install -r requirements.txt
```

`requirements.txt` pins:

- `cvxpy==1.6.0` — MPC solver
- `numpy==2.2.3`, `scipy==1.15.2` — numerics
- `matplotlib==3.10.0` — animation rendering
- `opencv-python==4.11.0.86`, `PyQt5==5.15.11` — display utilities

### 1.4 Quick verification

Run the simplest scenario to confirm the install:

```bash
python app2_standardized.py landing_pad configs/track_policy_baseline_closest.json
```

A GIF is written to `logs/Social-IMPC-DR/animations/`.

---

## 2. Method

### 2.1 Setting

A landing pad sits at a fixed location. Multiple drones converge on it
to deliver cargo. The pad supports **one drone at a time**: every step,
the controller decides who proceeds and who yields.

### 2.2 Decision model

Each scenario picks one component per role; the controller composes
them every step:

| Role         | What it decides                                                    |
|--------------|--------------------------------------------------------------------|
| `selector`   | Which inbound drone proceeds to the pad                            |
| `yielder`    | How non-winners hold (stay still vs orbit)                         |
| `lifecycle`  | What happens when a drone reaches the pad (land once vs round-trip)|
| `negotiator` | Optional override(s) before the selector commits a winner          |
| `hysteresis` | If on, the chosen winner keeps the lead until they land            |

Implementations available right now:

- `selector`: `closest_first`, `priority`
- `yielder`: `freeze`, `orbit`
- `lifecycle`: `one_way`, `round_trip`
- `negotiator`: `expiry_guard`, `eta_switch`

### 2.3 Per-step decision flow

1. `lifecycle.cleanup_landed` — handle drones that just touched down
   (teleport off-pad for `one_way`, or advance the FSM for `round_trip`).
2. `lifecycle.filter_inbound` — keep only candidates eligible to compete
   for the pad (`round_trip` excludes `OUTBOUND` drones).
3. `lifecycle.force_yield` — emergency overrides like the pad-busy guard
   while another drone is `UNLOADING`.
4. Negotiator chain (in declared order) — first non-`None` override wins.
5. Hysteresis (if enabled) — if the previous winner is still inbound
   and no `expiry_guard` is overriding, keep them.
6. `selector.select` — final fallback to score-based selection.
7. `yielder.apply` — apply the holding behaviour to all non-winners.

The decision is returned to the simulation loop in this schema:

```python
{"allowed": int | None, "yielding": set[int], "method": str, "scores": dict[int, float]}
```

### 2.4 Priority score (used by `priority` selector and the negotiators)

```
score(j) = w_cargo  * cargo_weight[cargo_type]
         + w_expiry * (1 - clamp(time_to_expiry / max_expiry))
         + w_dist   * (1 - clamp(distance_to_pad / max_distance))
         + w_acuity * acuity_score[patient_acuity]
```

Weights, cargo categories, acuity scores, and clamp bounds live in
`configs/priority_config.json`.

### 2.5 Negotiator rules

- `expiry_guard`: if any inbound drone’s `time_to_expiry < ETA_to_pad`,
  the most urgent one wins immediately. Treated as authoritative — it
  can break a held hysteresis winner.
- `eta_switch`: if the gap between the top two priority scores is below
  `eta_threshold`, the closer drone wins (tie-break by ETA).

### 2.6 Round-trip lifecycle FSM

Per drone:

```
INBOUND  -> UNLOADING -> OUTBOUND -> (next inbound)  ... -> DONE
```

- `INBOUND`: actively competes for the pad.
- `UNLOADING`: parked on the pad for `unload_steps`. Forces all other
  inbound drones to yield (pad-busy guard).
- `OUTBOUND`: flies home; not eligible for pad selection.
- `DONE`: parked at the home pad after `n_trips` trips.

---

## 3. Implementation

### 3.1 Package layout (flat plugin model)

```
src/methods/Social-IMPC-DR/
├── policy_yield/
│   ├── __init__.py            # public API: build_policy_yield_controller
│   ├── controller.py          # PolicyYieldController orchestrator
│   ├── registry.py            # name -> class lookup (one entry per role)
│   ├── selectors.py           # closest_first, priority
│   ├── yielders.py            # freeze, orbit
│   ├── lifecycles.py          # one_way, round_trip
│   ├── negotiators.py         # expiry_guard, eta_switch
│   └── context.py             # per-step Context dataclass + PAD_CENTER
├── app2_standardized.py       # entry point: scenario JSON or interactive
├── test.py                    # simulation loop, calls into the controller
├── priority.py                # weighted medical priority score
├── configs/
│   ├── priority_config.json
│   └── track_policy_*.json    # behaviour-named scenarios
└── (MPC core: run.py, avoid.py, uav.py, SET.py, others.py, plot.py, ...)
```

The MPC core (`run.py`, `avoid.py`, `uav.py`, `SET.py`, `dynamic.py`) is
unchanged from upstream Social-IMPC-DR.

### 3.2 Adding a new policy

To add e.g. an LLM-driven negotiator:

1. Subclass the role base class in the matching module
   (e.g. `Negotiator` in `policy_yield/negotiators.py`).
2. Implement the role’s single method (`override`, `select`, `apply`,
   or `cleanup_landed`/`filter_inbound`/`force_yield`/`step_update` for
   lifecycles).
3. Register the new name in `policy_yield/registry.py`:

   ```python
   NEGOTIATORS["llm_negotiator"] = LLMNegotiator
   ```
4. Reference the new name from any scenario JSON:

   ```json
   "negotiators": ["llm_negotiator"]
   ```

No subclassing of existing controllers is needed.

### 3.3 Data flow (per simulation step)

```
test.PLAN(...)
  └─ controller.cleanup_landed
  └─ controller.select_active_drone        ─► returns decision dict
        └─ lifecycle.filter_inbound
        └─ lifecycle.force_yield (optional)
        └─ negotiators[*].override
        └─ hysteresis logic (optional)
        └─ selector.select
  └─ controller.freeze_yielding             ─► yielder.apply
  └─ run_one_step (MPC) for non-yielding drones
  └─ controller.update_idle_positions
  └─ controller.step_update                 ─► lifecycle.step_update + tte countdown
  └─ controller.all_finished                ─► termination check
```

---

## 4. Run

### 4.1 Available scenarios

| Config file                                       | Recipe summary                                                                |
|---------------------------------------------------|-------------------------------------------------------------------------------|
| `track_policy_baseline_closest.json`              | `closest_first` + `freeze` + `one_way`                                        |
| `track_policy_priority.json`                      | `priority` + `freeze` + `one_way` + hysteresis                                |
| `track_policy_orbit_hold.json`                    | `priority` + `orbit` + `one_way` + hysteresis                                 |
| `track_policy_negotiation.json`                   | `priority` + `orbit` + `one_way` + `[expiry_guard, eta_switch]` + hysteresis |
| `track_policy_negotiation_no_hysteresis.json`     | Same as above with `use_hysteresis: false`                                    |
| `track_policy_negotiation_expiry_guard.json`      | Negotiation stack tuned to stress the expiry-guard rule                       |
| `track_policy_negotiation_eta_switch.json`        | Negotiation stack tuned to stress the ETA-switch rule                         |
| `track_policy_round_trip.json`                    | Full negotiation stack + `round_trip` lifecycle                               |

### 4.2 Commands

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib

# Baseline + priority
python app2_standardized.py landing_pad configs/track_policy_baseline_closest.json
python app2_standardized.py landing_pad configs/track_policy_priority.json

# Orbit hold + negotiation variants
python app2_standardized.py landing_pad configs/track_policy_orbit_hold.json
python app2_standardized.py landing_pad configs/track_policy_negotiation.json
python app2_standardized.py landing_pad configs/track_policy_negotiation_expiry_guard.json
python app2_standardized.py landing_pad configs/track_policy_negotiation_eta_switch.json
python app2_standardized.py landing_pad configs/track_policy_negotiation_no_hysteresis.json

# Round-trip lifecycle
python app2_standardized.py landing_pad configs/track_policy_round_trip.json
```

Outputs (per run):

- `logs/Social-IMPC-DR/animations/<test_name>.gif`
- `avg_delta_velocity_robot_*.csv`, `path_deviation_robot_*.csv`,
  `ttg_impc_dr.csv`, `completion_step.txt` in the run directory

---

## 5. Scenario configuration

Each scenario JSON has a top-level `policy` block selecting one component
per role plus the negotiator list. Example
(`configs/track_policy_round_trip.json`):

```json
{
    "test_name": "track_policy_round_trip",
    "env_type": "landing_pad",
    "num_moving_drones": 3,
    "max_steps": 1000,
    "policy": {
        "selector":       "priority",
        "yielder":        "orbit",
        "lifecycle":      "round_trip",
        "negotiators":    ["expiry_guard", "eta_switch"],
        "use_hysteresis": true,
        "orbit_radius":   0.7,
        "orbit_speed":    0.15,
        "safe_distance":  1.2,
        "nominal_speed":  0.1,
        "eta_threshold":  0.15,
        "n_trips":        2,
        "unload_steps":   5
    },
    "drones": [
        { "start": [3.0, 2.0], "return_point": [3.0, 2.0], "goal": [0.0, 0.0],
          "cargo_type": "equipment", "time_to_expiry": 800.0, "patient_acuity": "routine" },
        ...
    ]
}
```

### 5.1 Recipe fields

| Field             | Purpose                                                                |
|-------------------|------------------------------------------------------------------------|
| `selector`        | `closest_first` (distance) or `priority` (priority score)              |
| `yielder`         | `freeze` (zero velocity) or `orbit` (circular hold)                    |
| `lifecycle`       | `one_way` (land once) or `round_trip` (FSM with unload + return)       |
| `negotiators`     | List of peer overrides applied in order (e.g. `[expiry_guard, eta_switch]`) |
| `use_hysteresis`  | If true, lock the current winner until they finish                     |
| `orbit_radius`    | Orbit yielder: hold ring radius                                        |
| `orbit_speed`     | Orbit yielder: angular speed (rad/step)                                |
| `safe_distance`   | Orbit yielder: extra clearance from the active drone                   |
| `nominal_speed`   | Negotiators: speed used to compute ETA                                 |
| `eta_threshold`   | `eta_switch`: score-gap threshold below which it fires                 |
| `n_trips`         | `round_trip`: number of trips before `DONE`                            |
| `unload_steps`    | `round_trip`: pad-occupancy steps per landing                          |

### 5.2 Scenario-level fields

| Field                       | Purpose                                            |
|-----------------------------|----------------------------------------------------|
| `test_name`                 | Used for the output GIF filename                   |
| `env_type`                  | Must be `landing_pad` for this track               |
| `num_moving_drones`         | Number of moving drones                            |
| `max_steps`                 | Episode length cap                                 |
| `min_radius`, `epsilon`, `step_size`, `k_value`, `wall_collision_multiplier` | Underlying MPC/environment parameters              |
| `drones`                    | List of `{ start, goal, cargo_type, time_to_expiry, patient_acuity, return_point? }` |

---

## 6. Relation to other track

- This branch implements **Track 1 only** (policy/yield family).
- Trajectory-planner work (simultaneous-arrival speed scheduling) lives
  in `research/track-trajectory-planner`.
- LLM extensions for either track go on dedicated child branches; see
  `LLM_BRANCH_BOOTSTRAP.md`.
