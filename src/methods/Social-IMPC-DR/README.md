# Social-IMPC-DR — Track 2 (Trajectory Planner)

This branch implements **planner-based simultaneous inbound flight** for a
shared medical landing pad. Instead of choosing a single winner and
forcing the rest to yield (the Track 1 model), the planner computes a
per-drone cruise speed cap so all drones fly at the same time and arrive
at the pad in priority order with safe spacing.

The single-winner / yielding family (closest-first, priority, orbit,
ETA/expiry negotiation) is **not** included here — it lives in
`research/track-policy-yield`.

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

Run the planner one-way scenario to confirm the install:

```bash
python app2_standardized.py landing_pad configs/phase7_planner_oneway.json
```

A GIF is written to `logs/Social-IMPC-DR/animations/`.

---

## 2. Method

### 2.1 Setting

A landing pad sits at a fixed location. Multiple drones converge on it
to deliver cargo. Unlike Track 1, all drones fly **at the same time**;
the planner shapes their speeds so arrivals happen sequentially with
guaranteed spacing.

### 2.2 Speed-scaling formula

Drones are ranked by priority score (highest first). For drone `k` with
distance `d_k` to the pad, given the previous drone’s arrival time
`T_{k-1}` and assigned speed `v_{k-1}`, the planner computes:

```
T_arrive_k = max(
    d_k / max_speed,                              # physical floor
    T_{k-1} + unload_steps,                       # pad-clearance floor
    T_{k-1} + min_separation / v_{k-1},           # spatial-separation floor
)
v_k        = min(d_k / T_arrive_k, max_speed)
```

The top-ranked drone gets `T_arrive_1 = d_1 / max_speed` (full speed,
unconstrained). Each subsequent drone’s arrival ratchets forward.

`OUTBOUND` drones (returning home after a delivery) are restored to
`max_speed` — they do not contend with the inbound queue.

### 2.3 Replanning

The planner re-runs whenever the inbound set changes:

- A drone lands and starts unloading (`INBOUND -> UNLOADING`).
- A drone leaves the pad heading home (`UNLOADING -> OUTBOUND`).
- A drone re-enters the inbound queue (`OUTBOUND -> INBOUND`,
  round-trip mode).

This keeps the schedule consistent across the lifecycle.

### 2.4 Pad-busy safety net

Even with a feasible schedule, MPC overshoot or local collision
avoidance can drift an inbound drone toward the pad while another is
still unloading. As a guard:

- If any drone is `UNLOADING`, every inbound drone whose distance to
  the pad falls below `safe_distance` is frozen for that step.

This is a safety net, not a steady-state mechanism. With a feasible
`max_speed` / `min_separation` / `unload_steps`, it should rarely fire.

### 2.5 Lifecycle reuse

Lifecycle bookkeeping is shared with Track 1’s round-trip controller:

```
INBOUND  -> UNLOADING -> OUTBOUND -> (next inbound)  ... -> DONE
```

- One-way scenarios: set `n_trips = 1`.
- Round-trip scenarios: set `n_trips >= 2`.

`bind(target)`, target swapping, home-pad markers, and `all_finished`
are inherited unchanged.

---

## 3. Implementation

### 3.1 File layout

```
src/methods/Social-IMPC-DR/
├── trajectory_planner_controller.py   # planner scheduling + per-drone Vmax
├── round_trip_controller.py           # lifecycle FSM (reused base class)
├── negotiation_controller.py          # base class for round_trip
├── orbit_controller.py                # base class for negotiation
├── priority_manager.py                # base class for orbit
├── landing_pad.py                     # base class root
├── priority.py                        # weighted medical priority score
├── app2_standardized.py               # entry point: scenario JSON or interactive
├── test.py                            # simulation loop, controller selection
├── configs/
│   ├── phase7_planner_oneway.json     # one-way planner scenario
│   ├── phase7_planner_round_trip.json # round-trip planner scenario
│   ├── priority_config.json           # priority weights, cargo/acuity scores
│   └── (legacy phase1..6 configs kept for reference)
└── (MPC core: run.py, avoid.py, uav.py, SET.py, others.py, plot.py, ...)
```

The MPC core (`run.py`, `avoid.py`, `uav.py`, `SET.py`, `dynamic.py`) is
unchanged from upstream Social-IMPC-DR.

### 3.2 Controller class structure

`TrajectoryPlannerController` extends `RoundTripController` so the FSM,
target swapping, and `all_finished` logic are reused. Planner-specific
behaviour overrides:

- `select_active_drone(...)` — instead of picking one winner, returns
  `{ allowed: None, yielding: <near-pad drones during pad-busy>, method: "planner" }`.
- `_replan(...)` — internal: scores inbound drones, ranks them, and
  assigns `T_arrive` and `Vmax` per the speed-scaling formula. Pushes
  `Vmax` directly to each `uav` so MPC enforces it.
- `cleanup_landed(...)`, `step_update(...)` — wrap the FSM transitions
  and mark the schedule dirty so `_replan` runs next step.

### 3.3 Data flow (per simulation step)

```
test.PLAN(...)
  └─ controller.cleanup_landed             ─► FSM transitions, mark dirty
  └─ controller.select_active_drone        ─► may call _replan(...)
        └─ filter inbound (state == INBOUND)
        └─ if dirty or inbound set changed: _replan
            ├─ priority_score per drone
            ├─ rank, assign T_arrive + Vmax
            ├─ push Vmax into agent_list[j].Vmax
            └─ OUTBOUND drones get full max_speed
        └─ pad-busy safety net (freeze near-pad drones)
  └─ controller.freeze_yielding            ─► safety-net only; freezes near-pad inbound
  └─ run_one_step (MPC) for non-yielding drones
  └─ controller.update_idle_positions
  └─ controller.step_update                ─► FSM transitions, mark dirty
  └─ controller.all_finished               ─► termination check
```

### 3.4 Wiring in `test.py`

`PLAN(...)` chooses the controller based on flags inside `round_trip_params`:

- `use_trajectory_planner: true` → `TrajectoryPlannerController`
- else `round_trip: true`        → `RoundTripController`
- else negotiation/orbit/priority/baseline (Track 1 fallback path)

`app2_standardized.py` parses the scenario JSON, builds `round_trip_params`
(including `max_speed`, `min_separation`, `n_trips`, `unload_steps`, …),
and tags the output GIF as `planner` when planner mode is active.

---

## 4. Run

### 4.1 Available scenarios

| Config file                          | What it exercises                                  |
|--------------------------------------|----------------------------------------------------|
| `phase7_planner_oneway.json`         | 3 drones, simultaneous flight, single delivery     |
| `phase7_planner_round_trip.json`     | 3 drones, 2 trips each, full Source → Pad → Source loop |

### 4.2 Commands

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib

# One-way planner scenario
python app2_standardized.py landing_pad configs/phase7_planner_oneway.json

# Round-trip planner scenario
python app2_standardized.py landing_pad configs/phase7_planner_round_trip.json
```

Outputs (per run):

- `logs/Social-IMPC-DR/animations/<test_name>.gif`
- `avg_delta_velocity_robot_*.csv`, `path_deviation_robot_*.csv`,
  `ttg_impc_dr.csv`, `completion_step.txt` in the run directory

Verbose console output includes a periodic schedule dump:

```
[Planner] step 1 schedule -> D2: v=0.367 T=10.0, D0: v=0.071 T=20.0, ...
```

---

## 5. Scenario configuration

Example (`configs/phase7_planner_oneway.json`):

```json
{
    "test_name": "phase7_planner_oneway",
    "env_type": "landing_pad",
    "verbose": true,
    "num_moving_drones": 3,
    "max_steps": 600,
    "use_priority": true,
    "use_orbit": true,
    "use_negotiation": true,
    "use_trajectory_planner": true,
    "n_trips": 1,
    "unload_steps": 5,
    "max_speed": 1.0,
    "min_separation": 1.0,
    "safe_distance": 1.2,
    "nominal_speed": 0.1,
    "drones": [
        { "start": [ 1.0,  1.0], "goal": [0.0, 0.0],
          "cargo_type": "medication", "time_to_expiry": 50.0,  "patient_acuity": "urgent" },
        { "start": [-1.5,  1.5], "goal": [0.0, 0.0],
          "cargo_type": "organ",      "time_to_expiry": 200.0, "patient_acuity": "critical" },
        { "start": [-3.0, -2.0], "goal": [0.0, 0.0],
          "cargo_type": "equipment",  "time_to_expiry": 300.0, "patient_acuity": "routine" }
    ]
}
```

### 5.1 Planner-specific fields

| Field                      | Purpose                                                  |
|----------------------------|----------------------------------------------------------|
| `use_trajectory_planner`   | Must be `true` to activate `TrajectoryPlannerController` |
| `max_speed`                | Hard speed ceiling for any drone                         |
| `min_separation`           | Minimum spatial gap floor between consecutive arrivals   |
| `unload_steps`             | Pad occupancy time after a landing                       |
| `safe_distance`            | Pad-busy safety-net threshold                            |
| `nominal_speed`            | Used by base classes (kept for compatibility)            |
| `n_trips`                  | `1` for one-way, `>=2` for round-trip                    |
| `use_priority`             | Required so cargo metadata is loaded into the drones     |

`use_orbit` and `use_negotiation` are accepted but the planner does not
use orbits or ETA-switching — the base class plumbing only needs them
to construct cleanly. They can stay `true` without affecting behaviour.

### 5.2 Per-drone fields

| Field                      | Purpose                                                  |
|----------------------------|----------------------------------------------------------|
| `start`                    | Initial 2D position                                      |
| `goal`                     | Pad position (`[0, 0]`)                                  |
| `return_point`             | Round-trip only: home pad to fly back to                 |
| `cargo_type`               | `organ` / `blood_product` / `medication` / `equipment`   |
| `time_to_expiry`           | Steps before cargo expires (used by priority score)      |
| `patient_acuity`           | `critical` / `urgent` / `routine`                        |

### 5.3 Scenario-level fields

| Field                       | Purpose                                                  |
|-----------------------------|----------------------------------------------------------|
| `test_name`                 | Used for the output GIF filename                         |
| `env_type`                  | Must be `landing_pad` for this track                     |
| `num_moving_drones`         | Number of moving drones                                  |
| `max_steps`                 | Episode length cap                                       |
| `min_radius`, `epsilon`, `step_size`, `k_value`, `wall_collision_multiplier` | Underlying MPC/environment parameters                    |
| `drones`                    | Per-drone settings (see 5.2)                             |

---

## 6. Relation to other track

- This branch implements **Track 2 only** (trajectory planner family).
- Yield/orbit/negotiation work (single-winner discrete coordination)
  lives in `research/track-policy-yield`.
- LLM extensions for either track go on dedicated child branches; see
  `LLM_BRANCH_BOOTSTRAP.md` (in `dev/jinyoon` / Track 1 history) for the
  shared bootstrap notes.
