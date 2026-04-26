# Social-IMPC-DR — Track 2 (Trajectory Planner)

Track 2 focuses on **planner-based simultaneous inbound flight** for a shared
medical landing pad. Instead of selecting one winner and forcing others to
yield, this track computes per-drone cruise speed targets so arrivals are
ordered and spacing constraints are respected.

This branch intentionally centers planner behavior. Policy/yield-first work is
maintained in `research/track-policy-yield`.

---

## Scope Of This Track

This branch emphasizes the planner family:

- trajectory-planner scheduling (`TrajectoryPlannerController`)
- speed-scaling arrivals with constraints:
  - `max_speed`
  - `min_separation`
  - `unload_steps`
- replan on inbound-set changes
- pad-busy safety net for near-pad drift while unloading
- optional round-trip operation (`n_trips >= 2`)

### Planner Controller Line

```
RoundTripController (lifecycle FSM reused)
  -> TrajectoryPlannerController
```

---

## Quick Start

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib
```

Run planner scenarios:

```bash
python app2_standardized.py landing_pad configs/phase7_planner_oneway.json
python app2_standardized.py landing_pad configs/phase7_planner_round_trip.json
```

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Configuration

Planner scenarios currently use legacy `phase7_*.json` names for compatibility.
In this branch, these represent Track 2 scenarios.

Required planner controls:

- `use_trajectory_planner: true`
- `max_speed`: speed ceiling assigned to any drone
- `min_separation`: minimum scheduling gap floor between arrivals
- `unload_steps`: pad occupancy duration after landing
- `safe_distance`: near-pad safety-net threshold while pad is busy
- `n_trips`: `1` for one-way, `>=2` for round-trip planner runs

Shared fields (still used):

- `use_priority`, cargo fields (`cargo_type`, `time_to_expiry`, `patient_acuity`)
- MPC/environment fields (`min_radius`, `step_size`, `k_value`, etc.)

---

## Architecture

- `trajectory_planner_controller.py`: planner scheduling and per-drone `Vmax`
- `app2_standardized.py`: scenario parsing and planner flag dispatch
- `test.py`: controller selection and simulation loop
- `round_trip_controller.py`: reused lifecycle FSM for one-way/round-trip modes
- `priority.py`: score computation used for planner ranking order

Low-level MPC and collision-avoidance core remain compatible with original
Social-IMPC-DR internals.

---

## Formula Summary

For ranked drone `k` with distance `d_k` to pad:

- physical floor: `T >= d_k / max_speed`
- pad-clearance floor: `T >= T_prev + unload_steps`
- separation floor: `T >= T_prev + min_separation / v_prev`

Assigned speed:

- `v_k = min(d_k / T_k, max_speed)`

This allows simultaneous inbound motion while preserving order and spacing.

---

## Relation To Other Track

- This branch is **Track 2 only** (trajectory planner family).
- Yield/orbit/negotiation-first policy evolution is maintained in
  `research/track-policy-yield`.

---

## Environment Setup

```bash
conda create -n smglib python=3.10
conda activate smglib
pip install -r requirements.txt
```
