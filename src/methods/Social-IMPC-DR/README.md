# Social-IMPC-DR — Track 1 (Policy/Yield)

Track 1 focuses on **single-winner landing-pad control** for medical drone
coordination. Drones share one central pad; one inbound drone gets pad access
while the others yield according to a chosen policy.

This branch intentionally excludes planner-first framing. Planner work lives in
`research/track-trajectory-planner`.

---

## Architecture: flat plugin model

Track 1 is built around a single `PolicyYieldController` that *composes* peer
policy components. There is no inheritance chain — each component is selected
independently in the scenario JSON.

```
PolicyYieldController
├── selector     (closest_first | priority)
├── yielder      (freeze | orbit)
├── lifecycle    (one_way   | round_trip)
├── negotiators  ([]  |  [expiry_guard, eta_switch, ...])   # zero or more peers
└── use_hysteresis: bool                                     # optional toggle
```

Each role lives in its own module and is registered by name:

| Role            | Module                          | Implementations                   |
|-----------------|---------------------------------|-----------------------------------|
| Selector        | `policy_yield/selectors.py`     | `closest_first`, `priority`       |
| Yielder         | `policy_yield/yielders.py`      | `freeze`, `orbit`                 |
| Lifecycle       | `policy_yield/lifecycles.py`    | `one_way`, `round_trip`           |
| Negotiator      | `policy_yield/negotiators.py`   | `expiry_guard`, `eta_switch`      |
| Orchestrator    | `policy_yield/controller.py`    | `PolicyYieldController`           |
| Recipe builder  | `policy_yield/registry.py`      | `build_policy_yield_controller`   |

Adding a new policy is a peer change — implement the role's interface,
register the name in `policy_yield/registry.py`, and reference it from any
scenario. No existing class needs to be subclassed or modified.

---

## Quick Start

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib
```

Run a policy/yield scenario (configs are named by behaviour, not phase):

```bash
python app2_standardized.py landing_pad configs/track_policy_baseline_closest.json
python app2_standardized.py landing_pad configs/track_policy_priority.json
python app2_standardized.py landing_pad configs/track_policy_orbit_hold.json
python app2_standardized.py landing_pad configs/track_policy_negotiation.json
python app2_standardized.py landing_pad configs/track_policy_round_trip.json
```

Focused negotiation checks:

```bash
python app2_standardized.py landing_pad configs/track_policy_negotiation_expiry_guard.json
python app2_standardized.py landing_pad configs/track_policy_negotiation_eta_switch.json
python app2_standardized.py landing_pad configs/track_policy_negotiation_no_hysteresis.json
```

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Scenario Configuration

Each scenario JSON has a `policy` block selecting one component per role
plus a list of negotiator peers. Example (`track_policy_round_trip.json`):

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
    "drones": [ ... ]
}
```

### Recipe fields

| Key               | Purpose                                                          |
|-------------------|------------------------------------------------------------------|
| `selector`        | `closest_first` (distance-first) or `priority` (priority score)  |
| `yielder`         | `freeze` (zero velocity) or `orbit` (circular hold)              |
| `lifecycle`       | `one_way` (land once) or `round_trip` (FSM with unload + return) |
| `negotiators`     | List of peer overrides applied in order                          |
| `use_hysteresis`  | If true, the current winner stays winner until they finish       |
| `orbit_*`         | Radius / angular speed / safe distance for the orbit yielder     |
| `nominal_speed`   | Used to compute ETA in negotiators                               |
| `eta_threshold`   | Score-gap threshold below which `eta_switch` fires               |
| `n_trips`         | Round-trip lifecycle: number of round trips before `DONE`        |
| `unload_steps`    | Round-trip lifecycle: pad-occupancy time per landing             |

### Scenario reference

| Config file                                          | Recipe summary                                                              |
|------------------------------------------------------|-----------------------------------------------------------------------------|
| `track_policy_baseline_closest.json`                 | `closest_first` + `freeze` + `one_way`                                      |
| `track_policy_priority.json`                         | `priority` + `freeze` + `one_way` + hysteresis                              |
| `track_policy_orbit_hold.json`                       | `priority` + `orbit` + `one_way` + hysteresis                               |
| `track_policy_negotiation.json`                      | `priority` + `orbit` + `one_way` + `[expiry_guard, eta_switch]` + hysteresis|
| `track_policy_negotiation_no_hysteresis.json`        | Same as above with `use_hysteresis: false`                                  |
| `track_policy_negotiation_expiry_guard.json`         | Negotiation stack tuned to stress the expiry-guard rule                     |
| `track_policy_negotiation_eta_switch.json`           | Negotiation stack tuned to stress the ETA-switch rule                       |
| `track_policy_round_trip.json`                       | Full negotiation stack + `round_trip` lifecycle                             |

---

## File Layout

```
Social-IMPC-DR/
├── policy_yield/                # flat plugin package
│   ├── controller.py            # PolicyYieldController orchestrator
│   ├── registry.py              # recipe -> controller builder
│   ├── selectors.py             # closest_first, priority
│   ├── yielders.py              # freeze, orbit
│   ├── lifecycles.py            # one_way, round_trip
│   ├── negotiators.py           # expiry_guard, eta_switch
│   └── context.py               # per-step Context + PAD_CENTER
├── app2_standardized.py         # entry point (config or interactive)
├── test.py                      # simulation loop, calls into the controller
├── priority.py                  # weighted medical priority score
├── configs/track_policy_*.json  # behaviour-named scenarios
└── ...                          # MPC core (run.py, avoid.py, uav.py, SET.py, ...)
```

The MPC core (`run.py`, `avoid.py`, `uav.py`, etc.) is unchanged from upstream
Social-IMPC-DR.

---

## Relation To Other Track

- This branch is **Track 1 only** (policy/yield family).
- Planner-first scheduling work (simultaneous arrival speed planning) lives in
  `research/track-trajectory-planner`.
- LLM extensions for either track are developed on dedicated child branches
  (see `LLM_BRANCH_BOOTSTRAP.md`).

---

## Environment Setup

```bash
conda create -n smglib python=3.10
conda activate smglib
pip install -r requirements.txt
```
