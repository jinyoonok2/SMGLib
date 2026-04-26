# Social-IMPC-DR — Track 1 (Policy/Yield)

Track 1 focuses on **single-winner landing-pad control** for medical drone
coordination. Drones share one central pad; one inbound drone gets pad access,
and others yield according to policy rules.

This branch intentionally excludes planner-first framing. Planner work is
developed in `research/track-trajectory-planner`.

---

## Scope Of This Track

This branch covers the policy/yield family:

- closest-first baseline (`LandingPadController`)
- priority scoring (`PriorityManager`)
- orbit-based holding for yielding drones (`OrbitController`)
- ETA/expiry negotiation (`NegotiationController`)
- round-trip lifecycle with inbound/unloading/outbound FSM (`RoundTripController`)

### Included Controllers

```
LandingPadController
  -> PriorityManager
      -> OrbitController
          -> NegotiationController
              -> RoundTripController
```

---

## Quick Start

```bash
cd src/methods/Social-IMPC-DR
conda activate smglib
```

Run a policy/yield scenario (preferred **track** configs in `configs/`):

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

Legacy `phase*.json` files in `configs/` are kept as duplicates of the same
scenarios for backward compatibility; prefer `track_policy_*.json` for new work.

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Configuration

| Config file | What it exercises |
|---|---|
| `track_policy_baseline_closest.json` | Closest-first, no cargo priority |
| `track_policy_priority.json` | Priority scoring only |
| `track_policy_orbit_hold.json` | Priority + orbit holding while yielding |
| `track_policy_negotiation.json` | Priority + orbit + ETA/expiry negotiation |
| `track_policy_negotiation_no_hysteresis.json` | Same as negotiation demo, `use_hysteresis: false` |
| `track_policy_negotiation_expiry_guard.json` | Stresses expiry-guard behavior |
| `track_policy_negotiation_eta_switch.json` | Stresses ETA-switch behavior |
| `track_policy_round_trip.json` | Full stack + round-trip FSM |

Core JSON fields:

- `use_priority`: enable priority scoring instead of closest-first
- `use_orbit`: move yielding drones in circular holding orbits
- `use_negotiation`: enable ETA Switch + Expiry Guard
- `use_hysteresis`: keep winner stable until landing
- `round_trip`: enable inbound/unloading/outbound trip lifecycle
- `n_trips`, `unload_steps`: round-trip behavior controls

---

## Architecture

- `policy_yield_factory.py`: **single wiring point** — builds the right
  policy/yield controller from scenario flags (baseline, priority, orbit,
  negotiation, round-trip)
- `app2_standardized.py`: entrypoint (scenario config or interactive mode)
- `test.py`: simulation loop and controller dispatch
- `priority.py`: weighted medical-priority scoring
- `landing_pad.py`: baseline winner/yield control
- `priority_manager.py`: priority-based selection and expiry stepping
- `orbit_controller.py`: orbit holding for yielding drones
- `negotiation_controller.py`: Expiry Guard + ETA Switch
- `round_trip_controller.py`: trip-state machine and target swapping

The MPC core and low-level dynamics from original Social-IMPC-DR remain in
`run.py`, `avoid.py`, and UAV model files.

---

## Relation To Other Track

- This branch is **Track 1 only** (policy/yield family).
- Planner-first scheduling work (simultaneous arrival speed planning) lives in
  `research/track-trajectory-planner`.

---

## Environment Setup

```bash
conda create -n smglib python=3.10
conda activate smglib
pip install -r requirements.txt
```
