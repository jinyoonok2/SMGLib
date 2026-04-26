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

Run a policy/yield scenario:

```bash
python app2_standardized.py landing_pad configs/phase1_landing_pad.json
python app2_standardized.py landing_pad configs/phase2_landing_pad.json
python app2_standardized.py landing_pad configs/phase3_orbit.json
python app2_standardized.py landing_pad configs/phase4_negotiation.json
python app2_standardized.py landing_pad configs/phase6_round_trip.json
```

Rule-isolation checks:

```bash
python app2_standardized.py landing_pad configs/phase4_expiry_guard_test.json
python app2_standardized.py landing_pad configs/phase4_eta_switch_test.json
python app2_standardized.py landing_pad configs/phase4_negotiation_no_hysteresis.json
```

Animations are saved to `logs/Social-IMPC-DR/animations/`.

---

## Configuration

Scenario configs are currently legacy `phase*.json` names for compatibility.
They are treated as Track 1 scenarios in this branch.

Core fields:

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
