# Changelog — Social-IMPC-DR (Landing Pad Extension)

All notable changes from the original Social-IMPC-DR codebase.
For project overview, architecture, results, and usage, see [README.md](README.md).

---

## [Phase 4] — ETA-Aware Negotiation with Expiry Enforcement

### Added
- **`negotiation_controller.py`** — `NegotiationController(OrbitController)`
  - Overrides `negotiation_hook()` with two-rule logic: Expiry Guard and ETA Switch
  - Expiry Guard: if `time_to_expiry < distance / nominal_speed`, strict priority wins
  - ETA Switch: if score gap < `eta_threshold`, closer drone wins
  - Returns `None` if neither rule fires (falls through to normal priority)
- **`configs/phase4_expiry_guard_test.json`** — 2-drone scenario triggering Expiry Guard
- **`configs/phase4_eta_switch_test.json`** — 2-drone scenario triggering ETA Switch

### Changed
- **`test.py`** — Added `negotiation_params` kwarg to `PLAN()`; controller selection
  picks `NegotiationController` when `negotiation_params` is provided
- **`app2_standardized.py`** — Reads `use_negotiation`, `nominal_speed`, `eta_threshold`
  from scenario JSON; builds `negotiation_params` dict and passes to `PLAN()`
  — GIF filename uses `test_name` field from JSON if present

### Changed (all scenario configs)
- Added `test_name` field to all configs so output GIF filename matches scenario name

---

## [Phase 3] — Holding Orbit Patterns

### Added
- **`orbit_controller.py`** — `OrbitController(PriorityManager)`
  - Overrides `freeze_yielding()` to move waiting drones in a circular orbit
  - `_orbit_state` dict: maps drone index → `{center, angle}`
  - Each step: `angle += orbit_speed`; position and velocity set analytically
  - Safe-distance repulsion: orbit center pushed outward if active drone approaches
  - `pre_traj` filled with predicted orbit positions for active drone's MBVC constraints
- **`configs/phase3_orbit.json`** — 3-drone scenario with orbit params
- **`configs/phase2_leapfrog.json`** — same 3-drone scenario without orbit, for comparison

### Changed
- **`test.py`** — Added `orbit_params` kwarg to `PLAN()`; controller selection
  picks `OrbitController` when both `cargo_configs` and `orbit_params` are provided
- **`app2_standardized.py`** — Reads `use_orbit`, `orbit_radius`, `orbit_speed`,
  `safe_distance` from scenario JSON; builds `orbit_params` dict and passes to `PLAN()`

---

## [Phase 2] — Dynamic Priority Integration

### Added
- **`priority.py`** — Standalone scoring module
  - `priority_score()` → weighted float in [0, 1]
  - `rank_drones()` → sorted list by descending score
  - Weights: cargo 35%, expiry 30%, distance 15%, acuity 20%
- **`priority_manager.py`** — `PriorityManager(LandingPadController)`
  - Overrides `select_active_drone()` to pick highest-priority drone
  - Overrides `step_update()` to decrement `time_to_expiry` each step
  - Falls back to distance-based selection if no cargo config is present
- **`test_phase2.py`** — Standalone 3-drone test script (no animation)
- **`configs/phase2_landing_pad.json`** — 3-drone scenario config with mixed cargo

### Changed
- **`uav.py`** — Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes
  (backward-compatible; defaults to `None` if not set)
- **`test.py`** — Controller instantiation checks `cargo_configs` to choose between
  `LandingPadController` and `PriorityManager`
- **`app2_standardized.py`** — Reads drone cargo attributes from scenario JSON;
  passes them to `PLAN()` as `cargo_configs`

---

## [Phase 1] — Baseline Bottleneck Simulation

### Added
- **`landing_pad.py`** — `LandingPadController` base class
  - `select_active_drone()` — picks closest active drone; others yield
  - `freeze_yielding()` — zeros velocity for waiting drones
  - `cleanup_landed()` — teleports landed drones off-screen to unblock pad
  - `reset_mpc()` — warm-start reset when drone transitions yielding → active
  - `update_idle_positions()` — keeps animation position arrays aligned
  - `step_update()` — per-step hook (no-op; extensible)
  - `negotiation_hook()` — pre-selection hook (no-op; Phase 4 override point)
- **`configs/phase1_landing_pad.json`** — 2-drone scenario config, no priority
- **`configs/priority_config.json`** — Priority scoring weights and cargo/acuity tables
- **`src/utils.py`** — Added `landing_pad` environment type with shared goal,
  wall obstacle layout, and pad visualization marker

### Changed
- **`run.py`** — Replaced hardcoded `if 2 <= index <= 21` with `SET.num_moving_drones`
- **`avoid.py`** — Added `wall_collision_multiplier` and `env_type` parameters
- **`SET.py`** — Added `ENV_TYPE` and `num_moving_drones` global parameters
- **`test.py`** — Refactored simulation loop to delegate to controller classes;
  added early exit when all drones land
- **`app2_standardized.py`** — Added `landing_pad` scenario branch;
  added config-file mode (`python app2_standardized.py landing_pad configs/X.json`)

### Fixed
- MPC infeasibility from `4× r_min` inflation in `avoid.py` → removed inflation,
  use turn-based yielding instead
- Drone blocked by landed drone at pad → `cleanup_landed()` teleports to (100, 100)
- Drone oscillating near goal after long hold → `reset_mpc()` on yield release
- Animation desync from unequal position array lengths → `update_idle_positions()`

---

## Files Summary

### New files
| File | Phase | Purpose |
|---|---|---|
| `landing_pad.py` | 1 | Base controller — mutual exclusion, yielding, MPC reset |
| `priority.py` | 2 | Scoring math — `priority_score()`, `rank_drones()` |
| `priority_manager.py` | 2 | Priority-based controller — extends `LandingPadController` |
| `test_phase2.py` | 2 | Standalone Phase 2 test (no animation) |
| `orbit_controller.py` | 3 | Orbit holding controller — extends `PriorityManager` |
| `negotiation_controller.py` | 4 | Negotiation controller — extends `OrbitController` |
| `configs/phase1_landing_pad.json` | 1 | 2-drone closest-first scenario |
| `configs/phase2_landing_pad.json` | 2 | 3-drone priority-based scenario |
| `configs/phase2_leapfrog.json` | 2/3 | 3-drone priority scenario, orbit comparison baseline |
| `configs/phase3_orbit.json` | 3 | 3-drone orbit holding scenario |
| `configs/phase4_expiry_guard_test.json` | 4 | 2-drone Expiry Guard test scenario |
| `configs/phase4_eta_switch_test.json` | 4 | 2-drone ETA Switch test scenario |
| `configs/priority_config.json` | 2 | Priority scoring weights, cargo weights, acuity scores |

### Modified files
| File | What changed |
|---|---|
| `app2_standardized.py` | Config-file mode, landing pad branch, cargo/orbit/negotiation passthrough, GIF naming |
| `test.py` | Controller delegation, cargo/orbit/negotiation params, early exit |
| `uav.py` | Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes |
| `run.py` | Fixed hardcoded drone index threshold |
| `avoid.py` | Wall collision multiplier, env_type support |
| `SET.py` | Added `ENV_TYPE`, `num_moving_drones` globals |
| `src/utils.py` | Added `landing_pad` environment type |

### Deleted files
| File | Reason |
|---|---|
| `cargo_configs.json` | Replaced by per-scenario configs in `configs/` |
| `docs/PROJECT_GUIDELINE.md` | Content consolidated into README.md |

### Unchanged files
`others.py`, `dynamic.py`, `plot.py`, `plot_standardized.py`