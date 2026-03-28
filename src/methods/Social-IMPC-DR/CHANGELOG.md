# Changelog — Social-IMPC-DR (Landing Pad Extension)

All notable changes to the Social-IMPC-DR landing pad coordination module.
For project overview, architecture, and usage instructions, see [README.md](README.md).

---

## [Phase 2] — Dynamic Priority Integration

### Added
- **`priority.py`** — Standalone scoring module
  - `priority_score()` → weighted float in [0, 1]
  - `rank_drones()` → sorted list by descending score
  - Weights: cargo type 35%, time to expiry 30%, distance 15%, patient acuity 20%
- **`priority_manager.py`** — `PriorityManager(LandingPadController)`
  - Overrides `select_active_drone()` to pick highest-priority drone instead of closest
  - Overrides `step_update()` to decrement `time_to_expiry` each step (urgency increases over time)
  - Falls back to distance-based selection if no cargo config is present
- **`test_phase2.py`** — Standalone 3-drone test script (no animation, quick verification)
- **`scenarios/phase2_landing_pad.json`** — 3-drone scenario config with mixed cargo
  - Drone 0: organ / critical / 60s expiry
  - Drone 1: equipment / routine / 200s expiry
  - Drone 2: medication / urgent / 150s expiry

### Changed
- **`uav.py`** — Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes
  (backward-compatible; defaults to `None` if not set)
- **`test.py`** — Controller instantiation now checks `use_priority` flag
  to choose between `LandingPadController` and `PriorityManager`
- **`app2_standardized.py`** — Accepts optional scenario config JSON as CLI argument;
  reads drone cargo attributes from config and passes them to `PLAN()`

### Results
- 3 drones, mixed cargo, 1 pad
- Drone 0 (organ/critical) lands at step 36 — highest priority
- Drone 2 (medication/urgent) lands at step 67
- Drone 1 (equipment/routine) lands at step 103
- 100% success rate, correct priority ordering verified

---

## [Phase 1] — Baseline Bottleneck Simulation

### Added
- **`landing_pad.py`** — `LandingPadController` class with mutual exclusion logic
  - `select_active_drone()` — picks closest active drone; others yield
  - `freeze_yielding()` — zeros velocity for waiting drones
  - `cleanup_landed()` — removes landed drones from the simulation area
  - `reset_mpc()` — warm-start reset when drone transitions yielding → active
  - `update_idle_positions()` — keeps animation position arrays aligned
  - `step_update()` — per-step hook (no-op in Phase 1; extensible)
  - `negotiation_hook()` — pre-selection hook (no-op; override point for Phase 4)
- **`scenarios/phase1_landing_pad.json`** — 2-drone scenario config, no priority
- **`src/utils.py`** — Added `landing_pad` environment type with shared goal at
  `[0,0]`, wall obstacle layout, and pad visualization marker

### Changed
- **`run.py`** — Replaced hardcoded `if 2 <= index <= 21` with
  `SET.num_moving_drones` so drone indices beyond 2 can actually move
- **`avoid.py`** — Added `wall_collision_multiplier` and `env_type` parameters;
  stationary robot detection for larger wall safety margins
- **`SET.py`** — Added `ENV_TYPE` and `num_moving_drones` global parameters
- **`test.py`** — Refactored simulation loop to delegate to controller classes;
  accepts `num_moving_drones`, `env_type` parameters; early exit when all drones land
- **`app2_standardized.py`** — Added `landing_pad` scenario branch and animation legend fix

### Fixed
- Both drones stuck (never reaching pad) — 4× `r_min` inflation in `avoid.py`
  made MPC infeasible → removed inflation, use turn-based yielding instead
- Drone 1 blocked by landed Drone 0 at (0,0) — no mechanism to clear landed
  drones → teleport landed drones to (100, 100)
- Drone 1 oscillates ~0.2 from goal — stale MPC warm-start after long hold →
  skip MPC for yielding drones, reset warm-start on release
- Animation shows both drones moving simultaneously — skipped drones had shorter
  position arrays → manually append position for idle drones each step

### Results
- 2 drones, 1 pad
- Drone 0 lands at step 51, Drone 1 lands at step 102
- 100% success rate

---

## [Config Evolution] — Scenario Config System

### Added
- **`scenarios/` directory** — JSON config files that define complete simulation runs
  (environment type, simulation parameters, drone positions, and cargo attributes)

### Removed
- **`cargo_configs.json`** — Replaced by per-scenario configs in `scenarios/`
- **`docs/PROJECT_GUIDELINE.md`** — Content consolidated into README.md

### Changed
- **`app2_standardized.py`** — Now supports two modes:
  1. **Config-file mode:** `python app2_standardized.py landing_pad scenarios/phase2_landing_pad.json`
  2. **Interactive mode:** `python app2_standardized.py landing_pad` (manual parameter entry)
- Config data flows through function arguments (not global file reads) —
  `app2_standardized.py` loads the JSON, `test.py` receives it via `PLAN()` args

---

## Files Summary

### New files
| File | Phase | Purpose |
|---|---|---|
| `landing_pad.py` | 1 | Mutual exclusion controller — yielding, cleanup, MPC reset |
| `priority.py` | 2 | Scoring math — `priority_score()`, `rank_drones()` |
| `priority_manager.py` | 2 | Priority-based controller — extends `LandingPadController` |
| `test_phase2.py` | 2 | Standalone Phase 2 test (no animation) |
| `scenarios/phase1_landing_pad.json` | 1 | 2-drone closest-first scenario config |
| `scenarios/phase2_landing_pad.json` | 2 | 3-drone priority-based scenario config |

### Modified files
| File | What changed |
|---|---|
| `app2_standardized.py` | Config-file mode, landing pad scenario branch, cargo attribute passthrough |
| `test.py` | Controller delegation, accepts cargo/env/drone-count params, early exit |
| `uav.py` | Added `cargo_type`, `time_to_expiry`, `patient_acuity` attributes |
| `run.py` | Fixed hardcoded drone index threshold |
| `avoid.py` | Wall collision multiplier, env_type support |
| `SET.py` | Added `ENV_TYPE`, `num_moving_drones` globals |
| `src/utils.py` | Added `landing_pad` environment type |

### Deleted files
| File | Reason |
|---|---|
| `cargo_configs.json` | Replaced by scenario configs in `scenarios/` |
| `docs/PROJECT_GUIDELINE.md` | Content moved to README.md |

### Unchanged files
`run.py` (MPC solver core), `others.py`, `dynamic.py`, `plot.py`, `plot_standardized.py`

---

## Extension Points for Future Phases

Two hooks exist in the controller hierarchy for Phases 3–5.
Existing behavior is unchanged — the hooks are no-ops until overridden.

- **`negotiation_hook()`** — called before `select_active_drone()`. Return
  `None` for normal behavior, or a result dict to override. (Phase 4 target)
- **`step_update()`** — called each simulation step. Phase 2 uses it to
  decrement `time_to_expiry`. (Extensible for Phase 3+)

`select_active_drone()` returns a metadata dict for logging/comparison:

```python
{
    "allowed":  int | None,      # drone that may proceed
    "yielding": set[int],        # drones that must hold
    "method":   str,             # "distance" | "priority" | custom
    "scores":   dict[int, float] # per-drone scores
}
```
