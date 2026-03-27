# Changelog — Social-IMPC-DR (Landing Pad Extension)

---

## System Overview

### What this project does

A fleet of hospital drones carrying medical cargo must land on a **single
shared landing pad**.  Only one drone can land at a time, creating a
bottleneck.  A **policy** decides which drone goes next — Phase 1 picks the
closest, Phase 2 picks the highest-priority cargo.

### Full pipeline

```
app2_standardized.py          ← TOP-LEVEL ENTRY POINT
  │
  ├── 1. User picks scenario, drone count, cargo config
  │
  ├── 2. Calls PLAN() ────────► test.py (simulation engine)
  │                               │
  │                               ├── Creates drones ─────► uav.py
  │                               │                           Data object: position,
  │                               │                           velocity, cargo type,
  │                               │                           expiry, acuity.
  │                               │
  │                               ├── Creates policy ─────► LandingPadController
  │                               │       │                  or PriorityManager
  │                               │       │                  Decides WHO flies
  │                               │       │                  and WHO waits.
  │                               │       │
  │                               │       └── Uses ──────► priority.py
  │                               │                         Pure math: cargo +
  │                               │                         expiry + distance +
  │                               │                         acuity → score [0,1].
  │                               │
  │                               └── Runs physics ──────► run.py (MPC solver)
  │                                                         Moves drones toward
  │                                                         goal. Unchanged from
  │                                                         original codebase.
  │
  │         PLAN() returns agent_list with full position history
  │
  └── 3. Calls generate_animation_standardized()
          └── Reads position history → renders frames → saves .html/.gif/.avi
```

### One simulation step

```
Step i begins
  │
  ├─ 1. controller.cleanup_landed()       Remove landed drones from the pad
  ├─ 2. controller.select_active_drone()   Pick ONE drone to fly
  ├─ 3. controller.freeze_yielding()       Zero velocity for all others
  ├─ 4. run_one_step(active_drone)         MPC solver moves the chosen drone
  ├─ 5. controller.update_idle_positions() Keep animation aligned for frozen drones
  ├─ 6. controller.step_update()           Phase 2: decrement expiry timers
  └─ 7. Check: did the active drone land?  If all landed → simulation ends
```

### Policy class hierarchy

```
LandingPadController              ← Phase 1: closest drone goes first
    │
    └── PriorityManager           ← Phase 2: highest priority score goes first
            │
            └── (Future phases)   ← Override hooks to add new behavior
```

Each subclass only overrides the methods it changes.
`test.py` calls the same interface regardless of which controller is active.

---

## File Reference

### Original files (pre-existing, from Social-IMPC-DR)

| File | Original role | What we changed |
|---|---|---|
| `uav.py` | Drone agent class — position, velocity, dynamics, MPC state | **Added:** `cargo_type`, `time_to_expiry`, `patient_acuity` attributes (backward-compatible defaults) |
| `run.py` | Runs MPC solver per drone each step | **Fixed:** replaced hardcoded `if 2 <= index <= 21` with `SET.num_moving_drones` so drone 2+ can actually move |
| `avoid.py` | Builds pairwise collision avoidance constraints | **Added:** `wall_collision_multiplier` and `env_type` params; stationary robot detection for larger wall margins |
| `SET.py` | Global simulation parameters | **Added:** `ENV_TYPE`, `num_moving_drones` parameters |
| `test.py` | Main simulation loop | **Refactored:** delegates to controller classes; accepts `cargo_configs`, `num_moving_drones`, `env_type`; early exit when all drones land |
| `app2_standardized.py` | Interactive UI → run simulation → generate animation | **Added:** `landing_pad` scenario branch, cargo priority config prompt, legend fix |
| `src/utils.py` | Environment configs, positions, obstacles | **Added:** `landing_pad` environment type, shared goal at `[0,0]`, wall layout, pad visualization marker |
| `others.py` | Helper functions for data collection | No changes |
| `dynamic.py` | Dynamics utilities | No changes |
| `plot.py` / `plot_standardized.py` | Visualization and animation rendering | No changes |

### New files (created by us)

| File | Phase | Purpose |
|---|---|---|
| `landing_pad.py` | 1 | `LandingPadController` class — mutual exclusion, yielding, cleanup, MPC reset |
| `priority.py` | 2 | `priority_score()` and `rank_drones()` — standalone scoring math |
| `priority_manager.py` | 2 | `PriorityManager(LandingPadController)` — priority-based policy, expiry countdown |
| `test_phase2.py` | 2 | Standalone 3-drone test script (no animation, quick verification) |
| `docs/PROJECT_GUIDELINE.md` | 1 | Full project description with 5-phase roadmap |

---

## Phase 1 — Baseline Bottleneck Simulation

Introduces a **single landing pad** that all drones share, with a **mutual
exclusion constraint** — only one drone may approach the pad at a time.

### `LandingPadController` methods

| Method | What it does |
|---|---|
| `select_active_drone()` | Picks the **closest** active drone to proceed; all others yield |
| `freeze_yielding()` | Zeros velocity for yielding drones (no MPC call) |
| `cleanup_landed()` | Teleports landed drones to (100, 100) so they don't block the pad |
| `reset_mpc()` | Warm-start reset when a drone transitions from yielding → active |
| `update_idle_positions()` | Appends position history for drones that skipped MPC (animation alignment) |
| `step_update()` | Per-step hook (no-op in Phase 1; overridden in Phase 2) |
| `negotiation_hook()` | Pre-selection hook (no-op; override in Phase 4 for arbitration) |

### Bugs fixed during Phase 1

| # | Symptom | Root cause | Fix |
|---|---------|-----------|-----|
| 1 | Both drones stuck, never reaching pad | 4× `r_min` inflation in `avoid.py` made MPC infeasible | Removed inflation; use turn-based yielding instead |
| 2 | Drone 1 blocked by landed Drone 0 at (0,0) | No mechanism to clear landed drones | Teleport landed drones to (100, 100) |
| 3 | Drone 1 oscillates ~0.2 from goal | Stale MPC warm-start after long hold | Skip MPC for yielding drones; reset warm-start on release |
| 4 | Animation shows both drones moving simultaneously | Skipped drones had shorter position arrays | Manually append position for skipped drones |

### Phase 1 results
- 2 drones, 1 landing pad
- Drone 0 lands at step 51, Drone 1 lands at step 102
- 100% success rate

---

## Phase 2 — Dynamic Priority Integration

Replaces Phase 1's "closest drone first" policy with a **priority scoring
system** based on medical cargo criticality.  A farther drone carrying a
critical organ now lands before a closer drone carrying routine equipment.

### `priority.py` — scoring math

| Component | Values |
|---|---|
| Cargo weights | organ (1.0), blood_product (0.7), medication (0.4), equipment (0.1) |
| Acuity scores | critical (1.0), urgent (0.6), routine (0.2) |
| Factor weights | cargo 35%, expiry 30%, distance 15%, acuity 20% |
| Functions | `priority_score()` → float [0, 1]; `rank_drones()` → sorted list |

### `PriorityManager` — overrides from `LandingPadController`

| Override | What it does |
|---|---|
| `select_active_drone()` | Highest `priority_score()` proceeds instead of closest drone. Falls back to distance-based if no cargo config. |
| `step_update()` | Decrements `time_to_expiry` by 1 each step, making urgency increase over time |

### Phase 2 results
- 3 drones, mixed cargo, 1 landing pad
- Drone 0 (organ/critical) lands at step 36 — highest priority
- Drone 2 (medication/urgent) lands at step 67
- Drone 1 (equipment/routine) lands at step 103
- 100% success rate, priority ordering correct

---

## Integration Prep — Hooks for Future Phases

Two forward-looking changes to smooth integration for Phases 3–5.
Existing behavior is unchanged — the hooks are no-ops until overridden.

### `negotiation_hook()` — pre-selection override point

Called before `select_active_drone()` each step.  Return `None` to proceed
normally, or return a result dict to override the decision entirely.
Phase 4 overrides this for LLM-based arbitration.

### `select_active_drone()` — metadata dict return format

```python
{
    "allowed":  int | None,      # drone that may proceed
    "yielding": set[int],        # drones that must hold
    "method":   str,             # "distance" | "priority" | custom
    "scores":   dict[int, float] # per-drone score for logging/comparison
}
```

---

## Integration Guide for Future Phases

All future phases extend the controller hierarchy — subclass and override
hooks.  No changes to `test.py` or the MPC solver needed.

| Phase | Subclass | Override | What it adds |
|---|---|---|---|
| Phase 3 — Holding orbits | `OrbitController(PriorityManager)` | `freeze_yielding()` | Yielding drones fly circular orbits instead of freezing |
| Phase 4 — LLM negotiation | `NegotiationController(PriorityManager)` | `negotiation_hook()` | Pairwise drone arbitration via LLM; compare against rule-based scores |
| Phase 5 — Lookahead scheduling | `SchedulerController(PriorityManager)` | `select_active_drone()` | Plan full landing queue using `rank_drones()` + time-horizon optimizer |
