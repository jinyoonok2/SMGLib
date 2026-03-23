# Changelog — Social-IMPC-DR (Landing Pad Extension)

All notable changes to the Social-IMPC-DR method and supporting files for the
Priority-Aware Multi-Drone Coordination project.

## Phase 1 — Greedy Baseline (2026-03-23)

### New files
- `docs/PROJECT_GUIDELINE.md` — Full project description with 5-phase roadmap.

### Modified files

#### `src/utils.py`
- Added `get_landing_pad_obstacles()` — rectangular perimeter (40 wall agents)
  with approach gaps at top/bottom center.
- Added `LANDING_PAD_CENTER`, `LANDING_PAD_RADIUS` class attributes.
- Added `landing_pad` entry in `get_standard_agent_positions()` — 4 approach
  directions, all goals at (0, 0).
- Added landing-pad marker drawing (yellow circle + red cross) in visualization.

#### `SET.py`
- Added `ENV_TYPE` parameter and `env_type` global so other modules can read
  the active environment.

#### `app2_standardized.py`
- Added `landing_pad` scenario branch in the interactive menu.
- Fixed legend: `generate_animation_standardized` and `save_gif_standardized`
  now only show moving agents (not the 40 wall agents).

#### `avoid.py`
- Accepts `env_type` parameter (no landing-pad-specific logic remains after
  removing the infeasible 4× `r_min` inflation).

#### `run.py`
- Reads `env_type` and `wall_collision_multiplier` from `SET` module and
  forwards them to `GET_cons()`.

#### `test.py` (PLAN function) — core yielding logic
- **Target-based yielding**: at each step, only the closest active drone to the
  pad runs MPC; all others are frozen (velocity zeroed, no MPC call).
- **Clear landed drones**: teleport to (100, 100) — updating `p`, `v`, `state`,
  AND `pre_traj` — so collision avoidance doesn't see a ghost obstacle.
- **MPC warm-start reset**: when a drone transitions from yielding → active,
  `pre_traj` is re-tiled to the current position and `cost_index` is reset to 0.
- **Position history patch**: manually append to `agent.position` for drones
  that skip MPC, so animation frame indices stay aligned.
- Goal threshold relaxed to 0.05 for `landing_pad` (0.02 for others).
- Early `break` when all moving drones reach their goals.

### Bugs fixed
| # | Symptom | Root cause | Fix |
|---|---------|-----------|-----|
| 1 | Both drones stuck, never reaching pad | 4× `r_min` inflation in `avoid.py` made MPC infeasible | Removed inflation; use target-based yielding instead |
| 2 | Drone 1 blocked by landed Drone 0 at (0,0) | No mechanism to clear landed drones | Teleport landed drones to (100,100) |
| 3 | Drone 1 oscillates ~0.2 from goal | (a) Stale MPC warm-start after 51 steps of hold-position MPC; (b) `pre_traj` not updated on teleport → ghost obstacle | Skip MPC for yielding drones; reset warm-start on release; update `pre_traj` on teleport |
| 4 | GIF shows both drones moving simultaneously | `post_processing()` only runs inside MPC — yielding/landed drones had shorter `position` arrays | Manually append to `position` for skipped drones |

### Results
- Drone 0 lands at step 51, Drone 1 lands at step 102.
- 100% success rate, 120-step budget.
