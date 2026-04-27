# LLM Branch Bootstrap

This file defines the intended split for upcoming LLM work so Phase 5 and
Phase 7 can progress in parallel without merge conflicts.

## Branch Roles

- `research/phase5-llm`
  - Base branch: `research/phase2-6-policy-stack`
  - Scope: LLM replacement/augmentation for Phase 4 negotiation and Phase 5
    ablations on policy/yield controllers.
- `research/phase7-llm`
  - Base branch: `research/phase7-trajectory-planner`
  - Scope: LLM-enhanced scheduling for planner-driven arrivals (Phase 7 track).

## Ownership Guidance

- Keep Phase 5 LLM changes focused on:
  - `llm_controller.py`
  - `negotiation_controller.py`
  - `priority.py` / `priority_manager.py` (only when testing LLM scoring)
  - `configs/phase5_*.json`
- Keep Phase 7 LLM changes focused on:
  - `trajectory_planner_controller.py`
  - `configs/phase7_*.json`
  - optional adapter module for planner prompt/reasoning
- Shared files (`app2_standardized.py`, `test.py`) should only receive additive
  feature-flag plumbing to avoid cross-track coupling.

## Recommended Kickoff Commands

Run from repo root:

```bash
git checkout research/phase5-llm
git checkout research/phase7-llm
```

## Merge Policy

- Merge `research/phase5-llm` into upstream only after Phase 4/4.1 + Phase 6
  milestones are accepted.
- Merge `research/phase7-llm` only after Phase 7 planner milestone is accepted.
- Keep both LLM branches re-based or merged regularly from their parent tracks.
