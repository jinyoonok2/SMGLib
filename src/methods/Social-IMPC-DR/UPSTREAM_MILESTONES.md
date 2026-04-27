# Upstream Milestone PR Runbook

This runbook keeps milestone PRs small, reproducible, and easy to merge into
the professor's SMGLib repository while preserving branch history.

## PR Sequence

Submit milestone PRs in this order:

1. Core baseline + Phase 2/3
2. Phase 4/4.1 negotiation and hysteresis
3. Phase 6 round trip
4. Phase 7 planner
5. LLM extensions (Phase 5-LLM and Phase 7-LLM)

## Required PR Template

Copy this into each PR body.

```markdown
## Scope
- Milestone: <name>
- Branch: <branch-name>
- Controllers touched: <list>

## Repro Command
```bash
python app2_standardized.py landing_pad configs/<config-name>.json
```

## Expected Artifact
- logs/Social-IMPC-DR/animations/<test_name>.gif

## Verification Notes
- Expected behavior:
- Observed landing order:
- Completion step:
```

## Repro Matrix (by milestone)

Run from `src/methods/Social-IMPC-DR`.

- Milestone 1 (Phase 2/3 policy + orbit):
  - `python app2_standardized.py landing_pad configs/phase2_landing_pad.json`
  - `python app2_standardized.py landing_pad configs/phase3_orbit.json`
  - Artifacts:
    - `logs/Social-IMPC-DR/animations/phase2_landing_pad.gif`
    - `logs/Social-IMPC-DR/animations/phase3_orbit.gif`

- Milestone 2 (Phase 4/4.1 negotiation + hysteresis):
  - `python app2_standardized.py landing_pad configs/phase4_negotiation.json`
  - `python app2_standardized.py landing_pad configs/phase4_negotiation_no_hysteresis.json`
  - `python app2_standardized.py landing_pad configs/phase4_expiry_guard_test.json`
  - `python app2_standardized.py landing_pad configs/phase4_eta_switch_test.json`
  - Artifacts:
    - `logs/Social-IMPC-DR/animations/phase4_negotiation.gif`
    - `logs/Social-IMPC-DR/animations/phase4_negotiation_no_hysteresis.gif`
    - `logs/Social-IMPC-DR/animations/phase4_expiry_guard_test.gif`
    - `logs/Social-IMPC-DR/animations/phase4_eta_switch_test.gif`

- Milestone 3 (Phase 6 round trip):
  - `python app2_standardized.py landing_pad configs/phase6_round_trip.json`
  - Artifact:
    - `logs/Social-IMPC-DR/animations/phase6_round_trip.gif`

- Milestone 4 (Phase 7 planner):
  - `python app2_standardized.py landing_pad configs/phase7_planner_oneway.json`
  - `python app2_standardized.py landing_pad configs/phase7_planner_round_trip.json`
  - Artifacts:
    - `logs/Social-IMPC-DR/animations/phase7_planner_oneway.gif`
    - `logs/Social-IMPC-DR/animations/phase7_planner_round_trip.gif`

## Branch-to-Milestone Map

- `research/phase2-6-policy-stack`
  - Milestone 1, 2, 3
- `research/phase7-trajectory-planner`
  - Milestone 4
- `research/phase5-llm`
  - Milestone 5 (Phase 5 LLM)
- `research/phase7-llm`
  - Milestone 5 (Phase 7 LLM)
