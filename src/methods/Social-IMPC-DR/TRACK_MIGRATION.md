# Track Migration Checklist

Use this checklist to migrate from legacy phase framing to the new two-track
framework without breaking existing experiments.

## Branching

- [ ] Keep ongoing policy/yield work in `research/track-policy-yield`
- [ ] Keep ongoing planner work in `research/track-trajectory-planner`
- [ ] Keep `main` as stable integration target (no direct experimental rewrites)

## Backward Compatibility (temporary)

- [ ] Keep existing `phase*.json` configs in place during migration
- [ ] Keep phase-oriented flags working until both tracks are validated
- [ ] Treat legacy phase config names as compatibility aliases

## New Track Framing

- [ ] Define track-level naming in docs:
  - `track-policy-yield`
  - `track-trajectory-planner`
- [ ] Add track-focused config names (for example `track_policy_*.json`,
      `track_planner_*.json`) while retaining legacy phase configs
- [ ] Update runner/help text to describe control mode by track, not by phase

## Modular Structure Guidance

- [ ] Keep shared landing-pad domain code neutral and reusable
- [ ] Isolate policy/yield-specific logic in its own modules
- [ ] Isolate planner-specific logic in its own modules
- [ ] Limit shared file edits (`app2_standardized.py`, `test.py`) to additive
      feature-flag plumbing

## Validation Before Legacy Cleanup

- [ ] Re-run representative policy/yield scenarios and record artifacts
- [ ] Re-run representative planner scenarios and record artifacts
- [ ] Confirm both tracks run from the same shared landing-pad foundation
- [ ] Confirm no required workflow depends exclusively on phase naming

## Legacy Removal (final cleanup pass)

- [ ] Remove deprecated phase-only config files
- [ ] Remove phase-only doc language and examples
- [ ] Keep one migration note in changelog documenting the rename/removal
- [ ] Run final regression checks after removal

## Suggested Rollout

1. Build and validate both tracks with compatibility aliases in place.
2. Update docs/config naming to track-first language.
3. Remove legacy phase files in one dedicated cleanup PR.
