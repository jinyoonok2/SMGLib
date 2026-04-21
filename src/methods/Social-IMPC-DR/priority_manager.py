"""
Phase 2 — Priority-Aware Landing Pad Controller

Extends LandingPadController with dynamic priority scoring.  The yielding
decision is overridden: instead of closest-drone-first, the drone with the
highest ``priority_score()`` proceeds while all others hold.

Additionally, ``step_update()`` decrements each drone's ``time_to_expiry``
so that priority becomes more urgent over time.

Usage:
    controller = PriorityManager(cargo_configs)
    # … then use exactly the same interface as LandingPadController
"""

import numpy as np
from landing_pad import LandingPadController, PAD_CENTER
from priority import priority_score


class PriorityManager(LandingPadController):
    """Phase 2: priority-score-based landing policy."""

    def __init__(self, cargo_configs=None, use_hysteresis=True):
        super().__init__()
        self.cargo_configs = cargo_configs or []
        # When True, the chosen winner keeps the lead until they land
        # (only Expiry Guard can override). When False, the highest-scoring
        # drone is recomputed every step — produces visible chattering when
        # scores or distances are nearly tied.
        self._use_hysteresis = use_hysteresis
        self._held_winner    = None
        self._held_method    = None

    # ------------------------------------------------------------------
    # Internal helper: per-drone priority scores for the active set
    # ------------------------------------------------------------------
    def _score_active(self, agent_list, active_drones):
        """Return ``{drone_idx: priority_score}`` for every active drone."""
        scores = {}
        for j in active_drones:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            scores[j] = priority_score(
                cargo_type=a.cargo_type,
                time_to_expiry=a.time_to_expiry,
                distance_to_pad=dist,
                patient_acuity=a.patient_acuity,
            )
        return scores

    def _commit_winner(self, idx, method):
        """Record a winning drone for hysteresis bookkeeping."""
        self._held_winner = idx
        self._held_method = method

    # ------------------------------------------------------------------
    # Override: yielding decision (priority-based)
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        """Highest-priority active drone proceeds; others yield.

        Decision order:
          1. Trivial case (≤1 active drone)
          2. Expiry Guard override (always wins, even over hysteresis)
          3. Held winner (if hysteresis on and previous winner still active)
          4. Other negotiation override (e.g. ETA Switch)
          5. Fresh priority-score winner (or distance fallback)
        """
        # 1. Trivial case
        if len(active_drones) <= 1:
            idx = active_drones[0] if active_drones else None
            self._commit_winner(idx, "priority")
            return {"allowed": idx, "yielding": set(),
                    "method": "priority", "scores": {}}

        # 2. Expiry Guard always overrides (including held winner)
        override = self.negotiation_hook(agent_list, active_drones, step)
        if override is not None and override.get("method") == "expiry_guard":
            self._commit_winner(override["allowed"], "expiry_guard")
            return override

        # 3. Held winner (commit-and-complete)
        if (self._use_hysteresis
                and self._held_winner is not None
                and self._held_winner in active_drones):
            return self._use_held_winner(agent_list, active_drones)

        # 4. Other negotiation override (e.g. ETA Switch)
        if override is not None:
            self._commit_winner(override["allowed"],
                                override.get("method", "priority"))
            return override

        # 5. Fresh winner from priority scoring (or distance fallback)
        return self._pick_fresh_winner(agent_list, active_drones, step, verbose)

    # ------------------------------------------------------------------
    # Branch helpers for select_active_drone
    # ------------------------------------------------------------------
    def _use_held_winner(self, agent_list, active_drones):
        """Re-emit the held winner; recompute scores for label display only."""
        yielding = {j for j in active_drones if j != self._held_winner}
        return {"allowed":  self._held_winner,
                "yielding": yielding,
                "method":   self._held_method or "priority",
                "scores":   self._score_active(agent_list, active_drones)}

    def _pick_fresh_winner(self, agent_list, active_drones, step, verbose):
        """Compute a fresh winner from priority scores (or distance fallback)."""
        # Distance fallback: no drone has non-default cargo/acuity
        has_priority = any(
            agent_list[j].cargo_type != 'equipment'
            or agent_list[j].patient_acuity != 'routine'
            for j in active_drones
        )
        if not has_priority:
            return super().select_active_drone(
                agent_list, active_drones, step, verbose
            )

        scores = self._score_active(agent_list, active_drones)
        allowed_idx = max(scores, key=scores.get)
        self._commit_winner(allowed_idx, "priority")

        if verbose and step % 20 == 0:
            ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
            print(
                f"  Priority yielding: drone {allowed_idx} proceeds "
                f"(score={ranked[0][1]:.3f}), "
                f"others: {[(i, f'{s:.3f}') for i, s in ranked[1:]]}"
            )

        yielding = {j for j in active_drones if j != allowed_idx}
        return {"allowed": allowed_idx, "yielding": yielding,
                "method": "priority", "scores": scores}

    # ------------------------------------------------------------------
    # Override: per-step hook — decrement time_to_expiry
    # ------------------------------------------------------------------
    def step_update(self, agent_list, target_reached, num_moving_drones):
        """Decrement ``time_to_expiry`` for every active drone each step."""
        for j in range(num_moving_drones):
            if not target_reached[j]:
                agent_list[j].time_to_expiry = max(
                    0.0, agent_list[j].time_to_expiry - 1.0
                )
            agent_list[j].tte_history.append(agent_list[j].time_to_expiry)
