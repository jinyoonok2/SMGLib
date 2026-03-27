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

    def __init__(self, cargo_configs=None):
        self.cargo_configs = cargo_configs or []

    # ------------------------------------------------------------------
    # Override: yielding decision (priority-based)
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        """Highest-priority active drone proceeds; others yield.

        Falls back to Phase 1 distance-based selection when no drone has
        non-default cargo/acuity settings.
        """
        if len(active_drones) <= 1:
            idx = active_drones[0] if active_drones else None
            return {"allowed": idx, "yielding": set(),
                    "method": "priority", "scores": {}}

        # Check negotiation hook first
        override = self.negotiation_hook(agent_list, active_drones, step)
        if override is not None:
            return override

        # Check whether any drone has non-default priority attributes
        has_priority = any(
            agent_list[j].cargo_type != 'equipment'
            or agent_list[j].patient_acuity != 'routine'
            for j in active_drones
        )

        if not has_priority:
            # Delegate to Phase 1 distance-based logic
            return super().select_active_drone(
                agent_list, active_drones, step, verbose
            )

        # Compute priority scores
        scored = []
        for j in active_drones:
            a = agent_list[j]
            dist = np.linalg.norm(a.p - PAD_CENTER)
            s = priority_score(
                cargo_type=a.cargo_type,
                time_to_expiry=a.time_to_expiry,
                distance_to_pad=dist,
                patient_acuity=a.patient_acuity,
            )
            scored.append((j, s, dist))

        scored.sort(key=lambda x: x[1], reverse=True)
        allowed_idx = scored[0][0]

        if verbose and step % 20 == 0:
            print(
                f"  Priority yielding: drone {allowed_idx} proceeds "
                f"(score={scored[0][1]:.3f}, dist={scored[0][2]:.2f}), "
                f"others: {[(d[0], f'{d[1]:.3f}') for d in scored[1:]]}"
            )

        yielding = {j for j in active_drones if j != allowed_idx}
        scores = {j: s for j, s, _ in scored}
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
