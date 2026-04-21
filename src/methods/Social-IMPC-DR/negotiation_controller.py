"""
Phase 4 ΓÇö ETA-Weighted Negotiation Controller

Extends OrbitController with per-step negotiation via two rule-based
arbitration rules applied inside ``negotiation_hook()``:

Rule 1 ΓÇö Expiry Guard
    If any active drone's ``time_to_expiry`` is less than its ETA
    (distance_to_pad / nominal_speed), that drone will expire before it
    can land.  The most critical such drone (lowest tte/ETA ratio) wins
    unconditionally, regardless of priority scores.

Rule 2 ΓÇö ETA-Weighted Switching
    If the score gap between the top two priority-ranked drones is smaller
    than ``eta_threshold``, scores are too close to discriminate on merit
    alone ΓÇö the drone with the lower ETA (closer to the pad) wins instead.

If neither rule fires, ``negotiation_hook`` returns ``None`` and normal
priority-score ordering (Phase 2) applies.

Usage:
    controller = NegotiationController(
        cargo_configs,
        orbit_radius=0.7,
        orbit_speed=0.15,
        safe_distance=1.2,
        nominal_speed=0.1,   # metres per simulation step
        eta_threshold=0.15,  # max score gap for ETA tie-breaking
    )
"""

import numpy as np
from orbit_controller import OrbitController
from landing_pad import PAD_CENTER
from priority import priority_score


class NegotiationController(OrbitController):
    """Phase 4: ETA-weighted negotiation + expiry guard on top of orbit."""

    def __init__(self, cargo_configs=None, orbit_radius=0.7, orbit_speed=0.15,
                 safe_distance=1.2, nominal_speed=0.1, eta_threshold=0.15,
                 use_hysteresis=True):
        super().__init__(cargo_configs, orbit_radius, orbit_speed,
                         safe_distance, use_hysteresis=use_hysteresis)
        self._nominal_speed  = max(nominal_speed, 1e-6)  # metres / step
        self._eta_threshold  = eta_threshold

    # ------------------------------------------------------------------
    # Override: negotiation hook ΓÇö called before priority scoring
    # ------------------------------------------------------------------
    def negotiation_hook(self, agent_list, active_drones, step):
        """Apply expiry guard and ETA tie-breaking before priority scoring.

        Returns a result dict to override the normal decision, or None to
        let Phase 2 priority scoring proceed unchanged.
        """
        if len(active_drones) <= 1:
            return None

        # ΓöÇΓöÇ Gather per-drone ETA and priority info ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
        info = []
        for j in active_drones:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            eta  = dist / self._nominal_speed          # steps to reach pad
            tte  = float(a.time_to_expiry)
            s = priority_score(
                cargo_type=a.cargo_type,
                time_to_expiry=tte,
                distance_to_pad=dist,
                patient_acuity=a.patient_acuity,
            )
            info.append({
                'idx':  j,
                'dist': dist,
                'eta':  eta,
                'tte':  tte,
                'score': s,
            })

        scores_map = {d['idx']: d['score'] for d in info}

        # ΓöÇΓöÇ Rule 1: Expiry Guard ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
        # Any drone that will expire before it can reach the pad.
        expiring = [d for d in info if d['tte'] < d['eta']]
        if expiring:
            # Among expiring drones, pick the one closest to expiry
            # relative to its ETA (ratio closest to 0 = most urgent).
            expiring.sort(key=lambda d: d['tte'] / max(d['eta'], 1e-6))
            winner = expiring[0]
            allowed_idx = winner['idx']
            yielding = {j for j in active_drones if j != allowed_idx}
            if step % 10 == 0:
                print(
                    f"  [Negotiation] Expiry Guard at step {step}: "
                    f"drone {allowed_idx} wins "
                    f"(tte={winner['tte']:.1f} < eta={winner['eta']:.1f})"
                )
            return {
                "allowed":  allowed_idx,
                "yielding": yielding,
                "method":   "expiry_guard",
                "scores":   scores_map,
            }

        # ΓöÇΓöÇ Rule 2: ETA-Weighted Switching ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
        # Sort by priority score descending.
        info_by_score = sorted(info, key=lambda d: d['score'], reverse=True)
        top    = info_by_score[0]
        second = info_by_score[1]

        if abs(top['score'] - second['score']) < self._eta_threshold:
            # Scores are too close - let ETA decide (lower ETA = closer = wins).
            info_by_eta = sorted(info, key=lambda d: d['eta'])
            winner = info_by_eta[0]
            allowed_idx = winner['idx']
            yielding = {j for j in active_drones if j != allowed_idx}
            if step % 20 == 0:
                print(
                    f"  [Negotiation] ETA switch at step {step}: "
                    f"drone {allowed_idx} wins by proximity "
                    f"(score gap={abs(top['score'] - second['score']):.3f} "
                    f"< threshold={self._eta_threshold}, "
                    f"eta={winner['eta']:.1f} steps)"
                )
            return {
                "allowed":  allowed_idx,
                "yielding": yielding,
                "method":   "eta_switch",
                "scores":   scores_map,
            }

        return None
