"""Negotiators may override the selector decision before hysteresis runs.

Each negotiator returns either a full decision dict or ``None`` (in which
case the next negotiator, then the selector, is consulted). They are
peers: the controller calls them in the order listed in the recipe and
takes the first non-None result.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from priority import priority_score

from .context import Context, PAD_CENTER


class Negotiator:
    name: str = "negotiator"

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        raise NotImplementedError


def _gather_info(candidates: List[int], ctx: Context, nominal_speed: float):
    info = []
    for j in candidates:
        a = ctx.agent_list[j]
        dist = float(np.linalg.norm(a.p - PAD_CENTER))
        eta = dist / max(nominal_speed, 1e-6)
        tte = float(a.time_to_expiry)
        s = priority_score(
            cargo_type=a.cargo_type,
            time_to_expiry=tte,
            distance_to_pad=dist,
            patient_acuity=a.patient_acuity,
        )
        info.append({
            "idx":   j,
            "dist":  dist,
            "eta":   eta,
            "tte":   tte,
            "score": s,
        })
    return info


class ExpiryGuard(Negotiator):
    """If any candidate's TTE is below its ETA, that drone wins.

    Among multiple expiring candidates, the most urgent (lowest tte/eta
    ratio) is chosen. This negotiator is also treated as authoritative
    by the hysteresis layer — i.e. it can break a previously held
    winner, where other negotiators cannot.
    """

    name = "expiry_guard"

    def __init__(self, nominal_speed: float = 0.1) -> None:
        self.nominal_speed = nominal_speed

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        if len(candidates) <= 1:
            return None
        info = _gather_info(candidates, ctx, self.nominal_speed)
        scores_map = {d["idx"]: d["score"] for d in info}

        expiring = [d for d in info if d["tte"] < d["eta"]]
        if not expiring:
            return None
        expiring.sort(key=lambda d: d["tte"] / max(d["eta"], 1e-6))
        winner = expiring[0]
        allowed_idx = winner["idx"]
        yielding = {j for j in candidates if j != allowed_idx}
        if ctx.step % 10 == 0:
            print(
                f"  [Negotiation] Expiry Guard at step {ctx.step}: "
                f"drone {allowed_idx} wins "
                f"(tte={winner['tte']:.1f} < eta={winner['eta']:.1f})"
            )
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores_map,
        }


class EtaSwitch(Negotiator):
    """Tie-break by ETA when the top two priority scores are too close.

    If ``|score_top - score_second| < eta_threshold`` the closer drone
    wins; otherwise this negotiator returns ``None`` and lets normal
    selection proceed.
    """

    name = "eta_switch"

    def __init__(
        self,
        nominal_speed: float = 0.1,
        eta_threshold: float = 0.15,
    ) -> None:
        self.nominal_speed = nominal_speed
        self.eta_threshold = eta_threshold

    def override(self, candidates: List[int], ctx: Context) -> Optional[Dict]:
        if len(candidates) <= 1:
            return None
        info = _gather_info(candidates, ctx, self.nominal_speed)
        scores_map = {d["idx"]: d["score"] for d in info}

        info_by_score = sorted(info, key=lambda d: d["score"], reverse=True)
        top, second = info_by_score[0], info_by_score[1]
        if abs(top["score"] - second["score"]) >= self.eta_threshold:
            return None

        info_by_eta = sorted(info, key=lambda d: d["eta"])
        winner = info_by_eta[0]
        allowed_idx = winner["idx"]
        yielding = {j for j in candidates if j != allowed_idx}
        if ctx.step % 20 == 0:
            print(
                f"  [Negotiation] ETA switch at step {ctx.step}: "
                f"drone {allowed_idx} wins by proximity "
                f"(score gap={abs(top['score'] - second['score']):.3f} "
                f"< threshold={self.eta_threshold}, "
                f"eta={winner['eta']:.1f} steps)"
            )
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores_map,
        }
