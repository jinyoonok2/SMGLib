"""Finite-horizon lookahead trajectory planner.

This planner extends the baseline trajectory planner by evaluating
candidate landing orders and selecting the schedule with lowest
priority-weighted delay and expiry risk.
"""

import itertools
import numpy as np

from landing_pad import PAD_CENTER
from priority import priority_score

from .baseline import (
    TrajectoryPlannerController,
    OUTBOUND
)

class LookaheadTrajectoryPlannerController(TrajectoryPlannerController):
    """Finite-horizon lookahead version of the baseline trajectory planner."""
    
    def _choose_lookahead_order(self, info):
        """Choose landing order by evaluating all candidate schedules.

        Cost balances:
        - weighted delay: high-priority drones should arrive earlier
        - expiry risk: drones that would expire before arrival are heavily penalized
        """
        best_order = None
        best_cost = float("inf")

        for order in itertools.permutations(info):
            T_prev = 0.0
            v_prev = self._max_speed
            total_cost = 0.0

            for rank, d in enumerate(order):
                dist = d["dist"]
                score = d["score"]

                T_phys = dist / max(self._max_speed, 1e-6)
                T_pad = T_prev + self._unload_steps
                T_sep = T_prev + self._min_separation / max(v_prev, 1e-6)

                T_arrive = max(
                    T_phys,
                    T_pad if rank > 0 else 0.0,
                    T_sep if rank > 0 else 0.0,
                )

                v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)

                # Higher-priority drones waiting longer is worse.
                total_cost += score * T_arrive

                # Strong penalty if arrival occurs after cargo expiry.
                tte = d.get("time_to_expiry", 300.0)
                expiry_lateness = max(0.0, T_arrive - tte)
                total_cost += 100.0 * expiry_lateness

                T_prev = T_arrive
                v_prev = v_assign

            if total_cost < best_cost:
                best_cost = total_cost
                best_order = list(order)

        return best_order

    def _replan(self, agent_list, inbound, step):
        """Compute per-drone arrival slots and speed caps."""
        if not inbound:
            self._last_schedule = {}
            return

        # 1) Score inbound drones with the same priority function as Track 1.
        info = []
        for j in inbound:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            score = priority_score(
                cargo_type=getattr(a, "cargo_type", "equipment"),
                time_to_expiry=float(getattr(a, "time_to_expiry", 300.0)),
                distance_to_pad=dist,
                patient_acuity=getattr(a, "patient_acuity", "routine"),
            )
            info.append({"idx": j, "dist": dist, "score": score, "time_to_expiry": float(getattr(a, "time_to_expiry", 300.0))})

        if self._llm_advisor is not None:
            adjusted_scores = self._llm_advisor.maybe_adjust_scores(
                agent_list, info, step
            )
            if adjusted_scores is not None:
                for d in info:
                    d["score"] = adjusted_scores[d["idx"]]

        self._last_scores = {d["idx"]: d["score"] for d in info}

        # 2) Choose landing order using finite-horizon lookahead.
        info = self._choose_lookahead_order(info)

        # 3) Assign slot times and speed caps with three floors.
        #    T >= d/max_speed
        #    T >= T_prev + unload_steps
        #    T >= T_prev + min_separation/v_prev
        schedule = {}
        T_prev = 0.0
        v_prev = self._max_speed
        for rank, d in enumerate(info):
            j = d["idx"]
            dist = d["dist"]
            T_phys = dist / max(self._max_speed, 1e-6)
            T_pad  = T_prev + self._unload_steps
            T_sep  = T_prev + self._min_separation / max(v_prev, 1e-6)
            T_arrive = max(T_phys,
                            T_pad if rank > 0 else 0.0,
                            T_sep if rank > 0 else 0.0)
            v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)
            schedule[j] = {"T_arrive": T_arrive, "Vmax": v_assign,
                            "rank": rank, "dist": dist}
            T_prev = T_arrive
            v_prev = v_assign

        # 4) Push speed caps to MPC.
        for j, sch in schedule.items():
            agent_list[j].Vmax = sch["Vmax"]

        # 5) OUTBOUND drones are unconstrained by pad contention.
        for j in range(len(agent_list)):
            if self._state.get(j) == OUTBOUND:
                agent_list[j].Vmax = self._max_speed

        self._last_schedule = schedule
        self._last_inbound  = frozenset(inbound)
        self._planner_dirty = False

        if step % 25 == 1:
            ranked = sorted(schedule.items(), key=lambda kv: kv[1]["rank"])
            summary = ", ".join(
                f"D{j}: v={s['Vmax']:.3f} T={s['T_arrive']:.1f}"
                for j, s in ranked
            )
            print(f"  [Lookahead Planner] step {step} schedule -> {summary}")

        if self._llm_advisor is not None:
            self._llm_advisor.maybe_explain_schedule(
                agent_list, info, schedule, step
            )
