"""
Phase 7 - Trajectory Planner Controller (speed-scaling mechanism)

Alternative to the orbit/yield mechanism used in Phases 1-6. Instead of
making lower-priority drones hover in circular holding patterns, the
planner pre-computes a per-drone cruise speed (`Vmax`) so that all
drones fly simultaneously and arrive at the central pad in the desired
priority order, separated by `unload_steps` so each drone finishes
unloading before the next arrives.
"""

import numpy as np

from round_trip_controller import RoundTripController, INBOUND, UNLOADING, OUTBOUND
from landing_pad import PAD_CENTER
from priority import priority_score


class TrajectoryPlannerController(RoundTripController):
    """Phase 7: speed-scaled simultaneous arrivals (no orbit holding)."""

    def __init__(
        self,
        cargo_configs=None,
        return_points=None,
        n_trips=1,
        unload_steps=5,
        max_speed=1.0,
        min_separation=1.0,
        safe_distance=1.2,
        nominal_speed=0.1,
        # Kept for base-class compatibility (unused by planner logic).
        orbit_radius=0.7,
        orbit_speed=0.15,
        eta_threshold=0.15,
        use_hysteresis=True,
    ):
        super().__init__(
            cargo_configs,
            return_points=return_points,
            n_trips=n_trips,
            unload_steps=unload_steps,
            orbit_radius=orbit_radius,
            orbit_speed=orbit_speed,
            safe_distance=safe_distance,
            nominal_speed=nominal_speed,
            eta_threshold=eta_threshold,
            use_hysteresis=use_hysteresis,
        )
        self._max_speed = float(max_speed)
        self._min_separation = float(min_separation)
        self._safe_distance = float(safe_distance)
        self._last_inbound = frozenset()
        self._last_scores = {}
        self._last_schedule = {}  # j -> {"T_arrive", "Vmax", "rank"}
        self._planner_dirty = True

    def _replan(self, agent_list, inbound, step):
        """Compute per-drone arrival slots and speed caps."""
        if not inbound:
            self._last_schedule = {}
            return

        # 1) Score inbound drones with the same priority function as track 1.
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
            info.append({"idx": j, "dist": dist, "score": score})
        self._last_scores = {d["idx"]: d["score"] for d in info}

        # 2) Rank by score descending (higher priority first).
        info.sort(key=lambda d: d["score"], reverse=True)

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
            T_pad = T_prev + self._unload_steps
            T_sep = T_prev + self._min_separation / max(v_prev, 1e-6)
            T_arrive = max(T_phys, T_pad if rank > 0 else 0.0, T_sep if rank > 0 else 0.0)
            v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)
            schedule[j] = {"T_arrive": T_arrive, "Vmax": v_assign, "rank": rank, "dist": dist}
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
        self._last_inbound = frozenset(inbound)
        self._planner_dirty = False

        if step % 25 == 1:
            ranked = sorted(schedule.items(), key=lambda kv: kv[1]["rank"])
            summary = ", ".join(f"D{j}: v={s['Vmax']:.3f} T={s['T_arrive']:.1f}" for j, s in ranked)
            print(f"  [Planner] step {step} schedule -> {summary}")

    def select_active_drone(self, agent_list, active_drones, step, verbose):
        """Planner mode: everyone can fly; safety-net may freeze near-pad inbound drones."""
        self._ensure_init(len(agent_list), agent_list)
        inbound = [j for j in active_drones if self._state.get(j, INBOUND) == INBOUND]

        if frozenset(inbound) != self._last_inbound or self._planner_dirty:
            self._replan(agent_list, inbound, step)

        # Safety net only while the pad is occupied by an unloading drone.
        pad_busy = any(s == UNLOADING for s in self._state.values())
        yielding = set()
        if pad_busy:
            for j in inbound:
                d_pad = float(np.linalg.norm(agent_list[j].p - PAD_CENTER))
                if d_pad < self._safe_distance:
                    yielding.add(j)

        return {
            "allowed": None,  # planner does not use single-winner routing
            "yielding": yielding,
            "method": "planner",
            "scores": dict(self._last_scores),
        }

    def cleanup_landed(self, agent_list, target_reached, num_moving_drones, K):
        before = dict(self._state)
        super().cleanup_landed(agent_list, target_reached, num_moving_drones, K)
        if any(self._state.get(j) != before.get(j) for j in range(num_moving_drones)):
            self._planner_dirty = True
            for j in range(num_moving_drones):
                if self._state.get(j) == OUTBOUND:
                    agent_list[j].Vmax = self._max_speed

    def step_update(self, agent_list, target_reached, num_moving_drones):
        before = dict(self._state)
        super().step_update(agent_list, target_reached, num_moving_drones)
        if any(self._state.get(j) != before.get(j) for j in range(num_moving_drones)):
            self._planner_dirty = True
            for j in range(num_moving_drones):
                if self._state.get(j) == OUTBOUND:
                    agent_list[j].Vmax = self._max_speed
