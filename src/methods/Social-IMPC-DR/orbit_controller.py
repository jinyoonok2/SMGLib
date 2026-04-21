"""
Phase 3 — Holding-Pattern (Orbit) Controller

Extends PriorityManager so that yielding drones fly circular holding orbits
instead of freezing in place.

The orbit for each drone is initialised the first step it enters the yield
state:
  - ``orbit_center`` = the drone's current position at yield entry
  - ``angle``        = starts at 0, advances by ``orbit_speed`` each step

The angular speed (orbit_speed, radians/step) and radius (orbit_radius, metres)
are loaded from the scenario config and passed to the constructor.

Because ``pre_traj`` is filled with the predicted orbit positions, other
drones' MPC solvers see the circling drone's future trajectory and plan
accordingly.

Usage:
    controller = OrbitController(cargo_configs,
                                  orbit_radius=0.7,
                                  orbit_speed=0.15)
    # … same interface as PriorityManager / LandingPadController
"""

import numpy as np
from priority_manager import PriorityManager


class OrbitController(PriorityManager):
    """Phase 3: yielding drones fly circular holding orbits."""

    def __init__(self, cargo_configs=None, orbit_radius=0.7, orbit_speed=0.15,
                 safe_distance=1.2):
        super().__init__(cargo_configs)
        self._orbit_radius  = orbit_radius
        self._orbit_speed   = orbit_speed    # radians per step
        self._safe_distance = safe_distance  # min gap between orbit edge and active drone
        self._orbit_state   = {}             # drone_index -> {center, angle}
        self._active_idx    = None           # index of the currently allowed drone

    # ------------------------------------------------------------------
    # Override: cache active drone index so freeze_yielding can use it
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        result = super().select_active_drone(agent_list, active_drones, step, verbose)
        self._active_idx = result["allowed"]
        return result

    # ------------------------------------------------------------------
    # Override: orbit instead of freeze
    # ------------------------------------------------------------------
    def freeze_yielding(self, agent_list, yielding_drones):
        """Advance each yielding drone one step along its holding orbit.

        Replaces the parent's zero-velocity freeze with a continuous
        circular motion.  The orbit state is reset when a drone leaves
        the yield set (so re-entry starts a fresh orbit).

        If the orbit center would place the drone within
        ``orbit_radius + safe_distance`` of the active (higher-priority)
        drone, the center is pushed outward so the entire orbit stays
        at least ``safe_distance`` clear of the active drone.
        """
        # Remove stale orbit state for drones that are no longer yielding
        stale_keys = [j for j in list(self._orbit_state)
                      if j not in yielding_drones]
        for j in stale_keys:
            del self._orbit_state[j]

        # Resolve active drone position once per step
        active_p = None
        if (self._active_idx is not None
                and self._active_idx < len(agent_list)):
            active_p = agent_list[self._active_idx].p

        for j in yielding_drones:
            # Initialise orbit the first step this drone yields
            if j not in self._orbit_state:
                self._orbit_state[j] = {
                    'center': agent_list[j].p.copy(),
                    'angle': 0.0,
                }

            state = self._orbit_state[j]
            state['angle'] += self._orbit_speed

            # ----------------------------------------------------------
            # Safe-distance repulsion: push orbit center away from the
            # active drone so the whole orbit stays clear.
            # ----------------------------------------------------------
            if active_p is not None:
                vec = state['center'] - active_p
                dist_center = np.linalg.norm(vec)
                min_center_dist = self._orbit_radius + self._safe_distance
                if dist_center < min_center_dist:
                    push_dir = vec / dist_center if dist_center > 1e-6 \
                        else np.array([1.0, 0.0])
                    state['center'] = active_p + push_dir * min_center_dist

            a = state['angle']
            c = state['center']
            r = self._orbit_radius

            # New position on the circle
            new_p = c + r * np.array([np.cos(a), np.sin(a)])

            # Tangential velocity (derivative of position w.r.t. time)
            new_v = self._orbit_speed * r * np.array([-np.sin(a), np.cos(a)])

            agent_list[j].p     = new_p
            agent_list[j].v     = new_v
            agent_list[j].state = np.append(new_p, new_v)

            # Fill pre_traj with predicted orbit positions so that the
            # active drone's MPC sees where this drone will be.
            K = agent_list[j].K
            future_angles = a + self._orbit_speed * np.arange(1, K + 2)
            future_pos = c + r * np.column_stack(
                [np.cos(future_angles), np.sin(future_angles)]
            )
            agent_list[j].pre_traj = future_pos
