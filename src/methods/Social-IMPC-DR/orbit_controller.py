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

    def __init__(self, cargo_configs=None, orbit_radius=0.7, orbit_speed=0.15):
        super().__init__(cargo_configs)
        self._orbit_radius = orbit_radius
        self._orbit_speed  = orbit_speed   # radians per step
        self._orbit_state  = {}            # drone_index -> {center, angle}

    # ------------------------------------------------------------------
    # Override: orbit instead of freeze
    # ------------------------------------------------------------------
    def freeze_yielding(self, agent_list, yielding_drones):
        """Advance each yielding drone one step along its holding orbit.

        Replaces the parent's zero-velocity freeze with a continuous
        circular motion.  The orbit state is reset when a drone leaves
        the yield set (so re-entry starts a fresh orbit).
        """
        # Remove stale orbit state for drones that are no longer yielding
        stale_keys = [j for j in list(self._orbit_state)
                      if j not in yielding_drones]
        for j in stale_keys:
            del self._orbit_state[j]

        for j in yielding_drones:
            # Initialise orbit the first step this drone yields
            if j not in self._orbit_state:
                self._orbit_state[j] = {
                    'center': agent_list[j].p.copy(),
                    'angle': 0.0,
                }

            state = self._orbit_state[j]
            state['angle'] += self._orbit_speed

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
