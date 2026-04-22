"""
Phase 6 - Round-Trip Controller

Drones don't just arrive at the pad; they land, unload for a few steps,
then fly back to their origin. The pad becomes a two-way bottleneck in a
continuous Source -> Pad -> Source delivery loop.

Per-drone trip state machine:

    INBOUND  -> drone is heading to the pad (competes for landing slot)
    UNLOADING -> sitting on the pad with v=0 for `unload_steps` ticks
    OUTBOUND  -> heading back to its return point (NOT competing for pad)
    DONE      -> all `n_trips` round trips finished; teleported off-screen

Only INBOUND drones participate in Phase 4 negotiation. OUTBOUND drones
are still processed by MPC (so they actually fly home) and are visible
to other drones as moving obstacles, but they never yield to the pad.

Usage:
    controller = RoundTripController(
        cargo_configs,
        return_points=[start positions, one per drone],
        n_trips=2,
        unload_steps=5,
        ... (orbit/negotiation params inherited)
    )
    controller.bind(target_array)   # so the controller can swap goals
"""

import numpy as np

import SET
from negotiation_controller import NegotiationController
from landing_pad import PAD_CENTER

INBOUND   = 'INBOUND'
UNLOADING = 'UNLOADING'
OUTBOUND  = 'OUTBOUND'
DONE      = 'DONE'


class RoundTripController(NegotiationController):
    """Phase 6: round-trip lifecycle on top of Phase 4 negotiation."""

    def __init__(self, cargo_configs=None, return_points=None,
                 n_trips=2, unload_steps=5,
                 orbit_radius=0.7, orbit_speed=0.15, safe_distance=1.2,
                 nominal_speed=0.1, eta_threshold=0.15, use_hysteresis=True):
        super().__init__(cargo_configs, orbit_radius, orbit_speed,
                         safe_distance, nominal_speed, eta_threshold,
                         use_hysteresis=use_hysteresis)
        self._return_points    = [np.array(rp, dtype=float)
                                  for rp in (return_points or [])]
        self._n_trips          = max(1, int(n_trips))
        self._unload_steps     = max(0, int(unload_steps))
        self._state            = {}    # j -> state string
        self._unload_remaining = {}    # j -> int
        self._trips_done       = {}    # j -> int
        self._target_ref       = None  # bound from test.py before main loop
        self._initialized      = False

    # ------------------------------------------------------------------
    # External wiring
    # ------------------------------------------------------------------
    def bind(self, target_array):
        """Give the controller a handle to test.py's per-drone target list,
        so it can swap goals between PAD_CENTER and the return point."""
        self._target_ref = target_array

    def _ensure_init(self, num_moving_drones):
        if self._initialized:
            return
        for j in range(num_moving_drones):
            self._state[j]            = INBOUND
            self._unload_remaining[j] = 0
            self._trips_done[j]       = 0
        self._initialized = True

    # ------------------------------------------------------------------
    # Override: only INBOUND drones compete for the pad
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        self._ensure_init(len(agent_list))
        inbound = [j for j in active_drones
                   if self._state.get(j, INBOUND) == INBOUND]
        if not inbound:
            # Nobody wants to land right now; OUTBOUND drones just fly home
            # without yielding (empty yielding set lets MPC run on them).
            return {"allowed": None, "yielding": set(),
                    "method": "no_inbound", "scores": {}}

        # If the pad is occupied by an UNLOADING drone, no INBOUND drone
        # may approach -- everyone orbits until the pad clears. This
        # prevents MPC infeasibility from trying to land on an occupied pad.
        pad_busy = any(s == UNLOADING for s in self._state.values())
        if pad_busy:
            return {"allowed": None, "yielding": set(inbound),
                    "method": "pad_busy", "scores": {}}

        return super().select_active_drone(agent_list, inbound, step, verbose)

    # ------------------------------------------------------------------
    # Override: do not teleport on landing - manage round-trip FSM
    # ------------------------------------------------------------------
    def cleanup_landed(self, agent_list, target_reached, num_moving_drones, K):
        self._ensure_init(num_moving_drones)

        for j in range(num_moving_drones):
            if not target_reached[j]:
                continue
            state = self._state[j]

            if state == INBOUND:
                # Just touched down at the pad - start the unload timer.
                self._state[j]            = UNLOADING
                self._unload_remaining[j] = self._unload_steps
                self._park(agent_list[j], agent_list[j].p, K)
                print(f"  [Phase 6] Drone {j}: INBOUND -> UNLOADING "
                      f"(trip {self._trips_done[j] + 1}/{self._n_trips})")

            elif state == OUTBOUND:
                # Just reached the return point - count the trip.
                self._trips_done[j] += 1
                if self._trips_done[j] >= self._n_trips:
                    self._state[j] = DONE
                    self._teleport_away(agent_list[j], K)
                    print(f"  [Phase 6] Drone {j}: OUTBOUND -> DONE "
                          f"(completed {self._trips_done[j]} trips)")
                else:
                    # Begin the next inbound leg.
                    self._state[j] = INBOUND
                    self._switch_target(j, agent_list[j], target_reached,
                                        PAD_CENTER)
                    print(f"  [Phase 6] Drone {j}: OUTBOUND -> INBOUND "
                          f"(starting trip {self._trips_done[j] + 1})")

            elif state == DONE:
                # Keep parked off-screen.
                self._teleport_away(agent_list[j], K)

            # UNLOADING: handled by step_update countdown.

    # ------------------------------------------------------------------
    # Override: countdown unload, dispatch outbound
    # ------------------------------------------------------------------
    def step_update(self, agent_list, target_reached, num_moving_drones):
        super().step_update(agent_list, target_reached, num_moving_drones)
        self._ensure_init(num_moving_drones)

        for j in range(num_moving_drones):
            if self._state[j] != UNLOADING:
                continue
            self._unload_remaining[j] -= 1
            if self._unload_remaining[j] <= 0:
                self._state[j] = OUTBOUND
                rp = (self._return_points[j]
                      if j < len(self._return_points) else PAD_CENTER)
                self._switch_target(j, agent_list[j], target_reached, rp)
                print(f"  [Phase 6] Drone {j}: UNLOADING -> OUTBOUND "
                      f"(heading to {rp.tolist()})")

    # ------------------------------------------------------------------
    # Override: only end the simulation when EVERY drone has finished all trips
    # ------------------------------------------------------------------
    def all_finished(self, target_reached, num_moving_drones):
        if not self._initialized:
            return False
        return all(self._state.get(j) == DONE
                   for j in range(num_moving_drones))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _teleport_away(agent, K):
        agent.p     = np.array([100.0, 100.0])
        agent.v     = np.zeros(2)
        agent.state = np.append(agent.p, agent.v)
        agent.pre_traj = np.tile(np.array([100.0, 100.0]), (K + 1, 1))

    @staticmethod
    def _park(agent, position, K):
        """Freeze the agent in place at `position` (used during UNLOADING)."""
        agent.p     = np.array(position, dtype=float)
        agent.v     = np.zeros(2)
        agent.state = np.append(agent.p, agent.v)
        agent.pre_traj = np.tile(agent.p, (K + 1, 1))

    def _switch_target(self, j, agent, target_reached, new_target):
        """Switch this drone's goal to `new_target` and re-arm MPC."""
        new_target = np.array(new_target, dtype=float)
        agent.change_target(new_target.copy())
        if self._target_ref is not None:
            self._target_ref[j] = new_target.copy()
        # IMPORTANT: run.run_one_agent re-asserts SET.target[j] every MPC
        # step, so we must update the SET-level target too -- otherwise the
        # change_target() above is overwritten on the next step and the
        # drone keeps trying to reach the original goal.
        if j < len(SET.target):
            SET.target[j] = new_target.copy()
        target_reached[j] = False
        # Warm-start the MPC trajectory from the current position so the
        # first solve doesn't dart backwards toward the old goal.
        agent.pre_traj  = np.tile(agent.p, (agent.K + 1, 1))
        agent.cost_index = agent.K
        # Clear the MPC's "I've converged on the old target" termination
        # flags. Without this reset the optimiser thinks it is already at
        # goal and refuses to plan a fresh trajectory toward `new_target`.
        agent.term_overlap       = False
        agent.term_overlap_again = False
        agent.term_index         = 0
        agent.eta                = 1.0
        agent.term_last_pos      = agent.p.copy()
