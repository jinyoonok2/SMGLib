"""
Track 1 — policy/yield controller wiring.

``build_policy_yield_controller`` is the single integration point for landing-pad
runs that use the policy/yield stack. It picks the shallowest concrete
controller that matches the scenario flags passed from ``app2_standardized.py``.

The underlying classes stay modular (inheritance stack). This module only
removes scattered construction logic from ``test.py`` and avoids naming runs
by historical phase labels.
"""

from __future__ import annotations

from landing_pad import LandingPadController
from negotiation_controller import NegotiationController
from orbit_controller import OrbitController
from priority_manager import PriorityManager
from round_trip_controller import RoundTripController


def build_policy_yield_controller(
    ini_x,
    target,
    num_moving_drones,
    cargo_configs=None,
    orbit_params=None,
    negotiation_params=None,
    round_trip_params=None,
):
    """
    Build the landing-pad controller for the policy/yield track.

    Resolution order (first match wins):

    1. No cargo metadata → closest-first baseline (``LandingPadController``).
    2. ``round_trip_params`` set → ``RoundTripController`` (binds ``target``).
    3. ``negotiation_params`` set → ``NegotiationController``.
    4. ``orbit_params`` set → ``OrbitController``.
    5. Otherwise → ``PriorityManager`` (priority scoring only).

    Parameters
    ----------
    ini_x, target
        Initial positions and goal list from ``PLAN`` (``target`` is mutated
        by ``RoundTripController.bind`` when round-trip is active).
    num_moving_drones : int
    cargo_configs, orbit_params, negotiation_params, round_trip_params
        Optional dicts from scenario JSON; same schema as today.
    """
    if not cargo_configs:
        return LandingPadController()

    if round_trip_params:
        ctrl = RoundTripController(
            cargo_configs,
            return_points=round_trip_params.get(
                "return_points",
                [ini_x[i] for i in range(num_moving_drones)],
            ),
            n_trips=round_trip_params.get("n_trips", 2),
            unload_steps=round_trip_params.get("unload_steps", 5),
            orbit_radius=round_trip_params.get("orbit_radius", 0.7),
            orbit_speed=round_trip_params.get("orbit_speed", 0.15),
            safe_distance=round_trip_params.get("safe_distance", 1.2),
            nominal_speed=round_trip_params.get("nominal_speed", 0.1),
            eta_threshold=round_trip_params.get("eta_threshold", 0.15),
            use_hysteresis=round_trip_params.get("use_hysteresis", True),
        )
        ctrl.bind(target)
        return ctrl

    if negotiation_params:
        return NegotiationController(
            cargo_configs,
            orbit_radius=negotiation_params.get("orbit_radius", 0.7),
            orbit_speed=negotiation_params.get("orbit_speed", 0.15),
            safe_distance=negotiation_params.get("safe_distance", 1.2),
            nominal_speed=negotiation_params.get("nominal_speed", 0.1),
            eta_threshold=negotiation_params.get("eta_threshold", 0.15),
            use_hysteresis=negotiation_params.get("use_hysteresis", True),
        )

    if orbit_params:
        return OrbitController(
            cargo_configs,
            orbit_radius=orbit_params.get("orbit_radius", 0.7),
            orbit_speed=orbit_params.get("orbit_speed", 0.15),
            use_hysteresis=orbit_params.get("use_hysteresis", True),
        )

    return PriorityManager(cargo_configs)
