"""Trajectory planner track registry.

The final submission exposes several planner modes through the command
line. Only ``baseline`` is a complete controller today; the other entries
reserve stable names for teammate implementations.
"""

from .baseline import TrajectoryPlannerController
from .registry import SUPPORTED_TRAJECTORY_MODES, build_trajectory_controller

__all__ = [
    "SUPPORTED_TRAJECTORY_MODES",
    "TrajectoryPlannerController",
    "build_trajectory_controller",
]
