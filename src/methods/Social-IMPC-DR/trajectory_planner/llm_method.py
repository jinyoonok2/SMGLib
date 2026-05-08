"""Behavior-changing LLM trajectory-planner method.

This method keeps the Track 2 speed-scaling trajectory planner, but uses
the LLM advisor in score-adjust mode before ranking inbound drones. The
LLM does not directly control motion or MPC. It only adjusts priority
scores, and the deterministic planner still computes arrival order, Vmax,
pad spacing, lifecycle transitions, and safety behavior.
"""

from .baseline import TrajectoryPlannerController


class LLMScoreAdjustTrajectoryPlannerController(TrajectoryPlannerController):
    """Trajectory planner variant using LLM-adjusted priority scores."""

    pass