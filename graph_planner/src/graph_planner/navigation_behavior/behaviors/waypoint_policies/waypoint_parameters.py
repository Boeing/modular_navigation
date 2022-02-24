from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors.policy.parameters import BaseParameters


class PoseTolerance:
    def __init__(self, xy_goal_tolerance: float, yaw_goal_tolerance: float):
        self.xy_goal_tolerance = xy_goal_tolerance
        self.yaw_goal_tolerance = yaw_goal_tolerance


class PoseSampling:
    def __init__(self, std_x, std_y, std_w, max_samples):
        self.std_x = std_x
        self.std_y = std_y
        self.std_w = std_w
        self.max_samples = max_samples


class WaypointParameters(BaseParameters):
    def __init__(self, pose: PoseLike, tolerance: PoseTolerance, sampling: PoseSampling):
        self.pose: PoseLike = pose
        self.tolerance: PoseTolerance = tolerance
        self.sampling: PoseSampling = sampling

    def __str__(self):
        return str(self.pose)
