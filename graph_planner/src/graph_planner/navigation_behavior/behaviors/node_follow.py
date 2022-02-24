from logging import getLogger
from typing import List, Optional, Sequence

import numpy as np

from graph_map.area import PoseLike
from graph_map.node_graph_manager import NodeGraphManager
from .behavioral_policy import BehavioralPolicy
from .control_policies.control_policy import ControlPolicy, ControlPolicyOutcome, DummyControl
from .costmap_policies.costmap_policy import CostmapPolicy, CostmapPolicyOutcome, DummyCostmap
from .waypoint_policies.next_node import NextNode
from .waypoint_policies.waypoint_policy import WaypointPolicy, WaypointPolicyOutcome

logger = getLogger(__name__)


def pose_dist(a: PoseLike, b: PoseLike):
    return np.linalg.norm(np.array([a.x, a.y]) - np.array([b.x, b.y]), 2)


def lin_interp(start: PoseLike, end: PoseLike, target_dist: float):
    dist_ratio = target_dist / pose_dist(start, end)
    new_goal = (1 - dist_ratio) * start.x + dist_ratio * end.x, \
               (1 - dist_ratio) * start.y + dist_ratio * end.y

    return PoseLike(*new_goal)


class NodeFollow(BehavioralPolicy):
    def __init__(self, max_dist=10, goal_radius=0.01, node_radius=4, **kwargs):
        super(NodeFollow, self).__init__(**kwargs)
        self.goal_radius = goal_radius
        self.node_radius = node_radius  # in metres
        self.max_dist = max_dist  # the maximum distance you can plan to, use lin_interp to get the next point

        self._remaining_path = None
        self.__gm: Optional[NodeGraphManager] = None
        self.__path: Optional[Sequence[List]] = None
        self.__zones = None
        self.__goal: Optional[WaypointPolicy] = None

        self.waypoint_policy: Optional[WaypointPolicy] = NextNode(shared_data=self.shared_data)
        self.costmap_policy: Optional[CostmapPolicy] = DummyCostmap(shared_data=self.shared_data)
        self.control_policy: Optional[ControlPolicy] = DummyControl(shared_data=self.shared_data)

    def calculate_waypoint(self, cur_pose: PoseLike) -> WaypointPolicyOutcome:
        """
        Go to the next closest node in the path
        Each time this is called, the remaining path in calculated by starting at the closest node and trimming
        the start of the path.
        If the path is only the final node then go to the goal
        """
        waypoint_outcome = self.waypoint_policy.calculate_goal(cur_pose)

        return waypoint_outcome

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        """
        Always returns True
        """
        return True

    def calculate_costmap(self) -> CostmapPolicyOutcome:
        return self.costmap_policy.calculate_costmap()

    def calculate_control(self) -> ControlPolicyOutcome:
        return self.control_policy.calculate_control()

    def clear_state(self):
        self._remaining_path = None
        self.__gm = None
        self.__path = None
        self.__zones = None

        self.waypoint_policy.clear_state()

    @property
    def gm(self):
        if self.__gm is None or self.shared_data_updated:
            self.__gm = self.shared_data.gm
        return self.__gm

    @property
    def zones(self):
        if self.__zones is None or self.shared_data_updated:
            self.__zones = self.shared_data.zones
        return self.__zones

    @property
    def path(self):
        if self.__path is None or self.shared_data_updated:
            self.__path = self.shared_data.path
        return self.__path

    @property
    def goal(self):
        if self.__goal is None or self.shared_data_updated:
            self.__goal = self.shared_data.goal
        return self.__goal
