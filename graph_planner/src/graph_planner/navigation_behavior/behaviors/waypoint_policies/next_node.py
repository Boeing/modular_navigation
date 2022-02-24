from logging import getLogger
from typing import List, Optional, Sequence

import numpy as np
import rospy

from graph_map.area import PoseLike
from graph_map.node import Node
from graph_map.node_graph_manager import NodeGraphManager
from .waypoint_policy import WaypointPolicy, WaypointPolicyOutcome
from .waypoint_parameters import WaypointParameters, PoseSampling, PoseTolerance
from ..policy.error import WaypointPolicyError
from ..policy.status import Status

from visualization_msgs.msg import MarkerArray
from map_manager.visualise import build_graph_marker_array
from std_msgs.msg import ColorRGBA

logger = getLogger(__name__)


def pose_angle(a: PoseLike, b: PoseLike):
    if a.x == b.x and a.y == b.y:
        return 0.0
    else:
        return np.arctan2(b.y - a.y, b.x - a.x)


def pose_dist(a: PoseLike, b: PoseLike):
    return np.linalg.norm(np.array([a.x, a.y]) - np.array([b.x, b.y]), 2)


def lin_interp(start: PoseLike, end: PoseLike, target_dist: float):
    dist_ratio = target_dist / pose_dist(start, end)
    new_goal = (1 - dist_ratio) * start.x + dist_ratio * end.x, \
               (1 - dist_ratio) * start.y + dist_ratio * end.y

    return PoseLike(*new_goal, theta=end.theta)


class NextNode(WaypointPolicy):
    def __init__(self, max_dist=8, node_radius=3, **kwargs):
        super(NextNode, self).__init__(**kwargs)
        self.node_radius = node_radius  # acceptable distance to nodes in metres
        self.max_dist = max_dist  # the maximum distance you can plan to, use lin_interp to get the next point

        self._remaining_path: Optional[Sequence[Node]] = None
        self._last_goal_sent: bool = False

        self.__gm: Optional[NodeGraphManager] = None
        self.__path: Optional[Sequence[List]] = None
        self.__zones = None
        self.__goal: Optional[PoseLike] = None

        self.__remaining_path_pub = rospy.Publisher(
            '~remaining_path', data_class=MarkerArray, latch=True, queue_size=2)

    def closest_node_in_subgraph(self, pose):
        closest_node, closest_dist = self.gm.closest_node_in_list(pose, self.path)
        path_idx = self.path.index(closest_node)
        if closest_dist < self.node_radius:
            path_idx += 1
            # TOOD: Check transition to next node here

        self._remaining_path = self.path[path_idx:]

    @staticmethod
    def default_waypoint(pose: PoseLike):
        # Loose tolerances
        # 50cm, ~5.7 degrees
        sampling = PoseSampling(std_x=0.5, std_y=0.5, std_w=0.1, max_samples=15)
        # 30cm, ~5.7degrees
        tolerance = PoseTolerance(xy_goal_tolerance=0.3, yaw_goal_tolerance=0.1)

        waypoint = WaypointParameters(pose=pose, sampling=sampling, tolerance=tolerance)
        return waypoint

    def calculate_goal(self, cur_pose: PoseLike) -> WaypointPolicyOutcome:
        """
        Go to the next closest node in the path
        Each time this is called, the remaining path in calculated by starting at the closest node and trimming
        the start of the path.
        If the path is only the final node then go to the goal
        """
        status = Status.UNKNOWN
        waypoint = None
        error = None

        if self.path is None:
            # No path, fail
            status = Status.ERROR
            error = WaypointPolicyError('No path provided')
            return WaypointPolicyOutcome(status=status, error=error)

        elif self._remaining_path is None or \
                self.path[len(self.path) - len(self._remaining_path):] != self._remaining_path:
            # First node, use the path to tell you where to go
            closest_node, closest_dist = self.gm.closest_node_in_list(cur_pose, self.path)
            path_idx = self.path.index(closest_node)
            self._remaining_path = self.path[path_idx:]

            status = Status.RUNNING

        elif self.path[len(self.path) - len(self._remaining_path):] == self._remaining_path:
            # Chugging along
            status = Status.RUNNING

            closest_pose = PoseLike(self._remaining_path[0].x, self._remaining_path[0].y)
            closest_dist = pose_dist(cur_pose, closest_pose)

        else:
            # Error state, shouldn't be able to get here
            status = Status.ERROR
            error = WaypointPolicyError(
                'Path is in unknown state: {}, remaining: {}'.format(self.path, self._remaining_path))
            return WaypointPolicyOutcome(status=status, error=error)

        # Trim remaining path
        if len(self._remaining_path) >= 2 and closest_dist < self.node_radius:
            logger.info("next_node: Node reached, trimming from remaining path")
            # Remove the first node if we're close enough
            self._remaining_path = self._remaining_path[1:]
            # Recalculate closest node
            closest_pose = PoseLike(self._remaining_path[0].x, self._remaining_path[0].y)
            closest_dist = pose_dist(cur_pose, closest_pose)

        goal_dist = pose_dist(cur_pose, self.goal.pose)
        current_area = self.gm.in_area(cur_pose)
        goal_area = self.gm.in_area(self.goal.pose)

        self.__remaining_path_pub.publish(build_graph_marker_array(
            self.shared_data.gm.graph.subgraph(self._remaining_path),
            node_params={'color': ColorRGBA(0.8, 0.2, 0.2, 0.8), 'radius': 0.60},
            edge_params={'color': ColorRGBA(0.3, 0.1, 0, 0.6), 'diameter': 0.20}))

        # Go directly to goal if we are closer (but make sure it's the same area to prevent driving through wall) or
        # if we have reached the last node
        if (goal_dist < closest_dist and current_area is goal_area) or \
                (len(self._remaining_path) == 1 and closest_dist < self.node_radius):
            # Go straight to goal
            waypoint = self.goal
            logger.info('Sending last goal, {}'.format(closest_dist))
            self._last_goal_sent = True
        else:
            # Get the next node in the path
            next_node: Node = self._remaining_path[0]
            angle = pose_angle(cur_pose, PoseLike(*next_node.pos))
            pose = PoseLike(*next_node.pos, theta=angle)
            waypoint = self.default_waypoint(pose)

        # Make sure the goal is within the max planning distance
        if waypoint is not None:
            goal_dist = pose_dist(cur_pose, waypoint.pose)

            if goal_dist > self.max_dist:
                waypoint = self.default_waypoint(lin_interp(cur_pose, waypoint.pose, self.max_dist))
                self._last_goal_sent = False  # Waypoint has been interpolated, it's no longer the last goal
                status = Status.RUNNING

        if self._last_goal_sent:
            status = Status.SUCCESS

        outcome = WaypointPolicyOutcome(status=status, parameters=waypoint)

        return outcome

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        """
        Always returns True
        """
        return True

    def clear_state(self):
        self._last_goal_sent = False
        self._remaining_path = None
        self.__gm = None
        self.__path = None
        self.__zones = None

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
