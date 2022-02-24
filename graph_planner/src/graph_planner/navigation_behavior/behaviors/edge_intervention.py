from logging import getLogger
from typing import List, Optional, Sequence

from graph_map.area import PoseLike
from graph_map.node_graph_manager import NodeGraphManager
from .behavioral_policy import BehavioralPolicy
from .control_policies.control_policy import ControlPolicy
from .costmap_policies.costmap_policy import CostmapPolicy
from .waypoint_policies.wait_for_trigger_policy import WaitForTrigger
from .waypoint_policies.waypoint_policy import WaypointPolicy, WaypointPolicyOutcome
from .policy.status import Status

logger = getLogger(__name__)


class EdgeIntervention(BehavioralPolicy):
    def __init__(self, **kwargs):
        super(EdgeIntervention, self).__init__(**kwargs)

        self.__gm: Optional[NodeGraphManager] = None
        self.__path: Optional[Sequence[List]] = None
        self.__zones = None
        self.__goal: Optional[WaypointPolicy] = None

        self.waypoint_policy: Optional[WaypointPolicy] = WaitForTrigger(shared_data=self.shared_data,
                                                                        service='~' + self.name + '_trigger')
        self.costmap_policy: Optional[CostmapPolicy] = None
        self.control_policy: Optional[ControlPolicy] = None

        self.__wait_node = None  # Place to wait for the intervention
        self.__target_node = None  # Node we want to get to

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        # Check if next edge has the intervention attribute
        closest_node, _ = self.gm.closest_node_in_list(cur_pose, self.path, check_area=False)
        path_idx = self.path.index(closest_node)

        # As soon as it's started, keep running until we have reached the target node (other end of the edge)
        if self.last_outcome is not None:
            if self.last_outcome.status == Status.RUNNING or self.last_outcome.status == Status.INTERVENTION or \
                    (self.last_outcome.status == Status.SUCCESS and closest_node is not self.__target_node):
                return True

        # Make sure we're not at the end of the path
        if path_idx < len(self.path) - 1:
            next_node = self.path[path_idx + 1]
            if 'intervention' in self.gm.graph[closest_node][next_node]:
                logger.info('Edge intervention attribute: {}'
                            .format(self.gm.graph[closest_node][next_node]['intervention']))
                if self.gm.graph[closest_node][next_node]['intervention'] is True:
                    self.__wait_node = self.__get_wait_node(cur_pose, closest_node, next_node)
                    self.__target_node = next_node
                    return True

        self.waypoint_policy.clear_state()
        return False

    def calculate_waypoint(self, cur_pose: PoseLike) -> WaypointPolicyOutcome:
        """
        Go to the next closest node in the path
        Each time this is called, the remaining path in calculated by starting at the closest node and trimming
        the start of the path.
        If the path is only the final node then go to the goal
        """

        waypoint_outcome = self.waypoint_policy.calculate_goal(cur_pose=cur_pose, wait_pose=self.__wait_node.pose)

        return waypoint_outcome

    def calculate_costmap(self) -> None:
        return None

    def calculate_control(self) -> None:
        return None

    def clear_state(self):
        self._remaining_path = None
        self.__gm = None
        self.__path = None
        self.__zones = None

        self.waypoint_policy.clear_state()

    @property
    def gm(self) -> NodeGraphManager:
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

    # Get wait position
    def __get_wait_node(self, cur_pose, closest_node, next_node):
        if 'wait_node' in self.gm.graph[closest_node][next_node]:
            wait_node_id = self.gm.graph[closest_node][next_node]['wait_node']
            if isinstance(wait_node_id, list):
                wait_nodes = list()
                for id in wait_node_id:
                    if id in self.gm.node_ids.keys():
                        node = self.gm.node_ids[id]
                        wait_nodes.append(node)
                    else:
                        logger.warning('Wait node "{}" does not exist'.format(id))

                if len(wait_nodes) == 0:
                    wait_node = closest_node
                else:
                    try:
                        wait_node, _ = self.gm.closest_node_in_list(cur_pose, wait_nodes, check_area=False)
                    except ValueError:
                        wait_node = closest_node

            else:
                if wait_node_id in self.gm.node_ids.keys():
                    wait_node = self.gm.node_ids[wait_node_id]
                else:
                    logger.warning('Wait node "{}" does not exist'.format(wait_node_id))
                    wait_node = closest_node
        else:
            wait_node = closest_node

        logger.info('Using as wait position: {}'.format(wait_node.id))
        return wait_node
