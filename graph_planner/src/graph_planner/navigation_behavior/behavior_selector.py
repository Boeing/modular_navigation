
from typing import List, Any, Sequence

from graph_map.node import Node
from graph_map.node_graph_manager import NodeGraphManager
from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors.behavioral_policy import BehaviorOutcome, BehavioralPolicy
from graph_planner.navigation_behavior.behaviors.waypoint_policies.waypoint_policy import WaypointPolicy

from logging import getLogger
logger = getLogger(__name__)


class BehaviourSelector:
    def __init__(self, shared_data=None, default_behavior=None,
                 default_waypoint=None, default_control=None, default_costmap=None):
        self.default_behavior = default_behavior

        # [next_node, open_door, drive_like_pedestrians, drive_like_vehicle]
        self.__behaviors: List[BehavioralPolicy] = list()
        self.__conditions = list()  # [next_node_conditions, open_door_condition]

        self.shared_data = shared_data

        # Default parameters if none of the behaviours set them
        self.__default_waypoint = default_waypoint
        self.__default_control = default_control
        self.__default_costmap = default_costmap

    def register_behavior(self, behavior: BehavioralPolicy):
        behavior.set_shared_data(self.shared_data)
        self.__behaviors.append(behavior)
        self.update_behaviors()

    def update_map_data(self, gm: NodeGraphManager, zones: Any):
        self.shared_data.update(gm=gm, zones=zones)
        self.update_behaviors()

    def update_goal(self, goal: WaypointPolicy):
        self.shared_data.update(goal=goal)
        self.update_behaviors()

    @property
    def behaviors(self):
        return self.__behaviors

    def update_behaviors(self):
        for behavior in self.behaviors:
            behavior.set_updated()

    def clear_states(self):
        for behavior in self.behaviors:
            behavior.clear_state()

    def update_graph_plan(self, plan: Sequence[Node]):
        logger.info('Running update_graph_plan')
        self.shared_data.update(path=plan)

        # Make sure to tell all the behaviors that something has changed
        for behavior in self.__behaviors:
            behavior.set_updated()
        pass

    def update(self, pose: PoseLike) -> BehaviorOutcome:
        # Choose behavior and get parameters

        # For now, just always use the first one that meets the conditions
        combined_outcome = BehaviorOutcome()
        for behavior in self.behaviors:
            try:
                outcome: BehaviorOutcome = behavior.run(pose)
                logger.info('{} status: {}'.format(behavior.name, outcome.status))

                if combined_outcome.waypoint is None and outcome.waypoint is not None:
                    if outcome.waypoint.parameters is not None:
                        logger.info('Using Waypoint outcome from: {}'.format(behavior.name))
                        combined_outcome.waypoint = outcome.waypoint
                        combined_outcome.status = outcome.status
                if combined_outcome.control is None and outcome.control is not None:
                    if outcome.control.parameters is not None:
                        logger.info('Using Control outcome from: {}'.format(behavior.name))
                        combined_outcome.control = outcome.control
                if combined_outcome.costmap is None and outcome.costmap is not None:
                    if outcome.costmap.parameters is not None:
                        logger.info('Using Costmap outcome from: {}'.format(behavior.name))
                        combined_outcome.costmap = outcome.costmap

                if combined_outcome.waypoint is None:
                    combined_outcome.waypoint = self.__default_waypoint
                if combined_outcome.control is None:
                    combined_outcome.control = self.__default_control
                if combined_outcome.costmap is None:
                    combined_outcome.costmap = self.__default_costmap
            except Exception as e:
                logger.error('Behavior "{}" failed with: {}'.format(behavior.name, e))

        return combined_outcome
