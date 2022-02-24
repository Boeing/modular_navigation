from abc import abstractmethod
from copy import copy
from typing import Optional, Union

from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors.control_policies.control_policy import ControlPolicy
from graph_planner.navigation_behavior.behaviors.control_policies.control_policy import ControlPolicyOutcome
from graph_planner.navigation_behavior.behaviors.costmap_policies.costmap_policy import CostmapPolicy
from graph_planner.navigation_behavior.behaviors.costmap_policies.costmap_policy import CostmapPolicyOutcome
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from graph_planner.navigation_behavior.behaviors.shared_data import SharedData
from graph_planner.navigation_behavior.behaviors.waypoint_policies.waypoint_policy import WaypointPolicy, \
    WaypointPolicyOutcome


class BehaviorOutcome:
    def __init__(self,
                 status: Union[Status, str] = Status.UNKNOWN,
                 waypoint: Optional[WaypointPolicyOutcome] = None,
                 costmap: Optional[CostmapPolicyOutcome] = None,
                 control: Optional[ControlPolicyOutcome] = None,
                 error=None):
        self.waypoint: Optional[WaypointPolicyOutcome] = waypoint  # Next target, can be None
        self.costmap: Optional[CostmapPolicyOutcome] = costmap
        self.control: Optional[ControlPolicyOutcome] = control
        self.status: Status = Status(status)  # True if goal reach
        self.error = error  # Failed to calculate, reports the type of error


class BehavioralPolicy:
    def __init__(self, name: str = None, shared_data: SharedData = None):
        self.__name = name
        self.shared_data: SharedData = shared_data
        self.__path = None
        self.__shared_data_updated: bool = False
        self.__last_outcome: BehaviorOutcome = None
        self.waypoint_policy: Optional[WaypointPolicy] = None
        self.costmap_policy: Optional[CostmapPolicy] = None
        self.control_policy: Optional[ControlPolicy] = None

    def path_synced(self):
        return self.__path == self.shared_data.path

    def set_updated(self):
        self.__shared_data_updated = True

        if self.waypoint_policy is not None:
            self.waypoint_policy.shared_data_updated = True
        if self.control_policy is not None:
            self.costmap_policy.shared_data_updated = True
        if self.costmap_policy is not None:
            self.control_policy.shared_data_updated = True

    def set_shared_data(self, shared_data: SharedData):
        self.shared_data = shared_data
        if self.waypoint_policy is not None:
            self.waypoint_policy.set_shared_data(shared_data)
        if self.control_policy is not None:
            self.control_policy.set_shared_data(shared_data)
        if self.costmap_policy is not None:
            self.costmap_policy.set_shared_data(shared_data)

        self.__shared_data_updated = True

    def run(self, cur_pose: PoseLike):
        if self.conditions_met(cur_pose):
            costmap = self.calculate_costmap()
            control = self.calculate_control()
            waypoint = self.calculate_waypoint(cur_pose)

            if waypoint is not None:
                status = waypoint.status
            elif control is not None:
                status = control.status
            elif costmap is not None:
                status = costmap.status
            else:
                Status.UNKNOWN

            self.__last_outcome = BehaviorOutcome(status=status,
                                                  waypoint=waypoint,
                                                  control=control,
                                                  costmap=costmap)
        else:
            self.__last_outcome = BehaviorOutcome(status=Status.NOT_RUNNING)

        return self.__last_outcome

    @property
    def name(self) -> str:
        if self.__name:
            return copy(self.__name)
        return self.__class__.__name__

    @property
    def last_outcome(self):
        return self.__last_outcome

    @property
    def shared_data_updated(self):
        return self.__shared_data_updated

    @abstractmethod
    def calculate_waypoint(self, cur_pose: PoseLike) -> WaypointPolicyOutcome:
        raise NotImplementedError

    @abstractmethod
    def conditions_met(self, cur_pose: PoseLike) -> bool:
        """
        Define a set of conditions that must be met for this behavior to be evaluated
        """
        raise NotImplementedError

    @abstractmethod
    def calculate_costmap(self) -> CostmapPolicyOutcome:
        raise NotImplementedError

    @abstractmethod
    def calculate_control(self) -> ControlPolicyOutcome:
        raise NotImplementedError

    @abstractmethod
    def clear_state(self):
        """
        Clear any internal state that is used to calculate the next goal
        """
        raise NotImplementedError
