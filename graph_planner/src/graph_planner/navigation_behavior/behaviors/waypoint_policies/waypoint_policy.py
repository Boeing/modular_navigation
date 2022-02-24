from abc import abstractmethod
from typing import Optional

from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors.waypoint_policies.waypoint_parameters import WaypointParameters
from graph_planner.navigation_behavior.behaviors.policy.error import BaseError
from graph_planner.navigation_behavior.behaviors.policy.policy import BasePolicyOutcome, BasePolicy
from graph_planner.navigation_behavior.behaviors.policy.status import Status


class WaypointPolicyOutcome(BasePolicyOutcome):
    def __init__(self, status: Optional[Status], parameters: Optional[WaypointParameters] = None,
                 error: Optional[BaseError] = None):
        super(WaypointPolicyOutcome, self).__init__(status=status, parameters=parameters, error=error)


class WaypointPolicy(BasePolicy):
    def __init__(self, **kwargs):
        super(WaypointPolicy, self).__init__(**kwargs)

    @abstractmethod
    def calculate_goal(self, pose: PoseLike) -> WaypointPolicyOutcome:
        raise NotImplementedError
