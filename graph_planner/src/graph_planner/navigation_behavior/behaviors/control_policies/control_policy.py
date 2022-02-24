from abc import abstractmethod
from typing import Optional

from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors.policy.error import BaseError
from graph_planner.navigation_behavior.behaviors.policy.policy import BasePolicyOutcome, BasePolicy
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from graph_planner.navigation_behavior.behaviors.control_policies.control_parameters import ControlParameters
from graph_planner.navigation_behavior.behaviors.control_policies.control_parameters import VelocityLimits
from graph_planner.navigation_behavior.behaviors.control_policies.control_parameters import CostMultipliers


class ControlPolicyOutcome(BasePolicyOutcome):
    def __init__(self, status: Optional[Status], parameters: Optional[ControlParameters] = None,
                 error: Optional[BaseError] = None):
        super(ControlPolicyOutcome, self).__init__(status=status, parameters=parameters, error=error)


class ControlPolicy(BasePolicy):
    def __init__(self, **kwargs):
        super(ControlPolicy, self).__init__(**kwargs)

    @abstractmethod
    def calculate_control(self, pose: PoseLike) -> ControlPolicyOutcome:
        raise NotImplementedError


class DummyControl(ControlPolicy):
    def __init__(self, **kwargs):
        super(DummyControl, self).__init__(**kwargs)

    def calculate_control(self) -> ControlPolicyOutcome:
        return ControlPolicyOutcome(
            status=Status.RUNNING,
            parameters=ControlParameters(
                max_velocity=VelocityLimits(0.0, 0.0, 0.0),
                avoid_distance=0.0,
                multipliers=CostMultipliers(0.0, 0.0, 0.0)
            ),
            error=None
        )
