from abc import abstractmethod
from typing import Optional
from nav_msgs.msg import OccupancyGrid

from graph_planner.navigation_behavior.behaviors.policy.error import BaseError
from graph_planner.navigation_behavior.behaviors.policy.policy import BasePolicyOutcome, BasePolicy
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from graph_planner.navigation_behavior.behaviors.costmap_policies.costmap_parameters import CostmapParameters


class CostmapPolicyOutcome(BasePolicyOutcome):
    def __init__(self, status: Optional[Status], parameters: Optional[CostmapParameters] = None,
                 error: Optional[BaseError] = None):
        super(CostmapPolicyOutcome, self).__init__(status=status, parameters=parameters, error=error)


class CostmapPolicy(BasePolicy):
    def __init__(self, **kwargs):
        super(CostmapPolicy, self).__init__(**kwargs)

        # Use case:
        #  Likely to use zones to update costmap weights in majority of cases, unlikely to need finer control
        #  Finer control may be need for job specific movement, e.g. prioritise moving away back away from the tool
        self.costmap: OccupancyGrid = OccupancyGrid()  # iF we want complex cost interactions use this
        self.zones = None  # Able to modify costs of zones
        pass

    @abstractmethod
    def calculate_costmap(self) -> CostmapPolicyOutcome:
        raise NotImplementedError


class DummyCostmap(CostmapPolicy):
    def __init__(self, **kwargs):
        super(DummyCostmap, self).__init__(**kwargs)

    def calculate_costmap(self) -> CostmapPolicyOutcome:
        return CostmapPolicyOutcome(
            status=Status.RUNNING,
            parameters=CostmapParameters(),
            error=None
        )
