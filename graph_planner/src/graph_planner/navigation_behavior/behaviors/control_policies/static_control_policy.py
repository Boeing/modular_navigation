from graph_planner.navigation_behavior.behaviors.control_policy import ControlPolicy, ControlParameters
from graph_planner.navigation_behavior.behaviors.control_policy import ControlPolicyOutcome
from graph_planner.navigation_behavior.behaviors.control_policy import VelocityLimits, CostMultipliers

from graph_map.area import PoseLike


class StaticControl(ControlPolicy):
    def __init__(self,
                 max_velocity_x: float = None,
                 max_velocity_y: float = None,
                 max_velocity_w: float = None,
                 avoid_distance: float = None,
                 backwards_mult: float = None,
                 strafe_mult: float = None,
                 rotation_mult: float = None,
                 **kwargs):
        super(StaticControl, self).__init__(**kwargs)

        if max_velocity_x is None or max_velocity_x < 0.0:
            max_velocity_x = 0.0
        if max_velocity_y is None or max_velocity_y < 0.0:
            max_velocity_y = 0.0
        if max_velocity_w is None or max_velocity_w < 0.0:
            max_velocity_w = 0.0

        if avoid_distance is None or avoid_distance < 0.0:
            avoid_distance = 0.0

        if backwards_mult is None or backwards_mult < 0.0:
            backwards_mult = 0.0
        if strafe_mult is None or strafe_mult < 0.0:
            strafe_mult = 0.0
        if rotation_mult is None or rotation_mult < 0.0:
            rotation_mult = 0.0

        self.control_parameters = ControlParameters(
            max_velocity=VelocityLimits(max_velocity_x, max_velocity_y, max_velocity_w),
            avoid_distance=avoid_distance,
            multipliers=CostMultipliers(backwards_mult, strafe_mult, rotation_mult)
        )

    def calculate_control(self, pose: PoseLike) -> ControlPolicyOutcome:
        return ControlPolicyOutcome(parameters=self.control_parameters)
