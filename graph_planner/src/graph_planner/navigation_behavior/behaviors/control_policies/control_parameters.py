from typing import Optional
from graph_planner.navigation_behavior.behaviors.policy.parameters import BaseParameters


class VelocityLimits:
    def __init__(self, x: float = None, y: float = None, w: float = None):
        self.x = x
        self.y = y
        self.w = w


class CostMultipliers:
    def __init__(self, backwards_mult: float = None, strafe_mult: float = None, rotation_mult: float = None):
        self.backwards_mult = backwards_mult
        self.strafe_mult = strafe_mult
        self.rotation_mult = rotation_mult


class ControlParameters(BaseParameters):
    def __init__(self, max_velocity: Optional[VelocityLimits] = None,
                 avoid_distance: Optional[float] = None,
                 multipliers: Optional[CostMultipliers] = None):
        if max_velocity is None:
            self.max_velocity: VelocityLimits = VelocityLimits()
        else:
            self.max_velocity = max_velocity

        self.avoid_distance: float = avoid_distance

        if multipliers is None:
            self.multipliers: CostMultipliers = CostMultipliers()
        else:
            self.multipliers = multipliers

    def __str__(self):
        return 'Max velocities: ({}, {}, {})'.format(self.max_velocity.x, self.max_velocity.y, self.max_velocity.w) +\
            ', Avoid distance: {}'.format(self.avoid_distance) +\
            ', Backwards mult: {}'.format(self.multipliers.backwards_mult) +\
            ', Strafe mult: {}'.format(self.multipliers.strafe_mult) +\
            ', Rotation mult: {}'.format(self.multipliers.rotation_mult)
