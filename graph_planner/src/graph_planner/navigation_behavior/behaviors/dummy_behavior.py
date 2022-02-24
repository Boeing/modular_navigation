from typing import Optional

from graph_map.util import PoseLike
from graph_planner.navigation_behavior.behaviors import BehavioralPolicy
from graph_planner.navigation_behavior.behaviors.behavioral_policy import BehaviorOutcome
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from graph_planner.navigation_behavior.behaviors.shared_data import SharedData


class PoseShared(SharedData):
    def __init__(self, pose: Optional[PoseLike] = None, **kwargs):
        super(PoseShared, self).__init__(**kwargs)
        self.pose = pose

    def update(self, pose, **kwargs):
        super(PoseShared, self).update(**kwargs)
        self.pose = pose

    def clear(self):
        super(PoseShared, self).clear()
        with self.__lock:
            self.pose = None


class DummyBee(BehavioralPolicy):
    def __init__(self, name, **kwargs):
        super(DummyBee, self).__init__(**kwargs)

        self.name = name

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        for zone in self.shared_data.zones:
            if zone.is_in(cur_pose):
                print(zone.display_name)
        return True

    def run(self, cur_pose: PoseLike):
        pass

    def clear_state(self):
        return

    def calculate_waypoint(self, cur_pose: PoseLike) -> BehaviorOutcome:
        goal = cur_pose

        outcome = BehaviorOutcome(waypoint=goal, status=Status.RUNNING)
        return outcome


class DummyTwo(BehavioralPolicy):
    def __init__(self, name, **kwargs):
        super(DummyTwo, self).__init__(**kwargs)

        self.name = name

    def run(self, cur_pose: PoseLike):
        # Check if conditions met

        # Set up goals

        # Set up costmap

        # Run goal generator
        pass

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        # How do we check the current pose???
        pass

    def clear_state(self):
        return

    def calculate_waypoint(self, cur_pose: PoseLike) -> BehaviorOutcome:
        goal = cur_pose

        outcome = BehaviorOutcome(waypoint=goal, status=Status.RUNNING)
        return outcome
