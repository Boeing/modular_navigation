from logging import getLogger

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from graph_map.area import PoseLike
from .waypoint_parameters import WaypointParameters, PoseSampling, PoseTolerance
from .waypoint_policy import WaypointPolicy, WaypointPolicyOutcome
from ..policy.error import WaypointPolicyError
from ..policy.status import Status

logger = getLogger(__name__)


class WaitForTrigger(WaypointPolicy):
    def __init__(self, service=None, **kwargs):
        super(WaitForTrigger, self).__init__(**kwargs)

        self.__triggered = False
        self.__status = Status.NOT_RUNNING

        if service is not None:
            self.__service = service
        else:
            self.__service = '~' + self.name + '_trigger'
        self.__trigger_service = None

    def calculate_goal(self, cur_pose: PoseLike, wait_pose: PoseLike) -> WaypointPolicyOutcome:
        """
        Drives to a pose (if provided)

        Set up a trigger and wait. Sets the waypoint to None with a status of INTERVENTION.
        When trigger is received, status is changed to SUCCESS.
        """

        # Loose tolerances
        # 20cm, ~5.7 degrees
        sampling = PoseSampling(std_x=0.2, std_y=0.2, std_w=0.1, max_samples=15)
        # 10cm, ~5.7degrees
        tolerance = PoseTolerance(xy_goal_tolerance=0.1, yaw_goal_tolerance=0.1)
        waypoint = None
        error = None

        # Drive to waypoint
        if wait_pose is not None and \
                (self.__status == Status.NOT_RUNNING or
                    (self.__status == Status.RUNNING and not self.shared_data.autonomy_complete)):
            waypoint = WaypointParameters(pose=wait_pose, sampling=sampling, tolerance=tolerance)
            self.__status = Status.RUNNING

        # Start intervention
        elif self.__status == Status.NOT_RUNNING and wait_pose is None or \
                self.__status == Status.RUNNING and self.shared_data.autonomy_complete:

            self.__triggered = False
            waypoint = WaypointParameters(pose=cur_pose, sampling=sampling, tolerance=tolerance)
            self.__status = Status.INTERVENTION

            # Register trigger service
            self.__trigger_service = rospy.Service(
                name=self.__service,
                service_class=Trigger,
                handler=self.__trigger_callback
            )

        # Waiting for intervention to be triggered
        elif self.__status == Status.INTERVENTION and not self.__triggered:
            waypoint = WaypointParameters(pose=cur_pose, sampling=sampling, tolerance=tolerance)
            self.__status = Status.INTERVENTION

        # Intervention triggered
        elif self.__status == Status.INTERVENTION and self.__triggered:
            self.__status = Status.SUCCESS
            self.__trigger_service.shutdown()
            self.__trigger_service = None
            self.__triggered = False

        # Intervention complete, waiting for reset
        elif self.__status == Status.SUCCESS:
            # We have to return None to allow other behaviors to be used
            waypoint = None

        else:
            error = WaypointPolicyError('Wait for trigger is in unknown state')
            self.__status = Status.ERROR

        outcome = WaypointPolicyOutcome(status=self.__status, parameters=waypoint, error=error)

        return outcome

    def conditions_met(self, cur_pose: PoseLike) -> bool:
        """
        Always returns True
        """
        return True

    def clear_state(self):
        self.__triggered = False
        if self.__trigger_service is not None:
            self.__trigger_service.shutdown()
            self.__trigger_service = None
        self.__status = Status.NOT_RUNNING

    def __trigger_callback(self, req):
        self.__triggered = True
        with self.shared_data.cv:
            self.shared_data.cv.notify_all()
        return TriggerResponse(success=True, message='Intervention handled')

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
