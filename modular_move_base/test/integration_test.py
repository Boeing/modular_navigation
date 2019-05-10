#!/usr/bin/env python
import copy
import logging
import math
import threading
import time
import unittest
from collections import namedtuple

import math3d
import numpy
import rosbag
import rospy
import rostest
import typing
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerRequest

from modular_amcl.srv import SetInitialPose, SetInitialPoseRequest

logger = logging.getLogger(__name__)

TimedCmdVel = namedtuple('TimedCmdVel', ['cmd_vel', 'duration'])


def accumulate(iterator):
    total = 0
    for item in iterator:
        total += item
        yield total


def quaternion_from_rpy(roll, pitch, yaw):
    # type: (float, float, float) -> Quaternion
    qt = math3d.Orientation.new_euler(angles=[roll, pitch, yaw], encoding='xyz').get_quaternion()
    return Quaternion(
        x=qt.x, y=qt.y, z=qt.z, w=qt.s
    )


def get_pose_error(transform_a, transform_b):
    # type: (math3d.Transform, math3d.Transform) -> typing.Tuple[float, float]
    """
    Compute the error norm of position and orientation.
    """
    error_position = numpy.linalg.norm(transform_a.get_pos().get_array() - transform_b.get_pos().get_array(), ord=2)

    quaternion_a = transform_a.get_orient().get_quaternion()  # type: math3d.UnitQuaternion
    quaternion_b = transform_b.get_orient().get_quaternion()  # type: math3d.UnitQuaternion

    if quaternion_a.s < 0.0:
        quaternion_a.q *= -1.0

    if quaternion_b.s < 0.0:
        quaternion_b.q *= -1.0

    if numpy.allclose(quaternion_a.get_array(), quaternion_b.get_array(), atol=1.e-12):
        error_angle_rad = 0.0
    else:
        error_angle_rad = 2.0 * numpy.arccos((quaternion_a * quaternion_b.get_inverse()).s)

    if error_angle_rad > math.pi:
        error_angle_rad = math.fabs(2.0 * math.pi - error_angle_rad)

    return error_position, error_angle_rad


def set_gazebo_model_pose(model_name, pose):
    # type: (str, Pose) -> None

    get_model_state_srv = rospy.ServiceProxy(
        name='/gazebo/get_model_state',
        service_class=GetModelState
    )

    set_model_state_srv = rospy.ServiceProxy(
        name='/gazebo/set_model_state',
        service_class=SetModelState
    )

    ms = get_model_state_srv.call(
        GetModelStateRequest(
            model_name=model_name,
            relative_entity_name='world'
        )
    )  # type: GetModelStateResponse

    assert ms.success

    resp = set_model_state_srv.call(
        SetModelStateRequest(
            model_state=ModelState(
                model_name=model_name,
                pose=pose,
                twist=ms.twist,
                reference_frame='world'
            )
        )
    )  # type: SetModelStateResponse

    assert resp.success


class TopicBuffer(object):
    def __init__(self, name, data_class):
        self.__conditional = threading.Condition()
        self.__buffer = list()
        self.__expected_size = 0
        self.__sub = rospy.Subscriber(
            name=name,
            data_class=data_class,
            callback=self.__callback
        )

    def __callback(self, msg):
        with self.__conditional:
            self.__buffer.append((msg, rospy.Time.now()))
            self.__conditional.notify_all()

    def get_buffer(self):
        return copy.deepcopy(self.__buffer)

    def pop(self):
        with self.__conditional:
            item = self.__buffer.pop()
            self.__expected_size -= 1
            return item

    def clear(self):
        with self.__conditional:
            self.__expected_size = 0
            del self.__buffer[:]

    def wait_for_messages(self, number=1, timeout=5):
        self.__expected_size += number
        if len(self.__buffer) < self.__expected_size:
            with self.__conditional:
                self.__conditional.wait(timeout=timeout)
        return len(self.__buffer) >= self.__expected_size


class TestDetection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.start_srv = rospy.ServiceProxy(
            name='/modular_amcl/start',
            service_class=Trigger
        )
        cls.set_initial_pose_srv = rospy.ServiceProxy(
            name='/modular_amcl/set_initial_pose',
            service_class=SetInitialPose
        )
        cls.cmd_vel_pub = rospy.Publisher(
            name='cmd_vel',
            data_class=Twist
        )

    def _execute_timed_cmd_vel_sequence(self, cmd_vel_sequence):
        # type: (typing.Sequence[TimedCmdVel]) -> None

        i = 0
        start = rospy.Time.now()
        now = rospy.Time.now()
        while not rospy.is_shutdown() and now < start + cmd_vel_sequence[-1].duration:
            if now > start + cmd_vel_sequence[i].duration:
                i += 1
            if i > len(cmd_vel_sequence):
                break
            self.cmd_vel_pub.publish(cmd_vel_sequence[i].cmd_vel)
            time.sleep(0.01)
            now = rospy.Time.now()
        self.cmd_vel_pub.publish(
            Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0)
            )
        )

    def test_init(self):

        logger.info('Wait for gazebo to insert the model')
        time.sleep(5)

        logger.info('Setting robot start position')
        origin_pose = Pose(
            position=Point(0, 0, 0),
            orientation=Quaternion(
                x=0, y=0, z=0, w=1
            )
        )
        set_gazebo_model_pose(model_name='mobile_laser', pose=origin_pose)

        logger.info('Waiting for gazebo to generate a map')
        rospy.wait_for_message(topic='gazebo/map', topic_type=OccupancyGrid)

        logger.info('Starting Localisation')
        start_response = self.start_srv.call(
            TriggerRequest()
        )  # type: SetModelStateResponse
        self.assertTrue(start_response.success, msg=start_response)

        return

        # logger.info('Setting Initial Pose')
        # set_pose_response = self.set_initial_pose_srv.call(
        #     SetInitialPoseRequest(
        #         pose=PoseWithCovarianceStamped(
        #             header=Header(frame_id='map'),
        #             pose=PoseWithCovariance(
        #                 pose=origin_pose,
        #                 covariance=[0.002, 0.002, 0., 0., 0., 0.,
        #                             0.002, 0.002, 0., 0., 0., 0.,
        #                             0., 0., 0., 0., 0., 0.,
        #                             0., 0., 0., 0., 0., 0.,
        #                             0., 0., 0., 0., 0., 0.,
        #                             0., 0., 0., 0., 0., 0.01]
        #             )
        #         )
        #     )
        # )  # type: SetModelStateResponse
        # self.assertTrue(set_pose_response.success, msg=set_pose_response)

        instructions = (
            (0.2, 0.0, 0.0, 10),  # forward
            (0.0, 0.0, math.pi / 2 / 4, 4),  # turn left

            (0.2, 0.0, 0.0, 10),  # forward
            (0.0, 0.0, math.pi / 2 / 4, 4),  # turn left

            (0.2, 0.0, 0.0, 10),  # forward
            (0.0, 0.0, math.pi / 2 / 4, 4),  # turn left

            (0.2, 0.0, 0.0, 10),  # forward
            (0.0, 0.0, math.pi / 2 / 4, 4),  # turn left
        )

        instructions *= 2

        x, y, rot, t = zip(*instructions)
        instructions = zip(x, y, rot, accumulate(t))

        cmd_vel_sequence = [
            TimedCmdVel(
                cmd_vel=Twist(
                    linear=Vector3(x, y, 0),
                    angular=Vector3(0, 0, rot)
                ),
                duration=rospy.Duration(t)
            )
            for x, y, rot, t in instructions
        ]

        gazebo_state = TopicBuffer(name='gazebo/model_states', data_class=ModelStates)
        amcl_pose = TopicBuffer(name='modular_amcl/pose', data_class=PoseWithCovarianceStamped)

        start_time = rospy.Time.now()

        self._execute_timed_cmd_vel_sequence(cmd_vel_sequence=cmd_vel_sequence)

        bag = rosbag.Bag('/home/boeing/test.bag', 'w')

        gazebo_state_msgs = gazebo_state.get_buffer()  # type: typing.List[ModelStates]
        amcl_pose_msgs = amcl_pose.get_buffer()  # type: typing.List[PoseWithCovarianceStamped]

        logger.info('Logged {} state messages from gazebo'.format(len(gazebo_state_msgs)))
        logger.info('Logged {} pose messages from amcl'.format(len(amcl_pose_msgs)))

        del gazebo_state
        del amcl_pose

        for gazebo_state_msg, ros_time in gazebo_state_msgs:

            if ros_time < start_time:
                continue

            self.assertIsInstance(gazebo_state_msg, ModelStates)

            i = gazebo_state_msg.name.index('mobile_laser')
            ground_truth = gazebo_state_msg.pose[i]  # type: Pose

            ground_truth_pose_stamped = PoseStamped(
                header=Header(frame_id='world', stamp=ros_time),
                pose=ground_truth
            )
            bag.write(topic='ground_truth', msg=ground_truth_pose_stamped, t=ros_time)

        for amcl_pose_msg, ros_time in amcl_pose_msgs:

            if ros_time < start_time:
                continue

            self.assertIsInstance(amcl_pose_msg, PoseWithCovarianceStamped)

            amcl_pose_stamped = PoseStamped(
                header=Header(frame_id='world', stamp=amcl_pose_msg.header.stamp),
                pose=amcl_pose_msg.pose.pose
            )
            bag.write(topic='amcl', msg=amcl_pose_stamped, t=amcl_pose_stamped.header.stamp)

        bag.close()


if __name__ == '__main__':
    rospy.init_node('test_detection')

    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)
        for _logger_name, _logger in logging.Logger.manager.loggerDict.items():
            if _logger_name not in ['rosgraph', 'rospy', 'rosout'] and '.' not in _logger_name:
                if isinstance(_logger, logging.PlaceHolder):
                    _logger = logging.getLogger(_logger_name)
                _logger.addHandler(handler)
                _logger.setLevel(logging.DEBUG)

    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/get_model_state')

    rostest.rosrun('modular_amcl', 'test_detection', TestDetection)
