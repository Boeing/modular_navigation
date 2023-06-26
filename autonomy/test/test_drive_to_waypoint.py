import os
import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from ament_index_python import get_package_share_directory

import subprocess
import rclpy
import rclpy.clock
import rclpy.time
import pytest
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch_ros.actions import Node

from launch.conditions import IfCondition

from gazebo_msgs.srv import SpawnEntity, GetEntityState

from geometry_msgs.msg import Pose, PoseStamped
from autonomy_interface.action import Drive
from actionlib_msgs.msg import GoalStatus
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


import time
from math import pi


@pytest.mark.launch_test
def generate_test_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    autonomy_share_dir = get_package_share_directory('autonomy')

    world_file_name = os.path.join(get_package_share_directory('autonomy'),
                                   'test', 'resources', 'sim_map.world')
    robot_file_name_urdf = os.path.join(get_package_share_directory('autonomy'),
                                        'test', 'resources', 'test_robot.urdf')
    robot_file_name_sdf = os.path.join(get_package_share_directory('autonomy'),
                                       'test', 'resources', 'test_robot.sdf')
    navigation_config_file = os.path.join(autonomy_share_dir, 'test',
                                          'config/navigation.yaml')

    print('world_file_name : {}'.format(world_file_name))
    print('robot_urdf_file_name : {}'.format(robot_file_name_urdf))
    print('robot_sdf_file_name : {}'.format(robot_file_name_sdf))

    declared_arguments = [
        DeclareLaunchArgument(name='use_sim_time',
                              description='Use simulation (Gazebo) clock if true',
                              default_value='true', ),
        DeclareLaunchArgument(name='gazebo_client',
                              default_value='true',
                              description='If running in sim, whether or not to launch GZClient'),
    ]

    use_sim_time_arg = LaunchConfiguration("use_sim_time")
    gazebo_client_arg = LaunchConfiguration("gazebo_client")

    return launch.LaunchDescription(
        declared_arguments +
        [
            # Launch GAZEBO
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': world_file_name, 'gui': '1'}.items(),
                condition=IfCondition(LaunchConfiguration('use_sim_time'))
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzclient.launch.py')
                ),
                launch_arguments={'gui': '1'}.items(),
                # IfCondition was not meant to handle 2 conditionals, workaround
                # https://answers.ros.org/question/394181/multiple-conditions-for-ifcondition-in-ros2-launch-script/
                condition=IfCondition(
                                    PythonExpression(
                                        ["'", use_sim_time_arg, "' == 'true' and '", gazebo_client_arg, "' == 'true'"]
                                    )
                )
            ),

            # Launch robot_state_publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                arguments=[robot_file_name_urdf]
            ),

            # Fake localisation
            Node(
                package='autonomy',
                executable='fake_localisation.py',
                output={'full'},
                on_exit=Shutdown(),
            ),

            # Map manager
            Node(
                package='map_manager',
                executable='map_manager_node.py',
                name='map_manager',
                output={'full'},
                remappings=[
                ],
                on_exit=Shutdown()
            ),

            # Launch autonomy
            Node(
                package='autonomy',
                executable='autonomy',
                name='autonomy',
                output={'full'},
                parameters=[
                    {
                        'navigation_config': navigation_config_file,
                        'use_sim_time': LaunchConfiguration('use_sim_time')
                    }
                ],
                on_exit=Shutdown(),
                remappings=[
                    ('/autonomy/cmd_vel', '/cmd_vel'),  # Pub, left is original
                ],
            ),

            launch_testing.actions.ReadyToTest(),
        ]
    )


# There is a bug where the gzserver is not killed after the tests run.
# https://github.com/ros2/launch/issues/545
# The issue should be fixed by https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1376
# This is a workaround to kill the gzserver after the tests run.
# Remove this once gazebo updates the apt package to the latest version.
@ launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_kill_sim(self):
        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestDriveToWaypoint(unittest.TestCase):

    @ classmethod
    def setUpClass(cls):
        rclpy.init()

    @ classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.clock = rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.log = self.node.get_logger()

        # Clients for spawning robot and checking status in gazebo
        if (self.node.get_parameter('use_sim_time').get_parameter_value().bool_value):
            # Spawn service
            self.spawn_client = self.node.create_client(
                SpawnEntity, "/spawn_entity")
            while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                self.log.info('Spawn service not available, waiting again...')
            # State service
            self.entity_state_client = self.node.create_client(
                GetEntityState, "/gazebo/get_entity_state")
            while not self.entity_state_client.wait_for_service(timeout_sec=1.0):
                self.log.info(
                    'Entity state service not available, waiting again...')

        # Client for Drive action
        self.drive_action_client = ActionClient(self.node, Drive, "autonomy")
        while not self.drive_action_client.wait_for_server(timeout_sec=1.0):
            self.log.info(
                'Action server to drive the robotnot available, waiting again...')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Make constants constant
        self.robot_name = 'test_robot'
        self.global_frame = 'map'

        self.robot_pose = Pose()
        self.robot_pose.position.x = 20.0
        self.robot_pose.position.y = 7.0
        # Drive action field target_pose must be of type PoseStamped
        self.waypoint_offset = -2.0
        self.waypoint_pose = PoseStamped()
        self.waypoint_pose.pose.position.x = 22.0
        self.waypoint_pose.pose.position.y = 9.0

    def tearDown(self):
        self.node.destroy_node()

    def test_a_spawn_robot(self):
        # Gazebo stuff if use_sim_time = True
        if (self.node.get_parameter('use_sim_time').get_parameter_value().bool_value):
            # Spawn robot
            gz_urdf = os.path.join(get_package_share_directory('autonomy'),
                                   'test', 'resources', 'test_robot.sdf')
            self.log.debug('Opening file ' + gz_urdf)
            f = open(gz_urdf, 'r')
            urdf_file_data = f.read()
            self.log.debug('URDF loaded')
            self.spawn_req = SpawnEntity.Request()
            self.spawn_req.name = self.robot_name
            self.spawn_req.xml = urdf_file_data
            self.spawn_req.initial_pose = self.robot_pose
            self.log.debug('Calling spawn service')
            self.spawn_future = self.spawn_client.call_async(self.spawn_req)
            self.log.debug('Waiting for the service to end')
            rclpy.spin_until_future_complete(self.node, self.spawn_future)
            self.log.info('Returning result')
            self.spawn_response = self.spawn_future.result()
            self.log.info('Result:  %s' % self.spawn_response.success)
            self.log.info('Message: ' + self.spawn_response.status_message)
            self.assertTrue(self.spawn_response.success)

            # Check status of robot in Gazebo
            self.entity_state_req = GetEntityState.Request()
            self.entity_state_req.name = self.robot_name
            self.entity_state_future = self.entity_state_client.call_async(
                self.entity_state_req)
            rclpy.spin_until_future_complete(
                self.node, self.entity_state_future)
            self.entity_state_res = self.entity_state_future.result()
            self.log.info('Entity pose: (' + str(self.entity_state_res.state.pose.position.x) +
                          ', ' + str(self.entity_state_res.state.pose.position.y) + ')')
            self.assertTrue(self.entity_state_res.success)
        else:
            self.log.info('Not in simulation. Exiting.')
            self.assertTrue(True)

    def test_b_get_waypoint(self):
        # Create Drive action message
        self.drive_action = Drive.Goal()
        # Populate Drive action msg with the goal pose
        self.drive_action.target_pose = self.waypoint_pose
        self.drive_action.target_pose.header.frame_id = 'map'

        self.drive_action.max_velocity_x = 0.2
        self.drive_action.max_velocity_y = 0.2
        self.drive_action.max_velocity_w = 0.2
        self.drive_action.xy_goal_tolerance = 0.02
        self.drive_action.yaw_goal_tolerance = 0.01
        self.drive_action.avoid_distance = 0.02
        self.drive_action.backwards_mult = 1.5
        self.drive_action.strafe_mult = 1.5
        self.drive_action.rotation_mult = 0.3/pi

        # Wait for robot to be localised
        time.sleep(35)  # DEBUG

        # Send Goal
        drive_action_future = self.drive_action_client.send_goal_async(
            self.drive_action)
        # Wait for the goal to be accepted/rejected
        rclpy.spin_until_future_complete(self.node, drive_action_future)
        # Get the goal handle as result of the future
        drive_action_goal_handle: ClientGoalHandle = drive_action_future.result()

        if drive_action_goal_handle.accepted:
            self.log.info('Goal ACCEPTED')

            # Request the goal result
            drive_action_result_future = drive_action_goal_handle.get_result_async()

            # Wait for the goal result
            rclpy.spin_until_future_complete(
                self.node, drive_action_result_future)

            # Get the goal result
            drive_action_result = drive_action_result_future.result()

            # Assert if goal achieved
            self.assertTrue(drive_action_result.status)
            # TODO Assert that desired position is reached in gazebo
        else:
            self.log.warn('Autonomy goal rejected')
