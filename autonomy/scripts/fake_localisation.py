#!/usr/bin/python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
import sys
from cartographer_ros_msgs.msg import StatusCode, StatusResponse
from cartographer_ros_msgs.msg import SystemState
from cartographer_ros_msgs.srv import StartLocalisation
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

from gazebo_msgs.srv import GetEntityState
import time
from threading import Thread

class FakeLocalisationPublisher(Node):

    def __init__(self):
        super().__init__('fake_localisation')

        start_loc_srv = self.create_service(
            srv_name='/mapper/start_localisation',
            srv_type=StartLocalisation,
            callback=self.start_localisation_callback
        )
        stop_loc_srv = self.create_service(
            srv_name='/mapper/stop_localisation',
            srv_type=Trigger,
            callback=self.stop_localisation_callback
        )

        self.qos_profile = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )

        self.pub = self.create_publisher(SystemState, '/mapper/state', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)


        self.robot_x = 20.0
        self.robot_y = 7.0
        self.robot_z = 0.0
        self.robot_qx = 0.0
        self.robot_qy = 0.0
        self.robot_qz = 0.0
        self.robot_qw = 1.0

        self.logger = self.get_logger()

        #self.publish_transform_latched()


    def timer_gazebo_callback(self):
        # Create gazebo service client
        cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not cli.wait_for_service(timeout_sec=0.2):
            #self.get_logger().info('FAKE LOC: gazebo entity service not available, waiting again...') # DEBUG
            print('FAKE LOC: gazebo entity service not available, waiting again...') # DEBUG
        
        time.sleep(20)
        for i in range(320):    
            self.req = GetEntityState.Request()
            # Get map position from gazebo
            self.req.name = 'test_robot'
            self.res = cli.call(self.req) 

            self.robot_x =  self.res.state.pose.position.x
            self.robot_y =  self.res.state.pose.position.y
            self.robot_z =  self.res.state.pose.position.z
            self.robot_qx = self.res.state.pose.orientation.x
            self.robot_qy = self.res.state.pose.orientation.y
            self.robot_qz = self.res.state.pose.orientation.z
            self.robot_qw = self.res.state.pose.orientation.w

            i+=1
            time.sleep(1)
            

    def timer_callback(self):
        #self.logger.info("ROBOT POSITION IS X{}, Y{}, Z{}".format(self.robot_x, self.robot_y, self.robot_z))

        self.tr_map_to_odom = TransformStamped(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            child_frame_id='odom',
            transform=Transform(
                translation=Vector3(x=20., y=7., z=0.),
                rotation=Quaternion(x=0., y=0., z=0., w=1.)                
            )
        )

        self.tr_odom_to_base = TransformStamped(
            header=Header(frame_id='odom', stamp=self.get_clock().now().to_msg()),
            child_frame_id='base_footprint',
            transform=Transform(
                translation=Vector3(x=self.robot_x, y=self.robot_y, z=self.robot_z),
                rotation=Quaternion(x=self.robot_qx, y=self.robot_qy, z=self.robot_qz, w=self.robot_qw)
            )
        )

        self.pub.publish(
            SystemState(
                mode=SystemState.MODE_LOCALISING,
                localisation_status=SystemState.LOCALISED,
                map_loaded=True,
                localisation=self.tr_map_to_odom,
                number_of_global_constraints=4
            )
        )

        tf_static_broadcaster = TransformBroadcaster(self, 10)
        tf_static_broadcaster.sendTransform(self.tr_map_to_odom)

        tf_static_broadcaster = TransformBroadcaster(self, 10)
        tf_static_broadcaster.sendTransform(self.tr_map_to_odom)

        #tf_broadcaster = TransformBroadcaster(self, 10)
        #tf_broadcaster.sendTransform(self.tr_odom_to_base)

    def publish_transform_latched(self):
        #self.logger.info("ROBOT POSITION IS X{}, Y{}, Z{}".format(self.robot_x, self.robot_y, self.robot_z))

        self.tr_map_to_odom = TransformStamped(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            child_frame_id='odom',
            transform=Transform(
                translation=Vector3(x=20., y=7., z=0.),
                rotation=Quaternion(x=0., y=0., z=0., w=1.)                
            )
        )

        self.pub.publish(
            SystemState(
                mode=SystemState.MODE_LOCALISING,
                localisation_status=SystemState.LOCALISED,
                map_loaded=True,
                localisation=self.tr_map_to_odom,
                number_of_global_constraints=4
            )
        )

        tf_static_broadcaster = StaticTransformBroadcaster(self, 10)
        tf_static_broadcaster.sendTransform(self.tr_map_to_odom)


    def start_localisation_callback(self, res):
        return StartLocalisation.Response(status=StatusResponse(code=StatusCode.OK))

    def stop_localisation_callback(self, res):
        Trigger.Response(success=True)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = FakeLocalisationPublisher()
    
    # Start gazebo client in a thread
    thread = Thread(target = node.timer_gazebo_callback,)
    thread.start()

    rclpy.spin(node)
    thread.join()
    node.destroy_node()
    rclpy.shutdown()
