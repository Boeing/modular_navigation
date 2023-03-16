#!/usr/bin/python3

from matplotlib import transforms
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
        start_loc_srv  # avoid unused variable warning

        stop_loc_srv = self.create_service(
            srv_name='/mapper/stop_localisation',
            srv_type=Trigger,
            callback=self.stop_localisation_callback
        )
        stop_loc_srv  # avoid unused variable warning

        qos_profile = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST)

        self.state_pub = self.create_publisher(SystemState, '/mapper/state', qos_profile)
        self.static_tf_pub = self.create_publisher(TFMessage, '/tf_static', qos_profile)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        tr = TransformStamped(
            header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
            child_frame_id='odom',
            transform=Transform(
                translation=Vector3(x=0., y=0., z=0.),
                rotation=Quaternion(x=0., y=0., z=0., w=1.)
            )
        )

        self.state_pub.publish(
            SystemState(
                mode=SystemState.MODE_LOCALISING,
                localisation_status=SystemState.LOCALISED,
                map_loaded=True,
                localisation=tr,
                number_of_global_constraints=4
            )
        )

        self.static_tf_pub.publish(
            TFMessage(
                transforms=[tr]
            )
        )

    def start_localisation_callback(self, req, res):
        return StartLocalisation.Response(status=StatusResponse(code=StatusCode.OK))

    def stop_localisation_callback(self, req, res):
        Trigger.Response(success=True)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = FakeLocalisationPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
