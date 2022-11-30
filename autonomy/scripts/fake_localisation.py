#!/usr/bin/python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import sys
from cartographer_ros_msgs.msg import StatusCode, StatusResponse
from cartographer_ros_msgs.msg import SystemState
from cartographer_ros_msgs.srv import StartLocalisation
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def start_localisation_callback(_, res):
    return StartLocalisation.Response(status=StatusResponse(code=StatusCode.OK))


def stop_localisation_callback(_, res):
    Trigger.Response(success=True)


if __name__ == '__main__':
    
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('fake_localisation')

    start_loc_srv = node.create_service(
        srv_name='/mapper/start_localisation',
        srv_type=StartLocalisation,
        callback=start_localisation_callback
    )
    stop_loc_srv = node.create_service(
        srv_name='/mapper/stop_localisation',
        srv_type=Trigger,
        callback=stop_localisation_callback
    )

    qos_profile = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            )

    pub = node.create_publisher(topic='/mapper/state', msg_type=SystemState, qos_profile=qos_profile)

    tr = TransformStamped(
        header=Header(frame_id='map', stamp=node.get_clock().now().to_msg()),
        child_frame_id='odom',
        transform=Transform(
            translation=Vector3(x=0., y=0., z=0.),
            rotation=Quaternion(x=0., y=0., z=0., w=1.)
        )
    )

    pub.publish(
        SystemState(
            mode=SystemState.MODE_LOCALISING,
            localisation_status=SystemState.LOCALISED,
            map_loaded=True,
            localisation=tr,
            number_of_global_constraints=4
        )
    )

    #static_tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)
    #static_tf_pub.publish(TFMessage(transforms=[tr]))
    tf_static_broadcaster = StaticTransformBroadcaster(node=node, qos=qos_profile)
    tf_static_broadcaster.sendTransform(tr)

    rclpy.spin(node)
