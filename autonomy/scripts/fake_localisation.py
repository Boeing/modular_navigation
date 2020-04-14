#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from cartographer_ros_msgs.msg import SystemState

if __name__ == '__main__':
    rospy.init_node('fake_localisation')

    pub = rospy.Publisher(name='/mapper/state', data_class=SystemState, queue_size=1, latch=True)

    tr = TransformStamped(
        header=Header(frame_id='map', stamp=rospy.Time(0)),
        child_frame_id='odom',
        transform=Transform(
            translation=Vector3(0, 0, 0),
            rotation=Quaternion(0, 0, 0, 1)
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

    static_tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=1, latch=True)
    static_tf_pub.publish(TFMessage(transforms=[tr]))

    rospy.spin()
