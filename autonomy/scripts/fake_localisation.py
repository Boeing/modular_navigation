#!/usr/bin/python

import rospy
from cartographer_ros_msgs.msg import StatusCode, StatusResponse
from cartographer_ros_msgs.msg import SystemState
from cartographer_ros_msgs.srv import StartLocalisation, StartLocalisationResponse
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from tf2_msgs.msg import TFMessage


def start_localisation_callback(_):
    return StartLocalisationResponse(status=StatusResponse(code=StatusCode.OK))


def stop_localisation_callback(_):
    TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('fake_localisation')

    start_loc_srv = rospy.Service(
        name='/mapper/start_localisation',
        service_class=StartLocalisation,
        handler=start_localisation_callback
    )
    stop_loc_srv = rospy.Service(
        name='/mapper/stop_localisation',
        service_class=Trigger,
        handler=stop_localisation_callback
    )

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
