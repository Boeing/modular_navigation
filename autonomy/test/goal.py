import actionlib
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped
from math6d.geometry import Quaternion, Vector3
from std_msgs.msg import Header

from autonomy.msg import DriveAction, DriveGoal


def run(x, y, angle):
    client = actionlib.SimpleActionClient(ns='autonomy', ActionSpec=DriveAction)

    client.wait_for_server()

    qt = Quaternion.from_axis_angle(axis=Vector3(0, 0, 1), angle=angle)

    goal = DriveGoal(
        target_pose=PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(x=x, y=y),
                orientation=Quaternion(x=qt.x, y=qt.y, z=qt.z, w=qt.w)
            )
        )
    )

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()


if __name__ == '__main__':
    rospy.init_node('client')

    while not rospy.is_shutdown():
        _ = run(1, 1, 0)
        _ = run(2, 2, 0)
