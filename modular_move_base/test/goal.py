import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header


def run(x, y):
    client = actionlib.SimpleActionClient(ns='move_base', ActionSpec=MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal(
        target_pose=PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(x=x, y=y),
                orientation=Quaternion(w=1)
            )
        )
    )

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()


if __name__ == '__main__':
    rospy.init_node('client')

    while not rospy.is_shutdown():
        _ = run(2, 0)
        _ = run(-2, 0)

        # _ = run(0, -6.5)
        # _ = run(2, -4)
        # _ = run(2, -5)
        # _ = run(3, -5)
        # _ = run(4, -3)
