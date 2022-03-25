#! /usr/bin/env python3
import rospy

import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians
from cob_srvs.srv import Dock


def move_base_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    table_pose = PoseStamped()
    table_pose.header.frame_id = "table"
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = 0.0
    table_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(90)))

    client.wait_for_server()

    goal = MoveBaseGoal(target_pose=table_pose)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

def dock_on_table(y_coord):
    dock = rospy.ServiceProxy('/docker_control/dock_poses', Dock)
    table_pose = Pose()
    table_pose.position.x = 0.0
    table_pose.position.y = y_coord
    table_pose.position.z = 0.0
    table_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(0)))

    dock(frame_id='table', poses=[table_pose], dist_threshold=0.1)


if __name__ == '__main__':
    try:
        rospy.init_node('move_base_client')
        result = move_base_client()
        rospy.loginfo("Arrive at table, now docking")
        dock_on_table(-1.0)
        rospy.loginfo("Docked one end")
        dock_on_table(1.0)
        rospy.loginfo("Docked other end")
    except Exception as e:
        rospy.logerr(f'Error ocurred: {e}')
