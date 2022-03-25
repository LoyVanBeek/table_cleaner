#! /usr/bin/env python3
import rospy
import os
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians
from six.moves import input
import sys
import copy
from math import pi, tau, dist, fabs, cos
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from math import radians
from cob_srvs.srv import Dock

    # 1. Drive to table
    # 2. Dock to table for 'fine' positioning
    # 3. Move arm into 'wiper' position
    # 4. Dock to other side of table


def move_base_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    table_pose = PoseStamped()
    table_pose.header.frame_id = "table"
    table_pose.pose.position.x = 0.74
    table_pose.pose.position.y = -1.0
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
    table_pose.position.x = 0.74
    table_pose.position.y = y_coord
    table_pose.position.z = 0.0
    table_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(0)))

    dock(frame_id='table', poses=[table_pose], dist_threshold=0.1)

def move_arm_client():
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("move_arm", anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.5
    pose_goal.orientation.y = 0.5
    pose_goal.orientation.z = -0.5
    pose_goal.orientation.w = 0.5
    #pose_goal.orientation = Quaternion(*quaternion_from_euler(0, 0, 3.14))i
    pose_goal.position.x = -0.74
    pose_goal.position.y = 0.17
    pose_goal.position.z = 1.2

    rospy.loginfo(pose_goal)
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

def arm_to_transport():
    move_group = moveit_commander.MoveGroupCommander('arm')
    remembered = move_group.get_named_targets()
    rospy.loginfo(remembered)
    transport = move_group.get_named_target_values('transport')
    move_group.go(transport, wait=True)
    move_group.stop()


# move_arm_client()

if __name__ == '__main__':
    rospy.init_node('table_cleaner')
    rospy.loginfo("Going to clean table")
    arm_to_transport()
    rospy.loginfo("Moving to table")
    move_base_client()
    rospy.loginfo("Docking on table")
    dock_on_table(-1.0)
    rospy.loginfo("Moving arm into cleaning position")
    move_arm_client()
    rospy.loginfo("Moving to end of the table")
    dock_on_table(1.0)
    rospy.loginfo("Table clean!")
    arm_to_transport()
    rospy.loginfo("Arm back to transport")
