#! /usr/bin/env python3
from __future__ import print_function
from six.moves import input

import rospy
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


def move_arm_client():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_arm", anonymous=True)

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

# move_arm_client()
