#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from math import pi
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningSceneRequest, ApplyPlanningSceneResponse, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject

from tf.transformations import quaternion_from_euler


if __name__ == '__main__':
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('insert_table')

        apply_planning_scene = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        rospy.sleep(0.5)

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = 'table'
        co.header = box_pose.header
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1, 1, 1])]
        co.pose = box_pose.pose
        co.primitive_poses = [Pose(orientation=Quaternion(w=1.0))]

        aps_request = ApplyPlanningSceneRequest()
        aps_request.scene.is_diff = True  # We do not want to set the full state, just add one thing
        aps_request.scene.world.collision_objects = [co]

        rospy.loginfo("Applying planning scene now")
        apply_planning_scene(aps_request)  # type: ApplyPlanningSceneResponse

        rospy.loginfo("table inserted")
