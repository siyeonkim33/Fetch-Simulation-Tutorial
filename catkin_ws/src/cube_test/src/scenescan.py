#!/usr/bin/env python
import sys
import copy
import os
import actionlib
import rospy
import rospkg
import time
import moveit_commander

from math import sin, cos, pi
from moveit_python import (MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal

#from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

import pcl
import pcl_helper
import pcl_ros

from tf.transformations import quaternion_from_euler
from tf import (
    TransformListener, 
    Transformer,
    TransformerROS
)
#from tf_conversions.transformations import quaternion_from_euler

from math import *


from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    TransformStamped,
    Pose,
    Point,
    Quaternion,
)

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('cube_test')
    rospy.init_node("cube") # Create a node

    # get the height of the head pose and look at the origin at half of the head height
    pt_head = PointStamped()
    transformStamped = TransformStamped()
    initial_transformStamped = TransformStamped()

    t = Transformer()
    listener = TransformListener()

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    head_action = PointHeadClient()

    " !!!!!!!!!!!!!!!!!!!!!!!!Change these values!!!!!!!!!!!!!!!!!!!!! "
    degree_end = 120
    degree_start = -120

    ## keep the initial transformation to transform the point cloud into the same frame
    pt_head.header.frame_id = "/head_camera_rgb_optical_frame"
    pt_head = listener.transformPoint("/base_link", pt_head)
    rospy.loginfo("the head position point: %f, %f, %f", pt_head.point.x, pt_head.point.y, pt_head.point.z)
    head_action.look_at(1.0, 0.0, pt_head.point.z / 2, "base_link")

    # the time out is for the head to slide into the exact pose
    time.sleep(4.0)

    listener.lookupTransform("/head_camera_rgb_optical_frame", "/base_link", rospy.Time(), initial_transformStamped)
    (trans, rot) = initial_transformStamped
    print(initial_transformStamped)
    print(trans)

    full_range = degree_end - degree_start
    sampling_time = full_range / 30 + 1

    for i in range(0,sampling_time):
        cur_degree = degree_start + i*30

        # set the head camera position from degree_start to degree_end
        rospy.loginfo("current degree %f, point %f, %f, %f ", cur_degree, 1.0, sin( cur_degree * pi / 180.0 ), pt_head.point.z)
        head_action.look_at(1.0, sin( cur_degree * pi / 180.0 ), pt_head.point.z / 2, "base_link", duration=1.0)
        time.sleep(5.0)

        transformStamped = listener.lookupTransform("base_link", "head_camera_rgb_optical_frame", rospy.Time())
        print(transformStamped)

        p = pcl.PointCloud(10)

    head_action.look_at(1.0, 0.0, pt_head.point.z / 2, "base_link")
    time.sleep(3.0)

    rospy.loginfo("Finish scanning")
    time.sleep(1.0)