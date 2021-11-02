#!/usr/bin/env python

import copy
import actionlib
import rospy

import moveit_commander
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import (
    FollowJointTrajectoryAction, 
    FollowJointTrajectoryGoal,
    PointHeadAction, 
    PointHeadGoal,
    GripperCommandAction, 
    GripperCommandGoal,
)
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal, Object
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, DisplayTrajectory, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive

from pose_cnn.msg import EachObject, Class
from tf2_msgs.msg import TFMessage
import message_filters
import math
# from fetch_driver_msgs.msg import GripperState

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

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


# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        # self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        self.place_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers= 30.0)
        self.place_group.set_max_velocity_scaling_factor(0.8)
        self.place_group.set_max_acceleration_scaling_factor(0.8)
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers= 30.0)

        self.object_sub = rospy.Subscriber('posecnn_objects', Class, self.poseCNN)

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()
    

    def poseCNN(self, obj_msg):
        self.check_objects = list()
        
        for o in obj_msg.objects:
            self.check_objects.append(o)


    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        objects = list()
        idx = -1

        for obj in find_result.objects:
            idx += 1
            self.min_sum = 100

            # SET EACH OBJECT's NAME
            # Add pose_cnn detected objects
            for check in self.check_objects:

                x_diff = obj.object.primitive_poses[0].position.x - check.posestamped.pose.position.x
                y_diff = obj.object.primitive_poses[0].position.y - check.posestamped.pose.position.y
                z_diff = obj.object.primitive_poses[0].position.z - check.posestamped.pose.position.z
                sum = math.sqrt(pow(x_diff,2)+pow(y_diff,2)+pow(z_diff,2))

                if sum < self.min_sum:
                    self.min_sum = sum
                    self.min_name = check.name

            obj.object.name = self.min_name
            self.scene.addSolidPrimitive(obj.object.name,
                                        obj.object.primitives[0],
                                        obj.object.primitive_poses[0],
                                        use_service=False)
            if obj.object.primitive_poses[0].position.x < 0.85:
                objects.append([obj, obj.object.primitive_poses[0].position.z])



        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.1,
                                            2.0,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         use_service=False)

        self.scene.waitForSync()

        # store for grasping
        #self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

        # store graspable objects by Z
        objects.sort(key=lambda object: object[1])
        objects.reverse()
        self.objects = [object[0] for object in objects]
        #for object in objects:
        #    print(object[0].object.name, object[1])
        #exit(-1)

    def getGraspableObject(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            if obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25 or \
               obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25 or \
               obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            if obj.object.primitive_poses[0].position.y > 0.25 or obj.object.primitive_poses[0].position.y < -0.25:
                continue 
            # print obj.object.primitive_poses[0], obj.object.primitives[0]
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass


    def pick(self, block, grasps):

        for i in range(len(grasps)):
            # set pre_grasp pose
            print("pre_grasping ...", i)
            self.pose_target = Pose()
            self.pose_target.orientation = grasps[i].grasp_pose.pose.orientation
            self.pose_target.position.x = grasps[i].grasp_pose.pose.position.x
            self.pose_target.position.y = grasps[i].grasp_pose.pose.position.y
            self.pose_target.position.z = grasps[i].grasp_pose.pose.position.z + 0.1

            self.place_group.set_pose_target(self.pose_target)
            plan = self.place_group.go(wait=True)
            
            if plan:
                # approach to grasping pose
                print("approaching ...")
                self.pose_target.position.z = grasps[i].grasp_pose.pose.position.z + 0.02
                self.place_group.set_pose_target(self.pose_target)
                plan = self.place_group.go(wait=True)
                self.pick_num = i

                return True

        print("failed every picking ...")
        return False

    def liftup(self, block, grasps):
        print("lifting up ...")
        self.pose_target.position.z = grasps[self.pick_num].grasp_pose.pose.position.z + 0.2
        self.place_group.set_pose_target(self.pose_target)
        plan = self.place_group.go(wait=True)

    def place(self, block, goal):
        # eef_link = self.place_group.get_end_effector_link()

        pose_goal = Pose()
        # pose_goal.orientation.w = cube.primitive_poses[0].orientation.w
        pose_goal.orientation.w = 0
        pose_goal.position.x = goal[0] - 0.2
        pose_goal.position.y = goal[1]
        pose_goal.position.z = goal[2]

        for i in range(16):
            self.place_group.set_pose_target(pose_goal)
            plan = self.place_group.go(wait=True)
            self.place_group.stop()
            self.place_group.clear_pose_targets()

            current_pose = self.place_group.get_current_pose().pose
            # success = self.all_close(pose_goal, current_pose, 0.01)
            success = True

            if success:
                print("Successfully place object")
                self.scene.removeAttachedObject(block.name, False)
                # self.openGripper()
                # self.updateScene()
                return success

            pose_goal.orientation.w += 1/16

        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def intermediate_stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

class GripperClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/gripper_controller/gripper_action", GripperCommandAction)
        self.goal = GripperCommandGoal()
        rospy.loginfo("Waiting for gripper...")
        self.client.wait_for_server()

    def command(self, position, effort):
        print("gripper controlling ... ")
        self.goal.command.position = position
        self.goal.command.max_effort = effort
        self.client.send_goal(self.goal)

if __name__ == "__main__":
    # Create a node
    rospy.init_node("fetch_cube_pick_place")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    head_action = PointHeadClient()
    grasping_client = GraspingClient()
    gripper_client = GripperClient()


    grasping_client.updateScene()
    grasping_client.stow()

    cube_in_grapper = False
    success_pickplace = 0


    while not rospy.is_shutdown():
        head_action.look_at(1.5, 0.0, 0.0, "base_link")

        # Get block to pick
        fail_ct = 0
        while not rospy.is_shutdown() and not cube_in_grapper:
            rospy.loginfo("Picking object...")


            grasping_client.updateScene()
            # cube, grasps = grasping_client.getGraspableObject()

            # if cube == None:
    #             rospy.logwarn("Perception failed.")
    #             # grasping_client.intermediate_stow()
    #             grasping_client.stow()
    #             head_action.look_at(1.2, 0.0, 0.0, "base_link")
    #             continue

    #         # Pick the block
        
    #         # open the gripper
    #         gripper_client.command(position=0.08, effort=50.0)
    #         if grasping_client.pick(cube, grasps):
    #             # close the gripper
    #             gripper_client.command(position=0.0, effort=-50.0)
                    
    #             rospy.Duration(15.0)
    #             # lift the object
    #             grasping_client.liftup(cube, grasps)
                
    #             cube_in_grapper = True
    #             break
    #         rospy.logwarn("Grasping failed.")
    #         grasping_client.stow()
    #         if fail_ct > 15:
    #             fail_ct = 0
    #             break
    #         fail_ct += 1

    #     # if(gripper_client.)
    #     print("placing...", cube.name)
    #     # Place the block
    #     while not rospy.is_shutdown() and cube_in_grapper:
    #         rospy.loginfo("Placing object...")

    #         # set the goal position
    #         if (cube.name.find("red") == 0):
    #             print("find red and its goal...")
    #             goal = [0.85, 0.6, 1.1]
    #         else:
    #             print("find blue and its goal...")
    #             goal = [0.85, -0.6, 1.1]

    #         if grasping_client.place(cube, goal):

    #             gripper_client.command(position=0.08, effort=50.0)
    #             cube_in_grapper = False
    #             # success_pickplace +=1
    #             break
    #         rospy.logwarn("Placing failed.")
    #         grasping_client.intermediate_stow()
    #         grasping_client.stow()

    #         if fail_ct > 15:
    #             fail_ct = 0
    #             break

    #         fail_ct += 1
    #     # Tuck the arm, lower the torso
    #     rospy.sleep(rospy.Duration(10.0))

    #     grasping_client.intermediate_stow()
    #     grasping_client.stow()
    #     rospy.loginfo("Finished")
