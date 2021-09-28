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

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.place_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers= 30.0)
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers= 30.0)

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

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

            print(obj.object.primitives[0].dimensions)
            idx += 1
            if (obj.object.primitives[0].dimensions[0] > 0.075 or 
                obj.object.primitives[0].dimensions[1] > 0.075 or
                obj.object.primitives[0].dimensions[2] > 0.075):
                idx-=1
                print("%s basket is added" % obj.object.properties[0].name)
                obj.object.name = "basket_%s"% obj.object.properties[0].name
                obj.object.primitives[0].dimensions = [0.4, 0.3, 0.32]
                if obj.object.properties[0].name.find("red"):
                    obj.object.primitive_poses[0].position.x = 0.8
                    obj.object.primitive_poses[0].position.y = -0.42
                    obj.object.primitive_poses[0].position.z = 0.8
                else:
                    obj.object.primitive_poses[0].position.x = 0.8
                    obj.object.primitive_poses[0].position.y = 0.34
                    obj.object.primitive_poses[0].position.z = 0.8                   
                self.scene.addSolidPrimitive(obj.object.name,
                                            obj.object.primitives[0],
                                            obj.object.primitive_poses[0],
                                            use_service=False)
                continue
            else:
                print("%s block is added" % obj.object.properties[0].name)
                
                obj.object.name = "%s%d"%(obj.object.properties[0].name, idx)
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
            print obj.object.primitive_poses[0], obj.object.primitives[0]
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
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def openGripper(self):
        # Open the gripper
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()

    def closedGripper(self):
        # Open the gripper
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 0.00
        joint_goal[1] = 0.00
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()


    # def all_close(self, goal, actual, tolerance):
    #     all_equal = True

    #     if abs(actual - goal) > tolerance:
    #         return False

    #     return True

    def place(self, block, goal):

        self.place_group.set_max_velocity_scaling_factor(0.8)
        self.place_group.set_max_acceleration_scaling_factor(0.8)
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
                self.openGripper()
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

if __name__ == "__main__":
    # Create a node
    rospy.init_node("fetch_cube_pick_place")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    #move_base = MoveBaseClient()
    #torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    cube_in_grapper = False
    grasping_client.stow()

    robot = moveit_commander.RobotCommander()
    success_pickplace = 0

    while not rospy.is_shutdown():
        head_action.look_at(1.2, 0.0, 0.0, "base_link")

        # Get block to pick
        fail_ct = 0
        while not rospy.is_shutdown() and not cube_in_grapper:
            rospy.loginfo("Picking object...")
            grasping_client.updateScene()
            cube, grasps = grasping_client.getGraspableObject()
            if cube == None:
                rospy.logwarn("Perception failed.")
                # grasping_client.intermediate_stow()
                grasping_client.stow()
                head_action.look_at(1.2, 0.0, 0.0, "base_link")
                continue

            # Pick the block
            if grasping_client.pick(cube, grasps):
                cube_in_grapper = True
                break
            rospy.logwarn("Grasping failed.")
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                break
            fail_ct += 1


        print("placing...", cube.name)
        # Place the block
        while not rospy.is_shutdown() and cube_in_grapper:
            rospy.loginfo("Placing object...")

            # set the goal position
            if (cube.name.find("red") == 0):
                print("find red")
                goal = [0.85, 0.4, 1.1]
            else:
                print("find blue")
                goal = [0.85, -0.4, 1.1]

            if grasping_client.place(cube, goal):
                cube_in_grapper = False
                success_pickplace +=1
                break
            rospy.logwarn("Placing failed.")
            grasping_client.intermediate_stow()
            grasping_client.stow()

            if fail_ct > 15:
                fail_ct = 0
                break
            # if all pick-and-places are done, break the loop
            if success_pickplace > 5:
                break
            fail_ct += 1
        # Tuck the arm, lower the torso
        grasping_client.intermediate_stow()
        grasping_client.stow()
        rospy.loginfo("Finished")
