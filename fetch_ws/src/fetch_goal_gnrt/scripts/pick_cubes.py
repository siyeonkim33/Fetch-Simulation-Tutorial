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

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf.transformations import quaternion_from_euler, quaternion_multiply

from tf import (
    TransformListener, 
    Transformer,
    TransformerROS
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
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

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.angle_max = 180
        self.angle_step = 20
        self.quaternion_multiply = quaternion_multiply

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

        rospy.loginfo("Found %d objects" % len(find_result.objects))

        # remove previous objects
        #for name in self.scene.getKnownCollisionObjects():
        #    self.scene.removeCollisionObject(name, False)
        #for name in self.scene.getKnownAttachedObjects():
        #    self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        objects = list()
        idx = -1
        for obj in find_result.objects:
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            idx += 1

            #obj.object.name = find_result.objects[idx].name -> 'GraspableObject' object has no attribute 'name
            obj.object.name = "obj%d"%idx
            self.scene.addSolidPrimitive(obj.object.name, obj.object.primitives[0], obj.object.primitive_poses[0], use_service=False)
            if obj.object.primitive_poses[0].position.x < 1.3: #< 0.85:
                objects.append([obj, obj.object.primitive_poses[0].position.y]) ##should be it's goal position,,,

        self.scene.waitForSync()

        # store for grasping
        self.surfaces = find_result.support_surfaces

        # store graspable objects by Z -> y
        objects.sort(key=lambda object: object[1])
        #objects.reverse()
        self.objects = [object[0] for object in objects]

    def getGraspableObject(self, goal_obj_x, goal_obj_y):
        graspable = None
        for obj in self.objects:
              return obj.object
        # nothing detected
        return None
    
    def make_poseStamped(self, frame_id, pose, gripper_orientation):
        pose_stp = PoseStamped()
        pose_stp.header.stamp = rospy.Time.now()
        pose_stp.header.frame_id = frame_id
        pose_stp.pose.position.x = pose.position.x
        pose_stp.pose.position.y = pose.position.y
        pose_stp.pose.position.z = pose.position.z
        pose_stp.pose.orientation = pose.orientation

        return pose_stp


    def pick(self, obj, close_gripper_to=0.02, retry=1, tolerance=0.01, x_diff_pick=-0.01, z_diff_pick=0.1, x_diff_grasp=-0.01, z_diff_grasp=0.01):
        self.gripper_client.fully_open_gripper() ## open gripper

        #===== self.angle ====#
        angle_tmp = 0
        input_retry = retry
        success = False

        while angle_tmp <= self.angle_max and not success:
            radian = (angle_tmp / 2.0) * (pi / 180.0)    ## convert degree to radian
            rot_orientation = Quaternion(0.0, sin(radian), 0.0, cos(radian))

            ## orientation & quarternion
            obj_ori = obj.primitive_poses[0].orientation
            obj_quat = [obj_ori.x, obj_ori.y, obj_ori.z, obj_ori.w]

            ## rotations (roll, pitch, yaw)
            roll, pitch, yaw = transformations.euler_from_quaternion(obj_quat)

            ### extract the yaw orientation from the object 
            yaw_quat = transformations.quaternion_from_euler(0.0, 0.0, yaw)
            yaw_orientation = Quaternion(yaw_quat[0], yaw_quat[1], yaw_quat[2], yaw_quat[3])

            # multiply the previous pitch orientation that the gripper is going to rotate
            ## The new orientation will be the orientation of the gripper link
            gripper_orientation = self.quaternion_multiply(yaw_orientation, rot_orientation)


            # The position of the gripper link will be the position of the cube \\
            # plus x_diff_pick in gripper link x axis direction and plus z_diff_pick in the gripper link z axis direction

            # The reason to do this is that if the we plan the gripper to the exact object position, the grippers might hit the object due to the observation errors.
            first_poseStamped = self.make_poseStamped("base_link", obj.primitive_poses[0], gripper_orientation)
            first_poseStamped.pose.position.x += x_diff_pick     
            first_poseStamped.pose.position.z += z_diff_pick

            while retry > 0:
                move_pose_result = self.move_group.moveToPose(first_poseStamped, "gripper_link", tolerance=tolerance, PLAN_ONLY=True)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")               
                    rospy.loginfo(move_pose_result.error_code.val)
                retry -= 1
            angle_tmp += self.angle_step
            retry = input_retry
        if retry == 0:
            return False

        success = False
        curr_retry = retry
        while angle_tmp  <= 90 and not success:
            radian = (angle_tmp  / 2) * (pi / 180)
            rot_orientation = Quaternion(0.0, sin(radian), 0.0, cos(radian))

            gripper_orientation = self.quaternion_multiply(yaw_orientation, rot_orientation)
            gripper_pose_stamped = self.make_poseStamped("base_link", obj.primitive_poses[0], gripper_orientation)
            gripper_pose_stamped.pose.position.z += z_diff_grasp
            gripper_pose_stamped.pose.position.x += x_diff_grasp

            while curr_retry > 0:
                move_pose_result = self.move_group.moveToPose(gripper_pose_stamped, "gripper_link", tolerance=tolerance)
                rospy.sleep(1.0)
                if move_pose_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    success = True
                    break
                else:
                    if move_pose_result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                        rospy.loginfo("no valid IK found")      
                    rospy.loginfo(move_pose_result.error_code.val)
                curr_retry -= 1
            angle_tmp  += self.angle_step
            curr_retry = retry
            rospy.loginfo("closing the gripper")
            self.makeAttach(obj)
            self.gripper_client.close_gripper_to(close_gripper_to)
        if curr_retry == 0:
            return False
        rospy.loginfo("done picking")
        return True

    ## tuck pose
    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    # stow pose
    def stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        #pose = [-1.60, -1.10, -1.20, -1.50, 0.0, -1.51, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    # intermediate stow pose
    def intermediate_stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


def spawn_gazebo_model(model_path, model_name, model_pose, reference_frame="world"):
  model_xml = ''
  with open(model_path, "r") as model_file:
    model_xml = model_file.read().replace('\n', '')
  rospy.wait_for_service('/gazebo/spawn_sdf_model')
  try:
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    resp_sdf = spawn_sdf(model_name, model_xml, "/", model_pose, reference_frame)
  except rospy.ServiceException, e:
    rospy.logerr("Spawn SDF service call failed: {0}".format(e))


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pack_path = rospack.get_path('fetch_goal_gnrt')
    rospy.init_node("checking") # Create a node


    # get the height of the head pose and look at the origin at half of the head height
    pt_head = PointStamped()
    t = Transformer()
    listener = TransformListener()

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    head_action = PointHeadClient()

    pt_head.header.frame_id = "/head_camera_rgb_optical_frame"
    pt_head = listener.transformPoint("/base_link", pt_head)
    rospy.loginfo("the head position point: %f, %f, %f", pt_head.point.x, pt_head.point.y, pt_head.point.z)
    head_action.look_at(1.0, 0.0, pt_head.point.z, "base_link")


    grasping_client = GraspingClient() # Control scene, robot, arm
    grasping_client.stow() # Go to a default pose
    cube_in_grapper = False

    init_pose = [Pose(position=Point(x=0.8, y=0,   z=0.67)),
                Pose(position=Point(x=0.8, y=-0.2,  z=0.67)),
                Pose(position=Point(x=0.8, y=-0.1,  z=0.67)),
                Pose(position=Point(x=0.55, y=-0.1, z=0.67))]
                

    # object goal info tuble: (index, Pose)
    obj0_goal = (0, Pose(position=Point(x=0.6, y= 0.3,  z= 0.67)))
    obj1_goal = (1, Pose(position=Point(x=0.6, y= -0.3,  z= 0.67)))
    obj2_goal = (2, Pose(position=Point(x=0.6, y= 0.3, z= 0.67)))
    obj3_goal = (3, Pose(position=Point(x=0.6, y= 0.3, z= 0.67)))

    obj_goal = [obj0_goal, obj1_goal, obj2_goal, obj3_goal]

    sorted_init = sorted(range(len(init_pose)), key=lambda ind: (init_pose[ind].position.y, init_pose[ind].position.x))
    # sorted_obj_goal = sorted(obj_goal, key=lambda obj: (obj[1].position.y, -obj[1].position.x))

    sorted_init_pose = []
    sorted_obj_goal = []
    for i in range(len(sorted_init)):
        sorted_init_pose.append(init_pose[sorted_init[i]])
        sorted_obj_goal.append(obj_goal[sorted_init[i]])
    print("init~~~~~~~~:",sorted_init_pose)
    print(sorted_obj_goal)

    models = ['cylinder','cubes/red_cube', 'cubes/blue_cube', 'cylinder']

    object_no = 4
    for b in range(0, object_no): #generate block objects
        object_path = os.path.join(pack_path, 'models', models[b], 'model.sdf')
        object_name = 'obj'+str(b)
        object_pose = init_pose[b]
        spawn_gazebo_model(object_path, object_name, object_pose)
        time.sleep(0.3)

    obj_i = -1
    while not rospy.is_shutdown():
        obj_i += 1
        head_action.look_at(1.2, 0.0, pt_head.point.z / 2, "base_link")
        print("*******Waiting for look at the table")
        time.sleep(2.0)

        # Get block to pick
        fail_ct = 0
        while not rospy.is_shutdown() and not cube_in_grapper:
            rospy.loginfo("Picking object...")
            print("*******obj index: ", obj_i)
            grasping_client.updateScene()

            print("*******goal y index: ", obj_goal[obj_i])
            print("*******goal object y: ", sorted_init_pose[obj_i].position.y)

#            print(grasping_client.__dict__)
            print(sorted_init_pose[obj_i].position.x, sorted_init_pose[obj_i].position.y)
            print(obj_i)
            cube = grasping_client.getGraspableObject(sorted_init_pose[obj_i].position.x, sorted_init_pose[obj_i].position.y)
            
            # Pick the block
            if grasping_client.pick(cube):
                cube_in_grapper = True
                print("Pick Success")
                time.sleep(3.0)
                break
    rospy.loginfo("Finished")
