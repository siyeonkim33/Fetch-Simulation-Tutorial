// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Action client
#include <actionlib/client/simple_action_client.h>

// Message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/PointHeadGoal.h>
#include <grasping_msgs/FindGraspableObjectsAction.h>
#include <grasping_msgs/FindGraspableObjectsGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


// Point the head using controller
class PointHeadClient{
  private:
    const std::string HEAD_ACTION_NAME = "/head_controller/point_head";
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_client;

  public:
    // initializing
    PointHeadClient(ros::NodeHandle& nh) : head_client(HEAD_ACTION_NAME, true){ 
      while(!head_client.waitForServer(ros::Duration(2.0)) && nh.ok()){
        ROS_INFO("Waiting for %s ... ", HEAD_ACTION_NAME.c_str());
      }
    }

    void look_at(double x, double y, double z, std::string frame){
      control_msgs::PointHeadGoal goal;

      goal.target.header.stamp = ros::Time::now();
      goal.target.header.frame_id = frame;
      goal.target.point.x = x; 
      goal.target.point.y = y; 
      goal.target.point.z = z;
      goal.min_duration = ros::Duration(1.0);

      //send the goal
      head_client.sendGoal(goal);

      //wait for it to get there (abort after 2 secs to prevent getting stuck)
      head_client.waitForResult(ros::Duration(2.0));
    }
};


// Tools for grasping
class GraspingClient{
  private:  
    moveit::planning_interface::PlanningSceneInterface scene;
    moveit::planning_interface::MoveGroupInterface move_group;
    
    // Find objects client
    const std::string FIND_TOPIC = "basic_grasping_perception/find_objects";
    actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction> find_client;

  public:
    // initializing
    GraspingClient(ros::NodeHandle& nh) : find_client(FIND_TOPIC, true), 
                                          move_group("arm") 
    {
      move_group.setPlanningTime(45.0);

      while(!find_client.waitForServer(ros::Duration(2.0)) && nh.ok()){
        ROS_INFO("Waiting for %s ... ", FIND_TOPIC.c_str());
      }
    }

    // Open the grippers
    void openGripper(trajectory_msgs::JointTrajectory& posture){
      /* Add both finger joints of fetch robot. */
      posture.joint_names.resize(2);
      posture.joint_names[0] = "r_gripper_finger_link";
      posture.joint_names[1] = "l_gripper_finger_link";

      /* Set them as opened. */
      posture.points.resize(1);
      posture.points[0].positions.resize(2);
      posture.points[0].positions[0] = 0.04;
      posture.points[0].positions[1] = 0.04;
      posture.points[0].time_from_start = ros::Duration(0.5);
    }

    // Close the grippers
    void closedGripper(trajectory_msgs::JointTrajectory& posture){
      /* Add both finger joints of fetch robot. */
      posture.joint_names.resize(2);
      posture.joint_names[0] = "r_gripper_finger_link";
      posture.joint_names[1] = "l_gripper_finger_link";

      /* Set them as closed. */
      posture.points.resize(1);
      posture.points[0].positions.resize(2);
      posture.points[0].positions[0] = 0.00;
      posture.points[0].positions[1] = 0.00;
      posture.points[0].time_from_start = ros::Duration(0.5);
    }
};


// void pick(moveit::planning_interface::MoveGroupInterface& move_group)
// {

//   std::vector<moveit_msgs::Grasp> grasps;
//   grasps.resize(1);

//   // Setting grasp pose
//   // ++++++++++++++++++++++
//   // This is the pose of panda_link8. |br|
//   // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
//   // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
//   // transform from `"panda_link8"` to the palm of the end effector.
//   grasps[0].grasp_pose.header.frame_id = "base_link";
//   tf2::Quaternion orientation;
//   orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
//   grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//   grasps[0].grasp_pose.pose.position.x = 0.415;
//   grasps[0].grasp_pose.pose.position.y = 0;
//   grasps[0].grasp_pose.pose.position.z = 0.5;

//   // Setting pre-grasp approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
//   /* Direction is set as positive x axis */
//   grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//   grasps[0].pre_grasp_approach.min_distance = 0.095;
//   grasps[0].pre_grasp_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
//   /* Direction is set as positive z axis */
//   grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//   grasps[0].post_grasp_retreat.min_distance = 0.1;
//   grasps[0].post_grasp_retreat.desired_distance = 0.25;

//   // Setting posture of eef before grasp
//   // +++++++++++++++++++++++++++++++++++
//   openGripper(grasps[0].pre_grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick2
//   // Setting posture of eef during grasp
//   // +++++++++++++++++++++++++++++++++++
//   closedGripper(grasps[0].grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick3
//   // Set support surface as table1.
//   move_group.setSupportSurfaceName("table1");
//   // Call pick to pick up the object using the grasps given
//   move_group.pick("object", grasps);
//   // END_SUB_TUTORIAL
// }

// void place(moveit::planning_interface::MoveGroupInterface& group)
// {
//   // BEGIN_SUB_TUTORIAL place
//   // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
//   // location in verbose mode." This is a known issue. |br|
//   // |br|
//   // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
//   // a single place location.
//   std::vector<moveit_msgs::PlaceLocation> place_location;
//   place_location.resize(1);

//   // Setting place location pose
//   // +++++++++++++++++++++++++++
//   place_location[0].place_pose.header.frame_id = "base_link";
//   tf2::Quaternion orientation;
//   orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
//   place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

//   /* For place location, we set the value to the exact location of the center of the object. */
//   place_location[0].place_pose.pose.position.x = 0;
//   place_location[0].place_pose.pose.position.y = 0.5;
//   place_location[0].place_pose.pose.position.z = 0.5;

//   // Setting pre-place approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
//   /* Direction is set as negative z axis */
//   place_location[0].pre_place_approach.direction.vector.z = -1.0;
//   place_location[0].pre_place_approach.min_distance = 0.095;
//   place_location[0].pre_place_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
//   /* Direction is set as negative y axis */
//   place_location[0].post_place_retreat.direction.vector.y = -1.0;
//   place_location[0].post_place_retreat.min_distance = 0.1;
//   place_location[0].post_place_retreat.desired_distance = 0.25;

//   // Setting posture of eef after placing object
//   // +++++++++++++++++++++++++++++++++++++++++++
//   /* Similar to the pick case */
//   openGripper(place_location[0].post_place_posture);

//   // Set support surface as table2.
//   group.setSupportSurfaceName("table2");
//   // Call place to place the object using the place locations given.
//   group.place("object", place_location);
//   // END_SUB_TUTORIAL
// }


// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.resize(10);

//   // Add the first table where the cube will originally be kept.
//   collision_objects[0].id = "table";
//   collision_objects[0].header.frame_id = "panda_link0";

//   /* Define the primitive and its dimensions. */
//   collision_objects[0].primitives.resize(1);
//   collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
//   collision_objects[0].primitives[0].dimensions.resize(3);
//   collision_objects[0].primitives[0].dimensions[0] = 0.2;
//   collision_objects[0].primitives[0].dimensions[1] = 0.4;
//   collision_objects[0].primitives[0].dimensions[2] = 0.4;

//   /* Define the pose of the table. */
//   collision_objects[0].primitive_poses.resize(1);
//   collision_objects[0].primitive_poses[0].position.x = 0.5;
//   collision_objects[0].primitive_poses[0].position.y = 0;
//   collision_objects[0].primitive_poses[0].position.z = 0.2;
//   collision_objects[0].primitive_poses[0].orientation.w = 1.0;
//   // END_SUB_TUTORIAL

//   collision_objects[0].operation = collision_objects[0].ADD;

//   // BEGIN_SUB_TUTORIAL table2
//   // Add the second table where we will be placing the cube.
//   collision_objects[1].id = "table2";
//   collision_objects[1].header.frame_id = "panda_link0";

//   /* Define the primitive and its dimensions. */
//   collision_objects[1].primitives.resize(1);
//   collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[1].primitives[0].dimensions.resize(3);
//   collision_objects[1].primitives[0].dimensions[0] = 0.4;
//   collision_objects[1].primitives[0].dimensions[1] = 0.2;
//   collision_objects[1].primitives[0].dimensions[2] = 0.4;

//   /* Define the pose of the table. */
//   collision_objects[1].primitive_poses.resize(1);
//   collision_objects[1].primitive_poses[0].position.x = 0;
//   collision_objects[1].primitive_poses[0].position.y = 0.5;
//   collision_objects[1].primitive_poses[0].position.z = 0.2;
//   collision_objects[1].primitive_poses[0].orientation.w = 1.0;
//   // END_SUB_TUTORIAL

//   collision_objects[1].operation = collision_objects[1].ADD;

//   // BEGIN_SUB_TUTORIAL object
//   // Define the object that we will be manipulating
//   collision_objects[2].header.frame_id = "base_link";
//   collision_objects[2].id = "object";

//   /* Define the primitive and its dimensions. */
//   collision_objects[2].primitives.resize(1);
//   collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[2].primitives[0].dimensions.resize(3);
//   collision_objects[2].primitives[0].dimensions[0] = 0.02;
//   collision_objects[2].primitives[0].dimensions[1] = 0.02;
//   collision_objects[2].primitives[0].dimensions[2] = 0.2;

//   /* Define the pose of the object. */
//   collision_objects[2].primitive_poses.resize(1);
//   collision_objects[2].primitive_poses[0].position.x = 0.5;
//   collision_objects[2].primitive_poses[0].position.y = 0;
//   collision_objects[2].primitive_poses[0].position.z = 0.5;
//   collision_objects[2].primitive_poses[0].orientation.w = 1.0;
//   // END_SUB_TUTORIAL

//   collision_objects[2].operation = collision_objects[2].ADD;

//   planning_scene_interface.applyCollisionObjects(collision_objects);
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fetch_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // declare variables or buffers
  geometry_msgs::PointStamped pt_ref; 
  geometry_msgs::PointStamped pt_head;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::WallDuration(1.0).sleep();


  /* Head camera */
  // Setup clients
  PointHeadClient head_client(nh);

  pt_ref.header.frame_id = "base_link";   // set a reference frame
  tfBuffer.transform(pt_ref, pt_head, "head_camera_rgb_optical_frame"); // tranform points from head_camera w.r.t. base_link

  ROS_INFO("Look at ... (%f, %f, %f)", pt_head.point.x, pt_head.point.y, pt_head.point.z / 3 * 2);
  head_client.look_at(1.0, 0.0, pt_head.point.z * 2 / 3, "base_link");
  ros::Duration(4.0).sleep();


  /* Grippers */
  GraspingClient grasping_client(nh);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.header.frame_id = "base_link";
  grasping_client.openGripper(grasps[0]);
  grasping_client.closedGripper(grasps[0])

  // addCollisionObjects(planning_scene_interface);

  // // Wait a bit for ROS things to initialize
  // ros::WallDuration(1.0).sleep();

  // pick(group);

  // ros::WallDuration(1.0).sleep();

  // place(group);

  ros::waitForShutdown();
  return 0;
}
