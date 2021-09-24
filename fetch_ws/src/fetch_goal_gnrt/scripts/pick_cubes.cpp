// ROS
#include <iostream>
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
#include <grasping_msgs/FindGraspableObjectsResult.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

using namespace std;

struct gObjects{
  grasping_msgs::GraspableObject object;
  double z;
  int num;
};

struct graspObject{
  grasping_msgs::Object object;
  moveit_msgs::Grasp grasps;
};

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
    
    // Find objects client
    const std::string FIND_TOPIC = "basic_grasping_perception/find_objects";
    actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction> find_client;

  public:
    moveit::planning_interface::MoveGroupInterface move_group;
    ros::NodeHandle nh;
    struct gObjects *objects;

    // initializing
    GraspingClient(ros::NodeHandle& nodehandler) : find_client(FIND_TOPIC, true), 
                                          move_group("arm") 
    {
      move_group.setPlanningTime(45.0);
      nh = nodehandler;

      while(!find_client.waitForServer(ros::Duration(2.0)) && nh.ok()){
        ROS_INFO("Waiting for %s ... ", FIND_TOPIC.c_str());
      }
    }

    /* Add collision objects & Scene updating */
    void updateScene(void){
      // find objects
      grasping_msgs::FindGraspableObjectsResult result;
      grasping_msgs::FindGraspableObjectsGoal goal;
      std::vector<moveit_msgs::CollisionObject> collision_objects;

      goal.plan_grasps = true;
      find_client.sendGoal(goal);
      find_client.waitForResult(ros::Duration(2.0));

      result = *(find_client.getResult());
      collision_objects.resize(result.objects.size()+result.support_surfaces.size());
      ROS_INFO("Size of collision objects: %d",result.objects.size()+result.support_surfaces.size());

      /* remove previous objects */
      std::vector<std::string> known = scene.getKnownObjectNames();
      if(known.size()>0)  {scene.removeCollisionObjects(known);}
      // std::vector<std::string> attached = scene.getAttachedObjects();
      // if(attached.size()>0) {scene.removeCollisionObjects(attached);}
      // self.scene.waitForSync()

      grasping_msgs::GraspableObject obj;
      int idx = -1;
      objects = (struct gObjects*) malloc (sizeof(result.objects.size()));

      for(int i = 0; i < result.objects.size(); i++){
        obj = result.objects[i];
        ROS_INFO("%f %f %f",obj.object.primitives[0].dimensions[0], obj.object.primitives[0].dimensions[1], obj.object.primitives[0].dimensions[2]);
        idx += 1;

        /* Basket */
        if (obj.object.primitives[0].dimensions[0] > 0.07 || obj.object.primitives[0].dimensions[1] > 0.07 || obj.object.primitives[0].dimensions[2] > 0.07){
          ROS_INFO("idx: %d", idx);
          ROS_INFO("%s basket is added", (obj.object.properties[0].name).c_str());

          collision_objects[idx].header.frame_id = "base_link";
          collision_objects[idx].id = "basket_" + obj.object.properties[0].name;

          /* Define the primitive and its dimensions. */
          collision_objects[idx].primitives.resize(1);
          collision_objects[idx].primitives[0].dimensions.resize(3);
          collision_objects[idx].primitives[0].dimensions[0] = 0.35;
          collision_objects[idx].primitives[0].dimensions[1] = 0.2;
          collision_objects[idx].primitives[0].dimensions[2] = 0.2;

          collision_objects[idx].primitive_poses.resize(1);
          collision_objects[idx].primitive_poses[0].position.x = 0.8;
          collision_objects[idx].primitive_poses[0].position.y = 0.3;
          collision_objects[idx].primitive_poses[0].position.z = 0.8;
          collision_objects[idx].primitive_poses[0].orientation.w = obj.object.primitive_poses[0].orientation.w;

          // /* Distinguish between "Red" & "Blue" baskets */
          // if(obj.object.properties[0].name == "red")  {collision_objects[idx].primitive_poses[0].position.y = -0.35;} 
          // else                                        {collision_objects[idx].primitive_poses[0].position.y = 0.3;}
        } 
        /* Cubes */
        else{
          ROS_INFO((obj.object.properties[0].name).c_str(), " block is added");

          collision_objects[idx].header.frame_id = "base_link";
          collision_objects[idx].id = obj.object.properties[0].name + std::to_string(idx);

          /* Define the primitive and its dimensions. */
          collision_objects[idx].primitives.resize(1);
          collision_objects[idx].primitives[0] = obj.object.primitives[0];

          collision_objects[idx].primitive_poses.resize(1);
          collision_objects[idx].primitive_poses[0] = obj.object.primitive_poses[0];

          /*************************************
           * 
           * 
           * 
          ***************************************/
          if (collision_objects[idx].primitive_poses[0].position.x < 0.85){
            objects[i].object = obj;
            objects[i].z = obj.object.primitive_poses[0].position.z;
          }
        }

        collision_objects[idx].operation = collision_objects[idx].ADD;
      }


      /* Add tables */
      grasping_msgs::Object table;
      double height;

      for(int i = 0; i < result.support_surfaces.size(); i++){
        table = result.support_surfaces[i];
        idx++;

        collision_objects[idx].id = result.support_surfaces[i].name;
        collision_objects[idx].header.frame_id = "base_link";

        // extend surface to floor, and make wider since we have narrow field of view
        /* Define the primitive and its dimensions. */
        height = table.primitive_poses[0].position.z;

        collision_objects[idx].primitives.resize(1);
        collision_objects[idx].primitives[0].type = collision_objects[idx].primitives[0].BOX;
        collision_objects[idx].primitives[0].dimensions.resize(3);
        collision_objects[idx].primitives[0].dimensions[0] = table.primitives[0].dimensions[0] + 0.1;
        collision_objects[idx].primitives[0].dimensions[1] = 2.0;  // wider
        collision_objects[idx].primitives[0].dimensions[2] = table.primitives[0].dimensions[2] + height;

        collision_objects[idx].primitive_poses.resize(1);
        collision_objects[idx].primitive_poses[0].position.x = table.primitive_poses[0].position.x;
        collision_objects[idx].primitive_poses[0].position.y = table.primitive_poses[0].position.y;
        collision_objects[idx].primitive_poses[0].position.z = table.primitive_poses[0].position.z - height/2.0;
        collision_objects[idx].primitive_poses[0].orientation = table.primitive_poses[0].orientation;

        collision_objects[idx].operation = collision_objects[idx].ADD;
      }

      /* add collision objects into the scene */
      scene.applyCollisionObjects(collision_objects);
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


    void pick(void)
    {
      std::vector<moveit_msgs::Grasp> grasps;
      grasps.resize(1);

      // Setting grasp pose
      grasps[0].grasp_pose.header.frame_id = "base_link";

      // current pose 
      geometry_msgs::PoseStamped current = move_group.getCurrentPose();
      std::cout << current <<std::endl;

      grasps[0].grasp_pose.pose.position.x = current.pose.position.x;
      grasps[0].grasp_pose.pose.position.y = current.pose.position.y;
      grasps[0].grasp_pose.pose.position.z = current.pose.position.z;

      grasps[0].grasp_pose.pose.orientation.x = current.pose.orientation.x + 0.1;
      grasps[0].grasp_pose.pose.orientation.y = current.pose.orientation.y;
      grasps[0].grasp_pose.pose.orientation.z = current.pose.orientation.z;
      grasps[0].grasp_pose.pose.orientation.w = current.pose.orientation.w;

      // Setting pre-grasp approach
      grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
      /* Direction is set as positive x axis */
      grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
      grasps[0].pre_grasp_approach.min_distance = 0.095;
      grasps[0].pre_grasp_approach.desired_distance = 0.115;

      // Setting post-grasp retreat
      grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
      /* Direction is set as positive z axis */
      grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
      grasps[0].post_grasp_retreat.min_distance = 0.1;
      grasps[0].post_grasp_retreat.desired_distance = 0.25;

      // Setting posture of eef before grasp
      std::cout << "open gripper" << std::endl;
      openGripper(grasps[0].pre_grasp_posture);

      std::cout << "close gripper" << std::endl;
      closedGripper(grasps[0].grasp_posture);

      // Set support surface as table1.
      // move_group.setSupportSurfaceName("table1");

      // Call pick to pick up the object using the grasps given
      move_group.pick("object", grasps);
    }

    /* Tuck an arm */
    void tuck(void){
      std::map<std::string, double> target;

      target["shoulder_pan_joint"] = 1.32;
      target["shoulder_lift_joint"] = 1.40;
      target["upperarm_roll_joint"] = -0.2;
      target["elbow_flex_joint"] = 1.72;
      target["forearm_roll_joint"] = 0.0;
      target["wrist_flex_joint"] = 1.66;
      target["wrist_roll_joint"] = 0.0;
      
      while(nh.ok()){
        move_group.setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success){ 
          move_group.move();
          return; 
        }
      }
    }

    /* Stow an arm */
    void stow(void){
      std::map<std::string, double> target;

      target["shoulder_pan_joint"] = 1.32;
      target["shoulder_lift_joint"] = 0.7;
      target["upperarm_roll_joint"] = 0.0;
      target["elbow_flex_joint"] = -2.0;
      target["forearm_roll_joint"] = 0.0;
      target["wrist_flex_joint"] = -0.57;
      target["wrist_roll_joint"] = 0.0;
      
      while(nh.ok()){
        move_group.setJointValueTarget(target);

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // if planning is successul, execute the trajectory
        if(success){ 
          move_group.move();  // Actual execution
          return; 
        }
      }
    }

    /* Stow an arm to intermediate pose */
    void intermediate_stow(void){
      std::map<std::string, double> target;

      target["shoulder_pan_joint"] = 0.7;
      target["shoulder_lift_joint"] = -0.3;
      target["upperarm_roll_joint"] = 0.0;
      target["elbow_flex_joint"] = -0.3;
      target["forearm_roll_joint"] = 0.0;
      target["wrist_flex_joint"] = -0.57;
      target["wrist_roll_joint"] = 0.0;
      
      while(nh.ok()){
        move_group.setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success){ 
          move_group.move();
          return; 
        }
      }
    }
};



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
  // Setup head action client
  PointHeadClient head_client(nh);

  pt_ref.header.frame_id = "base_link";   // set a reference frame
  tfBuffer.transform(pt_ref, pt_head, "head_camera_rgb_optical_frame"); // tranform points from head_camera w.r.t. base_link

  ROS_INFO("Look at ... (%f, %f, %f)", pt_head.point.x, pt_head.point.y, pt_head.point.z / 3 * 2);
  head_client.look_at(1.0, 0.0, pt_head.point.z * 2 / 3, "base_link");
  ros::Duration(4.0).sleep();


  /* Grippers */
  // Setup grasping client
  GraspingClient grasping_client(nh);

  // grasping_client.pick();

  bool cube_in_grapper = false;
  int fail_ct;

  /* Stow the arm */
  grasping_client.stow();
  // grasping_client.intermediate_stow();


  // addCollisionObjects(planning_scene_interface);

  while(!ros::isShuttingDown()){
    // Get block to pick
    fail_ct = 0;

    while (!ros::isShuttingDown() && !cube_in_grapper){
      ROS_INFO("Picking object... (%d)", fail_ct);
      grasping_client.updateScene();


  //           if cube == None:
  //               rospy.logwarn("Perception failed.")
  //               # grasping_client.intermediate_stow()
  //               grasping_client.stow()
  //               head_action.look_at(1.2, 0.0, 0.0, "base_link")
  //               continue
  //           if (cube.name.find("basket")<0):
  //               # Pick the block
  //               if grasping_client.pick(cube, grasps):
  //                   cube_in_grapper = True
  //                   break
  //               rospy.logwarn("Grasping failed.")
  //               grasping_client.stow()
  //               if fail_ct > 15:
  //                   fail_ct = 0
  //                   break
  //               fail_ct += 1
    }
  }

  // // Wait a bit for ROS things to initialize
  // ros::WallDuration(1.0).sleep();

  // pick(group);

  // ros::WallDuration(1.0).sleep();

  // place(group);

  ros::waitForShutdown();
  return 0;
}
