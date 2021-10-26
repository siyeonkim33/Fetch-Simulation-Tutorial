cmake_minimum_required(VERSION 3.0.2)
project(pose_cnn)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  actionlib
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  message_generation
  sensor_msgs
  std_msgs
  control_msgs
  grasping_msgs
  geometry_msgs
  trajectory_msgs
  tf2_geometry_msgs
  tf
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Class.msg
  EachObject.msg
)

## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fetch_goal_gnrt
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(fetch_arm_pick_place scripts/pick_cubes.cpp)
# target_link_libraries(fetch_arm_pick_place ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
