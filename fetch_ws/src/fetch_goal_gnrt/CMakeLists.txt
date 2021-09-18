cmake_minimum_required(VERSION 3.0.2)
project(fetch_goal_gnrt)

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
  control_msgs
  grasping_msgs
  geometry_msgs
  trajectory_msgs
  tf2_geometry_msgs
  tf
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

add_executable(fetch_arm_pick_place scripts/pick_cubes.cpp)
target_link_libraries(fetch_arm_pick_place ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
