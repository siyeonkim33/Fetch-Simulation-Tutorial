cmake_minimum_required(VERSION 3.0.2)
project(fetch_object_recog)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  pcl_conversions
  pcl_ros
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
)
find_package(OpenCV REQUIRED)

catkin_package(
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(object_segment scripts/detect_object.cpp)
target_link_libraries(object_segment ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

