cmake_minimum_required(VERSION 2.8.3)
project(target_object_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_kinect_euclidean_cluster
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  target_object_detector_node
  src/target_object_detector.cpp
  src/target_object_detector_node.cpp
  )

target_link_libraries(target_object_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

