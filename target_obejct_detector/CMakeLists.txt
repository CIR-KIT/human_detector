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
find_package(PCL 1.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_kinect_euclidean_cluster
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  target_object_detector_node
  src/target_object_detector.cpp
  src/target_object_detector_node.cpp
  src/libsvm/svm.cpp
  )

target_link_libraries(target_object_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(
  segment_cluster_creator_node
  src/segment_cluster_creator.cpp
  src/segment_cluster_creator_node.cpp
  )

target_link_libraries(segment_cluster_creator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(
  train_data_create_tool
  src/train_data_create_tool.cpp
  )

target_link_libraries(train_data_create_tool
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
