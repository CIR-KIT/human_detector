cmake_minimum_required(VERSION 2.8.3)
project(fake_target_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp tf
)

add_executable(fake_target_detector_node src/fake_target_detector.cpp)

target_link_libraries(fake_target_detector_node
  ${catkin_LIBRARIES}
)
