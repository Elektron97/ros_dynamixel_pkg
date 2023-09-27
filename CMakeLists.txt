cmake_minimum_required(VERSION 3.0.2)
project(ros_dynamixel_pkg)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#### Libraries ####
# Dynamixel Library
add_library(dynamixel_utils src/dynamixel_utils.cpp)
target_link_libraries(dynamixel_utils ${catkin_LIBRARIES})