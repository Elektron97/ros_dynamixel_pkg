cmake_minimum_required(VERSION 3.0.2)
project(ros_dynamixel_pkg)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#### Libraries ####
# PID Library
add_library(pid_utils src/pid/pid.cpp)
target_link_libraries(pid_utils ${catkin_LIBRARIES})

# Dynamixel Library
add_library(${PROJECT_NAME} src/dynamixel_utils.cpp src/custom_current_control.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pid_utils)

#### Export Library ####
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)