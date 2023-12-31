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
# Dynamixel Library
add_library(abstract_dyna src/dynamixel_motors.cpp)
target_link_libraries(abstract_dyna ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/extpos_dynamixel.cpp src/current_dynamixel.cpp src/velocity_dynamixel.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} abstract_dyna)

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