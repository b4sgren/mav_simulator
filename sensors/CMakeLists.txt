cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  roscpp
  rospy
  std_msgs
  Eigen3
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensors
  CATKIN_DEPENDS mav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(imu src/imu.cpp src/imu_node.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})
