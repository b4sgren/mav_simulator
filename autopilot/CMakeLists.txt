cmake_minimum_required(VERSION 2.8.3)
project(autopilot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mav_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autopilot
   CATKIN_DEPENDS roscpp rospy std_msgs mav_msgs #message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(autopilot src/autopilot.cpp src/autopilot_node.cpp src/pid_controller.cpp)
target_link_libraries(autopilot ${catkin_LIBRARIES})
add_dependencies(autopilot ${catkin_EXPORTED_TARGETS})
