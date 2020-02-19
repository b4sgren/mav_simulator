cmake_minimum_required(VERSION 2.8.3)
project(dynamics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tools
  mav_msgs
)
find_package(Eigen3 REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tools 
  CATKIN_DEPENDS roscpp rospy std_msgs mav_msgs tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(dynamics src/mav_dynamics.cpp src/mav_dynamics_node.cpp)
target_link_libraries(dynamics ${catkin_LIBRARIES})

add_executable(windsim src/wind_sim.cpp src/wind_node.cpp)
target_link_libraries(windsim ${catkin_LIBRARIES})
