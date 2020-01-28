cmake_minimum_required(VERSION 2.8.3)
project(stefan_planning CXX)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trac_ik_lib
  kdl_parser
)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    trac_ik_lib
    kdl_parser
#  INCLUDE_DIRS include
#  LIBRARIES stefan_planning
#  CATKIN_DEPENDS roscpp rospy std_msgs trac-ik-lib
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/stefan_planning.cpp
# )

add_executable(stefan_planning src/planning.cpp)
target_link_libraries(stefan_planning
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
