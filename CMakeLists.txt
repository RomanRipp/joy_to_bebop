cmake_minimum_required(VERSION 2.8.3)
project(joy_to_bebop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
include_directories(include ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_to_bebop
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(workspace_DIR /Users/Roman/Documents/eclipse/Cworkspace)
#set(rosWorkspace_DIR ${workspace_DIR}/rosWorkspace)

#include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${rosWorkspace_DIR}/src)

add_executable(joy_to_bebop src/main.cpp src/joy_to_bebop.cpp)

## Add cmake target dependencies of the executable/library
target_link_libraries(joy_to_bebop
   ${catkin_LIBRARIES}
)