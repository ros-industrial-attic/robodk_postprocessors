cmake_minimum_required(VERSION 2.8.3)
project(robodk_postprocessors)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  DIRECTORY
  action
  FILES
  Fibonacci.action
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS
  actionlib
  message_runtime
  roscpp
)

###########
## Build ##
###########

## Setup include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install scripts and executables
install(PROGRAMS
  scripts/fibonacci_client.py
  scripts/fibonacci_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

