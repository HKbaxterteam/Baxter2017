cmake_minimum_required(VERSION 2.8.3)
project(ai)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  game_master
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs   
)



## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

find_package(game_master REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
 add_action_files(
    FILES
	DIRECTORY action
    FILES ai_game_master.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs#   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

#catkin_package(
#)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${game_master_INCLUDE_DIRS} )

add_executable(ai src/ai.cpp)

target_link_libraries(
  ai
  ${catkin_LIBRARIES}
  
)

add_dependencies(
  ai
  ${ai_EXPORTED_TARGETS}
  game_master
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs game_master
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)



#
#############
## Install ##
#############

#
#############
## Testing ##
#############
