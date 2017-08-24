cmake_minimum_required(VERSION 2.8.3)
project(grasping_baxter)


## TODO: clean up all the unecesarz alvars ...
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(MSG_DEPS
    ar_track_alvar_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
  ${MSG_DEPS}
  ar_track_alvar

)



## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


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
    FILES grasping_baxter_game_master.action
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

catkin_package(

)
catkin_package(
  LIBRARIES ar_track_alvar
  CATKIN_DEPENDS actionlib_msgs ar_track_alvar_msgs
)

add_executable(grasping_baxter src/grasping_baxter.cpp)

target_link_libraries(
  grasping_baxter ar_track_alvar 
  ${catkin_LIBRARIES}
)

add_dependencies(
  grasping_baxter
  ${grasping_baxter_EXPORTED_TARGETS} ar_track_alvar_msgs_gencpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


#
#############
## Install ##
#############

#
#############
## Testing ##
#############
