cmake_minimum_required(VERSION 2.8.3)
project(camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  OpenCV
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
   include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
 add_action_files(
    FILES
	DIRECTORY action
    FILES camera_game_master.action
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
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(camera src/camera.cpp)

target_link_libraries(
  camera
  ${catkin_LIBRARIES}
)

add_dependencies(
  camera
  ${camera_EXPORTED_TARGETS}
)

add_executable(board_cutout src/board_cutout.cpp)

target_link_libraries(
  board_cutout
  ${catkin_LIBRARIES}
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
