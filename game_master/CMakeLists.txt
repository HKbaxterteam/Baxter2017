cmake_minimum_required(VERSION 2.8.3)
project(game_master)

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


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(FILES 
 ai_game_master.action
 gui_game_master.action
 grasping_baxter_game_master.action
 camera_game_master.action
 game_master_gui.action
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(game_master src/game_master.cpp)

target_link_libraries(
  game_master
  ${catkin_LIBRARIES}
)

add_dependencies(
  game_master
  ${game_master_EXPORTED_TARGETS}
)
add_dependencies(game_master ${catkin_EXPORTED_TARGETS})

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#############
## Install ##
#############
## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


#############
## Testing ##
#############
