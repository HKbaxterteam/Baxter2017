cmake_minimum_required(VERSION 2.8.3)
project(gui)


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
  rviz
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)
## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/baxter_panel.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/baxter_panel.cpp
  ${MOC_FILES}
  )
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
    FILES gui_game_master.action
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

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib_msgs
)


add_library(${PROJECT_NAME} ${SOURCE_FILES})

#add_executable(gui src/gui.cpp)


target_link_libraries(
  gui
  ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES}
)

add_dependencies(
  gui
  ${gui_EXPORTED_TARGETS}
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
# Mark config files for installation
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#
#############
## Testing ##
#############
