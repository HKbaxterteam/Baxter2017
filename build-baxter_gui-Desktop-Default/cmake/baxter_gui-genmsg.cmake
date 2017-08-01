# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "baxter_gui: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(baxter_gui_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv" NAME_WE)
add_custom_target(_baxter_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "baxter_gui" "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(baxter_gui
  "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_gui
)

### Generating Module File
_generate_module_cpp(baxter_gui
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_gui
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(baxter_gui_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(baxter_gui_generate_messages baxter_gui_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv" NAME_WE)
add_dependencies(baxter_gui_generate_messages_cpp _baxter_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_gui_gencpp)
add_dependencies(baxter_gui_gencpp baxter_gui_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_gui_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(baxter_gui
  "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_gui
)

### Generating Module File
_generate_module_lisp(baxter_gui
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_gui
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(baxter_gui_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(baxter_gui_generate_messages baxter_gui_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv" NAME_WE)
add_dependencies(baxter_gui_generate_messages_lisp _baxter_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_gui_genlisp)
add_dependencies(baxter_gui_genlisp baxter_gui_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_gui_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(baxter_gui
  "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_gui
)

### Generating Module File
_generate_module_py(baxter_gui
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_gui
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(baxter_gui_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(baxter_gui_generate_messages baxter_gui_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/n/catkin_ws/src/Baxter2017/baxter_gui/srv/send_command.srv" NAME_WE)
add_dependencies(baxter_gui_generate_messages_py _baxter_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_gui_genpy)
add_dependencies(baxter_gui_genpy baxter_gui_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_gui_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_gui
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_gui
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_gui)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_gui\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_gui
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
