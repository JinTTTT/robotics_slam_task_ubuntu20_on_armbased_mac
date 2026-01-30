# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rbo_create: 1 messages, 3 services")

set(MSG_I_FLAGS "-Irbo_create:/workspace/src/rbo_create/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rbo_create_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_custom_target(_rbo_create_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbo_create" "/workspace/src/rbo_create/msg/SensorPacket.msg" "std_msgs/Header"
)

get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_custom_target(_rbo_create_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbo_create" "/workspace/src/rbo_create/srv/ResetOdom.srv" ""
)

get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_custom_target(_rbo_create_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbo_create" "/workspace/src/rbo_create/srv/Leds.srv" ""
)

get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_custom_target(_rbo_create_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbo_create" "/workspace/src/rbo_create/srv/Tank.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rbo_create
  "/workspace/src/rbo_create/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
)

### Generating Services
_generate_srv_cpp(rbo_create
  "/workspace/src/rbo_create/srv/ResetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
)
_generate_srv_cpp(rbo_create
  "/workspace/src/rbo_create/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
)
_generate_srv_cpp(rbo_create
  "/workspace/src/rbo_create/srv/Tank.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
)

### Generating Module File
_generate_module_cpp(rbo_create
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rbo_create_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rbo_create_generate_messages rbo_create_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_dependencies(rbo_create_generate_messages_cpp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_cpp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_cpp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_cpp _rbo_create_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbo_create_gencpp)
add_dependencies(rbo_create_gencpp rbo_create_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbo_create_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rbo_create
  "/workspace/src/rbo_create/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
)

### Generating Services
_generate_srv_eus(rbo_create
  "/workspace/src/rbo_create/srv/ResetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
)
_generate_srv_eus(rbo_create
  "/workspace/src/rbo_create/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
)
_generate_srv_eus(rbo_create
  "/workspace/src/rbo_create/srv/Tank.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
)

### Generating Module File
_generate_module_eus(rbo_create
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rbo_create_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rbo_create_generate_messages rbo_create_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_dependencies(rbo_create_generate_messages_eus _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_eus _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_eus _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_eus _rbo_create_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbo_create_geneus)
add_dependencies(rbo_create_geneus rbo_create_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbo_create_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rbo_create
  "/workspace/src/rbo_create/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
)

### Generating Services
_generate_srv_lisp(rbo_create
  "/workspace/src/rbo_create/srv/ResetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
)
_generate_srv_lisp(rbo_create
  "/workspace/src/rbo_create/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
)
_generate_srv_lisp(rbo_create
  "/workspace/src/rbo_create/srv/Tank.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
)

### Generating Module File
_generate_module_lisp(rbo_create
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rbo_create_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rbo_create_generate_messages rbo_create_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_dependencies(rbo_create_generate_messages_lisp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_lisp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_lisp _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_lisp _rbo_create_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbo_create_genlisp)
add_dependencies(rbo_create_genlisp rbo_create_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbo_create_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rbo_create
  "/workspace/src/rbo_create/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
)

### Generating Services
_generate_srv_nodejs(rbo_create
  "/workspace/src/rbo_create/srv/ResetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
)
_generate_srv_nodejs(rbo_create
  "/workspace/src/rbo_create/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
)
_generate_srv_nodejs(rbo_create
  "/workspace/src/rbo_create/srv/Tank.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
)

### Generating Module File
_generate_module_nodejs(rbo_create
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rbo_create_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rbo_create_generate_messages rbo_create_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_dependencies(rbo_create_generate_messages_nodejs _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_nodejs _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_nodejs _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_nodejs _rbo_create_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbo_create_gennodejs)
add_dependencies(rbo_create_gennodejs rbo_create_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbo_create_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rbo_create
  "/workspace/src/rbo_create/msg/SensorPacket.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
)

### Generating Services
_generate_srv_py(rbo_create
  "/workspace/src/rbo_create/srv/ResetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
)
_generate_srv_py(rbo_create
  "/workspace/src/rbo_create/srv/Leds.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
)
_generate_srv_py(rbo_create
  "/workspace/src/rbo_create/srv/Tank.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
)

### Generating Module File
_generate_module_py(rbo_create
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rbo_create_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rbo_create_generate_messages rbo_create_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/workspace/src/rbo_create/msg/SensorPacket.msg" NAME_WE)
add_dependencies(rbo_create_generate_messages_py _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/ResetOdom.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_py _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Leds.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_py _rbo_create_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/workspace/src/rbo_create/srv/Tank.srv" NAME_WE)
add_dependencies(rbo_create_generate_messages_py _rbo_create_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbo_create_genpy)
add_dependencies(rbo_create_genpy rbo_create_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbo_create_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbo_create
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rbo_create_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rbo_create_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rbo_create
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rbo_create_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rbo_create_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbo_create
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rbo_create_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rbo_create_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rbo_create
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rbo_create_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rbo_create_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbo_create
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rbo_create_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rbo_create_generate_messages_py std_msgs_generate_messages_py)
endif()
