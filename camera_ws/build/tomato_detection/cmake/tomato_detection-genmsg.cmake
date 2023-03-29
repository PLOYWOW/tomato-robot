# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tomato_detection: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tomato_detection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_custom_target(_tomato_detection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tomato_detection" "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" ""
)

get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_custom_target(_tomato_detection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tomato_detection" "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomato_detection
)
_generate_srv_cpp(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomato_detection
)

### Generating Module File
_generate_module_cpp(tomato_detection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomato_detection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tomato_detection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tomato_detection_generate_messages tomato_detection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_cpp _tomato_detection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_cpp _tomato_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomato_detection_gencpp)
add_dependencies(tomato_detection_gencpp tomato_detection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomato_detection_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomato_detection
)
_generate_srv_eus(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomato_detection
)

### Generating Module File
_generate_module_eus(tomato_detection
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomato_detection
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tomato_detection_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tomato_detection_generate_messages tomato_detection_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_eus _tomato_detection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_eus _tomato_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomato_detection_geneus)
add_dependencies(tomato_detection_geneus tomato_detection_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomato_detection_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomato_detection
)
_generate_srv_lisp(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomato_detection
)

### Generating Module File
_generate_module_lisp(tomato_detection
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomato_detection
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tomato_detection_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tomato_detection_generate_messages tomato_detection_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_lisp _tomato_detection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_lisp _tomato_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomato_detection_genlisp)
add_dependencies(tomato_detection_genlisp tomato_detection_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomato_detection_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomato_detection
)
_generate_srv_nodejs(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomato_detection
)

### Generating Module File
_generate_module_nodejs(tomato_detection
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomato_detection
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tomato_detection_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tomato_detection_generate_messages tomato_detection_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_nodejs _tomato_detection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_nodejs _tomato_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomato_detection_gennodejs)
add_dependencies(tomato_detection_gennodejs tomato_detection_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomato_detection_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection
)
_generate_srv_py(tomato_detection
  "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection
)

### Generating Module File
_generate_module_py(tomato_detection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tomato_detection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tomato_detection_generate_messages tomato_detection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/SelectTomato.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_py _tomato_detection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hayashi/worksp/camera_ws/src/tomato_detection/srv/switch_cam.srv" NAME_WE)
add_dependencies(tomato_detection_generate_messages_py _tomato_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tomato_detection_genpy)
add_dependencies(tomato_detection_genpy tomato_detection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tomato_detection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomato_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tomato_detection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tomato_detection_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomato_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tomato_detection
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tomato_detection_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomato_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tomato_detection
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tomato_detection_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomato_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tomato_detection
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tomato_detection_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tomato_detection
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tomato_detection_generate_messages_py std_msgs_generate_messages_py)
endif()
