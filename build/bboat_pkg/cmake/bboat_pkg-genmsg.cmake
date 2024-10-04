# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bboat_pkg: 2 messages, 8 services")

set(MSG_I_FLAGS "-Ibboat_pkg:/home/user/bboat_ws/src/bboat_pkg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bboat_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" "std_msgs/Float64"
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" ""
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" ""
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" ""
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" "std_msgs/Float64"
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" ""
)

get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_custom_target(_bboat_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bboat_pkg" "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_msg_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)

### Generating Services
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_cpp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
)

### Generating Module File
_generate_module_cpp(bboat_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bboat_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bboat_pkg_generate_messages bboat_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_cpp _bboat_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bboat_pkg_gencpp)
add_dependencies(bboat_pkg_gencpp bboat_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bboat_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_msg_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)

### Generating Services
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)
_generate_srv_eus(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
)

### Generating Module File
_generate_module_eus(bboat_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bboat_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bboat_pkg_generate_messages bboat_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_eus _bboat_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bboat_pkg_geneus)
add_dependencies(bboat_pkg_geneus bboat_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bboat_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_msg_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)

### Generating Services
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)
_generate_srv_lisp(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
)

### Generating Module File
_generate_module_lisp(bboat_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bboat_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bboat_pkg_generate_messages bboat_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_lisp _bboat_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bboat_pkg_genlisp)
add_dependencies(bboat_pkg_genlisp bboat_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bboat_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_msg_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)

### Generating Services
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)
_generate_srv_nodejs(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
)

### Generating Module File
_generate_module_nodejs(bboat_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bboat_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bboat_pkg_generate_messages bboat_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_nodejs _bboat_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bboat_pkg_gennodejs)
add_dependencies(bboat_pkg_gennodejs bboat_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bboat_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_msg_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)

### Generating Services
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)
_generate_srv_py(bboat_pkg
  "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
)

### Generating Module File
_generate_module_py(bboat_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bboat_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bboat_pkg_generate_messages bboat_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/cmd_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/msg/mode_msg.msg" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_lamb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/next_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/mode_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/lambert_ref_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/current_target_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/gain_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/reset_vsb_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/bboat_ws/src/bboat_pkg/srv/path_description_serv.srv" NAME_WE)
add_dependencies(bboat_pkg_generate_messages_py _bboat_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bboat_pkg_genpy)
add_dependencies(bboat_pkg_genpy bboat_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bboat_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bboat_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bboat_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(bboat_pkg_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bboat_pkg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(bboat_pkg_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bboat_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bboat_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(bboat_pkg_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bboat_pkg_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(bboat_pkg_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bboat_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bboat_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(bboat_pkg_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bboat_pkg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(bboat_pkg_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bboat_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bboat_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(bboat_pkg_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bboat_pkg_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(bboat_pkg_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bboat_pkg
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bboat_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(bboat_pkg_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bboat_pkg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(bboat_pkg_generate_messages_py sensor_msgs_generate_messages_py)
endif()
