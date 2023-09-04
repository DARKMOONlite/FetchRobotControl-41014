# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fetchit_challenge: 14 messages, 0 services")

set(MSG_I_FLAGS "-Ifetchit_challenge:/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fetchit_challenge_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" "actionlib_msgs/GoalID:fetchit_challenge/SchunkMachineGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" "actionlib_msgs/GoalID:fetchit_challenge/SickCameraResult:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" "actionlib_msgs/GoalID:fetchit_challenge/SickCameraGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" "fetchit_challenge/SchunkMachineActionResult:actionlib_msgs/GoalID:fetchit_challenge/SchunkMachineGoal:actionlib_msgs/GoalStatus:fetchit_challenge/SchunkMachineFeedback:fetchit_challenge/SchunkMachineActionGoal:std_msgs/Header:fetchit_challenge/SchunkMachineResult:fetchit_challenge/SchunkMachineActionFeedback"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" "actionlib_msgs/GoalID:fetchit_challenge/SickCameraFeedback:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" "actionlib_msgs/GoalID:fetchit_challenge/SickCameraActionGoal:actionlib_msgs/GoalStatus:fetchit_challenge/SickCameraActionResult:fetchit_challenge/SickCameraResult:std_msgs/Header:fetchit_challenge/SickCameraGoal:fetchit_challenge/SickCameraFeedback:fetchit_challenge/SickCameraActionFeedback"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:fetchit_challenge/SchunkMachineResult:std_msgs/Header"
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" ""
)

get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_custom_target(_fetchit_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetchit_challenge" "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:fetchit_challenge/SchunkMachineFeedback:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_cpp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
)

### Generating Services

### Generating Module File
_generate_module_cpp(fetchit_challenge
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fetchit_challenge_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fetchit_challenge_generate_messages fetchit_challenge_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_cpp _fetchit_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetchit_challenge_gencpp)
add_dependencies(fetchit_challenge_gencpp fetchit_challenge_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetchit_challenge_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_eus(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
)

### Generating Services

### Generating Module File
_generate_module_eus(fetchit_challenge
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fetchit_challenge_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fetchit_challenge_generate_messages fetchit_challenge_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_eus _fetchit_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetchit_challenge_geneus)
add_dependencies(fetchit_challenge_geneus fetchit_challenge_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetchit_challenge_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_lisp(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
)

### Generating Services

### Generating Module File
_generate_module_lisp(fetchit_challenge
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fetchit_challenge_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fetchit_challenge_generate_messages fetchit_challenge_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_lisp _fetchit_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetchit_challenge_genlisp)
add_dependencies(fetchit_challenge_genlisp fetchit_challenge_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetchit_challenge_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_nodejs(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fetchit_challenge
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fetchit_challenge_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fetchit_challenge_generate_messages fetchit_challenge_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_nodejs _fetchit_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetchit_challenge_gennodejs)
add_dependencies(fetchit_challenge_gennodejs fetchit_challenge_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetchit_challenge_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)
_generate_msg_py(fetchit_challenge
  "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
)

### Generating Services

### Generating Module File
_generate_module_py(fetchit_challenge
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fetchit_challenge_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fetchit_challenge_generate_messages fetchit_challenge_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraAction.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraResult.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SickCameraGoal.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_robot_simulation/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg" NAME_WE)
add_dependencies(fetchit_challenge_generate_messages_py _fetchit_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetchit_challenge_genpy)
add_dependencies(fetchit_challenge_genpy fetchit_challenge_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetchit_challenge_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetchit_challenge
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(fetchit_challenge_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetchit_challenge
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(fetchit_challenge_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetchit_challenge
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(fetchit_challenge_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetchit_challenge
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(fetchit_challenge_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetchit_challenge
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(fetchit_challenge_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
