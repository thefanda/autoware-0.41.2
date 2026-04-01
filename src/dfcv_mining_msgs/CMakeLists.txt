cmake_minimum_required(VERSION 3.5)
project(dfcv_mining_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# ros1
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   message_generation
# )

# ros2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# ros1
# add_message_files(
# FILES
# Plan_param.msg
# System_ADModeEx.msg
# System_ADModeMp.msg
# System_TboxEx.msg
# CAN_TO_CTRL_EBS.msg
# CAN_TO_CTRL_TCU.msg
# BehaviorEx.msg
# BehaviorMp.msg
# MotionEx.msg
# MotionMp.msg
# LatCtrlEX.msg
# LgtCtrlEX.msg
# LiftingEX.msg
# LatCtrlCp.msg
# LatCtrlMp.msg
# LgtCtrlCp.msg
# LiftingMp.msg
# CAN_TO_CTRL_EPS.msg
# CAN_TO_CTRL_HCU.msg
# BodyControl_ADCU.msg
# CAN_TO_CTRL_EPB1.msg
# EPBC1_ADU.msg
# EPBS1_EPB.msg
# PTODE.msg
# PTODE_ADCU.msg
# Radar.msg
# LocationEx.msg
# LocationIMUCp.msg
# PlanLibraryEx.msg
# ETC7.msg
# LgtCtrlMp.msg
# LatCtrlMp.msg
# LocationIMU.msg
# Video_decoded_data.msg
# PerceptionEx.msg
# BoundingBox.msg
# PerceptionPathEx.msg
# Freespace_Points.msg
# FreespaceEx.msg
# CAN_TO_CTRL_Lift.msg
# LiftCtrlEX.msg
# Estimate.msg
# CAN_TO_CTRL_BCM.msg
# LiftCtrlMp.msg
# BodyControl_Mp.msg
# Freespace_Points.msg
# CAN_TO_CTRL_EBC3.msg
# CAN_TO_CTRL_EBC5.msg
# CAN_TO_CTRL_HRW.msg
# VehicleCtrlMp.msg
# VehicleCtrlEx_CANFD.msg
# LocationEx_CANFD.msg
# PathEx.msg
# PathMp.msg
# CamLocation.msg
# CamState.msg
# Point.msg
# GridPathEx.msg
# Location.msg
# LatCtrlPid.msg
# ControlLibraryEx.msg
# plan.msg
# PthPlnEx.msg
# PthPlnMp.msg
# PCan_Msg.msg
# BCan_Msg.msg
# CCan_Msg.msg
# System_DataUploadEx.msg
# System_TBoxReplyEx.msg
# TboxCanFdMsg.msg
# FVCM_Msg.msg
# SweepingEX.msg
# route_planning.msg
# route_planningMp.msg
# VehicleCtrl_DataUpload.msg
# VehicleCtrlEx_CANFD.msg
# SweepingEX.msg
# LanelinePoints.msg
# LaneInfoTotal.msg
# LaneInfoBound.msg
# FreespaceEx.msg
# Diagnosis_Info.msg
# )

# generate_messages(DEPENDENCIES std_msgs)





# ros2
rosidl_generate_interfaces(${PROJECT_NAME}
# "msg/Plan_param.msg"
"msg/SystemADModeEx.msg"
# "msg/System_ADModeMp.msg"
# "msg/System_TboxEx.msg"
# "msg/CAN_TO_CTRL_EBS.msg"
# "msg/CAN_TO_CTRL_TCU.msg"
# "msg/BehaviorEx.msg"
# "msg/BehaviorMp.msg"
# "msg/MotionEx.msg"
# "msg/MotionMp.msg"
"msg/LatCtrlEX.msg"
"msg/LgtCtrlEX.msg"
# "msg/LiftingEX.msg"
# "msg/LatCtrlCp.msg"
# "msg/LatCtrlMp.msg"
# "msg/LgtCtrlCp.msg"
# "msg/LiftingMp.msg"
# "msg/CAN_TO_CTRL_EPS.msg"
# "msg/CAN_TO_CTRL_HCU.msg"
"msg/BodyControlADCU.msg"
# "msg/CAN_TO_CTRL_EPB1.msg"
# "msg/EPBC1_ADU.msg"
# "msg/EPBS1_EPB.msg"
# "msg/PTODE.msg"
# "msg/PTODE_ADCU.msg"
# "msg/Radar.msg"
# "msg/LocationEx.msg"
# "msg/LocationIMUCp.msg"
# "msg/PlanLibraryEx.msg"
# "msg/ETC7.msg"
# "msg/LgtCtrlMp.msg"
# "msg/LatCtrlMp.msg"
"msg/LocationIMU.msg"
# "msg/Video_decoded_data.msg"
# "msg/PerceptionEx.msg"
# "msg/BoundingBox.msg"
# "msg/PerceptionPathEx.msg"
# "msg/Freespace_Points.msg"
# "msg/FreespaceEx.msg"
# "msg/CAN_TO_CTRL_Lift.msg"
# "msg/LiftCtrlEX.msg"
# "msg/Estimate.msg"
# "msg/CAN_TO_CTRL_BCM.msg"
# "msg/LiftCtrlMp.msg"
# "msg/BodyControl_Mp.msg"
# "msg/Freespace_Points.msg"
# "msg/CAN_TO_CTRL_EBC3.msg"
# "msg/CAN_TO_CTRL_EBC5.msg"
# "msg/CAN_TO_CTRL_HRW.msg"
# "msg/VehicleCtrlMp.msg"
# "msg/VehicleCtrlEx_CANFD.msg"
# "msg/LocationEx_CANFD.msg"
# "msg/PathEx.msg"
# "msg/PathMp.msg"
# "msg/CamLocation.msg"
# "msg/CamState.msg"
# "msg/Point.msg"
# "msg/GridPathEx.msg"
# "msg/Location.msg"
# "msg/LatCtrlPid.msg"
# "msg/ControlLibraryEx.msg"
# "msg/plan.msg"
# "msg/PthPlnEx.msg"
# "msg/PthPlnMp.msg"
"msg/PCanMsg.msg"
"msg/BCanMsg.msg"
"msg/CCanMsg.msg"
"msg/SystemDataUploadEx.msg"
"msg/SystemTBoxReplyEx.msg"
"msg/TboxCanFdMsg.msg"
# "msg/FVCM_Msg.msg"
# "msg/SweepingEX.msg"
# "msg/route_planning.msg"
# "msg/route_planningMp.msg"
# "msg/VehicleCtrl_DataUpload.msg"
# "msg/VehicleCtrlEx_CANFD.msg"
"msg/SweepingEX.msg"
# "msg/LanelinePoints.msg"
# "msg/LaneInfoTotal.msg"
# "msg/LaneInfoBound.msg"
# "msg/FreespaceEx.msg"
"msg/DiagnosisInfo.msg"
"msg/GlobalRpt.msg"
"msg/SystemCmdFloat.msg"
"msg/SystemCmdInt.msg"
"msg/SystemRptFloat.msg"
"msg/SystemRptInt.msg"
"msg/WheelSpeedRpt.msg"
"msg/SteeringCmd.msg"
  DEPENDENCIES std_msgs
)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# ros1
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES dfcv_mining_msgs
#   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
# #  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dfcv_mining_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dfcv_mining_msgs_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dfcv_mining_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


# ros2
ament_package()
