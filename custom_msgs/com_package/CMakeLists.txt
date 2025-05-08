cmake_minimum_required(VERSION 3.5.0)
project(com_package LANGUAGES CXX)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pclound.msg
  imu_60.msg
  flyctrl_receive.msg
  flyctrl_send.msg
  fly.msg
  point.msg
  route.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES com_package
 CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
