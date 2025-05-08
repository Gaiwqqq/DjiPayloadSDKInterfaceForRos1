cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    message_generation
)

add_message_files(
        DIRECTORY msg
        FILES
        CustomPoint.msg
        CustomMsg.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES livox_ros_driver
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

