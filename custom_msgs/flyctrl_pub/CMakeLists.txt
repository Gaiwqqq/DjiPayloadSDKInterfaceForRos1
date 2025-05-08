cmake_minimum_required(VERSION 2.8.3)
project(flyctrl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  imu_60.msg
  gridMapRL.msg
  MapHeartBeat.msg
  route.msg
  point.msg
  flyctrl_receive.msg
  flyctrl_send.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)

add_executable(flyctrl_pub src/flyctrl_pub.cpp)



add_dependencies(flyctrl_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(flyctrl_pub
${catkin_LIBRARIES}
)











