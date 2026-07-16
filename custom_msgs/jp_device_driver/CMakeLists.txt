cmake_minimum_required(VERSION 3.0.2)
project(jp_device_driver)

# 设置是否开启调试模式
option(DEBUG "Enable debug mode" ON)#ON 开启调试模式, OFF 关闭调试模式

if(DEBUG)
    # 调试模式
    set(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS "-std=c++17 -O0 -Wall -g -ggdb")
else()
    # 运行模式
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall")
endif()

find_package(Boost REQUIRED COMPONENTS
	thread
	filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  serial
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LidarScanBundle.msg
  CustomPoint.msg
  CustomMsg.msg
  LidarStatus.msg
  RadarStatus.msg
  RadarMsg.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs   std_msgs
)

catkin_package(
   # INCLUDE_DIRS include
#  LIBRARIES jp_lidar_driver
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs
   DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


