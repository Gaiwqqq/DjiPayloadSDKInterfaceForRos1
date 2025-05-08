cmake_minimum_required(VERSION 3.0.2)

if (NOT USE_SYSTEM_ARCH)
    # select use platform 'LINUX' or 'RTOS' here, reset cache and reload cmake project
    set(USE_SYSTEM_ARCH LINUX)
endif ()

if (USE_SYSTEM_ARCH MATCHES RTOS)
    cmake_minimum_required(VERSION 3.0.2)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_AR arm-none-eabi-ar)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_OBJDUMP arm-none-eabi-objdump)
    set(SIZE arm-none-eabi-size)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif ()

project(payload_sdk_ros1)

if (USE_SYSTEM_ARCH MATCHES LINUX)
    add_definitions(-DSYSTEM_ARCH_LINUX)

    execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
    if (DEVICE_SYSTEM_ID MATCHES x86_64)
        set(LIBRARY_PATH psdk_lib/lib/x86_64-linux-gnu-gcc)
    elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
        set(LIBRARY_PATH psdk_lib/lib/aarch64-linux-gnu-gcc)
    else ()
        message(FATAL_ERROR "FATAL: Please confirm your platform.")
    endif ()

    install(FILES ${LIBRARY_PATH}/libpayloadsdk.a
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
    )
    install(DIRECTORY psdk_lib/include
            DESTINATION "${CMAKE_INSTALL_PREFIX}"
    )
elseif (USE_SYSTEM_ARCH MATCHES RTOS)
    add_definitions(-DSYSTEM_ARCH_RTOS)
    add_subdirectory(samples/sample_c/platform/rtos_freertos/stm32f4_discovery/project/armgcc)
endif ()

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -O3")
#set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
#add_definitions(-D_GNU_SOURCE)

if (MEMORY_LEAK_CHECK_ON MATCHES TRUE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=leak")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=leak")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lasan")
endif ()

if (BUILD_TEST_CASES_ON MATCHES TRUE)
    set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -Wno-deprecated-declarations")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
endif ()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_generation
    tf2_ros
    pcl_conversions
)

add_message_files(
        DIRECTORY msg
        FILES
        imu_60.msg
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
        LIBRARIES payload_sdk_ros1
        CATKIN_DEPENDS roscpp rospy std_msgs com_package flyctrl livox_ros_driver
        DEPENDS system_lib
)

include_directories(
    include
    psdk_lib/include
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

include_directories(${CMAKE_CURRENT_LIST_DIR}/samples/sample_c++/module_sample)
include_directories(${CMAKE_CURRENT_LIST_DIR}/samples/sample_c/module_sample)
include_directories(${CMAKE_CURRENT_LIST_DIR}/samples/sample_c++/platform/linux/common)
include_directories(${CMAKE_CURRENT_LIST_DIR}/samples/sample_c++/platform/linux/manifold2/application)
include_directories(${CMAKE_CURRENT_LIST_DIR}/psdk_lib/include)

file(GLOB_RECURSE MODULE_SAMPLE_SRC
        samples/sample_c++/module_sample/liveview/*.c*
        samples/sample_c++/module_sample/camera_manager/*.c*
        samples/sample_c++/module_sample/perception/*.c*
        samples/sample_c++/module_sample/gimbal/*.c*
        samples/sample_c++/module_sample/flight_controller/*.c*
        samples/sample_c++/module_sample/hms_manager/*.c*
        samples/sample_c++/module_sample/*.c
        samples/sample_c/module_sample/*.c*
)
file(GLOB_RECURSE MODULE_COMMON_SRC samples/sample_c++/platform/linux/common/*.c*)
file(GLOB_RECURSE MODULE_HAL_SRC samples/sample_c++/platform/linux/manifold2/hal/*.c*)
file(GLOB_RECURSE MODULE_APP_SRC samples/sample_c++/platform/linux/manifold2/application/*.c*)

execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_AARCH64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

link_directories(${CMAKE_CURRENT_LIST_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_CURRENT_LIST_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/samples/sample_c++/platform/linux/common/3rdparty)
message(STATUS " - CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
find_package(FFMPEG REQUIRED)
if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_output
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line ${ffmpeg_version_output})
    string(REGEX MATCH " .* " ffmpeg_version ${ffmpeg_version_line})
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version ${ffmpeg_version})

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()
    include_directories(${FFMPEG_INCLUDE_DIR})
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)

find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

message(STATUS " - MODULE_APP_SRC: ${MODULE_APP_SRC}")

add_executable(${PROJECT_NAME}
        ${MODULE_APP_SRC}
        ${MODULE_SAMPLE_SRC}
        ${MODULE_COMMON_SRC}
        ${MODULE_HAL_SRC}
        src/dji_ctrl_midware_node.cpp
        src/payload_sdk_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES})
target_link_libraries(${PROJECT_NAME} /usr/local/lib/libopus.so)
target_link_libraries(${PROJECT_NAME} usb-1.0)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test1 src/test.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(livox_data_trans src/livox_data_trans.cpp)
target_link_libraries(livox_data_trans ${catkin_LIBRARIES} ${PCL_LIBRARIES})
