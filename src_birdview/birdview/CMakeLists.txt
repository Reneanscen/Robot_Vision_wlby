cmake_minimum_required(VERSION 3.8)
project(zme_birdvision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# 定义输出的assets的路径参考
get_filename_component(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(WORKSPACE_DIR ${SOURCE_DIR} DIRECTORY)
message("SOURCE_DIR's Version is:${SOURCE_DIR}")
message("WORKSPACE_DIR's Version is:${WORKSPACE_DIR}")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# 没有指定CMAKE_BUILD_TYPE 默认为 Release
set(CMAKE_BUILD_TYPE Release)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

# 指定g++的版本 以及 Release编译时代码优化的程度
 set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# 设置colcon build过程打印消息的级别
set(ENV{COLCON_LOG_LEVEL} "info")

# 是否开启OpenMP
if(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
  set(OpenMP_FLAGS -fopenmp)
  set(OpenMP_LIBS gomp)
  add_definitions(-Wno-unknown-pragmas -fopenmp)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
set(custom_image_msg_DIR ${WORKSPACE_DIR}/install/custom_image_msg/share/custom_image_msg/cmake)
find_package(custom_image_msg REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
        ament_cmake
        rclcpp
        std_msgs
        sensor_msgs
        cv_bridge
        message_filters
        custom_image_msg
        tf2_ros
)

include_directories(
        "${PROJECT_SOURCE_DIR}/include/"
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARY_DIRS})
if (${OpenCV_FOUND})
  message("OpenCV's Version is:${OpenCV_VERSION}")
endif ()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
if (${Eigen3_FOUND})
  message("Eigen3's Version is:${Eigen3_VERSION}")
endif ()

# 设置动态库的路径，log系统
add_library(wlbylog SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("current computer architecture is x86_64")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("current computer architecture is arm")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/wlbylog_arm.so)
else()
  message("unknown architecture")
endif()

########## 配置CUDA核函数 ##########
#option(ENABLE_CUDA "Enable CUDA support" OFF)
#if(ENABLE_CUDA)
#  find_package(CUDA REQUIRED)
#  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++14 -Wno-deprecated-gpu-targets)
#  set(CUDA_TOOLKIT_ROOT_DIR "/home/xu/3rdparty/cuda_11.8")
#  set(CUDA_CUDART_LIBRARY "/home/xu/3rdparty/cuda_11.8/lib64/libcudart.so")
#  set(CUDA_TOOLKIT_INCLUDE "/home/xu/3rdparty/cuda_11.8/include")
#
#  cuda_add_executable(${PROJECT_NAME}_cuda
#          cuda_kernel/birdview_kernel.cu
#  )
#
#  target_link_libraries(${PROJECT_NAME}_cuda
#          ${CUDA_LIBRARIES}
#  )
#endif()

######## 配置可执行文件 ###########
set(EXE_NAME ${PROJECT_NAME}_node)

add_executable(${EXE_NAME}
        src/bird_vision_node.cpp
        include/bird_vision/bird_vision.cpp

        include/v4l2_stream/v4l2_capture/v4l2_capture.cpp
        include/v4l2_stream/mix_helper/mix_buffer.cpp
)

ament_target_dependencies(${EXE_NAME}
        ${dependencies}
)

target_link_libraries(${EXE_NAME}
        ${OpenCV_LIBRARIES}
        wlbylog
)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_include_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_include)
  target_link_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/aarch64-linux-gnu_libs)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_include_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_include)
  target_link_directories(${EXE_NAME} PUBLIC ${SOURCE_DIR}/v4l2_stream/3rdparty/agora_rtc_sdk/x86_64-linux-gnu_libs)
else()
  message("There is no agora_rtc_sdk which matches HOST_SYSTEM_PROCESSOR!")
endif()


install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
