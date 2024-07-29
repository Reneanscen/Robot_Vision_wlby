cmake_minimum_required(VERSION 3.8)
project(semantic_mapping)

set(WORKSPACE_DIR ${PROJECT_SOURCE_DIR}/../..)
set(COMMON_TOOLS_DIR ${WORKSPACE_DIR}/src/common_tools)
set(CMAKE_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/install/segment_msg/share/segment_msg/cmake)
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/install/control_msg/share/control_msg/cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(segment_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(CUDA REQUIRED)


set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++14 -Wno-deprecated-gpu-targets)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CUDA_CUDART_LIBRARY "/usr/local/cuda/lib64/libcudart.so")
set(CUDA_TOOLKIT_INCLUDE "/usr/local/cuda/include")

include_directories(
        ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}
        ${COMMON_TOOLS_DIR}
        ${PROJECT_SOURCE_DIR}/include
        "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        ${WORKSPACE_DIR}/install/control_msg/include/control_msg
        ${WORKSPACE_DIR}/install/segment_msg/include/segment_msg
        ${PCL_INCLUDE_DIRS}
        ${WORKSPACE_DIR}/src/yolov8_ncnn/include
)

add_library(PoseUtils_lib
        ${COMMON_TOOLS_DIR}/PoseUtils/angle_math_utils.cpp
        ${COMMON_TOOLS_DIR}/PoseUtils/Matrix.cpp
        ${COMMON_TOOLS_DIR}/PoseUtils/Pose2DDataStruct.cpp
        ${COMMON_TOOLS_DIR}/PoseUtils/Pose3DDataStruct.cpp
        ${COMMON_TOOLS_DIR}/PoseUtils/rot_fuc_deg.cpp
        ${COMMON_TOOLS_DIR}/PoseUtils/rot_fuc_deg2d.cpp
)

cuda_add_library(${PROJECT_NAME}_cuda
        src/depseg_kernel.cu
)
target_link_libraries(${PROJECT_NAME}_cuda ${CUDA_LIBRARIES})

add_executable(${PROJECT_NAME}_node

        src/semantic_mapping_node.cpp
        src/semantic_integrator.cpp
        src/parameter.cpp
        src/main_node.cpp
        src/dbscan.cpp


        ${COMMON_TOOLS_DIR}/common/time_utils.cpp
        ${COMMON_TOOLS_DIR}/common/common_type_define.cpp

        ${COMMON_TOOLS_DIR}/message_report/message_report.cpp

        ${COMMON_TOOLS_DIR}/sensor_data/sensor_data.cpp
        ${COMMON_TOOLS_DIR}/sensor_data/sensor_data_parser.cpp
        ${COMMON_TOOLS_DIR}/sensor_data/sensor_data_buffer.cpp
        ${COMMON_TOOLS_DIR}/sensor_data/synchron_sensor_data.cpp
)

# 设置动态库的路径，log系统
add_library(wlbylog SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("current computer architecture is x86_64")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${COMMON_TOOLS_DIR}/lib/wlbylog.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("current computer architecture is arm")
  set_target_properties(wlbylog PROPERTIES IMPORTED_LOCATION ${COMMON_TOOLS_DIR}/lib/wlbylog_arm.so)
else()
  message("unknown architecture")
endif()

if(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
  set(OpenMP_FLAGS -fopenmp)
  set(OpenMP_LIBS gomp)
  add_definitions(-Wno-unknown-pragmas -fopenmp)
endif()


ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        sensor_msgs
        segment_msg
        message_filters
        geometry_msgs
        nav_msgs
        visualization_msgs
        control_msg
        cv_bridge
        pcl_conversions
)

target_link_libraries(${PROJECT_NAME}_node
        PoseUtils_lib
        ${OpenMP_LIBS}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}_cuda
        wlbylog)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
