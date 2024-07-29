cmake_minimum_required(VERSION 3.8)
project(yolov8_ncnn)

set(WORKSPACE_DIR ${PROJECT_SOURCE_DIR}/../..)
set(COMMON_TOOLS_DIR ${WORKSPACE_DIR}/src/common_tools)
set(CMAKE_BUILD_TYPE Release)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(segment_msg)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ncnn REQUIRED)
find_package(Vulkan REQUIRED)


include_directories(
        ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${COMMON_TOOLS_DIR}
        "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        "/usr/local/ncnn/include"
        ${WORKSPACE_DIR}/install/segment_msg/include
)

add_executable(${PROJECT_NAME}_node

        src/main.cpp
        src/yolov8_ncnn_node.cpp
        src/parameter.cpp
        src/yolo.cpp

        ${COMMON_TOOLS_DIR}/common/time_utils.cpp
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


ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        sensor_msgs
        segment_msg
        builtin_interfaces
        cv_bridge
)

target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBS}
        ${Vulkan_LIBRARIES}
        ncnn
        wlbylog)

install(TARGETS ${PROJECT_NAME}_node
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
