cmake_minimum_required(VERSION 3.8)
project(zme_fusion_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(zme_msg_srv REQUIRED)
find_package(custom_image_msg REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl")
find_package(PCL 1.10 REQUIRED) # 确保版本与你的系统安装的 PCL 版本匹配



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 映射源文件与可执行文件
add_executable(${PROJECT_NAME}_node
  src/fusion_map.cpp
  src/ESDFMap.cpp
  src/raycast.cpp
  src/timing.cc
)

ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        sensor_msgs
        std_msgs
        tf2_ros
        tf2
        geometry_msgs
        laser_geometry
        nav_msgs
        tf2_eigen
        pcl_conversions
        zme_msg_srv
        custom_image_msg
        visualization_msgs)



target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

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
