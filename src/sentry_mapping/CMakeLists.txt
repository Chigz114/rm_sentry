cmake_minimum_required(VERSION 3.10)
project(sentry_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rog_map QUIET)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

set(INSTALL_TARGETS)

if(rog_map_FOUND)
  # ROG-Map 的 header 用了若干编译时 macro，但它的 ament export 没把这些
  # compile definitions 传递给下游。这里需要补齐，否则 include rog_map.h 会
  # 触发 #error "Please define either ORIGIN_AT_CORNER or ORIGIN_AT_CENTER..."
  # 以及 "ROOT_DIR was not declared in this scope"。
  add_definitions(
    -DUSE_ROS2
    -DORIGIN_AT_CORNER   # rog_map 默认用格子的角作为原点（对应其 ros2.CMakeLists）
  )
  # ROOT_DIR 原本指 rog_map 源码目录，仅用于 Config::replaceCmakeRootDir()
  # 解析 yaml 里的 ${CMAKE_ROOT_DIR}/ 前缀（用来加载 pcd）。我们 yaml 不用这个
  # 前缀，给一个无害占位值即可。
  add_definitions(-DROOT_DIR="/")

  add_executable(perception_mapper
    src/perception_mapper_node.cpp
  )

  target_include_directories(perception_mapper PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  ament_target_dependencies(perception_mapper
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    tf2_ros
    pcl_conversions
    rog_map
  )

  target_link_libraries(perception_mapper
    ${PCL_LIBRARIES}
    yaml-cpp
  )

  list(APPEND INSTALL_TARGETS perception_mapper)
else()
  message(STATUS "rog_map not found; skipping optional perception_mapper target")
endif()

add_executable(traversability_mapper
  src/traversability_mapper_node.cpp
)

target_include_directories(traversability_mapper PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
)

ament_target_dependencies(traversability_mapper
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2_ros
)

target_link_libraries(traversability_mapper
  ${PCL_LIBRARIES}
  tinyxml2::tinyxml2
)

list(APPEND INSTALL_TARGETS traversability_mapper)

install(TARGETS
  ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
