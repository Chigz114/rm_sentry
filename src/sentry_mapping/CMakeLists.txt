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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

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

install(TARGETS
  traversability_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
