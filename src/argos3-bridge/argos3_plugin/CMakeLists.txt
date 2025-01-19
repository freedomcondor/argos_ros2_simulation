project(argos3_plugin)
cmake_minimum_required(VERSION 3.8)

#  find ros2
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# find argos
cmake_policy(SET CMP0072 NEW)
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR})

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

# compile drone controller
add_library(drone_ros2_bridge_controller MODULE
  drone_ros2_bridge_controller.h
  drone_ros2_bridge_controller.cpp
)
target_link_libraries(drone_ros2_bridge_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_drone
    rclcpp::rclcpp
)
ament_target_dependencies(drone_ros2_bridge_controller
  rclcpp
  std_msgs
  geometry_msgs
)

# install drone controller into install folder
install(TARGETS
  drone_ros2_bridge_controller
  DESTINATION lib/${PROJECT_NAME}
)