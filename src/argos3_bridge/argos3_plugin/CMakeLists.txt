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

#-------------------------------------------------------------------
add_library(my_extensions SHARED
  # add the debug actuator
  extensions/debug/debug_entity.h
  extensions/debug/debug_entity.cpp
  extensions/debug/debug_default_actuator.h
  extensions/debug/debug_default_actuator.cpp
  # include the loop function
  extensions/my_loop_functions.h
  extensions/my_loop_functions.cpp
)
target_link_libraries(my_extensions
  argos3core_${ARGOS_BUILD_FOR}
  rclcpp::rclcpp
)
ament_target_dependencies(my_extensions
  rclcpp
  std_msgs
  geometry_msgs
)
target_link_libraries(my_extensions "${cpp_typesupport_target}")
if(ARGOS_USE_DOUBLE)
  target_compile_definitions(my_extensions PUBLIC BT_USE_DOUBLE_PRECISION)
endif(ARGOS_USE_DOUBLE)

# install into install folder
install(TARGETS
  my_extensions
  DESTINATION lib/${PROJECT_NAME}
)

if(ARGOS_QTOPENGL_FOUND)
  add_library(my_qtopengl_extensions SHARED
              qtopengl_extensions/my_qtopengl_user_functions.h
              qtopengl_extensions/my_qtopengl_user_functions.cpp
  )
  target_link_libraries(my_qtopengl_extensions
                        my_extensions)
endif(ARGOS_QTOPENGL_FOUND)

# install into install folder
install(TARGETS
  my_qtopengl_extensions
  DESTINATION lib/${PROJECT_NAME}
)