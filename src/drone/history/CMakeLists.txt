cmake_minimum_required(VERSION 3.8)
project(drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

#-------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mathlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Path.msg"
  "msg/PoseSharing.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(droneFlightSystem src/droneFlightSystem.cpp)
ament_target_dependencies(droneFlightSystem
  rclcpp
  std_msgs
  geometry_msgs
  mathlib
)

add_executable(droneSwarmSystem src/droneSwarmSystem.cpp)
ament_target_dependencies(droneSwarmSystem
  rclcpp
  std_msgs
  geometry_msgs
  mathlib
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(droneFlightSystem "${cpp_typesupport_target}")
target_link_libraries(droneSwarmSystem "${cpp_typesupport_target}")

install(TARGETS   droneFlightSystem  DESTINATION lib/${PROJECT_NAME})
install(TARGETS   droneSwarmSystem  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY scripts/  DESTINATION share/${PROJECT_NAME}/scripts)

configure_file(scripts/configuration_example.in.argos scripts/configuration_example.argos)
configure_file(scripts/configuration_template.in.argos scripts/configuration_template.argos)

#-------------------------------------------------------------------

ament_package()
