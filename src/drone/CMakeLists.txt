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
find_package(geometry_msgs REQUIRED)

add_executable(droneFlightSystem src/droneFlightSystem.cpp)
ament_target_dependencies(droneFlightSystem
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS   droneFlightSystem  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY scripts/  DESTINATION share/${PROJECT_NAME}/scripts)

configure_file(scripts/configuration.in.argos scripts/configuration.argos)

#-------------------------------------------------------------------

ament_package()
