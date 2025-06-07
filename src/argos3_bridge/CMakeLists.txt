cmake_minimum_required(VERSION 3.8)
project(argos3_bridge)

find_package(ament_cmake REQUIRED)

add_subdirectory(argos3_plugin)

ament_package()