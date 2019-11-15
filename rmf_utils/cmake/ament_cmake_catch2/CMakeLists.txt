cmake_minimum_required(VERSION 3.5)

project(ament_cmake_catch2 NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(CONFIG_EXTRAS
  cmake/ament_add_catch2.cmake
  cmake/ament_add_catch2_test.cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
