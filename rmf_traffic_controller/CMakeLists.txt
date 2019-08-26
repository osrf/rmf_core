cmake_minimum_required(VERSION 3.5.0)

project(rmf_traffic_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FindPkgConfig)
pkg_check_modules(PC_FCL REQUIRED fcl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


# ===== Traffic control library
add_library(rmf_traffic_controller
  src/Schedule.cpp
)

target_include_directories(rmf_traffic_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


# ===== Plan submission node component
add_library(submission_component
  src/submission_component.cpp
)

target_link_libraries(submission_component
  PRIVATE
    rmf_traffic_controller
    ${PC_FCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
)
target_include_directories(submission_component
  PRIVATE
    ${PC_FCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)
