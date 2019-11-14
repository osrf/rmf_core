cmake_minimum_required(VERSION 3.5.0)

project(rmf_traffic)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

include(FindPkgConfig)
pkg_check_modules(PC_FCL REQUIRED fcl)
pkg_check_modules(PC_CCD REQUIRED ccd)

find_package(ament_cmake REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO(MXG): rclcpp should not actually be needed here, because rmf_traffic_ros2
# has been split off into another package.

# ===== Traffic control library
file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic/*.cpp")
add_library(rmf_traffic SHARED
  ${core_lib_srcs}
)

if(BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)

  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    test_rmf_traffic test/main.cpp ${unit_test_srcs}
    TIMEOUT 300)
  target_link_libraries(test_rmf_traffic
      rmf_traffic
      ${PC_FCL_LIBRARIES}
  )

  target_include_directories(test_rmf_traffic
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )
endif()

target_link_libraries(rmf_traffic
  PUBLIC
    rmf_utils::rmf_utils
    ${rclcpp_LIBRARIES}
  PRIVATE
    ${PC_FCL_LIBRARIES}
    ${PC_CCD_LIBRARIES}
)

target_include_directories(rmf_traffic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_export_interfaces(rmf_traffic HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_utils rclcpp)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_traffic
  EXPORT  rmf_traffic
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# ===== Plan submission node component
add_library(submission_component SHARED
  src/submission_component.cpp
)

target_link_libraries(submission_component
  PRIVATE
    rmf_traffic
    ${PC_FCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
)
target_include_directories(submission_component
  PRIVATE
    ${PC_FCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_package()
