cmake_minimum_required(VERSION 3.5)

project(rmf_task_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

if (rmf_traffic_FOUND)
  message(STATUS "found rmf_traffic")
  message(STATUS "rmf_traffic_LIBRARIES: ${rmf_traffic_LIBRARIES}")
  message(STATUS "rmf_traffic_INCLUDE_DIRS: ${rmf_traffic_INCLUDE_DIRS}")
endif()

file(GLOB_RECURSE core_lib_srcs "src/dispatcher/*.cpp" "src/bidder/*.cpp")
add_library(rmf_task_ros2 SHARED ${core_lib_srcs})

target_link_libraries(rmf_task_ros2
  PUBLIC
    rmf_traffic::rmf_traffic
    ${rmf_task_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(rmf_task_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

ament_export_interfaces(rmf_task_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic rmf_task_msgs rclcpp)

#===============================================================================

add_executable(rmf_task_dispatcher src/dispatcher/main.cpp)
target_link_libraries(rmf_task_dispatcher PUBLIC rmf_task_ros2)

#===============================================================================
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_task_ros2 rmf_task_dispatcher
  EXPORT rmf_task_ros2
  RUNTIME DESTINATION lib/rmf_task_ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
