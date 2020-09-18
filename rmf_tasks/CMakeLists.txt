cmake_minimum_required(VERSION 3.5.0)

project(rmf_tasks VERSION 1.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

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

find_package(rmf_utils REQUIRED)
find_package(rmf_traffic REQUIRED)

# ===== Traffic control library
file(GLOB lib_srcs
  "src/agv/*.cpp"
  "src/*.cpp"
)

add_library(rmf_tasks SHARED
  ${lib_srcs}
)

target_link_libraries(rmf_tasks
  PRIVATE
    rmf_utils::rmf_utils
    rmf_traffic::rmf_traffic
)

target_include_directories(rmf_tasks
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# find_package(ament_cmake_catch2 QUIET)
# find_package(rmf_cmake_uncrustify QUIET)
# if(BUILD_TESTING AND ament_cmake_catch2_FOUND AND rmf_cmake_uncrustify_FOUND)
#   file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

#   ament_add_catch2(
#     test_rmf_traffic test/main.cpp ${unit_test_srcs}
#     TIMEOUT 300)
#   target_link_libraries(test_rmf_traffic
#       rmf_traffic
#       ${PC_FCL_LIBRARIES}
#       Threads::Threads
#   )

#   target_include_directories(test_rmf_traffic
#     PRIVATE
#       $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
#   )

#   find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")

#   rmf_uncrustify(
#     ARGN include src test
#     CONFIG_FILE ${uncrustify_config_file}
#     MAX_LINE_LENGTH 80
#   )
# endif()

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_tasks/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_tasks-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_tasks-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_tasks-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)

install(
  TARGETS rmf_tasks
  EXPORT  rmf_tasks-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_tasks
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_tasks-targets
  FILE rmf_tasks-targets.cmake
  NAMESPACE rmf_tasks::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_tasks-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_tasks-targets.cmake
  NAMESPACE rmf_tasks::
)

export(PACKAGE rmf_tasks)
