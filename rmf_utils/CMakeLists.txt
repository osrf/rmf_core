cmake_minimum_required(VERSION 3.5.0)

project(rmf_utils VERSION 1.0.2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

add_library(rmf_utils INTERFACE)

target_include_directories(rmf_utils
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
  DIRECTORY include/rmf_utils
  DESTINATION include
)

install(
  TARGETS rmf_utils
  EXPORT  rmf_utils-targets
  DESTINATION lib
)

install(
  EXPORT rmf_utils-targets
  DESTINATION lib/cmake/rmf_utils
)

install(
  FILES test/format/rmf_code_style.cfg
  DESTINATION ${CMAKE_INSTALL_DATADIR}/format
)

add_subdirectory(test/unit)

# Create cmake config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/rmf_utils-config-version.cmake"
  COMPATIBILITY ExactVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_utils-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/rmf_utils-config.cmake"
  INSTALL_DESTINATION "lib/cmake/rmf_utils"
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/rmf_utils-config-version.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/rmf_utils-config.cmake"
  DESTINATION lib/cmake/rmf_utils
)
export(
  EXPORT rmf_utils-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake/rmf_utils-targets.cmake
)
