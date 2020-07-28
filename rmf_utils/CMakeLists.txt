cmake_minimum_required(VERSION 3.5.0)

project(rmf_utils)

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
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
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
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_utils-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rmf_utils-config.cmake
  INSTALL_DESTINATION lib/cmake/rmf_utils
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
# write_basic_package_version_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/rmf_utils-config-version.cmake
#   VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion
# )

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/rmf_utils-config.cmake
    # ${CMAKE_CURRENT_BINARY_DIR}/rmf_utils-config-version.cmake
  DESTINATION lib/cmake/rmf_utils
)
export(
  EXPORT rmf_utils-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_utils-targets.cmake
)
