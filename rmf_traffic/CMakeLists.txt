cmake_minimum_required(VERSION 3.5.0)

project(rmf_traffic VERSION 1.0.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(fcl 0.6 QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  set(using_new_fcl true)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
else()
  include(FindPkgConfig)
  pkg_check_modules(PC_FCL REQUIRED fcl)
  pkg_check_modules(PC_CCD REQUIRED ccd)
  set(FCL_LIBRARIES ${PC_FCL_LIBRARIES} ${PC_CCD_LIBRARIES})
  set(using_new_fcl)
  message(STATUS "Using FCL version: ${PC_FCL_VERSION}")
endif()

find_package(rmf_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)

# ===== Traffic control library
file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic/*.cpp")
add_library(rmf_traffic SHARED
  ${core_lib_srcs}
)

find_package(ament_cmake_catch2 QUIET)
find_package(rmf_cmake_uncrustify QUIET)
if(BUILD_TESTING AND ament_cmake_catch2_FOUND AND rmf_cmake_uncrustify_FOUND)
  file(GLOB_RECURSE unit_test_srcs "test/*.cpp")

  ament_add_catch2(
    #test_rmf_traffic test/main.cpp ${unit_test_srcs}
    test_rmf_traffic test/main.cpp "test/unit/test_Conflict.cpp"
    TIMEOUT 300)
  target_link_libraries(test_rmf_traffic
      rmf_traffic
      ${FCL_LIBRARIES}
      Threads::Threads
  )

  if(using_new_fcl)
    target_compile_definitions(test_rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
  endif()

  target_include_directories(test_rmf_traffic
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/>
  )

  find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")

  rmf_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

if(using_new_fcl)
  target_compile_definitions(rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
endif()

target_link_libraries(rmf_traffic
  PUBLIC
    rmf_utils::rmf_utils
  PRIVATE
    ${FCL_LIBRARIES}
)

target_include_directories(rmf_traffic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_traffic/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_traffic-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY ExactVersion
)

install(
  TARGETS rmf_traffic
  EXPORT  rmf_traffic-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_traffic
  DESTINATION include
)

install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_traffic-targets
  FILE rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_traffic-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
)

export(PACKAGE rmf_traffic)
