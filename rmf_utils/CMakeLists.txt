cmake_minimum_required(VERSION 3.5.0)

project(rmf_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

add_library(rmf_utils INTERFACE)

target_include_directories(rmf_utils
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_interfaces(rmf_utils HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

set(extra_cmake_modules
  Catch.cmake
)

install(
  FILES cmake/CatchAddTests.cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake
)

foreach(module ${extra_cmake_modules})
  list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/${module})
endforeach()


install(
  TARGETS rmf_utils
  EXPORT  rmf_utils
)

add_subdirectory(test/unit)

ament_package()
