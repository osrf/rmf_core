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

foreach(module ${extra_cmake_modules})
  list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/${module})
endforeach()


install(
  TARGETS rmf_utils
  EXPORT  rmf_utils
)

install(
  FILES test/format/rmf_code_style.cfg
  DESTINATION ${CMAKE_INSTALL_DATADIR}/format
)

add_subdirectory(test/unit)
add_subdirectory(cmake/ament_cmake_catch2)
add_subdirectory(cmake/rmf_cmake_uncrustify)

ament_package()
