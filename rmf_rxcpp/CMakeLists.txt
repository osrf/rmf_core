cmake_minimum_required(VERSION 3.5.0)

project(rmf_rxcpp)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(rmf_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")
                
  rmf_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

find_package(rmf_traffic REQUIRED)

# -----------------------------------------------------------------------------

add_executable(rmf_rxcpp_test test/rxcpp.cpp)

target_link_libraries(rmf_rxcpp_test
  PUBLIC
    rmf_traffic::rmf_traffic
)

ament_package()
