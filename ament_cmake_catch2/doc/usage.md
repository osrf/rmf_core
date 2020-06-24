
## Usage

To use this package do the following things.
 - add a test dependency to your packages.xml: `<test_depend>ament_cmake_catch2</test_depend>`
 - Copy the catch2 headers to your project, or install as a system wide package.
 - Write a catch2 main, include the catch.hpp header and define the `CATCH_CONFIG_MAIN` symbol.
 - Find package catch2 in your CMakeLists.txt: `find_package(ament_cmake_catch2 REQUIRED)`.
 - Add a cmake target using the `ament_add_catch2` cmake macro, pass the main.cpp as your first argument and all the test files as additional arguments.
