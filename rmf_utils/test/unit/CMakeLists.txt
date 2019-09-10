
add_library(impl_ptr_test_lib TestClass.cpp)
target_link_libraries(impl_ptr_test_lib PUBLIC rmf_utils)

add_executable(test_impl_ptr test_impl_ptr.cpp)
target_link_libraries(test_impl_ptr PRIVATE impl_ptr_test_lib)
