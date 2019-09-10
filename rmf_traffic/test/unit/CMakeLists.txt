
add_executable(test_make_shapes make_shapes.cpp)
target_link_libraries(test_make_shapes PRIVATE rmf_traffic)


# ===== Simple executable for testing fcl
add_executable(fcl_test fcl_test.cpp)

target_link_libraries(fcl_test
  PRIVATE
    ${PC_FCL_LIBRARIES}
)
