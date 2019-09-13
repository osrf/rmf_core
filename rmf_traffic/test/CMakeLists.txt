
set(unit_tests
  fcl_test.cpp
  make_shapes.cpp
)

set(unit_test_srcs)
foreach(ut ${unit_tests})
  list(APPEND unit_test_srcs unit/${ut})
endforeach()

add_executable(test_rmf_traffic
  main.cpp
  ${unit_test_srcs}
)

target_link_libraries(test_rmf_traffic
  PRIVATE
    rmf_traffic
    ${PC_FCL_LIBRARIES}
)
