
set(unit_tests
  fcl_test.cpp
  make_shapes.cpp
  test_Trajectory.cpp
  test_Conflict.cpp
  test_Spline.cpp
  agv/test_Interpolate.cpp
  agv/test_VehicleTraits.cpp
  schedule/test_Query.cpp
  schedule/test_Database.cpp
  schedule/test_Mirror.cpp
  schedule/test_SpaceTime.cpp
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
  PUBLIC
    rmf_traffic
    ${PC_FCL_LIBRARIES}
)

target_include_directories(test_rmf_traffic
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
)
