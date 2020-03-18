cmake_minimum_required(VERSION 3.5)

project(rmf_traffic_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Box.msg"
  "msg/Circle.msg"
  "msg/ConvexShape.msg"
  "msg/ConvexShapeContext.msg"
  "msg/ItineraryClear.msg"
  "msg/ItineraryDelay.msg"
  "msg/ItineraryErase.msg"
  "msg/ItineraryExtend.msg"
  "msg/ItinerarySet.msg"
  "msg/MirrorWakeup.msg"
  "msg/ParticipantDescription.msg"
  "msg/Profile.msg"
  "msg/Region.msg"
  "msg/Route.msg"
  "msg/ScheduleChangeAdd.msg"
  "msg/ScheduleChangeCull.msg"
  "msg/ScheduleChangeDelay.msg"
  "msg/ScheduleConflictAck.msg"
  "msg/ScheduleConflictConclusion.msg"
  "msg/ScheduleConflictNotice.msg"
  "msg/ScheduleConflictProposal.msg"
  "msg/ScheduleConflictRejection.msg"
  "msg/ScheduleConflictRepeat.msg"
  "msg/ScheduleInconsistency.msg"
  "msg/ScheduleInconsistencyRange.msg"
  "msg/ScheduleParticipantPatch.msg"
  "msg/SchedulePatch.msg"
  "msg/ScheduleQuery.msg"
  "msg/ScheduleQueryParticipants.msg"
  "msg/ScheduleQuerySpacetime.msg"
  "msg/ScheduleRegister.msg"
  "msg/ScheduleWriterItem.msg"
  "msg/Shape.msg"
  "msg/ShapeContext.msg"
  "msg/Space.msg"
  "msg/Timespan.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryWaypoint.msg"

  # Test message definitions
  "msg/TestTaskRequest.msg"
)

set(srv_files
  "srv/MirrorUpdate.srv"
  "srv/RegisterQuery.srv"
  "srv/UnregisterQuery.srv"
  "srv/RegisterParticipant.srv"
  "srv/UnregisterParticipant.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
