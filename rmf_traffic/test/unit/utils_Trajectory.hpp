/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
#define RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP

#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/Trajectory.hpp>

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_utils/catch.hpp>

//==============================================================================
// Literal conversion to degrees
constexpr long double operator "" _deg(long double value)
{
  // TODO(MXG): Consider moving this into rmf_utils::math
  return value*M_PI/180.0;
}

//==============================================================================
enum TestProfileType
{
  UnitBox,
  UnitCircle
};

//==============================================================================
inline rmf_traffic::Profile create_test_profile(TestProfileType shape_type)
{
  rmf_traffic::geometry::ConstFinalConvexShapePtr shape;

  if (UnitBox == shape_type)
    shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(1.0, 1.0);
  else if (UnitCircle == shape_type)
    shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);

  return rmf_traffic::Profile(shape, shape);
}

//==============================================================================
struct TrajectoryInsertInput
{
  rmf_traffic::Time time;
  TestProfileType profile_type;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

//==============================================================================
inline rmf_traffic::Trajectory create_test_trajectory()
{
  rmf_traffic::Trajectory trajectory;
  return trajectory;
}

//==============================================================================
inline rmf_traffic::Trajectory create_test_trajectory(
  std::vector<TrajectoryInsertInput> param_list)
{
  rmf_traffic::Trajectory trajectory;
  for (auto x : param_list)
  {
    trajectory.insert(x.time, x.pos, x.vel);
  }
  return trajectory;
}

#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
