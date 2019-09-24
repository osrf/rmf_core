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

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_utils/catch.hpp>

using namespace rmf_traffic;
using namespace std;

//==============================================================================
enum TestProfileType
{
  UnitBox,
  UnitCircle
};

enum TestAgencyType
{
  Strict,
  Autonomous
};

inline Trajectory::ProfilePtr make_test_profile(TestProfileType shape)
{
  if (UnitBox == shape)
  {
    return Trajectory::Profile::make_strict(
        std::make_shared<geometry::Box>(1.0, 1.0));
  }
  else if (UnitCircle == shape)
  {
    return Trajectory::Profile::make_strict(
        std::make_shared<geometry::Circle>(1.0));
  }
  else
  {
    return nullptr;
  }
}

inline Trajectory::ProfilePtr make_test_profile(TestProfileType shape, TestAgencyType agency)
{
  if (shape == UnitBox && agency == Strict)
  {
    return Trajectory::Profile::make_strict(
        std::make_shared<geometry::Box>(1.0, 1.0));
  }
  else if (shape == UnitCircle && agency == Strict)
  {
    return Trajectory::Profile::make_strict(
        std::make_shared<geometry::Circle>(1.0));
  }
  else
  {
    return nullptr;
  }
}

//==============================================================================
struct TrajectoryInsertInput
{
  Time time;
  TestProfileType profile_type;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

inline Trajectory create_test_trajectory()
{
  Trajectory trajectory("test_map");
  return trajectory;
}

inline Trajectory create_test_trajectory(vector<TrajectoryInsertInput> param_list)
{
  Trajectory trajectory("test_map");
  for (auto x : param_list)
  {
    trajectory.insert(x.time, make_test_profile(x.profile_type), x.pos, x.vel);
  }
  return trajectory;
}

//==============================================================================
inline rmf_traffic::Trajectory make_test_trajectory(rmf_traffic::Time t, int length, int dur)
{
  using namespace std::chrono_literals;
  rmf_traffic::Trajectory trajectory("test_map");
  for (auto i = 0; i < length; ++i)
  {
    const auto finish_time = t + std::chrono::seconds(i * dur);
    const auto profile = make_test_profile(UnitBox);
    const Eigen::Vector3d final_pos = Eigen::Vector3d(1, 1, 1);
    const Eigen::Vector3d final_vel = Eigen::Vector3d(1, 1, 1);
    auto result = trajectory.insert(finish_time, profile, final_pos, final_vel);
  }
  return trajectory;
}

//==============================================================================

// inline void trajectories_are_identical(rmf_traffic::Trajectory &t1,
//                                        rmf_traffic::Trajectory &t2)
// {
//   CHECK(t1.size() == t2.size());
//   CHECK(t1.duration() == t2.duration());

//   rmf_traffic::Trajectory::iterator ot = t1.begin();
//   rmf_traffic::Trajectory::iterator ct = t2.begin();

//   for (; ot != t1.end() && ct != t2.end(); ++ot, ++ct)
//   {
//     CHECK(ot->get_profile() != ct->get_profile());
//     CHECK(ot->get_finish_position() != ct->get_finish_position());
//     CHECK(ot->get_finish_velocity() != ct->get_finish_velocity());
//     CHECK(ot->get_finish_time() != ct->get_finish_time());
//   }
//   CHECK(ot == t1.end());
//   CHECK(ct == t2.end());
// }

#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
