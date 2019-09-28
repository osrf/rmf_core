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

using AgencyType = rmf_traffic::Trajectory::Profile::Agency;

//==============================================================================
enum TestProfileType
{
  UnitBox,
  UnitCircle
};

inline rmf_traffic::Trajectory::ProfilePtr create_test_profile(TestProfileType shape,
                                                  AgencyType agency = AgencyType::Strict,
                                                  std::string queue_number = "0")
{
  if (UnitBox == shape && AgencyType::Strict == agency)
  {
    return rmf_traffic::Trajectory::Profile::make_strict(
        std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0));
  }
  else if (UnitCircle == shape && AgencyType::Strict == agency)
  {
    return rmf_traffic::Trajectory::Profile::make_strict(
        std::make_shared<rmf_traffic::geometry::Circle>(1.0));
  }
  else if (UnitBox == shape && AgencyType::Queued == agency)
  {
    return rmf_traffic::Trajectory::Profile::make_queued(
        std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0), queue_number);
  }
  else if (UnitCircle == shape && AgencyType::Queued == agency)
  {
    return rmf_traffic::Trajectory::Profile::make_queued(
        std::make_shared<rmf_traffic::geometry::Circle>(1.0), queue_number);
  }
  else
  {
    return nullptr;
  }
}

//==============================================================================
struct TrajectoryInsertInput
{
  rmf_traffic::Time time;
  TestProfileType profile_type;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

inline rmf_traffic::Trajectory create_test_trajectory()
{
  rmf_traffic::Trajectory trajectory("test_map");
  return trajectory;
}

inline rmf_traffic::Trajectory create_test_trajectory(std::vector<TrajectoryInsertInput> param_list)
{
  rmf_traffic::Trajectory trajectory("test_map");
  for (auto x : param_list)
  {
    trajectory.insert(x.time, create_test_profile(x.profile_type), x.pos, x.vel);
  }
  return trajectory;
}

// For backward compatibility in other tests
//==============================================================================
inline rmf_traffic::Trajectory::ProfilePtr make_test_profile(TestProfileType shape)
{
  if (UnitBox == shape)
  {
    return rmf_traffic::Trajectory::Profile::make_strict(
        std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0));
  }
  else if (UnitCircle == shape)
  {
    return rmf_traffic::Trajectory::Profile::make_strict(
        std::make_shared<rmf_traffic::geometry::Circle>(1.0));
  }
  else
  {
    return nullptr;
  }
}

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

#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
