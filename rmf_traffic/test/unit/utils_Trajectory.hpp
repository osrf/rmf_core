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

//==============================================================================
enum TestProfileType {
  UnitBox,
  UnitCircle
};

//==============================================================================
inline rmf_traffic::Trajectory::ProfilePtr make_test_profile(
    TestProfileType shape)
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

#endif // RMF_TRAFFIC__TEST__UNIT__UTILS_TRAJECTORY_HPP
