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

#ifndef SRC__RMF_TRAFFIC__INVALID_TRAJECTORY_ERROR_HPP
#define SRC__RMF_TRAFFIC__INVALID_TRAJECTORY_ERROR_HPP

#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {

//==============================================================================
class invalid_trajectory_error::Implementation
{
public:

  std::string what;

  static invalid_trajectory_error make_segment_num_error(
    std::size_t num_segments)
  {
    invalid_trajectory_error error;
    error._pimpl->what = std::string()
      + "[rmf_traffic::invalid_trajectory_error] Attempted to check a "
      + "conflict with a Trajectory that has [" + std::to_string(num_segments)
      + "] segments. This is not supported. Trajectories must have at least "
      + "2 segments to check them for conflicts.";
    return error;
  }

  static invalid_trajectory_error make_missing_shape_error(
    const Time time)
  {
    invalid_trajectory_error error;
    error._pimpl->what = std::string()
      + "[rmf_traffic::invalid_trajectory_error] Attempting to check a "
      + "conflict with a Trajectory that has no shape specified for the "
      + "profile of its waypoint at time ["
      + std::to_string(time.time_since_epoch().count())
      + "ns]. This is not supported.";

    return error;
  }
};

} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__INVALID_TRAJECTORY_ERROR_HPP
