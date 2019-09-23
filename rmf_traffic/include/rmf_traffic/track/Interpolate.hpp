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

#ifndef RMF_TRAFFIC__TRACK__INTERPOLATE_HPP
#define RMF_TRAFFIC__TRACK__INTERPOLATE_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_traffic/track/VehicleTraits.hpp>

namespace rmf_traffic {
namespace track {

//==============================================================================
class Interpolate
{
public:

  static Trajectory positions(
      const VehicleTraits& traits,
      const std::vector<Eigen::Vector3d>& positions);

};

} // namespace track
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TRACK__INTERPOLATE_HPP
