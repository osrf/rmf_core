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

#ifndef SRC__RMF_TRAFFIC__AGV__INTERPOLATEINTERNAL_HPP
#define SRC__RMF_TRAFFIC__AGV__INTERPOLATEINTERNAL_HPP

#include <rmf_traffic/agv/Interpolate.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Interpolate::Options::Implementation
{
public:

  Implementation(
      const bool always_stop,
      const double translation_thresh,
      const double rotation_thresh,
      const double corner_angle_thresh)
    : always_stop(always_stop),
      translation_thresh(translation_thresh),
      rotation_thresh(rotation_thresh),
      corner_angle_thresh(corner_angle_thresh)
  {
    // Do nothing
  }

  Implementation(const Options& options)
    : Implementation(*options._pimpl)
  {
    // Do nothing
  }

  bool always_stop;
  double translation_thresh;
  double rotation_thresh;
  double corner_angle_thresh;

  static const Implementation& get(const Options& options)
  {
    return *options._pimpl;
  }

};

namespace internal {

//==============================================================================
bool can_skip_interpolation(
    const Eigen::Vector3d& last_position,
    const Eigen::Vector3d& next_position,
    const Eigen::Vector3d& future_position,
    const Interpolate::Options::Implementation& options);

//==============================================================================
void interpolate_translation(
    Trajectory& trajectory,
    const double v_nom,
    const double a_nom,
    const Time start_time,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& finish,
    const Trajectory::ConstProfilePtr& profile,
    const double threshold);

//==============================================================================
void interpolate_rotation(
    Trajectory& trajectory,
    const double w_nom,
    const double alpha_nom,
    const Time start_time,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& finish,
    const Trajectory::ConstProfilePtr& profile,
    const double threshold);

} // namespace internal
} // namespace agv
} // namespace rmf_traffic


#endif // SRC__RMF_TRAFFIC__AGV__INTERPOLATEINTERNAL_HPP
