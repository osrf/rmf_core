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

#ifndef SRC__RMF_TRAFFIC__SPLINE_HPP
#define SRC__RMF_TRAFFIC__SPLINE_HPP

#include "TrajectoryInternal.hpp"

#include <rmf_traffic/Trajectory.hpp>

#include <fcl/ccd/motion.h>

#include <array>

namespace rmf_traffic {

/// A utility class to convert Trajectories into piecewise-splines.
///
/// \warning For now this class is only meant for internal use; it is not part
/// of the public API. If we ever decide to migrate it to the public API, then
/// we should apply the PIMPL pattern to it.
class Spline
{
public:

  /// Create a spline that goes from the end of the preceding to the Waypoint of
  /// `it`.
  Spline(const Trajectory::const_iterator& it);

  /// Create a spline that goes from the end of the preceding to the Waypoint of
  /// `it`.
  Spline(const internal::WaypointList::const_iterator& it);

  /// Compute the knots for the motion of this spline from start_time to
  /// finish_time, scaled to a "time" range of [0, 1].
  std::array<Eigen::Vector3d, 4> compute_knots(
    const Time start_time, const Time finish_time) const;

  fcl::SplineMotion to_fcl(const Time start_time, const Time finish_time) const;

  Time start_time() const;
  Time finish_time() const;

  struct Parameters
  {
    std::array<Eigen::Vector4d, 3> coeffs;
    double delta_t;
    std::array<Time, 2> time_range;
  };

  /// Compute the position of the spline at this moment in time
  Eigen::Vector3d compute_position(const Time at_time) const;

  /// Compute the velocity of the spline at this moment in time
  Eigen::Vector3d compute_velocity(const Time at_time) const;

  /// Compute the velocity of the spline at this moment in time
  Eigen::Vector3d compute_acceleration(const Time at_time) const;

  /// Get a const reference to the parameters of this spline
  const Parameters& get_params() const;

private:

  Parameters params;

};

class OutOfSplineRange : public std::runtime_error
{
public:

  OutOfSplineRange(Time t, std::array<Time,2> range);

};

} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SPLINE_HPP
