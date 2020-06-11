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

#ifndef RMF_TRAFFIC__MOTION_HPP
#define RMF_TRAFFIC__MOTION_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <memory>

namespace rmf_traffic {

//==============================================================================
/// Pure abstract interface for describing a continuous motion
class Motion
{
public:

  /// Get the lower bound on the time range where this motion is valid
  virtual Time start_time() const = 0;

  /// Get the upper bound on the time range where this motion is valid
  virtual Time finish_time() const = 0;

  /// Get the position of this motion at a point in time.
  ///
  /// \param[in] t
  ///   The time of interest. This time must be in the range
  ///   [start_time(), finish_time()], or else the output is undefined
  ///   and may result in an exception.
  virtual Eigen::Vector3d compute_position(Time t) const = 0;

  /// Get the velocity of this motion at a point in time.
  ///
  /// \param[in] t
  ///   The time of interest. This time must be in the range
  ///   [start_time(), finish_time()], or else the output is undefined
  ///   and may result in an exception.
  virtual Eigen::Vector3d compute_velocity(Time t) const = 0;

  /// Get the acceleration of this motion at a point in time.
  ///
  /// \param[in] t
  ///   The time of interest. This time must be in the range
  ///   [start_time(), finish_time()], or else the output is undefined
  ///   and may result in an exception.
  virtual Eigen::Vector3d compute_acceleration(Time t) const = 0;

  // Default destructor
  virtual ~Motion() = default;

  /// Compute a piecewise cubic spline motion object for a Trajectory from the
  /// begin iterator up to (but not including) the end iterator.
  ///
  /// \param[in] begin
  ///   The iterator of the first waypoint to include in the motion. It is
  ///   undefined behavior to pass in Trajectory::end() for this argument.
  ///
  /// \param[in] end
  ///   The iterator of the first waypoint to exclude from the motion. To
  ///   include all the way to the end of the trajectory, pass in
  ///   Trajectory::end(). An exception will be thrown if begin == end.
  static std::unique_ptr<Motion> compute_cubic_splines(
    const Trajectory::const_iterator& begin,
    const Trajectory::const_iterator& end);

  /// Compute a piecewise cubic spline motion object for an entire Trajectory.
  static std::unique_ptr<Motion> compute_cubic_splines(
    const Trajectory& trajectory);
};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__MOTION_HPP
