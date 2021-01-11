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

#ifdef RMF_TRAFFIC__USING_FCL_0_6
#include <fcl/math/motion/spline_motion.h>
#else
#include <fcl/ccd/motion.h>
#endif

#include <array>

namespace rmf_traffic {

#ifdef RMF_TRAFFIC__USING_FCL_0_6
  using FclSplineMotion = fcl::SplineMotion<double>;
#else
  using FclSplineMotion = fcl::SplineMotion;
#endif

//==============================================================================
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

  FclSplineMotion to_fcl(
    const Time start_time, const Time finish_time) const;

  FclSplineMotion to_fcl(const std::array<Eigen::Vector3d, 4>& knots) const;

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

FclSplineMotion to_fcl(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& x1,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1);

//==============================================================================
/// This class helps compute the differentials of the distance between two
/// splines.
class DistanceDifferential
{
public:

  DistanceDifferential(
      const Spline& spline_a,
      const Spline& spline_b);

  bool initially_approaching() const;

  /// Calculate the times within the relevant window when an "approach" is
  /// occuring. This means that the vehicles are getting closer together than
  /// they should.
  std::vector<Time> approach_times() const;

  Time start_time() const;
  Time finish_time() const;

private:
  Spline::Parameters _params;

};

//==============================================================================
struct ModelSpaceShape
{
  ModelSpaceShape(const fcl::Transform3d& tx, double r)
    :_transform(tx), _radius(r)
  { }
  fcl::Transform3d _transform;
  double _radius;
};

extern bool collide_seperable_circles(
  FclSplineMotion& motion_a,
  FclSplineMotion& motion_b,
  const std::vector<ModelSpaceShape>& a_shapes,
  const std::vector<ModelSpaceShape>& b_shapes,
  double& impact_time, uint& dist_checks,
  uint safety_maximum_checks, double tolerance);

} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SPLINE_HPP
