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

#include "MotionInternal.hpp"

namespace rmf_traffic {

//==============================================================================
std::unique_ptr<Motion> Motion::compute_cubic_splines(
  const Trajectory::const_iterator& input_begin,
  const Trajectory::const_iterator& input_end)
{
  if (input_begin == input_end)
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_traffic::Motion::compute_cubic_spline] invalid waypoint range: "
      "begin == end");
    // *INDENT-ON*
  }

  using const_iterator = internal::WaypointList::const_iterator;
  const const_iterator begin = internal::get_raw_iterator(input_begin);
  const const_iterator end = internal::get_raw_iterator(input_end);

  if (++const_iterator(begin) == end)
  {
    const auto& point = *begin;
    return std::make_unique<SinglePointMotion>(
      point.data.time, point.data.position, point.data.velocity);
  }

  std::vector<Spline> splines;
  for (auto it = ++const_iterator(begin); it != end; ++it)
    splines.emplace_back(Spline(it));

  if (splines.size() == 1)
    return std::make_unique<SplineMotion>(std::move(splines[0]));

  return std::make_unique<PiecewiseSplineMotion>(std::move(splines));
}

//==============================================================================
std::unique_ptr<Motion> Motion::compute_cubic_splines(
  const Trajectory& trajectory)
{
  return compute_cubic_splines(trajectory.begin(), trajectory.end());
}

//==============================================================================
SinglePointMotion::SinglePointMotion(
  const Time t,
  Eigen::Vector3d p,
  Eigen::Vector3d v)
: _t(t),
  _p(std::move(p)),
  _v(std::move(v))
{
  // Do nothing
}

//==============================================================================
Time SinglePointMotion::start_time() const
{
  return _t;
}

//==============================================================================
Time SinglePointMotion::finish_time() const
{
  return _t;
}

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_position(Time /*t*/) const
{
  return _p;
}

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_velocity(Time /*t*/) const
{
  return _v;
}

//==============================================================================
Eigen::Vector3d SinglePointMotion::compute_acceleration(Time /*t*/) const
{
  return Eigen::Vector3d::Zero();
}

//==============================================================================
SplineMotion::SplineMotion(Spline spline)
: _spline(std::move(spline))
{
  // Do nothing
}

//==============================================================================
Time SplineMotion::start_time() const
{
  return _spline.start_time();
}

//==============================================================================
Time SplineMotion::finish_time() const
{
  return _spline.finish_time();
}

//==============================================================================
Eigen::Vector3d SplineMotion::compute_position(Time t) const
{
  return _spline.compute_position(t);
}

//==============================================================================
Eigen::Vector3d SplineMotion::compute_velocity(Time t) const
{
  return _spline.compute_velocity(t);
}

//==============================================================================
Eigen::Vector3d SplineMotion::compute_acceleration(Time t) const
{
  return _spline.compute_acceleration(t);
}

//==============================================================================
PiecewiseSplineMotion::PiecewiseSplineMotion(std::vector<Spline> splines)
{
  assert(!splines.empty());
  for (auto& spline : splines)
  {
    const Time t = spline.finish_time();
    _splines.insert(std::make_pair(t, std::move(spline)));
  }

  _start_time = _splines.begin()->second.start_time();
  _finish_time = _splines.rbegin()->second.finish_time();
}

//==============================================================================
Time PiecewiseSplineMotion::start_time() const
{
  return _start_time;
}

//==============================================================================
Time PiecewiseSplineMotion::finish_time() const
{
  return _finish_time;
}

//==============================================================================
Eigen::Vector3d PiecewiseSplineMotion::compute_position(Time t) const
{
  return _splines.lower_bound(t)->second.compute_position(t);
}

//==============================================================================
Eigen::Vector3d PiecewiseSplineMotion::compute_velocity(Time t) const
{
  return _splines.lower_bound(t)->second.compute_velocity(t);
}

//==============================================================================
Eigen::Vector3d PiecewiseSplineMotion::compute_acceleration(Time t) const
{
  return _splines.lower_bound(t)->second.compute_acceleration(t);
}

} // namespace rmf_traffic
