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

#include "Spline.hpp"

namespace rmf_traffic {

namespace {

//==============================================================================
Eigen::Matrix4d make_M_inv()
{
  Eigen::Matrix4d M;
  M.block<1, 4>(0, 0) <<  1.0/6.0, 2.0/3.0, 1.0/6.0, 0.0;
  M.block<1, 4>(1, 0) << -1.0/2.0, 0.0, 1.0/2.0, 0.0;
  M.block<1, 4>(2, 0) <<  1.0/2.0, -1.0, 1.0/2.0, 0.0;
  M.block<1, 4>(3, 0) << -1.0/6.0, 1.0/2.0, -1.0/2.0, 1.0/6.0;

  return M.inverse();
}

//==============================================================================
double compute_delta_t(const Time& finish_time, const Time& start_time)
{
  using Sec64 = std::chrono::duration<double>;
  return std::chrono::duration_cast<Sec64>(finish_time - start_time).count();
}

//==============================================================================
std::array<Eigen::Vector4d, 3> compute_coefficients(
  const Eigen::Vector3d& x0,
  const Eigen::Vector3d& x1,
  const Eigen::Vector3d& v0,
  const Eigen::Vector3d& v1)
{
  std::array<Eigen::Vector4d, 3> coeffs;
  for (int i = 0; i < 3; ++i)
  {
    // *INDENT-OFF*
    std::size_t si = static_cast<std::size_t>(i);
    coeffs[si][0] =                                x0[i]; // = d
    coeffs[si][1] =            v0[i];                     // = c
    coeffs[si][2] = -v1[i] - 2*v0[i] + 3*x1[i] - 3*x0[i]; // = b
    coeffs[si][3] =  v1[i] +   v0[i] - 2*x1[i] + 2*x0[i]; // = a
    // *INDENT-ON*
  }

  return coeffs;
}

//==============================================================================
Spline::Parameters compute_parameters(
  const Trajectory::const_iterator& finish_it)
{
  const Trajectory::const_iterator start_it =
    --Trajectory::const_iterator(finish_it);

  const Trajectory::Waypoint& start = *start_it;
  const Trajectory::Waypoint& finish = *finish_it;

  const Time start_time = start.time();
  const Time finish_time = finish.time();

  const double delta_t = compute_delta_t(finish_time, start_time);

  const Eigen::Vector3d x0 = start.position();
  const Eigen::Vector3d x1 = finish.position();
  const Eigen::Vector3d v0 = delta_t * start.velocity();
  const Eigen::Vector3d v1 = delta_t * finish.velocity();

  return {
    compute_coefficients(x0, x1, v0, v1),
    delta_t,
    {start_time, finish_time}
  };
}

//==============================================================================
Spline::Parameters compute_parameters(
  const internal::WaypointList::const_iterator& finish_it)
{
  const internal::WaypointList::const_iterator start_it =
    --internal::WaypointList::const_iterator(finish_it);

  const internal::WaypointElement::Data& start = start_it->data;
  const internal::WaypointElement::Data& finish = finish_it->data;

  const Time start_time = start.time;
  const Time finish_time = finish.time;

  const double delta_t = compute_delta_t(finish_time, start_time);

  const Eigen::Vector3d x0 = start.position;
  const Eigen::Vector3d x1 = finish.position;
  const Eigen::Vector3d v0 = delta_t * start.velocity;
  const Eigen::Vector3d v1 = delta_t * finish.velocity;

  return {
    compute_coefficients(x0, x1, v0, v1),
    delta_t,
    {start_time, finish_time}
  };
}

//==============================================================================
double compute_scaled_time(const Time& time, const Spline::Parameters& params)
{
  using Sec64 = std::chrono::duration<double>;
  const double relative_time =
    std::chrono::duration_cast<Sec64>(time - params.time_range[0]).count();

  const double scaled_time = relative_time / params.delta_t;
  assert(0.0 - 1.0e-8 <= scaled_time);
  assert(scaled_time <= 1.0 + 1.0e-8);

  return scaled_time;
}

//==============================================================================
Eigen::Vector3d compute_position(
  const Spline::Parameters& params,
  const double time)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i)
  {
    const Eigen::Vector4d coeffs = params.coeffs[i];
    for (int j = 0; j < 4; ++j)
      result[i] += coeffs[j] * pow(time, j);
  }

  return result;
}

//==============================================================================
Eigen::Vector3d compute_velocity(
  const Spline::Parameters& params,
  const double time)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i)
  {
    const Eigen::Vector4d coeffs = params.coeffs[i];
    // Note: This is computing the derivative of the polynomial w.r.t. time
    for (int j = 1; j < 4; ++j)
      result[i] += j * coeffs[j] * pow(time, j-1);
  }

  return result;
}

//==============================================================================
Eigen::Vector3d compute_acceleration(
  const Spline::Parameters& params,
  const double time)
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i)
  {
    const Eigen::Vector4d coeffs = params.coeffs[i];
    // Note: This is computing the second derivative w.r.t. time
    for (int j = 2; j < 4; ++j)
      result[i] += j * (j-1) * coeffs[j] * pow(time, j-2);
  }

  return result;
}

} // anonymous namespace

//==============================================================================
const Eigen::Matrix4d M_inv = make_M_inv();

//==============================================================================
Spline::Spline(const Trajectory::const_iterator& it)
: params(compute_parameters(it))
{
  // Do nothing
}

//==============================================================================
Spline::Spline(const internal::WaypointList::const_iterator& it)
: params(compute_parameters(it))
{
  // Do nothing
}

//==============================================================================
std::array<Eigen::Vector3d, 4> Spline::compute_knots(
  const Time start_time, const Time finish_time) const
{
  assert(params.time_range[0] <= start_time);
  assert(finish_time <= params.time_range[1]);

  const double scaled_delta_t =
    compute_delta_t(finish_time, start_time) / params.delta_t;

  const double scaled_start_time = compute_scaled_time(start_time, params);
  const double scaled_finish_time = compute_scaled_time(finish_time, params);

  const Eigen::Vector3d x0 =
    rmf_traffic::compute_position(params, scaled_start_time);
  const Eigen::Vector3d x1 =
    rmf_traffic::compute_position(params, scaled_finish_time);
  const Eigen::Vector3d v0 =
    scaled_delta_t * rmf_traffic::compute_velocity(params, scaled_start_time);
  const Eigen::Vector3d v1 =
    scaled_delta_t * rmf_traffic::compute_velocity(params, scaled_finish_time);

  const std::array<Eigen::Vector4d, 3> subspline_coeffs =
    compute_coefficients(x0, x1, v0, v1);

  std::array<Eigen::Vector3d, 4> result;
  for (std::size_t i = 0; i < 3; ++i)
  {
    const Eigen::Vector4d p = M_inv * subspline_coeffs[i];
    for (int j = 0; j < 4; ++j)
      result[j][i] = p[j];
  }

  return result;
}

//==============================================================================
fcl::SplineMotion Spline::to_fcl(
  const Time start_time, const Time finish_time) const
{
  std::array<Eigen::Vector3d, 4> knots = compute_knots(start_time, finish_time);

  std::array<fcl::Vec3f, 4> Td;
  std::array<fcl::Vec3f, 4> Rd;

  for (std::size_t i = 0; i < 4; ++i)
  {
    const Eigen::Vector3d p = knots[i];
    Td[i] = fcl::Vec3f(p[0], p[1], 0.0);
    Rd[i] = fcl::Vec3f(0.0, 0.0, p[2]);
  }

  return fcl::SplineMotion(
    Td[0], Td[1], Td[2], Td[3],
    Rd[0], Rd[1], Rd[2], Rd[3]);
}

//==============================================================================
Time Spline::start_time() const
{
  return params.time_range[0];
}

//==============================================================================
Time Spline::finish_time() const
{
  return params.time_range[1];
}

//==============================================================================
Eigen::Vector3d Spline::compute_position(const Time at_time) const
{
  return rmf_traffic::compute_position(
    params, compute_scaled_time(at_time, params));
}

//==============================================================================
Eigen::Vector3d Spline::compute_velocity(const Time at_time) const
{
  const double delta_t_inv = 1.0/params.delta_t;
  return delta_t_inv * rmf_traffic::compute_velocity(
    params, compute_scaled_time(at_time, params));
}

//==============================================================================
Eigen::Vector3d Spline::compute_acceleration(const Time at_time) const
{
  const double delta_t_inv = 1.0/params.delta_t;
  return pow(delta_t_inv, 2) * rmf_traffic::compute_acceleration(
    params, compute_scaled_time(at_time, params));
}

//==============================================================================
const Spline::Parameters& Spline::get_params() const
{
  return params;
}

//==============================================================================
std::array<Eigen::Vector4d, 3> normalize_coefficients(
    const Time t0,
    const Time t1,
    const double delta_t,
    const Spline& spline)
{
  const Eigen::Vector3d x0 = spline.compute_position(t0);
  const Eigen::Vector3d x1 = spline.compute_position(t1);
  const Eigen::Vector3d v0 = delta_t * spline.compute_velocity(t0);
  const Eigen::Vector3d v1 = delta_t * spline.compute_velocity(t1);

  return compute_coefficients(x0, x1, v0, v1);
}

//==============================================================================
DistanceDifferential::DistanceDifferential(
    const Spline& spline_a,
    const Spline& spline_b)
{
  const Time t0 = std::max(spline_a.start_time(), spline_b.start_time());
  const Time t1 = std::min(spline_a.finish_time(), spline_b.finish_time());
  const double delta_t = compute_delta_t(t1, t0);

  const auto coeffs_a = normalize_coefficients(t0, t1, delta_t, spline_a);
  const auto coeffs_b = normalize_coefficients(t0, t1, delta_t, spline_b);

  _params.coeffs[0] = coeffs_a[0] - coeffs_b[0];
  _params.coeffs[1] = coeffs_a[1] - coeffs_b[1];
  // we ignore rotation when calculating distance
  _params.coeffs[2] = Eigen::Vector4d::Zero();

  _params.time_range[0] = t0;
  _params.time_range[1] = t1;
  _params.delta_t = delta_t;
}

namespace {
//==============================================================================
bool check_negative_derivative(
    const double scaled_time, const Spline::Parameters& params)
{
  const Eigen::Vector2d dp =
      compute_position(params, scaled_time).block<2,1>(0,0);

  const Eigen::Vector2d dv =
      compute_velocity(params, scaled_time).block<2,1>(0,0);

  return (dp.dot(dv) < 0.0);
}
} // anonymous namespace

//==============================================================================
bool DistanceDifferential::initially_negative_derivative() const
{
  return check_negative_derivative(0.0, _params);
}

//==============================================================================
std::vector<Time> DistanceDifferential::approach_times() const
{

}

////==============================================================================
//bool DistanceDifferential::negative_derivative_at(const Time time) const
//{
//  return check_negative_derivative(compute_scaled_time(time, _params), _params);
//}

////==============================================================================
//bool DistanceDifferential::negative_second_derivative_at(const Time time) const
//{
//  const double scaled_time = compute_scaled_time(time, _params);

//  const Eigen::Vector2d dp =
//      compute_position(_params, scaled_time).block<2,1>(0,0);

//  const Eigen::Vector2d dv =
//      compute_position(_params, scaled_time).block<2,1>(0,0);

//  const Eigen::Vector2d da =
//      compute_acceleration(_params, scaled_time).block<2,1>(0,0);

//  return (dv.dot(dv) + dp.dot(da) < 0.0);
//}

//==============================================================================
Time DistanceDifferential::start_time() const
{
  return _params.time_range[0];
}

//==============================================================================
Time DistanceDifferential::finish_time() const
{
  return _params.time_range[1];
}

} // namespace rmf_traffic
