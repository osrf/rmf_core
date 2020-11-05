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

#include <unordered_set>

namespace rmf_traffic {

#ifdef RMF_TRAFFIC__USING_FCL_0_6
  using FclVec3 = fcl::Vector3d;
#else
  using FclVec3 = fcl::Vec3f;
#endif

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
const Eigen::Matrix4d M_inv = make_M_inv();

FclSplineMotion to_fcl(
  const Eigen::Vector3d& x0,
  const Eigen::Vector3d& x1,
  const Eigen::Vector3d& v0,
  const Eigen::Vector3d& v1)
{
  const std::array<Eigen::Vector4d, 3> subspline_coeffs =
    compute_coefficients(x0, x1, v0, v1);

  std::array<Eigen::Vector3d, 4> knots;
  for (std::size_t i = 0; i < 3; ++i)
  {
    const Eigen::Vector4d p = M_inv * subspline_coeffs[i];
    for (int j = 0; j < 4; ++j)
      knots[j][i] = p[j];
  }

  std::array<FclVec3, 4> Td;
  std::array<FclVec3, 4> Rd;

  for (std::size_t i = 0; i < 4; ++i)
  {
    const Eigen::Vector3d p = knots[i];
    Td[i] = FclVec3(p[0], p[1], 0.0);
    Rd[i] = FclVec3(0.0, 0.0, p[2]);
  }

  return FclSplineMotion(
    Td[0], Td[1], Td[2], Td[3],
    Rd[0], Rd[1], Rd[2], Rd[3]);
}

namespace {

const double time_tolerance = 1e-4;

//==============================================================================
double compute_delta_t(const Time& finish_time, const Time& start_time)
{
  using Sec64 = std::chrono::duration<double>;
  return std::chrono::duration_cast<Sec64>(finish_time - start_time).count();
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
  printf("delta_t: %f\n", delta_t);
  printf("startvel: %f %f %f\n", start.velocity()[0], start.velocity()[1], start.velocity()[2]);
  printf("finvel: %f %f %f\n", finish.velocity()[0], finish.velocity()[1], finish.velocity()[2]);

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
  using SecF64 = std::chrono::duration<double>;
  const double relative_time =
    std::chrono::duration_cast<SecF64>(time - params.time_range[0]).count();

  const double scaled_time = relative_time / params.delta_t;
  assert(0.0 - 1.0e-8 <= scaled_time);
  assert(scaled_time <= 1.0 + 1.0e-8);

  return scaled_time;
}

//==============================================================================
Time compute_real_time(
    const std::array<Time, 2>& time_range,
    const double scaled_time)
{
  using SecF64 = std::chrono::duration<double>;
  const auto duration =
      scaled_time
      * std::chrono::duration_cast<SecF64>(
        time_range[1] - time_range[0]);

  using Nano = std::chrono::nanoseconds;
  return std::chrono::duration_cast<Nano>(duration) + time_range[0];
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

  //printf("scaled_time : %f %f scaled_delta_t:%f\n", scaled_start_time, scaled_finish_time, scaled_delta_t);
  printf("x0: %f %f %f x1: %f %f %f\n", x0[0], x0[1], x0[2], x1[0], x1[1], x1[2]);
  printf("v0: %f %f %f v1: %f %f %f\n", v0[0], v0[1], v0[2], v1[0], v1[1], v1[2]);
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
FclSplineMotion Spline::to_fcl(
  const Time start_time, const Time finish_time) const
{
  std::array<Eigen::Vector3d, 4> knots = compute_knots(start_time, finish_time);
  return to_fcl(knots);
}

//==============================================================================
FclSplineMotion Spline::to_fcl(
  const std::array<Eigen::Vector3d, 4>& knots) const
{
  std::array<FclVec3, 4> Td;
  std::array<FclVec3, 4> Rd;

  for (std::size_t i = 0; i < 4; ++i)
  {
    const Eigen::Vector3d p = knots[i];
    Td[i] = FclVec3(p[0], p[1], 0.0);
    Rd[i] = FclVec3(0.0, 0.0, p[2]);
  }

  return FclSplineMotion(
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
double compute_derivative(
    const double scaled_time,
    const Spline::Parameters& params)
{
  const Eigen::Vector2d dp =
      compute_position(params, scaled_time).block<2,1>(0,0);

  const Eigen::Vector2d dv =
      compute_velocity(params, scaled_time).block<2,1>(0,0);

  return dp.dot(dv);
}

//==============================================================================
bool is_negative_derivative(
    const double scaled_time, const Spline::Parameters& params)
{
  return (compute_derivative(scaled_time, params) < 0.0);
}

//==============================================================================
bool is_second_derivative_of_distance_negative(
    const Spline::Parameters& params,
    const double t)
{
  const Eigen::Vector2d dp =
      compute_position(params, t).block<2,1>(0,0);

  const Eigen::Vector2d dv =
      compute_velocity(params, t).block<2,1>(0,0);

  const Eigen::Vector2d da =
      compute_acceleration(params, t).block<2,1>(0,0);

  return (dv.dot(dv) + dp.dot(da) < 0.0);
}

//==============================================================================
/// Get the quadratic roots of the coefficients, but only if they fall in the
/// domain t = [0, 1]
// TODO(MXG): This will always return 2, 1, or 0 results, so a bounded vector
// would be preferable for a return value.
std::vector<double> compute_roots_in_unit_domain(const Eigen::Vector3d coeffs)
{
  const double tol = 1e-5;

  const double a = coeffs[2];
  const double b = coeffs[1];
  const double c = coeffs[0];

  if (std::abs(a) < tol)
  {
    if (std::abs(b) < tol)
    {
      return {};
    }

    const double t = -c/b;
    if (0.0 <= t && t <= 1.0)
    {
      return {t};
    }

    return {};
  }

  const double determinate = (b*b - 4*a*c);
  if (determinate < 0.0)
  {
    return {};
  }

  std::vector<double> output;
  const double t_m = (-b - std::sqrt(determinate))/(2*a);
  if (0.0 <= t_m && t_m <= 1.0)
    output.push_back(t_m);

  const double t_p = (-b + std::sqrt(determinate))/(2*a);
  if (0.0 <= t_p && t_p <= 1.0 && std::abs(t_p - t_m) > time_tolerance)
    output.push_back(t_p);

  return output;
}

//==============================================================================
Eigen::Vector3d compute_deriv_coeffs(const Eigen::Vector4d& coeffs)
{
  return {coeffs[1], 2.0*coeffs[2], 3.0*coeffs[3]};
}

} // anonymous namespace

//==============================================================================
bool DistanceDifferential::initially_approaching() const
{
  const double derivative = compute_derivative(0.0, _params);
  if (derivative < 0.0)
    return true;

  if (derivative < 1e-8
      && is_second_derivative_of_distance_negative(_params, 0.0))
    return true;

  return false;
}

//==============================================================================
bool is_in_eighth(
    const double theta,
    const double eighth_0,
    const double eighth_1)
{
  // Half of the angular range of one of the 1/8 divisions will be 2*pi/16
  const double half_range = M_PI/8.0;
  return (std::abs(theta - eighth_0) <= half_range)
      || (std::abs(theta - eighth_1) <= half_range);
}

//==============================================================================
bool contains(const std::vector<double>& times, const double t)
{
  for (const auto check : times)
  {
    if (std::abs(t-check) < time_tolerance)
      return true;
  }

  return false;
}

//==============================================================================
void insert_if_missing(std::vector<double>& times, const double t)
{
  if (!contains(times, t))
    times.push_back(t);
}

//==============================================================================
std::vector<Time> DistanceDifferential::approach_times() const
{
  // The idea behind finding the "approach times" is to find local maximum
  // points of the distance function. A local maximum on the distance function
  // implies that the vehicles are changing from moving away from each other to
  // moving towards each other.
  //
  // A local maximum would rigorously be defined by a point where the derivative
  // of the distance function is zero while the second derivative of the
  // distance function is negative. However, finding the roots of the derivative
  // in the general case would require solving a quintic polynomial. Solving
  // such a polynomial is possible, but would entail the following problems:
  //
  // 1. Only iterative methods are available, so the time required would be
  //    sensitive to a choice of precision.
  //
  // 2. The time required to iteratively solve for the roots may become
  //    prohibitive, considering how frequently conflicts need to be evaluated.
  //
  // 3. Iterative methods are sensitive to the choice of initial guesses, so we
  //    would need to carefully consider how to choose those points.
  //
  // 4. The complexity of implementing the iterative root finding algorithm
  //    implies some risk. It may be difficult to ensure a correct
  //    implementation without a great deal of careful testing.
  //
  // Instead we will linearize the problem. We identify 5 different categories
  // of derivative values which could imply that the distance is shrinking,
  // depending on the position coordinate at the time.
  //
  // vx: 0, vy: 0
  // vx: 0, vy: non-zero
  // vx: non-zero, vy: 0
  // vx + vy = 0
  // vx - vy = 0
  //
  // Depending on the position coordinates at the time of one of these
  // velocities, the vehicles may be approaching each other (or about to begin
  // approaching each other).

  const Eigen::Vector3d vx_coeffs = compute_deriv_coeffs(_params.coeffs[0]);
  const Eigen::Vector3d vy_coeffs = compute_deriv_coeffs(_params.coeffs[1]);

  const auto t_vx_zero = compute_roots_in_unit_domain(vx_coeffs);
  auto t_vy_zero = compute_roots_in_unit_domain(vy_coeffs);

  std::vector<double> t_full_zero;
  const double zero_tolerance = 1e-3;

  std::vector<Time> output;

  for (const double t : t_vx_zero)
  {
    const Eigen::Vector2d dv = compute_velocity(_params, t).block<2,1>(0,0);
    if (std::abs(dv.y()) < zero_tolerance)
    {
      insert_if_missing(t_full_zero, t);
      continue;
    }

    // This is (vx: 0, vy: anything)
    const Eigen::Vector2d dp = compute_position(_params, t).block<2,1>(0,0);
    const double theta = std::atan2(dp.y(), dp.x());
    if (is_in_eighth(theta, M_PI/2.0, -M_PI/2.0)
        || is_negative_derivative(t, _params))
    {
      output.push_back(compute_real_time(_params.time_range, t));
    }
  }

  for (const double t : t_vy_zero)
  {
    const Eigen::Vector2d dv = compute_velocity(_params, t).block<2,1>(0,0);
    if (std::abs(dv.x()) < zero_tolerance)
    {
      insert_if_missing(t_full_zero, t);
      continue;
    }

    const Eigen::Vector2d dp = compute_position(_params, t).block<2,1>(0,0);

    double theta = std::atan2(dp.y(), dp.x());
    if (theta < -M_PI/2.0)
      theta += 2.0*M_PI;

    if (is_in_eighth(theta, 0.0, M_PI)
        || is_negative_derivative(t, _params))
    {
      output.push_back(compute_real_time(_params.time_range, t));
    }
  }

  for (const double t : t_full_zero)
  {
    // This is a true critical point, so we should check the second
    // derivative. This is (vx: 0, vy: 0)
    if (is_second_derivative_of_distance_negative(_params, t))
    {
      output.push_back(compute_real_time(_params.time_range, t));
    }
  }

  const auto t_vx_p_vy = compute_roots_in_unit_domain(vx_coeffs + vy_coeffs);
  for (const double t : t_vx_p_vy)
  {
    if (contains(t_full_zero, t))
      continue;

    // This is (vx + vy = 0)
    const Eigen::Vector2d dp = compute_position(_params, t).block<2,1>(0,0);
    const double theta = std::atan2(dp.y(), dp.x());
    if (is_in_eighth(theta, M_PI/4.0, -3.0*M_PI/4.0)
        || is_negative_derivative(t, _params))
    {
      output.push_back(compute_real_time(_params.time_range, t));
    }
  }

  const auto t_vx_m_vy = compute_roots_in_unit_domain(vx_coeffs - vy_coeffs);
  for (const double t : t_vx_m_vy)
  {
    if (contains(t_full_zero, t))
      continue;

    // This is (vx - vy = 0)
    const Eigen::Vector2d dp = compute_position(_params, t).block<2,1>(0,0);
    const double theta = std::atan2(dp.y(), dp.x());
    if (is_in_eighth(theta, 3.0*M_PI/4.0, -M_PI/4.0)
        || is_negative_derivative(t, _params))
    {
      output.push_back(compute_real_time(_params.time_range, t));
    }
  }

  std::sort(output.begin(), output.end());
  return output;
}

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
