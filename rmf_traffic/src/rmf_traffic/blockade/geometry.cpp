/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "geometry.hpp"

#include <cmath>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
double compute_smallest_distance_squared(
    const Eigen::Vector2d p,
    const Eigen::Vector2d p_other0,
    const Eigen::Vector2d n_other,
    const double c_other)
{
  const double u_star = n_other.dot(p)/c_other;
  const double u = std::max(0.0, std::min(1.0, u_star));
  const Eigen::Vector2d n = u*n_other + p_other0;
  return n.dot(n);
}

//==============================================================================
ConflictInfo detect_conflict(
    const Segment& s_a,
    const Segment& s_b,
    const double angle_threshold)
{
  const Eigen::Vector2d p_a0 = s_a.start;
  const Eigen::Vector2d p_a1 = s_a.finish;
  const Eigen::Vector2d p_b0 = s_b.start;
  const Eigen::Vector2d p_b1 = s_b.finish;

  const Eigen::Vector2d n_a = p_a1 - p_a0;
  const Eigen::Vector2d n_b = p_b1 - p_b0;

  const double c_ab = n_a.dot(n_b);
  const double angle = std::acos(c_ab/(n_a.norm()*n_b.norm()));
  if (angle <= angle_threshold)
    return ConflictInfo::no_conflict();

  const double c_aa = n_a.dot(n_a);
  assert(c_aa != 0.0);

  const double c_bb = n_b.dot(n_b);
  assert(c_bb != 0.0);

  const double conflict_r = s_a.radius+s_b.radius;
  const double conflict_r_squared = conflict_r*conflict_r;

  const double denom = c_aa*c_bb - c_ab*c_ab;
  if (std::abs(denom) < 1e-8)
  {
    // This is a singularity which indicates that the paths are nearly perfectly
    // parallel. There will be a conflict if they touching, or there will be
    // no conflict if they are not touching.
    const std::array<Eigen::Vector2d, 2> p_b = {s_b.start, s_b.finish};
    std::array<double, 2> distances_squared;
    for (std::size_t i=0; i < 2; ++i)
    {
      distances_squared[i] =
          compute_smallest_distance_squared(p_b[i], p_a0, n_a, c_aa);
    }

    const double r_squared = *std::min_element(
          distances_squared.begin(), distances_squared.end());

    if (r_squared > conflict_r_squared)
      return ConflictInfo::no_conflict();
  }
  else
  {
    // This means we don't have a singularity, so there is a direct intersection
    // of the lines created by the pairs of points. Now we need to determine
    // if the closest points along those lines have an overlap.
    const double c_a = n_a.dot(p_b0) - n_a.dot(p_a0);
    const double c_b = n_b.dot(p_b0) - n_b.dot(p_a0);

    const double u_a_star = (c_a*c_bb - c_b*c_ab)/denom;
    const double u_a = std::max(0.0, std::min(1.0, u_a_star));
    const Eigen::Vector2d p_a = u_a*(p_a1 - p_a0) + p_a0;

    const double u_b_star = (c_a*c_ab - c_b*c_aa)/denom;
    const double u_b = std::max(0.0, std::min(1.0, u_b_star));
    const Eigen::Vector2d p_b = u_b*(p_b1 - p_b0) + p_b0;

    const Eigen::Vector2d n = p_b - p_a;
    const double r_squared = n.dot(n);

    if (r_squared > conflict_r_squared)
      return ConflictInfo::no_conflict();

    if (angle <= 90.0*M_PI/180.0)
    {
      if (u_a_star <= 0.0 || u_b_star <= 0.0)
        return ConflictInfo::no_conflict();
    }
  }

  ConflictInfo info;
  info.has_conflict = true;

  const double r_squared_a0 =
      compute_smallest_distance_squared(p_a0, p_b0, n_b, c_bb);
  const double r_squared_a1 =
      compute_smallest_distance_squared(p_a1, p_b0, n_b, c_bb);
  const double r_squared_b0 =
      compute_smallest_distance_squared(p_b0, p_a0, n_a, c_aa);
  const double r_squared_b1 =
      compute_smallest_distance_squared(p_b1, p_a0, n_a, c_aa);

  info.include_cap_a[ConflictInfo::Start] = r_squared_a0 < conflict_r_squared;
  info.include_cap_a[ConflictInfo::Finish] = r_squared_a1 < conflict_r_squared;
  info.include_cap_b[ConflictInfo::Start] = r_squared_b0 < conflict_r_squared;
  info.include_cap_b[ConflictInfo::Finish] = r_squared_b1 < conflict_r_squared;

  return info;
}

} // namespace blockade
} // namespace rmf_traffic
