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

#ifndef SRC__RMF_TRAFFIC__BLOCKADE__GEOMETRY_HPP
#define SRC__RMF_TRAFFIC__BLOCKADE__GEOMETRY_HPP

#include <Eigen/Geometry>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
struct Segment
{
  Eigen::Vector2d start;
  Eigen::Vector2d finish;
  double radius;
};

//==============================================================================
struct ConflictInfo
{
  bool has_conflict = false;

  enum Cap
  {
    Start = 0,
    Finish
  };

  // The following fields will be uninitialized when has_conflict is false
  std::array<bool, 2> include_cap_a;
  std::array<bool, 2> include_cap_b;

  static ConflictInfo no_conflict()
  {
    return ConflictInfo();
  }
};

//==============================================================================
ConflictInfo detect_conflict(
    const Segment& s_a,
    const Segment& s_b,
    double angle_threshold);

} // namespace blockade
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__BLOCKADE__GEOMETRY_HPP
