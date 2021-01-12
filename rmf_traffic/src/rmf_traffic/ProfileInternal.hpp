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

#ifndef SRC__RMF_TRAFFIC__PROFILEINTERNAL_HPP
#define SRC__RMF_TRAFFIC__PROFILEINTERNAL_HPP

#include <rmf_traffic/Profile.hpp>

namespace rmf_traffic {

//==============================================================================
class Profile::Implementation
{
public:

  geometry::ConstFinalConvexShapePtr footprint;
  geometry::ConstFinalConvexShapePtr vicinity;

  uint extra_footprint_count = 0;
  static const size_t MAX_EXTRA_FOOTPRINTS = 1;
  struct FootprintWithOffset
  {
    geometry::ConstFinalConvexShapePtr shape;
    Eigen::Vector3d offset;
  };
  using ExtraFootprintArray = 
    std::array<FootprintWithOffset, MAX_EXTRA_FOOTPRINTS>;
  ExtraFootprintArray extra_footprints;

  static const Implementation& get(const Profile& profile)
  {
    return *profile._pimpl;
  }

  void add_extra_footprint(geometry::ConstFinalConvexShapePtr shape, Eigen::Vector3d offset)
  {
    if (extra_footprint_count >= MAX_EXTRA_FOOTPRINTS)
      throw std::runtime_error("Maximum additional footprint shape count reached");

    auto& footprint_shape = extra_footprints[extra_footprint_count];
    footprint_shape.shape = std::move(shape);
    footprint_shape.offset = offset;
    ++extra_footprint_count;
  }

  void clear_extra_footprints()
  {
    for (uint i=0; i<extra_footprint_count; ++i)
      extra_footprints[i].shape.reset();
    extra_footprint_count = 0;
  }
};

} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__PROFILEINTERNAL_HPP
