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

#include <rmf_traffic/geometry/Space.hpp>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
class Space::Implementation
{
public:

  geometry::ConstFinalShapePtr shape;
  Eigen::Isometry2d pose;

};

//==============================================================================
Space::Space(ConstFinalShapePtr shape, Eigen::Isometry2d tf)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::move(shape), std::move(tf)}))
{
  // Do nothing
}

//==============================================================================
const ConstFinalShapePtr& Space::get_shape() const
{
  return _pimpl->shape;
}

//==============================================================================
Space& Space::set_shape(geometry::ConstFinalShapePtr shape)
{
  _pimpl->shape = std::move(shape);
  return *this;
}

//==============================================================================
const Eigen::Isometry2d& Space::get_pose() const
{
  return _pimpl->pose;
}

//==============================================================================
Space& Space::set_pose(Eigen::Isometry2d tf)
{
  _pimpl->pose = std::move(tf);
  return *this;
}

} // namespace geometry
} // namespace rmf_traffic
