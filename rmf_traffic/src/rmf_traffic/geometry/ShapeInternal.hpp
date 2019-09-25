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

#ifndef SRC__RMF_TRAFFIC__GEOMETRY__SHAPEINTERNAL_HPP
#define SRC__RMF_TRAFFIC__GEOMETRY__SHAPEINTERNAL_HPP

#include <rmf_traffic/geometry/Shape.hpp>

#include <fcl/collision_object.h>

#include <vector>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
/// \brief Implementations of this class must be created by the child classes of
/// Shape, and then passed to the constructor of Shape.
class Shape::Internal
{
public:

  using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometry>;
  using CollisionGeometries = std::vector<CollisionGeometryPtr>;

  virtual CollisionGeometries make_fcl() const = 0;

};

} // namespace geometry
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__GEOMETRY__SHAPEINTERNAL_HPP
