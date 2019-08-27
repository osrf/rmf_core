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

#include "ShapeInternal.hpp"

#include <rmf_traffic_controller/geometry/Circle.hpp>

#include <fcl/shape/geometric_shapes.h>

namespace rmf_traffic_controller {
namespace geometry {

//==============================================================================
class CircleInternal : public Shape::Internal
{
public:

  CircleInternal(double radius)
    : _radius(radius)
  {
    // Do nothing
  }

  std::shared_ptr<fcl::CollisionGeometry> make_fcl() const final
  {
    return std::make_shared<fcl::Sphere>(_radius);
  }

  double _radius;
};

//==============================================================================
Circle::Circle(double radius)
  : ConvexShape(std::make_unique<CircleInternal>(radius))
{
  // Do nothing
}

//==============================================================================
void Circle::set_radius(double r)
{
  static_cast<CircleInternal*>(_get_internal())->_radius = r;
}

//==============================================================================
double Circle::get_radius() const
{
  return static_cast<const CircleInternal*>(_get_internal())->_radius;
}

} // namespace geometry
} // namespace rmf_traffic_controller
