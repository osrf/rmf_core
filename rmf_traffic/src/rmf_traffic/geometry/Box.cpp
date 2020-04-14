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

#include <rmf_traffic/geometry/Box.hpp>

#include <fcl/shape/geometric_shapes.h>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
class BoxInternal : public Shape::Internal
{
public:

  BoxInternal(double x, double y)
  : _x(x),
    _y(y)
  {
    // Do nothing
  }

  CollisionGeometries make_fcl() const final
  {
    // Note: The z-value doesn't really matter, as long as it's greater than 0.0
    return {std::make_shared<fcl::Box>(_x, _y, 1.0)};
  }

  double _x;
  double _y;

};

//==============================================================================
Box::Box(double x_length, double y_length)
: ConvexShape(std::make_unique<BoxInternal>(x_length, y_length))
{
  // Do nothing
}

//==============================================================================
Box::Box(const Box& other)
: ConvexShape(std::make_unique<BoxInternal>(
      static_cast<const BoxInternal&>(*other._get_internal())))
{
  // Do nothing
}

//==============================================================================
Box& Box::operator=(const Box& other)
{
  static_cast<BoxInternal&>(*_get_internal()) =
    static_cast<const BoxInternal&>(*other._get_internal());

  return *this;
}

//==============================================================================
void Box::set_x_length(double x_length)
{
  static_cast<BoxInternal*>(_get_internal())->_x = x_length;
}

//==============================================================================
void Box::set_y_length(double y_length)
{
  static_cast<BoxInternal*>(_get_internal())->_y = y_length;
}

//==============================================================================
double Box::get_x_length() const
{
  return static_cast<const BoxInternal*>(_get_internal())->_x;
}

//==============================================================================
double Box::get_y_length() const
{
  return static_cast<const BoxInternal*>(_get_internal())->_y;
}

//==============================================================================
FinalShape Box::finalize() const
{
  double characteristic_length = 0.5 * std::sqrt(
    this->get_x_length() * this->get_x_length()
    + this->get_y_length() * this->get_y_length());
  return FinalShape::Implementation::make_final_shape(
    rmf_utils::make_derived_impl<const Shape, const Box>(*this),
    _get_internal()->make_fcl(), characteristic_length);
}

//==============================================================================
FinalConvexShape Box::finalize_convex() const
{
  double characteristic_length = 0.5 * std::sqrt(
    this->get_x_length() * this->get_x_length()
    + this->get_y_length() * this->get_y_length());
  return FinalConvexShape::Implementation::make_final_shape(
    rmf_utils::make_derived_impl<const Shape, const Box>(*this),
    _get_internal()->make_fcl(), characteristic_length);
}

} // namespace geometry
} // namespace rmf_traffic
