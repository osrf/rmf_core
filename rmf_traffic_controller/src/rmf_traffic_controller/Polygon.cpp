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

#include <rmf_traffic_controller/geometry/Polygon.hpp>

#include <fcl/shape/geometric_shapes.h>

#include <strstream>

namespace rmf_traffic_controller {
namespace geometry {

namespace {

//==============================================================================
template<typename T>
std::string eigen_to_string(const T& obj)
{
  std::strstream str;
  str << obj;
  return str.str();
}

//==============================================================================
std::string generate_invalid_polygon_message(
    const Polygon::Intersections& intersections)
{
  std::string output = "[rmf_traffic_controller::Polygon] Invalid polygon "
      "requested: " + std::to_string(intersections.size())
      + " pair(s) of edges intersect. See the following pairs where segment A "
      "intersects segment B:"
      + "\n * (index A0) <vertex A0> -> (index A1) <vertex A1>"
      + "\n   (index B0) <vertex B0> -> (index B1) <vertex B1>\n";

  for(const Polygon::IntersectingPair& pair : intersections)
  {
    for(std::size_t i=0; i < 2; ++i)
    {
      if(i==0)
        output += " * ";
      else
        output += "   ";

      output += "(" +
          std::to_string(pair[i].indices[0]) + ") <" +
          eigen_to_string(pair[i].points[0]) + "> | (" +
          std::to_string(pair[i].indices[1]) + ") <" +
          eigen_to_string(pair[i].points[1]) + ">\n";
    }
  }

  return output;
}
} // anonymous namespace

//==============================================================================
InvalidPolygonException::InvalidPolygonException(
    Polygon::Intersections intersections)
  : intersecting_pairs(std::move(intersections)),
    _what(generate_invalid_polygon_message(intersecting_pairs))
{
  // Do nothing
}

//==============================================================================
const char* InvalidPolygonException::what() const noexcept
{
  return _what.c_str();
}

//==============================================================================
class PolygonInternal : public Shape::Internal
{
public:

  PolygonInternal(std::vector<Eigen::Vector2d> points)
    : _points(points)
  {
    // Do nothing
  }

  bool check_self_intersections() const
  {

  }

  std::shared_ptr<fcl::CollisionGeometry> make_fcl() const final
  {
    std::shared_ptr<fcl::Convex> polygon;

    if(check_self_intersections())
    {
    }
  }



private:
  std::vector<Eigen::Vector2d> _points;
};

} // namespace geometry
} // namespace rmf_traffic_controller
