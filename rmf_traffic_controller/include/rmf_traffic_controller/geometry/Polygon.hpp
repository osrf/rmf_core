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


#ifndef RMF_TRAFFIC_CONTROLLER__GEOMETRY__POLYGON_HPP
#define RMF_TRAFFIC_CONTROLLER__GEOMETRY__POLYGON_HPP

#include <rmf_traffic_controller/geometry/Shape.hpp>

#include <Eigen/Geometry>

#include <array>
#include <exception>
#include <vector>

namespace rmf_traffic_controller {
namespace geometry {

//==============================================================================
class Polygon
{
public:

  struct EdgeInfo
  {
    std::array<std::size_t, 2> indices;
    std::array<Eigen::Vector2d, 2> points;
  };

  using IntersectingPair = std::array<EdgeInfo, 2>;
  using Intersections = std::vector<IntersectingPair>;

  Polygon(std::vector<Eigen::Vector2d> points);

  Intersections get_self_intersections() const;


};

//==============================================================================
struct InvalidPolygonException : public std::exception
{
  InvalidPolygonException(Polygon::Intersections intersections);

  const char* what() const noexcept final;

  const Polygon::Intersections intersecting_pairs;
  const std::string _what;
};

} // namespace geometry
} // namespace rmf_traffic_controller

#endif // RMF_TRAFFIC_CONTROLLER__GEOMETRY__POLYGON_HPP
