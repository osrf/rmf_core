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
/// \brief The SimplePolygon class represent a simple polygon. A polygon is
/// "simple" if it never intersects itself. It is also expected to have at least
/// 3 vertices. The polygon is allowed to be convex or concave.
///
/// A simple polygon is also closed, but this class will automatically "close"
/// itself by assuming a connection between the last vertex and the first.
class SimplePolygon
{
public:

  struct EdgeInfo
  {
    std::array<std::size_t, 2> indices;
    std::array<Eigen::Vector2d, 2> points;
  };

  using IntersectingPair = std::array<EdgeInfo, 2>;
  using Intersections = std::vector<IntersectingPair>;

  SimplePolygon(std::vector<Eigen::Vector2d> points);

  Intersections get_self_intersections() const;

  bool has_self_intersections() const;

  const std::vector<Eigen::Vector2d>& get_points() const;

  void remove_point(std::size_t index);

  void add_point(const Eigen::Vector2d& p);

  void insert_point(const Eigen::Vector2d& p);

  void set_point(std::size_t index, const Eigen::Vector2d& p);

};

//==============================================================================
/// \brief If an invalid simple polygon (a polygon having self-intersections or
/// having less than 3 vertices) is passed into a schedule, this exception will
/// be raised.
struct InvalidSimplePolygonException : public std::exception
{
  /// \brief Constructor for an invalid Polygon that has self-intersections.
  InvalidSimplePolygonException(
      SimplePolygon::Intersections intersections,
      std::size_t num_vertices);

  /// \brief Constructor for an invalid Polygon that has too few vertices.
  InvalidSimplePolygonException(std::size_t num_vertices);

  const char* what() const noexcept final;

  /// If the error was caused by intersecting pairs, this field will describe
  /// which pairs were the problem.
  const SimplePolygon::Intersections intersecting_pairs;

  /// This field will point to the number of vertices that were present.
  const std::size_t num_vertices;

  const std::string _what;
};

} // namespace geometry
} // namespace rmf_traffic_controller

#endif // RMF_TRAFFIC_CONTROLLER__GEOMETRY__POLYGON_HPP
