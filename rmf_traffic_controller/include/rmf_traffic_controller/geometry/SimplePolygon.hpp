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
class SimplePolygon : public Shape
{
public:

  /// \brief A simple struct to provide information about an edge on the
  /// polygon, including the index and location of each vertex.
  struct EdgeInfo
  {
    /// \brief The index of each vertex along this edge.
    std::array<std::size_t, 2> indices;

    /// \brief The location of each vertex on this edge.
    std::array<Eigen::Vector2d, 2> points;
  };

  using IntersectingPair = std::array<EdgeInfo, 2>;
  using Intersections = std::vector<IntersectingPair>;

  /// \brief Construct this SimplePolygon with a set of points.
  ///
  /// The set of points is expected to be a connected sequence of vertices. Each
  /// edge of the polygon will be created by sequential pairs of these points.
  ///
  /// Similarly, points inside the SimplePolygon shape will be stored in a
  /// sequence based on how they are connected to each other.
  ///
  /// The last point will be automatically connected to the first point to
  /// ensure a closed polygon.
  SimplePolygon(std::vector<Eigen::Vector2d> points);

  /// \brief Compute any self-intersections that may exist in this SimplePolygon
  /// due to the current arrangement of points. Note that if the SimplePolygon
  /// has any self-intersections, an exception will be raised if it gets passed
  /// to a scheduler.
  Intersections get_self_intersections() const;

  /// \brief Check whether or not the SimplePolygon has any self-intersections.
  /// This will return faster than get_self_intersections() in the event that a
  /// self-intersection exists.
  bool has_self_intersections() const;

  /// \brief Get an array of the current points.
  std::vector<Eigen::Vector2d> get_points() const;

  /// \brief Get the number of points currently in this polygon.
  std::size_t get_num_points() const;

  /// \brief Get a specific point of the polygon.
  Eigen::Vector2d& get_point(std::size_t index);

  /// \brief const-qualified version of get_point()
  const Eigen::Vector2d& get_point(std::size_t index) const;

  /// \brief Remove the point at index.
  void remove_point(std::size_t index);

  /// \brief Add a point to the end of the polygon sequence.
  void add_point(const Eigen::Vector2d& p);

  /// \brief Insert a point into the sequence at the specified index. All points
  /// currently at the specified index and after will have their indices
  /// incremented.
  void insert_point(std::size_t index, const Eigen::Vector2d& p);

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
