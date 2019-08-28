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

#include <rmf_traffic_controller/geometry/SimplePolygon.hpp>

#include <fcl/shape/geometric_shapes.h>

#include <sstream>

namespace rmf_traffic_controller {
namespace geometry {

namespace {

//==============================================================================
template<typename T>
std::string eigen_to_string(const T& obj)
{
  std::stringstream str;
  str << obj;
  return str.str();
}

//==============================================================================
std::string generate_self_intersection_polygon_message(
    const SimplePolygon::Intersections& intersections)
{
  std::string output = "[rmf_traffic_controller::Polygon] Invalid polygon "
      "requested: " + std::to_string(intersections.size())
      + " pair(s) of edges intersect. See the following pairs where segment A "
      "intersects segment B:"
      + "\n * (index A0) <vertex A0> -> (index A1) <vertex A1>"
      + "\n   (index B0) <vertex B0> -> (index B1) <vertex B1>\n";

  for(const SimplePolygon::IntersectingPair& pair : intersections)
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

//==============================================================================
std::string generate_insufficient_vertices_polygon_message(
    const std::size_t num_vertices)
{
  return "[rmf_traffic_controller::Polygon] Invalid polygon requested: "
      + std::to_string(num_vertices) + " vertices specified, but at least 3 "
      + "vertices are required for a polygon.";
}

//==============================================================================
bool compute_intersection(
    const SimplePolygon::EdgeInfo& edge_a,
    const SimplePolygon::EdgeInfo& edge_b,
    Eigen::Vector2d* intersection_point = nullptr)
{
  // Using as a reference: http://www.cs.swan.ac.uk/~cssimon/line_intersection.html

  const Eigen::Vector2d p_a0 = edge_a.points[0];
  const Eigen::Vector2d p_a1 = edge_a.points[1];
  const Eigen::Vector2d p_b0 = edge_b.points[0];
  const Eigen::Vector2d p_b1 = edge_b.points[1];

  Eigen::Matrix2d M;
  M.block<2,1>(0,0) =   p_b1 - p_b0;
  M.block<2,1>(1,0) = -(p_a1 - p_a0);

  const Eigen::Vector2d c = p_a0 - p_b0;

  // Before trying to solve the system, let's check if the edges are very
  // close to being parallel and quit early if they are.
  if(std::abs(M.determinant()) < 1e-8)
    return false;

  const Eigen::Vector2d t = M.colPivHouseholderQr().solve(c);
  for(int i=0; i < 2; ++i)
  {
    // If the "time" value is outside the range of [0.0, 1.0] then it is outside
    // the line segment that it's associated to, so we can disregard the alleged
    // intersection.
    const double t_i = t[i];
    if(t_i < 0.0 || 1.0 < t_i)
      return false;
  }

  if(intersection_point)
    *intersection_point = p_a0 + t[0]*(p_a1 - p_a0);

  return true;
}

//==============================================================================
using Triangle = std::array<std::size_t, 3>;

//==============================================================================
struct AliasVertex
{
  std::size_t original_index;
  Eigen::Vector2d point;
};
using Subpolygon = std::vector<AliasVertex>;

//==============================================================================
bool is_polygon_ear(
    const Subpolygon& polygon,
    const Triangle& triangle)
{
  using EdgeInfo = SimplePolygon::EdgeInfo;

  const std::size_t a0 = triangle[0];
  const std::size_t a1 = triangle[2]; // yes, [2] is intended
  const EdgeInfo edge_a =
      EdgeInfo{{a0, a1}, {polygon[a0].point, polygon[a1].point}};

  for(std::size_t b0=0; b0 < polygon.size(); ++b0)
  {
    const std::size_t b1 = (b0 == polygon.size()-1)? 0 : b0+1;

    // Ignore any edges that are in contact with the edge we care about.
    if(b0 == a0 || b0 == a1 || b1 == a0 || b1 == a1)
      continue;

    const EdgeInfo edge_b =
        EdgeInfo{{b0, b1}, {polygon[b0].point, polygon[b1].point}};

    if(compute_intersection(edge_a, edge_b))
      return false;
  }

  return true;
}

//==============================================================================
std::size_t find_deepest_reflex_point(
    const Subpolygon& polygon,
    const Triangle& triangle)
{

}

//==============================================================================
std::array<Subpolygon, 2> split_subpolygon(
    const Subpolygon& original,
    const std::array<std::size_t, 2>& edge)
{
  std::array<Subpolygon, 2> output;

  const std::size_t split_start = std::min(edge[0], edge[1]);
  const std::size_t split_end = std::max(edge[0], edge[1]);

  Subpolygon& output_0 = output[0];
  for(std::size_t i=0; i < original.size(); ++i)
  {
    output_0.push_back(original[i]);
    if(i == split_start)
    {
      // When we reach the split, just jump to the index that comes before the
      // endpoint of the split. Then on the next iteration of this loop, the
      // variable i will have the same value as split_end.
      i = split_end-1;
    }
  }

  Subpolygon& output_1 = output[1];
  for(std::size_t i = split_start; i <= split_end; ++i)
    output_1.push_back(original[i]);

  return output;
}

//==============================================================================
std::vector<Triangle> decompose_polygon(
    const std::vector<Eigen::Vector2d>& polygon)
{
  std::vector<Subpolygon> subpolygon_queue;

  // Creat the initial sub-polygon to analyze
  subpolygon_queue.emplace_back([&polygon]() -> Subpolygon
  {
    Subpolygon sp;
    for(std::size_t i=0; i < polygon.size(); ++i)
      sp.emplace_back(AliasVertex{i, polygon[i]});
    return sp;
  }());

  std::vector<Triangle> triangles;
  while(!subpolygon_queue.empty())
  {
    Subpolygon next_polygon = subpolygon_queue.back();
    subpolygon_queue.pop_back();

    if(next_polygon.size() <= 3)
    {
      triangles.push_back({
        next_polygon[0].original_index,
        next_polygon[1].original_index,
        next_polygon[2].original_index
      });

      continue;
    }

    const std::size_t N = next_polygon.size();
    const std::size_t pivot_point = N-2;
    const Triangle next_triangle = {pivot_point-1, pivot_point, pivot_point+1};
    if(is_polygon_ear(next_polygon, next_triangle))
    {
      // If the triangle is an ear (not intersected by any other lines of the
      // polygon), then we can just snip it off.

      triangles.push_back({
        next_polygon[next_triangle[0]].original_index,
        next_polygon[next_triangle[1]].original_index,
        next_polygon[next_triangle[2]].original_index
      });

      next_polygon.resize(N-2);
      subpolygon_queue.emplace_back(std::move(next_polygon));
      continue;
    }

    // If the polgyon is not an ear, we should find its deepest reflex point,
    // and then slice the polygon along the line that joins the deepest reflex
    // point to the triangle's pivot point.

    const std::size_t reflex_point =
        find_deepest_reflex_point(next_polygon, next_triangle);

    const auto new_subpolygons = split_subpolygon(
          next_polygon, {pivot_point, reflex_point});

    for(auto&& new_subpolygon : new_subpolygons)
      subpolygon_queue.emplace_back(std::move(new_subpolygon));
  }

  return triangles;
}

} // anonymous namespace

//==============================================================================
InvalidSimplePolygonException::InvalidSimplePolygonException(
    SimplePolygon::Intersections intersections,
    const std::size_t num_vertices)
  : intersecting_pairs(std::move(intersections)),
    num_vertices(num_vertices),
    _what(generate_self_intersection_polygon_message(intersecting_pairs))
{
  // Do nothing
}

//==============================================================================
InvalidSimplePolygonException::InvalidSimplePolygonException(
    const std::size_t num_vertices)
  : num_vertices(num_vertices),
    _what(generate_insufficient_vertices_polygon_message(num_vertices))
{
  // Do nothing
}

//==============================================================================
const char* InvalidSimplePolygonException::what() const noexcept
{
  return _what.c_str();
}

//==============================================================================
class PolygonInternal : public Shape::Internal
{
public:

  using EdgeInfo = SimplePolygon::EdgeInfo;
  using IntersectingPair = SimplePolygon::IntersectingPair;
  using Intersections = SimplePolygon::Intersections;

  PolygonInternal(std::vector<Eigen::Vector2d> points)
    : _points(points)
  {
    // Do nothing
  }

  bool check_self_intersections(Intersections* intersections) const
  {
    if(intersections)
      intersections->clear();

    for(std::size_t a0=0; a0 < _points.size()-1; ++a0)
    {
      const std::size_t a1 = a0+1;
      const EdgeInfo edge_a = EdgeInfo{{a0, a1}, {_points[a0], _points[a1]}};

      for(std::size_t b0=a0+2; b0 < _points.size(); ++b0)
      {
        const std::size_t b1 = (b0 == _points.size()-1)? 0 : b0+1;
        if(b1 == a0 || b1 == a1)
        {
          // Skip this edge if it's in contact with edge A
          continue;
        }

        const EdgeInfo edge_b = EdgeInfo{{b0, b1}, {_points[b0], _points[b1]}};

        if(compute_intersection(edge_a, edge_b))
        {
          if(intersections)
          {
            // If we have a non-null intersections argument, then the user is
            // asking to receive all intersections.
            intersections->push_back(IntersectingPair{edge_a, edge_b});
          }
          else
          {
            // If we have a nullptr intersection argument, then the user does
            // not care about getting all intersection data, and we can just
            // quit early.
            return true;
          }
        }
      }
    }

    // If we had to collect all the intersections, then return false if the
    // collection of intersections is empty. Return true if there were any
    // intersections computed.
    if(intersections)
      return !intersections->empty();

    // If we did not collect the intersections, then return false here because
    // no intersections were encountered along the way.
    return false;
  }

  void except_on_invalid_polygon() const
  {
    if(_points.size() < 3)
      throw InvalidSimplePolygonException(_points.size());

    Intersections intersections;
    if(check_self_intersections(&intersections))
      throw InvalidSimplePolygonException(std::move(intersections), _points.size());
  }

  std::shared_ptr<fcl::CollisionGeometry> make_fcl() const final
  {
    except_on_invalid_polygon();

    std::shared_ptr<fcl::Convex> polygon;
    const std::vector<Triangle> triangles = decompose_polygon(_points);


  }



private:
  std::vector<Eigen::Vector2d> _points;
};

} // namespace geometry
} // namespace rmf_traffic_controller
