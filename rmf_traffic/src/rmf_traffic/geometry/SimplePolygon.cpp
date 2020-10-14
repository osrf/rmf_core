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

//#include <rmf_traffic/geometry/SimplePolygon.hpp>
#include "SimplePolygon.hpp"

#include <fcl/geometry/shape/convex.h>

#include <sstream>

namespace rmf_traffic {
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
  std::string output = "[rmf_traffic::Polygon] Invalid polygon "
    "requested: " + std::to_string(intersections.size())
    + " pair(s) of edges intersect. See the following pairs where waypoint A "
    "intersects waypoint B:"
    + "\n * (index A0) <vertex A0> -> (index A1) <vertex A1>"
    + "\n   (index B0) <vertex B0> -> (index B1) <vertex B1>\n";

  for (const SimplePolygon::IntersectingPair& pair : intersections)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (i == 0)
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
  return "[rmf_traffic::Polygon] Invalid polygon requested: "
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
  M.block<2, 1>(0, 0) = p_b1 - p_b0;
  M.block<2, 1>(0, 1) = -(p_a1 - p_a0);

  const Eigen::Vector2d c = p_a0 - p_b0;

  // Before trying to solve the system, let's check if the edges are very
  // close to being parallel and quit early if they are.
  if (std::abs(M.determinant()) < 1e-8)
    return false;

  const Eigen::Vector2d t = M.colPivHouseholderQr().solve(c);
  for (int i = 0; i < 2; ++i)
  {
    // If the "time" value is outside the range of [0.0, 1.0] then it is outside
    // the line waypoint that it's associated to, so we can disregard the alleged
    // intersection.
    const double t_i = t[i];
    if (t_i < 0.0 || 1.0 < t_i)
      return false;
  }

  if (intersection_point)
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

  for (std::size_t b0 = 0; b0 < polygon.size(); ++b0)
  {
    const std::size_t b1 = (b0 == polygon.size()-1) ? 0 : b0+1;

    // Ignore any edges that are in contact with the edge we care about.
    if (b0 == a0 || b0 == a1 || b1 == a0 || b1 == a1)
      continue;

    const EdgeInfo edge_b =
      EdgeInfo{{b0, b1}, {polygon[b0].point, polygon[b1].point}};

    if (compute_intersection(edge_a, edge_b))
      return false;
  }

  return true;
}

//==============================================================================
std::size_t find_deepest_reflex_point(
  const Subpolygon& polygon,
  const Triangle& triangle)
{
  // Reference: https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_triangles
  // Simpler form: https://stackoverflow.com/a/2049712

  const Eigen::Vector2d p_pivot = polygon[triangle[1]].point;
  const Eigen::Vector2d p_preceding = polygon[triangle[0]].point;
  const Eigen::Vector2d p_successive = polygon[triangle[2]].point;

  const Eigen::Matrix2d M_inv = [&]()
    {
      Eigen::Matrix2d output;
      output.block<2, 1>(0, 0) = p_preceding - p_pivot;
      output.block<2, 1>(0, 0) = p_successive - p_pivot;
      return output.inverse();
    } ();

  const Eigen::Vector2d diagonal = (p_successive - p_preceding).normalized();

  // deepest is initialized to a virtually impossible value. A polygon this big
  // would be absurd.
  std::size_t deepest_index = static_cast<std::size_t>(-1);
  double deepest_value = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i)
  {
    // Skip any indices that are part of the triangle
    if (i == triangle[0] || i == triangle[1] || i == triangle[2])
      continue;

    const Eigen::Vector2d p = polygon[i].point;

    // Check if the point is inside the triangle, and skip to the next index
    // if it is not.
    const Eigen::Vector2d s = M_inv * (p - p_pivot);
    if (s[0] + s[1] > 1.0 || s[0] < 0.0 || s[1] < 0.0)
      continue;

    // If the point is inside the triangle, then compute its depth (distance
    // from the nearest point on the diagonal line waypoint).
    const Eigen::Vector2d p_rel = p - p_preceding;
    const double depth = (p_rel - diagonal.dot(p_rel) * diagonal).norm();

    // Compare the depth to the others, and find the deepest
    if (depth > deepest_value)
    {
      deepest_index = i;
      deepest_value = depth;
    }
  }

  return deepest_index;
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
  for (std::size_t i = 0; i < original.size(); ++i)
  {
    output_0.push_back(original[i]);
    if (i == split_start)
    {
      // When we reach the split, just jump the value of i to the index that
      // comes before the endpoint of the split. Then on the next iteration of
      // this loop, the variable i will have the same value as split_end.
      i = split_end-1;
    }
  }

  Subpolygon& output_1 = output[1];
  for (std::size_t i = split_start; i <= split_end; ++i)
    output_1.push_back(original[i]);

  return output;
}

//==============================================================================
Triangle get_original_indices(
  const Subpolygon& polygon,
  const Triangle& triangle)
{
  return Triangle{
    polygon[triangle[0]].original_index,
    polygon[triangle[1]].original_index,
    polygon[triangle[2]].original_index
  };
}

//==============================================================================
std::vector<Triangle> decompose_polygon(
  const std::vector<Eigen::Vector2d>& polygon)
{
  std::vector<Subpolygon> subpolygon_queue;

  // Creat the initial sub-polygon to analyze
  subpolygon_queue.emplace_back(
    [&polygon]() -> Subpolygon
    {
      Subpolygon sp;
      for (std::size_t i = 0; i < polygon.size(); ++i)
        sp.emplace_back(AliasVertex{i, polygon[i]});

      if ( (sp.back().point - sp.front().point).norm() < 1e-8)
      {
        // If the first and last point are very very close, we will effective snap
        // them together by deleting the last one.
        sp.pop_back();
      }

      return sp;
    } ());

  std::vector<Triangle> triangles;
  while (!subpolygon_queue.empty())
  {
    Subpolygon next_polygon = subpolygon_queue.back();
    subpolygon_queue.pop_back();

    const std::size_t N = next_polygon.size();
    if (N == 3)
    {
      triangles.push_back(get_original_indices(next_polygon, {0, 1, 2}));
      continue;
    }
    else if (N < 3)
    {
      std::cerr << "[rmf_traffic::geometry::decompose_polygon] "
                << "A subpolygon has an incorrect number of vertices: " << N
                << ". This is a bug that should never happen. Please report "
                << "this to the developers!" << std::endl;
      throw InvalidSimplePolygonException(N);
    }

    const std::size_t pivot_point = N-2;
    const Triangle next_triangle = {pivot_point-1, pivot_point, pivot_point+1};
    if (is_polygon_ear(next_polygon, next_triangle))
    {
      // If the triangle is an ear (not intersected by any other lines of the
      // polygon), then we can just snip it off.
      triangles.push_back(get_original_indices(next_polygon, next_triangle));

      // Removing the last two vertices will snip off the ear while leaving the
      // remaining polygon untouched.
      next_polygon.resize(N-2);
      subpolygon_queue.emplace_back(std::move(next_polygon));
      continue;
    }

    // If the polgyon is not an ear, we should find its deepest reflex point,
    // and then slice the polygon along the line that joins the deepest reflex
    // point to the triangle's pivot point.

    // NOTE(MXG): It might be possible to improve performance here by
    // remembering which line segments crossed the diagonal during the
    // computation of is_polygon_ear(~) and using that to narrow down which
    // vertices to look at when computing the deepest reflext point.
    const std::size_t reflex_point =
      find_deepest_reflex_point(next_polygon, next_triangle);

    const auto new_subpolygons = split_subpolygon(
      next_polygon, {pivot_point, reflex_point});

    for (auto&& new_subpolygon : new_subpolygons)
      subpolygon_queue.emplace_back(std::move(new_subpolygon));
  }

  return triangles;
}

//==============================================================================
double cross_product_2D(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
{
  return v0[0]*v1[1] - v0[1]*v1[0];
}

//==============================================================================
bool is_polygon_convex(const std::vector<Eigen::Vector2d>& polygon)
{
  // We know that the polygon is not self-intersecting, because that gets
  // checked earlier. To verify that it's convex, all we need to check is that
  // the cross-product of two consecutive edges never changes sign.

  const Eigen::Vector2d e0 = polygon[1] - polygon[0];
  const Eigen::Vector2d e1 = polygon[2] - polygon[1];

  // ccw: counter-clockwise
  const bool must_be_ccw = cross_product_2D(e0, e1) > 0.0;

  Eigen::Vector2d e_previous = e1;
  for (std::size_t i = 2; i < polygon.size(); ++i)
  {
    const std::size_t i_next = i+1 == polygon.size() ? 0 : i+1;
    const Eigen::Vector2d ei = polygon[i_next] - polygon[i];

    const bool is_ccw = cross_product_2D(e_previous, ei) > 0.0;
    if (is_ccw != must_be_ccw)
      return false;

    e_previous = ei;
  }

  return true;
}

//==============================================================================
class ConvexWrapper : public fcl::Convexd
{
public:
  using PointArray = std::vector<fcl::Vector3d>;

  ConvexWrapper(const std::shared_ptr<const PointArray>& points_)
  : fcl::Convexd(points_, 0, std::shared_ptr<const std::vector<int>>()),
    point_storage(std::move(points_))
  {
    // Do nothing
  }

  static std::shared_ptr<fcl::Convexd> make(
    const std::vector<Eigen::Vector2d>& points)
  {
    // Create an array with double the points, because we need to place the
    // points at both ground level and ceiling level.
    auto fcl_points = std::make_shared<PointArray>(2*points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
    {
      const Eigen::Vector2d& p = points[i];
      (*fcl_points)[2*i] = fcl::Vector3d(p[0], p[1], 0.0);
      (*fcl_points)[2*i + 1] = fcl::Vector3d(p[0], p[1], 1.0);
    }

    return std::make_shared<ConvexWrapper>(
      std::move(fcl_points));
  }

  std::shared_ptr<const PointArray> point_storage;

};

//==============================================================================
std::shared_ptr<fcl::Convexd> make_convex(
  const std::vector<Eigen::Vector2d>& polygon)
{
  return ConvexWrapper::make(polygon);
}

//==============================================================================
std::vector<std::shared_ptr<fcl::Convexd>> make_triangulation(
  const std::vector<Eigen::Vector2d>& polygon)
{
  const std::vector<Triangle> triangles = decompose_polygon(polygon);

  std::vector<std::shared_ptr<fcl::Convexd>> triangulation;

  std::vector<Eigen::Vector2d> points;
  points.reserve(3);
  for (const Triangle& triangle : triangles)
  {
    points.clear();
    for (const std::size_t index : triangle)
      points.push_back(polygon[index]);

    triangulation.push_back(ConvexWrapper::make(points));
  }

  return triangulation;
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
class SimplePolygonInternal : public Shape::Internal
{
public:

  using EdgeInfo = SimplePolygon::EdgeInfo;
  using IntersectingPair = SimplePolygon::IntersectingPair;
  using Intersections = SimplePolygon::Intersections;

  SimplePolygonInternal(std::vector<Eigen::Vector2d> points)
  : _points(points)
  {
    // Do nothing
  }

  bool check_self_intersections(Intersections* intersections) const
  {
    if (intersections)
      intersections->clear();

    for (std::size_t a0 = 0; a0 < _points.size()-1; ++a0)
    {
      const std::size_t a1 = a0+1;
      const EdgeInfo edge_a = EdgeInfo{{a0, a1}, {_points[a0], _points[a1]}};

      for (std::size_t b0 = a0+2; b0 < _points.size(); ++b0)
      {
        const std::size_t b1 = (b0 == _points.size()-1) ? 0 : b0+1;
        if (b1 == a0 || b1 == a1)
        {
          // Skip this edge if it's in contact with edge A
          continue;
        }

        const EdgeInfo edge_b = EdgeInfo{{b0, b1}, {_points[b0], _points[b1]}};

        if (compute_intersection(edge_a, edge_b))
        {
          if (intersections)
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
    if (intersections)
      return !intersections->empty();

    // If we did not collect the intersections, then return false here because
    // no intersections were encountered along the way.
    return false;
  }

  void except_on_invalid_polygon() const
  {
    if (_points.size() < 3)
      throw InvalidSimplePolygonException(_points.size());

    Intersections intersections;
    // *INDENT-OFF*
    if (check_self_intersections(&intersections))
      throw InvalidSimplePolygonException(std::move(intersections),
        _points.size());
    // *INDENT-ON*
  }

  CollisionGeometries make_fcl() const final
  {
    except_on_invalid_polygon();

    CollisionGeometries shapes;
    if (is_polygon_convex(_points))
    {
      shapes.push_back(make_convex(_points));
    }
    else
    {
      const auto triangles = make_triangulation(_points);
      shapes.reserve(triangles.size());
      for (auto&& triangle : triangles)
        shapes.push_back(std::move(triangle));
    }

    return shapes;
  }

  std::vector<Eigen::Vector2d> _points;
};

//==============================================================================
SimplePolygon::SimplePolygon(std::vector<Eigen::Vector2d> points)
: Shape(std::make_unique<SimplePolygonInternal>(std::move(points)))
{
  // Do nothing
}

//==============================================================================
SimplePolygon::SimplePolygon(const SimplePolygon& other)
: Shape(std::make_unique<SimplePolygonInternal>(
      static_cast<const SimplePolygonInternal&>(*other._get_internal())))
{
  // Do nothing
}

//==============================================================================
SimplePolygon& SimplePolygon::operator=(const SimplePolygon& other)
{
  static_cast<SimplePolygonInternal&>(*_get_internal()) =
    static_cast<const SimplePolygonInternal&>(*other._get_internal());

  return *this;
}

//==============================================================================
auto SimplePolygon::get_self_intersections() const -> Intersections
{
  Intersections intersections;
  static_cast<const SimplePolygonInternal*>(_get_internal())
  ->check_self_intersections(&intersections);
  return intersections;
}

//==============================================================================
bool SimplePolygon::has_self_intersections() const
{
  return static_cast<const SimplePolygonInternal*>(_get_internal())
    ->check_self_intersections(nullptr);
}

//==============================================================================
std::vector<Eigen::Vector2d> SimplePolygon::get_points() const
{
  return static_cast<const SimplePolygonInternal*>(_get_internal())->_points;
}

//==============================================================================
std::size_t SimplePolygon::get_num_points() const
{
  return static_cast<const SimplePolygonInternal*>(
    _get_internal())->_points.size();
}

//==============================================================================
Eigen::Vector2d& SimplePolygon::get_point(const std::size_t index)
{
  return static_cast<SimplePolygonInternal*>(
    _get_internal())->_points.at(index);
}

//==============================================================================
const Eigen::Vector2d& SimplePolygon::get_point(const std::size_t index) const
{
  return static_cast<const SimplePolygonInternal*>(
    _get_internal())->_points.at(index);
}

//==============================================================================
void SimplePolygon::remove_point(const std::size_t index)
{
  auto& points = static_cast<SimplePolygonInternal*>(_get_internal())->_points;
  points.erase(points.begin()+index);
}

//==============================================================================
void SimplePolygon::add_point(const Eigen::Vector2d& p)
{
  auto& points = static_cast<SimplePolygonInternal*>(_get_internal())->_points;
  points.push_back(p);
}

//==============================================================================
void SimplePolygon::insert_point(
  const std::size_t index,
  const Eigen::Vector2d& p)
{
  auto& points = static_cast<SimplePolygonInternal*>(_get_internal())->_points;
  points.insert(points.begin()+index, p);
}

//==============================================================================
FinalShape SimplePolygon::finalize() const
{
  double characteristic_length = 0;
  for (const Eigen::Vector2d& point : this->get_points())
  {
    const double distance = point.norm();
    if (distance > characteristic_length)
      characteristic_length = distance;
  }
  return FinalShape::Implementation::make_final_shape(
    rmf_utils::make_derived_impl<const Shape, const SimplePolygon>(*this),
    _get_internal()->make_fcl(), characteristic_length);
}

} // namespace geometry
} // namespace rmf_traffic
