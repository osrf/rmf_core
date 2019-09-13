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

#include <rmf_traffic/geometry/Shape.hpp>
#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/geometry/SimplePolygon.hpp>

#include <rmf_utils/catch.hpp>

TEST_CASE("Verify that these shape types can be upcast")
{
  // Make sure all the shape types can be constructed and cast to shapes
  std::shared_ptr<rmf_traffic::geometry::Shape> box =
      std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0);

  std::shared_ptr<rmf_traffic::geometry::Shape> circle =
      std::make_shared<rmf_traffic::geometry::Circle>(1.0);

  std::shared_ptr<rmf_traffic::geometry::Shape> polygon =
      std::make_shared<rmf_traffic::geometry::SimplePolygon>(
        std::vector<Eigen::Vector2d>());

  // Make sure the two convex shape type can be constructed and cast to Convex
  std::shared_ptr<rmf_traffic::geometry::ConvexShape> convex_box =
      std::make_shared<rmf_traffic::geometry::Box>(1.0, 1.0);

  std::shared_ptr<rmf_traffic::geometry::ConvexShape> convex_circle =
      std::make_shared<rmf_traffic::geometry::Circle>(1.0);
}
