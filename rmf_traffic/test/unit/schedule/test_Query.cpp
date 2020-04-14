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

#include <rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/schedule/Query.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test Query API")
{
  using namespace std::chrono_literals;

  rmf_traffic::schedule::Query query = rmf_traffic::schedule::make_query({});

  auto now = std::chrono::steady_clock::now();

  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  REQUIRE(query.spacetime().regions() != nullptr);

  query.spacetime().regions()->push_back(
    rmf_traffic::Region{"test_map", now, now+10s, {}});

  REQUIRE(query.spacetime().regions()->begin()
    != query.spacetime().regions()->end());

  auto& region = *query.spacetime().regions()->begin();

  REQUIRE(region.get_lower_time_bound() != nullptr);
  CHECK(*region.get_lower_time_bound() == now);
  REQUIRE(region.get_upper_time_bound() != nullptr);
  CHECK(*region.get_upper_time_bound() == now+10s);

  region.push_back(rmf_traffic::geometry::Space{final_box, tf});

  tf.rotate(Eigen::Rotation2Dd(90.0*M_PI/180.0));
  region.push_back(rmf_traffic::geometry::Space{final_box, tf});
  tf.rotate(Eigen::Rotation2Dd(180.0*M_PI/180.0));
  region.push_back(rmf_traffic::geometry::Space{final_box, tf});

  std::size_t regions = 0;
  std::size_t spaces = 0;
  for (const auto& region : *query.spacetime().regions())
  {
    ++regions;
    for (const auto& space : region)
    {
      ++spaces;

      // TODO(MXG): replace this with [[maybe_unused]] when we can use C++17
      (void)space;
    }
  }

  CHECK(regions == 1);
  CHECK(spaces == 3);

  // TODO(MXG): Write tests for every function to confirm that the
  // Query API works as intended
}
