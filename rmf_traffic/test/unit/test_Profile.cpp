/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

using namespace std::chrono_literals;

SCENARIO("Testing Construction")
{

using Profile = rmf_traffic::Profile;
using Trajecotry = rmf_traffic::Trajectory;

WHEN("When vicinity is not passed into the constructor")
{
  const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);

  const auto profile = Profile{shape};

  auto& footprint = profile.footprint();
  auto& vicinity = profile.vicinity();

  CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));
  CHECK((footprint->get_characteristic_length() - 1.0) == Approx(0.0).margin(1e-6));

  shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(2.0);
  profile.vicinity(shape);


}

}