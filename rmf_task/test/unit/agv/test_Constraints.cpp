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

#include <chrono>
#include <memory>

#include <rmf_task/agv/Constraints.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test Constraints")
{
  std::unique_ptr<rmf_task::agv::Constraints> constraints;

  WHEN("Minimum battery threshold")
  {
    CHECK_NOTHROW(
      constraints.reset(new rmf_task::agv::Constraints{0.0}));
  }

  WHEN("Maximum battery threshold")
  {
    CHECK_NOTHROW(
      constraints.reset(new rmf_task::agv::Constraints{1.0}));
  }

  WHEN("Half battery threshold")
  {
    CHECK_NOTHROW(
      constraints.reset(new rmf_task::agv::Constraints{0.5}));
  }

  WHEN("Below minimum battery threshold")
  {
    CHECK_THROWS(
      constraints.reset(new rmf_task::agv::Constraints{0.0 - 1e-4}));
  }

  WHEN("Above maximum battery threshold")
  {
    CHECK_THROWS(
      constraints.reset(new rmf_task::agv::Constraints{1.0 + 1e-4}));
  }
}
