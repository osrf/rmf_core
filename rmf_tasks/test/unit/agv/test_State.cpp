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

#include <rmf_tasks/agv/State.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Robot States")
{
  const rmf_traffic::agv::Plan::Start basic_start{
    std::chrono::steady_clock::now(),
    0,
    0.0};
  
  std::unique_ptr<rmf_tasks::agv::State> basic_state;

  WHEN("Empty battery")
  {
    CHECK_NOTHROW(basic_state.reset(new rmf_tasks::agv::State{
      basic_start,
      0,
      0.0}));
  }

  WHEN("Full battery")
  {
    CHECK_NOTHROW(basic_state.reset(new rmf_tasks::agv::State{
      basic_start,
      0,
      1.0}));
  }

  WHEN("Half battery")
  {
    CHECK_NOTHROW(basic_state.reset(new rmf_tasks::agv::State{
      basic_start,
      0,
      0.5}));
  }

  WHEN("Battery soc more than 1.0")
  {
    CHECK_THROWS(basic_state.reset(new rmf_tasks::agv::State{
      basic_start,
      0,
      1.0 + 1e-4}));
  }

  WHEN("Battery soc less than 0.0")
  {
    CHECK_THROWS(basic_state.reset(new rmf_tasks::agv::State{
      basic_start,
      0,
      0.0 - 1e-4}));
  }
}
