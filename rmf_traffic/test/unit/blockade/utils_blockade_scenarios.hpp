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

#ifndef TEST__UNIT__BLOCKADE__UTILS_BLOCKADE_SCENARIOS_HPP
#define TEST__UNIT__BLOCKADE__UTILS_BLOCKADE_SCENARIOS_HPP

#include <rmf_traffic/blockade/Writer.hpp>

//==============================================================================
template <std::size_t N>
std::vector<rmf_traffic::blockade::Writer::Checkpoint> make_path(
    const std::array<Eigen::Vector2d, N>& points)
{
  using namespace rmf_traffic::blockade;

  std::vector<Writer::Checkpoint> path;
  path.reserve(N);
  for (const auto& p : points)
    path.push_back(Writer::Checkpoint{p, "test_map", true});

  return path;
}

//==============================================================================
struct GridlockScenario
{
  using Checkpoint = rmf_traffic::blockade::Writer::Checkpoint;

  std::vector<std::vector<Checkpoint>> paths;
  std::vector<std::size_t> goals;
};

//==============================================================================
GridlockScenario flyby_uturn();

//==============================================================================
GridlockScenario fourway_standoff();

//==============================================================================
GridlockScenario threeway_standoff_with_redundant_leg();

//==============================================================================
GridlockScenario threeway_standoff_with_additional_conflict();

//==============================================================================
GridlockScenario crisscrossing_paths();

#endif // TEST__UNIT__BLOCKADE__UTILS_BLOCKADE_SCENARIOS_HPP
