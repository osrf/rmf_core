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

#include "utils_blockade_scenarios.hpp"

//==============================================================================
template <typename T>
std::vector<std::size_t> get_goals(
    const std::vector<std::vector<T>>& paths)
{
  std::vector<std::size_t> goals;
  for (const auto& p : paths)
    goals.push_back(p.size() - 1);

  return goals;
}

//==============================================================================
GridlockScenario flyby_uturn()
{
  using namespace rmf_traffic::blockade;

  GridlockScenario scenario;

  std::array<Eigen::Vector2d, 6> A;
  A[0] = { 0,  5};
  A[1] = { 5,  5};
  A[2] = {10,  5};
  A[3] = {10, 10};
  A[4] = { 5, 10};
  A[5] = { 0, 10};
  scenario.paths.push_back(make_path(A));

  std::array<Eigen::Vector2d, 4> B;
  B[0] = {20,  5};
  B[1] = {15,  5};
  B[2] = { 8,  5};
  B[3] = { 8,  0};
  scenario.paths.push_back(make_path(B));

  std::array<Eigen::Vector2d, 6> C;
  C[0] = { 8, 20};
  C[1] = { 8, 15};
  C[2] = { 8, 10};
  C[3] = {10, 10};
  C[4] = {15, 10};
  C[5] = {15,  0};
  auto path_C = make_path(C);
  path_C.at(2).can_hold = false;
  path_C.at(4).can_hold = false;
  scenario.paths.emplace_back(std::move(path_C));

  scenario.goals = get_goals(scenario.paths);

  return scenario;
}

//==============================================================================
GridlockScenario fourway_standoff()
{
  GridlockScenario scenario;

  std::array<Eigen::Vector2d, 3> A;
  A[0] = { 5,  0};
  A[1] = { 5,  5};
  A[2] = { 5, 15};
  scenario.paths.push_back(make_path(A));

  std::array<Eigen::Vector2d, 3> B;
  B[0] = { 0, 10};
  B[1] = { 5, 10};
  B[2] = {15, 10};
  scenario.paths.push_back(make_path(B));

  std::array<Eigen::Vector2d, 3> C;
  C[0] = {10, 15};
  C[1] = {10, 10};
  C[2] = {10,  0};
  scenario.paths.push_back(make_path(C));

  std::array<Eigen::Vector2d, 3> D;
  D[0] = {15,  5};
  D[1] = {10,  5};
  D[2] = { 0,  5};
  scenario.paths.push_back(make_path(D));

  scenario.goals = get_goals(scenario.paths);

  return scenario;
}

//==============================================================================
GridlockScenario threeway_standoff_with_redundant_leg()
{
  GridlockScenario scenario;

  std::array<Eigen::Vector2d, 3> A;
  A[0] = {0, 0};
  A[1] = {2, 2};
  A[2] = {6, 6};
  scenario.paths.push_back(make_path(A));

  std::array<Eigen::Vector2d, 3> B;
  B[0] = {2, 6};
  B[1] = {4, 4};
  B[2] = {8, 0};
  scenario.paths.push_back(make_path(B));

  std::array<Eigen::Vector2d, 3> C;
  C[0] = {1, 6};
  C[1] = {3, 3};
  C[2] = {7, 0};
  scenario.paths.push_back(make_path(C));

  std::array<Eigen::Vector2d, 3> D;
  D[0] = {8, 2};
  D[1] = {6, 2};
  D[2] = {0, 2};
  scenario.paths.push_back(make_path(D));

  scenario.goals = get_goals(scenario.paths);

  return scenario;
}

//==============================================================================
GridlockScenario threeway_standoff_with_additional_conflict()
{
  GridlockScenario scenario;

  std::array<Eigen::Vector2d, 3> A;
  A[0] = {0, 0};
  A[1] = {2, 2};
  A[2] = {6, 6};
  scenario.paths.push_back(make_path(A));

  std::array<Eigen::Vector2d, 3> B;
  B[0] = {2, 6};
  B[1] = {4, 4};
  B[2] = {8, 0};
  scenario.paths.push_back(make_path(B));

  std::array<Eigen::Vector2d, 3> C;
  C[0] = {8, 2};
  C[1] = {6, 2};
  C[2] = {0, 2};
  scenario.paths.push_back(make_path(C));

  std::array<Eigen::Vector2d, 2> D;
  D[0] = {0, 4};
  D[1] = {0, 0};
  scenario.paths.push_back(make_path(D));

  scenario.goals = get_goals(scenario.paths);

  return scenario;
}

//==============================================================================
GridlockScenario crisscrossing_paths()
{
  GridlockScenario scenario;

  std::array<Eigen::Vector2d, 6> A;
  A[0] = {4, 0};
  A[1] = {4, 2};
  A[2] = {8, 6};
  A[3] = {4, 6};
  A[4] = {8, 2};
  A[5] = {8, 0};
  scenario.paths.push_back(make_path(A));

  std::array<Eigen::Vector2d, 3> B;
  B[0] = {10, 4};
  B[1] = { 6, 4};
  B[2] = { 0, 4};
  scenario.paths.push_back(make_path(B));

  std::array<Eigen::Vector2d, 4> C;
  C[0] = { 2, 2};
  C[1] = { 2, 4};
  C[2] = { 2, 6};
  C[3] = {10, 6};
  auto path_C = make_path(C);
  path_C[2].can_hold = false;
  scenario.paths.emplace_back(std::move(path_C));

  scenario.goals = get_goals(scenario.paths);

  return scenario;
}
