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

#include <src/rmf_traffic/blockade/conflicts.hpp>

#include <rmf_utils/catch.hpp>

//==============================================================================
template <std::size_t N>
std::vector<rmf_traffic::blockade::Writer::Checkpoint> make_path(
    const std::array<Eigen::Vector2d, N>& points)
{
  using namespace rmf_traffic::blockade;

  const auto now = std::chrono::steady_clock::now();

  std::vector<Writer::Checkpoint> path;
  path.reserve(N);
  for (const auto& p : points)
    path.push_back(Writer::Checkpoint{p, now, true});

  return path;
}

//==============================================================================
SCENARIO("Compute conflict brackets")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double half_r = radius/2.0;
  const double max_angle = 1.0*M_PI/180.0;

  GIVEN("Converging paths")
  {
    std::array<Eigen::Vector2d, 6> A;
    A[0] = { 0,  0};
    A[1] = { 5,  0};
    A[2] = { 8,  5};
    A[3] = {16,  5};
    A[4] = {20,  0};
    A[5] = {25,  0};
    const auto path_A = make_path(A);

    const double b_height = 5 + half_r;
    std::array<Eigen::Vector2d, 6> B;
    for (std::size_t i=0; i < B.size(); ++i)
      B[i] = {5.0*i, b_height};

    WHEN("Forward")
    {
      const auto path_B = make_path(B);

      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 1);
      const auto& bracket = brackets.front();

      CHECK(bracket.A.start == 1);
      CHECK_FALSE(bracket.A.include_start);
      CHECK(bracket.A.finish == 2);
      CHECK(bracket.A.include_finish);

      CHECK(bracket.B.start == 1);
      CHECK_FALSE(bracket.B.include_start);
      CHECK(bracket.B.finish == 2);
      CHECK_FALSE(bracket.B.include_finish);
    }

    WHEN("Reverse")
    {
      std::reverse(B.begin(), B.end());
      const auto path_B = make_path(B);

      auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 1);
      const auto& bracket = brackets.front();

      CHECK(bracket.A.start == 1);
      CHECK_FALSE(bracket.A.include_start);
      CHECK(bracket.A.finish == 4);
      CHECK_FALSE(bracket.A.include_finish);

      CHECK(bracket.B.start == 1);
      CHECK_FALSE(bracket.B.include_start);
      CHECK(bracket.B.finish == 4);
      CHECK_FALSE(bracket.B.include_finish);
    }
  }

  GIVEN("Multipass paths")
  {
    std::array<Eigen::Vector2d, 9> A;
    A[0] = { 0, 10};
    A[1] = { 0,  7};
    A[2] = { 0,  5};
    A[3] = { 5,  5};
    A[4] = { 5,  0};
    A[5] = {10,  0};
    A[6] = {10,  5};
    A[7] = {15,  5};
    A[8] = {15, 10};
    auto path_A = make_path(A);

    std::array<Eigen::Vector2d, 5> B;
    B[0] = { 15, 10};
    B[1] = { 15,  5};
    B[2] = {7.5,  5};
    B[3] = {  0,  5};
    B[4] = {  0,  0};
    auto path_B = make_path(B);

    WHEN("Both can hold between passes")
    {
      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 2);

      // (1, 3] x (2, 3]
      CHECK(brackets[0].A.start == 1);
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK(brackets[0].B.start == 2);
      CHECK_FALSE(brackets[0].B.include_start);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // (5, 8] x [0, 2)
      CHECK(brackets[1].A.start == 5);
      CHECK_FALSE(brackets[1].A.include_start);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);
    }

    WHEN("Agent A cannot hold between passes")
    {
      path_A[4].can_hold = false;
      path_A[5].can_hold = false;

      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 2);

      // (1, 3] x (2, 3]
      CHECK(brackets[0].A.start == 1);
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK(brackets[0].B.start == 2);
      CHECK_FALSE(brackets[0].B.include_start);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // [4, 8] x [0, 2)
      CHECK(brackets[1].A.start == 4);
      CHECK(brackets[1].A.include_start);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);
    }

    WHEN("Agent B cannot hold between passes")
    {
      path_B[2].can_hold = false;

      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 2);

      // (1, 3] x [2, 3]
      CHECK(brackets[0].A.start == 1);
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK(brackets[0].B.start == 2);
      CHECK(brackets[0].B.include_start);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // (5, 8] x [0, 2)
      CHECK(brackets[1].A.start == 5);
      CHECK_FALSE(brackets[1].A.include_start);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);
    }
  }
}
