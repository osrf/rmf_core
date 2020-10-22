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

#include "utils_blockade_simulation.hpp"

#include <iostream>

//==============================================================================
using Checkpoint = rmf_traffic::blockade::Writer::Checkpoint;

//==============================================================================
template <std::size_t N>
std::vector<Checkpoint> make_path(const std::array<Eigen::Vector2d, N>& points)
{
  using namespace rmf_traffic::blockade;

  const auto now = std::chrono::steady_clock::now();

  std::vector<Writer::Checkpoint> path;
  path.reserve(N);
  for (const auto& p : points)
    path.push_back(Writer::Checkpoint{p, "test_map", now, true});

  return path;
}

//==============================================================================
rmf_traffic::blockade::Blockers make_ShouldGo_constraints(
  const std::vector<std::vector<Checkpoint>>& paths,
  const double radius,
  const double max_angle)
{
  using namespace rmf_traffic::blockade;

  PeerToPeerBlockers peer_blockers;
  for (std::size_t i=0; i < paths.size()-1; ++i)
  {
    for (std::size_t j=i+1; j < paths.size(); ++j)
    {
      const auto zero_order_blockers =
          compute_blockers(
            compute_conflict_brackets(
              paths[i], radius, paths[j], radius, max_angle),
            i, paths[i].size(), j, paths[j].size());

      peer_blockers[i][j] = zero_order_blockers.at(0);
      peer_blockers[j][i] = zero_order_blockers.at(1);
    }
  }

  return compute_final_ShouldGo_constraints(peer_blockers);
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

      // (1, 2] x (1, 2)
      CHECK_FALSE(bracket.A.include_start);
      CHECK(bracket.A.start == 1);
      CHECK(bracket.A.finish == 2);
      CHECK(bracket.A.include_finish);
      CHECK_FALSE(bracket.B.include_start);
      CHECK(bracket.B.start == 1);
      CHECK(bracket.B.finish == 2);
      CHECK_FALSE(bracket.B.include_finish);

      auto zero_order_blockers = compute_blockers(
            brackets, 0, path_A.size(), 1, path_B.size());

      PeerToPeerBlockers peer_blockers;
      peer_blockers[0][1] = zero_order_blockers.at(0);
      peer_blockers[1][0] = zero_order_blockers.at(1);

      CHECK(peer_blockers[0][1].size() == 1);
      CHECK(peer_blockers[1][0].size() == 1);

      const Blockers should_go =
          compute_final_ShouldGo_constraints(peer_blockers);

      simulate_all_sequences(should_go, nullptr, {5, 5});
    }

    WHEN("Reverse")
    {
      std::reverse(B.begin(), B.end());
      const auto path_B = make_path(B);

      auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 1);
      const auto& bracket = brackets.front();

      // (1, 4) x (1, 4)
      CHECK_FALSE(bracket.A.include_start);
      CHECK(bracket.A.start == 1);
      CHECK(bracket.A.finish == 4);
      CHECK_FALSE(bracket.A.include_finish);
      CHECK_FALSE(bracket.B.include_start);
      CHECK(bracket.B.start == 1);
      CHECK(bracket.B.finish == 4);
      CHECK_FALSE(bracket.B.include_finish);

      auto zero_order_blockers = compute_blockers(
            brackets, 0, path_A.size(), 1, path_B.size());

      PeerToPeerBlockers peer_blockers;
      peer_blockers[0][1] = zero_order_blockers.at(0);
      peer_blockers[1][0] = zero_order_blockers.at(1);

      CHECK(peer_blockers[0][1].size() == 1);
      CHECK(peer_blockers[1][0].size() == 1);

      const Blockers should_go =
          compute_final_ShouldGo_constraints(peer_blockers);

      simulate_all_sequences(should_go, nullptr, {5, 5});
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
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.start == 1);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK_FALSE(brackets[0].B.include_start);
      CHECK(brackets[0].B.start == 2);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // (5, 8] x [0, 2)
      CHECK_FALSE(brackets[1].A.include_start);
      CHECK(brackets[1].A.start == 5);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);

      auto zero_order_blockers = compute_blockers(
            brackets, 0, path_A.size(), 1, path_B.size());

      PeerToPeerBlockers peer_blockers;
      peer_blockers[0][1] = zero_order_blockers.at(0);
      peer_blockers[1][0] = zero_order_blockers.at(1);

      CHECK(peer_blockers[0][1].size() == 2);
      CHECK(peer_blockers[1][0].size() == 2);

      const Blockers should_go =
          compute_final_ShouldGo_constraints(peer_blockers);

      simulate_all_sequences(should_go, nullptr, {8, 4});
    }

    WHEN("Agent A cannot hold between passes")
    {
      path_A[4].can_hold = false;
      path_A[5].can_hold = false;

      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 2);

      // (1, 3] x (2, 3]
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.start == 1);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK_FALSE(brackets[0].B.include_start);
      CHECK(brackets[0].B.start == 2);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // [4, 8] x [0, 2)
      CHECK(brackets[1].A.include_start);
      CHECK(brackets[1].A.start == 4);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);

      auto zero_order_blockers = compute_blockers(
            brackets, 0, path_A.size(), 1, path_B.size());

      PeerToPeerBlockers peer_blockers;
      peer_blockers[0][1] = zero_order_blockers.at(0);
      peer_blockers[1][0] = zero_order_blockers.at(1);

      CHECK(peer_blockers[0][1].size() == 2);
      CHECK(peer_blockers[1][0].size() == 2);

      const Blockers should_go =
          compute_final_ShouldGo_constraints(peer_blockers);

      simulate_all_sequences(should_go, nullptr, {8, 4});
    }

    WHEN("Agent B cannot hold between passes")
    {
      path_B[2].can_hold = false;

      const auto brackets = compute_conflict_brackets(
            path_A, radius, path_B, radius, max_angle);

      REQUIRE(brackets.size() == 2);

      // (1, 3] x [2, 3]
      CHECK_FALSE(brackets[0].A.include_start);
      CHECK(brackets[0].A.start == 1);
      CHECK(brackets[0].A.finish == 3);
      CHECK(brackets[0].A.include_finish);
      CHECK(brackets[0].B.include_start);
      CHECK(brackets[0].B.start == 2);
      CHECK(brackets[0].B.finish == 3);
      CHECK(brackets[0].B.include_finish);

      // (5, 8] x [0, 2)
      CHECK_FALSE(brackets[1].A.include_start);
      CHECK(brackets[1].A.start == 5);
      CHECK(brackets[1].A.finish == 8);
      CHECK(brackets[1].A.include_finish);
      CHECK(brackets[1].B.include_start);
      CHECK(brackets[1].B.start == 0);
      CHECK(brackets[1].B.finish == 2);
      CHECK_FALSE(brackets[1].B.include_finish);

      const auto zero_order_blockers = compute_blockers(
            brackets, 0, path_A.size(), 1, path_B.size());

      PeerToPeerBlockers peer_blockers;
      peer_blockers[0][1] = zero_order_blockers.at(0);
      peer_blockers[1][0] = zero_order_blockers.at(1);

      CHECK(peer_blockers[0][1].size() == 2);
      CHECK(peer_blockers[1][0].size() == 2);

      const Blockers should_go =
          compute_final_ShouldGo_constraints(peer_blockers);

      simulate_all_sequences(should_go, nullptr, {8, 4});
    }
  }

  GIVEN("Floor changing pass-over")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 6> A;
    A[0] = {0, 0};
    A[1] = {2, 0};
    A[2] = {2, 0};
    A[3] = {6, 0};
    A[4] = {6, 0};
    A[5] = {8, 0};
    auto path_A = make_path(A);
    path_A[2].map_name = "test_map_2";
    path_A[3].map_name = "test_map_3";
    paths.emplace_back(std::move(path_A));

    std::array<Eigen::Vector2d, 5> B;
    B[0] = {8, 0};
    B[1] = {6, 0};
    B[2] = {4, 0};
    B[3] = {2, 0};
    B[4] = {0, 0};
    paths.push_back(make_path(B));

    const auto brackets = compute_conflict_brackets(
          paths[0], radius, paths[1], radius, max_angle);

    // [0, 1] x (2, 4]
    CHECK(brackets[0].A.include_start);
    CHECK(brackets[0].A.start == 0);
    CHECK(brackets[0].A.finish == 1);
    CHECK(brackets[0].A.include_finish);
    CHECK_FALSE(brackets[0].B.include_start);
    CHECK(brackets[0].B.start == 2);
    CHECK(brackets[0].B.finish == 4);
    CHECK(brackets[0].B.include_finish);

    // [4, 5] x [0, 2)
    CHECK(brackets[1].A.include_start);
    CHECK(brackets[1].A.start == 4);
    CHECK(brackets[1].A.finish == 5);
    CHECK(brackets[1].A.include_finish);
    CHECK(brackets[1].B.include_start);
    CHECK(brackets[1].B.start == 0);
    CHECK(brackets[1].B.finish == 2);
    CHECK_FALSE(brackets[1].B.include_finish);

    const auto zero_order_blockers = compute_blockers(
          brackets, 0, paths.at(0).size(), 1, paths.at(1).size());

    PeerToPeerBlockers peer_blockers;
    peer_blockers[0][1] = zero_order_blockers.at(0);
    peer_blockers[1][0] = zero_order_blockers.at(1);

    CHECK(peer_blockers[0][1].size() == 2);
    CHECK(peer_blockers[1][0].size() == 2);

    const Blockers should_go =
        compute_final_ShouldGo_constraints(peer_blockers);

    simulate_all_sequences(should_go, nullptr, {5, 4});
  }
}

//==============================================================================
SCENARIO("Simulate gridlocks")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double max_angle = 1.0*M_PI/180.0;

  GIVEN("Flyby U-turn")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 6> A;
    A[0] = { 0,  5};
    A[1] = { 5,  5};
    A[2] = {10,  5};
    A[3] = {10, 10};
    A[4] = { 5, 10};
    A[5] = { 0, 10};
    paths.push_back(make_path(A));

    std::array<Eigen::Vector2d, 4> B;
    B[0] = {20,  5};
    B[1] = {15,  5};
    B[2] = { 8,  5};
    B[3] = { 8,  0};
    paths.push_back(make_path(B));

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
    paths.emplace_back(std::move(path_C));

    simulate_all_sequences(
      make_ShouldGo_constraints(paths, radius, max_angle),
      nullptr,
      {5, 3, 5});
  }

  GIVEN("4-way standoff")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 3> A;
    A[0] = { 5,  0};
    A[1] = { 5,  5};
    A[2] = { 5, 15};
    paths.push_back(make_path(A));

    std::array<Eigen::Vector2d, 3> B;
    B[0] = { 0, 10};
    B[1] = { 5, 10};
    B[2] = {15, 10};
    paths.push_back(make_path(B));

    std::array<Eigen::Vector2d, 3> C;
    C[0] = {10, 15};
    C[1] = {10, 10};
    C[2] = {10,  0};
    paths.push_back(make_path(C));

    std::array<Eigen::Vector2d, 3> D;
    D[0] = {15,  5};
    D[1] = {10,  5};
    D[2] = { 0,  5};
    paths.push_back(make_path(D));

    simulate_all_sequences(
      make_ShouldGo_constraints(paths, radius, max_angle),
      nullptr,
      {2, 2, 2, 2});
  }

  GIVEN("3-way standoff with one redundant leg")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 3> A;
    A[0] = {0, 0};
    A[1] = {2, 2};
    A[2] = {6, 6};
    paths.push_back(make_path(A));

    std::array<Eigen::Vector2d, 3> B;
    B[0] = {2, 6};
    B[1] = {4, 4};
    B[2] = {8, 0};
    paths.push_back(make_path(B));

    std::array<Eigen::Vector2d, 3> C;
    C[0] = {1, 6};
    C[1] = {3, 3};
    C[2] = {7, 0};
    paths.push_back(make_path(C));

    std::array<Eigen::Vector2d, 3> D;
    D[0] = {8, 2};
    D[1] = {6, 2};
    D[2] = {0, 2};
    paths.push_back(make_path(D));

    simulate_all_sequences(
      make_ShouldGo_constraints(paths, radius, max_angle),
      nullptr,
      {2, 2, 2, 2});
  }

  GIVEN("Three-way standoff with an additional conflict")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 3> A;
    A[0] = {0, 0};
    A[1] = {2, 2};
    A[2] = {6, 6};
    paths.push_back(make_path(A));

    std::array<Eigen::Vector2d, 3> B;
    B[0] = {2, 6};
    B[1] = {4, 4};
    B[2] = {8, 0};
    paths.push_back(make_path(B));

    std::array<Eigen::Vector2d, 3> C;
    C[0] = {8, 2};
    C[1] = {6, 2};
    C[2] = {0, 2};
    paths.push_back(make_path(C));

    std::array<Eigen::Vector2d, 2> D;
    D[0] = {0, 4};
    D[1] = {0, 0};
    paths.push_back(make_path(D));

    simulate_all_sequences(
      make_ShouldGo_constraints(paths, radius, max_angle),
      nullptr,
      {2, 2, 2, 1});
  }

  GIVEN("Criss-crossing paths")
  {
    std::vector<std::vector<Checkpoint>> paths;

    std::array<Eigen::Vector2d, 6> A;
    A[0] = {4, 0};
    A[1] = {4, 2};
    A[2] = {8, 6};
    A[3] = {4, 6};
    A[4] = {8, 2};
    A[5] = {8, 0};
    paths.push_back(make_path(A));

    std::array<Eigen::Vector2d, 3> B;
    B[0] = {10, 4};
    B[1] = { 6, 4};
    B[2] = { 0, 4};
    paths.push_back(make_path(B));

    std::array<Eigen::Vector2d, 4> C;
    C[0] = { 2, 2};
    C[1] = { 2, 4};
    C[2] = { 2, 6};
    C[3] = {10, 6};
    auto path_C = make_path(C);
    path_C[2].can_hold = false;
    paths.emplace_back(std::move(path_C));

    simulate_all_sequences(
      make_ShouldGo_constraints(paths, radius, max_angle),
      nullptr,
      {5, 2, 3});
  }
}
