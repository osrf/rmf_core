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
#include "utils_blockade_scenarios.hpp"

#include <iostream>

//==============================================================================
using Checkpoint = rmf_traffic::blockade::Writer::Checkpoint;

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
            compute_brackets(
              paths[i], radius, paths[j], radius, max_angle).conflicts,
            i, paths[i].size(), j, paths[j].size());

      peer_blockers[i][j] = zero_order_blockers.at(0);
      peer_blockers[j][i] = zero_order_blockers.at(1);
    }
  }

  return compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;
}

//==============================================================================
void check_for_expected_brackets(
    const std::vector<rmf_traffic::blockade::BracketPair>& expectations,
    const std::vector<rmf_traffic::blockade::AlignedBracketPair>& actual)
{
  for (const auto& expected : expectations)
  {
    bool found_expected = false;
    for (const auto& a : actual)
    {
      if (a == expected)
      {
        found_expected = true;
        break;
      }
    }

    CHECK(found_expected);
    if (!found_expected)
    {
      std::cout << "Could not find: "
                << rmf_traffic::blockade::AlignedBracketPair{expected}
                << std::endl;
    }
  }

  CHECK(expectations.size() == actual.size());
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

      const auto brackets = compute_brackets(
            path_A, radius, path_B, radius, max_angle).conflicts;

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
          compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

      simulate_all_sequences(should_go, nullptr, {5, 5});
    }

    WHEN("Reverse")
    {
      std::reverse(B.begin(), B.end());
      const auto path_B = make_path(B);

      auto brackets = compute_brackets(
            path_A, radius, path_B, radius, max_angle).conflicts;

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
          compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

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
      const auto brackets = compute_brackets(
            path_A, radius, path_B, radius, max_angle).conflicts;

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
          compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

      simulate_all_sequences(should_go, nullptr, {8, 4});
    }

    WHEN("Agent A cannot hold between passes")
    {
      path_A[4].can_hold = false;
      path_A[5].can_hold = false;

      const auto brackets = compute_brackets(
            path_A, radius, path_B, radius, max_angle).conflicts;

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
          compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

      simulate_all_sequences(should_go, nullptr, {8, 4});
    }

    WHEN("Agent B cannot hold between passes")
    {
      path_B[2].can_hold = false;

      const auto brackets = compute_brackets(
            path_A, radius, path_B, radius, max_angle).conflicts;

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
          compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

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

    const auto brackets = compute_brackets(
          paths[0], radius, paths[1], radius, max_angle).conflicts;

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
        compute_final_ShouldGo_constraints(peer_blockers, {}).should_go;

    simulate_all_sequences(should_go, nullptr, {5, 4});
  }
}

//==============================================================================
SCENARIO("Compute alignment brackets")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double max_angle = 1.0*M_PI/180.0;

  std::array<Eigen::Vector2d, 6> B;
  B[0] = { 0, -5};
  B[1] = { 0, 0};
  B[2] = { 5, 0};
  B[3] = {10, 0};
  B[4] = {15, 0};
  B[5] = {20, 0};
  const auto path_B = make_path(B);

  GIVEN("u merge")
  {
    std::array<Eigen::Vector2d, 5> A;
    A[0] = {5, 5};
    A[1] = {5, 0};
    A[2] = {10, 0};
    A[3] = {15, 0};
    A[4] = {15, 5};
    const auto path_A = make_path(A);

    const auto brackets = compute_brackets(
          path_A, radius, path_B, radius, max_angle).alignments;

    REQUIRE(brackets.size() == 1);
    const auto& whole_bracket = brackets.front().whole_bracket;

    // [A1, A3] | (B1, B5)
    CHECK(whole_bracket.A.include_start);
    CHECK(whole_bracket.A.start == 1);
    CHECK(whole_bracket.A.finish == 3);
    CHECK(whole_bracket.A.include_finish);
    CHECK_FALSE(whole_bracket.B.include_start);
    CHECK(whole_bracket.B.start == 1);
    CHECK(whole_bracket.B.finish == 5);
    CHECK_FALSE(whole_bracket.B.include_finish);

    std::vector<BracketPair> expectations =
    {
      // [1, 2) | (1, 2]
      {
        {1, 2, true, false},
        {1, 2, false, true}
      },

      // [1, 2] | [2, 3]
      {
        {1, 2, true, true},
        {2, 3, true, true}
      },

      // (1, 2] | [3, 4)
      {
        {1, 2, false, true},
        {3, 4, true, false}
      },

      // [2, 3) | (2, 3]
      {
        {2, 3, true, false},
        {2, 3, false, true}
      },

      // [2, 3] | [3, 4]
      {
        {2, 3, true, true},
        {3, 4, true, true}
      },

      // (2, 3] | [4, 5)
      {
        {2, 3, false, true},
        {4, 5, true, false}
      },
    };

    check_for_expected_brackets(expectations, brackets.front().segments);
  }

  GIVEN("unu merge")
  {
    std::array<Eigen::Vector2d, 8> A;
    A[0] = {0, 5};
    A[1] = {0, 0};
    A[2] = {5, 0};
    A[3] = {5, 5};
    A[4] = {10, 5};
    A[5] = {10, 0};
    A[6] = {15, 0};
    A[7] = {15, 5};
    const auto path_A = make_path(A);

    const auto brackets = compute_brackets(
          path_A, radius, path_B, radius, max_angle).alignments;

    REQUIRE(brackets.size() == 2);

    {
      const auto& whole_bracket = brackets.front().whole_bracket;

      // [1, 2] | [1, 3)
      CHECK(whole_bracket.A.include_start);
      CHECK(whole_bracket.A.start == 1);
      CHECK(whole_bracket.A.finish == 2);
      CHECK(whole_bracket.A.include_finish);
      CHECK(whole_bracket.B.include_start);
      CHECK(whole_bracket.B.start == 1);
      CHECK(whole_bracket.B.finish == 3);
      CHECK_FALSE(whole_bracket.B.include_finish);

      std::vector<BracketPair> expectations =
      {
        // [1, 2] | [1, 2]
        {
          {1, 2, true, true},
          {1, 2, true, true}
        },

        // (1, 2] | [2, 3)
        {
          {1, 2, false, true},
          {2, 3, true, false}
        }
      };

      check_for_expected_brackets(expectations, brackets.front().segments);
    }

    {
      const auto& whole_bracket = brackets.back().whole_bracket;

      // [5, 6] | (2, 5)
      CHECK(whole_bracket.A.include_start);
      CHECK(whole_bracket.A.start == 5);
      CHECK(whole_bracket.A.finish == 6);
      CHECK(whole_bracket.A.include_finish);
      CHECK_FALSE(whole_bracket.B.include_start);
      CHECK(whole_bracket.B.start == 2);
      CHECK(whole_bracket.B.finish == 5);
      CHECK_FALSE(whole_bracket.B.include_finish);

      std::vector<BracketPair> expectations =
      {
        // [5, 6) | (2, 3]
        {
          {5, 6, true, false},
          {2, 3, false, true}
        },

        // [5, 6] | [3, 4]
        {
          {5, 6, true, true},
          {3, 4, true, true}
        },

        // (5, 6] | [4, 5)
        {
          {5, 6, false, true},
          {4, 5, true, false}
        }
      };

      check_for_expected_brackets(expectations, brackets.back().segments);
    }
  }

  GIVEN("uu merge")
  {
    std::array<Eigen::Vector2d, 9> A;
    A[0] = {0, 5};
    A[1] = {0, 0};
    A[2] = {5, 0};
    A[3] = {10, 0};
    A[4] = {5, 5};
    A[5] = {15, 5};
    A[6] = {10, 0};
    A[7] = {15, 0};
    A[8] = {20, 5};
    const auto path_A = make_path(A);

    const auto brackets = compute_brackets(
          path_A, radius, path_B, radius, max_angle).alignments;

    REQUIRE(brackets.size() == 2);

    {
      const auto& whole_bracket = brackets.front().whole_bracket;

      // [1, 3] | [1, 4)
      CHECK(whole_bracket.A.include_start);
      CHECK(whole_bracket.A.start == 1);
      CHECK(whole_bracket.A.finish == 3);
      CHECK(whole_bracket.A.include_finish);
      CHECK(whole_bracket.B.include_start);
      CHECK(whole_bracket.B.start == 1);
      CHECK(whole_bracket.B.finish == 4);
      CHECK_FALSE(whole_bracket.B.include_finish);

      std::vector<BracketPair> expectations =
      {
        // [1, 2] | [1, 2]
        {
          {1, 2, true, true},
          {1, 2, true, true}
        },

        // (1, 2] | [2, 3)
        {
          {1, 2, false, true},
          {2, 3, true, false}
        },

        // [2, 3) | (1, 2]
        {
          {2, 3, true, false},
          {1, 2, false, true}
        },

        // [2, 3] | [2, 3]
        {
          {2, 3, true, true},
          {2, 3, true, true}
        },

        // (2, 3] | [3, 4)
        {
          {2, 3, false, true},
          {3, 4, true, false}
        }
      };

      check_for_expected_brackets(expectations, brackets.front().segments);
    }

    {
      const auto& whole_bracket = brackets.back().whole_bracket;

      // [6, 7] | (2, 5)
      CHECK(whole_bracket.A.include_start);
      CHECK(whole_bracket.A.start == 6);
      CHECK(whole_bracket.A.finish == 7);
      CHECK(whole_bracket.A.include_finish);
      CHECK_FALSE(whole_bracket.B.include_start);
      CHECK(whole_bracket.B.start == 2);
      CHECK(whole_bracket.B.finish == 5);
      CHECK_FALSE(whole_bracket.B.include_finish);

      std::vector<BracketPair> expectations =
      {
        // [6, 7) | (2, 3]
        {
          {6, 7, true, false},
          {2, 3, false, true}
        },

        // [6, 7] | [3, 4]
        {
          {6, 7, true, true},
          {3, 4, true, true}
        },

        // (6, 7] | [4, 5)
        {
          {6, 7, false, true},
          {4, 5, true, false}
        }
      };

      check_for_expected_brackets(expectations, brackets.back().segments);
    }
  }
}

//==============================================================================
SCENARIO("Simulate gridlocks")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double max_angle = 1.0*M_PI/180.0;

  GridlockScenario scenario;

  GIVEN("Flyby U-turn")
  {
    scenario = flyby_uturn();
  }

  GIVEN("4-way standoff")
  {
    scenario = fourway_standoff();
  }

  GIVEN("3-way standoff with one redundant leg")
  {
    scenario = threeway_standoff_with_redundant_leg();
  }

  GIVEN("3-way standoff with an additional conflict")
  {
    scenario = threeway_standoff_with_additional_conflict();
  }

  GIVEN("Criss-crossing paths")
  {
    scenario = crisscrossing_paths();
  }

  simulate_all_sequences(
    make_ShouldGo_constraints(scenario.paths, radius, max_angle),
    nullptr,
    scenario.goals);
}
