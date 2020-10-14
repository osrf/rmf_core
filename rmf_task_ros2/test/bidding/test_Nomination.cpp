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

#include <rmf_task_ros2/bidding/Nomination.hpp>
#include <rmf_utils/catch.hpp>

//==============================================================================
using namespace rmf_task_ros2::bidding;

auto now = std::chrono::steady_clock::now();

Submission submission1{
  "fleet1", "", 2.3, 3.4, rmf_traffic::time::apply_offset(now, 5)
};
Submission submission2{
  "fleet2", "", 3.5, 3.6, rmf_traffic::time::apply_offset(now, 5.5)
};
Submission submission3{
  "fleet3", "", 0.0, 1.4, rmf_traffic::time::apply_offset(now, 3)
};
Submission submission4{
  "fleet4", "", 5.0, 5.4, rmf_traffic::time::apply_offset(now, 4)
};
Submission submission5{
  "fleet5", "", 0.5, 0.8, rmf_traffic::time::apply_offset(now, 3.5)
};

//==============================================================================
SCENARIO("Quickest Finish Time Evaluator", "[MinFinishTime]")
{
  WHEN("0 submission") 
  {
    std::vector<Submission> submissions{};
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(QuickestFinishEvaluator());
    REQUIRE(!winner); // no winner
  }
  WHEN("5 submissions") 
  {
    std::vector<Submission> submissions{
      submission1, submission2, submission3, submission4, submission5 };
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(QuickestFinishEvaluator());
    REQUIRE(winner->fleet_name == "fleet3"); // fastest agent
  }
}

//==============================================================================
SCENARIO("Least Diff Cost Evaluator", "[MinDiffCost]")
{
  WHEN("0 submission") 
  {
    std::vector<Submission> submissions{};
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(LeastFleetDiffCostEvaluator());
    REQUIRE(!winner); // no winner
  }
  WHEN("5 submissions") 
  {
    std::vector<Submission> submissions{
      submission1, submission2, submission3, submission4, submission5 };
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(LeastFleetDiffCostEvaluator());
    REQUIRE(winner->fleet_name == "fleet2"); // least diff cost agent
  }
}

//==============================================================================
SCENARIO("Least Fleet Cost Evaluator", "[MinNewCost]")
{
  WHEN("0 submission") 
  {
    std::vector<Submission> submissions{};
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(LeastFleetCostEvaluator());
    REQUIRE(!winner); // no winner
  }
  WHEN("5 submissions") 
  {
    std::vector<Submission> submissions{
      submission1, submission2, submission3, submission4, submission5 };
    Nomination task_nomination(submissions);
    auto winner = task_nomination.evaluate(LeastFleetCostEvaluator());
    REQUIRE(winner->fleet_name == "fleet5"); // least new cost agent
  }
}
