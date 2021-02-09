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

#include <rmf_task/Evaluator.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_task {

//==============================================================================
auto now = std::chrono::steady_clock::now();

Evaluator::Submission submission1{
  "fleet1", "", 2.3, 3.4, rmf_traffic::time::apply_offset(now, 5)
};
Evaluator::Submission submission2{
  "fleet2", "", 3.5, 3.6, rmf_traffic::time::apply_offset(now, 5.5)
};
Evaluator::Submission submission3{
  "fleet3", "", 0.0, 1.4, rmf_traffic::time::apply_offset(now, 3)
};
Evaluator::Submission submission4{
  "fleet4", "", 5.0, 5.4, rmf_traffic::time::apply_offset(now, 4)
};
Evaluator::Submission submission5{
  "fleet5", "", 0.5, 0.8, rmf_traffic::time::apply_offset(now, 3.5)
};

//==============================================================================
SCENARIO("Winner from Evaluator", "[Evaluator]")
{
  WHEN("Least Diff Cost Evaluator")
  {
    auto evaluator = std::make_shared<LeastFleetDiffCostEvaluator>();

    AND_WHEN("0 submissions")
    {
      std::vector<Evaluator::Submission> submissions{};
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice == std::nullopt); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Evaluator::Submission> submissions{
        submission1, submission2, submission3, submission4, submission5 };
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice != std::nullopt);
      REQUIRE(submissions[*choice].fleet_name == "fleet2"); // least diff cost agent
    }
  }

  WHEN("Least Fleet Cost Evaluator")
  {
    auto evaluator = std::make_shared<LeastFleetCostEvaluator>();

    AND_WHEN("0 submissions")
    {
      std::vector<Evaluator::Submission> submissions{};
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice == std::nullopt); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Evaluator::Submission> submissions{
        submission1, submission2, submission3, submission4, submission5 };
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice != std::nullopt);
      REQUIRE(submissions[*choice].fleet_name == "fleet5"); // least cost agent
    }
  }

  WHEN("Quickest Finish Time Evaluator")
  {
    auto evaluator = std::make_shared<QuickestFinishEvaluator>();

    AND_WHEN("0 submissions")
    {
      std::vector<Evaluator::Submission> submissions{};
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice == std::nullopt); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Evaluator::Submission> submissions{
        submission1, submission2, submission3, submission4, submission5 };
      auto choice = evaluator->choose(submissions);
      REQUIRE(choice != std::nullopt);
      REQUIRE(submissions[*choice].fleet_name == "fleet3"); // quickest agent
    }
  }
}

} // namespace rmf_task
