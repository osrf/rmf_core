/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef RMF_TASK__EVALUATOR_HPP
#define RMF_TASK__EVALUATOR_HPP

#include <vector>
#include <string>
#include <optional>
#include <rmf_traffic/Time.hpp>

namespace rmf_task {

//==============================================================================
/// A pure abstract interface class for the auctioneer to choose the best
/// submission.
class Evaluator
{
public:

  struct Submission
  {
    std::string fleet_name;
    std::string robot_name;
    double prev_cost = 0.0;
    double new_cost = std::numeric_limits<double>::max();
    rmf_traffic::Time finish_time;
  };

  using Submissions = std::vector<Submission>;

  /// Given a list of submissions, choose the one that is the "best". It is
  /// up to the implementation of the Evaluator to decide how to rank.
  ///
  /// \return
  ///   index of the best submission
  virtual std::optional<std::size_t> choose(
    const Submissions& submissions) const = 0;

  virtual ~Evaluator() = default;
};

//==============================================================================
class LeastFleetDiffCostEvaluator : public Evaluator
{
public:
  std::optional<std::size_t> choose(
    const Submissions& submissions) const final;
};

//==============================================================================
class LeastFleetCostEvaluator : public Evaluator
{
public:
  std::optional<std::size_t> choose(
    const Submissions& submissions) const final;
};

//==============================================================================
class QuickestFinishEvaluator : public Evaluator
{
public:
  std::optional<std::size_t> choose(
    const Submissions& submissions) const final;
};


} // namespace rmf_task

#endif // RMF_TASK__EVALUATOR_HPP
