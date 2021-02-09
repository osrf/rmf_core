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

#include <rmf_task/Evaluator.hpp>

namespace rmf_task {

//==============================================================================
std::optional<std::size_t> LeastFleetDiffCostEvaluator::choose(
  const Submissions& submissions) const
{
  if (submissions.empty())
    return std::nullopt;
  
  auto winner_it = submissions.begin();
  float winner_cost_diff = winner_it->new_cost - winner_it->prev_cost;
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    float nominee_cost_diff = nominee_it->new_cost - nominee_it->prev_cost;
    if (nominee_cost_diff < winner_cost_diff)
    {
      winner_it = nominee_it;
      winner_cost_diff = nominee_cost_diff;
    }
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::optional<std::size_t> LeastFleetCostEvaluator::choose(
  const Submissions& submissions) const
{
  if (submissions.empty())
    return std::nullopt;

  auto winner_it = submissions.begin();
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->new_cost < winner_it->new_cost)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

//==============================================================================
std::optional<std::size_t> QuickestFinishEvaluator::choose(
  const Submissions& submissions) const
{
  if (submissions.empty())
    return std::nullopt;

  auto winner_it = submissions.begin();
  for (auto nominee_it = ++submissions.begin();
    nominee_it != submissions.end(); ++nominee_it)
  {
    if (nominee_it->finish_time < winner_it->finish_time)
      winner_it = nominee_it;
  }
  return std::distance(submissions.begin(), winner_it);
}

} // namespace rmf_task
