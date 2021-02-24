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

#include "InvariantHeuristicQueue.hpp"

#include <cassert>
#include <algorithm>

namespace rmf_task {

//==============================================================================
InvariantHeuristicQueue::InvariantHeuristicQueue(
  std::vector<double> initial_values)
{
  assert(!initial_values.empty());
  std::sort(initial_values.begin(), initial_values.end());

  for (const auto value : initial_values)
    _stacks.push_back({{0, value}});
}

//==============================================================================
void InvariantHeuristicQueue::add(
  const double earliest_start_time, const double earliest_finish_time)
{
  double prev_end_value = _stacks[0].back().end;
  double new_end_value = prev_end_value + (earliest_finish_time - earliest_start_time);
  _stacks[0].push_back({earliest_start_time, new_end_value});

  // Find the largest stack that is still smaller than the current front
  const auto next_it = _stacks.begin() + 1;
  auto end_it = next_it;
  for (; end_it != _stacks.end(); ++end_it)
  {
    if (new_end_value <= end_it->back().end)
      break;
  }

  if (next_it != end_it)
  {
    // Rotate the vector elements to move the front stack to its new place
    // in the order
    std::rotate(_stacks.begin(), next_it, end_it);
  }
}

//==============================================================================
double InvariantHeuristicQueue::compute_cost() const
{
  double total_cost = 0.0;
  for (const auto& stack : _stacks)
  {
    // NOTE: We start iterating from i=1 because i=0 represents a component of
    // the cost that is already accounted for by g(n) and the variant
    // component of h(n)
    for (std::size_t i = 1; i < stack.size(); ++i)
    {
      // Set lower bound of 0 to account for case where optimistically calculated
      // end time is smaller than earliest start time
      total_cost += std::max(0.0, (stack[i].end - stack[i].start));
    }
  }

  return total_cost;
}

} // namespace rmf_task
