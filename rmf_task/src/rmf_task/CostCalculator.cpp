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

#include "CostCalculator.hpp"

namespace {

//==============================================================================
// Sorts and distributes tasks among agents based on the earliest finish time
// possible for each task (i.e. not accounting for any variant costs). Guaranteed
// to underestimate actual cost when the earliest start times for each task are
// similar (enforced by the segmentation_threshold).
class InvariantHeuristicQueue
{
public:

  InvariantHeuristicQueue(std::vector<double> initial_values)
  {
    assert(!initial_values.empty());
    std::sort(initial_values.begin(), initial_values.end());

    for (const auto value : initial_values)
      _stacks.push_back({{0, value}});
  }

  void add(const double earliest_start_time, const double earliest_finish_time)
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

  double compute_cost() const
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

private:
  struct element { double start; double end; };
  std::vector<std::vector<element>> _stacks;
};

} // anonymous namespace

namespace rmf_task {

//==============================================================================
class BinaryPriorityCostCalculator::Implementation
{
public:
  using TaskPlanner = rmf_task::agv::TaskPlanner;
  using Assignments = TaskPlanner::Assignments;

  double priority_penalty;

  double compute_g_assignment(const TaskPlanner::Assignment& assignment) const;

  double compute_g(const Assignments& assigned_tasks) const;

  double compute_g(const Node& node) const;

  double compute_h(const Node& node, const rmf_traffic::Time time_now) const;

  bool valid_assignment_priority(const Node& node) const;

  double compute_f(
    const Node& node,
    const rmf_traffic::Time time_now,
    bool check_priority) const;
};

//==============================================================================
auto BinaryPriorityCostCalculator::Implementation::compute_g_assignment(
  const TaskPlanner::Assignment& assignment) const -> double
{
  if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
    assignment.request()))
  {
    return 0.0; // Ignore charging tasks in cost
  }

  return rmf_traffic::time::to_seconds(assignment.state().finish_time()
    - assignment.request()->earliest_start_time());
}

//==============================================================================
auto BinaryPriorityCostCalculator::Implementation::compute_g(
  const Assignments& assigned_tasks) const -> double
{
  double cost = 0.0;
  for (const auto& agent : assigned_tasks)
  {
    for (const auto& assignment : agent)
    {
      cost += compute_g_assignment(assignment);
    }
  }

  return cost;
}

//==============================================================================
auto BinaryPriorityCostCalculator::Implementation::compute_g(
  const Node& node) const -> double
{
  double cost = 0.0;
  for (const auto& agent : node.assigned_tasks)
  {
    for (const auto& assignment : agent)
    {
      cost += compute_g_assignment(assignment.assignment);
    }
  }
  return cost;
}

//==============================================================================
auto BinaryPriorityCostCalculator::Implementation::compute_h(
  const Node& node, const rmf_traffic::Time time_now) const -> double
{
  std::vector<double> initial_queue_values(
    node.assigned_tasks.size(), std::numeric_limits<double>::infinity());

  // Determine the earliest possible time an agent can begin the invariant
  // portion of any of its next tasks
  for (const auto& u : node.unassigned_tasks)
  {
    const rmf_traffic::Time earliest_deployment_time =
        u.second.candidates.best_finish_time()
        - u.second.request->invariant_duration();
    const double earliest_deployment_time_s =
      rmf_traffic::time::to_seconds(
        earliest_deployment_time.time_since_epoch());

    const auto& range = u.second.candidates.best_candidates();
    for (auto it = range.begin; it != range.end; ++it)
    {
      const std::size_t candidate = it->second.candidate;
      if (earliest_deployment_time_s < initial_queue_values[candidate])
        initial_queue_values[candidate] = earliest_deployment_time_s;
    }
  }

  for (std::size_t i = 0; i < initial_queue_values.size(); ++i)
  {
    auto& value = initial_queue_values[i];
    if (std::isinf(value))
    {
      // Clear out any infinity placeholders. Those candidates simply don't have
      // any unassigned tasks that want to use it.
      const auto& assignments = node.assigned_tasks[i];
      if (assignments.empty())
        value = rmf_traffic::time::to_seconds(time_now.time_since_epoch());
      else
        value = rmf_traffic::time::to_seconds(
          assignments.back().assignment.state().finish_time().time_since_epoch());
    }
  }

  InvariantHeuristicQueue queue(std::move(initial_queue_values));
  // NOTE: It is crucial that we use the ordered set of unassigned_invariants
  // here. The InvariantHeuristicQueue expects the invariant costs to be passed
  // to it in order of smallest to largest. If that assumption is not met, then
  // the final cost that's calculated may be invalid.
  for (const auto& u : node.unassigned_invariants)
  {
    queue.add(u.earliest_start_time, u.earliest_finish_time);
  }
  return queue.compute_cost();
}

//==============================================================================
bool BinaryPriorityCostCalculator::Implementation::valid_assignment_priority(
  const Node& node) const
{
  // STEP 1: Checking for validity across agents
  const std::size_t num_agents = node.assigned_tasks.size();
  // Number of priority tasks assigned for each agent
  std::vector<std::size_t> priority_count;
  priority_count.resize(num_agents, 0);
  for (std::size_t i = 0; i < num_agents; ++i)
  {
    const auto& assignments = node.assigned_tasks[i];
    for (const auto& a : assignments)
    {
      if (a.assignment.request()->priority())
        priority_count[i] += 1;
    }
  }
  // Here we check if any of the agents is not assigned a priority task
  // while others are assigned more than one
  const std::size_t max_priority_count = *std::max_element(
    priority_count.begin(), priority_count.end());
  if (max_priority_count > 1)
  {
    for (const auto& c : priority_count)
    {
      if (c == 0)
        return false;
    }
  }

  // STEP 2: Checking for validity within assignments of an agent
  const auto& assignments = node.assigned_tasks;
  for (const auto& agent : assignments)
  {
    if (agent.empty())
      continue;
    
    auto it = agent.begin();
    // We update the iterator such that the first assignment is a non-charging task
    while (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
      it->assignment.request()))
    {
      ++it;
      if (it == agent.end())
        return true;
    } 

    auto prev_priority = it->assignment.request()->priority();
    ++it;
    for (; it != agent.end(); ++it)
    {
      if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
        it->assignment.request()))
        continue;
      auto curr_priority = it->assignment.request()->priority();
      if ((curr_priority != prev_priority) && (curr_priority))
        return false;

      prev_priority = curr_priority;
    }
  }

  return true; 
}

//==============================================================================
auto BinaryPriorityCostCalculator::Implementation::compute_f(
  const Node& node,
  const rmf_traffic::Time time_now,
  bool check_priority) const -> double
{
  const double g = compute_g(node);
  const double h = compute_h(node, time_now);

  if (check_priority)
  {
    if (!valid_assignment_priority(node))
      return priority_penalty * (g + h);
  }

  return g + h; 
}

//==============================================================================
BinaryPriorityCostCalculator::BinaryPriorityCostCalculator(
  double priority_penalty)
: _pimpl(rmf_utils::make_impl<Implementation>(
    Implementation{priority_penalty}))
{
  // Do nothing
}

//==============================================================================
double BinaryPriorityCostCalculator::compute_cost(
  const Node& n,
  rmf_traffic::Time time_now,
  bool check_priority) const
{
  return _pimpl->compute_f(n, time_now, check_priority);
}

//==============================================================================
double BinaryPriorityCostCalculator::compute_cost(
  rmf_task::agv::TaskPlanner::Assignments assignments) const
{
  return _pimpl->compute_g(assignments);
}

} // namespace rmf_task
