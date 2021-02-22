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

#include <rmf_task/Estimate.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/BinaryPriorityScheme.hpp>

#include "../BinaryPriorityCostCalculator.hpp"

#include <rmf_traffic/Time.hpp>

#include <limits>
#include <queue>

namespace rmf_task {
namespace agv {

//==============================================================================
class TaskPlanner::Configuration::Implementation
{
public:

  rmf_battery::agv::BatterySystem battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator;
};

TaskPlanner::Configuration::Configuration(
  rmf_battery::agv::BatterySystem battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        battery_system,
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(planner),
        std::move(cost_calculator)
      }))
{
  // Do nothing
}

//==============================================================================
const rmf_battery::agv::BatterySystem& TaskPlanner::Configuration::battery_system()
{
  return _pimpl->battery_system;
}

//==============================================================================
auto TaskPlanner::Configuration::battery_system(
  rmf_battery::agv::BatterySystem battery_system) -> Configuration&
{
  _pimpl->battery_system = battery_system;
  return *this;
}

//==============================================================================
const std::shared_ptr<rmf_battery::MotionPowerSink>&
TaskPlanner::Configuration::motion_sink() const
{
  return _pimpl->motion_sink;
}

//==============================================================================
auto TaskPlanner::Configuration::motion_sink(
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink) -> Configuration&
{
  if (motion_sink)
    _pimpl->motion_sink = motion_sink;
  return *this;
}

//==============================================================================
const std::shared_ptr<rmf_battery::DevicePowerSink>&
TaskPlanner::Configuration::ambient_sink() const
{
  return _pimpl->ambient_sink;
}

//==============================================================================
auto TaskPlanner::Configuration::ambient_sink(
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink) -> Configuration&
{
  if (ambient_sink)
    _pimpl->ambient_sink = ambient_sink;
  return *this;
}

//==============================================================================
const std::shared_ptr<rmf_traffic::agv::Planner>&
TaskPlanner::Configuration::planner() const
{
  return _pimpl->planner;
}

//==============================================================================
auto TaskPlanner::Configuration::planner(
  std::shared_ptr<rmf_traffic::agv::Planner> planner) -> Configuration&
{
  if (planner)
    _pimpl->planner = planner;
  return *this;
}

//==============================================================================
const std::shared_ptr<rmf_task::CostCalculator>&
TaskPlanner::Configuration::cost_calculator() const
{
  return _pimpl->cost_calculator;
}

//==============================================================================
auto TaskPlanner::Configuration::cost_calculator(
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator) -> Configuration&
{
  _pimpl->cost_calculator = cost_calculator;
  return *this;
}

//==============================================================================
class TaskPlanner::Assignment::Implementation
{
public:

  rmf_task::ConstRequestPtr request;
  State state;
  rmf_traffic::Time deployment_time;
};

//==============================================================================
TaskPlanner::Assignment::Assignment(
  rmf_task::ConstRequestPtr request,
  State state,
  rmf_traffic::Time deployment_time)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(request),
        std::move(state),
        deployment_time
      }))
{
  // Do nothing
}

//==============================================================================
const rmf_task::ConstRequestPtr& TaskPlanner::Assignment::request() const 
{
  return _pimpl->request;
}

//==============================================================================
const State& TaskPlanner::Assignment::state() const
{
  return _pimpl->state;
}

//==============================================================================
const rmf_traffic::Time TaskPlanner::Assignment::deployment_time() const
{
  return _pimpl->deployment_time;
}

//==============================================================================

namespace {

// ============================================================================
// The type of filter used for solving the task assignment problem
enum class FilterType
{
  Passthrough,
  Trie,
  Hash
};

// ============================================================================
class Filter
{
public:

  Filter(FilterType type, const std::size_t N_tasks)
    : _type(type),
      _set(N_tasks, AssignmentHash(N_tasks))
  {
    // Do nothing
  }

  bool ignore(const Node& node);

private:

  struct TaskTable;

  struct AgentTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<TaskTable>> agent;
  };

  struct TaskTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<AgentTable>> task;
  };
  
  struct AssignmentHash
  {
    AssignmentHash(std::size_t N)
    {
      // We add 1 to N because
      _shift = std::ceil(std::log2(N+1));
    }

    std::size_t operator()(const Node::AssignedTasks& assignments) const
    {
      std::size_t output = 0;
      std::size_t count = 0;
      for (const auto& a : assignments)
      {
        for (const auto& s : a)
        {
          // We add 1 to the task_id to differentiate between task_id == 0 and
          // a task being unassigned.
          const std::size_t id = s.internal_id + 1;
          output += id << (_shift * (count++));
        }
      }

      return output;
    }

    std::size_t _shift;
  };

  struct AssignmentEqual
  {
    bool operator()(
      const Node::AssignedTasks& A, const Node::AssignedTasks& B) const
    {
      if (A.size() != B.size())
        return false;

      for (std::size_t i=0; i < A.size(); ++i)
      {
        const auto& a = A[i];
        const auto& b = B[i];

        if (a.size() != b.size())
          return false;

        for (std::size_t j=0; j < a.size(); ++j)
        {
          if (a[j].internal_id != b[j].internal_id)
          {
            return false;
          }
        }
      }

      return true;
    }
  };

  using Set = std::unordered_set<Node::AssignedTasks, AssignmentHash, AssignmentEqual>;

  FilterType _type;
  AgentTable _root;
  Set _set;
};

bool Filter::ignore(const Node& node)
{
  if (_type == FilterType::Passthrough)
    return false;

  if (_type == FilterType::Hash)
    return !_set.insert(node.assigned_tasks).second;

  bool new_node = false;

  AgentTable* agent_table = &_root;
  std::size_t a = 0;
  std::size_t t = 0;
  while(a < node.assigned_tasks.size())
  {
    const auto& current_agent = node.assigned_tasks.at(a);

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].internal_id;
      const auto agent_insertion = agent_table->agent.insert({a, nullptr});
      if (agent_insertion.second)
        agent_insertion.first->second = std::make_unique<TaskTable>();

      auto* task_table = agent_insertion.first->second.get();

      const auto task_insertion = task_table->task.insert({task_id, nullptr});
      if (task_insertion.second)
      {
        new_node = true;
        task_insertion.first->second = std::make_unique<AgentTable>();
      }

      agent_table = task_insertion.first->second.get();
      ++t;
    }
    else
    {
      t = 0;
      ++a;
    }
  }

  return !new_node;
}

// ============================================================================
const rmf_traffic::Duration segmentation_threshold =
    rmf_traffic::time::from_seconds(1.0);

} // anonymous namespace

// ============================================================================
class TaskPlanner::Implementation
{
public:

  std::shared_ptr<Configuration> config;
  std::shared_ptr<EstimateCache> estimate_cache;
  bool check_priority = false;
  std::shared_ptr<rmf_task::CostCalculator> cost_calculator = nullptr;

  ConstRequestPtr make_charging_request(rmf_traffic::Time start_time)
  {
    return rmf_task::requests::ChargeBattery::make(
      config->battery_system(),
      config->motion_sink(),
      config->ambient_sink(),
      config->planner(),
      start_time,
      true);
  }

  TaskPlanner::Assignments prune_assignments(
    TaskPlanner::Assignments& assignments)
  {
    for (std::size_t a = 0; a < assignments.size(); ++a)
    {
      if (assignments[a].empty())
        continue;

      // Remove charging task at end of assignments if any
      // TODO(YV): Remove this after fixing the planner
      if (std::dynamic_pointer_cast<
        const rmf_task::requests::ChargeBatteryDescription>(
          assignments[a].back().request()->description()))
        assignments[a].pop_back();
    }

    return assignments;
  }

  ConstNodePtr prune_assignments(ConstNodePtr parent)
  {
    auto node = std::make_shared<Node>(*parent);

    for (auto& agent : node->assigned_tasks)
    {
      if (agent.empty())
        continue;

      if (std::dynamic_pointer_cast<
        const rmf_task::requests::ChargeBatteryDescription>(
          agent.back().assignment.request()->description()))
      agent.pop_back();
    }

    return node;
  }

  Result complete_solve(
    rmf_traffic::Time time_now,
    std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    const std::vector<ConstRequestPtr>& requests,
    const std::function<bool()> interrupter,
    bool greedy)
  {
    assert(initial_states.size() == constraints_set.size());

    cost_calculator = config->cost_calculator() ? config->cost_calculator() :
      rmf_task::BinaryPriorityScheme::make_cost_calculator();

    // Check if a high priority task exists among the requests.
    // If so the cost function for a node will be modified accordingly.
    for (const auto& request : requests)
    {
      if (request->priority())
      {
        check_priority = true;
        break;
      }      
    }
  
    TaskPlannerError error;
    auto node = make_initial_node(
      initial_states, constraints_set, requests, time_now, error);
    if (!node)
      return error;

    TaskPlanner::Assignments complete_assignments;
    complete_assignments.resize(node->assigned_tasks.size());

    while (node)
    {
      if (greedy)
        node = greedy_solve(node, initial_states, constraints_set, time_now);
      else
        node = solve(node, initial_states, constraints_set, requests.size(), time_now, interrupter);

      if (!node)
        return {};

      // Here we prune assignments to remove any charging tasks at the back of
      // the assignment list
      node = prune_assignments(node);
      assert(complete_assignments.size() == node->assigned_tasks.size());
      for (std::size_t i = 0; i < complete_assignments.size(); ++i)
      {
        auto& all_assignments = complete_assignments[i];
        const auto& new_assignments = node->assigned_tasks[i];
        for (const auto& a : new_assignments)
        {
          all_assignments.push_back(a.assignment);
        }
      }

      if (node->unassigned_tasks.empty())
      {
        return prune_assignments(complete_assignments);
      }

      std::vector<ConstRequestPtr> new_tasks;
      for (const auto& u : node->unassigned_tasks)
        new_tasks.push_back(u.second.request);

      // copy final state estimates 
      std::vector<State> estimates;
      rmf_traffic::agv::Plan::Start empty_new_location{
        time_now, 0, 0.0};
      estimates.resize(
        node->assigned_tasks.size(),
        State{empty_new_location, 0, 0.0});
      for (std::size_t i = 0; i < node->assigned_tasks.size(); ++i)
      {
        const auto& assignments = node->assigned_tasks[i];
        if (assignments.empty())
          estimates[i] = initial_states[i];
        else
          estimates[i] = assignments.back().assignment.state();
      }

      node = make_initial_node(
        estimates, constraints_set, new_tasks, time_now, error);
      if (!node)
        return error;
      initial_states = estimates;
    }

    return complete_assignments;
  }

  ConstNodePtr make_initial_node(
    std::vector<State> initial_states,
    std::vector<Constraints> constraints_set,
    std::vector<ConstRequestPtr> requests,
    rmf_traffic::Time time_now,
    TaskPlannerError& error)
  {
    auto initial_node = std::make_shared<Node>();

    initial_node->assigned_tasks.resize(initial_states.size());

    // TODO(YV): Come up with a better solution for charge_battery_request
    auto charge_battery = make_charging_request(time_now);
    for (const auto& request : requests)
    {
      // Generate a unique internal id for the request. Currently, multiple
      // requests with the same string id will be assigned different internal ids
      std::size_t internal_id = initial_node->get_available_internal_id();
      const auto pending_task= PendingTask::make(
          initial_states,
          constraints_set,
          request,
          charge_battery->description(),
          estimate_cache,
          error);
      
      if (!pending_task)
        return nullptr;

      initial_node->unassigned_tasks.insert(
        {
          internal_id,
          *pending_task
        });
    }

    initial_node->cost_estimate = cost_calculator->compute_cost(
      *initial_node, time_now, check_priority);

    initial_node->sort_invariants();

    initial_node->latest_time = [&]() -> rmf_traffic::Time
    {
      rmf_traffic::Time latest = rmf_traffic::Time::min();
      for (const auto& s : initial_states)
      {
        if (latest < s.finish_time())
          latest = s.finish_time();
      }

      return latest;
    }();

    rmf_traffic::Time wait_until = rmf_traffic::Time::max();

    for (const auto& u : initial_node->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        if (it->second.wait_until < wait_until)
          wait_until = it->second.wait_until;
      }
    }

    if (initial_node->latest_time < wait_until)
      initial_node->latest_time = wait_until;

    return initial_node;
  }

  rmf_traffic::Time get_latest_time(const Node& node)
  {
    rmf_traffic::Time latest = rmf_traffic::Time::min();
    for (const auto& a : node.assigned_tasks)
    {
      if (a.empty())
        continue;
      
      const auto finish_time = a.back().assignment.state().finish_time();
      if (latest < finish_time)
        latest = finish_time;
    }
    
    assert (latest > rmf_traffic::Time::min());
    return latest;
  }

  ConstNodePtr expand_candidate(
    const Candidates::Map::const_iterator& it,
    const Node::UnassignedTasks::value_type& u,
    const ConstNodePtr& parent,
    Filter* filter,
    rmf_traffic::Time time_now,
    const std::vector<Constraints>& constraints_set)

  {
    const auto& entry = it->second;
    const auto& constraints = constraints_set[entry.candidate];

    if (parent->latest_time + segmentation_threshold < entry.wait_until)
    {

      // No need to assign task as timeline is not relevant
      return nullptr;
    }

    auto new_node = std::make_shared<Node>(*parent);

    // Assign the unassigned task after checking for implicit charging requests
    if (entry.require_charge_battery)
    {
      // Check if a battery task already precedes the latest assignment
      auto& assignments = new_node->assigned_tasks[entry.candidate];
      if (assignments.empty() || !std::dynamic_pointer_cast<
        const rmf_task::requests::ChargeBatteryDescription>(
          assignments.back().assignment.request()->description()))
      {
        auto charge_battery = make_charging_request(entry.previous_state.finish_time());
        auto battery_estimate = charge_battery->description()->estimate_finish(
          entry.previous_state, constraints, estimate_cache);
        if (battery_estimate.has_value())
        {
          assignments.push_back(
            Node::AssignmentWrapper
            { u.first,
              Assignment
              {
                charge_battery,
                battery_estimate.value().finish_state(),
                battery_estimate.value().wait_until()
              }
            }
          );
        }
      }
    }
    new_node->assigned_tasks[entry.candidate].push_back(
      Node::AssignmentWrapper{u.first,
        Assignment{u.second.request, entry.state, entry.wait_until}});
    
    // Erase the assigned task from unassigned tasks
    new_node->pop_unassigned(u.first);

    // Update states of unassigned tasks for the candidate
    bool add_charger = false;
    for (auto& new_u : new_node->unassigned_tasks)
    {
      const auto finish =
        new_u.second.request->description()->estimate_finish(
          entry.state, constraints, estimate_cache);

      if (finish.has_value())
      {
        new_u.second.candidates.update_candidate(
          entry.candidate,
          finish.value().finish_state(),
          finish.value().wait_until(),
          entry.state,
          false);
      }
      else
      {
        // TODO(YV): Revisit this strategy
        // auto battery_estimate =
        //   config->charge_battery_request()->estimate_finish(entry.state, constraints);
        // if (battery_estimate.has_value())
        // {
        //   auto new_finish =
        //     new_u.second.request->estimate_finish(
        //       battery_estimate.value().finish_state(),
        //       constraints);
        //   assert(new_finish.has_value());
        //   new_u.second.candidates.update_candidate(
        //     entry.candidate,
        //     new_finish.value().finish_state(),
        //     new_finish.value().wait_until());
        // }
        // else
        // {
        //   // Unable to reach charger
        //   return nullptr;
        // }

        add_charger = true;
        break;
      }
    }

    if (add_charger)
    {
      auto charge_battery = make_charging_request(entry.state.finish_time());
      auto battery_estimate = charge_battery->description()->estimate_finish(
        entry.state, constraints, estimate_cache);
      if (battery_estimate.has_value())
      {
        new_node->assigned_tasks[entry.candidate].push_back(
          { new_node->get_available_internal_id(true),
            Assignment
            {
              charge_battery,
              battery_estimate.value().finish_state(),
              battery_estimate.value().wait_until()
            }});
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish =
            new_u.second.request->description()->estimate_finish(battery_estimate.value().finish_state(),
              constraints, estimate_cache);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              entry.candidate, finish.value().finish_state(), finish.value().wait_until(), entry.state, false);
          }
          else
          {
            // We should stop expanding this node
            return nullptr;
          }
        }
        
      }
      else
      {
        // Agent cannot make it back to the charger
        return nullptr;
      }
    }

    // Update the cost estimate for new_node
    new_node->cost_estimate = cost_calculator->compute_cost(
      *new_node, time_now, check_priority);
    new_node->latest_time = get_latest_time(*new_node);

    // Apply filter
    if (filter && filter->ignore(*new_node))
    {
      return nullptr;
    }

    return new_node;

  }

  ConstNodePtr expand_charger(
    ConstNodePtr parent,
    const std::size_t agent,
    const std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    rmf_traffic::Time time_now)
  {
    auto new_node = std::make_shared<Node>(*parent);
     // Assign charging task to an agent
    State state = initial_states[agent];
    auto& assignments = new_node->assigned_tasks[agent];

    // If the assignment set for a candidate is empty we do not want to add a
    // charging task as this is taken care of in expand_candidate(). Without this
    // step there is chance for the planner to get stuck in an infinite loop when
    // a charging task is required before any other task can be assigned.
    if (assignments.empty())
      return nullptr;

    if (!assignments.empty())
    {
      if (std::dynamic_pointer_cast<
        const rmf_task::requests::ChargeBatteryDescription>(
          assignments.back().assignment.request()->description()))
        return nullptr;
      state = assignments.back().assignment.state();
    }

    auto charge_battery = make_charging_request(state.finish_time());
    auto estimate = charge_battery->description()->estimate_finish(
      state, constraints_set[agent], estimate_cache);
    if (estimate.has_value())
    {
      new_node->assigned_tasks[agent].push_back(
        Node::AssignmentWrapper
        {
          new_node->get_available_internal_id(true),
          Assignment
          {
            charge_battery,
            estimate.value().finish_state(),
            estimate.value().wait_until()
          }
        });
      for (auto& new_u : new_node->unassigned_tasks)
      {
        const auto finish =
          new_u.second.request->description()->estimate_finish(estimate.value().finish_state(),
            constraints_set[agent], estimate_cache);
        if (finish.has_value())
        {
          new_u.second.candidates.update_candidate(
            agent,
            finish.value().finish_state(),
            finish.value().wait_until(),
            state,
            false);
        }
        else
        {
          return nullptr;
        }
      }

      new_node->cost_estimate = cost_calculator->compute_cost(
        *new_node, time_now, check_priority);
      new_node->latest_time = get_latest_time(*new_node);
      return new_node;
    }

    return nullptr;
  }

  ConstNodePtr greedy_solve(
    ConstNodePtr node,
    const std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    rmf_traffic::Time time_now)
  {
    while (!finished(*node))
    {
      ConstNodePtr next_node = nullptr;
      for (const auto& u : node->unassigned_tasks)
      {
        const auto& range = u.second.candidates.best_candidates();
        for (auto it = range.begin; it != range.end; ++it)
        {
          if (auto n = expand_candidate(
            it, u, node, nullptr, time_now, constraints_set))
          {
            if (!next_node || (n->cost_estimate < next_node->cost_estimate))
              {
                next_node = std::move(n);
              }
          }
          else
          {
            // expand_candidate returned nullptr either due to start time
            // segmentation or insufficient charge to return to its charger. 
            // For the later case, we aim to backtrack and assign a charging
            // task to the agent.
            if (node->latest_time + segmentation_threshold > it->second.wait_until)
            {
              auto parent_node = std::make_shared<Node>(*node);
              while (!parent_node->assigned_tasks[it->second.candidate].empty())
              {
                parent_node->assigned_tasks[it->second.candidate].pop_back();
                auto new_charge_node = expand_charger(
                  parent_node,
                  it->second.candidate,
                  initial_states,
                  constraints_set,
                  time_now);
                if (new_charge_node)
                {
                  next_node = std::move(new_charge_node);
                  break;
                }
              }
            }
          }
        }
      }

      node = next_node;
      assert(node);
    }

    return node;
  }

  std::vector<ConstNodePtr> expand(
    ConstNodePtr parent,
    Filter& filter,
    const std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    rmf_traffic::Time time_now)
  {
    std::vector<ConstNodePtr> new_nodes;
    new_nodes.reserve(
      parent->unassigned_tasks.size() + parent->assigned_tasks.size());
    for (const auto& u : parent->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it!= range.end; it++)
      {
        if (auto new_node = expand_candidate(
          it, u, parent, &filter, time_now, constraints_set))
          new_nodes.push_back(std::move(new_node));
      }
    }

    // Assign charging task to each robot
    for (std::size_t i = 0; i < parent->assigned_tasks.size(); ++i)
    {
      if (auto new_node = expand_charger(
        parent, i, initial_states, constraints_set, time_now))
        new_nodes.push_back(std::move(new_node));
    }

    return new_nodes;
  }

  bool finished(const Node& node)
  {
    for (const auto& u : node.unassigned_tasks)
    {
      const auto range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it!= range.end; ++it)
      {
        const auto wait_time = it->second.wait_until;
        if (wait_time <= node.latest_time + segmentation_threshold)
          return false;
      }
    }

    return true;
  }

  ConstNodePtr solve(
    ConstNodePtr initial_node,
    const std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    const std::size_t num_tasks,
    rmf_traffic::Time time_now,
    std::function<bool()> interrupter)
  {
    using PriorityQueue = std::priority_queue<
      ConstNodePtr,
      std::vector<ConstNodePtr>,
      LowestCostEstimate>;

    PriorityQueue priority_queue;
    priority_queue.push(std::move(initial_node));

    Filter filter{FilterType::Hash, num_tasks};
    ConstNodePtr top = nullptr;

    while (!priority_queue.empty() && !(interrupter && interrupter()))
    {
      top = priority_queue.top();

      // Pop the top of the priority queue
      priority_queue.pop();

      // Check if unassigned tasks is empty -> solution found
      if (finished(*top))
      {
        return top;
      }

      // Apply possible actions to expand the node
      const auto new_nodes = expand(
        top, filter, initial_states, constraints_set, time_now);

      // Add copies and with a newly assigned task to queue
      for (const auto&n : new_nodes)
        priority_queue.push(n);
    }

    return nullptr;
  }
  
};

// ============================================================================
TaskPlanner::TaskPlanner(std::shared_ptr<Configuration> config)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        config,
        std::make_shared<EstimateCache>(
          config->planner()->get_configuration().graph().num_waypoints())
      }))
{
  // Do nothing
}

// ============================================================================
auto TaskPlanner::greedy_plan(
  rmf_traffic::Time time_now,
  std::vector<State> initial_states,
  std::vector<Constraints> constraints_set,
  std::vector<ConstRequestPtr> requests) -> Result
{
  return _pimpl->complete_solve(
    time_now,
    initial_states,
    constraints_set,
    requests,
    nullptr,
    true);
}

// ============================================================================
auto TaskPlanner::optimal_plan(
  rmf_traffic::Time time_now,
  std::vector<State> initial_states,
  std::vector<Constraints> constraints_set,
  std::vector<ConstRequestPtr> requests,
  std::function<bool()> interrupter) -> Result
{
  return _pimpl->complete_solve(
    time_now,
    initial_states,
    constraints_set,
    requests,
    interrupter,
    false);
}

// ============================================================================
auto TaskPlanner::compute_cost(const Assignments& assignments) const -> double
{
  if (_pimpl->config->cost_calculator())
    return _pimpl->config->cost_calculator()->compute_cost(assignments);

  const auto cost_calculator =
    rmf_task::BinaryPriorityScheme::make_cost_calculator();
  return cost_calculator->compute_cost(assignments);
  
}

// ============================================================================
const std::shared_ptr<EstimateCache>& TaskPlanner::estimate_cache() const
{
  return _pimpl->estimate_cache;
}

// ============================================================================
const std::shared_ptr<TaskPlanner::Configuration>& TaskPlanner::config() const
{
  return _pimpl->config;
}

} // namespace agv
} // namespace rmf_task
