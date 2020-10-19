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

#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/Estimate.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>

#include <rmf_traffic/Time.hpp>

#include <thread>
#include <map>
#include <set>
#include <algorithm>
#include <unordered_map>
#include <limits>
#include <queue>
#include <iostream>

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
  FilterType filter_type;

};

TaskPlanner::Configuration::Configuration(
  rmf_battery::agv::BatterySystem battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner,
  const FilterType filter_type)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        battery_system,
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(planner),
        filter_type
      }))
{
  // Do nothing
}

//==============================================================================
rmf_battery::agv::BatterySystem& TaskPlanner::Configuration::battery_system()
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
std::shared_ptr<rmf_battery::MotionPowerSink> TaskPlanner::Configuration::motion_sink() const
{
  return _pimpl->motion_sink;
}

//==============================================================================
std::shared_ptr<rmf_battery::DevicePowerSink> TaskPlanner::Configuration::ambient_sink() const
{
  return _pimpl->ambient_sink;
}

//==============================================================================
std::shared_ptr<rmf_traffic::agv::Planner> TaskPlanner::Configuration::planner() const
{
  return _pimpl->planner;
} 

//==============================================================================
TaskPlanner::FilterType TaskPlanner::Configuration::filter_type() const
{
  return _pimpl->filter_type;
}

//==============================================================================
auto TaskPlanner::Configuration::filter_type(
  TaskPlanner::FilterType filter_type) -> Configuration&
{
  _pimpl->filter_type = filter_type;
  return *this;
}

//==============================================================================
class TaskPlanner::Assignment::Implementation
{
public:

  rmf_task::RequestPtr request;
  State state;
  rmf_traffic::Time deployment_time;
};

//==============================================================================
TaskPlanner::Assignment::Assignment(
  rmf_task::RequestPtr request,
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
rmf_task::RequestPtr TaskPlanner::Assignment::request() const 
{
  return _pimpl->request;
}

//==============================================================================
const State& TaskPlanner::Assignment::state() const
{
  return _pimpl->state;
}

//==============================================================================
const rmf_traffic::Time& TaskPlanner::Assignment::deployment_time() const
{
  return _pimpl->deployment_time;
}

//==============================================================================

namespace {

// ============================================================================
struct Invariant
{
  std::size_t task_id;
  double invariant_cost;
};

// ============================================================================
struct InvariantLess
{
  bool operator()(const Invariant& a, const Invariant& b) const
  {
    return a.invariant_cost < b.invariant_cost;
  }
};

// ============================================================================
class Candidates
{
public:

  struct Entry
  {
    std::size_t candidate;
    State state;
    rmf_traffic::Time wait_until;
  };

  // Map finish time to Entry
  using Map = std::multimap<rmf_traffic::Time, Entry>;

  static Candidates make(
      const std::vector<State>& initial_states,
      const std::vector<StateConfig>& state_configs,
      const rmf_task::Request& request,
      const rmf_task::Request& charge_battery_request);

  Candidates(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
  }

  Candidates& operator=(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
    return *this;
  }

  Candidates(Candidates&&) = default;
  Candidates& operator=(Candidates&&) = default;

  // We may have more than one best candidate so we store their iterators in
  // a Range
  struct Range
  {
    Map::const_iterator begin;
    Map::const_iterator end;
  };

  Range best_candidates() const
  {
    assert(!_value_map.empty());

    Range range;
    range.begin = _value_map.begin();
    auto it = range.begin;
    while (it->first == range.begin->first)
      ++it;

    range.end = it;
    return range;
  }

  rmf_traffic::Time best_finish_time() const
  {
    assert(!_value_map.empty());
    return _value_map.begin()->first;
  }

  void update_candidate(
    std::size_t candidate,
    State state,
    rmf_traffic::Time wait_until)
  {
    const auto it = _candidate_map.at(candidate);
    _value_map.erase(it);
    _candidate_map[candidate] = _value_map.insert(
      {
        state.finish_time(),
        Entry{candidate, state, wait_until}
      });
  }

private:
  Map _value_map;
  std::vector<Map::iterator> _candidate_map;

  Candidates(Map candidate_values)
    : _value_map(std::move(candidate_values))
  {
    _update_map();
  }

  void _update_map()
  {
    for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
    {
      const auto c = it->second.candidate;
      if (_candidate_map.size() <= c)
        _candidate_map.resize(c+1);

      _candidate_map[c] = it;
    }
  }
};

Candidates Candidates::make(
    const std::vector<State>& initial_states,
    const std::vector<StateConfig>& state_configs,
    const rmf_task::Request& request,
    const rmf_task::Request& charge_battery_request)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto& state_config = state_configs[i];
    const auto finish = request.estimate_finish(state, state_config);
    if (finish.has_value())
    {
      initial_map.insert({
        finish.value().finish_state().finish_time(),
        Entry{i, finish.value().finish_state(), finish.value().wait_until()}});
    }
    else
    {
      auto battery_estimate =
        charge_battery_request.estimate_finish(state, state_config);
      if (battery_estimate.has_value())
      {
        auto new_finish = request.estimate_finish(
          battery_estimate.value().finish_state(), state_config);
        assert(new_finish.has_value());
        initial_map.insert(
          {new_finish.value().finish_state().finish_time(),
          Entry{i, new_finish.value().finish_state(), new_finish.value().wait_until()}});
      }
      else
      {
        std::cerr << "Unable to create entry for candidate [" << i 
                  << "] and request [" << request.id() << " ]" << std::endl;
        assert(false);
      }
    }  
    
  }

  return Candidates(std::move(initial_map));
}

// ============================================================================
struct PendingTask
{
  PendingTask(
      std::vector<rmf_task::agv::State>& initial_states,
      std::vector<rmf_task::agv::StateConfig>& state_configs,
      rmf_task::Request::SharedPtr request_,
      rmf_task::Request::SharedPtr charge_battery_request)
    : request(std::move(request_)),
      candidates(Candidates::make(
        initial_states, state_configs, *request, *charge_battery_request)),
      earliest_start_time(request->earliest_start_time())
  {
    // Do nothing
  }

  rmf_task::Request::SharedPtr request;
  Candidates candidates;
  rmf_traffic::Time earliest_start_time;
};

// ============================================================================
struct Node
{
  using AssignedTasks = TaskPlanner::Assignments;
  using UnassignedTasks =
    std::unordered_map<std::size_t, PendingTask>;
  using InvariantSet = std::multiset<Invariant, InvariantLess>;

  AssignedTasks assigned_tasks;
  UnassignedTasks unassigned_tasks;
  double cost_estimate;
  rmf_traffic::Time latest_time;
  InvariantSet unassigned_invariants;

  void sort_invariants()
  {
    unassigned_invariants.clear();
    for (const auto& u : unassigned_tasks)
    {
      unassigned_invariants.insert(
        Invariant{
          u.first, 
          rmf_traffic::time::to_seconds(u.second.request->invariant_duration())
        });
    }
  }

  void pop_unassigned(std::size_t task_id)
  {
    unassigned_tasks.erase(task_id);

    bool popped_invariant = false;
    InvariantSet::iterator erase_it;
    for (auto it = unassigned_invariants.begin();
         it != unassigned_invariants.end(); ++it)
    {
      if (it->task_id == task_id)
      {
        popped_invariant = true;
        erase_it = it;
        break;
      }
    }
    unassigned_invariants.erase(erase_it);
    assert(popped_invariant);
  }
};


using NodePtr = std::shared_ptr<Node>;
using ConstNodePtr = std::shared_ptr<const Node>;

// ============================================================================
struct LowestCostEstimate
{
  bool operator()(const ConstNodePtr& a, const ConstNodePtr& b)
  {
    return b->cost_estimate < a->cost_estimate;
  }
};

//==============================================================================
class InvariantHeuristicQueue
{
public:

  InvariantHeuristicQueue(std::vector<double> initial_values)
  {
    assert(!initial_values.empty());
    std::sort(initial_values.begin(), initial_values.end());

    for (const auto value : initial_values)
      _stacks.push_back({value});
  }

  void add(double new_value)
  {
    // Add the new value to the smallest stack
    const double value = _stacks[0].back() + new_value;
    _stacks[0].push_back(value);

    // Find the largest stack that is still smaller than the current front
    const auto next_it = _stacks.begin() + 1;
    auto end_it = next_it;
    for (; end_it != _stacks.end(); ++end_it)
    {
      if (value <= end_it->back())
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
      for (std::size_t i=1; i < stack.size(); ++i)
        total_cost += stack[i];
    }

    return total_cost;
  }

private:
  std::vector<std::vector<double>> _stacks;
};

// ============================================================================
class Filter
{
public:

  Filter(TaskPlanner::FilterType type, const std::size_t N_tasks)
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
          const std::size_t id = s.request()->id() + 1;
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
          if (a[j].request()->id() != b[j].request()->id())
            return false;
        }
      }

      return true;
    }
  };

  using Set = std::unordered_set<Node::AssignedTasks, AssignmentHash, AssignmentEqual>;

  TaskPlanner::FilterType _type;
  AgentTable _root;
  Set _set;
};

bool Filter::ignore(const Node& node)
{
  if (_type == TaskPlanner::FilterType::Passthrough)
    return false;

  if (_type == TaskPlanner::FilterType::Hash)
    return !_set.insert(node.assigned_tasks).second;

  bool new_node = false;

  // TODO(MXG): Consider replacing this tree structure with a hash set

  AgentTable* agent_table = &_root;
  std::size_t a = 0;
  std::size_t t = 0;
  while(a < node.assigned_tasks.size())
  {
    const auto& current_agent = node.assigned_tasks.at(a);

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].request()->id();
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

const rmf_traffic::Duration segmentation_threshold =
    rmf_traffic::time::from_seconds(1.0);

} // anonymous namespace


class TaskPlanner::Implementation
{
public:

  std::shared_ptr<Configuration> config;

  RequestPtr make_charging_request(rmf_traffic::Time start_time)
  {
    return rmf_task::requests::ChargeBattery::make(
      config->battery_system(),
      config->motion_sink(),
      config->ambient_sink(),
      config->planner(),
      start_time,
      true);
  }

  double compute_g(const Assignments& assigned_tasks)
  {
    double cost = 0.0;
    for (const auto& agent : assigned_tasks)
    {
      for (const auto& assignment : agent)
      {
        cost +=
          rmf_traffic::time::to_seconds(
            assignment.state().finish_time() - assignment.request()->earliest_start_time());
      }
    }

    return cost;
  }

  Assignments prune_assignments(Assignments& assignments)
  {
    for (std::size_t a = 0; a < assignments.size(); ++a)
    {
      if (assignments[a].empty())
        continue;

      // Remove charging task at end of assignments if any
      // TODO(YV): Remove this after fixing the planner
      if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
          assignments[a].back().request()))
        assignments[a].pop_back();      
    }

    return assignments;
  }
  
  Assignments complete_solve(
    rmf_traffic::Time time_now,
    std::vector<State>& initial_states,
    const std::vector<StateConfig>& state_configs,
    const std::vector<Request::SharedPtr>& requests,
    const std::function<bool()> interrupter,
    bool greedy)
  {
    assert(initial_states.size() == state_configs.size());

    auto node = make_initial_node(initial_states, state_configs, requests, time_now);

    Node::AssignedTasks complete_assignments;
    complete_assignments.resize(node->assigned_tasks.size());

    while (node)
    {
      if (greedy)
        node = greedy_solve(node, initial_states, state_configs, time_now);
      else
        node = solve(node, initial_states, state_configs, requests.size(), time_now, interrupter);

      if (!node)
        return {};

      assert(complete_assignments.size() == node->assigned_tasks.size());
      for (std::size_t i = 0; i < complete_assignments.size(); ++i)
      {
        auto& all_assignments = complete_assignments[i];
        const auto& new_assignments = node->assigned_tasks[i];
        for (const auto& a : new_assignments)
        {
          all_assignments.push_back(a);
          // all_assignments.back().task_id = task_id_map.at(a.task_id);
        }
      }

      if (node->unassigned_tasks.empty())
        return prune_assignments(complete_assignments);

      std::vector<Request::SharedPtr> new_tasks;
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
          estimates[i] = assignments.back().state();
      }

      node = make_initial_node(estimates, state_configs, new_tasks, time_now);
      initial_states = estimates;
    }

    return complete_assignments;
  }

  double compute_g(const Node& node)
  {
    return compute_g(node.assigned_tasks);
  }

  double compute_h(const Node& node, const rmf_traffic::Time time_now)
  {
    std::vector<double> initial_queue_values;
    initial_queue_values.resize(
          node.assigned_tasks.size(), std::numeric_limits<double>::infinity());

    for (const auto& u : node.unassigned_tasks)
    {
      // We subtract the invariant duration here because otherwise its
      // contribution to the cost estimate will be duplicated in the next section,
      // which could result in an overestimate.
      const rmf_traffic::Time variant_time =
          u.second.candidates.best_finish_time()
          - u.second.request->invariant_duration();
      const double variant_value =
        rmf_traffic::time::to_seconds(variant_time - time_now);

      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        const std::size_t candidate = it->second.candidate;
        if (variant_value < initial_queue_values[candidate])
          initial_queue_values[candidate] = variant_value;
      }
    }

    for (std::size_t i=0; i < initial_queue_values.size(); ++i)
    {
      auto& value = initial_queue_values[i];
      if (std::isinf(value))
      {
        // Clear out any infinity placeholders. Those candidates simply don't have
        // any unassigned tasks that want to use it.
        const auto& assignments = node.assigned_tasks[i];
        if (assignments.empty())
          value = 0.0;
        else
          value =
            rmf_traffic::time::to_seconds(
              assignments.back().state().finish_time() - time_now);
      }
    }

    InvariantHeuristicQueue queue(std::move(initial_queue_values));
    // NOTE: It is crucial that we use the ordered set of unassigned_invariants
    // here. The InvariantHeuristicQueue expects the invariant costs to be passed
    // to it in order of smallest to largest. If that assumption is not met, then
    // the final cost that's calculated may be invalid.
    for (const auto& u : node.unassigned_invariants)
      queue.add(u.invariant_cost);

    return queue.compute_cost();
  }

  double compute_f(const Node& n, const rmf_traffic::Time time_now)
  {
    return compute_g(n) + compute_h(n, time_now);
  }

  ConstNodePtr make_initial_node(
    std::vector<State> initial_states,
    std::vector<StateConfig> state_configs,
    std::vector<Request::SharedPtr> requests,
    rmf_traffic::Time time_now)
  {
    auto initial_node = std::make_shared<Node>();

    initial_node->assigned_tasks.resize(initial_states.size());

    // TODO(YV): Come up with a better solution for charge_battery_request
    auto charge_battery = make_charging_request(time_now);
    for (const auto& request : requests)
      initial_node->unassigned_tasks.insert(
        {
          request->id(),
          PendingTask(initial_states, state_configs, request, charge_battery)
        });

    initial_node->cost_estimate = compute_f(*initial_node, time_now);

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
      
      const auto finish_time = a.back().state().finish_time();
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
    const std::vector<StateConfig>& state_configs)

  {
    const auto& entry = it->second;

    if (parent->latest_time + segmentation_threshold < entry.wait_until)
    {

      // No need to assign task as timeline is not relevant
      return nullptr;
    }

    auto new_node = std::make_shared<Node>(*parent);

    // Assign the unassigned task
    new_node->assigned_tasks[entry.candidate].push_back(
      Assignment{u.second.request, entry.state, entry.wait_until});
    
    // Erase the assigned task from unassigned tasks
    new_node->pop_unassigned(u.first);

    // Update states of unassigned tasks for the candidate
    const auto& state_config = state_configs[entry.candidate];
    bool add_charger = false;
    for (auto& new_u : new_node->unassigned_tasks)
    {
      const auto finish =
        new_u.second.request->estimate_finish(
          entry.state, state_config);

      if (finish.has_value())
      {
        new_u.second.candidates.update_candidate(
          entry.candidate,
          finish.value().finish_state(),
          finish.value().wait_until());
      }
      else
      {
        // TODO(YV): Revisit this strategy
        // auto battery_estimate =
        //   config->charge_battery_request()->estimate_finish(entry.state, state_config);
        // if (battery_estimate.has_value())
        // {
        //   auto new_finish =
        //     new_u.second.request->estimate_finish(
        //       battery_estimate.value().finish_state(),
        //       state_config);
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
      auto battery_estimate = charge_battery->estimate_finish(entry.state, state_config);
      if (battery_estimate.has_value())
      {
        new_node->assigned_tasks[entry.candidate].push_back(
          Assignment
          {
            charge_battery,
            battery_estimate.value().finish_state(),
            battery_estimate.value().wait_until()
          });
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish =
            new_u.second.request->estimate_finish(battery_estimate.value().finish_state(), state_config);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              entry.candidate, finish.value().finish_state(), finish.value().wait_until());
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
    new_node->cost_estimate = compute_f(*new_node, time_now);
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
    const std::vector<StateConfig>& state_configs,
    rmf_traffic::Time time_now)
  {
    auto new_node = std::make_shared<Node>(*parent);
     // Assign charging task to an agent
    State state = initial_states[agent];
    auto& assignments = new_node->assigned_tasks[agent];

    if (!assignments.empty())
    {
      if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
          assignments.back().request()))
        return nullptr;
      state = assignments.back().state();
    }

    auto charge_battery = make_charging_request(state.finish_time());
    auto estimate = charge_battery->estimate_finish(
      state, state_configs[agent]);
    if (estimate.has_value())
    {
      new_node->assigned_tasks[agent].push_back(
        Assignment{
          charge_battery,
          estimate.value().finish_state(),
          estimate.value().wait_until()});

      for (auto& new_u : new_node->unassigned_tasks)
      {
        const auto finish =
          new_u.second.request->estimate_finish(
            estimate.value().finish_state(), state_configs[agent]);
        if (finish.has_value())
        {
          new_u.second.candidates.update_candidate(
            agent, finish.value().finish_state(), finish.value().wait_until());
        }
        else
        {
          return nullptr;
        }
      }

      new_node->cost_estimate = compute_f(*new_node, time_now);
      new_node->latest_time = get_latest_time(*new_node);
      return new_node;
    }

    return nullptr;
  }

  ConstNodePtr greedy_solve(
    ConstNodePtr node,
    const std::vector<State>& initial_states,
    const std::vector<StateConfig>& state_configs,
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
            it, u, node, nullptr, time_now, state_configs))
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
                  state_configs,
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
    const std::vector<StateConfig>& state_configs,
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
          it, u, parent, &filter, time_now, state_configs))
          new_nodes.push_back(std::move(new_node));
      }
    }

    // Assign charging task to each robot
    for (std::size_t i = 0; i < parent->assigned_tasks.size(); ++i)
    {
      if (auto new_node = expand_charger(
        parent, i, initial_states, state_configs, time_now))
        new_nodes.push_back(new_node);
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
    const std::vector<StateConfig>& state_configs,
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

    Filter filter{config->filter_type(), num_tasks};
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
        top, filter, initial_states, state_configs, time_now);

      // Add copies and with a newly assigned task to queue
      for (const auto&n : new_nodes)
        priority_queue.push(n);
      
    }

    return nullptr;
  }
  
};

TaskPlanner::TaskPlanner(std::shared_ptr<Configuration> config)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(config)
      }))
{
  // Do nothing
}

auto TaskPlanner::greedy_plan(
  rmf_traffic::Time time_now,
  std::vector<State> initial_states,
  std::vector<StateConfig> state_configs,
  std::vector<Request::SharedPtr> requests) -> Assignments
{
  return _pimpl->complete_solve(
    time_now,
    initial_states,
    state_configs,
    requests,
    nullptr,
    true);
}

auto TaskPlanner::optimal_plan(
  rmf_traffic::Time time_now,
  std::vector<State> initial_states,
  std::vector<StateConfig> state_configs,
  std::vector<Request::SharedPtr> requests,
  std::function<bool()> interrupter) -> Assignments
{
  return _pimpl->complete_solve(
    time_now,
    initial_states,
    state_configs,
    requests,
    interrupter,
    false);
}

auto TaskPlanner::compute_cost(const Assignments& assignments) -> double
{
  return _pimpl->compute_g(assignments);
}




} // namespace agv
} // namespace rmf_task
