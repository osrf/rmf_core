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
};

TaskPlanner::Configuration::Configuration(
  rmf_battery::agv::BatterySystem battery_system,
  std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
  std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
  std::shared_ptr<rmf_traffic::agv::Planner> planner)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        battery_system,
        std::move(motion_sink),
        std::move(ambient_sink),
        std::move(planner)
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
std::shared_ptr<rmf_battery::MotionPowerSink>
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
std::shared_ptr<rmf_battery::DevicePowerSink>
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
std::shared_ptr<rmf_traffic::agv::Planner>
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
rmf_task::ConstRequestPtr TaskPlanner::Assignment::request() const 
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
struct Invariant
{
  std::size_t task_id;
  double earliest_start_time;
  double earliest_finish_time;
};

// ============================================================================
struct InvariantLess
{
  bool operator()(const Invariant& a, const Invariant& b) const
  {
    return a.earliest_finish_time < b.earliest_finish_time;
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
    State previous_state;
    bool require_charge_battery = false;
  };

  // Map finish time to Entry
  using Map = std::multimap<rmf_traffic::Time, Entry>;

  static std::shared_ptr<Candidates> make(
    const std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    const rmf_task::Request& request,
    const rmf_task::requests::ChargeBattery& charge_battery_request,
    const std::shared_ptr<EstimateCache> estimate_cache,
    TaskPlanner::TaskPlannerError& error);

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
    rmf_traffic::Time wait_until,
    State previous_state,
    bool require_charge_battery)
  {
    const auto it = _candidate_map.at(candidate);
    _value_map.erase(it);
    _candidate_map[candidate] = _value_map.insert(
      {
        state.finish_time(),
        Entry{candidate, state, wait_until, previous_state, require_charge_battery}
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

std::shared_ptr<Candidates> Candidates::make(
  const std::vector<State>& initial_states,
  const std::vector<Constraints>& constraints_set,
  const rmf_task::Request& request,
  const rmf_task::requests::ChargeBattery& charge_battery_request,
  const std::shared_ptr<EstimateCache> estimate_cache,
  TaskPlanner::TaskPlannerError& error)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto& constraints = constraints_set[i];
    const auto finish = request.estimate_finish(
      state, constraints, estimate_cache);
    if (finish.has_value())
    {
      initial_map.insert({
        finish.value().finish_state().finish_time(),
        Entry{
          i,
          finish.value().finish_state(),
          finish.value().wait_until(),
          state,
          false}});
    }
    else
    {
      auto battery_estimate =
        charge_battery_request.estimate_finish(
          state, constraints, estimate_cache);
      if (battery_estimate.has_value())
      {
        auto new_finish = request.estimate_finish(
          battery_estimate.value().finish_state(), constraints, estimate_cache);
        if (new_finish.has_value())
        {
          initial_map.insert(
            {new_finish.value().finish_state().finish_time(),
            Entry{
              i,
              new_finish.value().finish_state(),
              new_finish.value().wait_until(),
              state,
              true}});
        }
        else
        {
          error = TaskPlanner::TaskPlannerError::limited_capacity;
        }
        
      }
      else
      {
        // Control reaches here either if ChargeBattery::estimate_finish() was
        // called on initial state with full battery or low battery such that
        // agent is unable to make it back to the charger
        if (abs(
          state.battery_soc() - charge_battery_request.max_charge_soc()) < 1e-3) 
            error = TaskPlanner::TaskPlannerError::limited_capacity;
        else
          error = TaskPlanner::TaskPlannerError::low_battery;
      }
    }
  }

  if (initial_map.empty())
  {
    return nullptr;
  }

  std::shared_ptr<Candidates> candidates(
    new Candidates(std::move(initial_map)));
  return candidates;
}

// ============================================================================
class PendingTask
{
public:

  static std::shared_ptr<PendingTask> make(
      std::vector<rmf_task::agv::State>& initial_states,
      std::vector<rmf_task::agv::Constraints>& constraints_set,
      rmf_task::ConstRequestPtr request_,
      rmf_task::ConstRequestPtr charge_battery_request,
      std::shared_ptr<EstimateCache> estimate_cache,
      TaskPlanner::TaskPlannerError& error);

  rmf_task::ConstRequestPtr request;
  Candidates candidates;

private:
  PendingTask(
      rmf_task::ConstRequestPtr request_,
      Candidates candidates_)
    : request(std::move(request_)),
      candidates(candidates_)
  {
    // Do nothing
  }
};

std::shared_ptr<PendingTask> PendingTask::make(
    std::vector<rmf_task::agv::State>& initial_states,
    std::vector<rmf_task::agv::Constraints>& constraints_set,
    rmf_task::ConstRequestPtr request_,
    rmf_task::ConstRequestPtr charge_battery_request,
    std::shared_ptr<EstimateCache> estimate_cache,
    TaskPlanner::TaskPlannerError& error)
{

  auto battery_request = std::dynamic_pointer_cast<
    const rmf_task::requests::ChargeBattery>(charge_battery_request);

  const auto candidates = Candidates::make(initial_states, constraints_set,
        *request_, *battery_request, estimate_cache, error);

  if (!candidates)
    return nullptr;

  std::shared_ptr<PendingTask> pending_task(
    new PendingTask(request_, *candidates));
  return pending_task;
}



// ============================================================================
struct Node
{
  struct AssignmentWrapper
  {
    std::size_t internal_id;
    TaskPlanner::Assignment assignment;
  };

  using AssignedTasks = std::vector<std::vector<AssignmentWrapper>>;
  using UnassignedTasks =
    std::unordered_map<std::size_t, PendingTask>;
  using InvariantSet = std::multiset<Invariant, InvariantLess>;

  AssignedTasks assigned_tasks;
  UnassignedTasks unassigned_tasks;
  double cost_estimate;
  rmf_traffic::Time latest_time;
  InvariantSet unassigned_invariants;
  std::size_t next_available_internal_id = 1;

  // ID 0 is reserved for charging tasks
  std::size_t get_available_internal_id(bool charging_task = false)
  {
    return charging_task ? 0 : next_available_internal_id++;
  }

  void sort_invariants()
  {
    unassigned_invariants.clear();
    for (const auto& u : unassigned_tasks)
    {
      double earliest_start_time = rmf_traffic::time::to_seconds(
        u.second.request->earliest_start_time().time_since_epoch());
      double earliest_finish_time = earliest_start_time
        + rmf_traffic::time::to_seconds(u.second.request->invariant_duration());

      unassigned_invariants.insert(
        Invariant{
          u.first,
          earliest_start_time,
          earliest_finish_time
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

// ============================================================================
inline double compute_g_assignment(const TaskPlanner::Assignment& assignment)
{
  if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
    assignment.request()))
  {
    return 0.0; // Ignore charging tasks in cost
  }

  return rmf_traffic::time::to_seconds(assignment.state().finish_time()
    - assignment.request()->earliest_start_time());
}

} // anonymous namespace

// ============================================================================
class TaskPlanner::Implementation
{
public:

  std::shared_ptr<Configuration> config;
  std::shared_ptr<EstimateCache> estimate_cache;
  const double priority_penalty = 10000;

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

  double compute_g(const Assignments& assigned_tasks)
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

  TaskPlanner::Assignments prune_assignments(
    TaskPlanner::Assignments& assignments)
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

  Result complete_solve(
    rmf_traffic::Time time_now,
    std::vector<State>& initial_states,
    const std::vector<Constraints>& constraints_set,
    const std::vector<ConstRequestPtr>& requests,
    const std::function<bool()> interrupter,
    bool greedy)
  {
    assert(initial_states.size() == constraints_set.size());
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

      assert(complete_assignments.size() == node->assigned_tasks.size());
      // std::size_t agent_count = 0;
      // std::cout << "Assignments from winning node: " << std::endl;
      // for (const auto& agent : node ->assigned_tasks)
      // {
      //   std::cout << "Agent: " << agent_count << std::endl;
      //   for (const auto& a : agent)
      //   {
      //     std::cout << "--" << a.assignment.request()->id() << std::endl;
      //   }
      //   agent_count++;
      // }
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

  double compute_g(const Node& node)
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

  double compute_h(const Node& node, const rmf_traffic::Time time_now)
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

  bool valid_assignment_priority(const Node& n)
  {
    const auto& assignments = n.assigned_tasks;
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

      bool prev_priority = it->assignment.request()->priority();
      ++it;
      for (; it != agent.end(); ++it)
      {
        if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
          it->assignment.request()))
          continue;
        bool curr_priority = it->assignment.request()->priority();
        if ((curr_priority != prev_priority) && (curr_priority == true))
          return false;

        prev_priority = curr_priority;
      }
    }

    return true;
  }

  double compute_f(const Node& n, const rmf_traffic::Time time_now)
  {
    const double g = compute_g(n);
    const double h = compute_h(n, time_now);

    if (!valid_assignment_priority(n))
      return priority_penalty * (g + h);
    
    return g + h;
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
          charge_battery,
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
      if (assignments.empty() || !std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
          assignments.back().assignment.request()))
      {
        auto charge_battery = make_charging_request(entry.previous_state.finish_time());
        auto battery_estimate = charge_battery->estimate_finish(
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
        new_u.second.request->estimate_finish(
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
      auto battery_estimate = charge_battery->estimate_finish(
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
            new_u.second.request->estimate_finish(battery_estimate.value().finish_state(),
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
    const std::vector<Constraints>& constraints_set,
    rmf_traffic::Time time_now)
  {
    auto new_node = std::make_shared<Node>(*parent);
     // Assign charging task to an agent
    State state = initial_states[agent];
    auto& assignments = new_node->assigned_tasks[agent];

    if (!assignments.empty())
    {
      if (std::dynamic_pointer_cast<const rmf_task::requests::ChargeBattery>(
          assignments.back().assignment.request()))
        return nullptr;
      state = assignments.back().assignment.state();
    }

    auto charge_battery = make_charging_request(state.finish_time());
    auto estimate = charge_battery->estimate_finish(
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
          new_u.second.request->estimate_finish(estimate.value().finish_state(),
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

      new_node->cost_estimate = compute_f(*new_node, time_now);
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
auto TaskPlanner::compute_cost(const Assignments& assignments) -> double
{
  return _pimpl->compute_g(assignments);
}

// ============================================================================
const std::shared_ptr<EstimateCache> TaskPlanner::estimate_cache() const
{
  return _pimpl->estimate_cache;
}

// ============================================================================
const std::shared_ptr<TaskPlanner::Configuration> TaskPlanner::config() const
{
  return _pimpl->config;
}

} // namespace agv
} // namespace rmf_task
