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

#ifndef SRC__RMF_TASK__AGV__INTERNAL_TASK_PLANNING_HPP
#define SRC__RMF_TASK__AGV__INTERNAL_TASK_PLANNING_HPP

#include <rmf_task/agv/TaskPlanner.hpp>
#include <rmf_task/requests/ChargeBattery.hpp>

#include <map>
#include <set>
#include <algorithm>
#include <unordered_map>
#include <limits>

namespace rmf_task {
namespace agv{

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
    const rmf_task::requests::ChargeBatteryDescription& charge_battery_desc,
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

// ============================================================================
class PendingTask
{
public:

  static std::shared_ptr<PendingTask> make(
      std::vector<rmf_task::agv::State>& initial_states,
      std::vector<rmf_task::agv::Constraints>& constraints_set,
      rmf_task::ConstRequestPtr request_,
      rmf_task::Request::DescriptionPtr charge_battery_desc,
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
        + rmf_traffic::time::to_seconds(u.second.request->description()->invariant_duration());

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

} // namespace agv
} // namespace rmf_task

#endif // SRC__RMF_TASK__AGV__INTERNAL_TASK_PLANNING_HPP
