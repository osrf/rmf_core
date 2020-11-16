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

#include "Constraint.hpp"

#include <vector>
#include <cassert>


#include <iostream>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
// To Uppercase Letter
std::string toul(const std::size_t input)
{
  const std::size_t TotalLetters = 90-65+1;
  std::string output;
  std::size_t value = input;
  do
  {
    const std::size_t digit = value % TotalLetters;
    char offset = output.empty()? 'A' : 'A'-1;
    output += static_cast<char>(digit + offset);
    value /= TotalLetters;
  } while (value > 0);

  std::reverse(output.begin(), output.end());
  return output;
}

//==============================================================================
class BlockageConstraint : public Constraint
{
public:

  BlockageConstraint(
      std::size_t blocked_by,
      std::optional<std::size_t> blocker_hold_point,
      std::optional<BlockageEndCondition> end_condition)
    : _blocked_by(blocked_by),
      _blocker_hold_point(blocker_hold_point),
      _end_condition(end_condition)
  {
    _dependencies.insert(_blocked_by);
  }

  bool evaluate(const State& state) const final
  {
    const auto it = state.find(_blocked_by);
    if (it == state.end())
    {
      std::string error = "Failed to evaluate BlockageConstraint ";
      error += _description();

      error += " for state: <";
      for (const auto& s : state)
      {
        const auto p = std::to_string(s.first);
        const auto begin = std::to_string(s.second.begin);
        const auto end = std::to_string(s.second.end);

        error += p + ":[" + begin + "," + end + "]";
      }

      error += ">";

      throw std::runtime_error(error);
    }

    return _evaluate(it->second);
  }

  const std::unordered_set<std::size_t>& dependencies() const final
  {
    return _dependencies;
  }

  std::optional<bool> partial_evaluate(const State& state) const final
  {
    const auto it = state.find(_blocked_by);
    if (it == state.end())
      return std::nullopt;

    return _evaluate(it->second);
  }

  std::string detail(const State& state) const final
  {
    const auto& range = state.at(_blocked_by);

    std::stringstream str;

    const bool two_parts =
        _blocker_hold_point.has_value() && _end_condition.has_value();
    const bool hold_failed = !_evaluate_can_hold(range);
    const bool end_failed = !_evaluate_has_reached(range);
    const bool whole_failed = hold_failed && end_failed;

    if (two_parts)
    {
      if (whole_failed)
        str << "{";
      else
        str << "[";
    }

    if (_blocker_hold_point.has_value())
    {
      if (hold_failed)
        str << "{";
      str << "h(" << toul(_blocked_by) << _blocker_hold_point.value() << ")";
      if (hold_failed)
        str << "}";
    }

    if (two_parts)
      str << "|";

    if (_end_condition.has_value())
    {
      if (end_failed)
        str << "{";
      if (_end_condition.value().condition == BlockageEndCondition::HasReached)
        str << "r(";
      else
        str << "p(";
      str << toul(_blocked_by) << _end_condition.value().index << ")";
      if (end_failed)
        str << "}";
    }

    if (two_parts)
    {
      if (whole_failed)
        str << "}";
      else
        str << "]";
    }

    return str.str();
  }

private:

  bool _evaluate_can_hold(const ReservedRange& range) const
  {
    if (_blocker_hold_point)
    {
      if (range.end <= _blocker_hold_point)
        return true;
    }

    return false;
  }

  bool _evaluate_has_reached(const ReservedRange& range) const
  {
    if (_end_condition)
    {
      const std::size_t has_reached = _end_condition->index;

      if (range.begin < has_reached)
        return false;

      // Implicit: has_reached <= range.begin
      if (has_reached < range.end)
        return true;

      if (_end_condition->condition == BlockageEndCondition::HasReached)
      {
        if (_end_condition->index == range.begin)
          return true;
      }
    }

    return false;
  }

  bool _evaluate(const ReservedRange& range) const
  {
    if (_evaluate_can_hold(range))
      return true;

    if (_evaluate_has_reached(range))
      return true;

    return false;
  }

  std::string _description() const
  {
    const auto h = _blocker_hold_point?
          std::to_string(*_blocker_hold_point) : std::string("null");

    std::string desc = std::to_string(_blocked_by) + ":(" + h;
    if (_end_condition)
    {
      desc += ", " + std::to_string(_end_condition->index);
      if (_end_condition->condition == BlockageEndCondition::HasReached)
        desc += ")";
      else
        desc+= "]";
    }
    else
    {
      desc += ")";
    }

    return desc;
  }

  bool print_fail(const ReservedRange& range) const
  {
    std::cout << " :: Blocked by " << _description()
              << " whose range is: " << range.begin << " --> "
              << range.end << std::endl;
    return false;
  }

  std::size_t _blocked_by;
  std::optional<std::size_t> _blocker_hold_point;
  std::optional<BlockageEndCondition> _end_condition;
  std::unordered_set<std::size_t> _dependencies;

};

//==============================================================================
ConstConstraintPtr blockage(
    std::size_t blocked_by,
    std::optional<std::size_t> blocker_hold_point,
    std::optional<BlockageEndCondition> end_condition)
{
  return std::make_shared<BlockageConstraint>(
        blocked_by, blocker_hold_point, end_condition);
}

//==============================================================================
class PassedConstraint : public Constraint
{
public:

  PassedConstraint(
      std::size_t participant,
      std::size_t index)
    : _participant(participant),
      _index(index)
  {
    _dependencies.insert(_participant);
  }

  bool evaluate(const State& state) const final
  {
    const auto it = state.find(_participant);
    if (it == state.end())
    {
      std::string error = "Failed to evaluate PassedConstraint because "
          "participant " + std::to_string(_participant)
          + " is missing from the state.";

      throw std::runtime_error(error);
    }

    return _evaluate(it->second);
  }

  const std::unordered_set<std::size_t>& dependencies() const final
  {
    return _dependencies;
  }

  std::optional<bool> partial_evaluate(const State& state) const final
  {
    const auto it = state.find(_participant);
    if (it == state.end())
      return std::nullopt;

    return _evaluate(it->second);
  }

  std::string detail(const State& state) const final
  {
    std::stringstream str;

    const auto& range = state.at(_participant);
    const bool failed = !_evaluate(range);
    if (failed)
      str << "{";

    str << "p(" << toul(_participant) << _index << ")";
    if (failed)
      str << "}";

    return str.str();
  }

private:

  bool _evaluate(const ReservedRange& range) const
  {
    if (_index < range.begin)
      return true;

    if (range.begin < _index)
      return false;

    return _index < range.end;
  }

  std::size_t _participant;
  std::size_t _index;
  std::unordered_set<std::size_t> _dependencies;
};

//==============================================================================
ConstConstraintPtr passed(
    std::size_t participant,
    std::size_t index)
{
  return std::make_shared<PassedConstraint>(participant, index);
}

//==============================================================================
class AlwaysValid : public Constraint
{
public:

  bool evaluate(const State&) const final
  {
    return true;
  }

  const std::unordered_set<std::size_t> & dependencies() const final
  {
    static const std::unordered_set<std::size_t> empty_set;
    return empty_set;
  }

  std::optional<bool> partial_evaluate(const State&) const final
  {
    return true;
  }

  std::string detail(const State&) const final
  {
    return "True";
  }

};

//==============================================================================
AndConstraint::AndConstraint(const std::vector<ConstConstraintPtr>& constraints)
{
  for (const auto& c : constraints)
    add(c);
}

//==============================================================================
void AndConstraint::add(ConstConstraintPtr new_constraint)
{
  for (const auto& dep : new_constraint->dependencies())
    _dependencies.insert(dep);

  _constraints.emplace(std::move(new_constraint));
}

//==============================================================================
bool AndConstraint::evaluate(const State& state) const
{
  for (const auto& c : _constraints)
  {
    if (!c->evaluate(state))
      return false;
  }

  return true;
}

//==============================================================================
const std::unordered_set<std::size_t>& AndConstraint::dependencies() const
{
  return _dependencies;
}

//==============================================================================
std::optional<bool> AndConstraint::partial_evaluate(const State& state) const
{
  for (const auto& c : _constraints)
  {
    if (const auto opt_value = c->partial_evaluate(state))
    {
      const auto value = *opt_value;
      if (!value)
        return false;
    }
  }

  return std::nullopt;
}

//==============================================================================
std::string AndConstraint::detail(const State& state) const
{
  if (_constraints.empty())
    return "And-Empty-True";

  if (_constraints.size() == 1)
  {
    return (*_constraints.begin())->detail(state);
  }

  std::stringstream str;

  const bool failed = !evaluate(state);
  if (failed)
    str << "{ ";
  else
    str << "[ ";

  bool first = true;
  for (const auto& c : _constraints)
  {
    if (first)
      first = false;
    else
      str << " & ";

    str << c->detail(state);
  }

  if (failed)
    str << " }";
  else
    str << " ]";

  return str.str();
}

//==============================================================================
OrConstraint::OrConstraint(const std::vector<ConstConstraintPtr>& constraints)
{
  for (const auto& c : constraints)
    add(c);
}

//==============================================================================
void OrConstraint::add(ConstConstraintPtr new_constraint)
{
  for (const auto& dep : new_constraint->dependencies())
    _dependencies.insert(dep);

  _constraints.emplace(std::move(new_constraint));
}

//==============================================================================
bool OrConstraint::evaluate(const State& state) const
{
  for (const auto& c : _constraints)
  {
    if (c->evaluate(state))
      return true;
  }

  if (_constraints.empty())
    return true;

  return false;
}

//==============================================================================
const std::unordered_set<std::size_t>& OrConstraint::dependencies() const
{
  return _dependencies;
}

//==============================================================================
std::optional<bool> OrConstraint::partial_evaluate(const State& state) const
{
  for (const auto& c : _constraints)
  {
    if (const auto opt_value = c->partial_evaluate(state))
    {
      const auto value = *opt_value;
      if (value)
        return true;
    }
  }

  return std::nullopt;
}

//==============================================================================
std::string OrConstraint::detail(const State& state) const
{
  if (_constraints.empty())
    return "Or-Empty-True";

  if (_constraints.size() == 1)
  {
    return (*_constraints.begin())->detail(state);
  }

  std::stringstream str;

  const bool failed = !evaluate(state);
  if (failed)
    str << "{ ";
  else
    str << "[ ";

  bool first = true;
  for (const auto& c : _constraints)
  {
    if (first)
      first = false;
    else
      str << " | ";

    str << c->detail(state);
  }

  if (failed)
    str << " }";
  else
    str << " ]";

  return str.str();
}

namespace {

//==============================================================================
using Cache = std::unordered_map<std::size_t, std::unordered_set<std::size_t>>;

//==============================================================================
struct GridlockNode;
using GridlockNodePtr = std::shared_ptr<const GridlockNode>;

//==============================================================================
struct Blocker
{
  std::size_t participant;
  std::size_t index;
  ConstConstraintPtr constraint;

  bool operator==(const Blocker& other) const
  {
    return participant == other.participant && index == other.index;
  }

  bool operator!=(const Blocker& other) const
  {
    return !(*this == other);
  }
};

//==============================================================================
struct GridlockNode
{
  Blocker blocker;
  GridlockNodePtr parent;
  Cache visited;
};

} // anonymous namespace

//==============================================================================
ConstConstraintPtr compute_gridlock_constraint(GridlockNodePtr node)
{
  auto or_constraint = std::make_shared<OrConstraint>();
  const auto target_blocker = node->blocker;

  do
  {
    or_constraint->add(node->blocker.constraint);
    node = node->parent;
    assert(node);
  } while (target_blocker != node->blocker);

  return or_constraint;
}

//==============================================================================
ConstConstraintPtr compute_gridlock_constraint(const Blockers& blockers)
{
  std::vector<GridlockNodePtr> queue;
  std::unordered_map<std::size_t, std::vector<Blocker>> dependents;
  for (const auto& b : blockers)
  {
    const auto participant = b.first;
    const auto& points = b.second;
    for (const auto& p : points)
    {
      const auto index = p.first;
      const auto constraint = p.second;
      Blocker blocker{participant, index, constraint};

      for (const std::size_t dependency : constraint->dependencies())
      {
        auto& v = dependents.insert({dependency, {}}).first->second;
        v.push_back(blocker);
      }

      auto node = std::make_shared<GridlockNode>(
            GridlockNode{blocker, nullptr, {}});
      node->visited[participant].insert(index);

      queue.push_back(std::move(node));
    }
  }

  Cache expanded_nodes;
  State test_state;

  std::vector<ConstConstraintPtr> gridlock_constraints;

  while (!queue.empty())
  {
    const auto top = queue.back();
    queue.pop_back();

    const bool is_new_node = expanded_nodes
        .insert({top->blocker.participant, {}})
        .first->second.insert(top->blocker.index).second;

    if (!is_new_node)
      continue;

    const auto participant = top->blocker.participant;
    const auto index = top->blocker.index;

    const auto it = dependents.find(top->blocker.participant);
    if (it == dependents.end())
      continue;

    test_state.clear();
    test_state[participant] = ReservedRange{index, index};

    for (const auto& dep : it->second)
    {
      std::optional<bool> opt_eval = dep.constraint->partial_evaluate(test_state);
      if (!opt_eval.has_value() || opt_eval.value())
        continue;

      // This constraint is blocked by holding here, so we'll add it to the
      // chain
      Cache new_visited = top->visited;
      const bool loop_found = !new_visited[dep.participant]
          .insert(dep.index).second;

      auto next = std::make_shared<GridlockNode>(
            GridlockNode{dep, top, std::move(new_visited)});

      if (loop_found)
        gridlock_constraints.push_back(compute_gridlock_constraint(next));
      else
        queue.push_back(next);
    }
  }

  if (gridlock_constraints.empty())
    return std::make_shared<AlwaysValid>();

  return std::make_shared<AndConstraint>(gridlock_constraints);
}

} // namespace blockade
} // namespace rmf_traffic
