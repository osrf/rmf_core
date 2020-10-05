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

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class BlockageConstraint : public Constraint
{
public:

  BlockageConstraint(
      std::size_t blocked_by,
      std::size_t blocker_hold_point,
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
      const auto blocker = std::to_string(_blocked_by);
      const auto h = std::to_string(_blocker_hold_point);
      std::string error = "Failed to evaluate BlockageConstraint ";
      error += blocker + ":(" + h;
      if (_end_condition)
      {
        error += ", " + std::to_string(_end_condition->index);
        if (_end_condition->condition == BlockageEndCondition::HasReached)
          error += ")";
        else
          error += "]";
      }
      else
      {
        error += ")";
      }

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

private:

  bool _evaluate(const ReservedRange& range) const
  {
    if (range.end <= _blocker_hold_point)
      return true;

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

  std::size_t _blocked_by;
  std::size_t _blocker_hold_point;
  std::optional<BlockageEndCondition> _end_condition;
  std::unordered_set<std::size_t> _dependencies;

};

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

};

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
ConstConstraintPtr compute_gridlock_constraint(
    const std::unordered_map<std::size_t, Blockage>& blockers)
{

}

} // namespace blockade
} // namespace rmf_traffic
