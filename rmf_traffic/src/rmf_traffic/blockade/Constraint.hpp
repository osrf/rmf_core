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

#ifndef SRC__RMF_TRAFFIC__BLOCKADE__CONSTRAINT_HPP
#define SRC__RMF_TRAFFIC__BLOCKADE__CONSTRAINT_HPP

#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <memory>

#include <rmf_traffic/blockade/Writer.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
using State = std::unordered_map<ParticipantId, ReservedRange>;

//==============================================================================
class Constraint
{
public:

  /// Return true if the State is acceptable to this Constraint.
  /// Return false if the State violates this Constraint.
  virtual bool evaluate(const State& state) const = 0;

  virtual const std::unordered_set<std::size_t>& dependencies() const = 0;

  virtual std::optional<bool> partial_evaluate(const State& state) const = 0;

  virtual ~Constraint() = default;
};

//==============================================================================
using ConstraintPtr = std::shared_ptr<Constraint>;
using ConstConstraintPtr = std::shared_ptr<const Constraint>;

//==============================================================================
struct BlockageEndCondition
{
  enum Condition
  {
    HasReached,
    HasPassed
  };

  std::size_t index;
  Condition condition;
};

//==============================================================================
std::shared_ptr<Constraint> blockage(
    std::size_t blocked_by,
    std::optional<std::size_t> blocker_hold_point,
    std::optional<BlockageEndCondition> end_condition);

//==============================================================================
class AndConstraint : public Constraint
{
public:

  AndConstraint(const std::vector<ConstConstraintPtr>& constraints = {});

  void add(ConstConstraintPtr new_constraint);

  bool evaluate(const State& state) const final;
  const std::unordered_set<std::size_t>& dependencies() const final;
  std::optional<bool> partial_evaluate(const State& state) const final;

private:
  std::unordered_set<ConstConstraintPtr> _constraints;
  std::unordered_set<std::size_t> _dependencies;
};

//==============================================================================
class OrConstraint : public Constraint
{
public:

  OrConstraint(const std::vector<ConstConstraintPtr>& constraints = {});

  void add(ConstConstraintPtr new_constraint);

  bool evaluate(const State& state) const final;
  const std::unordered_set<std::size_t>& dependencies() const final;
  std::optional<bool> partial_evaluate(const State& state) const final;

private:
  std::unordered_set<ConstConstraintPtr> _constraints;
  std::unordered_set<std::size_t> _dependencies;
};

//==============================================================================
using IndexToConstraint = std::unordered_map<std::size_t, ConstConstraintPtr>;

//==============================================================================
using Blockers = std::unordered_map<std::size_t, IndexToConstraint>;

//==============================================================================
ConstConstraintPtr compute_gridlock_constraint(const Blockers& blockers);

} // namespace blockade
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__BLOCKADE__CONSTRAINT_HPP
