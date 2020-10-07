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

#include <src/rmf_traffic/blockade/Constraint.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

//==============================================================================
rmf_traffic::blockade::BlockageEndCondition reach(const std::size_t index)
{
  return rmf_traffic::blockade::BlockageEndCondition{
    index, rmf_traffic::blockade::BlockageEndCondition::HasReached
  };
}

//==============================================================================
rmf_traffic::blockade::BlockageEndCondition pass(const std::size_t index)
{
  return rmf_traffic::blockade::BlockageEndCondition{
    index, rmf_traffic::blockade::BlockageEndCondition::HasPassed
  };
}

//==============================================================================
std::size_t hold_at(std::size_t index)
{
  return index;
}

//==============================================================================
rmf_traffic::blockade::ReservedRange range(
    const std::size_t begin,
    const std::size_t end)
{
  return {begin, end};
}

//==============================================================================
std::optional<std::size_t> cannot_hold()
{
  return std::nullopt;
}

//==============================================================================
void ImplAnd(rmf_traffic::blockade::AndConstraint&)
{
  // Do nothing
}

//==============================================================================
template<typename... Args>
void ImplAnd(
    rmf_traffic::blockade::AndConstraint& and_constraint,
    rmf_traffic::blockade::ConstConstraintPtr next_constraint,
    const Args&... args)
{
  and_constraint.add(next_constraint);
  ImplAnd(and_constraint, args...);
}

//==============================================================================
template<typename... Args>
rmf_traffic::blockade::ConstConstraintPtr And(const Args&... args)
{
  const auto and_constraint =
      std::make_shared<rmf_traffic::blockade::AndConstraint>();

  ImplAnd(*and_constraint, args...);

  return and_constraint;
}

//==============================================================================
SCENARIO("Test gridlock detection")
{
  using namespace rmf_traffic::blockade;

  const std::size_t A = 0;
  const std::size_t B = 1;
  const std::size_t C = 2;
  const std::size_t D = 3;

  Blockers g;

  GIVEN("Simple 4-way blockage")
  {
    g[A][0] = blockage(D, hold_at(1), reach(2));
    g[A][1] = blockage(B, hold_at(0), reach(2));
    g[B][0] = blockage(A, hold_at(1), reach(2));
    g[B][1] = blockage(C, hold_at(0), reach(2));
    g[C][0] = blockage(B, hold_at(1), reach(2));
    g[C][1] = blockage(D, hold_at(0), reach(2));
    g[D][0] = blockage(C, hold_at(1), reach(2));
    g[D][1] = blockage(A, hold_at(0), reach(2));

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= D; ++i)
      CHECK(no_gridlock->dependencies().count(i));

    State state;
    state[A] = range(0, 0);
    state[B] = range(0, 0);
    state[C] = range(0, 0);
    state[D] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[C] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[D] = range(0, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[B] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[C] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[C] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[D] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[D] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(1, 1);
    state[B] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    state[C] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));

    state[C] = range(1, 1);
    state[D] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));
  }

  GIVEN("Triangle with one double-side")
  {
    g[A][0] = blockage(D, hold_at(1), reach(2));
    g[A][1] = And(
          blockage(B, hold_at(0), reach(2)),
          blockage(C, hold_at(0), reach(2)));
    g[B][0] = blockage(A, hold_at(1), reach(2));
    g[B][1] = blockage(D, hold_at(0), reach(2));
    g[C][0] = blockage(A, hold_at(1), reach(2));
    g[C][1] = blockage(D, hold_at(0), reach(2));
    g[D][0] = And(
          blockage(B, hold_at(1), reach(2)),
          blockage(C, hold_at(1), reach(2)));
    g[D][1] = blockage(A, hold_at(0), reach(2));

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= D; ++i)
      CHECK(no_gridlock->dependencies().count(i));

    State state;
    state[A] = range(0, 0);
    state[B] = range(0, 0);
    state[C] = range(0, 0);
    state[D] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[C] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[D] = range(0, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[B] = range(0, 0);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[C] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[B] = range(0, 0);
    state[C] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[D] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    CHECK(no_gridlock->evaluate(state));

    state[D] = range(1, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(1, 1);
    state[B] = range(2, 2);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[C] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    state[C] = range(2, 2);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[B] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(1, 1);
    state[D] = range(2, 2);
    CHECK(no_gridlock->evaluate(state));
  }
}
