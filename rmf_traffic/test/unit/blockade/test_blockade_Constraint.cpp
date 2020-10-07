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

rmf_traffic::blockade::BlockageEndCondition reach(const std::size_t index)
{
  return rmf_traffic::blockade::BlockageEndCondition{
    index, rmf_traffic::blockade::BlockageEndCondition::HasReached
  };
}

rmf_traffic::blockade::BlockageEndCondition pass(const std::size_t index)
{
  return rmf_traffic::blockade::BlockageEndCondition{
    index, rmf_traffic::blockade::BlockageEndCondition::HasPassed
  };
}

std::size_t hold_at(std::size_t index)
{
  return index;
}

std::optional<std::size_t> cannot_hold()
{
  return std::nullopt;
}

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

    ConstConstraintPtr gridlock_constraint = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= D; ++i)
      CHECK(gridlock_constraint->dependencies().count(i));

    State state;
    state[A] = ReservedRange{0, 0};
    state[B] = ReservedRange{0, 0};
    state[C] = ReservedRange{0, 0};
    state[D] = ReservedRange{0, 0};
    CHECK(gridlock_constraint->evaluate(state));

    state[A] = ReservedRange{0, 1};
    CHECK(gridlock_constraint->evaluate(state));

    state[B] = ReservedRange{0, 1};
    CHECK(gridlock_constraint->evaluate(state));

    state[C] = ReservedRange{0, 1};
    CHECK(gridlock_constraint->evaluate(state));

    state[D] = ReservedRange{0, 1};
    CHECK_FALSE(gridlock_constraint->evaluate(state));

    state[A] = ReservedRange{0, 0};
    CHECK(gridlock_constraint->evaluate(state));

    state[A] = ReservedRange{1, 1};
    CHECK_FALSE(gridlock_constraint->evaluate(state));

    state[B] = ReservedRange{0, 0};
    CHECK(gridlock_constraint->evaluate(state));

    state[B] = ReservedRange{1, 1};
    CHECK_FALSE(gridlock_constraint->evaluate(state));

    state[C] = ReservedRange{0, 0};
    CHECK(gridlock_constraint->evaluate(state));

    state[C] = ReservedRange{1, 1};
    CHECK_FALSE(gridlock_constraint->evaluate(state));

    state[D] = ReservedRange{0, 0};
    CHECK(gridlock_constraint->evaluate(state));

    state[D] = ReservedRange{1, 1};
    CHECK_FALSE(gridlock_constraint->evaluate(state));

    state[A] = ReservedRange{2, 2};
    CHECK(gridlock_constraint->evaluate(state));

    state[A] = ReservedRange{1, 1};
    state[B] = ReservedRange{2, 2};
    CHECK(gridlock_constraint->evaluate(state));

    state[B] = ReservedRange{1, 1};
    state[C] = ReservedRange{2, 2};
    CHECK(gridlock_constraint->evaluate(state));

    state[C] = ReservedRange{1, 1};
    state[D] = ReservedRange{2, 2};
    CHECK(gridlock_constraint->evaluate(state));
  }
}
