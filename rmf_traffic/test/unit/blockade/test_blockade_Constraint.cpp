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

#include "utils_blockade_simulation.hpp"

#include <algorithm>

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
SCENARIO("Test gridlock detection")
{
  using namespace rmf_traffic::blockade;

  const std::size_t A = 0;
  const std::size_t B = 1;
  const std::size_t C = 2;
  const std::size_t D = 3;
  const std::size_t E = 4;

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

    simulate_all_sequences(g, no_gridlock, {2, 2, 2, 2});
  }

  GIVEN("Triangle with one double-side")
  {
    g[A][0] = blockage(D, hold_at(1), reach(2));
    g[A][1] =
        And(
          blockage(B, hold_at(0), reach(2)),
          blockage(C, hold_at(0), reach(2))
        );
    g[B][0] = blockage(A, hold_at(1), reach(2));
    g[B][1] = blockage(D, hold_at(0), reach(2));
    g[C][0] = blockage(A, hold_at(1), reach(2));
    g[C][1] = blockage(D, hold_at(0), reach(2));
    g[D][0] =
        And(
          blockage(B, hold_at(1), reach(2)),
          blockage(C, hold_at(1), reach(2))
        );
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

    simulate_all_sequences(g, no_gridlock, {2, 2, 2, 2});
  }

  GIVEN("Criss-cross intersection")
  {
    g[A][1] =
        And(
          blockage(B, hold_at(0), reach(2)),
          blockage(C, hold_at(1), reach(2))
        );
    g[A][3] = blockage(B, hold_at(0), reach(2));
    g[B][0] =
        And(
          blockage(A, hold_at(1), reach(2)),
          blockage(A, hold_at(3), reach(4))
        );
    g[B][1] = blockage(C, hold_at(0), reach(2));
    g[C][0] = blockage(B, hold_at(1), reach(2));
    g[C][1] = blockage(A, hold_at(1), pass(3));

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= C; ++i)
      CHECK(no_gridlock->dependencies().count(i));

    State state;
    state[A] = range(0, 1);
    state[B] = range(0, 0);
    state[C] = range(0, 0);
    CHECK(no_gridlock->evaluate(state));

    state[A] = range(2, 3);
    CHECK(no_gridlock->evaluate(state));

    state[B] = range(0, 1);
    CHECK(no_gridlock->evaluate(state));

    state[C] = range(0, 1);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(2, 2);
    CHECK_FALSE(no_gridlock->evaluate(state));

    state[A] = range(3, 3);
    CHECK_FALSE(no_gridlock->evaluate(state));

    // This violates the constraint g(A3), but the gridlock constraint
    // should be okay with it
    state[A] = range(3, 4);
    CHECK(no_gridlock->evaluate(state));
    CHECK_FALSE(g[A][3]->evaluate(state));

    state[B] = range(0, 0);

    simulate_all_sequences(g, no_gridlock, {5, 2, 2});
  }

  GIVEN("Three-way standoff with slash")
  {
    g[A][0] = blockage(D, hold_at(0), reach(1));
    g[A][1] = blockage(C, hold_at(1), reach(2));
    g[A][2] = blockage(B, hold_at(0), reach(2));
    g[B][0] = blockage(A, hold_at(2), reach(3));
    g[B][1] = blockage(C, hold_at(0), reach(2));
    g[C][0] = blockage(B, hold_at(1), reach(2));
    g[C][1] =
        And(
          blockage(A, hold_at(1), reach(3)),
          blockage(D, std::nullopt, reach(1))
        );
    g[D][0] =
        And(
          blockage(A, hold_at(0), reach(2)),
          blockage(C, hold_at(1), std::nullopt)
        );

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= D; ++i)
      CHECK(no_gridlock->dependencies().count(i));

    simulate_all_sequences(g, no_gridlock, {3, 2, 2, 1});
  }

  GIVEN("Three-way standoff with an interior cross slash")
  {
    g[A][0] = blockage(C, hold_at(1), reach(2));
    g[A][1] =
        And(
          blockage(B, hold_at(0), reach(2)),
          blockage(D, hold_at(0), reach(2))
        );
    g[B][0] = blockage(A, hold_at(1), reach(2));
    g[B][1] = blockage(C, hold_at(0), reach(2));
    g[C][0] = blockage(B, hold_at(1), reach(2));
    g[C][1] = blockage(A, hold_at(0), reach(2));
    g[D][0] = blockage(A, hold_at(1), reach(2));
    g[D][1] = blockage(E, hold_at(0), reach(1));
    g[E][0] = blockage(D, hold_at(1), reach(2));

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    for (std::size_t i=A; i <= D; ++i)
      CHECK(no_gridlock->dependencies().count(i));

    CHECK_FALSE(no_gridlock->dependencies().count(E));

    simulate_all_sequences(g, no_gridlock, {2, 2, 2, 2, 1});
  }

  GIVEN("Multiple head-to-head conflicts between two participants")
  {
    g[A][1] = blockage(B, hold_at(2), pass(3));
    g[A][5] = blockage(B, std::nullopt, reach(2));
    g[B][0] = blockage(A, hold_at(5), std::nullopt);
    g[B][2] = blockage(A, hold_at(1), pass(3));

    ConstConstraintPtr no_gridlock = compute_gridlock_constraint(g);
    CHECK(no_gridlock->dependencies().empty());

    simulate_all_sequences(g, no_gridlock, {8, 4});
  }
}
