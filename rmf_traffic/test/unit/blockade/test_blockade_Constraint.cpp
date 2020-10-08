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

#include <algorithm>

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
struct StateHash
{
  StateHash(std::size_t highest_goal)
  {
    _edge_shift = std::ceil(std::log2(highest_goal+1));
    _participant_shift = 2*_edge_shift;
  }

  std::size_t operator()(const rmf_traffic::blockade::State& state) const
  {
    std::size_t output = 0;
    for (const auto& s : state)
    {
      const std::size_t participant = s.first;
      const std::size_t begin = s.second.begin;
      const std::size_t end = s.second.end;

      if (begin == end)
        output += end << (_participant_shift*participant);
      else
        output += (end << _edge_shift) << (_participant_shift*participant);
    }

    return output;
  }

  std::size_t _edge_shift;
  std::size_t _participant_shift;
};

//==============================================================================
rmf_traffic::blockade::State make_initial_state(
    const std::size_t N_participants)
{
  rmf_traffic::blockade::State state;
  for (std::size_t i=0; i < N_participants; ++i)
    state[i] = rmf_traffic::blockade::ReservedRange{0, 0};

  return state;
}

//==============================================================================
bool have_reached_goal(
    const rmf_traffic::blockade::State& state,
    const std::vector<std::size_t>& goal)
{
  for (const auto& s : state)
  {
    REQUIRE(s.second.begin <= s.second.end);
    REQUIRE(s.second.end <= goal.at(s.first));
  }

  for (const auto& s : state)
  {
    if (s.second.begin != s.second.end)
      return false;

    if (s.second.end != goal.at(s.first))
      return false;
  }

  return true;
}

//==============================================================================
/// This function simulates all possible command sequences
void simulate_all_sequences(
    const rmf_traffic::blockade::Blockers& blockers,
    const rmf_traffic::blockade::ConstConstraintPtr& no_gridlock,
    const std::vector<std::size_t>& goal)
{
  using namespace rmf_traffic::blockade;
  REQUIRE(!goal.empty());

  std::unordered_set<State, StateHash> cache(
        0, StateHash(*std::max_element(goal.begin(), goal.end())));

  std::vector<IndexToConstraint> should_go;
  should_go.resize(goal.size(), {});
  for (const auto& b : blockers)
  {
    auto& G = should_go.at(b.first);
    for (const auto& g : b.second)
      G[g.first] = And(no_gridlock, g.second);
  }

  std::vector<State> queue;
  queue.push_back(make_initial_state(goal.size()));

  while (!queue.empty())
  {
    const auto top = queue.back();
    queue.pop_back();

    if (!cache.insert(top).second)
      continue;

    if (have_reached_goal(top, goal))
      continue;

    std::size_t choices = 0;
    for (const auto& s : top)
    {
      const std::size_t participant = s.first;
      const std::size_t begin = s.second.begin;
      const std::size_t end = s.second.end;

      State next = top;
      if (begin == end)
      {
        if (end == goal[participant])
          continue;

        ++next[participant].end;
      }
      else
      {
        ++next[participant].begin;
      }

      bool okay = true;
      const auto& np = next.at(participant);
      const auto& constraints = should_go.at(participant);
      for (std::size_t i = np.begin; i < np.end; ++i)
      {
        const auto it = constraints.find(i);
        if (it != constraints.end() && !it->second->evaluate(next))
          okay = false;
      }

      if (!okay)
        continue;

      queue.push_back(next);
      ++choices;
    }

    CHECK(choices > 0);
  }
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
}
