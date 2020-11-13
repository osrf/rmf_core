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

#include <rmf_traffic/blockade/Moderator.hpp>
#include <rmf_traffic/blockade/Participant.hpp>

#include <rmf_utils/catch.hpp>

#include <random>

#include "utils_blockade_scenarios.hpp"

//==============================================================================
class UnreliableModerator : public rmf_traffic::blockade::Writer
{
public:

  using ParticipantId = rmf_traffic::blockade::ParticipantId;
  using ReservationId = rmf_traffic::blockade::ReservationId;
  using CheckpointId = rmf_traffic::blockade::CheckpointId;
  using Status = rmf_traffic::blockade::Status;
  using Assignments = rmf_traffic::blockade::Moderator::Assignments;

  UnreliableModerator(const double success_rate = 0.2)
    : _real_moderator(std::make_shared<rmf_traffic::blockade::Moderator>()),
      _success_rate(success_rate)
  {
    // Do nothing
  }

  void set(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const Reservation& reservation) final
  {
    if (_dist(_rng) < _success_rate)
      _real_moderator->set(participant_id, reservation_id, reservation);
  }

  void ready(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const CheckpointId checkpoint) final
  {
    if (_dist(_rng) < _success_rate)
      _real_moderator->ready(participant_id, reservation_id, checkpoint);
  }

  void reached(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const CheckpointId checkpoint) final
  {
    if (_dist(_rng) < _success_rate)
      _real_moderator->reached(participant_id, reservation_id, checkpoint);
  }

  void cancel(ParticipantId participant_id, ReservationId reservation_id) final
  {
    if (_dist(_rng) < _success_rate)
      _real_moderator->cancel(participant_id, reservation_id);
  }

  void cancel(ParticipantId participant_id) final
  {
    if (_dist(_rng) < _success_rate)
      _real_moderator->cancel(participant_id);
  }

  const std::shared_ptr<rmf_traffic::blockade::Moderator> real_moderator() const
  {
    return _real_moderator;
  }

private:
  std::shared_ptr<rmf_traffic::blockade::Moderator> _real_moderator;
  std::default_random_engine _rng = std::default_random_engine(33);
  std::uniform_real_distribution<double> _dist =
      std::uniform_real_distribution<double>(0,1);
  double _success_rate;
};

//==============================================================================
class SimParticipant
{
public:

  using Factory = rmf_traffic::blockade::RectificationRequesterFactory;

  SimParticipant(
      const rmf_traffic::blockade::ParticipantId participant_id,
      const double radius,
      std::shared_ptr<rmf_traffic::blockade::Writer> writer,
      std::shared_ptr<Factory> factory,
      std::vector<rmf_traffic::blockade::Writer::Checkpoint> path,
      const std::size_t steps_per_waypoint = 3)
    : _participant(
        rmf_traffic::blockade::make_participant(
          participant_id, radius, std::move(writer), std::move(factory))),
      _steps_per_waypoint(steps_per_waypoint)
  {
    _participant.set(std::move(path));
    REQUIRE(!_participant.path().empty());
  }

  void step(const rmf_traffic::blockade::ReservedRange& range)
  {
    if (_participant.path().size() <= _participant.last_reached()+1)
      return;

    if (range.end == _participant.last_reached())
    {
      if (_participant.last_ready() != range.end)
        _participant.ready(range.end);

      return;
    }

    if (_current_steps < _steps_per_waypoint)
    {
      ++_current_steps;
      return;
    }

    _current_steps = 0;
    const std::size_t next = _participant.last_reached()+1;
    _participant.reached(next);
  }

  const rmf_traffic::blockade::Participant& participant() const
  {
    return _participant;
  }

private:
  rmf_traffic::blockade::Participant _participant;
  const std::size_t _steps_per_waypoint;
  std::size_t _current_steps = 0;
};

//==============================================================================
using ModeratorRectFactory =
  rmf_traffic::blockade::ModeratorRectificationRequesterFactory;

//==============================================================================
struct ModeratorContext
{
  std::shared_ptr<rmf_traffic::blockade::Moderator> moderator;
  std::shared_ptr<rmf_traffic::blockade::Writer> writer;
  std::shared_ptr<ModeratorRectFactory> rectifier_factory;
};

//==============================================================================
ModeratorContext make_reliable()
{
  ModeratorContext context;
  context.moderator = std::make_shared<rmf_traffic::blockade::Moderator>();
  context.writer = context.moderator;

  return context;
}

//==============================================================================
ModeratorContext make_unreliable()
{
  ModeratorContext context;
  const auto unreliable_moderator = std::make_shared<UnreliableModerator>();
  context.rectifier_factory =
      std::make_shared<ModeratorRectFactory>(
        unreliable_moderator->real_moderator());

  context.moderator = unreliable_moderator->real_moderator();
  context.writer = unreliable_moderator;

  return context;
}

//==============================================================================
bool finished(const std::vector<SimParticipant>& participants)
{
  for (const auto& p : participants)
  {
    if (p.participant().last_reached() < p.participant().path().size() - 1)
      return false;
  }

  return true;
}

//==============================================================================
using MapOfRanges =
  std::unordered_map<uint64_t, rmf_traffic::blockade::ReservedRange>;

//==============================================================================
bool all_assignments_reached_end(const MapOfRanges& ranges)
{
  for (const auto& r : ranges)
  {
    if (r.second.begin < r.second.end)
      return false;
  }

  return true;
}

//==============================================================================
void simulate_moderator(
    ModeratorContext context,
    GridlockScenario scenario,
    const double radius)
{
  std::vector<SimParticipant> participants;
  for (std::size_t i=0; i < scenario.paths.size(); ++i)
  {
    participants.emplace_back(
          i, radius, context.writer, context.rectifier_factory,
          std::move(scenario.paths[i]), 3);
  }

  while (!finished(participants))
  {
    const auto& assignments = context.moderator->assignments();

    for (auto& p : participants)
    {
      const auto r_it = assignments.ranges().find(p.participant().id());
      if (r_it != assignments.ranges().end())
      {
        const auto& range = r_it->second;
        p.step(range);
        CHECK(p.participant().last_reached() <= range.end);
      }
    }

    if (context.rectifier_factory)
      context.rectifier_factory->rectify();
  }

  while (!all_assignments_reached_end(
           context.moderator->assignments().ranges()))
  {
    // Assignments should only ever fail to reach the end if this is an
    // unreliable moderator.
    REQUIRE(context.rectifier_factory);

    // Since this is an unreliable moderator, we will continue to rectify until
    // the beginning of the range reaches the end.
    if (context.rectifier_factory)
      context.rectifier_factory->rectify();
  }

  // If the test eventually ends then it was successful
}

//==============================================================================
SCENARIO("Test blockade moderator")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;

  GridlockScenario scenario;
  ModeratorContext context;

  GIVEN("4-way standoff")
  {
    scenario = fourway_standoff();

    WHEN("Reliable moderator")
    {
      context = make_reliable();
    }

    WHEN("Unreliable moderator")
    {
      context = make_unreliable();
    }
  }

  GIVEN("Flyby U-turn")
  {
    scenario = flyby_uturn();

    WHEN("Reliable moderator")
    {
      context = make_reliable();
    }

    WHEN("Unreliable moderator")
    {
      context = make_unreliable();
    }
  }

  GIVEN("3-way standoff with one redundant leg")
  {
    scenario = threeway_standoff_with_redundant_leg();

    WHEN("Reliable moderator")
    {
      context = make_reliable();
    }

    WHEN("Unreliable moderator")
    {
      context = make_unreliable();
    }
  }

  GIVEN("3-way standoff with additional conflict")
  {
    scenario = threeway_standoff_with_additional_conflict();

    WHEN("Reliable moderator")
    {
      context = make_reliable();
    }

    WHEN("Unreliable moderator")
    {
      context = make_unreliable();
    }
  }

  GIVEN("Criss-crossing paths")
  {
    scenario = crisscrossing_paths();

    WHEN("Reliable moderator")
    {
      context = make_reliable();
    }

    WHEN("Unreliable moderator")
    {
      context = make_unreliable();
    }
  }

  REQUIRE(context.writer);
  REQUIRE(context.moderator);

  simulate_moderator(std::move(context), std::move(scenario), radius);
}

//==============================================================================
SCENARIO("Test lane sharing")
{
  std::array<Eigen::Vector2d, 6> A;
  A[0] = { 0, 0};
  A[1] = { 5, 0};
  A[2] = {10, 0};
  A[3] = {15, 0};
  A[4] = {20, 0};
  A[5] = {25, 0};
  const auto path_A = make_path(A);

  std::array<Eigen::Vector2d, 5> B;
  B[0] = {10, 5};
  B[1] = {10, 0};
  B[2] = {15, 0};
  B[3] = {20, 0};
  B[4] = {18, 5};
  const auto path_B = make_path(B);

  const auto moderator = std::make_shared<rmf_traffic::blockade::Moderator>();
  auto p_A = rmf_traffic::blockade::make_participant(0, 0.1, moderator);
  auto p_B = rmf_traffic::blockade::make_participant(1, 0.1, moderator);

  p_A.set(path_A);
  p_B.set(path_B);

  p_B.ready(1);
  REQUIRE(moderator->assignments().ranges().at(1).end == 2);
  p_B.reached(2);
  REQUIRE(moderator->assignments().ranges().at(1).begin == 2);
  REQUIRE(moderator->assignments().ranges().at(1).end == 2);

  p_A.ready(4);
  // The moderator should know to stop A by A3 so that B can make the turn
  // towards B4
  CHECK(moderator->assignments().ranges().at(0).end == 3);

  p_B.ready(2);
  CHECK(moderator->assignments().ranges().at(1).end == 3);
}
