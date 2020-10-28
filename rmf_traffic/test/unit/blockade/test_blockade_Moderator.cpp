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
SCENARIO("Test blockade moderator")
{
  using namespace rmf_traffic::blockade;

  std::shared_ptr<Writer> writer;
  std::shared_ptr<ModeratorRectificationRequesterFactory> rectifier_factory;
  GIVEN("Reliable moderator")
  {
    writer = std::make_shared<Moderator>();
  }

  GIVEN("Unreliable moderator")
  {
    const auto unreliable_moderator = std::make_shared<UnreliableModerator>();
    rectifier_factory =
        std::make_shared<ModeratorRectificationRequesterFactory>(
          unreliable_moderator->real_moderator());

    writer = unreliable_moderator;
  }



}
