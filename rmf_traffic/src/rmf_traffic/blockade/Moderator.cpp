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

#include <rmf_utils/Modular.hpp>

#include "conflicts.hpp"

#include <deque>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Moderator::Assignments::Implementation
{
public:

  static Assignments make()
  {
    Assignments output;
    output._pimpl = rmf_utils::make_impl<Implementation>();
    return output;
  }

  static Implementation& get(Assignments& obj)
  {
    return *obj._pimpl;
  }

  static const Implementation& get(const Assignments& obj)
  {
    return *obj._pimpl;
  }

  static void increment(Assignments& obj)
  {
    ++obj._pimpl->version;
  }

  std::size_t version;
  std::unordered_map<ParticipantId, ReservedRange> ranges;
};

//==============================================================================
Moderator::Assignments::Assignments()
{
  // Do nothing
}

//==============================================================================
class Moderator::Implementation
{
public:

  struct ReservationInfo
  {
    ReservationId id;
    Reservation reservation;

    ReservationInfo(ReservationId id_)
    : id(id_)
    {
      // Do nothing
    }
  };

  struct ReadyInfo
  {
    ParticipantId participant_id;
    ReservationId reservation_id;
    CheckpointId checkpoint;
  };

  double min_conflict_angle;

  std::deque<ReadyInfo> ready_queue;

  std::unordered_map<ParticipantId, ReservationInfo> last_known_reservation;
  Assignments assignments;
  std::unordered_map<ParticipantId, Status> statuses;

  PeerToPeerBlockers peer_blockers;
  Blockers should_go;

  Implementation(const double min_conflict_angle_)
    : min_conflict_angle(min_conflict_angle_),
      assignments(Assignments::Implementation::make())
  {
    // Do nothing
  }

  void process_ready_queue()
  {
    // TODO(MXG): Implement this function
    return
  }

  void set(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const Reservation& reservation)
  {
    const auto insertion = last_known_reservation.insert(
      {participant_id, reservation_id});
    const bool inserted = insertion.second;
    const auto& r_it = insertion.first;
    auto& current_reservation = r_it->second;

    if (!inserted)
    {
      const auto current_id = current_reservation.id;
      if (rmf_utils::modular(reservation_id).less_than_or_equal(current_id))
        return;

      current_reservation.id = reservation_id;
    }

    current_reservation.reservation = reservation;

    const auto peer_insertion = peer_blockers.insert({participant_id, {}});
    const auto peer_inserted = peer_insertion.second;
    const auto peer_it = peer_insertion.first;
    if (!peer_inserted)
      peer_it->second.clear();

    for (const auto& other_r : last_known_reservation)
    {
      const auto other_participant = other_r.first;
      if (other_participant == participant_id)
        continue;

      const auto this_constraint_it = peer_it->second.insert_or_assign(
            other_participant, IndexToConstraint{});
      auto& this_constraint_map = this_constraint_it.first->second;

      const auto& other_reservation = other_r.second.reservation;

      auto& other_peer_map = peer_blockers[other_participant];
      const auto other_peer_it = other_peer_map.insert_or_assign(
            participant_id, IndexToConstraint{});
      auto& other_constraint_map = other_peer_it.first->second;

      const auto conflict_brackets = compute_conflict_brackets(
            reservation.path, reservation.radius,
            other_reservation.path, other_reservation.radius,
            min_conflict_angle);

      auto zero_order_blockers = compute_blockers(
            conflict_brackets,
            participant_id, reservation.path.size(),
            other_participant, other_reservation.path.size());

      this_constraint_map = std::move(zero_order_blockers.at(0));
      other_constraint_map = std::move(zero_order_blockers.at(1));
    }

    should_go = compute_final_ShouldGo_constraints(peer_blockers);

    Assignments::Implementation::get(assignments).ranges
        .insert_or_assign(participant_id, ReservedRange{0, 0});
    Assignments::Implementation::increment(assignments);

    statuses[participant_id] = Status{reservation_id, std::nullopt, 0};

    process_ready_queue();
  }

  void ready(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      const CheckpointId checkpoint)
  {
    const auto r_it = last_known_reservation.find(participant_id);
    if (r_it == last_known_reservation.end())
      return;

    if (r_it->second.id != reservation_id)
      return;

    const auto& path = r_it->second.reservation.path;
    if (path.empty())
      return;

    if (checkpoint >= path.size() - 1)
      return;

    auto& status = statuses.at(participant_id);
    if (checkpoint <= status.last_ready)
      return;

    status.last_ready = checkpoint;
    ready_queue.push_back(
          ReadyInfo{participant_id, reservation_id, checkpoint});

    process_ready_queue();
  }

  void reached(
      const ParticipantId participant_id,
      const ReservationId reservation_id,
      CheckpointId checkpoint)
  {
    const auto r_it = last_known_reservation.find(participant_id);
    if (r_it == last_known_reservation.end())
      return;

    if (r_it->second.id != reservation_id)
      return;

    const auto& path = r_it->second.reservation.path;
    // TODO(MXG): Should this trigger a warning or exception?
    if (checkpoint >= path.size())
      checkpoint = path.size()-1;

    auto& status = statuses.at(participant_id);
    if (checkpoint <= status.last_reached)
      return;

    status.last_reached = checkpoint;

    auto& range = Assignments::Implementation::get(assignments)
        .ranges.at(participant_id);

    range.begin = checkpoint;

    // TODO(MXG): Should this trigger a warning or exception?
    assert(range.begin <= range.end);

    process_ready_queue();
  }

  void cancel(
      const ParticipantId participant_id,
      const ReservationId reservation_id)
  {
    const auto r_it = last_known_reservation.find(participant_id);
    if (r_it == last_known_reservation.end())
      return;

    if (reservation_id < r_it->second.id)
      return;

    cancel(participant_id);
  }

  void cancel(const ParticipantId participant_id)
  {
    last_known_reservation.erase(participant_id);
    statuses.erase(participant_id);
    peer_blockers.erase(participant_id);
    Assignments::Implementation::get(assignments).ranges.erase(participant_id);

    for (auto& peer : peer_blockers)
      peer.second.erase(participant_id);

    should_go = compute_final_ShouldGo_constraints(peer_blockers);

    process_ready_queue();
  }
};

//==============================================================================
Moderator::Moderator(const double min_conflict_angle)
  : _pimpl(rmf_utils::make_impl<Implementation>(min_conflict_angle))
{
  // Do nothing
}

//==============================================================================
void Moderator::set(
    const ParticipantId participant_id,
    const ReservationId reservation_id,
    const Reservation& reservation)
{
  _pimpl->set(participant_id, reservation_id, reservation);
}

//==============================================================================
void Moderator::ready(
    const ParticipantId participant_id,
    const ReservationId reservation_id,
    const CheckpointId checkpoint)
{
  _pimpl->ready(participant_id, reservation_id, checkpoint);
}

//==============================================================================
void Moderator::reached(
    const ParticipantId participant_id,
    const ReservationId reservation_id,
    const CheckpointId checkpoint)
{
  _pimpl->reached(participant_id, reservation_id, checkpoint);
}

//==============================================================================
void Moderator::cancel(
    const ParticipantId participant_id,
    const ReservationId reservation_id)
{
  _pimpl->cancel(participant_id, reservation_id);
}

//==============================================================================
void Moderator::cancel(const ParticipantId participant_id)
{
  _pimpl->cancel(participant_id);
}

//==============================================================================
double Moderator::minimum_conflict_angle() const
{
  return _pimpl->min_conflict_angle;
}

//==============================================================================
Moderator& Moderator::minimum_conflict_angle(const double new_value)
{
  _pimpl->min_conflict_angle = new_value;
  return *this;
}

//==============================================================================
const Moderator::Assignments& Moderator::assignments() const
{
  return _pimpl->assignments;
}

//==============================================================================
const std::unordered_map<ParticipantId, Status>& Moderator::statuses() const
{
  return _pimpl->statuses;
}

} // namespace blockade
} // namespace rmf_traffic
