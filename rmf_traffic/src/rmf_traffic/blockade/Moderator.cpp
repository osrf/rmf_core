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

#include <list>
#include <sstream>

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

  static Implementation& modify(Assignments& obj)
  {
    ++obj._pimpl->version;
    return *obj._pimpl;
  }

  static const Implementation& get(const Assignments& obj)
  {
    return *obj._pimpl;
  }

  std::size_t version;
  std::unordered_map<ParticipantId, ReservedRange> ranges;
};

//==============================================================================
std::size_t Moderator::Assignments::version() const
{
  return _pimpl->version;
}

//==============================================================================
const std::unordered_map<ParticipantId, ReservedRange>&
Moderator::Assignments::ranges() const
{
  return _pimpl->ranges;
}

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

  std::function<void(std::string)> info_logger;
  std::function<void(std::string)> debug_logger;
  double min_conflict_angle;

  std::list<ReadyInfo> ready_queue;

  std::unordered_map<ParticipantId, ReservationInfo> last_known_reservation;
  Assignments assignments;
  std::unordered_map<ParticipantId, Status> statuses;

  PeerToPeerBlockers peer_blockers;
  PeerToPeerAlignment peer_alignment;
  FinalConstraints final_constraints;

  Implementation(
      std::function<void(std::string)> info,
      std::function<void(std::string)> debug,
      const double min_conflict_angle_)
    : info_logger(std::move(info)),
      debug_logger(std::move(debug)),
      min_conflict_angle(min_conflict_angle_),
      assignments(Assignments::Implementation::make())
  {
    // Do nothing
  }

  enum ReadyStatus
  {
    Skip = 0,
    Incomplete,
    Finished
  };

  ReadyStatus check_reservation(const ReadyInfo& check)
  {
    const auto r_it = last_known_reservation.find(check.participant_id);
    if (r_it == last_known_reservation.end())
      return Finished;

    if (r_it->second.id != check.reservation_id)
      return Finished;

    auto state = assignments.ranges();
    auto& s = state.at(check.participant_id);
    if (check.checkpoint < s.end)
      return Finished;

    const auto constraints_it =
        final_constraints.should_go.find(check.participant_id);
    if (constraints_it == final_constraints.should_go.end())
    {
      // There are no constraints for this participant, so we will just allow it
      // to go all the way.
      Assignments::Implementation::modify(assignments)
          .ranges[check.participant_id].end = check.checkpoint + 1;
      return Finished;
    }

    const auto& constraints = constraints_it->second;

    const std::size_t current_end = s.end;
    const std::size_t i_max = (check.checkpoint+1) - (current_end+1);
    for (std::size_t i=0; i <= i_max; ++i)
    {
      // We will try to expand the range of this participant's reservation,
      // preferably as far out as checkpoint+1, but if that fails then we will
      // incrementally try smaller reservations until we reach current_end+1. If
      // we cannot even reserve current_end+1, then the participant is stuck for
      // now.
      const std::size_t check_end = check.checkpoint+1 - i;
      s.end = check_end;

      // TODO(MXG): We could probably get slightly better performance here if
      // should_go used an ordered std::map instead of std::unordered_map.
      bool acceptable = true;
      for (std::size_t c=s.begin; c < s.end; ++c)
      {
        const auto it = constraints.find(c);
        if (it != constraints.end() && !it->second->evaluate(state))
        {
          if (debug_logger)
          {
            std::stringstream str;
            const std::string P = toul(check.participant_id);
            str << "Cannot reserve [" << P << s.begin
                << " -> " << P << check_end
                << "]. Blocked at " << P << c << " by: "
                << it->second->detail(state);
            debug_logger(str.str());
          }

          acceptable = false;
          break;
        }
      }

      if (acceptable)
      {
        Assignments::Implementation::modify(assignments)
            .ranges[check.participant_id].end = check_end;

        if (i==0)
          return Finished;

        return Incomplete;
      }
    }

    return Skip;
  }

  void process_ready_queue()
  {
    auto next = ready_queue.begin();
    while (next != ready_queue.end())
    {
      const auto result = check_reservation(*next);
      if (result == Finished)
      {
        ready_queue.erase(next);
        next = ready_queue.begin();
      }
      else if (result == Incomplete)
      {
        next = ready_queue.begin();
      }
      else
      {
        ++next;
      }
    }
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

    if (info_logger)
    {
      std::stringstream str;
      str << "New path for " << toul(participant_id) << " with "
          << reservation.path.size() << " checkpoints";
      info_logger(str.str());
    }

    current_reservation.reservation = reservation;

    const auto peer_blocker_insertion =
        peer_blockers.insert({participant_id, {}});
    const auto peer_blocker_inserted = peer_blocker_insertion.second;
    const auto peer_blocker_it = peer_blocker_insertion.first;
    if (!peer_blocker_inserted)
      peer_blocker_it->second.clear();

    const auto peer_aligned_insertion =
        peer_alignment.insert({participant_id, {}});
    const auto peer_aligned_inserted = peer_aligned_insertion.second;
    const auto peer_aligned_it = peer_aligned_insertion.first;
    if (!peer_aligned_inserted)
      peer_aligned_it->second.clear();

    for (const auto& other_r : last_known_reservation)
    {
      const auto other_participant = other_r.first;
      if (other_participant == participant_id)
        continue;

      const auto& other_reservation = other_r.second.reservation;

      const auto brackets = compute_brackets(
            reservation.path, reservation.radius,
            other_reservation.path, other_reservation.radius,
            min_conflict_angle);

      auto zero_order_blockers = compute_blockers(
            brackets.conflicts,
            participant_id, reservation.path.size(),
            other_participant, other_reservation.path.size());

      auto alignments = compute_alignments(brackets.alignments);

      const auto this_blocker_it =
          peer_blocker_it->second.insert_or_assign(
            other_participant, IndexToConstraint{});
      auto& this_blocker_map = this_blocker_it.first->second;
      this_blocker_map = std::move(zero_order_blockers.at(0));

      auto& other_peer_blocker_map = peer_blockers[other_participant];
      const auto other_peer_blocker_it =
          other_peer_blocker_map.insert_or_assign(
            participant_id, IndexToConstraint{});
      auto& other_blocker_map = other_peer_blocker_it.first->second;
      other_blocker_map = std::move(zero_order_blockers.at(1));

      const auto this_aligned_it =
          peer_aligned_it->second.insert_or_assign(
            other_participant, std::vector<Alignment>{});
      auto& this_aligned_map = this_aligned_it.first->second;
      this_aligned_map = std::move(alignments.at(0));

      auto& other_peer_aligned_map = peer_alignment[other_participant];
      const auto other_peer_aligned_it =
          other_peer_aligned_map.insert_or_assign(
            participant_id, std::vector<Alignment>{});
      auto& other_aligned_map = other_peer_aligned_it.first->second;
      other_aligned_map = std::move(alignments.at(1));
    }

    final_constraints = compute_final_ShouldGo_constraints(
          peer_blockers, peer_alignment);

    Assignments::Implementation::modify(assignments).ranges
        .insert_or_assign(participant_id, ReservedRange{0, 0});

    statuses[participant_id] = Status{reservation_id, std::nullopt, 0, false};

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

    if (info_logger)
    {
      info_logger("Ready: "+toul(participant_id)+std::to_string(checkpoint));
    }

    status.last_ready = checkpoint;
    ready_queue.push_back(
          ReadyInfo{participant_id, reservation_id, checkpoint});

    process_ready_queue();
  }

  void release(
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
    if (path.empty())
      return;

    if (checkpoint >= path.size() - 1)
      return;

    auto& status = statuses.at(participant_id);
    if (checkpoint <= status.last_reached)
      checkpoint = status.last_reached;

    std::optional<std::size_t> new_ready;
    if (checkpoint > 0)
      new_ready = checkpoint - 1;

    status.last_ready = new_ready;

    auto& range = Assignments::Implementation::modify(assignments)
        .ranges.at(participant_id);

    if (checkpoint < range.end)
      range.end = checkpoint;

    if (new_ready.has_value())
    {
      for (auto r_it = ready_queue.begin(); r_it != ready_queue.end(); ++r_it)
      {
        if (participant_id != r_it->participant_id)
          continue;

        // This ready entry probably shouldn't exist anyway, but we'll leave
        // that up to process_ready_queue() to deal with.
        if (reservation_id != r_it->reservation_id)
          continue;

        if (checkpoint <= r_it->checkpoint)
          r_it->checkpoint = checkpoint;
      }
    }
    else
    {
      // The participant no longer has any checkpoints that it is ready to move
      // from, so we should erase any entries that this participant has in the
      // ready queue.
      auto r_it = ready_queue.begin();
      while (r_it != ready_queue.end())
      {
        if (participant_id != r_it->participant_id)
        {
          ++r_it;
          continue;
        }

        ready_queue.erase(r_it++);
      }
    }

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

    auto& range = Assignments::Implementation::modify(assignments)
        .ranges.at(participant_id);

    // TODO(MXG): Should this trigger a warning or exception?
    if (checkpoint > range.end)
    {
      const bool had_critical_error = status.critical_error;
      status.critical_error = true;

      if (!had_critical_error)
      {
        std::stringstream str;
        str << "[rmf_traffic::blockade::Participant::reached] Participant ["
            << participant_id << "] reached an invalid checkpoint ["
            << checkpoint << "] when it was only assigned up to [" << range.end
            << "]";

        throw std::runtime_error(str.str());
      }

      return;
    }

    if (info_logger)
    {
      info_logger("Reached: "+toul(participant_id)+std::to_string(checkpoint));
    }

    status.last_reached = checkpoint;

    range.begin = checkpoint;

    process_ready_queue();
  }

  void cancel(
      const ParticipantId participant_id,
      const ReservationId reservation_id)
  {
    const auto r_it = last_known_reservation.find(participant_id);
    if (r_it == last_known_reservation.end())
    {
      return;
    }

    if (reservation_id < r_it->second.id)
    {
      return;
    }

    if (info_logger)
    {
      info_logger("Canceling: " + toul(participant_id));
    }

    cancel(participant_id);
  }

  void cancel(const ParticipantId participant_id)
  {
    if (info_logger)
    {
      info_logger("Canceling: " + toul(participant_id));
    }

    last_known_reservation.erase(participant_id);
    statuses.erase(participant_id);
    peer_blockers.erase(participant_id);
    peer_alignment.erase(participant_id);
    Assignments::Implementation::modify(assignments)
        .ranges.erase(participant_id);

    for (auto& peer : peer_blockers)
      peer.second.erase(participant_id);

    for (auto& peer : peer_alignment)
      peer.second.erase(participant_id);

    final_constraints = compute_final_ShouldGo_constraints(
          peer_blockers, peer_alignment);

    process_ready_queue();
  }
};

//==============================================================================
Moderator::Moderator(
    std::function<void(std::string)> info,
    std::function<void(std::string)> debug,
    const double min_conflict_angle)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             std::move(info),
             std::move(debug),
             min_conflict_angle))
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
void Moderator::release(
    ParticipantId participant_id,
    ReservationId reservation_id,
    CheckpointId checkpoint)
{
  _pimpl->release(participant_id, reservation_id, checkpoint);
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
Moderator& Moderator::info_logger(std::function<void(std::string)> info)
{
  _pimpl->info_logger = std::move(info);
  return *this;
}

//==============================================================================
Moderator& Moderator::debug_logger(std::function<void(std::string)> debug)
{
  _pimpl->debug_logger = std::move(debug);
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

//==============================================================================
bool Moderator::has_gridlock() const
{
  if (!_pimpl->final_constraints.gridlock)
    return false;

  return !_pimpl->final_constraints.gridlock->evaluate(assignments().ranges());
}

} // namespace blockade
} // namespace rmf_traffic
