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

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Moderator::Assignments::Implementation
{
public:

  static Assignments make()
  {
    Assignments output;
    output._pimpl = rmf_utils::make_unique_impl<Implementation>();
    return output;
  }

};

//==============================================================================
class Moderator::Implementation
{
public:

  std::unordered_map<ParticipantId, ReservationId> last_known_reservation;
  Assignments assignments;
  std::unordered_map<ParticipantId, Status> statuses;

  PeerToPeerBlockers peer_blockers;
  Blockers should_go;

  Implementation()
    : assignments(Assignments::Implementation::make())
  {
    // Do nothing
  }

};

//==============================================================================
void Moderator::set(
    ParticipantId participant_id,
    ReservationId reservation_id,
    const Reservation& reservation)
{
  const auto insertion = _pimpl->last_known_reservation.insert(
    {participant_id, reservation_id});
  const bool inserted = insertion.second;

  if (!inserted)
  {
    const auto& r_it = insertion.first;
    if (rmf_utils::modular(reservation_id).less_than_or_equal(r_it->second))
      return;

    r_it->second = reservation_id;
  }

  // TODO(MXG): Finish this;
  return
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
