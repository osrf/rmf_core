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

#ifndef RMF_TRAFFIC__BLOCKADE__MODERATOR_HPP
#define RMF_TRAFFIC__BLOCKADE__MODERATOR_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_traffic/blockade/Writer.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Moderator : public Writer
{
public:

  /// Default constructor
  Moderator();

  void set(
      ParticipantId participant_id,
      ReservationId reservation_id,
      const Reservation& reservation) final;

  void ready(
      ParticipantId participant_id,
      ReservationId reservation_id,
      CheckpointId checkpoint) final;

  void reached(
      ParticipantId participant_id,
      ReservationId reservation_id,
      CheckpointId checkpoint) final;

  void cancel(
      ParticipantId participant_id,
      ReservationId reservation_id) final;

  class Assignments
  {
  public:

    /// Get the version of the current assignment sets. The version number will
    /// increase by at least 1 each time the assignments change. This can be
    /// used to identify when new assignment notifications are necessary.
    std::size_t version() const;

    /// Get the ranges that are assigned to each participant.
    const std::unordered_map<ParticipantId, ReservedRange>& ranges() const;

    class Implementation;
  private:
    Assignments();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  /// Get the current set of assignments.
  const Assignments& assignments() const;

  const std::unordered_map<ParticipantId, Status>& statuses() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__MODERATOR_HPP
