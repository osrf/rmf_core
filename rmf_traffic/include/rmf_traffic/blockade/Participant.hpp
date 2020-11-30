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

#ifndef RMF_TRAFFIC__BLOCKADE__PARTICIPANT_HPP
#define RMF_TRAFFIC__BLOCKADE__PARTICIPANT_HPP

#include <rmf_traffic/blockade/Rectifier.hpp>
#include <rmf_traffic/blockade/Writer.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Participant
{
public:

  /// Change the radius for this participant. This will only take effect when a
  /// new path is set using the set() function.
  void radius(double new_radius);

  /// Get the radius that's being used for this participant.
  double radius() const;

  /// Set the path for this participant.
  ///
  /// \param[in] path
  ///   The path that this participant intends to follow.
  void set(std::vector<Writer::Checkpoint> path);

  /// Get the current path for this participant.
  const std::vector<Writer::Checkpoint>& path() const;

  /// Tell the blockade writer that the participant is ready to depart from the
  /// given checkpoint.
  void ready(CheckpointId checkpoint);

  /// Tell the blockade writer that the participant is releasing its departure
  /// from the given checkpoint.
  void release(CheckpointId checkpoint);

  /// Get the last checkpoint that this participant said it is ready to depart
  /// from.
  std::optional<CheckpointId> last_ready() const;

  /// Tell the blockade writer that the participant has reached the given
  /// checkpoint.
  void reached(CheckpointId checkpoint);

  /// Get the last checkpoint that this participant said it has reached.
  CheckpointId last_reached() const;

  /// Get the ID that was assigned to this participant.
  ParticipantId id() const;

  /// Get the current reservation ID
  std::optional<ReservationId> reservation_id() const;

  class Implementation;
private:
  Participant();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Make a blockade participant.
///
/// \param[in] participant_id
///   Every blockade participant must also be a schedule participant. Pass in
///   the schedule participant ID here.
///
/// \param[in] radius
///   The initial default radius to use for this participant's blockade.
///
/// \param[in] writer
///   The writer that this participant should interact with.
///
/// \param[in] rectifier_factory
///   The factory that this participant should use to create a rectifier for
///   itself. If no factory is provided, we will assume the writer is always
///   perfectly reliable.
Participant make_participant(
    ParticipantId participant_id,
    double radius,
    std::shared_ptr<Writer> writer,
    std::shared_ptr<RectificationRequesterFactory> rectifier_factory = nullptr);

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__PARTICIPANT_HPP
