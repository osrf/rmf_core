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

  //============================================================================
  // Writer API
  //============================================================================

  // Documentation inherited
  void set(
      ParticipantId participant_id,
      ReservationId reservation_id,
      const Reservation& reservation) final;

  // Documentation inherited
  void ready(
      ParticipantId participant_id,
      ReservationId reservation_id,
      CheckpointId checkpoint) final;

  // Documentation inherited
  void reached(
      ParticipantId participant_id,
      ReservationId reservation_id,
      CheckpointId checkpoint) final;

  // Documentation inherited
  void cancel(
      ParticipantId participant_id,
      ReservationId reservation_id) final;

  // Documentation inherited
  void cancel(ParticipantId participant_id) final;


  //============================================================================
  // Moderator API
  //============================================================================

  /// Default constructor
  ///
  /// \param[in] info_logger
  ///   Provide a callback for logging informational updates about changes in
  ///   the blockades, e.g. when a new path arrives, when a checkpoint is
  ///   reached, or when one is ready.
  ///
  /// \param[in] debug_logger
  ///   Provide a callback for logging debugging information, e.g. which
  ///   constraints are blocking a participant from advancing.
  ///
  /// \param[in] min_conflict_angle
  ///   If the angle between two path segments is greater than this value
  ///   (radians), then the segments are considered to be in conflict. The
  ///   default value for this parameter is 5-degrees. Something larger than 0
  ///   is recommended to help deal with numerical precision concerns.
  Moderator(
      std::function<void(std::string)> info_logger = nullptr,
      std::function<void(std::string)> debug_logger = nullptr,
      double min_conflict_angle = 5.0*M_PI/180.0);

  /// Get the minimum angle that will trigger a conflict.
  double minimum_conflict_angle() const;

  /// Set the minimum angle that will trigger a conflict.
  Moderator& minimum_conflict_angle(double new_value);

  /// Set the information logger for this Moderator. Pass in a nullptr to
  /// disable any information logging.
  Moderator& info_logger(std::function<void(std::string)> info);

  /// Set the debug logger for this Moderator. Pass in a nullptr to disable
  /// any debug logging.
  Moderator& debug_logger(std::function<void(std::string)> debug);

  /// This class indicates the range of each reservation that the blockade
  /// moderator has assigned as active. Each robot is allowed to move at will
  /// from the begin checkpoint to the end checkpoint in the range assigned for
  /// it.
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
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the current set of assignments.
  const Assignments& assignments() const;

  /// Get the current known statuses of each participant.
  const std::unordered_map<ParticipantId, Status>& statuses() const;

  /// Return true if the system is experiencing a gridlock
  bool has_gridlock() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__MODERATOR_HPP
