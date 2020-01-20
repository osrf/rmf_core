/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
#define RMF_TRAFFIC__SCHEDULE__DATABASE_HPP

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Patch.hpp>

#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class that maintains a database of scheduled Trajectories. This class is
/// intended to be used only for the canonical RMF traffic schedule database.
/// All local mirror copy
///
/// The Viewer API can be queried to find Trajectories that match certain
/// criteria.
///
/// You can also retrieve update patches from a database. To apply those patches
/// to a downstream Viewer, it is strongly advised to use the
/// rmf_traffic::schedule::Mirror class.
class Database : public Viewer
{
public:

  /// Initialize a Database
  Database();

  /// Get the changes in this Database that match the given Query parameters.
  Patch changes(const Query& parameters) const;

  //============================================================================
  /// The ItineraryVersion number is used to identify the current version of a
  /// participant's itinerary. This will be used to ensure consistency between
  /// a schedule database's information and the updates that are sent by remote
  /// participants.
  ///
  /// As updates stream in from the remote participants, the updates will be
  /// stamped with itinerary version numbers. If a version is missing from the
  /// sequence of updates for a participant, then the schedule will send out a
  /// request for a fresh Put for that participant.
  struct ItineraryVersion
  {
    /// The most recent version that the remote participant knows the schedule
    /// contains.
    Version base_version;

    /// The version that the remote participant has assigned to this change.
    Version change_version;
  };

  //============================================================================
  /// The ChangeResult is produced by the functions below which modify
  /// itineraries in the database.
  struct ChangeResult
  {
    /// The new version of the overall schedule.
    Version version;

    /// True if the result should be consistent with the remote participant's
    /// intended itinerary. False if the schedule may have gotten out of sync
    /// with the remote participant and therefore needs a new put or erase.
    bool consistent;
  };

  /// Put in a brand new itinerary for a participant. This will replace any
  /// itinerary that is already in the schedule for the participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] itinerary
  ///   The new itinerary of the participant.
  ///
  /// \param[in] version
  ///   The version for the itinerary that is being put into the schedule.
  ///
  /// \return the result of making this change. If consistent is false, that
  /// means the version of this put is lower than the latest itinerary version
  /// that is in this schedule.
  ChangeResult put(
      ParticipantId participant,
      Itinerary itinerary,
      ItineraryVersion version);

  /// Add a route to the itinerary of this participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] route
  ///   The route that should be added to the itinerary.
  ///
  /// \param[in] version
  ///   The version of the itinerary that should result from this change.
  ///
  /// \return the result of making this change.
  ChangeResult post(
      ParticipantId participant,
      Route route,
      ItineraryVersion version);

  /// Add a delay to the itinerary from the specified Time.
  ///
  /// Nothing about the routes in the itinerary will be changed except that
  /// waypoints which come after the specified time will be pushed back by the
  /// specified delay.
  ///
  /// \note This can create distortions in the Trajectory movement that leads
  /// up to the `from` Time, so use with caution. This is primarily intended to
  /// make corrections to live Trajectories based on incoming state information.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] from
  ///   All Trajectory Waypoints that end after this time point will be pushed
  ///   back by the delay.
  ///
  /// \param[in] delay
  ///   This is the duration of time to delay all qualifying Trajectory
  ///   Waypoints.
  ///
  /// \return the result of making this change.
  ChangeResult delay(
      ParticipantId participant,
      Time from,
      Duration delay,
      ItineraryVersion version);

  /// Erase an itinerary from this database.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \return the result of making this change. If consistent is false, that
  /// means the version of this erasure is lower than the latest itinerary
  /// version that is in this schedule.
  ChangeResult erase(ParticipantId participant);

  /// Erase a route from an itinerary.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] routes
  ///   The indices of the routes that should be erased.
  ///
  /// \return the result of making this change. If consistent is false, that
  /// means the version of this erasure is lower than the latest itinerary.
  ChangeResult erase(
      ParticipantId participant,
      const std::vector<RouteId>& routes);

  /// Throw away all itineraries up to the specified time.
  ///
  /// \param[in] time
  ///   All Trajectories that finish before this time will be culled from the
  ///   Database. Their data will be completely deleted from this Database
  ///   object.
  ///
  /// \return The new version of the schedule database. If nothing was culled,
  /// this version number will remain the same.
  Version cull(Time time);

  /// The return structure from registering a participant.
  struct RegistrationResult
  {
    /// The new version of the schedule after registering the participant.
    Version version;

    /// The ID of the newly registered participant.
    ParticipantId id;
  };

  /// Register a new participant.
  ///
  /// \param[in] participant_info
  ///   Information about the new participant.
  ///
  /// \return result of registering the new participant.
  RegistrationResult register_participant(Participant participant_info);

  /// Unregister an existing participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant to unregister.
  ///
  /// \return the new version of the schedule.
  Version unregister_participant(ParticipantId participant);

};

} // namespace schedule

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__DATABASE_HPP
