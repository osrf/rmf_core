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

#ifndef RMF_TRAFFIC__SCHEDULE__WRITER_HPP
#define RMF_TRAFFIC__SCHEDULE__WRITER_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/ParticipantDescription.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A pure abstract interface class that defines an API for writing to the
/// schedule database. This API is implemented by the Database class, but it
/// should also be implemented for any middleware that intends to have a
/// schedule participant write changes to a remote database.
class Writer
{
public:

  /// Put in a brand new itinerary for a participant. This will replace any
  /// itinerary that is already in the schedule for the participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] itinerary
  ///   The new itinerary of the participant.
  ///
  /// \param[in] retransmission
  ///   Set this to true if this is a retransmission of an update that was
  ///   previously lost. Otherwise set it to false.
  ///
  virtual void put(
      ParticipantId participant,
      Itinerary itinerary,
      bool retransmission) = 0;

  /// Add a set of routes to the itinerary of this participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] itinerary
  ///   The set of routes that should be added to the itinerary.
  ///
  /// \param[in] retransmission
  ///   Set this to true if this is a retransmission of an update that was
  ///   previously lost. Otherwise set it to false.
  ///
  virtual void post(
      ParticipantId participant,
      Itinerary itinerary,
      bool retransmission) = 0;

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
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  /// \param[in] retransmission
  ///   Set this to true if this is a retransmission of an update that was
  ///   previously lost. Otherwise set it to false.
  ///
  virtual void delay(
      ParticipantId participant,
      Time from,
      Duration delay,
      ItineraryVersion version,
      bool retransmission) = 0;

  /// Erase an itinerary from this database.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  /// \param[in] retransmission
  ///   Set this to true if this is a retransmission of an update that was
  ///   previously lost. Otherwise set it to false.
  ///
  virtual void erase(
      ParticipantId participant,
      ItineraryVersion version,
      bool retransmission) = 0;

  /// Erase a route from an itinerary.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] routes
  ///   The indices of the routes that should be erased.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  /// \param[in] retransmission
  ///   Set this to true if this is a retransmission of an update that was
  ///   previously lost. Otherwise set it to false.
  ///
  virtual void erase(
      ParticipantId participant,
      const std::vector<ItineraryVersion>& routes,
      ItineraryVersion version,
      bool retransmission) = 0;

  /// Register a new participant.
  ///
  /// \param[in] participant_info
  ///   Information about the new participant.
  ///
  /// \return result of registering the new participant.
  virtual ParticipantId register_participant(
      ParticipantDescription participant_info) = 0;

  /// Unregister an existing participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant to unregister.
  ///
  /// \return the new version of the schedule.
  virtual void unregister_participant(ParticipantId participant) = 0;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__WRITER_HPP
