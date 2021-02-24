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

  struct Item
  {
    RouteId id;
    ConstRoutePtr route;
  };

  using Input = std::vector<Item>;

  /// Set a brand new itinerary for a participant. This will replace any
  /// itinerary that is already in the schedule for the participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] itinerary
  ///   The new itinerary of the participant.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void set(
    ParticipantId participant,
    const Input& itinerary,
    ItineraryVersion version) = 0;

  /// Add a set of routes to the itinerary of this participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being updated.
  ///
  /// \param[in] routes
  ///   The set of routes that should be added to the itinerary.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void extend(
    ParticipantId participant,
    const Input& routes,
    ItineraryVersion version) = 0;

  /// Add a delay to the itinerary from the specified Time.
  ///
  /// Nothing about the routes in the itinerary will be changed except that
  /// waypoints will shifted through time.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being delayed.
  ///
  /// \param[in] delay
  ///   This is the duration of time to delay all qualifying Trajectory
  ///   Waypoints.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void delay(
    ParticipantId participant,
    Duration delay,
    ItineraryVersion version) = 0;

  /// Erase an itinerary from this database.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being erased.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void erase(
    ParticipantId participant,
    ItineraryVersion version) = 0;

  /// Erase a route from an itinerary.
  ///
  /// \param[in] participant
  ///   The ID of the participant whose routes are being erased.
  ///
  /// \param[in] routes
  ///   The indices of the routes that should be erased.
  ///
  /// \param[in] version
  ///   The version for this itinerary change
  ///
  virtual void erase(
    ParticipantId participant,
    const std::vector<RouteId>& routes,
    ItineraryVersion version) = 0;

  /// Information resulting from registering a participant
  class Registration
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID for the registered participant
    ///
    /// \param[in] version
    ///   The initial version for the registered participant
    Registration(
      ParticipantId id,
      ItineraryVersion version);

    /// The ID of the registered participant
    ParticipantId id() const;

    /// The initial itinerary version of the registered participant.
    ///
    /// This value might vary for systems that enforce participant uniqueness.
    /// If this participant was registered in the past and is now being
    /// re-registered, then the version number will pick up where it previously
    /// left off.
    ItineraryVersion itinerary_version() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Register a new participant.
  ///
  /// \param[in] participant_info
  ///   Information about the new participant.
  ///
  /// \param[in] time
  ///   The time at which the registration is being requested.
  ///
  /// \return result of registering the new participant.
  virtual Registration register_participant(
    ParticipantDescription participant_info) = 0;

  /// Unregister an existing participant.
  ///
  /// \param[in] participant
  ///   The ID of the participant to unregister.
  ///
  /// \return the new version of the schedule.
  virtual void unregister_participant(
    ParticipantId participant) = 0;

  // virtual destructor
  virtual ~Writer() = default;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__WRITER_HPP
