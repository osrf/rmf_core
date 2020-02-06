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

#ifndef RMF_TRAFFIC__SCHEDULE__CHANGE_HPP
#define RMF_TRAFFIC__SCHEDULE__CHANGE_HPP

#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Version.hpp>

#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

/// A class that describes a change within the schedule
class Change
{
public:

  /// Enumeration for what type of change has occurred
  enum class Mode : uint16_t
  {
    /// This is an invalid Change::Mode. If you ever find this a Change object
    /// in this mode, please report it as a bug.
    Invalid = 0,

    /// The whole itinerary of a participant was inserted or replaced
    Put,

    /// A route was added to a participant's existing itinerary
    Post,

    /// A delay was introduced to an itinerary
    Delay,

    /// An itinerary was erased
    Erase,

    /// Some itineraries were culled
    Cull,

    /// The number of Modes. This must always come last in the list. This is
    /// never a valid Mode. Please report if this is ever returned by the
    /// get_mode() function.
    NUM,
  };

  /// Put a new itinerary in for a participant. This will replace the existing
  /// itinerary for this participant, or create a new one if it didn't have one
  /// already.
  ///
  /// \param[in] participant
  ///   The ID of the participant that the itinerary is for.
  ///
  /// \param[in] itinerary
  ///   The itinerary that is being put for this participant.
  ///
  static Change make_put(
      ParticipantId participant,
      Itinerary itinerary,
      Version schedule_version);

  /// Post a new route to the participant's itinerary. This will add the route
  /// to the participant's existing itinerary without impacting the routes that
  /// are already in it.
  ///
  /// \param[in] participant
  ///   The ID of the participant that the route is for.
  ///
  /// \param[in] route
  ///   The route being added to the itinerary.
  static Change make_post(
      ParticipantId participant,
      Route route,
      Version schedule_version);

  /// Add a delay to the existing itinerary. All
  ///
  /// \param[in] participant
  ///   The ID of the participant that is being delayed.
  ///
  /// \param[in] from
  ///   The point in time where the delay originates. All trajectory elements
  ///   that end after this time point will be pushed back by the delay.
  ///
  /// \param[in] delay
  ///   The duration of the delay (how far back the trajectory elements that
  ///   come after `from` should be pushed).
  ///
  static Change make_delay(
      ParticipantId participant,
      Time from,
      Duration delay,
      Version schedule_version);

  /// Erase the itinerary of a participant
  ///
  /// \param[in] participant
  ///   The ID of the participant whose itinerary is being erased.
  ///
  static Change make_erase(
      ParticipantId participant,
      Version schedule_version);

  /// Erase one or more routes of a participant
  ///
  /// \param[in] participant
  ///   The ID of the participant whose routes are being erased.
  ///
  /// \param[in] routes
  ///   The indices of the routes in the itinerary that are being erased.
  ///
  static Change make_erase(
      ParticipantId participant,
      std::vector<RouteId> routes,
      Version schedule_version);

  /// The API for a Put change
  class Set
  {
  public:

    /// A reference to the Trajectory that was inserted.
    const Itinerary& itinerary() const;

    class Implementation;
  private:
    Set();
    friend class Change;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The API for a Post change
  class Extend
  {
  public:

    /// A reference to the Route that was inserted.
    const Itinerary& route() const;

    class Implementation;
  private:
    Extend();
    friend class Change;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The API for a Delay change
  class Delay
  {
  public:

    /// The time that the delay began.
    Time from() const;

    /// The duration of the delay.
    Duration duration() const;

    class Implementation;
  private:
    Delay();
    friend class Change;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The API for an erasure
  class Erase
  {
  public:

    /// True if the entire itinerary should be erased. False if only a set of
    /// routes should be erased.
    bool entire_itinerary() const;

    /// The indices of the routes that should be erased if entire_itinerary() is
    /// false.
    const std::vector<RouteId>& routes() const;

    class Implementation;
  private:
    Erase();
    friend class Change;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Get the type of Change
  Mode get_mode() const;

  /// Get the version ID that this change refers to
  ParticipantId participant() const;

  /// Get the Put interface if this is a Put type change. Otherwise this returns
  /// a nullptr.
  const Set* set() const;

  /// Get the Post interface if this is an Post type change. Otherwise
  /// this returns a nullptr.
  const Extend* extend() const;

  /// Get the Delay interface if this is a Delay type change. Otherwise this
  /// returns a nullptr.
  const Delay* delay() const;

  /// Get the Erase interface if this is an Erase type change. Otherwise this
  /// returns a nullptr.
  const Erase* erase() const;

  class Implementation;
private:
  Change();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__CHANGE_HPP
