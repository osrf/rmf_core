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

#ifndef RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
#define RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP

#include <rmf_traffic/schedule/ParticipantDescription.hpp>
#include <rmf_traffic/schedule/Writer.hpp>
#include <rmf_traffic/schedule/Rectifier.hpp>

#include <unordered_set>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Participant
{
public:

  /// Set the whole itinerary for the participant. Every route that was
  /// previously in the itinerary will be removed and replaced with these new
  /// routes.
  ///
  /// \param[in] itinerary
  ///   The new itinerary that the participant should reflect in the schedule.
  ///
  /// \returns the highest RouteId of this participant before this change was
  /// made. The RouteId of each new route will be this return value plus
  /// the route's index in the input vector.
  RouteId set(std::vector<Route> itinerary);

  /// Add more routes for the participant. All of the routes currently in the
  /// itinerary will still be in it.
  ///
  /// \param[in] additional_routes
  ///   The new routes to add to the itinerary.
  ///
  /// \returns the highest RouteId of this participant before this change was
  /// made. The RouteId of each new route will be this return value plus the
  /// route's index in the input vector.
  RouteId extend(const std::vector<Route>& additional_routes);

  /// Delay the current itinerary.
  ///
  /// \param[in] delay
  ///   The amount of time to push back the relevant waypoints.
  void delay(Duration delay);

  /// The cumulative delay that has built up since the last call to set().
  ///
  /// \note This value will not grow when there are no are itineraries for this
  /// participant.
  //
  // TODO(MXG): Should extend() also reset this value? Currently it does not.
  Duration delay() const;

  /// Erase certain routes from the itinerary.
  ///
  /// \param[in] routes
  ///   The list of routes to erase.
  void erase(const std::unordered_set<RouteId>& routes);

  /// Clear all routes from the itinerary.
  void clear();

  /// Get the last RouteId used by this Participant. The next RouteId that gets
  /// issued will be incremented from this value.
  RouteId last_route_id() const;

  /// Get the current itinerary of the participant.
  //
  // TODO(MXG): The implementation of this class could be simpler and more
  // efficient if we did not provide this function. But not having it might hurt
  // usability if end users want some verification or reflection of the changes
  // that they are making to the schedule. Perhaps we can create a second class
  // that extends the functionality of this one, where it will both make the
  // changes to the schedule and reflect the changes locally.
  const Writer::Input& itinerary() const;

  /// Get the current itinerary version for this participant.
  //
  // TODO(MXG): This function needs to be unit tested.
  ItineraryVersion version() const;

  /// Get the description of this participant.
  const ParticipantDescription& description() const;

  /// Get the ID that was assigned to this participant.
  ParticipantId id() const;

  // This class supports moving but not copying
  Participant(Participant&&) = default;
  Participant& operator=(Participant&&) = default;

  /// The destructor will automatically tell the Writer to unregister this
  /// participant.
  ~Participant() = default;

  class Implementation;
  class Debug;
private:
  // The constructor for this class is private. It should only be constructed
  // using make_participant
  Participant();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Make a participant for the schedule.
///
/// \warning This will throw a std::runtime_error if you pass a nullptr writer.
///
/// \param[in] description
///   A descrition of the participant.
///
/// \param[in] writer
///   An interface to use when writing to the schedule.
///
/// \param[in] rectifier_factory
///   A reference to a factory that can produce a rectifier for this
///   Participant. This is useful for distributed schedule systems that have
///   unreliable connections between the Database and the Participant. Passing
///   in a nullptr indicates that there will never be a need for rectification.
///   For example, if the \code{writer} argument refers to a Database instance,
///   then there is no need for a RectifierRequesterFactory.
Participant make_participant(
  ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory = nullptr);

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
