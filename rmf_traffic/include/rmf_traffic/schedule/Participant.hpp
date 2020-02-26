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

#include <future>

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
  /// \param[in] from
  ///   The time where the delay pushback begins. Waypoints that come after this
  ///   time will be pushed back by the delay amount.
  ///
  /// \param[in] delay
  ///   The amount of time to push back the relevant waypoints.
  void delay(Time from, Duration delay);

  /// Erase certain routes from the itinerary.
  ///
  /// \param[in] routes
  ///   The list of routes to erase.
  void erase(std::vector<RouteId> routes);

  /// Clear all routes from the itinerary.
  void clear();

  /// Get the last RouteId used by this Participant. The next RouteId that gets
  /// issued will be incremented from this value.
  RouteId last_route_id() const;

  /// Get the current itinerary of the participant.
  const Writer::Input& itinerary() const;

  /// Get the description of this participant.
  const ParticipantDescription& description() const;

  class Implementation;
private:
  // The constructor for this class is private. It should only be constructed
  // using make_participant
  Participant();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
Participant make_participant(
    ParticipantDescription description,
    Writer& writer,
    RectificationRequesterFactory* rectifier_factory);

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__PARTICIPANT_HPP
