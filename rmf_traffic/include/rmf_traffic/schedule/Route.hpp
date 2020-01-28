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

#ifndef RMF_TRAFFIC__SCHEDULDE__ROUTE_HPP
#define RMF_TRAFFIC__SCHEDULDE__ROUTE_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A route on the schedule. This is used as a component of a schedule
/// participant's itinerary.
class Route
{
public:

  /// Constructor
  ///
  /// \param[in] map
  ///   The map that the trajectory is on
  ///
  /// \param[in] trajectory
  ///   The scheduled trajectory
  ///
  Route(
      std::string map,
      Trajectory trajectory);

  /// Set the map for this itinerary element
  Route& map(std::string value);

  /// Get the map for this itinerary element
  const std::string& map() const;

  /// Set the trajectory for this itinerary element
  Route& trajectory(Trajectory value);

  /// Get the trajectory for this itinerary element
  const Trajectory& trajectory() const;

private:
  class Implementation;
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RoutePtr = std::shared_ptr<Route>;
using ConstRoutePtr = std::shared_ptr<const Route>;

} // namespace schedule
} // namespace rmf_traffic


#endif // RMF_TRAFFIC__SCHEDULDE__ROUTE_HPP
