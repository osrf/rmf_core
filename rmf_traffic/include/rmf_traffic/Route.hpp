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

#ifndef RMF_TRAFFIC__ROUTE_HPP
#define RMF_TRAFFIC__ROUTE_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {

//==============================================================================
using RouteId = uint64_t;

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

  /// Set the map for this route
  Route& map(std::string value);

  /// Get the map for this route
  const std::string& map() const;

  /// Set the trajectory for this route
  Route& trajectory(Trajectory value);

  /// Get the trajectory for this route
  Trajectory& trajectory();

  /// Get the trajectory for this route
  const Trajectory& trajectory() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RoutePtr = std::shared_ptr<Route>;
using ConstRoutePtr = std::shared_ptr<const Route>;

} // namespace rmf_traffic


#endif // RMF_TRAFFIC__SCHEDULDE__ROUTE_HPP
