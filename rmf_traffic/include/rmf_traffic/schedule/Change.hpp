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

//==============================================================================
/// A class that describes a change within the schedule
class Change
{
public:

  /// The API for an Add change
  class Add
  {
  public:

    /// A description of an addition
    struct Item
    {
      /// The ID of the route being added
      RouteId id;

      /// The information for the route being added
      ConstRoutePtr route;
    };

    /// Add a set of routes
    Add(std::vector<Item> additions);

    /// A reference to the Trajectory that was inserted.
    const std::vector<Item>& items() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The API for a Delay change
  class Delay
  {
  public:

    /// Add a delay
    ///
    /// \param[in] from
    ///   The time that the delay began.
    ///
    /// \param[in] duration
    ///   The duration of that delay.
    Delay(
        Time from,
        Duration duration);

    /// The time that the delay began.
    Time from() const;

    /// The duration of the delay.
    Duration duration() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// A class that describes an erasing change.
  class Erase
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   The ID of the route that should be erased
    Erase(std::vector<RouteId> ids);

    const std::vector<RouteId>& ids() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Cull
  {
  public:

    /// Constructor
    ///
    /// \param[in] time
    ///   The time before which all routes should be culled
    Cull(Time time);

    Time time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__CHANGE_HPP
