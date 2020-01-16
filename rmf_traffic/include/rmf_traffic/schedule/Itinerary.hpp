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

#ifndef RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP
#define RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/detail/bidirectional_iterator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Itinerary
{
public:

  class Element
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
    Element(
        std::string map,
        Trajectory trajectory);

    /// Set the map for this itinerary element
    Element& map(std::string value);

    /// Get the map for this itinerary element
    const std::string& map() const;

    /// Set the trajectory for this itinerary element
    Element& trajectory(Trajectory value);

    /// Get the trajectory for this itinerary element
    const Trajectory& trajectory() const;

  private:
    class Implementation;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  template<typename E, typename I, typename F>
  using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  class IterImpl;
  using iterator = base_iterator<Element, IterImpl, Itinerary>;
  using const_iterator = base_iterator<const Element, IterImpl, Itinerary>;


private:
  class Implementation;
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP
