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

#ifndef RMF_TRAFFIC__SCHEDULE__INCONSISTENCY_HPP
#define RMF_TRAFFIC__SCHEDULE__INCONSISTENCY_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_traffic/detail/bidirectional_iterator.hpp>
#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

namespace rmf_traffic {
namespace schedule {

//============================================================================
/// An Inconsistency occurs when one or more ItineraryVersion values get
/// skipped by the inputs into the database. This container expresses the ranges
/// of which ItineraryVersions were skipped for a single Participant.
///
/// Iterators
class Inconsistencies
{
public:

  template<typename E, typename I, typename F>
  using base_iter = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  /// A container of the ranges of inconsistencies for a single participant
  class Ranges
  {
  public:

    /// A single range of inconsistencies within a participant.
    ///
    /// Every version between (and including) the lower and upper versions have
    /// not been received by the Database.
    struct Range
    {
      ItineraryVersion lower;
      ItineraryVersion upper;
    };



    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// An element of the Inconsistencies container. This tells the ranges of
  /// inconsistencies that are present for the specified Participant.
  struct Element
  {
    ParticipantId participant;
    Ranges ranges;
  };

  class IterImpl;
  using const_iterator = base_iter<const Element, IterImpl, Inconsistencies>;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__INCONSISTENCY_HPP
