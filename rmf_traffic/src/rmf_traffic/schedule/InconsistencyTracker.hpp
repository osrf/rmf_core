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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCYTRACKER_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCYTRACKER_HPP

#include <rmf_traffic/schedule/Inconsistencies.hpp>
#include <rmf_traffic/schedule/Itinerary.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_utils/optional.hpp>
#include <rmf_utils/Modular.hpp>

#include <set>
#include <map>
#include <unordered_map>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
struct RangeLess
{
  using Range = Inconsistencies::Ranges::Range;

  bool operator()(const Range& lhs, const Range& rhs)
  {
    return rmf_utils::modular(lhs.upper).less_than(rhs.upper);
  }
};

//==============================================================================
using RangesSet = std::set<Inconsistencies::Ranges::Range, RangeLess>;

//==============================================================================
using InconsistenciesMap = std::unordered_map<ParticipantId, RangesSet>;

//==============================================================================
class InconsistencyTracker
{
public:

  InconsistencyTracker(
    RangesSet& parent,
    ItineraryVersion& last_known_version);

  /// The Ticket class is a way to inform the caller that there is an
  /// inconsistency with the version of an incoming change. When the
  /// Inconsistency::check() function returns a Ticket, the caller must pass
  /// along a callback function that represents the intended change.
  ///
  /// We use a ticket class for this instead of having the caller pass a
  /// callback to the check() function because generally the callback will be a
  /// lambda that requires memory allocation and copy operations that won't be
  /// necessary if there's no inconsistency.
  class Ticket
  {
  public:

    /// Set the change for this ticket.
    void set(std::function<void()> change);

    Ticket(
      InconsistencyTracker& parent,
      std::function<void()>& callback);

    Ticket(const Ticket&) = delete;
    Ticket& operator=(const Ticket&) = delete;

    /// This custom destructor will check whether the inconsistency has been
    /// resolved, and if so then it will apply the changes
    ~Ticket();

  private:

    friend class InconsistencyTracker;

    InconsistencyTracker& _parent;
    std::function<void()>& _callback;
    bool _set = false;
  };

  /// Check whether an entry has an inconsistency
  ///
  /// \param[in] version
  ///   The declared version of this change
  ///
  /// \param[in] nullifying
  ///   Whether this change will nullify the changes that came before it. If
  ///   true, we will discard the recording of the changes that predate this
  ///   one.
  ///
  /// \return an inconistency ticket which must get set with a change by the
  /// caller of check().
  //
  // TODO(MXG): When we are allowed to use std::optional, check if that might
  // work here. rmf_utils::optional does not work for this because the move
  // semantics aren't right, but maybe the STL version would. Or maybe we need
  // a moveable version of std::reference_wrapper.
  std::unique_ptr<Ticket> check(
    ItineraryVersion version,
    bool nullifying = false);

  ItineraryVersion expected_version() const
  {
    return _expected_version;
  }

  ItineraryVersion last_known_version() const
  {
    return _last_known_version;
  }

private:

  void _apply_changes();

  // TODO(MXG): Consider a more robust way of keeping _ranges up to date. Right
  // now we calculate both _changes and _ranges independently, but they are
  // really coupled. Maybe after each modification to _changes we should simply
  // recompute _ranges from scratch. This would be less efficient but more
  // robust.
  RangesSet& _ranges;

  ItineraryVersion& _last_known_version;
  ItineraryVersion _expected_version = 0;
  std::map<ItineraryVersion, std::function<void()>> _changes;

  // TODO(MXG): Consider whether this _ready flag is superfluous since a Ticket
  // can simply check _ranges.empty() to determine if the changes are ready to
  // be applied.
  bool _ready = false;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCYTRACKER_HPP
