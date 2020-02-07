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

#include "InconsistencyTracker.hpp"
#include "Modular.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
InconsistencyTracker::InconsistencyTracker(
    ParticipantId id)
: _participant(id)
{
  // Do nothing
}

//==============================================================================
void InconsistencyTracker::reset(
    ItineraryVersion current_version,
    Database::Inconsistencies& inconsistencies)
{
  _current_version = current_version;

  const auto inc_it = inconsistencies.find(_participant);
  if (inc_it == inconsistencies.end())
  {
    inc_it->second.clear();
  }

  // TODO(MXG): Erase internal tracking mechanisms
}

//==============================================================================
void InconsistencyTracker::Ticket::set(std::function<void ()> change)
{
  _set = true;
  _callback = change;
}

//==============================================================================
InconsistencyTracker::Ticket::~Ticket()
{
  if (!_set)
  {
    throw std::runtime_error(
          "[rmf_traffic::schedule::InconsistencyTracker::~Ticket] "
          "Caller neglected to set callback for the inconsistent change. This "
          "indicates a critical error in rmf_traffic and should be reported to "
          "the developers immediately.");
  }

  // TODO(MXG): Consider whether this operation should be performed here in the
  // destructor or above in the set() function. An argument for doing it here is
  // that there would be no negative side-effects of the Ticket-holder calling
  // set() multiple times, although it would be dubious for the caller to do
  // that in the first place.
  if (_parent._ready)
  {
    _parent._apply_changes();
  }
}

//==============================================================================
InconsistencyTracker::Ticket::Ticket(
    InconsistencyTracker& parent,
    std::function<void()>& callback)
: _parent(parent),
  _callback(callback)
{
  // Do nothing
}

//==============================================================================
auto InconsistencyTracker::check(
    ItineraryVersion version,
    Database::Inconsistencies& inconsistencies)
-> rmf_utils::optional<Ticket>
{
  auto inc_it = inconsistencies.find(_participant);
  if (inc_it == inconsistencies.end())
  {
    if (version == _current_version + 1)
    {
      // This means we have no inconsistencies to worry about
      _current_version = version;
      return rmf_utils::nullopt;
    }

    // This should have been checked earlier by the caller, but we will assert
    // it here just to make sure.
    assert(!modular(version).less_or_equal(_current_version));

    // We didn't have any inconsistencies before, but we do now.
    inc_it = inconsistencies.insert(
          std::make_pair(_participant, Database::InconsistencyRanges())).first;

    // This is the only inconsistency, so it should be easy to fill in the map:
    Database::InconsistencyRanges& ranges = inc_it->second;
    ranges[version-1] = _current_version + 1;
  }
  else
  {
    // There are already inconsistencies with this participant, so we will check
    // how this new entry affects the current ranges of inconsistencies and
    // adjust them as needed.

    // FIXME(MXG): Finish this
  }

  // TODO(MXG): Should we care about cases where a repeat ticket is issued for
  // the same inconsistent change number? It would be largely irrelevant for a
  // new copy of the change to replace the old copy, because they should be
  // functionally the same as long as the Participant is a reliable actor. It
  // would duplicate some allocation and copying needlessly, but that probably
  // won't have a noticeable impact. When we have benchmark tests for the
  // schedule we can try to make it more efficient by exposing when repeat
  // tickets are issued and see if that improves performance.
  const auto c_it = _changes.insert(std::make_pair(version, nullptr)).first;
  return Ticket(*this, c_it->second);
}

} // namespace schedule
} // namespace rmf_traffic
