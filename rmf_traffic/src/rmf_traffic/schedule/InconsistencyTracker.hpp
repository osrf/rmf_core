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

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

class InconsistencyTracker
{
public:

  InconsistencyTracker(ParticipantId id);

  void reset(
      ItineraryVersion current_version,
      Database::Inconsistencies& inconsistencies);

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

    /// This custom destructor will throw an exception if the callback wasn't
    /// set before the ticket object dies, because that implies that the
    /// inconsistency ticket was not handled correctly.
    ///
    /// \note It is generally considered bad practice to throw an exception from
    /// a destructor because there are cases where this can cause undefined
    /// behavior and make an application crash with no chance of recovery.
    /// However Ticket is a private class within the implementation of
    /// rmf_traffic, so we have full control over its use, and any situation
    /// where this exception might occur. If this ever throws an exception,
    /// that indicates a bug in rmf_traffic that should be fixed immediately.
    ~Ticket();

  private:

    friend class InconsistencyTracker;

    Ticket(
        InconsistencyTracker& parent,
        std::function<void()>& callback);

    std::function<void()>& _callback;
    InconsistencyTracker& _parent;
    bool _set = false;
  };

  rmf_utils::optional<Ticket> check(
      ItineraryVersion version,
      Database::Inconsistencies& inconsistencies);

  ItineraryVersion version() const
  {
    return _current_version;
  }

private:

  void _apply_changes();

  ParticipantId _participant;
  ItineraryVersion _current_version;
  std::map<ItineraryVersion, std::function<void()>> _changes;
  bool _ready = false;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCYTRACKER_HPP
