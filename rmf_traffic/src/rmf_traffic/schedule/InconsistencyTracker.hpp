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

    /// This custom destructor will check whether the inconsistency has been
    /// resolved, and if so then it will apply the changes
    ~Ticket();

  private:

    friend class InconsistencyTracker;

    Ticket(
        InconsistencyTracker& parent,
        std::function<void()>& callback);

    InconsistencyTracker& _parent;
    std::function<void()>& _callback;
    bool _set = false;
  };

  /// Check whether an entry has an inconsistency
  ///
  /// \param[in] version
  ///   The declared version of this change
  ///
  /// \param[in] inconsistencies
  ///   The current record of inconsistencies
  ///
  /// \param[in] reset
  ///   Whether this change will nullify the changes that came before it. If
  ///   true, we will discard the recording of the changes that predate this
  ///   one.
  ///
  /// \return an inconistency ticket which must get set with a change by the
  /// caller of check().
  rmf_utils::optional<Ticket> check(
      ItineraryVersion version,
      Database::Inconsistencies& inconsistencies,
      bool nullifying = false);

  ItineraryVersion expected_version() const
  {
    return _expected_version;
  }

private:

  void _apply_changes();

  ParticipantId _participant;
  ItineraryVersion _expected_version = 0;
  std::map<ItineraryVersion, std::function<void()>> _changes;
  bool _ready = false;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCYTRACKER_HPP
