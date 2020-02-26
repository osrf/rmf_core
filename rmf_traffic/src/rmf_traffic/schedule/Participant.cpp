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

#include "ParticipantInternal.hpp"
#include "RectifierInternal.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
Participant Participant::Implementation::make(
    ParticipantDescription description,
    Writer& writer,
    RectificationRequesterFactory* rectifier_factory)
{
  const ParticipantId id = writer.register_participant(description);

  Participant participant;
  participant._pimpl = rmf_utils::make_unique_impl<Implementation>(
        std::move(description), writer);

  participant._pimpl->_rectification =
      rectifier_factory->make(
        Rectifier::Implementation::make(*participant._pimpl), id);

  return participant;
}

//==============================================================================
void Participant::Implementation::retransmit(
    const std::vector<Rectifier::Range>& ranges,
    const ItineraryVersion last_known_version)
{
  for (const auto& range : ranges)
  {
    assert(range.lower < range.upper);

    // We use lower_bound because it is possible that some of this range has
    // been truncated because of a nullifying change. Therefore we should accept
    // the lowest change in the map if the lower end of this range is gone.
    const auto begin_it = _change_history.lower_bound(range.lower);

    const auto end_it = _change_history.find(range.upper);
    if (end_it == _change_history.end())
    {
      // These changes are no longer relevant. The inconsistency should get
      // fixed by either one of the later ranges or by fixing the tail at the
      // end of this function.
      continue;
    }

    // The _change_map will always contain complete sequences with no gaps, so
    // if end_it was found, then begin_it should be an entry less than or equal
    // to end_it.
    assert(begin_it->first <= end_it->first);

    for (auto it = begin_it; it->first <= end_it->first; ++it)
      it->second();
  }

  // In case the database doesn't have the most recent changes, we will
  // retransmit them.
  const auto tail_begin = _change_history.upper_bound(last_known_version);
  for (auto it = tail_begin; it != _change_history.end(); ++it)
    it->second();
}

//==============================================================================
ItineraryVersion Participant::Implementation::current_version() const
{
  return _version;
}

//==============================================================================
RouteId Participant::set(std::vector<Route> itinerary)
{
  _pimpl->_change_history.clear();

}

} // namespace schedule
} // namespace rmf_traffic
