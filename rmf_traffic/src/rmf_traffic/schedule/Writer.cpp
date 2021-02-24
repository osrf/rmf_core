/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_traffic/schedule/Writer.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Writer::Registration::Implementation
{
public:

  ParticipantId id;
  ItineraryVersion version;

};

//==============================================================================
Writer::Registration::Registration(
  ParticipantId id,
  ItineraryVersion version)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{id, version}))
{
  // Do nothing
}

//==============================================================================
ParticipantId Writer::Registration::id() const
{
  return _pimpl->id;
}

//==============================================================================
ItineraryVersion Writer::Registration::itinerary_version() const
{
  return _pimpl->version;
}

} // namespace schedule
} // namespace rmf_traffic
