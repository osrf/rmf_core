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

#include "ChangeInternal.hpp"

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Change::Add::Implementation
{
public:

  std::vector<Item> additions;

};

//==============================================================================
Change::Add::Add(std::vector<Item> additions)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{std::move(additions)}))
{
  // Do nothing
}

//==============================================================================
auto Change::Add::items() const -> const std::vector<Item>&
{
  return _pimpl->additions;
}

//==============================================================================
Change::Delay::Delay(Time from, Duration duration)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{from, duration}))
{
  // Do nothing
}

//==============================================================================
Time Change::Delay::from() const
{
  return _pimpl->from;
}

//==============================================================================
Duration Change::Delay::duration() const
{
  return _pimpl->delay;
}

//==============================================================================
class Change::Erase::Implementation
{
public:

  std::vector<RouteId> ids;

};

//==============================================================================
Change::Erase::Erase(std::vector<RouteId> ids)
: _pimpl(rmf_utils::make_impl<Implementation>(std::move(ids)))
{
  // Do nothing
}

//==============================================================================
const std::vector<RouteId>& Change::Erase::ids() const
{
  return _pimpl->ids;
}

//==============================================================================
class Change::RegisterParticipant::Implementation
{
public:

  ParticipantId id;
  ParticipantDescription description;

};

//==============================================================================
Change::RegisterParticipant::RegisterParticipant(
    ParticipantId id,
    ParticipantDescription description)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{id, description}))
{
  // Do nothing
}

//==============================================================================
ParticipantId Change::RegisterParticipant::id() const
{
  return _pimpl->id;
}

//==============================================================================
const ParticipantDescription& Change::RegisterParticipant::description() const
{
  return _pimpl->description;
}

//==============================================================================
class Change::UnregisterParticipant::Implementation
{
public:

  ParticipantId id;

};

//==============================================================================
Change::UnregisterParticipant::UnregisterParticipant(ParticipantId id)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{id}))
{
  // Do nothing
}

//==============================================================================
ParticipantId Change::UnregisterParticipant::id() const
{
  return _pimpl->id;
}

//==============================================================================
class Change::Cull::Implementation
{
public:

  Time time;

};

//==============================================================================
Change::Cull::Cull(Time time)
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation{time}))
{
  // Do nothing
}

//==============================================================================
Time Change::Cull::time() const
{
  return _pimpl->time;
}

} // namespace schedule
} // namespace rmf_traffic
