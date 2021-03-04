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

#include <rmf_traffic/schedule/Patch.hpp>
#include "../detail/internal_bidirectional_iterator.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Patch::Participant::Implementation
{
public:

  ParticipantId id;
  Change::Erase erasures;
  std::vector<Change::Delay> delays;
  Change::Add additions;

};

//==============================================================================
Patch::Participant::Participant(
  ParticipantId id,
  Change::Erase erasures,
  std::vector<Change::Delay> delays,
  Change::Add additions)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        id,
        std::move(erasures),
        std::move(delays),
        std::move(additions)
      }))
{
  // Do nothing
}

//==============================================================================
ParticipantId Patch::Participant::participant_id() const
{
  return _pimpl->id;
}

//==============================================================================
const Change::Erase& Patch::Participant::erasures() const
{
  return _pimpl->erasures;
}

//==============================================================================
const std::vector<Change::Delay>& Patch::Participant::delays() const
{
  return _pimpl->delays;
}

//==============================================================================
const Change::Add& Patch::Participant::additions() const
{
  return _pimpl->additions;
}

//==============================================================================
class Patch::Implementation
{
public:

  std::vector<Change::UnregisterParticipant> unregistered;
  std::vector<Change::RegisterParticipant> registered;
  std::vector<Change::UpdateParticipantInfo> updated;
  std::vector<Participant> changes;
  
  rmf_utils::optional<Change::Cull> cull;
  Version latest_version;

};

//==============================================================================
class Patch::IterImpl
{
public:

  std::vector<Participant>::const_iterator iter;

};

//==============================================================================
Patch::Patch(
  std::vector<Change::UnregisterParticipant> removed_participants,
  std::vector<Change::RegisterParticipant> new_participants,
  std::vector<Change::UpdateParticipantInfo> updated_participants,
  std::vector<Participant> changes,
  rmf_utils::optional<Change::Cull> cull,
  Version latest_version)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(removed_participants),
        std::move(new_participants),
        std::move(updated_participants),
        std::move(changes),
        cull,
        latest_version
      }))
{
  // Do nothing
}

//==============================================================================
const std::vector<Change::UpdateParticipantInfo>& Patch::update() const
{
  return _pimpl->updated;
}

//==============================================================================
const std::vector<Change::UnregisterParticipant>& Patch::unregistered() const
{
  return _pimpl->unregistered;
}

//==============================================================================
const std::vector<Change::RegisterParticipant>& Patch::registered() const
{
  return _pimpl->registered;
}

//==============================================================================
auto Patch::begin() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.begin()});
}

//==============================================================================
auto Patch::end() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.end()});
}

//==============================================================================
std::size_t Patch::size() const
{
  return _pimpl->changes.size();
}

//==============================================================================
const Change::Cull* Patch::cull() const
{
  if (_pimpl->cull)
    return &_pimpl->cull.value();

  return nullptr;
}

//==============================================================================
Version Patch::latest_version() const
{
  return _pimpl->latest_version;
}

} // namespace schedule


namespace detail {

template class bidirectional_iterator<
    const schedule::Patch::Participant,
    schedule::Patch::IterImpl,
    schedule::Patch
>;

} // namespace detail
} // namespace rmf_traffic
