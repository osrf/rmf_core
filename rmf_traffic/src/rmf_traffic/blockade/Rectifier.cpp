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

#include "internal_Rectifier.hpp"

#include <rmf_traffic/blockade/Moderator.hpp>

#include <unordered_set>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
Rectifier Rectifier::Implementation::make(
    Participant::Implementation& participant)
{
  Rectifier output;
  output._pimpl = rmf_utils::make_unique_impl<Implementation>(
        Implementation{participant});

  return output;
}

//==============================================================================
void Rectifier::check(const Status& status)
{
  _pimpl->participant.check(status);
}

//==============================================================================
Rectifier::Rectifier()
{
  // Do nothing
}

//==============================================================================
RectificationRequester::~RectificationRequester()
{
  // Do nothing
}

//==============================================================================
struct ModeratorRectifierInfo
    : public std::enable_shared_from_this<ModeratorRectifierInfo>
{
  std::unordered_map<ParticipantId, Rectifier> active_rectifiers;
  std::unordered_set<ParticipantId> dead_rectifiers;

  std::unique_ptr<RectificationRequester> make(
      Rectifier rectifier,
      ParticipantId participant_id);
};

//==============================================================================
class ModeratorRectificationRequester : public RectificationRequester
{
public:

  ModeratorRectificationRequester(
      ParticipantId participant_id,
      const std::shared_ptr<ModeratorRectifierInfo>& info)
    : _participant_id(participant_id),
      _weak_info(info)
  {
    // Do nothing
  }

  virtual ~ModeratorRectificationRequester()
  {
    if (const auto info = _weak_info.lock())
    {
      info->dead_rectifiers.insert(_participant_id);
      info->active_rectifiers.erase(_participant_id);
    }
  }

private:
  const ParticipantId _participant_id;
  std::weak_ptr<ModeratorRectifierInfo> _weak_info;
};

//==============================================================================
std::unique_ptr<RectificationRequester> ModeratorRectifierInfo::make(
    Rectifier rectifier,
    ParticipantId participant_id)
{
  active_rectifiers.insert_or_assign(participant_id, std::move(rectifier));
  dead_rectifiers.erase(participant_id);

  return std::make_unique<ModeratorRectificationRequester>(
        participant_id, shared_from_this());
}

//==============================================================================
class ModeratorRectificationRequesterFactory::Implementation
{
public:

  std::shared_ptr<Moderator> moderator;
  std::shared_ptr<ModeratorRectifierInfo> info;

  Implementation(std::shared_ptr<Moderator> moderator_)
    : moderator(std::move(moderator_)),
      info(std::make_shared<ModeratorRectifierInfo>())
  {
    // Do nothing
  }

  void rectify()
  {
    const auto statuses = moderator->statuses();
    for (const auto& s : statuses)
    {
      const auto participant = s.first;
      const auto r_it = info->active_rectifiers.find(participant);
      if (r_it != info->active_rectifiers.end())
        r_it->second.check(s.second);
    }

    auto dead_it = info->dead_rectifiers.begin();
    while (dead_it != info->dead_rectifiers.end())
    {
      const auto participant_id = *dead_it;
      const auto s_it = statuses.find(participant_id);
      if (s_it != statuses.end())
        moderator->cancel(participant_id);

      info->dead_rectifiers.erase(dead_it++);
    }
  }
};

//==============================================================================
ModeratorRectificationRequesterFactory::ModeratorRectificationRequesterFactory(
    std::shared_ptr<Moderator> moderator)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(std::move(moderator)))
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<RectificationRequester>
ModeratorRectificationRequesterFactory::make(
    Rectifier rectifier,
    const ParticipantId participant_id)
{
  return _pimpl->info->make(std::move(rectifier), participant_id);
}

//==============================================================================
void ModeratorRectificationRequesterFactory::rectify()
{
  _pimpl->rectify();
}

} // namespace blockade
} // namespace rmf_traffic
