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

#include "RectifierInternal.hpp"

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
Rectifier Rectifier::Implementation::make(
    Participant::Implementation& participant)
{
  Rectifier rectifier;
  rectifier._pimpl = rmf_utils::make_unique_impl<Implementation>(
        Implementation{participant});

  return rectifier;
}

//==============================================================================
void Rectifier::retransmit(
    const std::vector<Range>& ranges,
    ItineraryVersion last_known_version)
{
  _pimpl->participant.retransmit(ranges, last_known_version);
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
class DatabaseRectificationRequester : public RectificationRequester
{
public:

  struct Handle
  {
    DatabaseRectificationRequester& requester;
  };

  DatabaseRectificationRequester(
      const Database& database,
      Rectifier rectifier,
      ParticipantId id)
    : _database(database),
      _handle(std::make_shared<Handle>(Handle{*this})),
      _rectifier(std::move(rectifier)),
      _participant_id(id)
  {
    // Do nothing
  }

  void rectify()
  {
    const auto& inconsistencies = _database.inconsistencies();
    for (const auto& p : inconsistencies)
    {
      if (p.participant != _participant_id)
        continue;

      const ItineraryVersion last_known_version = p.ranges.last_known_version();
      std::vector<Rectifier::Range> ranges;
      ranges.reserve(p.ranges.size());
      for (const auto& r : p.ranges)
        ranges.push_back({r.lower, r.upper});

      _rectifier.retransmit(ranges, last_known_version);

      // We don't need to look through any more of the ranges, so we can break
      // here.
      break;
    }
  }

  const Database& _database;
  std::shared_ptr<Handle> _handle;
  Rectifier _rectifier;
  ParticipantId _participant_id;

};

//==============================================================================
class DatabaseRectificationRequesterFactory::Implementation
{
public:

  const Database& _database;
  std::vector<std::weak_ptr<DatabaseRectificationRequester::Handle>> _handles;

};

//==============================================================================
DatabaseRectificationRequesterFactory::DatabaseRectificationRequesterFactory(
    const Database& database)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(
             Implementation{database, {}}))
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<RectificationRequester>
DatabaseRectificationRequesterFactory::make(
    Rectifier rectifier,
    ParticipantId participant_id)
{
  auto requester = std::make_unique<DatabaseRectificationRequester>(
        _pimpl->_database, std::move(rectifier), participant_id);

  _pimpl->_handles.push_back(requester->_handle);
  return std::move(requester);
}

//==============================================================================
void DatabaseRectificationRequesterFactory::rectify()
{
  using WeakHandlePtr = std::weak_ptr<DatabaseRectificationRequester::Handle>;

  auto& handles = _pimpl->_handles;
  for (const auto& weak_handle : handles)
  {
    const auto handle = weak_handle.lock();
    if (handle)
      handle->requester.rectify();
  }

  // If any handles have expired, remove them
  handles.erase(std::remove_if(
                  handles.begin(),
                  handles.end(),
                  [](const WeakHandlePtr& h){ return h.expired(); }),
                handles.end());
}

} // namespace schedule
} // namespace rmf_traffic
