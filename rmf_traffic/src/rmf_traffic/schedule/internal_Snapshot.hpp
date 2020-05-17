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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_SNAPSHOT_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_SNAPSHOT_HPP

#include <rmf_traffic/schedule/Snapshot.hpp>

#include "Timeline.hpp"
#include "ViewerInternal.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
template <typename RouteEntry, typename QueryInspector>
class SnapshotImplementation : public Snapshot
{
public:

  using ParticipantMap =
    std::unordered_map<
      ParticipantId,
      std::shared_ptr<const ParticipantDescription>
    >;

  View query(const Query& parameters) const final
  {
    return query(parameters.spacetime(), parameters.participants());
  }

  View query(
      const Query::Spacetime& spacetime,
      const Query::Participants& participants) const final
  {
    QueryInspector inspector;
    _timeline->inspect(spacetime, participants, inspector);
    return Viewer::View::Implementation::make_view(std::move(inspector.routes));
  }

  const std::unordered_set<ParticipantId>& participant_ids() const final
  {
    return _ids;
  }

  std::shared_ptr<const ParticipantDescription> get_participant(
      ParticipantId participant_id) const final
  {
    const auto it = _participants.find(participant_id);
    if (it == _participants.end())
      return nullptr;

    return it->second;
  }

  Version latest_version() const final
  {
    return _version;
  }

  SnapshotImplementation(
    std::shared_ptr<const TimelineView<const RouteEntry>> timeline,
    std::unordered_set<ParticipantId> ids,
    ParticipantMap participants,
    Version version)
  : _timeline(std::move(timeline)),
    _ids(std::move(ids)),
    _participants(std::move(participants)),
    _version(version)
  {
    // Do nothing
  }

private:

  std::shared_ptr<const TimelineView<const RouteEntry>> _timeline;
  std::unordered_set<ParticipantId> _ids;
  ParticipantMap _participants;
  Version _version;

};

} // namespace schedule
} // namespace rmf_traffic


#endif // SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_SNAPSHOT_HPP
