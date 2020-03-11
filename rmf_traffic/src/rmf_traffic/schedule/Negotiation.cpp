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

#include <rmf_traffic/schedule/Negotiation.hpp>

#include "Timeline.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Negotiation::Table::Implementation
{
public:

  std::vector<ParticipantId> parents;


};

//==============================================================================
class Negotiation::Implementation
{
public:

  struct RouteEntry
  {
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<const ParticipantDescription> description;
    std::shared_ptr<void> timeline_handle;
  };
  using RouteEntryPtr = std::shared_ptr<RouteEntry>;

  struct TableEntry;
  using TableEntryPtr = std::unique_ptr<TableEntry>;
  using TableMap = std::unordered_map<ParticipantId, TableEntryPtr>;

  struct TableEntry
  {
    Table table;
    rmf_utils::optional<Itinerary> itinerary;
    Timeline<RouteEntry> timeline;
    TableMap descendants;
  };

  TableMap tables;

  const TableEntry* get_entry(
      const ParticipantId for_participant,
      const std::vector<ParticipantId>& to_accommodate) const
  {
    const TableMap* map = &tables;
    for (const auto p : to_accommodate)
    {
      const auto it = map->find(p);
      if (it == map->end())
        return nullptr;

      map = &it->second->descendants;
    }

    const auto it = map->find(for_participant);
    if (it == map->end())
      return nullptr;

    return it->second.get();
  }

};

//==============================================================================
auto Negotiation::table(
    const ParticipantId for_participant,
    const std::vector<ParticipantId>& to_accommodate) const -> const Table*
{
  if (const auto entry = _pimpl->get_entry(for_participant, to_accommodate))
    return &entry->table;

  return nullptr;
}

} // namespace schedule
} // namespace rmf_traffic
