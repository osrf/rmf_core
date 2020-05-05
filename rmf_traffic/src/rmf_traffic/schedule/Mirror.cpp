/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <rmf_traffic/schedule/Mirror.hpp>

#include "ChangeInternal.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Mirror::Implementation
{
public:

  struct RouteEntry
  {
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<const ParticipantDescription> description;
  };
  using ConstRouteEntryPtr = std::shared_ptr<const RouteEntry>;

  struct RouteStorage
  {
    ConstRouteEntryPtr entry;
    std::shared_ptr<void> timeline_handle;
  };

  Timeline<const RouteEntry> timeline;

  struct ParticipantState
  {
    std::unordered_map<RouteId, RouteStorage> storage;
    std::shared_ptr<const ParticipantDescription> description;
  };

  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  std::unordered_set<ParticipantId> participant_ids;

  Version latest_version = 0;

  static void erase_routes(
    const ParticipantId participant,
    ParticipantState& state,
    const Change::Erase& erase)
  {
    for (const RouteId id : erase.ids())
    {
      const auto r_it = state.storage.find(id);
      assert(r_it != state.storage.end());
      if (r_it == state.storage.end())
      {
        std::cerr << "[Mirror::update] Erasing unrecognized route [" << id
                  << "] for participant [" << participant << "]" << std::endl;
        continue;
      }

      state.storage.erase(r_it);
    }
  }

  void apply_delay(
    ParticipantState& state,
    const Change::Delay& delay)
  {
    for (auto& s : state.storage)
    {
      RouteStorage& entry_storage = s.second;
      assert(entry_storage.entry);
      assert(entry_storage.entry->route);
      auto delayed = schedule::apply_delay(
        entry_storage.entry->route->trajectory(),
        delay.from(), delay.duration());

      if (!delayed)
        continue;

      auto new_route = std::make_shared<Route>(
        entry_storage.entry->route->map(), std::move(*delayed));

      // We create a new entry because
      auto new_entry = std::make_shared<RouteEntry>(*entry_storage.entry);
      new_entry->route = std::move(new_route);
      entry_storage.entry = new_entry;
      entry_storage.timeline_handle = timeline.insert(new_entry);
    }
  }

  void add_routes(
    const ParticipantId participant,
    ParticipantState& state,
    const Change::Add& add)
  {
    for (const auto& item : add.items())
    {
      const RouteId route_id = item.id;
      auto insertion = state.storage.insert({route_id, RouteStorage()});
      const bool inserted = insertion.second;
      assert(inserted);
      if (!inserted)
      {
        std::cerr << "[Mirror::update] Inserting a route [" << item.id
                  << "] which already exists for participant [" << participant
                  << "]" << std::endl;
        // NOTE(MXG): We will continue anyway. The new route will simply
        // overwrite the old one.
      }

      auto route = std::make_shared<Route>(*item.route);

      auto& entry_storage = insertion.first->second;
      entry_storage.entry = std::make_shared<RouteEntry>(
        RouteEntry{
          std::move(route),
          participant,
          route_id,
          state.description
        });

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }
};

namespace {
//==============================================================================
class MirrorViewRelevanceInspector
  : public TimelineInspector<Mirror::Implementation::RouteEntry>
{
public:

  using RouteEntry = Mirror::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry);
    assert(entry->route);
    if (relevant(*entry))
    {
      routes.emplace_back(
        Storage{
          entry->participant,
          entry->route_id,
          entry->route,
          entry->description
        });
    }
  }

};

//==============================================================================
class MirrorCullRelevanceInspector
  : public TimelineInspector<Mirror::Implementation::RouteEntry>
{
public:

  using RouteEntry = Mirror::Implementation::RouteEntry;
  struct Info
  {
    ParticipantId participant;
    RouteId route_id;
  };

  std::vector<Info> info;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry);
    assert(entry->route);
    if (relevant(*entry))
      info.emplace_back(Info{entry->participant, entry->route_id});
  }

};

} // anonymous namespace

//==============================================================================
Viewer::View Mirror::query(const Query& parameters) const
{
  return query(parameters.spacetime(), parameters.participants());
}

//==============================================================================
Viewer::View Mirror::query(
    const Query::Spacetime& spacetime,
    const Query::Participants& participants) const
{
  MirrorViewRelevanceInspector inspector;
  _pimpl->timeline.inspect(spacetime, participants, inspector);
  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Mirror::participant_ids() const
{
  return _pimpl->participant_ids;
}

//==============================================================================
std::shared_ptr<const ParticipantDescription> Mirror::get_participant(
  std::size_t participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return nullptr;

  return p->second.description;
}

//==============================================================================
rmf_utils::optional<Itinerary> Mirror::get_itinerary(
  std::size_t participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return rmf_utils::nullopt;

  const auto& state = p->second;
  Itinerary itinerary;
  itinerary.reserve(state.storage.size());
  for (const auto& s : state.storage)
    itinerary.push_back(s.second.entry->route);

  return std::move(itinerary);
}

//==============================================================================
Version Mirror::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
Mirror::Mirror()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Version Mirror::update(const Patch& patch)
{
  for (const auto& unregistered : patch.unregistered())
  {
    const ParticipantId id = unregistered.id();
    const auto p_it = _pimpl->states.find(id);
    assert(p_it != _pimpl->states.end());
    if (p_it == _pimpl->states.end())
    {
      std::cerr << "[Mirror::update] Unrecognized participant ["
                << id << "] being unregistered" << std::endl;
      continue;
    }

    _pimpl->states.erase(p_it);
    _pimpl->participant_ids.erase(id);
  }

  for (const auto& registered : patch.registered())
  {
    const ParticipantId id = registered.id();
    const bool inserted = _pimpl->states.insert(
      std::make_pair(
        id,
        Implementation::ParticipantState{
          {},
          std::make_shared<ParticipantDescription>(registered.description())
        })).second;

    _pimpl->participant_ids.insert(id);

    assert(inserted);
    if (!inserted)
    {
      std::cerr << "[Mirror::update] Duplicate participant ID ["
                << id << "] while trying to register a new participant"
                << std::endl;
    }
  }

  for (const auto& p : patch)
  {
    const ParticipantId participant = p.participant_id();
    Implementation::ParticipantState& state = _pimpl->states.at(participant);

    Implementation::erase_routes(participant, state, p.erasures());

    for (const auto& delay : p.delays())
      _pimpl->apply_delay(state, delay);

    _pimpl->add_routes(participant, state, p.additions());
  }

  if (const Change::Cull* cull = patch.cull())
  {
    const Time time = cull->time();
    Query query = query_all();
    query.spacetime().query_timespan().set_upper_time_bound(time);

    MirrorCullRelevanceInspector inspector;
    _pimpl->timeline.inspect(
          query.spacetime(), query.participants(), inspector);

    for (const auto& route : inspector.info)
    {
      auto p_it = _pimpl->states.find(route.participant);
      assert(p_it != _pimpl->states.end());
      if (p_it == _pimpl->states.end())
      {
        std::cerr << "[Mirror::update] Non-existent participant ["
                  << route.participant << "] in timeline entry" << std::endl;
        continue;
      }

      p_it->second.storage.erase(route.route_id);
    }
  }

  _pimpl->latest_version = patch.latest_version();
  return _pimpl->latest_version;
}

} // schedule
} // rmf_traffic
