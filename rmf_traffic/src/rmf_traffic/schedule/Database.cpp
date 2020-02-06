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

#include "Timeline.hpp"
#include "Modular.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"

#include <rmf_traffic/schedule/Database.hpp>

#include <algorithm>
#include <list>

namespace rmf_traffic {
namespace schedule {

namespace {

} // anonymous namespace

class Database::Implementation
{
public:

#ifndef NDEBUG
  // This field is used, only in DEBUG mode, to keep track of all route IDs that
  // have ever been added. If there is ever a repeat, an exception will be
  // thrown.
  std::unordered_set<RouteId> all_route_ids_ever;
#endif // NDEBUG

  struct ParticipantState;

  struct Transition;
  using TransitionPtr = std::unique_ptr<Transition>;
  using ConstTransitionPtr = std::unique_ptr<const Transition>;

  struct RouteEntry;
  using RouteEntryPtr = std::unique_ptr<RouteEntry>;

  using RouteHistory = std::list<RouteEntryPtr>;

  struct Transition
  {
    Change change;
    RouteEntryPtr predecessor;
  };

  struct RouteEntry
  {
    // ===== Mandatory fields for a Timeline Entry =====
    ConstRoutePtr route;
    ParticipantId participant;
    std::shared_ptr<void> timeline_handle;

    // ===== Additional fields for this timeline entry =====
    // TODO(MXG): Consider defining a base Timeline::Entry class, and then use
    // templates to automatically mix these custom fields with the required
    // fields of the base Entry
    RouteId route_id;
    Version schedule_version;
    TransitionPtr predecessor;
    RouteEntry* successor;
  };

  Timeline<RouteEntry> timeline;

  using ParticipantStorage = std::unordered_map<RouteId, RouteEntryPtr>;
  using Storage = std::unordered_map<ParticipantId, ParticipantStorage>;
  Storage storage;

  struct ParticipantState
  {
    std::unordered_set<RouteId> active_routes;
    ItineraryVersion itinerary_version;
  };
  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  Inconsistencies inconsistencies;

  Version schedule_version = 0;
};

//==============================================================================
void Database::set(
    ParticipantId participant,
    Input itinerary,
    ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    throw std::runtime_error(
          "[Database::set] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  if (Modular<ItineraryVersion>(version).less_or_equal(state.itinerary_version))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  // NOTE(MXG): We store references to the values of the route entries, because
  // iterators to a std::unordered_map can be invalidated by insertions, but
  // pointers and references to the values inside the container to do not get
  // invalidated by inserstion. Source:
  // https://en.cppreference.com/w/cpp/container/unordered_map#Iterator_invalidation
  std::vector<Implementation::RouteEntryPtr&> entries;
  entries.reserve(itinerary.size());

  Implementation::ParticipantStorage& routes = _pimpl->storage.at(participant);

  // Verify that the new route IDs do not overlap any that are still in the
  // database
  for (const Item& item : itinerary)
  {
    const auto insertion = routes.insert(std::make_pair(item.id, nullptr));

    // If the result of this insertion attempt was anything besides a nullptr,
    // then that means this route ID was already taken, so we should reject this
    // itinerary change.
    if (insertion.first->second)
    {
      throw std::runtime_error(
            "[Database::set] New route ID [" + std::to_string(item.id)
            + "] collides with one already in the database");
    }

    entries.push_back(insertion.first->second);
  }

  //======== All validation is complete ===========
  const Version schedule_version = ++_pimpl->schedule_version;

  // Since this is a nullifying change, we can erase any prior inconsistencies
  // that may have accumulated for this participant.
  const auto inc_it = _pimpl->inconsistencies.find(participant);
  if (inc_it == _pimpl->inconsistencies.end())
  {
    inc_it->second.clear();
  }

  state.active_routes.clear();

  for (std::size_t i=0; i < itinerary.size(); ++i)
  {
    const auto& item = itinerary[i];
    const RouteId route_id = item.id;
    state.active_routes.insert(route_id);

    Implementation::RouteEntryPtr& entry = entries[i];
    entry = std::make_unique<Implementation::RouteEntry>(
          Implementation::RouteEntry{
            item.route,
            participant,
            nullptr,
            route_id,
            schedule_version,
            nullptr,
            nullptr
          });

    _pimpl->timeline.insert(*entry);
  }

  state.itinerary_version = version;
}

namespace internal {

//==============================================================================
void ChangeRelevanceInspector::version_range(VersionRange range)
{
  versions = std::move(range);
}

//==============================================================================
void ChangeRelevanceInspector::after(const Version* _after)
{
  after_version = _after;
}

//==============================================================================
void ChangeRelevanceInspector::reserve(std::size_t size)
{
  relevant_changes.reserve(size);
}

//==============================================================================
namespace {

ConstEntryPtr get_last_known_ancestor(
    ConstEntryPtr from,
    const Version last_known_version,
    const VersionRange& versions)
{
  while(from && versions.less(last_known_version, from->schedule_version))
    from = from->succeeds;

  return from;
}

} // anonymous namespace

//==============================================================================
void ChangeRelevanceInspector::inspect(
    const ConstRoutePtr& route,
    const RouteInfo& info,
    const std::function<bool(const ConstRoutePtr&)>& relevant)
{
  const auto& entry = info.latest_entry;
  assert(!entry->succeeded_by);

  if(after_version
     && versions.less_or_equal(entry->schedule_version, *after_version))
    return;

  const bool needed = relevant(route);

  if(needed)
  {
    // Check if this entry descends from an entry that the remote mirror does
    // not know about.
    ConstEntryPtr record_changes_from = nullptr;
    if(after_version)
    {
      const ConstEntryPtr check =
          get_last_known_ancestor(entry, *after_version, versions);

      if(check)
      {
        if(relevant(check))
        {
          // The remote mirror already knows the lineage of this entry, so we
          // will transmit all of its changes from the last version that the
          // mirror knew about.
          record_changes_from = check;
        }
        else
        {
          // The remote mirror does not know the lineage of this entry, so we
          // will simply transmit the current entry as an insertion. We simply
          // leave record_changes_from as a nullptr.
        }
      }
      else
      {
        // The remote mirror does not know the lineage of this entry, so we
        // will simply transmit the current entry as an insertion. We simply
        // leave record_changes_from as a nullptr.
      }
    }

    if(record_changes_from)
    {
      // TODO(MXG): We can improve bandwidth usage a bit if we check whether a
      // Replace operation has taken place since the last known ancestor. If
      // that is the case, then we can skip recording all of the changes and
      // just use a single replace from the old version number to the current
      // version of the trajectory.
      ConstEntryPtr record = record_changes_from->succeeded_by;
      while(record)
      {
        relevant_changes.emplace_back(*record->change);
        record = record->succeeded_by;
      }
    }
    else
    {
      // We are not transmitting the chain of changes that led to this entry,
      // so just create an insertion for it and transmit that.
      relevant_changes.emplace_back(
            Change::Implementation::make_insert_ref(
              &route->trajectory, route->version));
    }
  }
  else if(after_version)
  {
    // Figure out if this trajectory needs to be erased
    const ConstEntryPtr check =
        get_last_known_ancestor(route, *after_version, versions);

    if(check)
    {
      if(relevant(check))
      {
        // This trajectory is no longer relevant to the remote mirror, so we
        // will tell the remote mirror to erase it rather than continuing to
        // transmit its change history. If a later version of this trajectory
        // becomes relevant again, we will tell it to insert it at that time.
        relevant_changes.emplace_back(
              Change::make_erase(check->version, route->version));
      }
    }
  }
  else
  {
    // The remote mirror never knew about the lineage of this entry, so there's
    // no need to transmit any information about it at all.
  }
}

//==============================================================================
void ChangeRelevanceInspector::inspect(
    const ConstRoutePtr& route,
    const RouteInfo& info,
    const rmf_traffic::internal::Spacetime& spacetime)
{
  inspect(route, info, [&](const ConstRoutePtr& r) -> bool {
    const Trajectory& trajectory = r->trajectory();
    if(trajectory.start_time())
    {
      return rmf_traffic::internal::detect_conflicts(
            trajectory, spacetime, nullptr);
    }
    else
    {
      return false;
    }
  });
}

//==============================================================================
void ChangeRelevanceInspector::inspect(
    const ConstRoutePtr& route,
    const RouteInfo& info,
    const Time* const lower_time_bound,
    const Time* const upper_time_bound)
{
  inspect(route, info, [&](const ConstRoutePtr& r) -> bool {
    const Trajectory& trajectory = r->trajectory();
    if(trajectory.start_time())
    {
      if(lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
        return false;

      if(upper_time_bound && *upper_time_bound < *trajectory.start_time())
        return false;

      return true;
    }
    else
    {
      return false;
    }
  });
}

} // namespace internal

//==============================================================================
Database::Database()
{
  // Do nothing
}

//==============================================================================
auto Database::changes(
    const Query& parameters,
    rmf_utils::optional<Version> after) const -> Patch
{
  auto relevant_changes = _pimpl->inspect<internal::ChangeRelevanceInspector>(
        parameters).relevant_changes;

  if(_pimpl->cull_has_occurred)
  {
    const auto* after = parameters.versions().after();
    const auto& last_cull = _pimpl->last_cull;
    if(after)
    {
      const auto range = internal::VersionRange(_pimpl->oldest_version);
      if(range.less(after->get_version(), last_cull.first))
      {
        relevant_changes.push_back(
              Change::make_cull(last_cull.second, last_cull.first));
      }
    }
    else
    {
      relevant_changes.push_back(
            Change::make_cull(last_cull.second, last_cull.first));
    }
  }

  return Patch(relevant_changes, latest_version());
}

//==============================================================================
Version Database::insert(Trajectory trajectory)
{
  internal::EntryPtr new_entry =
      std::make_shared<internal::Entry>(
        std::move(trajectory),
        ++_pimpl->latest_version);

  new_entry->change = std::make_unique<Change>(
        Change::Implementation::make_insert_ref(
          &new_entry->trajectory, new_entry->version));

  _pimpl->add_entry(new_entry);

  return new_entry->version;
}

//==============================================================================
Version Database::interrupt(
    Version id,
    Trajectory interruption_trajectory,
    Duration delay)
{
  const internal::EntryPtr old_entry =
      _pimpl->get_entry_iterator(id, "interruption")->second;

  Trajectory new_trajectory = add_interruption(
        old_entry->trajectory, interruption_trajectory, delay);

  const Version new_version = ++_pimpl->latest_version;
  Change change = Database::Change::make_interrupt(
        id, std::move(interruption_trajectory), delay, new_version);

  old_entry->succeeded_by = _pimpl->add_entry(
      std::make_shared<internal::Entry>(
        std::move(new_trajectory),
        new_version,
        old_entry,
        std::make_unique<Change>(std::move(change))));

  return new_version;
}

//==============================================================================
Version Database::delay(
    const Version id,
    const Time from,
    const Duration delay)
{
  const internal::EntryPtr old_entry =
      _pimpl->get_entry_iterator(id, "delay")->second;

  Trajectory new_trajectory = add_delay(
        old_entry->trajectory, from, delay);

  const Version new_version = ++_pimpl->latest_version;
  Change change = Database::Change::make_delay(id, from, delay, new_version);

  old_entry->succeeded_by = _pimpl->add_entry(
        std::make_shared<internal::Entry>(
          std::move(new_trajectory),
          new_version,
          old_entry,
          std::make_unique<Change>(std::move(change))));

  return new_version;
}

//==============================================================================
Version Database::replace(
    Version previous_id,
    Trajectory trajectory)
{
  const internal::EntryPtr old_entry =
      _pimpl->get_entry_iterator(previous_id, "replacement")->second;

  const Version new_version = ++_pimpl->latest_version;
  internal::EntryPtr new_entry =
      std::make_shared<internal::Entry>(
        std::move(trajectory),
        new_version,
        old_entry);

  new_entry->change = std::make_unique<Change>(
        Change::Implementation::make_replace_ref(
          previous_id, &new_entry->trajectory, new_version));

  old_entry->succeeded_by = _pimpl->add_entry(new_entry);

  return new_version;
}

//==============================================================================
Version Database::erase(Version id)
{
  const internal::EntryPtr old_entry =
      _pimpl->get_entry_iterator(id, "erasure")->second;

  const Version new_version = ++_pimpl->latest_version;

  old_entry->succeeded_by = _pimpl->add_entry(
        std::make_shared<internal::Entry>(
          Trajectory{old_entry->trajectory.get_map_name()},
          new_version,
          old_entry,
          std::make_unique<Change>(Change::make_erase(id, new_version))), true);

  return new_version;
}

//==============================================================================
Version Database::cull(Time time)
{
  _pimpl->cull(++_pimpl->latest_version, time);

  return _pimpl->latest_version;
}

} // namespace schedule

} // namespace rmf_traffic
