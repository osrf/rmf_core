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

#include "ChangeInternal.hpp"
#include "InconsistenciesInternal.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"
#include "debug_Database.hpp"
#include "internal_Snapshot.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"

#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_utils/Modular.hpp>

#include <algorithm>
#include <list>

namespace rmf_traffic {
namespace schedule {

namespace {

//==============================================================================
Writer::Input deep_copy_input(Writer::Input input)
{
  Writer::Input copy;
  for (const auto& item : input)
  {
    auto copy_item = Writer::Item{
      item.id,
      std::make_shared<Route>(*item.route)};

    copy.emplace_back(std::move(copy_item));
  }

  return copy;
}

} // anonymous namespace

//==============================================================================
class Database::Implementation
{
public:

  struct ParticipantState;

  struct Transition;
  using TransitionPtr = std::unique_ptr<Transition>;
  using ConstTransitionPtr = std::unique_ptr<const Transition>;

  struct RouteEntry;
  using RouteEntryPtr = std::shared_ptr<RouteEntry>;

  struct RouteStorage
  {
    RouteEntryPtr entry;
    std::shared_ptr<void> timeline_handle;
  };

  struct Transition
  {
    // If this has a delay value, then the change is a delay to the route.
    // If this contains a nullopt, then the change is an erasure.
    rmf_utils::optional<Change::Delay::Implementation> delay;

    // The previous route entry that this transition is based on.
    RouteStorage predecessor;
  };

  struct RouteEntry
  {
    // ===== Mandatory fields for a Timeline Entry =====
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<const ParticipantDescription> description;

    // ===== Additional fields for this timeline entry =====
    // TODO(MXG): Consider defining a base Timeline::Entry class, and then use
    // templates to automatically mix these custom fields with the required
    // fields of the base Entry
    Version schedule_version;
    TransitionPtr transition;
    std::weak_ptr<RouteEntry> successor;
  };

  Timeline<RouteEntry> timeline;

  // TODO(MXG): Should storage be merged with state?
  using ParticipantStorage = std::unordered_map<RouteId, RouteStorage>;

  struct ParticipantState
  {
    std::unordered_set<RouteId> active_routes;
    std::unique_ptr<InconsistencyTracker> tracker;
    ParticipantStorage storage;
    const std::shared_ptr<const ParticipantDescription> description;
    const Version initial_schedule_version;
  };
  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  // This violates the single-source-of-truth principle, but it helps make it
  // more efficient to create snapshots
  using ParticipantDescriptions =
    std::unordered_map<ParticipantId,
      std::shared_ptr<const ParticipantDescription>
    >;
  ParticipantDescriptions descriptions;

  using ParticipantRegistrationVersions = std::map<Version, ParticipantId>;
  ParticipantRegistrationVersions add_participant_version;

  struct RemoveParticipantInfo
  {
    ParticipantId id;
    Version original_version;
  };
  using ParticipantUnregistrationVersion =
    std::map<Version, RemoveParticipantInfo>;
  ParticipantUnregistrationVersion remove_participant_version;

  using ParticipantRegistrationTime = std::map<Time, Version>;
  ParticipantRegistrationTime remove_participant_time;

  // NOTE(MXG): We store this record of inconsistency ranges here as a single
  // group that covers all participants in order to make it easy for us to share
  // it externally using the inconsistencies() function.
  //
  // This field should only be modified by InconsistencyTracker or
  // unregister_participant. We are also trusting the InconsistencyTracker
  // instance of each ParticipantState to not touch any other ParticipantState's
  // entry in this field.
  Inconsistencies inconsistencies;

  using StagedChanges =
    std::unordered_map<
    ParticipantId,
    std::map<ItineraryVersion, Change>>;
  StagedChanges staged_changes;

  std::unordered_set<ParticipantId> participant_ids;

  Version schedule_version = 0;

  struct CullInfo
  {
    Change::Cull cull;
    Version version;
  };

  rmf_utils::optional<CullInfo> last_cull;

  /// The current time is used to know when participants can be culled after
  /// getting unregistered
  rmf_traffic::Time current_time = rmf_traffic::Time(rmf_traffic::Duration(0));

  /// This function verifies that the route IDs specified in the input are not
  /// already being used. If that ever happens, it is indicative of a bug or a
  /// malformed input into the database.
  ///
  /// This returns a vector of pointers to RouteEntryPtr objects. The entries in
  /// that vector can be used to efficiently populate the route information,
  /// saving us the cost of a second map lookup later on.
  std::vector<RouteStorage*> check_route_ids(
    ParticipantState& state,
    const Input& input)
  {
    // NOTE(MXG): We store references to the values of the route entries,
    // because iterators to a std::unordered_map can be invalidated by
    // insertions, but pointers and references to the values inside the
    // container to do not get invalidated by inserstion. Source:
    // https://en.cppreference.com/w/cpp/container/unordered_map#Iterator_invalidation
    std::vector<RouteStorage*> entries;
    entries.reserve(input.size());

    ParticipantStorage& storage = state.storage;

    // Verify that the new route IDs do not overlap any that are still in the
    // database
    for (const Item& item : input)
    {
      const auto insertion = storage.insert({item.id, RouteStorage()});

      // If the result of this insertion attempt was anything besides a nullptr,
      // then that means this route ID was already taken, so we should reject
      // this itinerary change.
      if (insertion.first->second.entry)
      {
        // *INDENT-OFF*
        throw std::runtime_error(
          "[Database::set] New route ID [" + std::to_string(item.id)
          + "] collides with one already in the database");
        // *INDENT-ON*
      }

      entries.push_back(&insertion.first->second);
    }

    return entries;
  }

  /// This function is used to insert items into the Database. This function
  /// assumes that the items have already been fully validated and that there
  /// would be no negative side-effects to entering them.
  void insert_items(
    ParticipantId participant,
    ParticipantState& state,
    const std::vector<RouteStorage*>& entries,
    const Input& input)
  {
    assert(entries.size() == input.size());

    for (std::size_t i = 0; i < input.size(); ++i)
    {
      const auto& item = input[i];
      const RouteId route_id = item.id;
      state.active_routes.insert(route_id);

      RouteStorage& entry_storage = *entries[i];
      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          item.route,
          participant,
          route_id,
          state.description,
          schedule_version,
          nullptr,
          RouteEntryPtr()
        });

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }

  void apply_delay(
    ParticipantId participant,
    ParticipantState& state,
    Duration delay)
  {
    ParticipantStorage& storage = state.storage;
    for (const RouteId id : state.active_routes)
    {
      assert(storage.find(id) != storage.end());
      auto& entry_storage = storage.at(id);
      const Trajectory& old_trajectory =
        entry_storage.entry->route->trajectory();

      assert(old_trajectory.start_time());

      auto delayed = schedule::apply_delay(old_trajectory, delay);
      if (!delayed)
        continue;

      auto new_route = std::make_shared<Route>(
        entry_storage.entry->route->map(), std::move(*delayed));

      auto transition = std::make_unique<Transition>(
        Transition{
          Change::Delay::Implementation{delay},
          std::move(entry_storage)
        });

      // NOTE(MXG): The previous contents of entry have been moved into the
      // predecessor field of transition, so we are free to refill entry with
      // the newly created data.
      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          std::move(new_route),
          participant,
          id,
          state.description,
          schedule_version,
          std::move(transition),
          RouteEntryPtr()
        });

      entry_storage.entry->transition->predecessor.entry->successor =
        entry_storage.entry;

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }

  void erase_routes(
    ParticipantId participant,
    ParticipantState& state,
    const std::unordered_set<RouteId>& routes)
  {
    ParticipantStorage& storage = state.storage;
    for (const RouteId id : routes)
    {
      assert(storage.find(id) != storage.end());
      auto& entry_storage = storage.at(id);

      auto transition = std::make_unique<Transition>(
        Transition{
          rmf_utils::nullopt,
          std::move(entry_storage)
        });

      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          nullptr,
          participant,
          id,
          state.description,
          schedule_version,
          std::move(transition),
          RouteEntryPtr()
        });

      entry_storage.entry->transition->predecessor.entry->successor =
        entry_storage.entry;

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }

    // TODO(MXG): Consider erasing the routes from the active_routes field of
    // the state using this function. It gets tricky, though since the "erase"
    // argument is sometimes a reference to the active_routes field.
  }

  ParticipantId get_next_participant_id()
  {
    // This will cycle through the set of currently active participant IDs until
    // it finds a value which is not already taken. If it cycles through the
    // entire set of possible values and cannot find a value that is available,
    // then we will quit and throw an exception. Note that it is
    // incomprehensible to have that many participants in the schedule.
    const ParticipantId initial_suggestion = _next_participant_id;
    do
    {
      const auto insertion = participant_ids.insert(_next_participant_id);
      ++_next_participant_id;
      if (insertion.second)
        return *insertion.first;

    } while (_next_participant_id != initial_suggestion);

    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::Implementation::get_next_participant_id] There are no "
      "remaining Participant ID values available. This should never happen."
      " Please report this as a serious bug.");
    // *INDENT-ON*
  }

private:
  ParticipantId _next_participant_id = 0;
};

//==============================================================================
std::size_t Database::Debug::current_entry_history_count(
  const Database& database)
{
  std::size_t count = 0;
  for (const auto& p : database._pimpl->states)
    count += p.second.storage.size();

  return count;
}

//==============================================================================
std::size_t Database::Debug::current_removed_participant_count(
  const Database& database)
{
  return database._pimpl->remove_participant_version.size();
}

//==============================================================================
rmf_utils::optional<Writer::Input> Database::Debug::get_itinerary(
  const Database& database,
  const ParticipantId participant)
{
  const auto state_it = database._pimpl->states.find(participant);
  if (state_it == database._pimpl->states.end())
    return rmf_utils::nullopt;

  const Implementation::ParticipantState& state = state_it->second;

  Writer::Input itinerary;
  itinerary.reserve(state.active_routes.size());
  for (const RouteId route : state.active_routes)
    itinerary.push_back({route, state.storage.at(route).entry->route});

  return std::move(itinerary);
}

//==============================================================================
void Database::set(
  ParticipantId participant,
  const Input& input,
  ItineraryVersion version)
{
  auto itinerary = deep_copy_input(input);

  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::set] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version, true))
  {
    ticket->set([=]() { this->set(participant, itinerary, version); });
    return;
  }

  std::vector<Implementation::RouteStorage*> entries =
    _pimpl->check_route_ids(state, itinerary);

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;

  // Erase the routes that are currently active
  _pimpl->erase_routes(participant, state, state.active_routes);

  // Clear the list of routes that are currently active
  state.active_routes.clear();

  // Insert the new routes into the current itinerary
  _pimpl->insert_items(participant, state, entries, input);
}

//==============================================================================
void Database::extend(
  ParticipantId participant,
  const Input& input,
  ItineraryVersion version)
{
  auto routes = deep_copy_input(input);

  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::extend] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  // Check if the version on this change has any inconsistencies
  if (auto ticket = state.tracker->check(version))
  {
    // If we got a ticket from the inconsistency tracker, then pass along a
    // callback to call this
    ticket->set([=]() { this->extend(participant, routes, version); });
    return;
  }

  std::vector<Implementation::RouteStorage*> entries =
    _pimpl->check_route_ids(state, routes);

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;

  _pimpl->insert_items(participant, state, entries, input);
}

//==============================================================================
void Database::delay(
  ParticipantId participant,
  Duration delay,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::delay] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=]() { this->delay(participant, delay, version); });
    return;
  }

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;
  _pimpl->apply_delay(participant, state, delay);
}

//==============================================================================
void Database::erase(
  ParticipantId participant,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::erase] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=]() { this->erase(participant, version); });
    return;
  }

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;
  _pimpl->erase_routes(participant, state, state.active_routes);
  state.active_routes.clear();
}

//==============================================================================
void Database::erase(
  ParticipantId participant,
  const std::vector<RouteId>& routes,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::erase] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=]() { this->erase(participant, routes, version); });
    return;
  }

  std::unordered_set<RouteId> route_set;
  route_set.reserve(routes.size());
  for (const RouteId id : routes)
  {
    if (state.active_routes.count(id) == 0)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[Database::erase] The route with ID [" + std::to_string(id)
        + "] is not active!");
      // *INDENT-ON*
    }

    route_set.insert(id);
  }

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;
  _pimpl->erase_routes(participant, state, route_set);
  for (const RouteId id : routes)
    state.active_routes.erase(id);
}

//==============================================================================
ParticipantId Database::register_participant(
  ParticipantDescription description)
{
  const ParticipantId id = _pimpl->get_next_participant_id();
  auto tracker = Inconsistencies::Implementation::register_participant(
    _pimpl->inconsistencies, id);

  const Version version = ++_pimpl->schedule_version;

  const auto description_ptr =
    std::make_shared<ParticipantDescription>(std::move(description));

  _pimpl->states.insert(
    std::make_pair(
      id,
      Implementation::ParticipantState{
        {},
        std::move(tracker),
        {},
        description_ptr,
        version
      }));

  _pimpl->descriptions.insert({id, description_ptr});

  _pimpl->add_participant_version[version] = id;
  return id;
}

//==============================================================================
void Database::unregister_participant(
  ParticipantId participant)
{
  const auto id_it = _pimpl->participant_ids.find(participant);
  const auto state_it = _pimpl->states.find(participant);

  if (id_it == _pimpl->participant_ids.end()
    && state_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::unregister_participant] Requested unregistering an "
      "inactive participant ID [" + std::to_string(participant) + "]");
    // *INDENT-ON*
  }
  else if (id_it == _pimpl->participant_ids.end()
    || state_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::unregister_participant] Inconsistency in participant "
      "registration ["
      + std::to_string(id_it == _pimpl->participant_ids.end()) + ":"
      + std::to_string(state_it == _pimpl->states.end())
      + "]. Please report this as a serious bug!");
    // *INDENT-ON*
  }

  _pimpl->inconsistencies._pimpl->unregister_participant(participant);

  const Version initial_version = state_it->second.initial_schedule_version;
  _pimpl->add_participant_version.erase(initial_version);

  _pimpl->participant_ids.erase(id_it);
  _pimpl->states.erase(state_it);
  _pimpl->descriptions.erase(participant);

  const Version version = ++_pimpl->schedule_version;
  _pimpl->remove_participant_version[version] = {participant, initial_version};
  _pimpl->remove_participant_time[_pimpl->current_time] = version;
}

//==============================================================================
namespace {

//==============================================================================
const Database::Implementation::RouteEntry* get_most_recent(
  const Database::Implementation::RouteEntry* from)
{
  assert(from);
  while (const auto successor = from->successor.lock())
    from = successor.get();

  return from;
}

//==============================================================================
struct Delay
{
  Duration duration;
};

//==============================================================================
struct ParticipantChanges
{
  std::vector<Change::Add::Item> additions;
  std::map<Version, Delay> delays;
  std::vector<RouteId> erasures;
};

//==============================================================================
class PatchRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  PatchRelevanceInspector(Version after)
  : _after(after)
  {
    // Do nothing
  }

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  const RouteEntry* get_last_known_ancestor(const RouteEntry* from) const
  {
    assert(from);
    while (from && rmf_utils::modular(_after).less_than(from->schedule_version))
    {
      if (from->transition)
        from = from->transition->predecessor.entry.get();
      else
        return nullptr;
    }

    while (const auto successor = from->successor.lock())
    {
      if (rmf_utils::modular(_after).less_than(successor->schedule_version))
        break;

      from = successor.get();
    }

    return from;
  }

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    const RouteEntry* const last = get_last_known_ancestor(entry);
    const RouteEntry* const newest = get_most_recent(entry);

    if (last == newest)
    {
      // There are no changes for this route to give the mirror
      return;
    }

    if (last && last->route && relevant(*last))
    {
      // The mirror knew about a previous version of this route
      if (newest->route && relevant(*newest))
      {
        // The newest version of this route is relevant to the mirror
        const RouteEntry* traverse = newest;
        ParticipantChanges& p_changes = changes[newest->participant];
        while (traverse != last)
        {
          const auto* const transition = traverse->transition.get();
          assert(transition);
          assert(transition->delay);
          const auto& delay = *transition->delay;
          const auto insertion = p_changes.delays.insert(
            std::make_pair(
              traverse->schedule_version,
              Delay{
                delay.duration
              }));
#ifndef NDEBUG
          // When compiling in debug mode, if we see a duplicate insertion,
          // let's make sure that the previously entered data matches what we
          // wanted to enter just now.
          if (!insertion.second)
          {
            const Delay& previous = insertion.first->second;
            assert(previous.duration == delay.duration);
          }
#else
          // When compiling in release mode, cast the return value to void to
          // suppress compiler warnings.
          (void)(insertion);
#endif // NDEBUG
          traverse = traverse->transition->predecessor.entry.get();
        }
      }
      else
      {
        // The newest version of this route is not relevant to the mirror, so
        // we will erase it from the mirror.
        changes[newest->participant].erasures.emplace_back(newest->route_id);
      }
    }
    else
    {
      // No version of this route has been seen by the mirror
      if (newest->route && relevant(*newest))
      {
        // The newest version of this route is relevant to the mirror
        changes[newest->participant].additions.emplace_back(
          Change::Add::Item{
            newest->route_id,
            newest->route
          });
      }
      else
      {
        // Ignore this route. The mirror has no need to know about it.
      }
    }
  }

private:
  const Version _after;
};

//==============================================================================
class FirstPatchRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    const RouteEntry* const newest = get_most_recent(entry);
    if (newest->route && relevant(*newest))
    {
      changes[newest->participant].additions.emplace_back(
        Change::Add::Item{
          newest->route_id,
          newest->route
        });
    }
  }
};

//==============================================================================
class ViewRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    entry = get_most_recent(entry);
    if (entry->route && relevant(*entry))
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
class ViewerAfterRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  const Version after;

  ViewerAfterRelevanceInspector(Version _after)
  : after(_after)
  {
    // Do nothing
  }

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    entry = get_most_recent(entry);
    if (rmf_utils::modular(after).less_than(entry->schedule_version)
      && entry->route && relevant(*entry))
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
class CullRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  CullRelevanceInspector(Time cull_time)
  : _cull_time(cull_time)
  {
    // Do nothing
  }

  using RouteEntry = Database::Implementation::RouteEntry;

  struct Info
  {
    ParticipantId participant;
    RouteId route_id;
  };

  std::vector<Info> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& /*relevant*/) final
  {
    while (const auto successor = entry->successor.lock())
    {
      if (!successor->route)
        break;

      entry = successor.get();
    }

    assert(entry->route->trajectory().finish_time());
    if (*entry->route->trajectory().finish_time() < _cull_time)
    {
      routes.emplace_back(Info{entry->participant, entry->route_id});
    }
  }

private:
  Time _cull_time;
};

} // anonymous namespace

//==============================================================================
Viewer::View Database::query(const Query& parameters) const
{
  return query(parameters.spacetime(), parameters.participants());
}

//==============================================================================
Viewer::View Database::query(
  const Query::Spacetime& spacetime,
  const Query::Participants& participants) const
{
  ViewRelevanceInspector inspector;
  _pimpl->timeline.inspect(spacetime, participants, inspector);
  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Database::participant_ids() const
{
  return _pimpl->participant_ids;
}

//==============================================================================
std::shared_ptr<const ParticipantDescription> Database::get_participant(
  std::size_t participant_id) const
{
  const auto state_it = _pimpl->descriptions.find(participant_id);
  if (state_it == _pimpl->descriptions.end())
    return nullptr;

  return state_it->second;
}

//==============================================================================
rmf_utils::optional<Itinerary> Database::get_itinerary(
  std::size_t participant_id) const
{
  const auto state_it = _pimpl->states.find(participant_id);
  if (state_it == _pimpl->states.end())
    return rmf_utils::nullopt;

  const Implementation::ParticipantState& state = state_it->second;

  Itinerary itinerary;
  itinerary.reserve(state.active_routes.size());
  for (const RouteId route : state.active_routes)
    itinerary.push_back(state.storage.at(route).entry->route);

  return std::move(itinerary);
}

//==============================================================================
Version Database::latest_version() const
{
  return _pimpl->schedule_version;
}

//==============================================================================
std::shared_ptr<const Snapshot> Database::snapshot() const
{
  using SnapshotType =
    SnapshotImplementation<Implementation::RouteEntry, ViewRelevanceInspector>;

  return std::make_shared<SnapshotType>(
    _pimpl->timeline.snapshot(),
    _pimpl->participant_ids,
    _pimpl->descriptions,
    _pimpl->schedule_version);
}

//==============================================================================
Database::Database()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
const Inconsistencies& Database::inconsistencies() const
{
  return _pimpl->inconsistencies;
}

//==============================================================================
auto Database::changes(
  const Query& parameters,
  rmf_utils::optional<Version> after) const -> Patch
{
  std::unordered_map<ParticipantId, ParticipantChanges> changes;
  if (after)
  {
    PatchRelevanceInspector inspector(*after);
    _pimpl->timeline.inspect(
      parameters.spacetime(), parameters.participants(), inspector);

    changes = inspector.changes;
  }
  else
  {
    FirstPatchRelevanceInspector inspector;
    _pimpl->timeline.inspect(
      parameters.spacetime(), parameters.participants(), inspector);

    changes = inspector.changes;
  }

  std::vector<Patch::Participant> part_patches;
  for (const auto& p : changes)
  {
    std::vector<Change::Delay> delays;
    for (const auto& d : p.second.delays)
    {
      delays.emplace_back(
        Change::Delay{
          d.second.duration
        });
    }

    part_patches.emplace_back(
      Patch::Participant{
        p.first,
        Change::Erase(std::move(p.second.erasures)),
        std::move(delays),
        Change::Add(std::move(p.second.additions))
      });
  }

  std::vector<Change::RegisterParticipant> registered;
  std::vector<Change::UnregisterParticipant> unregistered;
  if (after)
  {
    const Version after_v = *after;

    auto add_it = _pimpl->add_participant_version.upper_bound(after_v);
    for (; add_it != _pimpl->add_participant_version.end(); ++add_it)
    {
      const auto p_it = _pimpl->states.find(add_it->second);
      assert(p_it != _pimpl->states.end());
      registered.emplace_back(p_it->first, *p_it->second.description);
    }

    auto remove_it = _pimpl->remove_participant_version.upper_bound(after_v);
    for (; remove_it != _pimpl->remove_participant_version.end(); ++remove_it)
    {
      // We should only unregister this if it was registered before the last
      // update to this mirror
      if (remove_it->second.original_version <= *after)
        unregistered.emplace_back(remove_it->second.id);
    }
  }
  else
  {
    // If this is a mirror's first pull from the database, then we should send
    // all the participant information.
    for (const auto& p : _pimpl->states)
      registered.emplace_back(p.first, *p.second.description);

    // We do not need to mention any participants that have unregistered.
  }

  rmf_utils::optional<Change::Cull> cull;
  if (_pimpl->last_cull && after && *after < _pimpl->last_cull->version)
  {
    cull = _pimpl->last_cull->cull;
  }

  return Patch(
    std::move(unregistered),
    std::move(registered),
    std::move(part_patches),
    cull,
    _pimpl->schedule_version);
}

//==============================================================================
Viewer::View Database::query(const Query& parameters, const Version after) const
{
  ViewerAfterRelevanceInspector inspector{after};
  _pimpl->timeline.inspect(
    parameters.spacetime(), parameters.participants(), inspector);

  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
Version Database::cull(Time time)
{
  Query::Spacetime spacetime;
  spacetime.query_timespan().set_upper_time_bound(time);

  CullRelevanceInspector inspector(time);
  _pimpl->timeline.inspect(
    spacetime, Query::Participants::make_all(), inspector);

  // TODO(MXG) This iterating could probably be made more efficient by grouping
  // together the culls of each participant.
  for (const auto& route : inspector.routes)
  {
    auto p_it = _pimpl->states.find(route.participant);
    assert(p_it != _pimpl->states.end());

    auto& storage = p_it->second.storage;
    const auto r_it = storage.find(route.route_id);
    assert(r_it != storage.end());

    storage.erase(r_it);
  }

  _pimpl->timeline.cull(time);

  // Erase all trace of participants that were removed before the culling time.
  const auto p_cull_begin = _pimpl->remove_participant_time.begin();
  const auto p_cull_end = _pimpl->remove_participant_time.upper_bound(time);
  for (auto p_cull_it = p_cull_begin; p_cull_it != p_cull_end; ++p_cull_it)
  {
    const auto remove_it =
      _pimpl->remove_participant_version.find(p_cull_it->second);
    assert(remove_it != _pimpl->remove_participant_version.end());

    _pimpl->remove_participant_version.erase(remove_it);
  }

  if (p_cull_begin != p_cull_end)
    _pimpl->remove_participant_time.erase(p_cull_begin, p_cull_end);

  // Update the version of the schedule
  ++_pimpl->schedule_version;

  // Record the occurrence of this cull
  _pimpl->last_cull = Implementation::CullInfo{
    Change::Cull(time),
    _pimpl->schedule_version
  };

  return _pimpl->schedule_version;
}

//==============================================================================
void Database::set_current_time(Time time)
{
  _pimpl->current_time = time;
}

//==============================================================================
ItineraryVersion Database::itinerary_version(ParticipantId participant) const
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::itinerary_version] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  return p_it->second.tracker->last_known_version();
}

} // namespace schedule
} // namespace rmf_traffic
