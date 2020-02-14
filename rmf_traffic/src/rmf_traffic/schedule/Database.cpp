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
#include "Modular.hpp"
#include "Timeline.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"

#include <rmf_traffic/schedule/Database.hpp>

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

  struct Transition
  {
    // If this has a delay value, then the change is a delay to the route.
    // If this contains a nullopt, then the change is an erasure.
    rmf_utils::optional<Change::Delay::Implementation> delay;

    // The previous route entry that this transition is based on.
    RouteEntryPtr predecessor;
  };

  struct RouteEntry
  {
    // ===== Mandatory fields for a Timeline Entry =====
    ConstRoutePtr route;
    ParticipantId participant;
    RouteId route_id;
    std::shared_ptr<void> timeline_handle;

    // ===== Additional fields for this timeline entry =====
    // TODO(MXG): Consider defining a base Timeline::Entry class, and then use
    // templates to automatically mix these custom fields with the required
    // fields of the base Entry
    Version schedule_version;
    TransitionPtr transition;
    RouteEntry* successor;
  };

  Timeline<RouteEntry> timeline;

  // TODO(MXG): Should storage be merged with state?
  using ParticipantStorage = std::unordered_map<RouteId, RouteEntryPtr>;

  struct ParticipantState
  {
    std::unordered_set<RouteId> active_routes;
    std::unique_ptr<InconsistencyTracker> tracker;
    ParticipantStorage storage;
    const ParticipantDescription description;
    const Version initial_schedule_version;
  };
  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  using ParticipantRegistrationVersions = std::map<Version, ParticipantId>;
  ParticipantRegistrationVersions add_participant_version;
  ParticipantRegistrationVersions remove_participant_version;

  using ParticipantRegistrationTime = std::map<Time, ParticipantId>;
  ParticipantRegistrationTime add_participant_time;
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

  rmf_utils::optional<Change::Cull> last_cull;

  /// This function verifies that the route IDs specified in the input are not
  /// already being used. If that ever happens, it is indicative of a bug or a
  /// malformed input into the database.
  ///
  /// This returns a vector of pointers to RouteEntryPtr objects. The entries in
  /// that vector can be used to efficiently populate the route information,
  /// saving us the cost of a second map lookup later on.
  std::vector<RouteEntryPtr*> check_route_ids(
      ParticipantState& state,
      const Input& input)
  {
    // NOTE(MXG): We store references to the values of the route entries,
    // because iterators to a std::unordered_map can be invalidated by
    // insertions, but pointers and references to the values inside the
    // container to do not get invalidated by inserstion. Source:
    // https://en.cppreference.com/w/cpp/container/unordered_map#Iterator_invalidation
    std::vector<RouteEntryPtr*> entries;
    entries.reserve(input.size());

    ParticipantStorage& storage = state.storage;

    // Verify that the new route IDs do not overlap any that are still in the
    // database
    for (const Item& item : input)
    {
      const auto insertion = storage.insert(std::make_pair(item.id, nullptr));

      // If the result of this insertion attempt was anything besides a nullptr,
      // then that means this route ID was already taken, so we should reject
      // this itinerary change.
      if (insertion.first->second)
      {
        throw std::runtime_error(
              "[Database::set] New route ID [" + std::to_string(item.id)
              + "] collides with one already in the database");
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
      const std::vector<RouteEntryPtr*>& entries,
      const Input& input)
  {
    assert(entries.size() == input.size());

    for (std::size_t i=0; i < input.size(); ++i)
    {
      const auto& item = input[i];
      const RouteId route_id = item.id;
      state.active_routes.insert(route_id);

      RouteEntryPtr& entry = *entries[i];
      entry = std::make_unique<RouteEntry>(
            RouteEntry{
              item.route,
              participant,
              route_id,
              nullptr,
              schedule_version,
              nullptr,
              nullptr
            });

      timeline.insert(*entry);
    }
  }

  void apply_delay(
      ParticipantId participant,
      ParticipantState& state,
      Time from,
      Duration delay)
  {
    ParticipantStorage& storage = state.storage;
    for (const RouteId id : state.active_routes)
    {
      assert(storage.find(id) != storage.end());
      auto& entry = storage.at(id);
      const Trajectory& old_trajectory = entry->route->trajectory();
      assert(old_trajectory.start_time());

      if (*old_trajectory.finish_time() < from)
        continue;

      Trajectory new_trajectory = old_trajectory;
      if (from < *old_trajectory.start_time())
      {
        new_trajectory.begin()->adjust_times(delay);
      }
      else
      {
        new_trajectory.find(from)->adjust_times(delay);
      }

      auto new_route = std::make_shared<Route>(
            entry->route->map(), std::move(new_trajectory));

      auto transition = std::make_unique<Transition>(
            Transition{
              Change::Delay::Implementation{from, delay},
              std::move(entry)
            });

      // NOTE(MXG): The previous contents of entry have been moved into the
      // predecessor field of transition, so we are free to refill entry with
      // the newly created data.
      entry = std::make_unique<RouteEntry>(
            RouteEntry{
              std::move(new_route),
              participant,
              id,
              nullptr,
              schedule_version,
              std::move(transition),
              nullptr
            });

      entry->transition->predecessor->successor = entry.get();
      timeline.insert(*entry);
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
      auto& old_entry = storage.at(id);

      auto transition = std::make_unique<Transition>(
            Transition{
              rmf_utils::nullopt,
              std::move(old_entry)
            });

      RouteEntryPtr entry = std::make_unique<RouteEntry>(
            RouteEntry{
              nullptr,
              participant,
              id,
              nullptr,
              schedule_version,
              std::move(transition),
              nullptr
            });

      entry->transition->predecessor->successor = entry.get();
      timeline.insert(*entry);
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

    throw std::runtime_error(
          "[Database::Implementation::get_next_participant_id] There are no "
          "remaining Participant ID values available. This should never happen."
          " Please report this as a serious bug.");
  }

private:
  ParticipantId _next_participant_id = 0;
};

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
    throw std::runtime_error(
          "[Database::set] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  if (modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version, true))
  {
    ticket->set([=](){ this->set(participant, itinerary, version); });
    return;
  }

  std::vector<Implementation::RouteEntryPtr*> entries =
      _pimpl->check_route_ids(state, itinerary);

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;

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
    throw std::runtime_error(
          "[Database::extend] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (modular(version).less_than(state.tracker->expected_version()))
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
    ticket->set([=](){ this->extend(participant, routes, version); });
    return;
  }

  std::vector<Implementation::RouteEntryPtr*> entries =
      _pimpl->check_route_ids(state, routes);

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;

  _pimpl->insert_items(participant, state, entries, input);
}

//==============================================================================
void Database::delay(
    ParticipantId participant,
    Time from,
    Duration delay,
    ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    throw std::runtime_error(
          "[Database::delay] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=](){ this->delay(participant, from, delay, version); });
    return;
  }

  //======== All validation is complete ===========
  ++_pimpl->schedule_version;
  _pimpl->apply_delay(participant, state, from, delay);
}

//==============================================================================
void Database::erase(
    ParticipantId participant,
    ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    throw std::runtime_error(
          "[Database::erase] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=](){ this->erase(participant, version); });
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
    throw std::runtime_error(
          "[Database::erase] No participant with ID ["
          + std::to_string(participant) + "]");
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=](){ this->erase(participant, routes, version); });
    return;
  }

  std::unordered_set<RouteId> route_set;
  route_set.reserve(routes.size());
  for (const RouteId id : routes)
  {
    if (state.active_routes.count(id) == 0)
    {
      throw std::runtime_error(
            "[Database::erase] The route with ID [" + std::to_string(id)
            + "] is not active!");
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
    ParticipantDescription participant_info,
    Time time)
{
  const ParticipantId id = _pimpl->get_next_participant_id();
  auto tracker = Inconsistencies::Implementation::register_participant(
        _pimpl->inconsistencies, id);

  const Version version = ++_pimpl->schedule_version;

  _pimpl->states.insert(
        std::make_pair(
          id,
          Implementation::ParticipantState{
            {},
            std::move(tracker),
            {},
            std::move(participant_info),
            version
          }));

  _pimpl->add_participant_version[version] = id;
  _pimpl->add_participant_time[time] = id;
}

//==============================================================================
void Database::unregister_participant(
    ParticipantId participant,
    const Time time)
{
  const auto id_it = _pimpl->participant_ids.find(participant);
  const auto state_it = _pimpl->states.find(participant);

  if (id_it == _pimpl->participant_ids.end()
      && state_it == _pimpl->states.end())
  {
    throw std::runtime_error(
          "[Database::unregister_participant] Requested unregistering an "
          "inactive participant ID [" + std::to_string(participant) + "]");
  }
  else if (id_it == _pimpl->participant_ids.end()
           || state_it == _pimpl->states.end())
  {
    throw std::runtime_error(
          "[Database::unregister_participant] Inconsistency in participant "
          "registration ["
          + std::to_string(id_it == _pimpl->participant_ids.end()) + ":"
          + std::to_string(state_it == _pimpl->states.end())
          + "]. Please report this as a serious bug!");
  }

  _pimpl->participant_ids.erase(id_it);
  _pimpl->states.erase(state_it);

  const Version version = ++_pimpl->schedule_version;
  _pimpl->remove_participant_version[version] = participant;
  _pimpl->remove_participant_time[time] = participant;
}

//==============================================================================
const std::unordered_set<ParticipantId>& Database::participant_ids() const
{
  return _pimpl->participant_ids;
}

//==============================================================================
rmf_utils::optional<const ParticipantDescription&> Database::get_participant(
    std::size_t participant_id) const
{
  const auto state_it = _pimpl->states.find(participant_id);
  if (state_it == _pimpl->states.end())
    return rmf_utils::nullopt;

  return state_it->second.description;
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
    itinerary.push_back(state.storage.at(route)->route);

  return itinerary;
}

//==============================================================================
Version Database::latest_version() const
{
  return _pimpl->schedule_version;
}

//==============================================================================
const Inconsistencies& Database::inconsistencies() const
{
  return _pimpl->inconsistencies;
}

namespace {

//==============================================================================
const Database::Implementation::RouteEntry* get_most_recent(
    const Database::Implementation::RouteEntry* from)
{
  assert(from);
  while (from->successor)
    from = from->successor;

  return from;
}

//==============================================================================
struct Delay
{
  Time from;
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

  Version _after;

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  const RouteEntry* get_last_known_ancestor(const RouteEntry* from) const
  {
    assert(from);
    while (from && modular(_after).less_than(from->schedule_version))
    {
      if (from->transition)
        from = from->transition->predecessor.get();
      else
        return nullptr;
    }

    while (from->successor
           && modular(from->successor->schedule_version)
                .less_than_or_equal(_after))
    {
      from = from->successor;
    }

    return from;
  }

  void inspect(
      const RouteEntry* entry,
      const std::function<bool(const Route &)>& relevant) override
  {
    const RouteEntry* const last = get_last_known_ancestor(entry);
    const RouteEntry* const newest = get_most_recent(entry);

    if (last == newest)
    {
      // There are no changes for this route to give the mirror
      return;
    }

    if (last && last->route && relevant(*last->route))
    {
      // The mirror knew about a previous version of this route
      if (newest->route && relevant(*newest->route))
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
                    delay.from,
                    delay.duration
                  }));
#ifndef NDEBUG
          // When compiling in debug mode, if we see a duplicate insertion,
          // let's make sure that the previously entered data matches what we
          // wanted to enter just now.
          if (!insertion.second)
          {
            const Delay& previous = insertion.first->second;
            assert(previous.from == delay.from);
            assert(previous.duration == delay.duration);
          }
#elif
          // When compiling in release mode, cast the return value to void to
          // suppress compiler warnings.
          (void)(insertion);
#endif // NDEBUG
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
      if (newest->route && relevant(*newest->route))
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
};

//==============================================================================
class FirstPatchRelevantInspector
    : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  void inspect(
      const RouteEntry* entry,
      const std::function<bool(const Route&)>& relevant) final
  {
    const RouteEntry* const newest = get_most_recent(entry);
    if (newest->route && relevant(*newest->route))
    {
      changes[newest->participant].additions.emplace_back(
            Change::Add::Item{
              newest->route_id,
              newest->route
            });
    }
  }
};

} // anonymous namespace

//==============================================================================
Database::Database()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
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
    _pimpl->timeline.inspect(parameters, inspector);
    changes = inspector.changes;
  }
  else
  {
    FirstPatchRelevantInspector inspector;
    _pimpl->timeline.inspect(parameters, inspector);
    changes = inspector.changes;
  }

  std::vector<Patch::Participant> part_patches;
  for (const auto p : changes)
  {
    std::vector<Change::Delay> delays;
    for (const auto d : p.second.delays)
    {
      delays.emplace_back(
            Change::Delay{
              d.second.from,
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

  return Patch(
        std::move(part_patches),
        _pimpl->last_cull,
        _pimpl->schedule_version);
}

//==============================================================================
Version Database::cull(Time time)
{
  _pimpl->cull(++_pimpl->latest_version, time);

  return _pimpl->latest_version;
}

} // namespace schedule
} // namespace rmf_traffic
