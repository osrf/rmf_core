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

#include "ViewerInternal.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"

#include <rmf_traffic/schedule/Database.hpp>

#include <algorithm>

namespace rmf_traffic {
namespace schedule {

class Database::Implementation : public Viewer::Implementation
{
public:



};

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
    const ConstEntryPtr& entry,
    const std::function<bool(const ConstEntryPtr&)>& relevant)
{
  if(entry->succeeded_by)
    return;

  if(after_version
     && versions.less_or_equal(entry->schedule_version, *after_version))
    return;

  const bool needed = relevant(entry);

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
              &entry->trajectory, entry->version));
    }
  }
  else if(after_version)
  {
    // Figure out if this trajectory needs to be erased
    const ConstEntryPtr check =
        get_last_known_ancestor(entry, *after_version, versions);

    if(check)
    {
      if(relevant(check))
      {
        // This trajectory is no longer relevant to the remote mirror, so we
        // will tell the remote mirror to erase it rather than continuing to
        // transmit its change history. If a later version of this trajectory
        // becomes relevant again, we will tell it to insert it at that time.
        relevant_changes.emplace_back(
              Database::Change::make_erase(check->version, entry->version));
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
    const ConstEntryPtr& entry,
    const rmf_traffic::internal::Spacetime& spacetime)
{
  inspect(entry, [&](const ConstEntryPtr& e) -> bool {
    const Trajectory& trajectory = e->trajectory;
    if(trajectory.start_time())
    {
      return rmf_traffic::internal::detect_conflicts(
            e->trajectory, spacetime, nullptr);
    }
    else
    {
      assert(e->change->get_mode() == Database::Change::Mode::Erase);
      return false;
    }
  });
}

//==============================================================================
void ChangeRelevanceInspector::inspect(
    const ConstEntryPtr& entry,
    const Time* const lower_time_bound,
    const Time* const upper_time_bound)
{
  inspect(entry, [&](const ConstEntryPtr& e) -> bool {
    const Trajectory& trajectory = e->trajectory;
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
      assert(e->change->get_mode() == Database::Change::Mode::Erase);
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
auto Database::changes(const Query& parameters) const -> Patch
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
