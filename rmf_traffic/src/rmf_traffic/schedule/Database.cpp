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

//==============================================================================
class Database::Change::Implementation
{
public:

  Mode mode;
  Insert insert;
  Interrupt interrupt;
  Delay delay;
  Replace replace;
  Erase erase;
  Cull cull;

  Version id;

  static Change make(const Mode mode, const Version id)
  {
    Change change;
    change._pimpl->mode = mode;
    change._pimpl->id = id;

    return change;
  }

  static Change make_void()
  {
    Change change;
    change._pimpl->mode = Mode::Void;
    return change;
  }

  static Change make_insert_ref(
      const Trajectory* trajectory,
      Version id);

  // Note(MXG): We're not using make_interrupt_ref yet, and perhaps we never
  // will. In theory this function could be used to save time and memory when a
  // Database generates a Patch that includes an interrupt, but it would add
  // complexity to the implementation of Patch generation, so I'm deferring that
  // feature for later.
  static Change make_interrupt_ref(
      Version original_id,
      const Trajectory* interruption_trajectory,
      Duration delay,
      Version id);

  static Change make_replace_ref(
      Version original_id,
      const Trajectory* trajectory,
      Version id);
};

namespace {
//==============================================================================
// TODO(MXG): Consider generalizing this class using templates if it ends up
// being useful in any other context.
class DeepOrShallowTrajectory
{
public:

  const Trajectory* get() const
  {
    if(is_deep)
      return deep.get();

    return shallow;
  }

  rmf_utils::impl_ptr<const Trajectory> deep;
  const Trajectory* shallow = nullptr;
  bool is_deep = true;

};

//==============================================================================
DeepOrShallowTrajectory make_deep(const Trajectory* const deep)
{
  DeepOrShallowTrajectory traj;
  traj.is_deep = true;
  if(deep)
    traj.deep = rmf_utils::make_impl<const Trajectory>(*deep);

  return traj;
}

DeepOrShallowTrajectory make_shallow(const Trajectory* const shallow)
{
  DeepOrShallowTrajectory traj;
  traj.is_deep = false;
  traj.shallow = shallow;

  return traj;
}

} // anonymous namespace

//==============================================================================
class Database::Change::Insert::Implementation
{
public:

  /// The trajectory that was inserted
  DeepOrShallowTrajectory trajectory;

  /// This is used to create a Change whose lifetime is not tied to the Database
  /// instance that constructed it.
  static Insert make_copy(const Trajectory* const trajectory)
  {
    Insert result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_deep(trajectory)});

    return result;
  }

  /// This is used by Database instances to create a Change without the overhead
  /// of copying a Trajectory instance. The Change's lifetime will be tied to
  /// the lifetime of the Database object.
  static Insert make_ref(const Trajectory* const trajectory)
  {
    Insert result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_shallow(trajectory)});

    return result;
  }
};

//==============================================================================
Database::Change::Insert::Insert()
{
  // Do nothing
}

//==============================================================================
Database::Change::Change()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_insert(
    const Trajectory* const trajectory,
    Version id) -> Change
{
  Change change = Implementation::make(Mode::Insert, id);
  change._pimpl->insert = Insert::Implementation::make_copy(trajectory);

  return change;
}

//==============================================================================
auto Database::Change::Implementation::make_insert_ref(
    const Trajectory* const trajectory,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Insert, id);
  change._pimpl->insert = Insert::Implementation::make_ref(trajectory);

  return change;
}

//==============================================================================
class Database::Change::Interrupt::Implementation
{
public:

  DeepOrShallowTrajectory trajectory;
  Version original_id;
  Duration delay;

  template<typename... Args>
  static Interrupt make_copy(
      const Trajectory* const trajectory,
      Args&&... args)
  {
    Interrupt result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_deep(trajectory), std::forward<Args>(args)...});

    return result;
  }

  template<typename... Args>
  static Interrupt make_ref(
      const Trajectory* const trajectory,
      Args&&... args)
  {
    Interrupt result;

    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{make_shallow(trajectory),
                         std::forward<Args>(args)...});

    return result;
  }

};

//==============================================================================
Database::Change::Interrupt::Interrupt()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_interrupt(
    Version original_id,
    const Trajectory* const interruption_trajectory,
    Duration delay,
    Version id) -> Change
{
  Change change = Implementation::make(Mode::Interrupt, id);
  change._pimpl->interrupt = Interrupt::Implementation::make_copy(
        interruption_trajectory, original_id, delay);
  return change;
}

//==============================================================================
auto Database::Change::Implementation::make_interrupt_ref(
    const Version original_id,
    const Trajectory* const interruption_trajectory,
    const Duration delay,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Interrupt, id);
  change._pimpl->interrupt = Interrupt::Implementation::make_ref(
        interruption_trajectory, original_id, delay);
  return change;
}

//==============================================================================
class Database::Change::Delay::Implementation
{
public:

  Version original_id;
  Time from;
  Duration delay;

  template<typename... Args>
  static Delay make(Args&&... args)
  {
    Delay result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::forward<Args>(args)...});
    return result;
  }

};

//==============================================================================
Database::Change::Delay::Delay()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_delay(
    Version original_id,
    Time from,
    Duration delay,
    Version id) -> Change
{
  Change change = Implementation::make(Mode::Delay, id);
  change._pimpl->delay = Delay::Implementation::make(
        original_id, from, delay);
  return change;
}

//==============================================================================
class Database::Change::Replace::Implementation
{
public:

  Version original_id;
  DeepOrShallowTrajectory trajectory;

  static Replace make_copy(
      const Version original_id,
      const Trajectory* const trajectory)
  {
    Replace result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{original_id, make_deep(trajectory)});
    return result;
  }

  static Replace make_ref(
      const Version original_id,
      const Trajectory* const trajectory)
  {
    Replace result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{original_id, make_shallow(trajectory)});
    return result;
  }

};

//==============================================================================
Database::Change::Replace::Replace()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_replace(
    const Version original_id,
    const Trajectory* const trajectory,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Replace, id);
  change._pimpl->replace = Replace::Implementation::make_copy(
        original_id, trajectory);
  return change;
}

//==============================================================================
auto Database::Change::Implementation::make_replace_ref(
    const Version original_id,
    const Trajectory* const trajectory,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Replace, id);
  change._pimpl->replace = Replace::Implementation::make_ref(
        original_id, trajectory);
  return change;
}

//==============================================================================
class Database::Change::Erase::Implementation
{
public:

  Version original_id;

  static Erase make(const Version original_id)
  {
    Erase result;
    result._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{original_id});
    return result;
  }

};

//==============================================================================
Database::Change::Erase::Erase()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_erase(
    const Version original_id,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Erase, id);
  change._pimpl->erase = Erase::Implementation::make(original_id);
  return change;
}

//==============================================================================
class Database::Change::Cull::Implementation
{
public:

  Time time;

  static Cull make(Time time)
  {
    Cull result;
    result._pimpl = rmf_utils::make_impl<Implementation>(Implementation{time});
    return result;
  }

};

//==============================================================================
Database::Change::Cull::Cull()
{
  // Do nothing
}

//==============================================================================
auto Database::Change::make_cull(
    const Time time,
    const Version id) -> Change
{
  Change change = Implementation::make(Mode::Cull, id);
  change._pimpl->cull = Cull::Implementation::make(time);
  return change;
}

//==============================================================================
const Trajectory* Database::Change::Insert::trajectory() const
{
  return _pimpl->trajectory.get();
}

//==============================================================================
Version Database::Change::Interrupt::original_id() const
{
  return _pimpl->original_id;
}

//==============================================================================
const Trajectory* Database::Change::Interrupt::interruption() const
{
  return _pimpl->trajectory.get();
}

//==============================================================================
Duration Database::Change::Interrupt::delay() const
{
  return _pimpl->delay;
}

//==============================================================================
Version Database::Change::Delay::original_id() const
{
  return _pimpl->original_id;
}

//==============================================================================
Time Database::Change::Delay::from() const
{
  return _pimpl->from;
}

//==============================================================================
Duration Database::Change::Delay::duration() const
{
  return _pimpl->delay;
}

//==============================================================================
Version Database::Change::Replace::original_id() const
{
  return _pimpl->original_id;
}

//==============================================================================
const Trajectory* Database::Change::Replace::trajectory() const
{
  return _pimpl->trajectory.get();
}

//==============================================================================
Version Database::Change::Erase::original_id() const
{
  return _pimpl->original_id;
}

//==============================================================================
Time Database::Change::Cull::time() const
{
  return _pimpl->time;
}

//==============================================================================
auto Database::Change::get_mode() const -> Mode
{
  return _pimpl->mode;
}

//==============================================================================
Version Database::Change::id() const
{
  return _pimpl->id;
}

//==============================================================================
auto Database::Change::insert() const -> const Insert*
{
  if(Mode::Insert == _pimpl->mode)
    return &_pimpl->insert;

  return nullptr;
}

//==============================================================================
auto Database::Change::interrupt() const -> const Interrupt*
{
  if(Mode::Interrupt == _pimpl->mode)
    return &_pimpl->interrupt;

  return nullptr;
}

//==============================================================================
auto Database::Change::delay() const -> const Delay*
{
  if(Mode::Delay == _pimpl->mode)
    return &_pimpl->delay;

  return nullptr;
}

//==============================================================================
auto Database::Change::replace() const -> const Replace*
{
  if(Mode::Replace == _pimpl->mode)
    return &_pimpl->replace;

  return nullptr;
}

//==============================================================================
auto Database::Change::erase() const -> const Erase*
{
  if(Mode::Erase == _pimpl->mode)
    return &_pimpl->erase;

  return nullptr;
}

//==============================================================================
auto Database::Change::cull() const -> const Cull*
{
  if(Mode::Cull == _pimpl->mode)
    return &_pimpl->cull;

  return nullptr;
}

//==============================================================================
class Database::Patch::Implementation
{
public:

  std::vector<Change> changes;

  Version latest_version;

  Implementation()
  {
    // Do nothing
  }

  Implementation(std::vector<Change> _changes, Version _latest_version)
    : changes(std::move(_changes)),
      latest_version(_latest_version)
  {
    // Sort the changes to make sure they get applied in the correct order
    std::sort(changes.begin(), changes.end(),
              [](const Change& c1, const Change& c2) {
      return c1.id() < c2.id();
    });
  }

};

//==============================================================================
class Database::Patch::IterImpl
{
public:

  std::vector<Change>::const_iterator iter;

};

//==============================================================================
Database::Patch::Patch(std::vector<Change> changes, Version latest_version)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             std::move(changes), latest_version))
{
  // Do nothing
}

//==============================================================================
auto Database::Patch::begin() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.begin()});
}

//==============================================================================
auto Database::Patch::end() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.end()});
}

//==============================================================================
std::size_t Database::Patch::size() const
{
  return _pimpl->changes.size();
}

//==============================================================================
Version Database::Patch::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
Database::Patch::Patch()
{
  // Do nothing
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
  while(from && versions.less(last_known_version, from->version))
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

  if(after_version && versions.less_or_equal(entry->version, *after_version))
    return;

  const bool needed = relevant(entry);
  std::cout << " +++ needed: " << needed << std::endl;

  if(needed)
  {
    std::cout << " ++++ needed is true" << std::endl;
    // Check if this entry descends from an entry that the remote mirror does
    // not know about.
    ConstEntryPtr record_changes_from = nullptr;
    if(after_version)
    {
      std::cout << " ++++ version is specified: " << *after_version << std::endl;
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
      std::cout << " ++++ we should record changes" << std::endl;
      ConstEntryPtr record = record_changes_from->succeeded_by;
      while(record)
      {
        relevant_changes.emplace_back(*record->change);
        record = record->succeeded_by;
      }
    }
    else
    {
      std::cout << " ++++ we will just insert the trajectory" << std::endl;
      // We are not transmitting the chain of changes that led to this entry,
      // so just create an insertion for it and transmit that.
      relevant_changes.emplace_back(
            Database::Change::Implementation::make_insert_ref(
              &entry->trajectory, entry->version));
    }
  }
  else if(after_version)
  {
    std::cout << " +++ needed is not true, but we have an after version" << std::endl;
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
    std::cout << " +++ the remote mirror already knows about this entry" << std::endl;
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
      std::cout << " --- trajectory detected" << std::endl;
      return rmf_traffic::internal::detect_conflicts(
            e->trajectory, spacetime, nullptr);
    }
    else
    {
      std::cout << " --- NO trajectory detected! " << std::endl;
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
  const internal::EntryPtr& old_entry =
      _pimpl->get_entry_iterator(id, "interruption")->second;

  Trajectory new_trajectory = add_interruption(
        old_entry->trajectory, interruption_trajectory, delay);

  const Version new_version = ++_pimpl->latest_version;
  Change change = Database::Change::make_interrupt(
        id, &interruption_trajectory, delay, new_version);

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
  const internal::EntryPtr& old_entry =
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
  const internal::EntryPtr& old_entry =
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
  const internal::EntryPtr& old_entry =
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

namespace detail {

template class bidirectional_iterator<
    const schedule::Database::Change,
    schedule::Database::Patch::IterImpl,
    schedule::Database::Patch
>;

} // namespace detail

} // namespace rmf_traffic
