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

#include "../detail/internal_bidirectional_iterator.hpp"
#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <map>
#include <unordered_map>
#include <unordered_set>

namespace rmf_traffic {
namespace schedule {

namespace {

struct Entry;
using EntryPtr = std::shared_ptr<Entry>;

struct Entry
{
  // The change that led to this entry
  Database::Change change;

  // The trajectory for this entry
  Trajectory trajectory;

  // The version number of this entry
  std::size_t version;

  // Succeeds
  EntryPtr succeeds;

  // A version that succeeded this entry, if such a version exists
  EntryPtr succeeded_by;
};

struct DeepIterator
{
  std::vector<const Trajectory*>::const_iterator it;

  const Trajectory& operator*()
  {
    return **it;
  }

  const Trajectory& operator*() const
  {
    return **it;
  }

  const Trajectory* operator->()
  {
    return *it;
  }

  const Trajectory* operator->() const
  {
    return *it;
  }

  DeepIterator& operator++()
  {
    ++it;
    return *this;
  }

  DeepIterator& operator--()
  {
    --it;
    return *this;
  }

  DeepIterator operator++(int)
  {
    DeepIterator original = *this;
    ++it;
    return original;
  }

  DeepIterator operator--(int)
  {
    DeepIterator original = *this;
    --it;
    return original;
  }

  bool operator==(const DeepIterator& other) const
  {
    return it == other.it;
  }

  bool operator!=(const DeepIterator& other) const
  {
    return it != other.it;
  }
};

//==============================================================================
/// Pure abstract interface class for the
/// Viewer::Implementation::inspect_spacetime_region_entries utility
class RelevanceInspector
{
public:

  virtual void inspect(
      const EntryPtr& entry,
      const internal::Spacetime& spacetime) = 0;

};

//==============================================================================
/// This class inspects for whether an entry is relevant for a View::query()
class ViewRelevanceInspector : public RelevanceInspector
{
public:

  ViewRelevanceInspector(
      const std::size_t* after_version,
      const std::size_t reserve_size)
    : after_version(after_version)
  {
    relevant_entries.reserve(reserve_size);
  }

  const std::size_t* after_version;

  std::vector<EntryPtr> relevant_entries;

  void inspect(
      const EntryPtr& entry,
      const internal::Spacetime& spacetime_region) override final
  {
    if(entry->succeeded_by)
      return;

    if(after_version && entry->version < *after_version)
      return;

    if(internal::detect_conflicts(entry->trajectory, spacetime_region, nullptr))
      relevant_entries.push_back(entry);
  }

};

//==============================================================================
/// This class inspects for whether an entry is relevant for a
/// Database::changes() request
class ChangeRelevanceInspector : public RelevanceInspector
{
public:

  // TODO(MXG): Implement this class

};

} // anonymous namespace

//==============================================================================
class Viewer::Implementation
{
public:

  using Bucket = std::vector<EntryPtr>;

  // TODO(MXG): A possible performance improvement could be to introduce spatial
  // buckets that are orthogonal to the time buckets. This could be added later
  // without negatively impacting the API or ABI.
  using Timeline = std::map<Time, Bucket>;
  using MapToTimeline = std::unordered_map<std::string, Timeline>;

  MapToTimeline maps;

  using EntryMap = std::map<std::size_t, EntryPtr>;

  struct ChangeData
  {
    Database::Change::Mode mode;

    // Used in all modes
    std::size_t id;

    // Used in Insert and Replace mode
    EntryPtr entry;

    // Used in Replace and Erase mode
    std::size_t original_id;

    // Used in Cull mode
    std::vector<std::size_t> culled_ids;
  };

  using ChangeMap = std::map<std::size_t, ChangeData>;

  std::vector<EntryPtr> all_entries;

  template<typename RelevanceInspectorT>
  void inspect_spacetime_region_entries(
      const Query::Spacetime::Regions& regions,
      RelevanceInspectorT& inspector) const
  {
    std::unordered_set<std::size_t> checked;
    checked.reserve(all_entries.size());

    for(const Query::Spacetime::Region& region : regions)
    {
      const std::string& map = region.get_map();
      const auto map_it = maps.find(map);
      if(map_it == maps.end())
        continue;

      const Timeline timeline = map_it->second;
      const Time* const lower_time_bound = region.get_lower_time_bound();
      const Time* const upper_time_bound = region.get_upper_time_bound();

      const auto timeline_begin =
          (lower_time_bound == nullptr)?
            timeline.begin() : timeline.lower_bound(*lower_time_bound);

      const auto timeline_end =
          (upper_time_bound == nullptr)?
            timeline.end() : timeline.upper_bound(*upper_time_bound);

      internal::Spacetime spacetime_data;
      spacetime_data.lower_time_bound = lower_time_bound;
      spacetime_data.upper_time_bound = upper_time_bound;
      for(auto space_it=region.begin(); space_it != region.end(); ++space_it)
      {
        spacetime_data.pose = space_it->get_pose();
        spacetime_data.shape = space_it->get_shape();

        auto timeline_it = timeline_begin;
        for(; timeline_it != timeline_end; ++timeline_it)
        {
          const Bucket& bucket = timeline_it->second;

          auto entry_it = bucket.begin();
          for(; entry_it != bucket.end(); ++entry_it)
          {
            const EntryPtr& entry_ptr = *entry_it;
            // Test if we have already checked this entry
            if(!checked.insert(entry_ptr->version).second)
              continue;

            inspector.inspect(entry_ptr, spacetime_data);
          }
        }
      }
    }
  }

};

//==============================================================================
class Viewer::View::Implementation
{
public:

  std::vector<const Trajectory*> entries;

  static View make_view(std::vector<const Trajectory*> entries)
  {
    View view;
    view._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::move(entries)});
    return view;
  }
};

//==============================================================================
class Viewer::View::IterImpl
{
public:

  DeepIterator iter;

};

//==============================================================================
auto Viewer::View::begin() const -> const_iterator
{
  return const_iterator{IterImpl{DeepIterator{_pimpl->entries.begin()}}};
}

//==============================================================================
auto Viewer::View::end() const -> const_iterator
{
  return const_iterator{IterImpl{DeepIterator{_pimpl->entries.end()}}};
}

//==============================================================================
std::size_t Viewer::View::size() const
{
  return _pimpl->entries.size();
}

//==============================================================================
Viewer::View Viewer::query(Query parameters) const
{
  const Query::Spacetime& spacetime = parameters.spacetime();
  const Query::Spacetime::Mode spacetime_mode = spacetime.get_mode();

  const Query::Versions& versions = parameters.versions();
  const Query::Versions::Mode versions_mode = versions.get_mode();

  std::vector<EntryPtr> qualified_entries;

  std::size_t after_version;
  const std::size_t* after_version_ptr = nullptr;
  switch(versions_mode)
  {
    case Query::Versions::Mode::All:
    {
      // Do nothing
      break;
    }

    case Query::Versions::Mode::After:
    {
      assert(versions.after() != nullptr);
      after_version = versions.after()->get_version();
      after_version_ptr = &after_version;
      break;
    }
  }

  // We use a switch here so that we'll get a compiler warning if a new
  // Spacetime::Mode type is ever added and we forget to handle it.
  switch(spacetime_mode)
  {
    case Query::Spacetime::Mode::All:
    {
      qualified_entries = _pimpl->all_entries;
      if(after_version_ptr)
      {
        const auto removed = std::remove_if(
              qualified_entries.begin(), qualified_entries.end(),
              [&](const EntryPtr& entry){
                return entry->version <= after_version;
              });
        qualified_entries.erase(removed, qualified_entries.end());
      }
      break;
    }

    case Query::Spacetime::Mode::Regions:
    {
      assert(spacetime.regions() != nullptr);

      ViewRelevanceInspector inspector{
        after_version_ptr, _pimpl->all_entries.size()};
      _pimpl->inspect_spacetime_region_entries(*spacetime.regions(), inspector);
      qualified_entries = inspector.relevant_entries;
      break;
    }
  }

  std::vector<const Trajectory*> trajectories;
  trajectories.reserve(qualified_entries.size());
  for(const auto& q : qualified_entries)
  {
    if(!q->succeeded_by)
      trajectories.push_back(&q->trajectory);
  }

  return View::Implementation::make_view(std::move(trajectories));
}

} // namespace schedule

namespace detail {

template class bidirectional_iterator<
    const Trajectory,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic
