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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP

#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <map>
#include <unordered_map>
#include <unordered_set>

namespace rmf_traffic {
namespace schedule {

namespace internal {

struct Entry;
using EntryPtr = std::shared_ptr<Entry>;

using ChangePtr = std::shared_ptr<Database::Change>;

struct Entry
{
  // The change that led to this entry
  ChangePtr change;

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
      const rmf_traffic::internal::Spacetime& spacetime) = 0;

  virtual void inspect(
      const EntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) = 0;

};

//==============================================================================
/// This class inspects for whether an entry is relevant for a View::query()
class ViewRelevanceInspector : public RelevanceInspector
{
public:

  ViewRelevanceInspector(
      const std::size_t* after_version,
      const std::size_t reserve_size);

  const std::size_t* after_version;

  std::vector<EntryPtr> relevant_entries;

  void inspect(
      const EntryPtr& entry,
      const rmf_traffic::internal::Spacetime& spacetime_region) final;

  void inspect(
      const EntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;
};

//==============================================================================
/// This class inspects for whether an entry is relevant for a
/// Database::changes() request
class ChangeRelevanceInspector : public RelevanceInspector
{
public:

  const std::size_t* after_version;

  std::vector<EntryPtr> relevant_entries;

  void inspect(
      const EntryPtr& entry,
      const rmf_traffic::internal::Spacetime& spacetime) final;

  void inspect(
      const EntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;

};

} // namespace internal

//==============================================================================
class Viewer::Implementation
{
public:

  using Bucket = std::vector<internal::EntryPtr>;

  // TODO(MXG): A possible performance improvement could be to introduce spatial
  // buckets that are orthogonal to the time buckets. This could be added later
  // without negatively impacting the API or ABI.
  using Timeline = std::map<Time, Bucket>;
  using MapToTimeline = std::unordered_map<std::string, Timeline>;

  using EntryMap = std::map<std::size_t, internal::EntryPtr>;

  MapToTimeline timelines;
  std::vector<internal::EntryPtr> all_entries;

  /// A map from the version number of each culling to the culls that took place
  std::map<std::size_t, std::vector<std::size_t>> culls;

  template<typename RelevanceInspectorT>
  void inspect_spacetime_region_entries(
      const Query::Spacetime::Regions& regions,
      RelevanceInspectorT& inspector) const
  {
    std::unordered_set<std::size_t> checked;
    checked.reserve(all_entries.size());

    for(const Region& region : regions)
    {
      const std::string& map = region.get_map();
      const auto map_it = timelines.find(map);
      if(map_it == timelines.end())
        continue;

      const Timeline& timeline = map_it->second;
      const Time* const lower_time_bound = region.get_lower_time_bound();
      const Time* const upper_time_bound = region.get_upper_time_bound();

      const auto timeline_begin =
          (lower_time_bound == nullptr)?
            timeline.begin() : timeline.lower_bound(*lower_time_bound);

      const auto timeline_end =
          (upper_time_bound == nullptr)?
            timeline.end() : timeline.upper_bound(*upper_time_bound);

      rmf_traffic::internal::Spacetime spacetime_data;
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
            const internal::EntryPtr& entry_ptr = *entry_it;
            // Test if we have already checked this entry
            if(!checked.insert(entry_ptr->version).second)
              continue;

            inspector.inspect(entry_ptr, spacetime_data);
          }
        }
      }
    }
  }

  template<typename RelevanceInspectorT>
  void inspect_timespan(
      const std::unordered_set<std::string>& maps,
      const Time* lower_time_bound,
      const Time* upper_time_bound,
      RelevanceInspectorT& inspector) const
  {
    std::unordered_set<std::size_t> checked;
    checked.reserve(all_entries.size());

    for(const std::string& map : maps)
    {
      const auto map_it = timelines.find(map);
      if(map_it == timelines.end())
        continue;

      const Timeline& timeline = map_it->second;

      const auto timeline_begin =
          (lower_time_bound == nullptr)?
            timeline.begin() : timeline.lower_bound(*lower_time_bound);

      const auto timeline_end =
          (upper_time_bound == nullptr)?
            timeline.end() : timeline.upper_bound(*upper_time_bound);

      auto timeline_it = timeline_begin;
      for(; timeline_it != timeline_end; ++timeline_it)
      {
        const Bucket& bucket = timeline_it->second;

        auto entry_it = bucket.begin();
        for(; entry_it != bucket.end(); ++entry_it)
        {
          const internal::EntryPtr& entry_ptr = *entry_it;
          if(!checked.insert(entry_ptr->version).second)
            continue;

          inspector.inspect(entry_ptr, lower_time_bound, upper_time_bound);
        }
      }
    }
  }

};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
