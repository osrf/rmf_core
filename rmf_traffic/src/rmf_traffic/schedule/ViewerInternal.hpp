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
using ConstEntryPtr = std::shared_ptr<const Entry>;
using ChangePtr = std::unique_ptr<Database::Change>;
using ConstChangePtr = std::unique_ptr<const Database::Change>;

//==============================================================================
struct Entry
{
  // The trajectory for this entry
  Trajectory trajectory;

  // The version number of this entry
  Version version;

  // Succeeds
  ConstEntryPtr succeeds;

  // A version that succeeded this entry, if such a version exists
  ConstEntryPtr succeeded_by;

  // The change that led to this entry
  ConstChangePtr change;

  // Initialize this entry
  Entry(
      Trajectory _trajectory,
      Version _version,
      ConstEntryPtr _succeeds = nullptr,
      ChangePtr _change = nullptr);
};

//==============================================================================
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
/// This class allows us to correctly handle version number overflow. Since the
/// schedule needs to continue running for an arbitrarily long time, we cannot
/// expect its versions numbers to get reset before it reaches the limit of
/// Version. This class allows us to compare version numbers that could have
/// overflowed at some point. As long as database entries are getting culled
/// before all version numbers are taken up, this should be guaranteed to handle
/// the eventual integer overflow correctly.
class VersionRange
{
public:

  VersionRange() = default;

  VersionRange(Version oldest);

  bool less(Version lhs, Version rhs) const;

  bool less_or_equal(Version lhs, Version rhs) const;

private:

  Version _oldest;

};

//==============================================================================
/// Pure abstract interface class for the
/// Viewer::Implementation::inspect_spacetime_region_entries utility
class RelevanceInspector
{
public:
  virtual void version_range(VersionRange range) = 0;
  virtual void after(const Version* after) = 0;
  virtual void reserve(Version size) = 0;

  virtual void inspect(
      const ConstEntryPtr& entry,
      const rmf_traffic::internal::Spacetime& spacetime) = 0;

  virtual void inspect(
      const ConstEntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) = 0;

  virtual ~RelevanceInspector() = default;
};

//==============================================================================
/// This class inspects for whether an entry is relevant for a View::query()
class ViewRelevanceInspector : public RelevanceInspector
{
public:

  void version_range(VersionRange range) final;

  void after(const Version* _after) final;

  void reserve(std::size_t size) final;

  void inspect(
      const ConstEntryPtr& entry,
      const rmf_traffic::internal::Spacetime& spacetime_region) final;

  void inspect(
      const ConstEntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;

  VersionRange versions;

  const Version* after_version;

  std::vector<const Trajectory*> trajectories;

};

//==============================================================================
/// This class inspects for whether an entry is relevant for a
/// Database::changes() request
class ChangeRelevanceInspector : public RelevanceInspector
{
public:

  void version_range(VersionRange range) final;

  void after(const Version* _after) final;

  void reserve(Version size) final;

  void inspect(
      const ConstEntryPtr& entry,
      const std::function<bool(const ConstEntryPtr&)>& relevant);

  void inspect(
      const ConstEntryPtr& entry,
      const rmf_traffic::internal::Spacetime& spacetime) final;

  void inspect(
      const ConstEntryPtr& entry,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;

  VersionRange versions;

  const Version* after_version;

  std::vector<Database::Change> relevant_changes;
};

} // namespace internal

//==============================================================================
class Viewer::Implementation
{
public:

  using Bucket = std::vector<internal::ConstEntryPtr>;

  // TODO(MXG): A possible performance improvement could be to introduce spatial
  // buckets that are orthogonal to the time buckets. This could be added later
  // without negatively impacting the API or ABI.

  //
  using Timeline = std::map<Time, Bucket>;
  using MapToTimeline = std::unordered_map<std::string, Timeline>;


  MapToTimeline timelines;

  // TODO(MXG): Consider using a sorted vector here instead of a std::map.
  using EntryMap = std::map<Version, internal::EntryPtr>;
  EntryMap all_entries;

  Version oldest_version = 0;
  Version latest_version = 0;

  /// Remembers the version number and time value of the last culling that took
  /// place.
  bool cull_has_occurred = false;
  std::pair<Version, Time> last_cull;

  static constexpr std::size_t ChangeModeNum =
      static_cast<std::size_t>(Database::Change::Mode::NUM);
  using Changers =
      std::array<std::function<void(const Database::Change&)>, ChangeModeNum>;
  /// Used by the Mirror class to apply changes to its record.
  ///
  /// This field does not get used by the Database class
  Changers changers;

  internal::EntryPtr add_entry(internal::EntryPtr entry, bool erasure = false);

  /// Used by the Mirror class to make efficient changes to entries
  void modify_entry(const internal::EntryPtr& entry,
      Trajectory new_trajectory, const Version new_id);

  /// Used by the Mirror class to erase entries that are no longer needed
  void erase_entry(Version id);

  Timeline::iterator get_timeline_iterator(
      Timeline& timeline, Time time);

  EntryMap::iterator get_entry_iterator(
      Version id,
      const std::string& operation);

  void cull(Version id, Time time);

  template<typename RelevanceInspectorT>
  void inspect_spacetime_region(
      const Query::Spacetime::Regions& regions,
      RelevanceInspectorT& inspector) const
  {
    std::unordered_set<Version> checked;
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

      const auto timeline_end = [&]()
      {
        if(upper_time_bound == nullptr)
          return timeline.end();

        auto end = timeline.upper_bound(*upper_time_bound);
        if(end == timeline.end())
          return end;

        return ++end;
      }();

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
            const internal::ConstEntryPtr& entry_ptr = *entry_it;
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
    std::unordered_set<Version> checked;
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
          const internal::ConstEntryPtr& entry_ptr = *entry_it;
          if(!checked.insert(entry_ptr->version).second)
            continue;

          inspector.inspect(entry_ptr, lower_time_bound, upper_time_bound);
        }
      }
    }
  }

  template<typename RelevanceInspectorT>
  void inspect_all(RelevanceInspectorT& inspector) const
  {
    for(const auto pair : all_entries)
    {
      const internal::ConstEntryPtr& entry_ptr = pair.second;
      inspector.inspect(entry_ptr, nullptr, nullptr);
    }
  }

  template<typename RelevanceInspectorT>
  RelevanceInspectorT inspect(const Query& parameters) const
  {
    const Query::Spacetime& spacetime = parameters.spacetime();
    const Query::Spacetime::Mode spacetime_mode = spacetime.get_mode();

    const Query::Versions& versions = parameters.versions();
    const Query::Versions::Mode versions_mode = versions.get_mode();


    Version after_version;
    const Version* after_version_ptr = nullptr;
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

    RelevanceInspectorT inspector;
    inspector.after(after_version_ptr);
    inspector.reserve(all_entries.size());

    // We use a switch here so that we'll get a compiler warning if a new
    // Spacetime::Mode type is ever added and we forget to handle it.
    switch(spacetime_mode)
    {
      case Query::Spacetime::Mode::All:
      {
        inspect_all(inspector);
        break;
      }

      case Query::Spacetime::Mode::Regions:
      {
        assert(spacetime.regions() != nullptr);
        inspect_spacetime_region(*spacetime.regions(), inspector);
        break;
      }

      case Query::Spacetime::Mode::Timespan:
      {
        assert(spacetime.timespan() != nullptr);
        const Query::Spacetime::Timespan& timespan = *spacetime.timespan();

        inspect_timespan(
              timespan.get_maps(),
              timespan.get_lower_time_bound(),
              timespan.get_upper_time_bound(),
              inspector);
        break;
      }
    }

    return inspector;
  }

};

//==============================================================================
Trajectory add_interruption(
    Trajectory old_trajectory,
    const Trajectory& interruption_trajectory,
    const Duration delay);

//==============================================================================
Trajectory add_delay(
    Trajectory old_trajectory,
    const Time time,
    const Duration delay);

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
