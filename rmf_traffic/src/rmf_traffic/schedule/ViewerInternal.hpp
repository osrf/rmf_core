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
using ChangePtr = std::unique_ptr<Change>;
using ConstChangePtr = std::unique_ptr<const Change>;

//==============================================================================
struct Entry : std::enable_shared_from_this<Entry>
{
  // The participant that this entry is related to
  ParticipantId participant;

  // The trajectory for this entry
  Itinerary itinerary;

  // The schedule version number of this entry
  Version schedule_version;

  // The change that led to this entry
  ConstChangePtr change;

  // The entry that this one succeeds
  EntryPtr succeeds;

  // A pointer to an entry that succeeded this entry, if such an entry exists
  Entry* succeeded_by;

  Entry(
      ParticipantId _participant,
      Itinerary _itinerary,
      Version _schedule_version,
      ConstChangePtr _change);

  // Create a new entry for a participant.
  template<typename... Args>
  static EntryPtr make(Args&&... args)
  {
    return std::make_shared<Entry>(std::forward<Args>(args)...);
  }
};

//==============================================================================
/// The most current itineraries for each participant
using Itineraries =
    std::unordered_map<ParticipantId, internal::EntryPtr>;

using ParticipantMap = std::unordered_map<ParticipantId, Participant>;


using Bucket = std::vector<ConstRoutePtr>;
using BucketPtr = std::unique_ptr<Bucket>;

//==============================================================================
// TODO(MXG): A possible performance improvement could be to introduce spatial
// buckets that are orthogonal to the time buckets. This could be added later
// without negatively impacting the API or ABI.

// Each bucket stores trajectories whose time span intersects with the range
// ( key(timeline_it - 1), key(timeline_it) ].
using Timeline = std::map<Time, BucketPtr>;
using MapToTimeline = std::unordered_map<std::string, Timeline>;

//==============================================================================
struct RouteInfo
{
  std::vector<Bucket*> buckets;

  // NOTE(MXG): Keeping track of the "latest" entry makes an assumption that
  // ConstRoutePtr values will never be shared between multiple participants.
  // We enforce this by having the Database::put(~) command make copies of all
  // the route entries that are passed into it so that those routes cannot be
  // shared between participants.
  internal::EntryPtr latest_entry;
};

//==============================================================================
using RouteToInfo =
    std::unordered_map<ConstRoutePtr, RouteInfo>;

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

  Version _oldest = 0;

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
      const ConstRoutePtr& route,
      const RouteInfo& info,
      const rmf_traffic::internal::Spacetime& spacetime) = 0;

  virtual void inspect(
      const ConstRoutePtr& route,
      const RouteInfo& info,
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
      const ConstRoutePtr& route,
      const RouteInfo& info,
      const rmf_traffic::internal::Spacetime& spacetime_region) final;

  void inspect(
      const ConstRoutePtr& route,
      const RouteInfo& info,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;

  VersionRange versions;

  const Version* after_version;

  struct Element
  {
    ParticipantId participant;
    ConstRoutePtr route;
  };

  std::vector<Element> elements;

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
      const ConstRoutePtr& route,
      const RouteInfo& info,
      const std::function<bool(const ConstRoutePtr&)>& relevant);

  void inspect(
      const ConstRoutePtr& route,
      const RouteInfo& info,
      const rmf_traffic::internal::Spacetime& spacetime) final;

  void inspect(
      const ConstRoutePtr& entry,
      const RouteInfo& info,
      const Time* lower_time_bound,
      const Time* upper_time_bound) final;

  VersionRange versions;

  const Version* after_version;

  std::vector<Change> relevant_changes;

  std::unordered_set<ConstChangePtr> checked_changes;
};

} // namespace internal

//==============================================================================
class Viewer::Implementation
{
public:

  internal::Itineraries current_itineraries;
  internal::RouteToInfo routes;
  internal::MapToTimeline timelines;

  internal::ParticipantMap current_participants;
  std::unordered_set<ParticipantId> current_participant_ids;

  Version oldest_version = 0;
  Version latest_version = 0;

  /// Remembers the version number and time value of the last culling that took
  /// place.
  rmf_utils::optional<std::pair<Version, Time>> last_cull;

  static constexpr std::size_t ChangeModeNum =
      static_cast<std::size_t>(Change::Mode::NUM);
  using Changers =
      std::array<std::function<void(const Change&)>, ChangeModeNum>;
  /// Used by the Mirror class to apply changes to its record.
  ///
  /// This field does not get used by the Database class
  Changers changers;

  internal::EntryPtr add_entry(internal::EntryPtr entry);

  /// Used by the Mirror class to erase entries that are no longer needed
  void remove_entry(internal::EntryPtr entry);

  /// Used by the Mirror class to make efficient changes to entries
  void replace_entry(internal::EntryPtr entry);

  internal::Timeline::iterator get_timeline_iterator(
      Timeline& timeline, Time time);

  void cull(Version id, Time time);

  static internal::Timeline::const_iterator get_timeline_end(
      const internal::Timeline& timeline, const Time* upper_time_bound)
  {
    if(upper_time_bound == nullptr)
      return timeline.end();

    auto end = timeline.upper_bound(*upper_time_bound);
    if(end == timeline.end())
      return end;

    return ++end;
  }

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

      const auto timeline_end = get_timeline_end(timeline, upper_time_bound);

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

      const auto timeline_end = get_timeline_end(timeline, upper_time_bound);

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
      case Query::Versions::Mode::Invalid:
      {
        throw std::runtime_error(
            "[rmf_traffic::schedule::Viewer] Invalid Query::Version::Mode "
            "used. Please report this as a bug.");
      }
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
      case Query::Spacetime::Mode::Invalid:
      {
        throw std::runtime_error(
            "[rmf_traffic::schedule::Viewer] Invalid Query::Spacetime::Mode "
            "used. Please report this as a bug.");
      }

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
Trajectory add_delay(
    Trajectory old_trajectory,
    const Time time,
    const Duration delay);

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
