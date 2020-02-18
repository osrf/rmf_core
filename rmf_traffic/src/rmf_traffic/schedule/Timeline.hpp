/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP

#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Query.hpp>

#include <map>
#include <unordered_map>

namespace rmf_traffic {
namespace schedule {

namespace {

// TODO(MXG): Consider allowing these values to be configured

// Each Timeline Bucket spans a range of 1 minute.
const Duration BucketDuration = std::chrono::minutes(1);

// This constant is used during the creation of the first bucket for a timeline.
// It's a very minor optimization that avoids making a bucket that will
// potentially not be very useful.
const Duration PartialBucketDuration = std::chrono::seconds(50);

} // anonymous namespace

//==============================================================================
struct ParticipantFilter
{
  static std::unordered_set<ParticipantId> convert(
      const std::vector<ParticipantId>& ids)
  {
    std::unordered_set<ParticipantId> output;
    for (const auto id : ids)
      output.insert(id);

    return output;
  }

  //============================================================================
  struct AllowAll
  {
    bool ignore(ParticipantId) const
    {
      return false;
    }
  };

  //============================================================================
  struct Include
  {
    Include(const std::vector<ParticipantId>& ids)
    : _ids(convert(ids))
    {
      // Do nothing
    }

    bool ignore(ParticipantId id) const
    {
      return _ids.find(id) == _ids.end();
    }

  private:
    std::unordered_set<ParticipantId> _ids;
  };

  //============================================================================
  struct Exclude
  {
    Exclude(const std::vector<ParticipantId>& ids)
    : _ids(convert(ids))
    {
      // Do nothing
    }

    bool ignore(ParticipantId id) const
    {
      return _ids.find(id) != _ids.end();
    }

  private:
    std::unordered_set<ParticipantId> _ids;
  };
};

//==============================================================================
template<typename Entry>
class TimelineInspector;

//==============================================================================
template<typename Entry>
class Timeline
{
public:

  using Bucket = std::vector<const Entry*>;
  using BucketPtr = std::shared_ptr<Bucket>;
  using Checked =
      std::unordered_map<ParticipantId, std::unordered_set<RouteId>>;

  // TODO(MXG): Come up with a better name for this data structure than Entries
  using Entries = std::map<Time, BucketPtr>;
  using MapNameToEntries = std::unordered_map<std::string, Entries>;

  /// This Timeline::Handle class allows us to use RAII so that when an Entry is
  /// deleted it will automatically be removed from any of its timeline buckets.
  struct Handle
  {
    Handle(
        const Entry* entry,
        std::vector<std::weak_ptr<Bucket>> buckets)
    : _entry(entry),
      _buckets(std::move(buckets))
    {
      // Do nothing
    }

    ~Handle()
    {
      for (const auto& b : _buckets)
      {
        BucketPtr bucket = b.lock();
        if (!bucket)
          continue;

        const auto it = std::find(bucket->begin(), bucket->end(), _entry);
        if (it != bucket->end())
          bucket->erase(it);
      }
    }

  private:
    const Entry* _entry;
    std::vector<std::weak_ptr<Bucket>> _buckets;
  };

  /// Insert a new entry into the timeline
  void insert(Entry& entry)
  {
    std::vector<std::weak_ptr<Bucket>> buckets;
    _all_bucket->push_back(&entry);
    buckets.emplace_back(_all_bucket);

    if (entry.route && entry.route->trajectory().start_time())
    {
      const Time start_time = *entry.route->trajectory().start_time();
      const Time finish_time = *entry.route->trajectory().finish_time();
      const std::string& map_name = entry.route->map();

      const auto map_it = _timelines.insert(
            std::make_pair(map_name, Entries())).first;

      Entries& timeline = map_it->second;

      const auto start_it = get_timeline_iterator(timeline, start_time);
      const auto end_it = ++get_timeline_iterator(timeline, finish_time);

      for (auto it = start_it; it != end_it; ++it)
      {
        it->second->push_back(&entry);
        buckets.emplace_back(it->second);
      }
    }

    entry.timeline_handle = std::make_shared<Handle>(
          &entry, std::move(buckets));
  }

  /// Inspect the timeline for entries that match the query
  template<typename Inspector>
  void inspect(
      const Query& query,
      Inspector& inspector) const
  {
    const Query::Participants& participants = query.participants();
    const Query::Participants::Mode mode = participants.get_mode();

    if (Query::Participants::Mode::All == mode)
    {
      inspect_spacetime(
            query.spacetime(),
            ParticipantFilter::AllowAll(),
            inspector);
    }
    else if (Query::Participants::Mode::Include == mode)
    {
      inspect_spacetime(
            query.spacetime(),
            ParticipantFilter::Include(participants.include()->get_ids()),
            inspector);
    }
    else if (Query::Participants::Mode::Exclude == mode)
    {
      inspect_spacetime(
            query.spacetime(),
            ParticipantFilter::Exclude(participants.exclude()->get_ids()),
            inspector);
    }
    else
    {
      throw std::runtime_error(
            "Unexpected Query::Participants mode: "
            + std::to_string(static_cast<uint16_t>(mode)));
    }
  }

  void cull(const Time time)
  {
    for (auto& pair : _timelines)
    {
      auto& timeline = pair.second;

      // NOTE(MXG): It is not an error that we are using get_timeline_begin() to
      // find the ending iterator. We want to stop just before the first bucket
      // that contains the cull time, because we only want to erase times that
      // come before it.
      const auto end_it = get_timeline_begin(timeline, &time);
      if (end_it != timeline.begin())
        timeline.erase(timeline.begin(), end_it);
    }
  }

private:

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime(
      const Query::Spacetime& spacetime,
      const ParticipantFilter& participant_filter,
      Inspector& inspector) const
  {
    const Query::Spacetime::Mode mode = spacetime.get_mode();

    if (Query::Spacetime::Mode::All == mode)
    {
      inspect_all_spacetime(participant_filter, inspector);
    }
    else if (Query::Spacetime::Mode::Regions == mode)
    {
      inspect_spacetime_regions(
            *spacetime.regions(), participant_filter, inspector);
    }
    else if (Query::Spacetime::Mode::Timespan == mode)
    {
      inspect_spacetime_timespan(
            *spacetime.timespan(), participant_filter, inspector);
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_all_spacetime(
      const ParticipantFilter& participant_filter,
      Inspector& inspector) const
  {
    Checked checked;

    const auto relevant = [](const Entry&) -> bool { return true; };
    for (const auto& entry : *_all_bucket)
    {
      if (participant_filter.ignore(entry->participant))
        continue;

      if (!checked[entry->participant].insert(entry->route_id).second)
        continue;

      inspector.inspect(entry, relevant);
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime_regions(
      const Query::Spacetime::Regions& regions,
      const ParticipantFilter& participant_filter,
      Inspector& inspector) const
  {
    Checked checked;

    rmf_traffic::internal::Spacetime spacetime_data;
    const auto relevant = [&spacetime_data](const Entry& entry) -> bool {
      return rmf_traffic::internal::detect_conflicts(
            entry.description.profile(),
            entry.route->trajectory(),
            spacetime_data);
    };

    for (const Region& region : regions)
    {
      const std::string& map = region.get_map();
      const auto map_it = _timelines.find(map);
      if (map_it == _timelines.end())
        continue;

      const Entries& timeline = map_it->second;
      const Time* const lower_time_bound = region.get_lower_time_bound();
      const Time* const upper_time_bound = region.get_upper_time_bound();

      spacetime_data.lower_time_bound = lower_time_bound;
      spacetime_data.upper_time_bound = upper_time_bound;

      const auto timeline_begin =
          (lower_time_bound == nullptr)?
            timeline.begin() : timeline.lower_bound(*lower_time_bound);

      const auto timeline_end = get_timeline_end(timeline, upper_time_bound);

      for (auto space_it = region.begin(); space_it != region.end(); ++space_it)
      {
        spacetime_data.pose = space_it->get_pose();
        spacetime_data.shape = space_it->get_shape();

        inspect_entries(
              relevant,
              participant_filter,
              inspector,
              timeline_begin,
              timeline_end,
              checked);
      }
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_spacetime_timespan(
      const Query::Spacetime::Timespan& timespan,
      const ParticipantFilter& participant_filter,
      Inspector& inspector) const
  {
    Checked checked;

    const Time* const lower_time_bound = timespan.get_lower_time_bound();
    const Time* const upper_time_bound = timespan.get_upper_time_bound();

    const auto relevant = [&lower_time_bound, &upper_time_bound](
        const Entry& entry) -> bool
    {
      const Trajectory& trajectory = entry.route->trajectory();
      assert(trajectory.start_time());
      if (lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
        return false;

      if (upper_time_bound && *upper_time_bound < *trajectory.start_time())
        return false;

      return true;
    };

    if (timespan.all_maps())
    {
      for (const auto& timeline_it : _timelines)
      {
        const Entries& timeline = timeline_it.second;
        inspect_entries(
              relevant,
              participant_filter,
              inspector,
              get_timeline_begin(timeline, lower_time_bound),
              get_timeline_end(timeline, upper_time_bound),
              checked);
      }
    }
    else
    {
      const auto& maps = timespan.maps();
      for (const std::string& map : maps)
      {
        const auto map_it = _timelines.find(map);
        if (map_it == _timelines.end())
          continue;

        const Entries& timeline = map_it->second;
        inspect_entries(
              relevant,
              participant_filter,
              inspector,
              get_timeline_begin(timeline, lower_time_bound),
              get_timeline_end(timeline, upper_time_bound),
              checked);
      }
    }
  }

  template<typename Inspector, typename ParticipantFilter>
  void inspect_entries(
      const std::function<bool(const Entry&)>& relevant,
      const ParticipantFilter& participant_filter,
      Inspector& inspector,
      const typename Entries::const_iterator& timeline_begin,
      const typename Entries::const_iterator& timeline_end,
      Checked& checked) const
  {
    auto timeline_it = timeline_begin;
    for (; timeline_it != timeline_end; ++timeline_it)
    {
      const Bucket& bucket = *timeline_it->second;

      auto entry_it = bucket.begin();
      for (; entry_it != bucket.begin(); ++entry_it)
      {
        const Entry* entry = *entry_it;

        if (participant_filter.ignore(entry->participant))
          continue;

        if (!checked[entry->participant].insert(entry->route_id).second)
          continue;

        inspector.inspect(entry, relevant);
      }
    }
  }

  //============================================================================
  static typename Entries::iterator get_timeline_iterator(
      Entries& timeline, const Time time)
  {
    auto start_it = timeline.lower_bound(time);

    if(start_it == timeline.end())
    {
      if(timeline.empty())
      {
        // This timeline is completely empty, so we'll begin creating buckets
        // starting from the time of this trajectory.
        return timeline.insert(
              timeline.end(),
              std::make_pair(
                time + PartialBucketDuration,
                std::make_shared<Bucket>()));
      }

      auto last_it = --timeline.end();
      while(last_it->first < time)
      {
        last_it = timeline.insert(
              timeline.end(),
              std::make_pair(
                last_it->first + BucketDuration,
                std::make_shared<Bucket>()));
      }

      return last_it;
    }

    while(time + BucketDuration < start_it->first)
    {
      start_it = timeline.insert(
            start_it,
            std::make_pair(
              start_it->first - BucketDuration,
              std::make_shared<Bucket>()));
    }

    return start_it;
  }

  static typename Entries::const_iterator get_timeline_begin(
      const Entries& timeline,
      const Time* const lower_time_bound)
  {
    return (lower_time_bound == nullptr)?
          timeline.begin() : timeline.lower_bound(*lower_time_bound);
  }

  static typename Entries::const_iterator get_timeline_end(
      const Entries& timeline,
      const Time* upper_time_bound)
  {
    if(upper_time_bound == nullptr)
      return timeline.end();

    auto end = timeline.upper_bound(*upper_time_bound);
    if(end == timeline.end())
      return end;

    return ++end;
  }

  MapNameToEntries _timelines;
  BucketPtr _all_bucket;

};

//==============================================================================
template<typename Entry>
class TimelineInspector
{
public:

  virtual void inspect(
      const Entry* entry,
      const std::function<bool(const Entry& entry)>& relevant) = 0;

  virtual ~TimelineInspector() = default;
};


} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__TIMELINE_HPP
