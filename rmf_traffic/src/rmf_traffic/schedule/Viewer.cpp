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
#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

namespace {

// Each Timeline Bucket spans a range of 1 minute.
const Duration BucketDuration = std::chrono::minutes(1);

// This constant is used during the creation of the first bucket for a timeline.
// It's a very minor optimization that avoids making a bucket that will
// potentially not be very useful.
const Duration PartialBucketDuration = std::chrono::seconds(50);

} // anonymous namespace

namespace internal {

//==============================================================================
Entry::Entry(Trajectory _trajectory,
    Version _version,
    ConstEntryPtr _succeeds,
    ChangePtr _change)
  : trajectory(std::move(_trajectory)),
    version(_version),
    succeeds(std::move(_succeeds)),
    change(std::move(_change))
{
  // Do nothing
}

//==============================================================================
VersionRange::VersionRange(const Version oldest)
  : _oldest(oldest)
{
  // Do nothing
}

//==============================================================================
bool VersionRange::less(const Version lhs, const Version rhs) const
{
  // This modular arithmetic should guarantee that even if version numbers
  // overflow after the schedule has been operating for a very long time, we
  // will still compare versions correctly.

  // TODO(MXG): Make sure tests get written to verify the behavior for this is
  // correct.
  return (lhs - _oldest) < (rhs - _oldest);
}

//==============================================================================
bool VersionRange::less_or_equal(const Version lhs, const Version rhs) const
{
  return (lhs == rhs) || less(lhs, rhs);
}

//==============================================================================
void ViewRelevanceInspector::version_range(VersionRange _range)
{
  versions = std::move(_range);
}

//==============================================================================
void ViewRelevanceInspector::after(const Version* _after)
{
  after_version = _after;
}

//==============================================================================
void ViewRelevanceInspector::reserve(const std::size_t size)
{
  trajectories.reserve(size);
}

//==============================================================================
void ViewRelevanceInspector::inspect(
    const ConstEntryPtr& entry,
    const rmf_traffic::internal::Spacetime& spacetime_region)
{
  if(entry->succeeded_by)
    return;

  if(after_version && versions.less_or_equal(entry->version, *after_version))
    return;

  if(rmf_traffic::internal::detect_conflicts(
       entry->trajectory, spacetime_region, nullptr))
    trajectories.push_back(&entry->trajectory);
}

//==============================================================================
void ViewRelevanceInspector::inspect(
    const ConstEntryPtr& entry,
    const Time* lower_time_bound,
    const Time* upper_time_bound)
{
  if(entry->succeeded_by)
    return;

  if(after_version && versions.less_or_equal(entry->version, *after_version))
    return;

  const Trajectory& trajectory = entry->trajectory;
  assert(trajectory.start_time() != nullptr);

  if(lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
    return;

  if(upper_time_bound && *upper_time_bound < *trajectory.start_time())
    return;

  trajectories.push_back(&entry->trajectory);
}

} // namespace internal

//==============================================================================
internal::EntryPtr Viewer::Implementation::add_entry(internal::EntryPtr entry)
{
  all_entries.insert(std::make_pair(entry->version, entry));

  const Trajectory& trajectory = entry->trajectory;
  assert(trajectory.start_time());
  const Time start_time = *trajectory.start_time();
  const Time finish_time = *trajectory.finish_time();

  const MapToTimeline::iterator map_it = timelines.insert(
        std::make_pair(entry->trajectory.get_map_name(), Timeline())).first;

  Timeline& timeline = map_it->second;

  const Timeline::iterator start_it =
      get_timeline_iterator(timeline, start_time);
  const Timeline::iterator finish_it =
      get_timeline_iterator(timeline, finish_time);

  const Timeline::const_iterator end_it = ++Timeline::iterator(finish_it);

  for(auto it = start_it; it != end_it; ++it)
  {
    it->second.push_back(entry);
  }

  return entry;
}

//==============================================================================
void Viewer::Implementation::modify_entry(
    const internal::EntryPtr& entry,
    Trajectory new_trajectory,
    const Version new_id)
{
  entry->version = new_id;

  // TODO(MXG): It should be posssible to improve performance for entry
  // modifications by applying the change directly to the original Trajectory
  // object instead of making a copy.

  Timeline& timeline = timelines.at(entry->trajectory.get_map_name());

  const Time old_start = *entry->trajectory.start_time();
  const Time old_end = *entry->trajectory.finish_time();
  // TODO(MXG): A micro-optimization might be to store these iterators in the
  // entry data so we don't need to look them up again.
  const Timeline::iterator old_start_it = timeline.lower_bound(old_start);
  const Timeline::iterator old_end_it = timeline.lower_bound(old_end);

  const Time new_start = *new_trajectory.start_time();
  const Time new_end = *new_trajectory.finish_time();
  const Timeline::iterator new_start_it =
      get_timeline_iterator(timeline, new_start);
  const Timeline::iterator new_end_it =
      get_timeline_iterator(timeline, new_end);

  entry->trajectory = std::move(new_trajectory);

  // Fix the bucketing for this entry
  if(old_end_it->first < new_start_it->first
     || new_end_it->first < old_start_it->first)
  {
    for(auto it = new_start_it; it != ++Timeline::iterator(new_end_it); ++it)
    {
      Bucket& bucket = it->second;
      bucket.push_back(entry);
    }

    for(auto it = old_start_it; it != ++Timeline::iterator(old_end_it); ++it)
    {
      Bucket& bucket = it->second;
      bucket.erase(std::remove(bucket.begin(), bucket.end(), entry),
                   bucket.end());
    }
  }
  else
  {
    if(old_start_it->first < new_start_it->first)
    {
      for(auto it = old_start_it; it != new_start_it; ++it)
      {
        Bucket& bucket = it->second;
        bucket.erase(std::remove(bucket.begin(), bucket.end(), entry),
                     bucket.end());
      }
    }
    else
    {
      for(auto it = new_start_it; it != old_start_it; ++it)
      {
        Bucket& bucket = it->second;
        bucket.push_back(entry);
      }
    }

    if(new_end_it->first < old_end_it->first)
    {
      const auto begin_erase = ++Timeline::iterator(new_end_it);
      const auto end_erase = ++Timeline::iterator(old_end_it);
      for(auto it = begin_erase; it != end_erase; ++it)
      {
        Bucket& bucket = it->second;
        bucket.erase(std::remove(bucket.begin(), bucket.end(), entry),
                     bucket.end());
      }
    }
    else
    {
      const auto begin_insert = ++Timeline::iterator(old_end_it);
      const auto end_insert = ++Timeline::iterator(new_end_it);
      for(auto it = begin_insert; it != end_insert; ++it)
      {
        Bucket& bucket = it->second;
        bucket.push_back(entry);
      }
    }
  }
}

//==============================================================================
void Viewer::Implementation::erase_entry(Version id)
{
  const internal::EntryPtr& entry = get_entry_iterator(id, "erasure")->second;

  Timeline& timeline = timelines.at(entry->trajectory.get_map_name());

  const Time old_start = *entry->trajectory.start_time();
  const Time old_end = *entry->trajectory.finish_time();
  // TODO(MXG): A micro-optimization might be to store these iterators in the
  // entry data so we don't need to look them up again.
  const Timeline::iterator begin_it = timeline.lower_bound(old_start);
  const Timeline::iterator end_it = ++timeline.lower_bound(old_end);
  for(auto it = begin_it; it != end_it; ++it)
  {
    Bucket& bucket = it->second;
    bucket.erase(std::remove(bucket.begin(), bucket.end(), entry),
                 bucket.end());
  }

  all_entries.erase(id);
}

//==============================================================================
auto Viewer::Implementation::get_timeline_iterator(
    Timeline& timeline, const Time time) -> Timeline::iterator
{
  Timeline::iterator start_it = timeline.lower_bound(time);

  if(start_it == timeline.end())
  {
    if(timeline.empty())
    {
      // This timeline is completely empty, so we'll begin creating buckets
      // starting from the time of this trajectory.
      return timeline.insert(
            timeline.end(),
            std::make_pair(time + PartialBucketDuration, Bucket()));
    }

    Timeline::iterator last_it = --timeline.end();
    while(last_it->first < time)
    {
      last_it = timeline.insert(
            timeline.end(),
            std::make_pair(last_it->first + BucketDuration, Bucket()));
    }

    return last_it;
  }

  while(time + BucketDuration < start_it->first)
  {
    start_it = timeline.insert(
          start_it,
          std::make_pair(start_it->first - BucketDuration, Bucket()));
  }

  return start_it;
}

namespace {
//==============================================================================
std::string throw_missing_id_error(
    const std::string& operation,
    const Version id,
    const Version oldest,
    const Version latest)
{
  const std::string message = std::string()
      + "Requested " + operation + " for ID that does not exist in this "
        "Database: " + std::to_string(id) + ". The oldest known id is ["
      + std::to_string(oldest) + "] and the latest is ["
      + std::to_string(latest) + "], but note that IDs inside that range can "
        "still be invalid if they have been culled or erased.";
  throw std::runtime_error(message);
}
} // anonymous namespace

//==============================================================================
auto Viewer::Implementation::get_entry_iterator(
    const Version id,
    const std::string& operation) -> EntryMap::iterator
{
  const auto old_entry_it = all_entries.find(id);
  if(old_entry_it == all_entries.end())
  {
    throw_missing_id_error(
          operation, id, oldest_version, latest_version);
  }

  return old_entry_it;
}

//==============================================================================
void Viewer::Implementation::cull(Version id, Time time)
{
  cull_has_occurred = true;
  last_cull = std::make_pair(id, time);

  std::unordered_set<Version> culled;
  for(auto& pair : timelines)
  {
    Timeline& timeline = pair.second;
    const Timeline::iterator last_it = timeline.lower_bound(time);
    const Timeline::iterator end_it = last_it == timeline.end()?
          timeline.end() : ++Timeline::iterator(last_it);

    for(Timeline::iterator it = timeline.begin(); it != end_it; ++it)
    {
      Bucket& bucket = it->second;
      const Bucket::iterator removed =
          std::remove_if(bucket.begin(), bucket.end(),
                     [&](const internal::ConstEntryPtr& entry) -> bool
      {
        return *entry->trajectory.finish_time() < time;
      });

      for(Bucket::iterator bit = removed; bit != bucket.end(); ++bit)
        culled.insert((*bit)->version);

      bucket.erase(removed, bucket.end());
    }

    Timeline::iterator stop_erasing = timeline.begin();
    for(Timeline::iterator it = timeline.begin(); it != end_it; ++it)
    {
      const Bucket& bucket = it->second;
      if(!bucket.empty())
      {
        // If this bucket is not empty, then stop the erasing now
        break;
      }

      // If this bucket is empty, then stop the erasing at the next entry
      stop_erasing = ++Timeline::iterator(it);
    }

    timeline.erase(timeline.begin(), stop_erasing);
  }

  for(const Version v : culled)
    all_entries.erase(v);

  if(!all_entries.empty())
    oldest_version = all_entries.begin()->first;
}

//==============================================================================
Trajectory add_interruption(
    // Note: This argument is intentionally named differently here than the name
    // given to it in the declaration, because we want the user to pass in the
    // old trajectory, but we will now be turning it into the new trajectory.
    Trajectory new_trajectory,
    const Trajectory& interruption_trajectory,
    const Duration delay)
{
  assert(interruption_trajectory.start_time());
  const Time interrupt_start_time = *interruption_trajectory.start_time();
  const Time interrupt_finish_time = *interruption_trajectory.finish_time();
  const Duration total_delay =
      interrupt_finish_time - interrupt_start_time + delay;

  Trajectory::iterator delayed_segment =
      new_trajectory.find(*interruption_trajectory.start_time());

  if(delayed_segment != new_trajectory.end())
    delayed_segment->adjust_finish_times(total_delay);

  for(const Trajectory::Segment& interrupt_segment : interruption_trajectory)
  {
    new_trajectory.insert(
          interrupt_segment.get_finish_time(),
          interrupt_segment.get_profile(),
          interrupt_segment.get_finish_position(),
          interrupt_segment.get_finish_velocity());
  }

  return new_trajectory;
}

//==============================================================================
Trajectory add_delay(
    Trajectory new_trajectory,
    const Time time,
    const Duration delay)
{
  Trajectory::iterator delayed_segment = new_trajectory.find(time);
  assert(delayed_segment != new_trajectory.end());
  if(delayed_segment != new_trajectory.end())
    delayed_segment->adjust_finish_times(delay);

  return new_trajectory;
}

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

  internal::DeepIterator iter;

};

//==============================================================================
auto Viewer::View::begin() const -> const_iterator
{
  return const_iterator{IterImpl{
      internal::DeepIterator{_pimpl->entries.begin()}}};
}

//==============================================================================
auto Viewer::View::end() const -> const_iterator
{
  return const_iterator{IterImpl{
      internal::DeepIterator{_pimpl->entries.end()}}};
}

//==============================================================================
std::size_t Viewer::View::size() const
{
  return _pimpl->entries.size();
}

//==============================================================================
Viewer::View Viewer::query(const Query& parameters) const
{
  return View::Implementation::make_view(
        std::move(_pimpl->inspect<internal::ViewRelevanceInspector>(
                    parameters).trajectories));
}

//==============================================================================
Version Viewer::oldest_version() const
{
  return _pimpl->oldest_version;
}

//==============================================================================
Version Viewer::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
Viewer::Viewer()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
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
