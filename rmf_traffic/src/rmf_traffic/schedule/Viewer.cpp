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
#include "debug_Viewer.hpp"

#include <algorithm>

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

namespace {

//==============================================================================
Viewer::Implementation::Timeline::iterator get_timeline_iterator(
    Viewer::Implementation::Timeline& timeline, const Time time)
{
  Viewer::Implementation::Timeline::iterator start_it =
      timeline.lower_bound(time);

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
              std::make_unique<Viewer::Implementation::Bucket>()));
    }

    Viewer::Implementation::Timeline::iterator last_it = --timeline.end();
    while(last_it->first < time)
    {
      last_it = timeline.insert(
            timeline.end(),
            std::make_pair(
              last_it->first + BucketDuration,
              std::make_unique<Viewer::Implementation::Bucket>()));
    }

    return last_it;
  }

  while(time + BucketDuration < start_it->first)
  {
    start_it = timeline.insert(
          start_it,
          std::make_pair(
            start_it->first - BucketDuration,
            std::make_unique<Viewer::Implementation::Bucket>()));
  }

  return start_it;
}

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
Entry::Entry(
    ParticipantId _participant,
    Itinerary _itinerary,
    Version _schedule_version,
    ConstChangePtr _change)
  : participant(_participant),
    itinerary(std::move(_itinerary)),
    schedule_version(_schedule_version),
    change(std::move(_change)),
    succeeds(nullptr),
    succeeded_by(nullptr)
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
  elements.reserve(size);
}

//==============================================================================
void ViewRelevanceInspector::inspect(
    const ConstRoutePtr& route,
    const RouteInfo& info,
    const rmf_traffic::internal::Spacetime& spacetime_region)
{
  const auto& entry = info.latest_entry;
  assert(!entry->succeeded_by);

  if(after_version
     && versions.less_or_equal(entry->schedule_version, *after_version))
    return;

  if(rmf_traffic::internal::detect_conflicts(
       route->trajectory(), spacetime_region, nullptr))
    elements.emplace_back(Element{entry->participant, route});
}

//==============================================================================
void ViewRelevanceInspector::inspect(
    const ConstRoutePtr& route,
    const RouteInfo& info,
    const Time* lower_time_bound,
    const Time* upper_time_bound)
{
  const auto& entry = info.latest_entry;
  assert(!entry->succeeded_by);

  if(after_version
     && versions.less_or_equal(entry->schedule_version, *after_version))
    return;

  const Trajectory& trajectory = route->trajectory();
  assert(trajectory.start_time() != nullptr);

  if(lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
    return;

  if(upper_time_bound && *upper_time_bound < *trajectory.start_time())
    return;

  elements.emplace_back(Element{entry->participant, route});
}

//==============================================================================
void add_route_to_timeline(
    MapToTimeline& timelines,
    ConstRoutePtr route,
    RouteInfo& info)
{
  const Time start_time = *route->trajectory().start_time();
  const Time finish_time = *route->trajectory().finish_time();
  const std::string& map_name = route->map();

  const internal::MapToTimeline::iterator map_it = timelines.insert(
        std::make_pair(map_name, internal::Timeline())).first;

  internal::Timeline& timeline = map_it->second;

  const internal::Timeline::iterator start_it =
      get_timeline_iterator(timeline, start_time);
  const internal::Timeline::const_iterator end_it =
      ++get_timeline_iterator(timeline, finish_time);

  for (auto it = start_it; it != end_it; ++it)
  {
    it->second->push_back(route);
    info.buckets.push_back(it->second.get());
  }
}

//==============================================================================
void remove_route_from_timelines(
    const ConstRoutePtr& route,
    RouteInfo& info)
{
  for (auto* bucket : info.buckets)
  {
    const auto bucket_it = std::find(bucket->begin(), bucket->end(), route);
    if (bucket_it != bucket->end())
      bucket->erase(bucket_it);
  }
}

} // namespace internal

//==============================================================================
internal::EntryPtr Viewer::Implementation::add_entry(internal::EntryPtr entry)
{
  std::vector<ConstRoutePtr> removed_routes;

  const auto previous_it = current_itineraries.find(entry->participant);
  if (previous_it != current_itineraries.end())
    removed_routes = previous_it->second->itinerary;

  for (const auto& route : entry->itinerary)
  {
    // We may receive nullptr routes which implies that a participant has a
    // route in its itinerary that is filtered out by this mirror's query
    // parameters.
    if (!route)
      continue;

    internal::RouteToInfo::iterator route_it;
    bool inserted;
    std::tie(route_it, inserted) =
        routes.insert(std::make_pair(route, internal::RouteInfo{}));
    internal::RouteInfo& route_info = route_it->second;
    route_info.latest_entry = entry;

    if (inserted)
      add_route_to_timeline(timelines, route, route_info);

    const auto remove_it =
        std::find(removed_routes.begin(), removed_routes.end(), route);
    if (remove_it != removed_routes.end())
      removed_routes.erase(remove_it);
  }

  current_itineraries.at(entry->participant) = entry;

  for (const auto& route : removed_routes)
  {
    internal::RouteInfo& info = routes.at(route);
    remove_route_from_timelines(route, info);
  }

  return entry;
}

//==============================================================================
void Viewer::Implementation::remove_entry(internal::EntryPtr entry)
{
  const auto itinerary_it = current_itineraries.find(entry->participant);
  if (itinerary_it != current_itineraries.end())
  {
    // If the current itinerary for this participant is the one we are removing,
    // then erase the current itinerary for this participant altogether.
    if (itinerary_it->second == entry)
      itinerary_it->second = nullptr;
  }

  for (const auto& route : entry->itinerary)
  {
    internal::RouteInfo& info = routes.at(route);
    if (info.latest_entry == entry)
    {
      remove_route_from_timelines(route, info);
      info.latest_entry = nullptr;
      routes.erase(route);
    }
  }

  if (entry->succeeded_by)
    entry->succeeded_by->succeeds = nullptr;
}

//==============================================================================
void Viewer::Implementation::replace_entry(internal::EntryPtr entry)
{
  const auto previous_it = current_itineraries.find(entry->participant);
  internal::EntryPtr previous_entry;
  if (previous_it != current_itineraries.end())
    previous_entry = previous_it->second;

  add_entry(entry);

  if (previous_entry)
    remove_entry(previous_entry);
}

//==============================================================================
void Viewer::Implementation::cull(Version id, Time time)
{
  last_cull = std::make_pair(id, time);

  for(auto& pair : timelines)
  {
    internal::Timeline& timeline = pair.second;
    const internal::Timeline::iterator last_it = timeline.lower_bound(time);
    const internal::Timeline::iterator end_it = last_it == timeline.end()?
          timeline.end() : ++internal::Timeline::iterator(last_it);

    for(internal::Timeline::iterator it = timeline.begin(); it != end_it; ++it)
    {
      internal::Bucket& bucket = *it->second;
      const internal::Bucket::iterator removed =
          std::remove_if(bucket.begin(), bucket.end(),
                     [&](const internal::ConstEntryPtr& entry) -> bool
      {
        return *entry->trajectory.finish_time() < time;
      });

      for(internal::Bucket::iterator bit = removed; bit != bucket.end(); ++bit)
        culled.insert((*bit)->version);

      bucket.erase(removed, bucket.end());
    }

    internal::Timeline::iterator stop_erasing = timeline.begin();
    for(internal::Timeline::iterator it = timeline.begin(); it != end_it; ++it)
    {
      const internal::Bucket& bucket = *it->second;
      if(!bucket.empty())
      {
        // If this bucket is not empty, then stop the erasing now
        break;
      }

      // If this bucket is empty, then stop the erasing at the next entry
      stop_erasing = ++internal::Timeline::iterator(it);
    }

    timeline.erase(timeline.begin(), stop_erasing);
  }
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
    delayed_segment->adjust_times(total_delay);

  for(const Trajectory::Waypoint& interrupt_segment : interruption_trajectory)
  {
    new_trajectory.insert(
          interrupt_segment.time(),
          interrupt_segment.get_profile(),
          interrupt_segment.position(),
          interrupt_segment.velocity());
  }

  return new_trajectory;
}

//==============================================================================
Trajectory add_delay(
    Trajectory new_trajectory,
    const Time time,
    const Duration delay)
{
  assert(new_trajectory.start_time());
  if (time <= *new_trajectory.start_time())
  {
    new_trajectory.begin()->adjust_times(delay);
    return new_trajectory;
  }
  else if(*new_trajectory.finish_time() < time)
  {
    // No need for an adjustment
    return new_trajectory;
  }

  if (delay.count() < 0)
  {
    // For now we'll accept "negative delays" to convey a jump ahead in time,
    // but we apply it to the entire trajectory without being concerned about
    // the from_time parameter.
    // TODO(MXG): Consider if there is a more "correct" way to support this.
    new_trajectory.begin()->adjust_times(delay);
    return new_trajectory;
  }

  Trajectory::iterator delayed_segment = new_trajectory.find(time);
  assert(delayed_segment != new_trajectory.end());

  // The delay is generally meant to apply to the current moment in time.
  // Therefore the previous waypoint on the trajectory is still relevant to
  // the timing of the current trajectory. If we don't also shift the previous
  // waypoint by the delay, then the current trajectory will get warped, and
  // the schedule will predict a warped motion for the robot.
  if (delayed_segment != new_trajectory.begin())
    --delayed_segment;

  // TODO(MXG): Consider inserting a new waypoint(s) in the trajectory when
  // adding the delay. That may help to smooth things out further.

  delayed_segment->adjust_times(delay);

  return new_trajectory;
}

//==============================================================================
class Viewer::View::Implementation
{
public:

  std::vector<Element> elements;

  static View make_view(std::vector<Element> elements)
  {
    View view;
    view._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{std::move(elements)});
    return view;
  }
};

//==============================================================================
class Viewer::View::IterImpl
{
public:

  std::vector<Element>::const_iterator iter;

};

//==============================================================================
auto Viewer::View::begin() const -> const_iterator
{
  return const_iterator{IterImpl{_pimpl->elements.begin()}};
}

//==============================================================================
auto Viewer::View::end() const -> const_iterator
{
  return const_iterator{IterImpl{_pimpl->elements.end()}};
}

//==============================================================================
std::size_t Viewer::View::size() const
{
  return _pimpl->elements.size();
}

//==============================================================================
Viewer::View Viewer::query(const Query& parameters) const
{
  return View::Implementation::make_view(
        std::move(_pimpl->inspect<internal::ViewRelevanceInspector>(
                    parameters).elements));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Viewer::participant_ids() const
{
  return _pimpl->current_participant_ids;
}

//==============================================================================
rmf_utils::optional<const Participant&> Viewer::get_participant(
    std::size_t participant_id) const
{
  const Implementation::ParticipantMap::const_iterator it =
      _pimpl->current_participants.find(participant_id);

  if (it == _pimpl->current_participants.end())
    return rmf_utils::nullopt;

  return it->second;
}

//==============================================================================
rmf_utils::optional<Itinerary> Viewer::get_itinerary(
    std::size_t participant_id) const
{
  const Implementation::Itineraries::const_iterator it =
      _pimpl->current_itineraries.find(participant_id);

  if (it == _pimpl->current_itineraries.end())
    return rmf_utils::nullopt;

  if (!it->second)
    return rmf_utils::nullopt;

  return it->second->itinerary;
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
    const schedule::Viewer::View::Element,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic
