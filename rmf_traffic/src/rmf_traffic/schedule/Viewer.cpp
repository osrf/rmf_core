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

namespace internal {
//==============================================================================
ViewRelevanceInspector::ViewRelevanceInspector(
    const std::size_t* after_version,
    const std::size_t reserve_size)
  : after_version(after_version)
{
  relevant_entries.reserve(reserve_size);
}

//==============================================================================
void ViewRelevanceInspector::inspect(
    const EntryPtr& entry,
    const rmf_traffic::internal::Spacetime& spacetime_region)
{
  if(entry->succeeded_by)
    return;

  if(after_version && entry->version < *after_version)
    return;

  if(rmf_traffic::internal::detect_conflicts(
       entry->trajectory, spacetime_region, nullptr))
    relevant_entries.push_back(entry);
}

void ViewRelevanceInspector::inspect(
    const EntryPtr& entry,
    const Time* lower_time_bound,
    const Time* upper_time_bound)
{
  if(entry->succeeded_by)
    return;

  if(after_version && entry->version < *after_version)
    return;

  const Trajectory& trajectory = entry->trajectory;
  assert(trajectory.start_time() != nullptr);

  if(lower_time_bound && *trajectory.finish_time() < *lower_time_bound)
    return;

  if(upper_time_bound && *upper_time_bound < *trajectory.start_time())
    return;

  relevant_entries.push_back(entry);
}

} // namespace internal

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
Viewer::View Viewer::query(Query parameters) const
{
  const Query::Spacetime& spacetime = parameters.spacetime();
  const Query::Spacetime::Mode spacetime_mode = spacetime.get_mode();

  const Query::Versions& versions = parameters.versions();
  const Query::Versions::Mode versions_mode = versions.get_mode();

  std::vector<internal::EntryPtr> qualified_entries;

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
              [&](const internal::EntryPtr& entry){
                return entry->version <= after_version;
              });
        qualified_entries.erase(removed, qualified_entries.end());
      }
      break;
    }

    case Query::Spacetime::Mode::Regions:
    {
      assert(spacetime.regions() != nullptr);

      internal::ViewRelevanceInspector inspector{
        after_version_ptr, _pimpl->all_entries.size()};
      _pimpl->inspect_spacetime_region_entries(*spacetime.regions(), inspector);
      qualified_entries = inspector.relevant_entries;
      break;
    }

    case Query::Spacetime::Mode::Timespan:
    {
      assert(spacetime.timespan() != nullptr);
      const Query::Spacetime::Timespan& timespan = *spacetime.timespan();

      internal::ViewRelevanceInspector inspector{
        after_version_ptr, _pimpl->all_entries.size()};
      _pimpl->inspect_timespan(
            timespan.get_maps(),
            timespan.get_lower_time_bound(),
            timespan.get_upper_time_bound(),
            inspector);
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
