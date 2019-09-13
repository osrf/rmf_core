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

#include <rmf_traffic/Trajectory.hpp>

#include <map>
#include <vector>

namespace rmf_traffic {

//==============================================================================
namespace {
struct SegmentData
{
  Trajectory::Time finish_time;
  Trajectory::ConstProfilePtr profile;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

} // anonymous namespace

//==============================================================================
template<typename SegT>
class Trajectory::base_iterator<SegT>::Implementation
{
public:

  std::shared_ptr<std::size_t> index;
  Trajectory::Implementation* parent;

};

//==============================================================================
template<typename SegT>
Trajectory::base_iterator<SegT>::base_iterator()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
class Trajectory::Implementation
{
public:

  template<typename SegT>
  base_iterator<SegT> make_iterator(std::shared_ptr<std::size_t> index)
  {
    base_iterator<SegT> it;
    it._pimpl->index = index;
    it._pimpl->parent = this;
  }

  using OrderMap = std::map<Time, std::shared_ptr<std::size_t>>;

  std::string map_name;
  OrderMap ordering;
  std::vector<SegmentData> segments;

  OrderMap::iterator result;
  InsertionResult insert(SegmentData data)
  {
    const OrderMap::iterator hint = ordering.lower_bound(data.finish_time);
    if(hint->first == data.finish_time)
    {
      // We already have a Segment in the Trajectory that ends at this same
      // exact moment in time, so we will return the existing iterator along
      // with inserted==false.
      return InsertionResult{make_iterator<Segment>(hint->second), false};
    }

    if(hint == ordering.end())
    {
      // We know we should insert this at the back of the vector
      result = ordering.emplace_hint(
            ordering.end(),
            data.finish_time,
            std::make_shared<std::size_t>(segments.size()));

      segments.emplace_back(std::move(data));
    }
    else
    {
      // The index is where the new segment should be inserted in the vector
      const std::size_t index = *hint->second;
      result = ordering.emplace_hint(
            hint, std::make_shared<std::size_t>(index));

      // Increment the indices of the Segments that come after this one so that
      // they point to the correct element in the data set.
      for(OrderMap::iterator it = hint; it != ordering.end(); ++it)
        ++(*it->second);

      segments.emplace(segments.begin()+index, std::move(data));
    }

    return InsertionResult{make_iterator<Segment>(result->second), true};
  }

};

//==============================================================================
class Trajectory::Profile::Implementation
{
public:

  Implementation(geometry::ConstConvexShapePtr shape)
    : shape(std::move(shape)),
      queue_info(this)
  {
    // Do nothing
  }

  // Basic information
  geometry::ConstConvexShapePtr shape;

  // Queue information (only used when in Queue movement mode)
  QueueInfo queue_info;
  std::string queue_id;

  // Movement mode
  Movement movement;
};

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_strict(
    geometry::ConstConvexShapePtr shape)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_strict();
  return result;
}

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_autonomous(
    geometry::ConstConvexShapePtr shape)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_autonomous();
  return result;
}

//==============================================================================
Trajectory::ProfilePtr Trajectory::Profile::make_queued(
    geometry::ConstConvexShapePtr shape,
    const std::string& queue_id)
{
  ProfilePtr result(new Profile(std::move(shape)));
  result->set_to_queued(queue_id);
  return result;
}

//==============================================================================
geometry::ConstConvexShapePtr Trajectory::Profile::get_shape() const
{
  return _pimpl->shape;
}

//==============================================================================
void Trajectory::Profile::set_shape(geometry::ConstConvexShapePtr new_shape)
{
  _pimpl->shape = std::move(new_shape);
}

//==============================================================================
Trajectory::Profile::Movement Trajectory::Profile::get_movement() const
{
  return _pimpl->movement;
}

//==============================================================================
void Trajectory::Profile::set_to_strict()
{
  _pimpl->movement = Strict;
}

//==============================================================================
void Trajectory::Profile::set_to_autonomous()
{
  _pimpl->movement = Autonomous;
}

//==============================================================================
void Trajectory::Profile::set_to_queued(const std::string& queue_id)
{
  _pimpl->movement = Queued;
  _pimpl->queue_id = queue_id;
}

//==============================================================================
std::string Trajectory::Profile::QueueInfo::get_queue_id() const
{
  return static_cast<const Profile::Implementation*>(_pimpl)->queue_id;
}

//==============================================================================
Trajectory::Profile::QueueInfo::QueueInfo(void* pimpl)
  : _pimpl(pimpl)
{
  // Do nothing
}

//==============================================================================
auto Trajectory::Profile::get_queue_info() const -> const QueueInfo*
{
  if(Queued == _pimpl->movement)
    return &_pimpl->queue_info;

  return nullptr;
}

//==============================================================================
Trajectory::Profile::Profile(geometry::ConstConvexShapePtr shape)
  : _pimpl(rmf_utils::make_impl<Implementation>(std::move(shape)))
{
  // Do nothing
}

//==============================================================================
class Trajectory::Segment::Implementation
{
public:

  std::shared_ptr<std::size_t> index;
  Trajectory::Implementation* parent;

  Implementation(Trajectory::Implementation* parent)
    : parent(parent)
  {
    // Do nothing
  }

  SegmentData& data()
  {
    return parent->segments[*index];
  }

  const SegmentData& data() const
  {
    return parent->segments[*index];
  }

  Time time() const
  {
    return data().finish_time;
  }

};

//==============================================================================
auto Trajectory::Segment::get_profile() const -> ConstProfilePtr
{
  return _pimpl->data().profile;
}

//==============================================================================
void Trajectory::Segment::set_profile(ConstProfilePtr new_profile)
{
  _pimpl->data().profile = std::move(new_profile);
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_position() const
{
  return _pimpl->data().position;
}

//==============================================================================
void Trajectory::Segment::set_position(Eigen::Vector3d new_position)
{
  _pimpl->data().position = std::move(new_position);
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_velocity() const
{
  return _pimpl->data().velocity;
}

//==============================================================================
void Trajectory::Segment::set_velocity(Eigen::Vector3d new_velocity)
{
  _pimpl->data().velocity = std::move(new_velocity);
}

//==============================================================================
Trajectory::Time Trajectory::Segment::get_finish_time() const
{
  return _pimpl->time();
}

//==============================================================================
void Trajectory::Segment::set_finish_time(const Time new_time)
{
  using OrderMap = Trajectory::Implementation::OrderMap;
  const std::size_t current_index = *_pimpl->index;
  OrderMap& ordering = _pimpl->parent->ordering;
  std::vector<SegmentData>& segments = _pimpl->parent->segments;

  const SegmentData& current_data = segments[current_index];
  const Time current_time = current_data.finish_time;

  if(current_time == new_time)
  {
    // Short-circuit, since nothing is changing. The erase and re-insertion
    // would be a waste of time in this case.
    return;
  }

  OrderMap::iterator it = ordering.find(current_time);
  assert(it != ordering.end());






  SegmentData data = std::move(it->second);
  // Note: erasing an iterator does not invalidate any iterators for the map,
  // except the one that was erased. Remember that `it` is a mutable reference,
  // so we have its current value erased in this next line, but we will be
  // updating its value in a moment when we insert the new version.
  segments.erase(it);

  if(possible_hint == segments.end() || new_time < possible_hint->first)
  {
    // This will make a valid hint for the insertion
    it = segments.emplace_hint(possible_hint, new_time, std::move(data));
  }
  else
  {
    // We do not have a hint readily available for the new entry, so we'll just
    // use the normal map insertion method. We also don't know if the new time
    // might conflict with an existing waypoint's time, so we should check for
    // that as well.
    const auto result = segments.emplace(new_time, std::move(data));

    if(!result.second)
    {
      // The new time conflicts with an existing time, so we will throw an
      // exception.
      throw std::invalid_argument(
            "[Trajectory::Segment::set_finish_time] Attempted to set time to "
            + std::to_string(new_time.time_since_epoch().count())
            + "ns, but a waypoint already exists at that timestamp.");
    }

    it = result.first;
  }
}

//==============================================================================
void Trajectory::Segment::adjust_finish_times(Duration delta_t)
{
  SegmentDataMap::iterator& begin_it = *_pimpl->it;
  SegmentDataMap& segments = _pimpl->parent->segments;

  std::vector<SegmentData> temp_data_storage;
  temp_data_storage.reserve(segments.size());
  std::vector<Time> temp_time_storage;
  temp_time_storage.reserve(segments.size());
  for(SegmentDataMap::iterator it = begin_it; it != segments.end(); ++it)
  {
    // Store all the waypoints as cheaply as possible
    temp_time_storage.emplace_back(it->first);
    temp_data_storage.emplace_back(std::move(it->second));
  }

  // Erase the elements we're going to modify from the container
  segments.erase(begin_it, segments.end());

  for(std::size_t i=0; i < temp_data_storage.size(); ++i)
  {
    // Put all the elements back in, with their new time value.
    // By providing the hint that they belong as the back of the map, we can
    // ensure that each insertion has constant-time complexity.
    const Time new_time = temp_time_storage[i] + delta_t;
    segments.emplace_hint(
          segments.end(), new_time, std::move(temp_data_storage[i]));
  }
}

//==============================================================================
std::string Trajectory::get_map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
void Trajectory::set_map_name(std::string name)
{
  _pimpl->map_name = std::move(name);
}

//==============================================================================
Trajectory::iterator Trajectory::insert(
    Time finish_time,
    ConstProfilePtr profile,
    Eigen::Vector3d position,
    Eigen::Vector3d velocity)
{
  return _pimpl->insert(
        SegmentData{
          std::move(finish_time),
          std::move(profile),
          std::move(position),
          std::move(velocity)});
}

//==============================================================================
template class Trajectory::base_iterator<Trajectory::Segment>;
template class Trajectory::base_iterator<const Trajectory::Segment>;

} // namespace rmf_traffic
