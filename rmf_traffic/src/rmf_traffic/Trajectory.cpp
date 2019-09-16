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
#include <list>
#include <string>

namespace rmf_traffic {

//==============================================================================
namespace {

struct SegmentData;
using SegmentList = std::list<SegmentData>;
using OrderMap = std::map<Trajectory::Time, SegmentList::iterator>;

struct SegmentData
{
  Trajectory::Time finish_time;
  Trajectory::ConstProfilePtr profile;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;

  // We store a Trajectory::Segment in this struct so that we can always safely
  // return a reference to a Trajectory::Segment object. As long as this
  // SegmentData is alive, any Trajectory::Segment reference that refers to it
  // will remain valid.
  Trajectory::Segment myself;
};

} // anonymous namespace

//==============================================================================
namespace detail {
class TrajectoryIteratorImplementation
{
public:

  SegmentList::iterator raw_iterator;
  const Trajectory::Implementation* parent;

  template<typename SegT>
  Trajectory::base_iterator<SegT> make_iterator(SegmentList::iterator it)
  {
    Trajectory::base_iterator<SegT> result;
    result._pimpl->raw_iterator = it;
    result._pimpl->parent = parent;

    return result;
  }

  template<typename SegT>
  Trajectory::base_iterator<SegT> post_increment()
  {
    const Trajectory::base_iterator<SegT> old_it =
        make_iterator<SegT>(raw_iterator);

    ++raw_iterator;

    return old_it;
  }

  template<typename SegT>
  Trajectory::base_iterator<SegT> post_decrement()
  {
    const Trajectory::base_iterator<SegT> old_it =
        make_iterator<SegT>(raw_iterator);

    --raw_iterator;

    return old_it;
  }

};
} // namespace detail

//==============================================================================
class Trajectory::Segment::Implementation
{
public:

  // Note: these fields will be filled in by the
  // Trajectory::Implementation::insert() function.
  SegmentList::iterator myself;
  Trajectory::Implementation* parent;

  SegmentData& data()
  {
    return *myself;
  }

  const SegmentData& data() const
  {
    return *myself;
  }

  Time time() const
  {
    return data().finish_time;
  }

};

//==============================================================================
template<typename SegT>
Trajectory::base_iterator<SegT>::base_iterator()
  : _pimpl(rmf_utils::make_impl<detail::TrajectoryIteratorImplementation>())
{
  // Do nothing
}

//==============================================================================
class Trajectory::Implementation
{
public:

  template<typename SegT>
  base_iterator<SegT> make_iterator(SegmentList::iterator iterator) const
  {
    base_iterator<SegT> it;
    it._pimpl->raw_iterator = std::move(iterator);
    it._pimpl->parent = this;

    return it;
  }

  std::string map_name;
  OrderMap ordering;
  SegmentList segments;

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

    const SegmentList::const_iterator list_destination =
        (hint == ordering.end()) ? segments.end() : hint->second;

    const SegmentList::iterator result =
        segments.emplace(list_destination, std::move(data));
    result->myself._pimpl->myself = result;
    result->myself._pimpl->parent = this;

    ordering.emplace_hint(hint, data.finish_time, result);

    return InsertionResult{make_iterator<Segment>(result), true};
  }

  iterator find(Time time)
  {
    const auto it = ordering.lower_bound(time);
    if(it == ordering.end())
      return make_iterator<Segment>(segments.end());

    return make_iterator<Segment>(it->second);
  }

  iterator erase(iterator segment)
  {
    ordering.erase(segment->_pimpl->myself->finish_time);
    return make_iterator<Segment>(segments.erase(segment->_pimpl->myself));
  }

  iterator erase(iterator first, iterator last)
  {
    const auto seg_begin = first->_pimpl->myself;
    const auto seg_end = last->_pimpl->myself;

    const auto order_start = ordering.find(seg_begin->finish_time);
    const auto order_end = seg_end == segments.end()?
          ordering.end() : ordering.find(seg_end->finish_time);

    ordering.erase(order_start, order_end);
    return make_iterator<Segment>(segments.erase(seg_begin, seg_end));
  }

  iterator end()
  {
    return make_iterator<Segment>(segments.end());
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
  SegmentList::iterator data_it = _pimpl->myself;
  SegmentData& current_data = *data_it;
  const Time current_time = current_data.finish_time;

  if(current_time == new_time)
  {
    // Short-circuit, since nothing is changing. The erase and re-insertion
    // would be a waste of time in this case.
    return;
  }

  OrderMap& ordering = _pimpl->parent->ordering;
  SegmentList& segments = _pimpl->parent->segments;
  const OrderMap::const_iterator current_order_it = ordering.find(current_time);
  assert(current_order_it != ordering.end());
  const OrderMap::const_iterator hint = ordering.lower_bound(new_time);

  if(current_order_it == hint)
  {
    // The Segment is already in the correct location within the list, so it
    // does not need to be moved. We can just update its entry in the OrderMap.

    // We need to create a new_hint iterator which points to the iterator after
    // hint because the hint iterator will be invalidated when we erase
    // current_order_it (because they are iterators to the same element).
    const OrderMap::const_iterator new_hint = ++OrderMap::const_iterator(hint);
    ordering.erase(current_order_it);
    ordering.emplace_hint(new_hint, new_time, std::move(data_it));
  }
  else if(hint == ordering.end())
  {
    // This Segment must be moved to the end of the list.
    segments.splice(segments.end(), segments, data_it);
    ordering.erase(current_order_it);
    ordering.emplace_hint(hint, new_time, std::move(data_it));
  }
  else
  {
    const SegmentList::const_iterator destination = hint->second;
    assert(destination != segments.end());

    if(destination->finish_time == new_time)
    {
      // The new time conflicts with an existing time, so we will throw an
      // exception.
      throw std::invalid_argument(
            "[Trajectory::Segment::set_finish_time] Attempted to set time to "
            + std::to_string(new_time.time_since_epoch().count())
            + "ns, but a waypoint already exists at that timestamp.");
    }

    segments.splice(destination, segments, data_it);
    ordering.erase(current_order_it);
    ordering.emplace_hint(hint, new_time, std::move(data_it));
  }

  // Update the finish_time value in the data field.
  current_data.finish_time = new_time;
}

//==============================================================================
void Trajectory::Segment::adjust_finish_times(Duration delta_t)
{
  SegmentList& segments = _pimpl->parent->segments;
  const SegmentList::iterator begin_it = _pimpl->myself;

  if(delta_t.count() < 0 && begin_it != segments.begin())
  {
    // If delta_t is negative and this is not the first Segment in the
    // Trajectory, make sure the change in time does not make it dip beneath its
    // predecessor Segment.
    const SegmentList::const_iterator predecessor_it =
        ++SegmentList::iterator(begin_it);
    const auto new_time = begin_it->finish_time + delta_t;
    if(new_time <= predecessor_it->finish_time)
    {
      const auto tp = predecessor_it->finish_time.time_since_epoch().count();
      const auto tc = (new_time).time_since_epoch().count();

      const std::string error =
          std::string("[Trajectory::Segment::adjust_finish_times] ")
          + "The given negative change in time: "
          + std::to_string(delta_t.count()) + "ns caused the Segment's new "
          + "time window [" + std::to_string(tc)
          + "] to overlap with its precedessor's [" + std::to_string(tp)
          + "]";
    }
  }

  // Adjust the times for the segments and collect their iterators
  std::vector<SegmentList::iterator> list_iterators;
  list_iterators.reserve(segments.size());
  for(SegmentList::iterator it = begin_it; it != segments.end(); ++it)
  {
    it->finish_time += delta_t;
    list_iterators.push_back(it);
  }

  OrderMap& ordering = _pimpl->parent->ordering;
  const OrderMap::iterator order_it = ordering.find(begin_it->finish_time);
  assert(order_it != ordering.end());

  // Erase the existing ordering entries for all the modified Trajectory
  // Segments.
  ordering.erase(order_it, ordering.end());

  // Add new entries one at a time, supplying the emplacement operator with the
  // hint that it can always append the entry to the end of the map.
  for(SegmentList::iterator& it : list_iterators)
  {
    const Time new_time = it->finish_time;
    ordering.emplace_hint(ordering.end(), new_time, std::move(it));
  }
}

//==============================================================================
Trajectory::Segment::Segment()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
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
Trajectory::InsertionResult Trajectory::insert(
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
          std::move(velocity),
          Segment{}});
}

//==============================================================================
Trajectory::iterator Trajectory::find(Time time)
{
  return _pimpl->find(time);
}

//==============================================================================
Trajectory::const_iterator Trajectory::find(Time time) const
{
  return const_cast<Implementation&>(*_pimpl).find(time);
}

//==============================================================================
Trajectory::iterator Trajectory::erase(iterator segment)
{
  return _pimpl->erase(segment);
}

//==============================================================================
Trajectory::iterator Trajectory::erase(iterator first, iterator last)
{
  return _pimpl->erase(first, last);
}

//==============================================================================
Trajectory::iterator Trajectory::end()
{
  return _pimpl->end();
}

//==============================================================================
Trajectory::const_iterator Trajectory::end() const
{
  return const_cast<Implementation&>(*_pimpl).end();
}

//==============================================================================
Trajectory::const_iterator Trajectory::cend() const
{
  return const_cast<Implementation&>(*_pimpl).end();
}

//==============================================================================
template<typename SegT>
SegT& Trajectory::base_iterator<SegT>::operator*() const
{
  return _pimpl->raw_iterator->myself;
}

//==============================================================================
template<typename SegT>
SegT* Trajectory::base_iterator<SegT>::operator->() const
{
  return &_pimpl->raw_iterator->myself;
}

//==============================================================================
template<typename SegT>
auto Trajectory::base_iterator<SegT>::operator++() -> base_iterator&
{
  ++_pimpl->raw_iterator;
  return *this;
}

//==============================================================================
template<typename SegT>
auto Trajectory::base_iterator<SegT>::operator--() -> base_iterator&
{
  --_pimpl->raw_iterator;
  return *this;
}

//==============================================================================
template<typename SegT>
auto Trajectory::base_iterator<SegT>::operator++(int) -> base_iterator
{
  return _pimpl->post_increment<SegT>();
}

//==============================================================================
template<typename SegT>
auto Trajectory::base_iterator<SegT>::operator--(int) -> base_iterator
{
  return _pimpl->post_decrement<SegT>();
}

//==============================================================================
#define DEFINE_BASIC_ITERATOR_OP(op) \
  template<typename SegT> \
  bool Trajectory::base_iterator<SegT>::operator op ( \
      const base_iterator& other) const \
  { \
    return _pimpl->raw_iterator op other._pimpl->raw_iterator; \
  }

DEFINE_BASIC_ITERATOR_OP(==)
DEFINE_BASIC_ITERATOR_OP(!=)

template<typename SegT>
bool Trajectory::base_iterator<SegT>::operator<(
    const base_iterator& other) const
{
  return (other._pimpl->raw_iterator == _pimpl->raw_iterator);
}

//==============================================================================
template class Trajectory::base_iterator<Trajectory::Segment>;
template class Trajectory::base_iterator<const Trajectory::Segment>;

} // namespace rmf_traffic
