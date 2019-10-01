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

#include "debug_Trajectory.hpp"
#include "MotionInternal.hpp"
#include "TrajectoryInternal.hpp"

#include <iostream>
#include <string>

namespace rmf_traffic {

//==============================================================================
namespace detail {
class TrajectoryIteratorImplementation
{
public:

  internal::SegmentList::iterator raw_iterator;
  const Trajectory::Implementation* parent;

  template<typename SegT>
  Trajectory::base_iterator<SegT> make_iterator(
      internal::SegmentList::iterator it) const
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
  internal::SegmentList::iterator myself;
  Trajectory::Implementation* parent;

  internal::SegmentElement::Data& data()
  {
    return myself->data;
  }

  const internal::SegmentElement::Data& data() const
  {
    return myself->data;
  }

  Time time() const
  {
    return data().finish_time;
  }

};

//==============================================================================
class Trajectory::Implementation
{
public:

  std::string map_name;
  internal::OrderMap ordering;
  internal::SegmentList segments;

  template<typename SegT>
  base_iterator<SegT> make_iterator(
      internal::SegmentList::iterator iterator) const
  {
    base_iterator<SegT> it;
    it._pimpl->raw_iterator = std::move(iterator);
    it._pimpl->parent = this;

    return it;
  }

  std::unique_ptr<Segment> make_segment(internal::SegmentList::iterator iterator)
  {
    std::unique_ptr<Segment> seg(new Segment);
    seg->_pimpl->myself = std::move(iterator);
    seg->_pimpl->parent = this;

    return seg;
  }

  Implementation(std::string map_name)
    : map_name(std::move(map_name))
  {
    // Do nothing
  }

  Implementation(const Implementation& other)
  {
    *this = other;
  }

  Implementation& operator=(const Implementation& other)
  {
    // Start by making a normal copy
    map_name = other.map_name;
    ordering = other.ordering;
    segments = other.segments;

    // Now correct all the iterators to point to the freshly copied container
    internal::SegmentList::iterator sit = segments.begin();
    internal::OrderMap::iterator oit = ordering.begin();
    for( ; sit != segments.end(); ++sit, ++oit)
    {
      sit->myself = make_segment(sit);
      oit->second = sit;
    }

    return *this;
  }

  InsertionResult insert(internal::SegmentElement::Data data)
  {
    const internal::OrderMap::iterator hint = ordering.lower_bound(data.finish_time);
    if(hint->first == data.finish_time)
    {
      // We already have a Segment in the Trajectory that ends at this same
      // exact moment in time, so we will return the existing iterator along
      // with inserted==false.
      return InsertionResult{make_iterator<Segment>(hint->second), false};
    }

    const internal::SegmentList::const_iterator list_destination =
        (hint == ordering.end()) ? segments.end() : hint->second;

    const internal::SegmentList::iterator result =
        segments.emplace(list_destination, std::move(data));
    result->myself = make_segment(result);

    ordering.emplace_hint(hint, data.finish_time, result);

    return InsertionResult{make_iterator<Segment>(result), true};
  }

  iterator find(Time time)
  {
    const auto it = ordering.lower_bound(time);
    if(it == ordering.end())
      return make_iterator<Segment>(segments.end());

    // If the time comes before the start of the Trajectory, then we return
    // the end() iterator
    if(time < segments.begin()->data.finish_time)
      return make_iterator<Segment>(segments.end());

    return make_iterator<Segment>(it->second);
  }

  iterator erase(iterator segment)
  {
    ordering.erase(segment->_pimpl->myself->data.finish_time);
    return make_iterator<Segment>(segments.erase(segment->_pimpl->myself));
  }

  iterator erase(iterator first, iterator last)
  {
    const auto seg_begin = first->_pimpl->myself;
    const auto seg_end = last._pimpl->raw_iterator == segments.end()?
          segments.end() : last->_pimpl->myself;

    const auto order_start = ordering.find(seg_begin->data.finish_time);
    const auto order_end = seg_end == segments.end()?
          ordering.end() : ordering.find(seg_end->data.finish_time);

    ordering.erase(order_start, order_end);
    return make_iterator<Segment>(segments.erase(seg_begin, seg_end));
  }

  iterator begin()
  {
    return make_iterator<Segment>(segments.begin());
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
      strict_info(this),
      autonomous_info(this),
      queue_info(this)
  {
    // Do nothing
  }

  // Basic information
  geometry::ConstConvexShapePtr shape;

  // Strict mode information (only used when in the Strict agency mode)
  StrictInfo strict_info;

  // Autonomous mode information (only used when in the Autonomous agency mode)
  AutonomousInfo autonomous_info;

  // Queued mode information (only used when in the Queued agency mode)
  QueueInfo queue_info;
  std::string queue_id;

  // Agency mode
  Agency agency_mode;
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
Trajectory::Profile& Trajectory::Profile::set_shape(
    geometry::ConstConvexShapePtr new_shape)
{
  _pimpl->shape = std::move(new_shape);
  return *this;
}

//==============================================================================
Trajectory::Profile::Agency Trajectory::Profile::get_agency() const
{
  return _pimpl->agency_mode;
}

//==============================================================================
Trajectory::Profile::StrictInfo::StrictInfo(void* pimpl)
  : _pimpl(pimpl)
{
  // Do nothing
}

//==============================================================================
Trajectory::Profile::StrictInfo& Trajectory::Profile::set_to_strict()
{
  _pimpl->agency_mode = Agency::Strict;
  return _pimpl->strict_info;
}

//==============================================================================
Trajectory::Profile::AutonomousInfo::AutonomousInfo(void* pimpl)
  : _pimpl(pimpl)
{
  // Do nothing
}

//==============================================================================
Trajectory::Profile::AutonomousInfo& Trajectory::Profile::set_to_autonomous()
{
  _pimpl->agency_mode = Agency::Autonomous;
  return _pimpl->autonomous_info;
}

//==============================================================================
Trajectory::Profile::QueueInfo& Trajectory::Profile::set_to_queued(
    const std::string& queue_id)
{
  _pimpl->agency_mode = Agency::Queued;
  _pimpl->queue_id = queue_id;
  return _pimpl->queue_info;
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
  if(Agency::Queued == _pimpl->agency_mode)
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
Trajectory::Segment& Trajectory::Segment::set_profile(
    ConstProfilePtr new_profile)
{
  _pimpl->data().profile = std::move(new_profile);
  return *this;
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_finish_position() const
{
  return _pimpl->data().position;
}

//==============================================================================
Trajectory::Segment& Trajectory::Segment::set_finish_position(
    Eigen::Vector3d new_position)
{
  _pimpl->data().position = std::move(new_position);
  return *this;
}

//==============================================================================
Eigen::Vector3d Trajectory::Segment::get_finish_velocity() const
{
  return _pimpl->data().velocity;
}

//==============================================================================
Trajectory::Segment& Trajectory::Segment::set_finish_velocity(
    Eigen::Vector3d new_velocity)
{
  _pimpl->data().velocity = std::move(new_velocity);
  return *this;
}

//==============================================================================
Time Trajectory::Segment::get_finish_time() const
{
  return _pimpl->time();
}

//==============================================================================
Trajectory::Segment& Trajectory::Segment::set_finish_time(const Time new_time)
{
  internal::SegmentList::iterator data_it = _pimpl->myself;
  internal::SegmentElement::Data& current_data = data_it->data;
  const Time current_time = current_data.finish_time;

  if(current_time == new_time)
  {
    // Short-circuit, since nothing is changing. The erase and re-insertion
    // would be a waste of time in this case.
    return *this;
  }

  internal::OrderMap& ordering = _pimpl->parent->ordering;
  internal::SegmentList& segments = _pimpl->parent->segments;
  const internal::OrderMap::const_iterator current_order_it =
      ordering.find(current_time);
  assert(current_order_it != ordering.end());

  const internal::OrderMap::const_iterator hint =
      ordering.lower_bound(new_time);

  if(current_order_it == hint)
  {
    // The Segment is already in the correct location within the list, so it
    // does not need to be moved. We can just update its entry in the OrderMap.

    // We need to create a new_hint iterator which points to the iterator after
    // hint because the hint iterator will be invalidated when we erase
    // current_order_it (because they are iterators to the same element).
    const internal::OrderMap::const_iterator new_hint =
        ++internal::OrderMap::const_iterator(hint);
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
    const internal::SegmentList::const_iterator destination = hint->second;
    assert(destination != segments.end());

    if(destination->data.finish_time == new_time)
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

  return *this;
}

//==============================================================================
void Trajectory::Segment::adjust_finish_times(Duration delta_t)
{
  internal::SegmentList& segments = _pimpl->parent->segments;
  const internal::SegmentList::iterator begin_it = _pimpl->myself;
  const Time original_begin_time = begin_it->data.finish_time;

  if(delta_t.count() < 0 && begin_it != segments.begin())
  {
    // If delta_t is negative and this is not the first Segment in the
    // Trajectory, make sure the change in time does not make it dip beneath its
    // predecessor Segment.
    const internal::SegmentList::const_iterator predecessor_it =
        --internal::SegmentList::iterator(begin_it);
    const auto new_time = begin_it->data.finish_time + delta_t;
    if(new_time <= predecessor_it->data.finish_time)
    {
      const auto tp = predecessor_it->data.finish_time
          .time_since_epoch().count();
      const auto tc = (new_time).time_since_epoch().count();

      const std::string error =
          std::string("[Trajectory::Segment::adjust_finish_times] ")
          + "The given negative change in time: "
          + std::to_string(delta_t.count()) + "ns caused the Segment's new "
          + "time window [" + std::to_string(tc)
          + "] to overlap with its precedessor's [" + std::to_string(tp)
          + "]";

      throw std::invalid_argument(error);
    }
  }

  // Adjust the times for the segments and collect their iterators
  std::vector<internal::SegmentList::iterator> list_iterators;
  list_iterators.reserve(segments.size());
  for(internal::SegmentList::iterator it = begin_it; it != segments.end(); ++it)
  {
    it->data.finish_time += delta_t;
    list_iterators.push_back(it);
  }

  internal::OrderMap& ordering = _pimpl->parent->ordering;
  const internal::OrderMap::iterator order_it =
      ordering.find(original_begin_time);
  assert(order_it != ordering.end());

  // Erase the existing ordering entries for all the modified Trajectory
  // Segments.
  ordering.erase(order_it, ordering.end());

  // Add new entries one at a time, supplying the emplacement operator with the
  // hint that it can always append the entry to the end of the map.
  for(internal::SegmentList::iterator& it : list_iterators)
  {
    const Time new_time = it->data.finish_time;
    ordering.emplace_hint(ordering.end(), new_time, std::move(it));
  }
}

//==============================================================================
std::unique_ptr<Motion> Trajectory::Segment::compute_motion() const
{
  const internal::SegmentList& segments = _pimpl->parent->segments;
  const internal::SegmentList::const_iterator& it = _pimpl->myself;
  const internal::SegmentElement::Data& finish_data = it->data;

  if(it == segments.begin())
  {
    return std::make_unique<SinglePointMotion>(
          finish_data.finish_time,
          finish_data.position,
          finish_data.velocity);
  }

  return std::make_unique<SplineMotion>(Spline(it));
}

//==============================================================================
Trajectory::Segment::Segment()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Trajectory::Trajectory(std::string map_name)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(std::move(map_name)))
{
  // Do nothing
}

//==============================================================================
Trajectory::Trajectory(const Trajectory& other)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(*other._pimpl))
{
  // Do nothing
}

//==============================================================================
Trajectory& Trajectory::operator=(const Trajectory& other)
{
  *_pimpl = *other._pimpl;
  return *this;
}

//==============================================================================
std::string Trajectory::get_map_name() const
{
  return _pimpl->map_name;
}

//==============================================================================
Trajectory& Trajectory::set_map_name(std::string name)
{
  _pimpl->map_name = std::move(name);
  return *this;
}

//==============================================================================
Trajectory::InsertionResult Trajectory::insert(
    Time finish_time,
    ConstProfilePtr profile,
    Eigen::Vector3d position,
    Eigen::Vector3d velocity)
{
  return _pimpl->insert(
        internal::SegmentElement::Data{
          std::move(finish_time),
          std::move(profile),
          std::move(position),
          std::move(velocity)});
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
Trajectory::iterator Trajectory::begin()
{
  return _pimpl->begin();
}

//==============================================================================
Trajectory::const_iterator Trajectory::begin() const
{
  return const_cast<Implementation&>(*_pimpl).begin();
}

//==============================================================================
Trajectory::const_iterator Trajectory::cbegin() const
{
  return const_cast<Implementation&>(*_pimpl).begin();
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
const Time* Trajectory::start_time() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() == 0? nullptr : &segments.front().data.finish_time;
}

//==============================================================================
const Time* Trajectory::finish_time() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() == 0? nullptr : &segments.back().data.finish_time;
}

//==============================================================================
Duration Trajectory::duration() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() < 2?
        Duration(0) :
        segments.back().data.finish_time - segments.front().data.finish_time;
}

//==============================================================================
std::size_t Trajectory::size() const
{
  return _pimpl->segments.size();
}

//==============================================================================
template<typename SegT>
SegT& Trajectory::base_iterator<SegT>::operator*() const
{
  return *_pimpl->raw_iterator->myself;
}

//==============================================================================
template<typename SegT>
SegT* Trajectory::base_iterator<SegT>::operator->() const
{
  return _pimpl->raw_iterator->myself.get();
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
  const bool this_is_end =
      this->_pimpl->raw_iterator == _pimpl->parent->segments.end();
  const bool other_is_end =
      other._pimpl->raw_iterator == _pimpl->parent->segments.end();

  if(this_is_end || other_is_end)
  {
    // If the other is the end iterator but this iterator is not, then we return
    // true, because the end iterator is "larger" than any valid iterator.
    // If the other is not the end iterator but this iterator is, then we return
    // false, because this iterator is definitely larger than anything the other
    // can be.
    return other_is_end && !this_is_end;
  }

  // If they are both valid iterators, then we can compare their times.
  return this->_pimpl->raw_iterator->data.finish_time
      < other._pimpl->raw_iterator->data.finish_time;
}

//==============================================================================
template<typename SegT>
bool Trajectory::base_iterator<SegT>::operator>(
    const base_iterator& other) const
{
  const bool this_is_end =
      this->_pimpl->raw_iterator == _pimpl->parent->segments.end();
  const bool other_is_end =
      other._pimpl->raw_iterator == _pimpl->parent->segments.end();

  if(this_is_end || other_is_end)
  {
    // See the logic above for operator<, but in this case we want the opposite
    // conclusion.
    return this_is_end && !other_is_end;
  }

  // If they are both valid iterators, then we can compare their times.
  return this->_pimpl->raw_iterator->data.finish_time
      > other._pimpl->raw_iterator->data.finish_time;
}

//==============================================================================
template<typename SegT>
bool Trajectory::base_iterator<SegT>::operator<=(
    const base_iterator& other) const
{
  return (*this == other) || (*this < other);
}

//==============================================================================
template<typename SegT>
bool Trajectory::base_iterator<SegT>::operator>=(
    const base_iterator& other) const
{
  return (*this == other) || (*this > other);
}

//==============================================================================
template<typename SegT>
Trajectory::base_iterator<SegT>::operator const_iterator() const
{
  return _pimpl->make_iterator<const SegT>(_pimpl->raw_iterator);
}

//==============================================================================
template<typename SegT>
Trajectory::base_iterator<SegT>::base_iterator()
  : _pimpl(rmf_utils::make_impl<detail::TrajectoryIteratorImplementation>())
{
  // Do nothing
}

//==============================================================================
template class Trajectory::base_iterator<Trajectory::Segment>;
template class Trajectory::base_iterator<const Trajectory::Segment>;

//==============================================================================
bool Trajectory::Debug::check_iterator_time_consistency(
    const Trajectory& trajectory, const bool print_inconsistency)
{
  assert(trajectory._pimpl);

  const internal::SegmentList& segments = trajectory._pimpl->segments;
  const internal::OrderMap& ordering = trajectory._pimpl->ordering;

  bool consistent = true;

  internal::SegmentList::const_iterator s_it = segments.begin();
  internal::OrderMap::const_iterator o_it = ordering.begin();
  for( ; s_it != segments.end() && o_it != ordering.end(); ++s_it, ++o_it)
  {
    consistent &= s_it->data.finish_time == o_it->first;
  }

  consistent &= s_it == segments.end();
  consistent &= o_it == ordering.end();

  if(print_inconsistency && !consistent)
  {
    std::size_t index = 0;
    s_it = segments.begin();
    o_it = ordering.begin();
    std::cout << "Trajectory time inconsistency detected: "
              << "( ordering | segments | difference )\n";
    for( ; s_it != segments.end() && o_it != ordering.end();
         ++s_it, ++o_it, ++index)
    {
      const auto difference = o_it->first - s_it->data.finish_time;
      std::cout << " -- [" << index << "] "
                << o_it->first.time_since_epoch().count()/1e9 << " | "
                << s_it->data.finish_time.time_since_epoch().count()/1e9
                << " | Difference: " << difference.count()/1e9 << "\n";
    }

    if(s_it != segments.end())
    {
      std::cout << " -- more elements in segments\n";
      for( ; s_it != segments.end(); ++s_it, ++index)
      {
        std::cout << "      -- [" << index << "] "
                  << s_it->data.finish_time.time_since_epoch().count()/1e9
                  << "\n";
      }
    }
    if(o_it != ordering.end())
    {
      std::cout << " -- more elements in ordering:\n";
      for( ; o_it != ordering.end(); ++o_it, ++index)
      {
        std::cout << "     -- [" << index << "] "
                  << o_it->first.time_since_epoch().count()/1e9 << "\n";
      }
    }
    std::cout << std::endl;
  }

  return consistent;
}

} // namespace rmf_traffic
