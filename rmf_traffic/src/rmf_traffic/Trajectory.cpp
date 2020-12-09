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

namespace internal {
//==============================================================================
class TrajectoryIteratorImplementation
{
public:

  WaypointList::iterator raw_iterator;
  const Trajectory::Implementation* parent;

  template<typename SegT>
  Trajectory::base_iterator<SegT> make_iterator(
    internal::WaypointList::iterator it) const
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

  static WaypointList::const_iterator raw(
    const Trajectory::const_iterator& iterator)
  {
    return iterator._pimpl->raw_iterator;
  }
};

//==============================================================================
WaypointList::const_iterator get_raw_iterator(
  const Trajectory::const_iterator& iterator)
{
  return TrajectoryIteratorImplementation::raw(iterator);
}

} // namespace internal

//==============================================================================
class Trajectory::Waypoint::Implementation
{
public:

  // Note: these fields will be filled in by the
  // Trajectory::Implementation::insert() function.
  internal::WaypointList::iterator myself;
  Trajectory::Implementation* parent;

  internal::WaypointElement::Data& data()
  {
    return myself->data;
  }

  const internal::WaypointElement::Data& data() const
  {
    return myself->data;
  }

  Time time() const
  {
    return data().time;
  }

};

//==============================================================================
class Trajectory::Implementation
{
public:

  internal::OrderMap ordering;
  internal::WaypointList segments;

  template<typename SegT>
  base_iterator<SegT> make_iterator(
    internal::WaypointList::iterator iterator) const
  {
    base_iterator<SegT> it;
    it._pimpl->raw_iterator = std::move(iterator);
    it._pimpl->parent = this;

    return it;
  }

  std::unique_ptr<Waypoint> make_segment(
    internal::WaypointList::iterator iterator)
  {
    std::unique_ptr<Waypoint> seg(new Waypoint);
    seg->_pimpl->myself = std::move(iterator);
    seg->_pimpl->parent = this;

    return seg;
  }

  Implementation()
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
    ordering = other.ordering;
    segments = other.segments;

    // Now correct all the iterators to point to the freshly copied container
    internal::WaypointList::iterator sit = segments.begin();
    internal::OrderMap::iterator oit = ordering.begin();
    for ( ; sit != segments.end(); ++sit, ++oit)
    {
      sit->myself = make_segment(sit);
      oit->value = sit;
    }

    return *this;
  }

  InsertionResult insert(internal::WaypointElement::Data data)
  {
    const internal::OrderMap::iterator hint = ordering.lower_bound(data.time);
    if (hint != ordering.end() && hint->key == data.time)
    {
      // We already have a Waypoint in the Trajectory that ends at this same
      // exact moment in time, so we will return the existing iterator along
      // with inserted==false.
      assert(segments.size() > 0);
      return InsertionResult{make_iterator<Waypoint>(hint->value), false};
    }

    const internal::WaypointList::const_iterator list_destination =
      (hint == ordering.end()) ? segments.end() : hint->value;

    const internal::WaypointList::iterator result =
      segments.emplace(list_destination, std::move(data));
    result->myself = make_segment(result);
    assert(segments.size() > 0);

    ordering.emplace_hint(hint, data.time, result);
    assert(ordering.size() > 0);

    return InsertionResult{make_iterator<Waypoint>(result), true};
  }

  iterator find(Time time)
  {
    const auto it = ordering.lower_bound(time);
    if (it == ordering.end())
      return make_iterator<Waypoint>(segments.end());

    // If the time comes before the start of the Trajectory, then we return
    // the end() iterator
    if (time < segments.begin()->data.time)
      return make_iterator<Waypoint>(segments.end());

    return make_iterator<Waypoint>(it->value);
  }

  iterator lower_bound(Time time)
  {
    const auto it = ordering.lower_bound(time);
    if (it == ordering.end())
      return make_iterator<Waypoint>(segments.end());

    return make_iterator<Waypoint>(it->value);
  }

  iterator erase(iterator waypoint)
  {
    ordering.erase(waypoint->_pimpl->myself->data.time);
    return make_iterator<Waypoint>(segments.erase(waypoint->_pimpl->myself));
  }

  iterator erase(iterator first, iterator last)
  {
    const auto seg_begin = first->_pimpl->myself;
    const auto seg_end = last._pimpl->raw_iterator == segments.end() ?
      segments.end() : last->_pimpl->myself;

    const auto order_start = ordering.find(seg_begin->data.time);
    const auto order_end = seg_end == segments.end() ?
      ordering.end() : ordering.find(seg_end->data.time);

    ordering.erase(order_start, order_end);
    return make_iterator<Waypoint>(segments.erase(seg_begin, seg_end));
  }

  iterator begin()
  {
    return make_iterator<Waypoint>(segments.begin());
  }

  iterator end()
  {
    return make_iterator<Waypoint>(segments.end());
  }

};

//==============================================================================
Eigen::Vector3d Trajectory::Waypoint::position() const
{
  return _pimpl->data().position;
}

//==============================================================================
Trajectory::Waypoint& Trajectory::Waypoint::position(
  Eigen::Vector3d new_position)
{
  _pimpl->data().position = std::move(new_position);
  return *this;
}

//==============================================================================
Eigen::Vector3d Trajectory::Waypoint::velocity() const
{
  return _pimpl->data().velocity;
}

//==============================================================================
Trajectory::Waypoint& Trajectory::Waypoint::velocity(
  Eigen::Vector3d new_velocity)
{
  _pimpl->data().velocity = std::move(new_velocity);
  return *this;
}

//==============================================================================
Time Trajectory::Waypoint::time() const
{
  return _pimpl->time();
}

//==============================================================================
Trajectory::Waypoint& Trajectory::Waypoint::change_time(const Time new_time)
{
  internal::WaypointList::iterator data_it = _pimpl->myself;
  internal::WaypointElement::Data& current_data = data_it->data;
  const Time current_time = current_data.time;

  if (current_time == new_time)
  {
    // Short-circuit, since nothing is changing. The erase and re-insertion
    // would be a waste of time in this case.
    return *this;
  }

  internal::OrderMap& ordering = _pimpl->parent->ordering;
  internal::WaypointList& segments = _pimpl->parent->segments;
  const internal::OrderMap::const_iterator current_order_it =
    ordering.find(current_time);
  assert(current_order_it != ordering.end());

  const internal::OrderMap::iterator hint =
    ordering.lower_bound(new_time);

  if (current_order_it == hint)
  {
    // The Waypoint is already in the correct location within the list, so it
    // does not need to be moved. We can just update its entry in the OrderMap.

    // We need to create a new_hint iterator which points to the iterator after
    // hint because the hint iterator will be invalidated when we erase
    // current_order_it (because they are iterators to the same element).
    const internal::OrderMap::iterator new_hint =
      ++internal::OrderMap::iterator(hint);
    ordering.erase(current_order_it);
    ordering.emplace_hint(new_hint, new_time, std::move(data_it));
  }
  else if (hint == ordering.end())
  {
    // This Waypoint must be moved to the end of the list.
    segments.splice(segments.end(), segments, data_it);
    ordering.erase(current_order_it);
    ordering.emplace_hint(hint, new_time, std::move(data_it));
  }
  else
  {
    const internal::WaypointList::const_iterator destination = hint->value;
    assert(destination != segments.end());

    if (destination->data.time == new_time)
    {
      // The new time conflicts with an existing time, so we will throw an
      // exception.
      // *INDENT-OFF*
      throw std::invalid_argument(
        "[Trajectory::Waypoint::change_time] Attempted to set time to "
        + std::to_string(new_time.time_since_epoch().count())
        + "ns, but a waypoint already exists at that timestamp.");
      // *INDENT-ON*
    }

    segments.splice(destination, segments, data_it);
    ordering.erase(current_order_it);
    ordering.emplace_hint(hint, new_time, std::move(data_it));
  }

  // Update the time value in the data field.
  current_data.time = new_time;

  return *this;
}

//==============================================================================
void Trajectory::Waypoint::adjust_times(Duration delta_t)
{
  internal::WaypointList& segments = _pimpl->parent->segments;
  const internal::WaypointList::iterator begin_it = _pimpl->myself;
  const Time original_begin_time = begin_it->data.time;

  if (delta_t.count() < 0 && begin_it != segments.begin())
  {
    // If delta_t is negative and this is not the first Waypoint in the
    // Trajectory, make sure the change in time does not make it dip beneath its
    // predecessor Waypoint.
    const internal::WaypointList::const_iterator predecessor_it =
      --internal::WaypointList::iterator(begin_it);
    const auto new_time = begin_it->data.time + delta_t;
    if (new_time <= predecessor_it->data.time)
    {
      const auto tp = predecessor_it->data.time
        .time_since_epoch().count();
      const auto tc = (new_time).time_since_epoch().count();

      const std::string error =
        std::string("[Trajectory::Waypoint::adjust_times] ")
        + "The given negative change in time: "
        + std::to_string(delta_t.count()) + "ns caused the Waypoint's new "
        + "time window [" + std::to_string(tc)
        + "] to overlap with its precedessor's [" + std::to_string(tp)
        + "]";

      throw std::invalid_argument(error);
    }
  }

  // Adjust the times for the segments and collect their iterators
  std::vector<internal::WaypointList::iterator> list_iterators;
  list_iterators.reserve(segments.size());
  for (internal::WaypointList::iterator it = begin_it; it != segments.end();
    ++it)
  {
    it->data.time += delta_t;
    list_iterators.push_back(it);
  }

  internal::OrderMap& ordering = _pimpl->parent->ordering;
  const internal::OrderMap::iterator order_it =
    ordering.find(original_begin_time);
  assert(order_it != ordering.end());

  // Erase the existing ordering entries for all the modified Trajectory
  // Waypoints.
  ordering.erase(order_it, ordering.end());

  // Add new entries one at a time, supplying the emplacement operator with the
  // hint that it can always append the entry to the end of the map.
  for (internal::WaypointList::iterator& it : list_iterators)
  {
    const Time new_time = it->data.time;
    ordering.emplace_hint(ordering.end(), new_time, std::move(it));
  }
}

//==============================================================================
Trajectory::Waypoint::Waypoint()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Trajectory::Trajectory()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
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
Trajectory::InsertionResult Trajectory::insert(
  Time time,
  Eigen::Vector3d position,
  Eigen::Vector3d velocity)
{
  return _pimpl->insert(
    internal::WaypointElement::Data{
      std::move(time),
      std::move(position),
      std::move(velocity)});
}

//==============================================================================
Trajectory::InsertionResult Trajectory::insert(const Waypoint& other)
{
  return _pimpl->insert(internal::WaypointElement::Data{other._pimpl->data()});
}

//==============================================================================
Trajectory::iterator Trajectory::find(Time time)
{
  return _pimpl->find(time);
}

//==============================================================================
Trajectory::Waypoint& Trajectory::operator[](const std::size_t index)
{
  return *_pimpl->ordering[index].value->myself;
}

//==============================================================================
const Trajectory::Waypoint& Trajectory::operator[](
    const std::size_t index) const
{
  return *_pimpl->ordering[index].value->myself;
}

//==============================================================================
Trajectory::Waypoint& Trajectory::at(const std::size_t index)
{
  return *_pimpl->ordering.at(index).value->myself;
}

//==============================================================================
const Trajectory::Waypoint& Trajectory::at(const std::size_t index) const
{
  return *_pimpl->ordering.at(index).value->myself;
}

//==============================================================================
Trajectory::const_iterator Trajectory::find(Time time) const
{
  return const_cast<Implementation&>(*_pimpl).find(time);
}

//==============================================================================
Trajectory::iterator Trajectory::lower_bound(Time time)
{
  return _pimpl->lower_bound(time);
}

//==============================================================================
Trajectory::const_iterator Trajectory::lower_bound(Time time) const
{
  return const_cast<Implementation&>(*_pimpl).lower_bound(time);
}

//==============================================================================
Trajectory::iterator Trajectory::erase(iterator waypoint)
{
  return _pimpl->erase(waypoint);
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
auto Trajectory::front() -> Waypoint&
{
  return *_pimpl->segments.front().myself;
}

//==============================================================================
auto Trajectory::front() const -> const Waypoint&
{
  return *_pimpl->segments.front().myself;
}

//==============================================================================
auto Trajectory::back() -> Waypoint&
{
  return *_pimpl->segments.back().myself;
}

//==============================================================================
auto Trajectory::back() const -> const Waypoint&
{
  return *_pimpl->segments.back().myself;
}

//==============================================================================
const Time* Trajectory::start_time() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() == 0 ? nullptr : &segments.front().data.time;
}

//==============================================================================
const Time* Trajectory::finish_time() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() == 0 ? nullptr : &segments.back().data.time;
}

//==============================================================================
Duration Trajectory::duration() const
{
  const auto& segments = _pimpl->segments;
  return segments.size() < 2 ?
    Duration(0) :
    segments.back().data.time - segments.front().data.time;
}

//==============================================================================
std::size_t Trajectory::size() const
{
  return _pimpl->segments.size();
}

//==============================================================================
bool Trajectory::empty() const
{
  return _pimpl->segments.empty();
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
  bool Trajectory::base_iterator<SegT>::operator op( \
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

  if (this_is_end || other_is_end)
  {
    // If the other is the end iterator but this iterator is not, then we return
    // true, because the end iterator is "larger" than any valid iterator.
    // If the other is not the end iterator but this iterator is, then we return
    // false, because this iterator is definitely larger than anything the other
    // can be.
    return other_is_end && !this_is_end;
  }

  // If they are both valid iterators, then we can compare their times.
  return this->_pimpl->raw_iterator->data.time
    < other._pimpl->raw_iterator->data.time;
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

  if (this_is_end || other_is_end)
  {
    // See the logic above for operator<, but in this case we want the opposite
    // conclusion.
    return this_is_end && !other_is_end;
  }

  // If they are both valid iterators, then we can compare their times.
  return this->_pimpl->raw_iterator->data.time
    > other._pimpl->raw_iterator->data.time;
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
: _pimpl(rmf_utils::make_impl<internal::TrajectoryIteratorImplementation>())
{
  // Do nothing
}

//==============================================================================
template class Trajectory::base_iterator<Trajectory::Waypoint>;
template class Trajectory::base_iterator<const Trajectory::Waypoint>;

//==============================================================================
bool Trajectory::Debug::check_iterator_time_consistency(
  const Trajectory& trajectory, const bool print_inconsistency)
{
  assert(trajectory._pimpl);

  const internal::WaypointList& segments = trajectory._pimpl->segments;
  const internal::OrderMap& ordering = trajectory._pimpl->ordering;

  bool consistent = true;

  internal::WaypointList::const_iterator s_it = segments.begin();
  internal::OrderMap::const_iterator o_it = ordering.begin();
  for ( ; s_it != segments.end() && o_it != ordering.end(); ++s_it, ++o_it)
  {
    consistent &= s_it->data.time == o_it->key;
  }

  consistent &= s_it == segments.end();
  consistent &= o_it == ordering.end();

  if (print_inconsistency && !consistent)
  {
    std::size_t index = 0;
    s_it = segments.begin();
    o_it = ordering.begin();
    std::cout << "Trajectory time inconsistency detected: "
              << "( ordering | segments | difference )\n";
    for ( ; s_it != segments.end() && o_it != ordering.end();
      ++s_it, ++o_it, ++index)
    {
      const auto difference = o_it->key - s_it->data.time;
      std::cout << " -- [" << index << "] "
                << o_it->key.time_since_epoch().count()/1e9 << " | "
                << s_it->data.time.time_since_epoch().count()/1e9
                << " | Difference: " << difference.count()/1e9 << "\n";
    }

    if (s_it != segments.end())
    {
      std::cout << " -- more elements in segments\n";
      for ( ; s_it != segments.end(); ++s_it, ++index)
      {
        std::cout << "      -- [" << index << "] "
                  << s_it->data.time.time_since_epoch().count()/1e9
                  << "\n";
      }
    }
    if (o_it != ordering.end())
    {
      std::cout << " -- more elements in ordering:\n";
      for ( ; o_it != ordering.end(); ++o_it, ++index)
      {
        std::cout << "     -- [" << index << "] "
                  << o_it->key.time_since_epoch().count()/1e9 << "\n";
      }
    }
    std::cout << std::endl;
  }

  return consistent;
}

} // namespace rmf_traffic
