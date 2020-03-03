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

#include "InconsistenciesInternal.hpp"
#include "../detail/internal_forward_iterator.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Inconsistencies::Ranges::IterImpl
{
public:

  RangesSet::const_iterator iter;

};

//==============================================================================
class Inconsistencies::Ranges::Implementation
{
public:

  Implementation(const RangesSet& set_)
  : set(set_)
  {
    // Do nothing
  }

  const RangesSet& set;
  ItineraryVersion last_known_version =
      std::numeric_limits<ItineraryVersion>::max();

  using raw_iterator = RangesSet::const_iterator;
  static const_iterator make_iterator(raw_iterator it)
  {
    const_iterator result;
    result._pimpl = rmf_utils::make_impl<const_iterator::Implementation>(
          const_iterator::Implementation{it});
    return result;
  }

  static Ranges make(const RangesSet& set)
  {
    Ranges result;
    result._pimpl = rmf_utils::make_unique_impl<Implementation>(set);
    return result;
  }

  static ItineraryVersion& get_last_known_version(Ranges& ranges)
  {
    return ranges._pimpl->last_known_version;
  }

};

//==============================================================================
auto Inconsistencies::Ranges::begin() const -> const_iterator
{
  return Implementation::make_iterator(_pimpl->set.begin());
}

//==============================================================================
auto Inconsistencies::Ranges::cbegin() const -> const_iterator
{
  return begin();
}

//==============================================================================
auto Inconsistencies::Ranges::end() const -> const_iterator
{
  return Implementation::make_iterator(_pimpl->set.end());
}

//==============================================================================
auto Inconsistencies::Ranges::cend() const -> const_iterator
{
  return end();
}

//==============================================================================
std::size_t Inconsistencies::Ranges::size() const
{
  return _pimpl->set.size();
}

//==============================================================================
ItineraryVersion Inconsistencies::Ranges::last_known_version() const
{
  return _pimpl->last_known_version;
}

//==============================================================================
std::unique_ptr<InconsistencyTracker>
Inconsistencies::Implementation::register_participant(
    Inconsistencies& inconsistencies,
    ParticipantId id)
{
  auto& _inconsistencies = inconsistencies._pimpl->_inconsistencies;
  auto& _api = inconsistencies._pimpl->_api;

  assert(_inconsistencies.find(id) == _inconsistencies.end());
  const auto it = _inconsistencies.insert(std::make_pair(id, RangesSet()));
  assert(it.second);
  RangesSet& ranges = it.first->second;
  const auto ranges_it = _api.insert(
        std::make_pair(
          id,
          Element{
            id,
            Ranges::Implementation::make(ranges)
          }));

  auto& ranges_api = ranges_it.first->second.ranges;
  return std::make_unique<InconsistencyTracker>(
        ranges,
        Ranges::Implementation::get_last_known_version(ranges_api));
}

//==============================================================================
void Inconsistencies::Implementation::unregister_participant(ParticipantId id)
{
  _inconsistencies.erase(id);
  _api.erase(id);
}

//==============================================================================
class Inconsistencies::IterImpl
{
public:

  struct Iter
  {
    ApiMap::const_iterator it;

    const Element& operator*() const
    {
      return it->second;
    }

    const Element* operator->() const
    {
      return &it->second;
    }

    Iter& operator++()
    {
      ++it;
      return *this;
    }

    Iter operator++(int)
    {
      Iter original{*this};
      ++it;
      return original;
    }

    bool operator==(const Iter& other) const
    {
      return it == other.it;
    }

    bool operator!=(const Iter& other) const
    {
      return it != other.it;
    }
  };

  Iter iter;

};

//==============================================================================
auto Inconsistencies::Implementation::make_iterator(raw_iterator it)
-> const_iterator
{
  const_iterator result;
  result._pimpl = rmf_utils::make_impl<const_iterator::Implementation>(
        const_iterator::Implementation{it});
  return result;
}

//==============================================================================
auto Inconsistencies::begin() const -> const_iterator
{
  return Implementation::make_iterator(_pimpl->_api.begin());
}

//==============================================================================
auto Inconsistencies::cbegin() const -> const_iterator
{
  return begin();
}

//==============================================================================
auto Inconsistencies::end() const -> const_iterator
{
  return Implementation::make_iterator(_pimpl->_api.end());
}

//==============================================================================
auto Inconsistencies::cend() const -> const_iterator
{
  return end();
}

//==============================================================================
auto Inconsistencies::find(const ParticipantId id) const -> const_iterator
{
  return Implementation::make_iterator(_pimpl->_api.find(id));
}

//==============================================================================
std::size_t Inconsistencies::size() const
{
  return _pimpl->_api.size();
}

//==============================================================================
Inconsistencies::Inconsistencies()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

} // namespace schedule

namespace detail {

//==============================================================================
template class forward_iterator<
    const schedule::Inconsistencies::Ranges::Range,
    schedule::Inconsistencies::Ranges::IterImpl,
    schedule::Inconsistencies::Ranges
>;

//==============================================================================
template class forward_iterator<
    const schedule::Inconsistencies::Element,
    schedule::Inconsistencies::IterImpl,
    schedule::Inconsistencies
>;

}

} // namespace rmf_traffic
