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

#include <rmf_traffic/Region.hpp>

#include "detail/internal_bidirectional_iterator.hpp"

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {

//==============================================================================
class Region::IterImpl
{
public:

  std::vector<geometry::Space>::iterator iter;

};

//==============================================================================
class Region::Implementation
{
public:

  std::string map;
  rmf_utils::optional<Time> lower_bound;
  rmf_utils::optional<Time> upper_bound;

  using Spaces = std::vector<geometry::Space>;
  Spaces spaces;

  using raw_iterator = Spaces::iterator;
  static iterator make_iterator(raw_iterator it)
  {
    iterator result;
    result._pimpl = rmf_utils::make_impl<iterator::Implementation>(
          iterator::Implementation{it});
    return result;
  }

};

//==============================================================================
Region::Region(
    std::string map,
    Time lower_bound,
    Time upper_bound,
    std::vector<geometry::Space> spaces)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(map),
               lower_bound,
               upper_bound,
               std::move(spaces)}))
{
  // Do nothing
}

//==============================================================================
Region::Region(
    std::string map,
    std::vector<Space> spaces)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(map),
               rmf_utils::nullopt,
               rmf_utils::nullopt,
               std::move(spaces)}))
{
  // Do nothing
}

//==============================================================================
const std::string& Region::get_map() const
{
  return _pimpl->map;
}

//==============================================================================
auto Region::set_map(std::string map) -> Region&
{
  _pimpl->map = std::move(map);
  return *this;
}

//==============================================================================
const Time* Region::get_lower_time_bound() const
{
  if(_pimpl->lower_bound)
    return &(*_pimpl->lower_bound);

  return nullptr;
}

//==============================================================================
auto Region::set_lower_time_bound(Time time) -> Region&
{
  _pimpl->lower_bound = time;
  return *this;
}

//==============================================================================
auto Region::remove_lower_time_bound() -> Region&
{
  _pimpl->lower_bound = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
const Time* Region::get_upper_time_bound() const
{
  if(_pimpl->upper_bound)
    return &(*_pimpl->upper_bound);

  return nullptr;
}

//==============================================================================
auto Region::set_upper_time_bound(Time time) -> Region&
{
  _pimpl->upper_bound = time;
  return *this;
}

//==============================================================================
auto Region::remove_upper_time_bound() -> Region&
{
  _pimpl->upper_bound = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
void Region::push_back(geometry::Space space)
{
  _pimpl->spaces.push_back(space);
}

//==============================================================================
void Region::pop_back()
{
  _pimpl->spaces.pop_back();
}

//==============================================================================
auto Region::erase(iterator it) -> iterator
{
  return Implementation::make_iterator(
        _pimpl->spaces.erase(it._pimpl->iter));
}

//==============================================================================
auto Region::erase(iterator first, iterator last) -> iterator
{
  return Implementation::make_iterator(
        _pimpl->spaces.erase(first._pimpl->iter, last._pimpl->iter));
}

//==============================================================================
auto Region::begin() -> iterator
{
  return Implementation::make_iterator(_pimpl->spaces.begin());
}

//==============================================================================
auto Region::begin() const -> const_iterator
{
  return Implementation::make_iterator(
        const_cast<Implementation::Spaces&>(_pimpl->spaces).begin());
}

//==============================================================================
auto Region::cbegin() const -> const_iterator
{
  return begin();
}

//==============================================================================
auto Region::end() -> iterator
{
  return Implementation::make_iterator(_pimpl->spaces.end());
}

//==============================================================================
auto Region::end() const -> const_iterator
{
  return Implementation::make_iterator(
        const_cast<Implementation::Spaces&>(_pimpl->spaces).end());
}

//==============================================================================
auto Region::cend() const -> const_iterator
{
  return end();
}

//==============================================================================
std::size_t Region::num_spaces() const
{
  return _pimpl->spaces.size();
}

namespace detail {

//==============================================================================
template class bidirectional_iterator<
    geometry::Space, Region::IterImpl, Region
>;

//==============================================================================
template class bidirectional_iterator<
    const geometry::Space, Region::IterImpl, Region
>;

} // namespace detail

} // namespace rmf_traffic
