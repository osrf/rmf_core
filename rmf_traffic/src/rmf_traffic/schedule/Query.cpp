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

#include "../detail/internal_bidirectional_iterator.hpp"

#include <rmf_traffic/schedule/Query.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Query::Spacetime::Regions::IterImpl
{
public:

  std::vector<Region>::iterator iter;

};


//==============================================================================
class Query::Spacetime::Regions::Implementation
{
public:

  using RegionSet = std::vector<Region>;
  RegionSet regions;

  using raw_iterator = RegionSet::iterator;
  static iterator make_iterator(raw_iterator it)
  {
    return iterator(IterImpl{it});
  }

  static Regions make(std::vector<Region> input_regions)
  {
    Regions result;
    result._pimpl->regions = std::move(input_regions);
    return result;
  }

};

//==============================================================================
class Query::Spacetime::Timespan::Implementation
{
public:

  std::unordered_set<std::string> maps;

  // TODO(MXG): Replace with std::optional when we have C++17 support
  bool has_lower_bound = false;
  Time lower_bound;

  // TODO(MXG): Replace with std::optional when we have C++17 support
  bool has_upper_bound = false;
  Time upper_bound;

  static Timespan make(
      std::vector<std::string> maps,
      const Time* lower_bound,
      const Time* upper_bound)
  {
    Timespan span;
    span._pimpl->maps = std::unordered_set<std::string>{
          std::make_move_iterator(maps.begin()),
          std::make_move_iterator(maps.end())};

    span._pimpl->has_lower_bound = static_cast<bool>(lower_bound);
    if(lower_bound)
      span._pimpl->lower_bound = *lower_bound;

    span._pimpl->has_upper_bound = static_cast<bool>(upper_bound);
    if(upper_bound)
      span._pimpl->upper_bound = *upper_bound;

    return span;
  }
};

//==============================================================================
class Query::Spacetime::Implementation
{
public:

  Mode mode;
  All all_instance;
  Regions regions_instance;
  Timespan timespan_instance;

  // TODO(MXG): We can make this more efficient by leaving the pimpls of
  // regions_instance and timespan_instance uninitialized until they actually
  // get used.
  Implementation()
    : regions_instance(Regions::Implementation::make({})),
      timespan_instance(Timespan::Implementation::make({}, nullptr, nullptr))
  {
    // Do nothing
  }
};

//==============================================================================
Query::Spacetime::Spacetime()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_all();
}

//==============================================================================
Query::Spacetime::Spacetime(std::vector<Region> regions)
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_regions(std::move(regions));
}

//==============================================================================
Query::Spacetime::Spacetime(std::vector<std::string> maps)
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_timespan(std::move(maps));
}

//==============================================================================
Query::Spacetime::Spacetime(
    std::vector<std::string> maps,
    Time lower_bound)
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_timespan(std::move(maps), lower_bound);
}

//==============================================================================
Query::Spacetime::Spacetime(
    std::vector<std::string> maps,
    Time lower_bound,
    Time upper_bound)
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_timespan(std::move(maps), lower_bound, upper_bound);
}

//==============================================================================
auto Query::Spacetime::get_mode() const -> Mode
{
  return _pimpl->mode;
}

//==============================================================================
auto Query::Spacetime::query_all() -> All&
{
  _pimpl->mode = Mode::All;
  return _pimpl->all_instance;
}

//==============================================================================
void Query::Spacetime::Regions::push_back(Region region)
{
  _pimpl->regions.push_back(std::move(region));
}

//==============================================================================
void Query::Spacetime::Regions::pop_back()
{
  _pimpl->regions.pop_back();
}

//==============================================================================
auto Query::Spacetime::Regions::erase(iterator it) -> iterator
{
  return Implementation::make_iterator(_pimpl->regions.erase(it._pimpl->iter));
}

//==============================================================================
auto Query::Spacetime::Regions::erase(
    iterator first, iterator last) -> iterator
{
  return Implementation::make_iterator(
        _pimpl->regions.erase(first._pimpl->iter, last._pimpl->iter));
}

//==============================================================================
auto Query::Spacetime::Regions::begin() -> iterator
{
  return Implementation::make_iterator(_pimpl->regions.begin());
}

//==============================================================================
auto Query::Spacetime::Regions::begin() const -> const_iterator
{
  return Implementation::make_iterator(
        const_cast<Implementation::RegionSet&>(_pimpl->regions).begin());
}

//==============================================================================
auto Query::Spacetime::Regions::cbegin() const -> const_iterator
{
  return begin();
}

//==============================================================================
auto Query::Spacetime::Regions::end() -> iterator
{
  return Implementation::make_iterator(_pimpl->regions.end());
}

//==============================================================================
auto Query::Spacetime::Regions::end() const -> const_iterator
{
  return Implementation::make_iterator(
        const_cast<Implementation::RegionSet&>(_pimpl->regions).end());
}

//==============================================================================
auto Query::Spacetime::Regions::cend() const -> const_iterator
{
  return end();
}

//==============================================================================
std::size_t Query::Spacetime::Regions::size() const
{
  return _pimpl->regions.size();
}

//==============================================================================
Query::Spacetime::Regions::Regions()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
const std::unordered_set<std::string>&
Query::Spacetime::Timespan::get_maps() const
{
  return _pimpl->maps;
}

//==============================================================================
auto Query::Spacetime::Timespan::add_map(std::string map_name) -> Timespan&
{
  _pimpl->maps.insert(map_name);
  return *this;
}

//==============================================================================
auto Query::Spacetime::Timespan::remove_map(const std::string& map_name)
  -> Timespan&
{
  _pimpl->maps.erase(map_name);
  return *this;
}

//==============================================================================
const Time* Query::Spacetime::Timespan::get_lower_time_bound() const
{
  if(_pimpl->has_lower_bound)
    return &_pimpl->lower_bound;

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::Timespan::set_lower_time_bound(Time time) -> Timespan&
{
  _pimpl->has_lower_bound = true;
  _pimpl->lower_bound = time;
  return *this;
}

//==============================================================================
auto Query::Spacetime::Timespan::remove_lower_time_bound() -> Timespan&
{
  _pimpl->has_lower_bound = false;
  return *this;
}

//==============================================================================
const Time* Query::Spacetime::Timespan::get_upper_time_bound() const
{
  if(_pimpl->has_upper_bound)
    return &_pimpl->upper_bound;

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::Timespan::set_upper_time_bound(Time time) -> Timespan&
{
  _pimpl->has_upper_bound = true;
  _pimpl->upper_bound = time;
  return *this;
}

//==============================================================================
auto Query::Spacetime::Timespan::remove_upper_time_bound() -> Timespan&
{
  _pimpl->has_upper_bound = false;
  return *this;
}

//==============================================================================
Query::Spacetime::Timespan::Timespan()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Query::Spacetime::query_regions(std::vector<Region> regions) -> Regions&
{
  _pimpl->mode = Mode::Regions;
  _pimpl->regions_instance =
      Regions::Implementation::make(std::move(regions));
  return _pimpl->regions_instance;
}

//==============================================================================
auto Query::Spacetime::regions() -> Regions*
{
  if(Mode::Regions == _pimpl->mode)
    return &_pimpl->regions_instance;

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::regions() const -> const Regions*
{
  if(Mode::Regions == _pimpl->mode)
    return &_pimpl->regions_instance;

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::query_timespan(
    std::vector<std::string> maps,
    Time lower_bound,
    Time upper_bound) -> Timespan&
{
  _pimpl->mode = Mode::Timespan;
  _pimpl->timespan_instance =
      Timespan::Implementation::make(
        std::move(maps), &lower_bound, &upper_bound);

  return _pimpl->timespan_instance;
}

//==============================================================================
auto Query::Spacetime::query_timespan(
    std::vector<std::string> maps,
    Time lower_bound) -> Timespan&
{
  _pimpl->mode = Mode::Timespan;
  _pimpl->timespan_instance =
      Timespan::Implementation::make(
        std::move(maps), &lower_bound, nullptr);

  return _pimpl->timespan_instance;
}

//==============================================================================
auto Query::Spacetime::query_timespan(
    std::vector<std::string> maps) -> Timespan&
{
  _pimpl->mode = Mode::Timespan;
  _pimpl->timespan_instance =
      Timespan::Implementation::make(std::move(maps), nullptr, nullptr);

  return _pimpl->timespan_instance;
}

//==============================================================================
auto Query::Spacetime::timespan() -> Timespan*
{
  if(Mode::Timespan == _pimpl->mode)
    return &_pimpl->timespan_instance;

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::timespan() const -> const Timespan*
{
  if(Mode::Timespan == _pimpl->mode)
    return &_pimpl->timespan_instance;

  return nullptr;
}

//==============================================================================
class Query::Versions::All::Implementation
{
  // Empty placeholder class
};

//==============================================================================
class Query::Versions::After::Implementation
{
public:

  std::size_t after_version;

};

//==============================================================================
Query::Versions::After::After(Version version)
  : _pimpl(rmf_utils::make_impl<Implementation>(Implementation{version}))
{
  // Do nothing
}

//==============================================================================
Query::Versions::After::After()
{
  // Do nothing
}

//==============================================================================
class Query::Versions::Implementation
{
public:

  Mode mode;
  All all_instance;
  After after_instance;

};

//==============================================================================
Version Query::Versions::After::get_version() const
{
  return _pimpl->after_version;
}

//==============================================================================
auto Query::Versions::After::set_version(Version version) -> After&
{
  _pimpl->after_version = version;
  return *this;
}

//==============================================================================
Query::Versions::Versions()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_all();
}

//==============================================================================
Query::Versions::Versions(Version version)
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  query_after(version);
}

//==============================================================================
Query::Versions::Mode Query::Versions::get_mode() const
{
  return _pimpl->mode;
}

//==============================================================================
auto Query::Versions::query_all() -> All&
{
  _pimpl->mode = Mode::All;
  return _pimpl->all_instance;
}

//==============================================================================
auto Query::Versions::query_after(Version version) -> After&
{
  _pimpl->mode = Mode::After;
  if(_pimpl->after_instance._pimpl)
    _pimpl->after_instance.set_version(version);
  else
    _pimpl->after_instance = After(version);

  return _pimpl->after_instance;
}

//==============================================================================
auto Query::Versions::after() -> After*
{
  if(Mode::After == _pimpl->mode)
    return &_pimpl->after_instance;

  return nullptr;
}

//==============================================================================
auto Query::Versions::after() const -> const After*
{
  if(Mode::After == _pimpl->mode)
    return &_pimpl->after_instance;

  return nullptr;
}

//==============================================================================
class Query::Implementation
{
public:

  Spacetime spacetime_instance;
  Versions versions_instance;

  static Query query_everything()
  {
    return Query();
  }

  static Query make_query(std::size_t after_version)
  {
    Query result;
    result.versions().query_after(after_version);
    return result;
  }

  static Query make_query(
      std::vector<Region> regions)
  {
    Query result;
    result.spacetime().query_regions(std::move(regions));
    return result;
  }

  static Query make_query(
      std::vector<std::string> maps,
      const Time* start_time,
      const Time* finish_time)
  {
    Query result;
    result.spacetime().query_timespan(std::move(maps));
    auto& timespan = *result.spacetime().timespan();
    if(start_time)
      timespan.set_lower_time_bound(*start_time);

    if(finish_time)
      timespan.set_upper_time_bound(*finish_time);

    return result;
  }

  static Query make_query(
      std::size_t after_version,
      std::vector<Region> regions)
  {
    Query result;
    result.versions().query_after(after_version);
    result.spacetime().query_regions(std::move(regions));
    return result;
  }

};

//==============================================================================
auto Query::spacetime() -> Spacetime&
{
  return _pimpl->spacetime_instance;
}

//==============================================================================
auto Query::spacetime() const -> const Spacetime&
{
  return _pimpl->spacetime_instance;
}

//==============================================================================
auto Query::versions() -> Versions&
{
  return _pimpl->versions_instance;
}

//==============================================================================
auto Query::versions() const -> const Versions&
{
  return _pimpl->versions_instance;
}

//==============================================================================
Query::Query()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Query query_everything()
{
  return Query::Implementation::query_everything();
}

//==============================================================================
Query make_query(Version after_version)
{
  return Query::Implementation::make_query(after_version);
}

//==============================================================================
Query make_query(std::vector<Region> regions)
{
  return Query::Implementation::make_query(std::move(regions));
}

//==============================================================================
Query make_query(
    std::vector<std::string> maps,
    const Time* start_time,
    const Time* finish_time)
{
  return Query::Implementation::make_query(
        std::move(maps), start_time, finish_time);
}

//==============================================================================
Query make_query(
    Version after_version,
    std::vector<Region> regions)
{
  return Query::Implementation::make_query(after_version, std::move(regions));
}

} // namespace schedule

namespace detail {

//==============================================================================
template class bidirectional_iterator<
    Region,
    schedule::Query::Spacetime::Regions::IterImpl,
    schedule::Query::Spacetime::Regions
>;

//==============================================================================
template class bidirectional_iterator<
    const Region,
    schedule::Query::Spacetime::Regions::IterImpl,
    schedule::Query::Spacetime::Regions
>;

} // namespace detail

} // namespace rmf_traffic
