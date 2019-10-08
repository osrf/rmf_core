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

};

//==============================================================================
class Query::Spacetime::Implementation
{
public:

  Mode mode;
  All all_instance;
  Regions regions_instance;

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
Query::Spacetime::Regions::Regions(std::vector<Region> regions)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{std::move(regions)}))
{
  // Do nothing
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
  return Implementation::make_iterator(
        const_cast<Implementation::RegionSet&>(_pimpl->regions).begin());
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
        const_cast<Implementation::RegionSet&>(_pimpl->regions).begin());
}

//==============================================================================
auto Query::Spacetime::Regions::cend() const -> const_iterator
{
  return Implementation::make_iterator(
        const_cast<Implementation::RegionSet&>(_pimpl->regions).begin());
}

//==============================================================================
std::size_t Query::Spacetime::Regions::size() const
{
  return _pimpl->regions.size();
}

//==============================================================================
auto Query::Spacetime::query_regions(std::vector<Region> regions) -> Regions&
{
  _pimpl->mode = Mode::Regions;
  _pimpl->regions_instance = Regions(std::move(regions));
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
Query::Versions::After::After(std::size_t version)
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
std::size_t Query::Versions::After::get_version() const
{
  return _pimpl->after_version;
}

//==============================================================================
auto Query::Versions::After::set_version(std::size_t version) -> After&
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
Query::Versions::Versions(std::size_t version)
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
auto Query::Versions::query_after(std::size_t version) -> After&
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
Query make_query(std::size_t after_version)
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
    std::size_t after_version,
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
