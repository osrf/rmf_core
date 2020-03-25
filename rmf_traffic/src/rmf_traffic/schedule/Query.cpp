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

#include <rmf_utils/optional.hpp>

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
  bool all_maps;

  rmf_utils::optional<Time> lower_bound;
  rmf_utils::optional<Time> upper_bound;

  static Timespan make(
      std::vector<std::string> maps,
      rmf_utils::optional<Time> lower_bound,
      rmf_utils::optional<Time> upper_bound)
  {
    Timespan span;
    span._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            std::unordered_set<std::string>{
              std::make_move_iterator(maps.begin()),
              std::make_move_iterator(maps.end())
            },
            false,
            lower_bound,
            upper_bound
          });

    return span;
  }

  static Timespan make(
      bool query_all_maps,
      rmf_utils::optional<Time> lower_bound,
      rmf_utils::optional<Time> upper_bound)
  {
    Timespan span;
    span._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            {},
            query_all_maps,
            lower_bound,
            upper_bound
          });

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
      timespan_instance(
        Timespan::Implementation::make(
          {}, rmf_utils::nullopt, rmf_utils::nullopt))
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
Query::Spacetime::Timespan::maps() const
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
auto Query::Spacetime::Timespan::clear_maps() -> Timespan&
{
  _pimpl->maps.clear();
  return *this;
}

//==============================================================================
bool Query::Spacetime::Timespan::all_maps() const
{
  return _pimpl->all_maps;
}

//==============================================================================
auto Query::Spacetime::Timespan::all_maps(bool query_all_maps) -> Timespan&
{
  _pimpl->all_maps = query_all_maps;
  return *this;
}

//==============================================================================
const Time* Query::Spacetime::Timespan::get_lower_time_bound() const
{
  if(_pimpl->lower_bound)
    return &(*_pimpl->lower_bound);

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::Timespan::set_lower_time_bound(Time time) -> Timespan&
{
  _pimpl->lower_bound = time;
  return *this;
}

//==============================================================================
auto Query::Spacetime::Timespan::remove_lower_time_bound() -> Timespan&
{
  _pimpl->lower_bound = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
const Time* Query::Spacetime::Timespan::get_upper_time_bound() const
{
  if(_pimpl->upper_bound)
    return &(*_pimpl->upper_bound);

  return nullptr;
}

//==============================================================================
auto Query::Spacetime::Timespan::set_upper_time_bound(Time time) -> Timespan&
{
  _pimpl->upper_bound = time;
  return *this;
}

//==============================================================================
auto Query::Spacetime::Timespan::remove_upper_time_bound() -> Timespan&
{
  _pimpl->upper_bound = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
Query::Spacetime::Timespan::Timespan()
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
        std::move(maps), lower_bound, upper_bound);

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
        std::move(maps), lower_bound, rmf_utils::nullopt);

  return _pimpl->timespan_instance;
}

//==============================================================================
auto Query::Spacetime::query_timespan(
    std::vector<std::string> maps) -> Timespan&
{
  _pimpl->mode = Mode::Timespan;
  _pimpl->timespan_instance =
      Timespan::Implementation::make(
        std::move(maps),
        rmf_utils::nullopt,
        rmf_utils::nullopt);

  return _pimpl->timespan_instance;
}

//==============================================================================
auto Query::Spacetime::query_timespan(bool query_all_maps) -> Timespan&
{
  _pimpl->mode = Mode::Timespan;
  _pimpl->timespan_instance =
      Timespan::Implementation::make(
        query_all_maps,
        rmf_utils::nullopt,
        rmf_utils::nullopt);

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
class Query::Participants::Implementation
{
public:

  Mode mode = Mode::All;
  All all;
  Include include;
  Exclude exclude;

};

//==============================================================================
class Query::Participants::All::Implementation
{
public:

  // This class is just a placeholder until we have any use for an All API

};

//==============================================================================
Query::Participants::All::All()
{
  // Do nothing
}

namespace {
//==============================================================================
std::vector<ParticipantId> uniquify(std::vector<ParticipantId> ids)
{
  std::unordered_set<ParticipantId> unique_ids;
  for (const auto id : ids)
    unique_ids.insert(id);

  ids.assign(unique_ids.begin(), unique_ids.end());
  return ids;
}
} // anonymous namespace

//==============================================================================
class Query::Participants::Include::Implementation
{
public:

  std::vector<ParticipantId> ids;

};

//==============================================================================
Query::Participants::Include::Include(std::vector<ParticipantId> ids)
: _pimpl(rmf_utils::make_impl<Implementation>(
           Implementation{uniquify(std::move(ids))}))
{
  // Do nothing
}

//==============================================================================
const std::vector<ParticipantId>& Query::Participants::Include::get_ids() const
{
  return _pimpl->ids;
}

//==============================================================================
auto Query::Participants::Include::set_ids(std::vector<ParticipantId> ids)
-> Include&
{
  _pimpl->ids = uniquify(std::move(ids));
  return *this;
}

//==============================================================================
Query::Participants::Include::Include()
{
  // Do nothing
}

//==============================================================================
class Query::Participants::Exclude::Implementation
{
public:

  std::vector<ParticipantId> ids;

};

//==============================================================================
Query::Participants::Exclude::Exclude(std::vector<ParticipantId> ids)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{uniquify(std::move(ids))}))
{
  // Do nothing
}

//==============================================================================
const std::vector<ParticipantId>& Query::Participants::Exclude::get_ids() const
{
  return _pimpl->ids;
}

//==============================================================================
auto Query::Participants::Exclude::set_ids(std::vector<ParticipantId> ids)
-> Exclude&
{
  _pimpl->ids = uniquify(std::move(ids));
  return *this;
}

//==============================================================================
Query::Participants::Exclude::Exclude()
{
  // Do nothing
}

//==============================================================================
Query::Participants::Participants()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
auto Query::Participants::make_all() -> Participants
{
  return Participants();
}

//==============================================================================
auto Query::Participants::make_only(std::vector<ParticipantId> ids)
-> Participants
{
  Participants participants;
  participants._pimpl->mode = Mode::Include;
  participants._pimpl->include._pimpl =
      rmf_utils::make_impl<Include::Implementation>();

  participants._pimpl->include.set_ids(std::move(ids));
  return participants;
}

//==============================================================================
auto Query::Participants::make_all_except(std::vector<ParticipantId> ids)
-> Participants
{
  Participants participants;
  participants._pimpl->mode = Mode::Exclude;
  participants._pimpl->exclude._pimpl =
      rmf_utils::make_impl<Exclude::Implementation>();

  participants._pimpl->exclude.set_ids(std::move(ids));
  return participants;
}

//==============================================================================
auto Query::Participants::get_mode() const -> Mode
{
  return _pimpl->mode;
}

//==============================================================================
auto Query::Participants::all() -> All*
{
  if (Mode::All == _pimpl->mode)
    return &_pimpl->all;

  return nullptr;
}

//==============================================================================
auto Query::Participants::all() const -> const All*
{
  return const_cast<Participants*>(this)->all();
}

//==============================================================================
auto Query::Participants::include() -> Include*
{
  if (Mode::Include == _pimpl->mode)
    return &_pimpl->include;

  return nullptr;
}

//==============================================================================
auto Query::Participants::include() const -> const Include*
{
  return const_cast<Participants*>(this)->include();
}

//==============================================================================
auto Query::Participants::include(std::vector<ParticipantId> ids)
-> Participants&
{
  *this = make_only(std::move(ids));
  return *this;
}

//==============================================================================
auto Query::Participants::exclude() -> Exclude*
{
  if (Mode::Exclude == _pimpl->mode)
    return &_pimpl->exclude;

  return nullptr;
}

//==============================================================================
auto Query::Participants::exclude() const -> const Exclude*
{
  return const_cast<Participants*>(this)->exclude();
}

//==============================================================================
auto Query::Participants::exclude(std::vector<ParticipantId> ids)
-> Participants&
{
  *this = make_all_except(std::move(ids));
  return *this;
}

//==============================================================================
class Query::Implementation
{
public:

  Spacetime spacetime_instance;
  Participants participants_instance;

  static Query query_all()
  {
    return Query();
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
auto Query::participants() -> Participants&
{
  return _pimpl->participants_instance;
}

//==============================================================================
auto Query::participants() const -> const Participants&
{
  return _pimpl->participants_instance;
}

//==============================================================================
Query::Query()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
Query query_all()
{
  return Query::Implementation::query_all();
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
