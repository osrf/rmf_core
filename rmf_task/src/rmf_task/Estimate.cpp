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

#include <unordered_map>
#include <mutex>

#include <rmf_task/Estimate.hpp>

namespace rmf_task {

//==============================================================================
class Estimate::Implementation
{
public:

  Implementation(agv::State finish_state, rmf_traffic::Time wait_until)
  : _finish_state(std::move(finish_state)),
    _wait_until(std::move(wait_until))
  {}

  agv::State _finish_state;
  rmf_traffic::Time _wait_until;
};

//==============================================================================
Estimate::Estimate(agv::State finish_state, rmf_traffic::Time wait_until)
: _pimpl(rmf_utils::make_impl<Implementation>(
    std::move(finish_state), std::move(wait_until)))
{}

//==============================================================================
agv::State Estimate::finish_state() const
{
  return _pimpl->_finish_state;
}

//==============================================================================
Estimate& Estimate::finish_state(agv::State new_finish_state)
{
  _pimpl->_finish_state = std::move(new_finish_state);
  return *this;
}

//==============================================================================
rmf_traffic::Time Estimate::wait_until() const
{
  return _pimpl->_wait_until;
}

//==============================================================================
Estimate& Estimate::wait_until(rmf_traffic::Time new_wait_until)
{
  _pimpl->_wait_until = std::move(new_wait_until);
  return *this;
}

//==============================================================================
class EstimateCache::Implementation
{
public:

  Implementation(std::size_t N)
  : _cache(N, PairHash(N))
  {

  }

  struct PairHash
  {
    PairHash(std::size_t N)
    {
      // We add 1 to N because
      _shift = std::ceil(std::log2(N+1));
    }

    size_t operator()(const std::pair<size_t,size_t>& p) const
    {
      return p.first + (p.second << _shift);
    }

    std::size_t _shift;
  };

  using Cache = std::unordered_map<std::pair<size_t,size_t>,
    CacheElement, PairHash>;

  Cache _cache;
  std::shared_ptr<std::mutex> _mutex = std::make_shared<std::mutex>();
};

//==============================================================================
EstimateCache::EstimateCache(std::size_t N)
  : _pimpl(rmf_utils::make_impl<Implementation>(Implementation(N)))
{}

//==============================================================================
std::optional<EstimateCache::CacheElement> EstimateCache::get(
  std::pair<size_t, size_t> waypoints) const
{
  std::lock_guard<std::mutex> guard(*_pimpl->_mutex);
  auto it = _pimpl->_cache.find(waypoints);
  if (it != _pimpl->_cache.end())
  {
    return it->second;
  }
  return std::nullopt;
}

//==============================================================================
void EstimateCache::set(std::pair<size_t, size_t> waypoints,
  rmf_traffic::Duration duration, double dsoc)
{
  std::lock_guard<std::mutex> guard(*_pimpl->_mutex);
  _pimpl->_cache[waypoints] = CacheElement{duration, dsoc};
}

} // namespace rmf_task
