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

#include "internal_planning.hpp"
#include "GraphInternal.hpp"

#include <map>
#include <unordered_map>

namespace rmf_traffic {
namespace internal {
namespace planning {

//==============================================================================
Cache::Cache(const Cache&)
{
  // Do nothing
}

//==============================================================================
Cache::Cache(Cache&&)
{
  // Do nothing
}

//==============================================================================
Cache& Cache::operator=(const Cache&)
{
  // Do nothing
  return *this;
}

//==============================================================================
Cache& Cache::operator=(Cache&&)
{
  // Do nothing
  return *this;
}

//==============================================================================
CacheHandle::CacheHandle(CachePtr original)
  : _original(std::move(original))
{
  std::unique_lock<std::mutex> lock(_original->mutex);
  _copy = original->clone();
}

//==============================================================================
Result CacheHandle::plan(
    agv::Planner::Start start,
    agv::Planner::Goal goal,
    agv::Planner::Options options)
{
  return _copy->plan(std::move(start), std::move(goal), std::move(options));
}

//==============================================================================
CacheHandle::~CacheHandle()
{
  // We are now done with the copy, so we will update the original cache with
  // whatever new searches have been accomplished by the copy.
  std::unique_lock<std::mutex> lock(_original->mutex);
  _original->update(*_copy);
}

//==============================================================================
CacheManager::CacheManager(CachePtr cache)
  : _cache(std::move(cache))
{
  // Do nothing
}

//==============================================================================
CacheHandle CacheManager::get() const
{
  return CacheHandle(_cache);
}

//==============================================================================
struct DifferentialDriverExpander
{
  struct Node
  {
    std::size_t waypoint;
    double orientation;
  };

  class Heuristic
  {
  public:

    double estimate_remaining_cost(const Node& node)
    {
      auto wp_costs = *known_costs.insert(
            std::make_pair(node.waypoint, OrientationToCostMap{})).first;

      const auto insertion = wp_costs.second.insert(
            std::make_pair(node.orientation, ))
    }

    void update(const Heuristic& other)
    {
      for(const auto& wp_costs : other.known_costs)
      {
        OrientationToCostMap& wp_it = known_costs.insert(
              std::make_pair(wp_costs.first, OrientationToCostMap{}))
            .first->second;

        for(const auto& cost : wp_costs.second)
          wp_it.insert(cost);
      }
    }

  private:

    using OrientationToCostMap = std::map<double, double>;
    std::unordered_map<std::size_t, OrientationToCostMap> known_costs;
  };

  struct Context
  {
    const agv::Graph::Implementation& graph;
    const std::size_t final_waypoint;
    const double* const final_orientation;
  };
};

//==============================================================================
class DifferentialDriveCache : public Cache
{
public:

  using Heuristic = DifferentialDriverExpander::Heuristic;

  DifferentialDriveCache(agv::Planner::Configuration config)
  {
    // Do nothing
  }

  CachePtr clone() const override final
  {
    return std::make_shared<DifferentialDriveCache>(*this);
  }

  void update(const Cache& newer_cache) override final
  {
    const auto& newer = static_cast<const DifferentialDriveCache&>(newer_cache);

    for(const auto& h : newer.heuristics)
    {
      auto& heuristic =
          heuristics.insert(std::make_pair(h.first, Heuristic{})).first->second;

      heuristic.update(h.second);
    }
  }

  Result plan(
      agv::Planner::Start start,
      agv::Planner::Goal goal,
      agv::Planner::Options options) override final
  {

  }

private:

  // This maps from a goal waypoint to the cached Heuristic object that tries to
  // plan to that goal waypoint.
  using HeuristicDatabase = std::unordered_map<std::size_t, Heuristic>;
  HeuristicDatabase heuristics;
};

//==============================================================================
CacheManager make_cache(agv::Planner::Configuration config)
{
  if(config.vehicle_traits().get_differential())
  {
    return CacheManager(std::make_shared<DifferentialDriveCache>(
                          std::move(config)));
  }

  throw std::runtime_error(
        "[rmf_traffic::agv::Planner] Planning utilities are currently only "
        "implemented for AGVs that use a differential drive.");
}

//==============================================================================


} // namespace planning
} // namespace internal
} // namespace rmf_traffic
