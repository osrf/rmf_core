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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNINGINTERNAL_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNINGINTERNAL_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <memory>
#include <mutex>

namespace rmf_traffic {
namespace internal {
namespace planning {

class Cache;
using CachePtr = std::shared_ptr<Cache>;

//==============================================================================
struct Result
{
  std::vector<Route> routes;
  std::vector<agv::Plan::Waypoint> waypoints;

  agv::Planner::Start start;
  agv::Planner::Goal goal;
  agv::Planner::Options options;
};

//==============================================================================
class Cache
{
public:

  // TODO(MXG): Consider using libguarded instead of using mutexes manually
  std::mutex mutex;

  // We need to explicitly define these because the std::mutex does not have
  // a copy constructor
  Cache() = default;
  Cache(const Cache&);
  Cache(Cache&&);
  Cache& operator=(const Cache&);
  Cache& operator=(Cache&&);

  virtual CachePtr clone() const = 0;

  virtual void update(const Cache& other) = 0;

  virtual rmf_utils::optional<Result> plan(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) = 0;

  virtual const agv::Planner::Configuration& get_configuration() const = 0;

  virtual ~Cache() = default;
};

//==============================================================================
class CacheHandle
{
public:

  CacheHandle(CachePtr original);

  // Copying this class does not make sense
  CacheHandle(const CacheHandle&) = delete;

  // Moving it is okay
  CacheHandle(CacheHandle&&) = default;

  rmf_utils::optional<Result> plan(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options);

  ~CacheHandle();

private:

  CachePtr _copy;

  CachePtr _original;

};

//==============================================================================
class CacheManager
{
public:

  CacheManager(CachePtr cache);

  CacheHandle get() const;

  const agv::Planner::Configuration& get_configuration() const;

private:
  CachePtr _cache;

};

//==============================================================================
CacheManager make_cache(agv::Planner::Configuration config);

} // namespace planning
} // namespace internal
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNINGINTERNAL_HPP
