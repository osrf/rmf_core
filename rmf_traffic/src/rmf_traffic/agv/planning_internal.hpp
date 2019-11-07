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
class Cache
{
public:

  // TODO(MXG): Consider using libguarded instead of using mutexes manually
  std::mutex mutex;

  virtual CachePtr clone() const = 0;

  virtual void update(const Cache& other) = 0;

  virtual agv::Plan plan(
      agv::Planner::Start start,
      agv::Planner::Goal goal,
      agv::Planner::Options options) = 0;

};

//==============================================================================
class CacheHandle
{
public:

  CacheHandle(CachePtr original)
    : _original(std::move(original))
  {
    std::unique_lock<std::mutex> lock(_original->mutex);
    _copy = original->clone();
  }

  agv::Plan plan(
      agv::Planner::Start start,
      agv::Planner::Goal goal,
      agv::Planner::Options options)
  {
    return _copy->plan(std::move(start), std::move(goal), std::move(options));
  }

  ~CacheHandle()
  {
    // We are now done with the copy, so we will update the original cache with
    // whatever new searches have been accomplished by the copy.
    std::unique_lock<std::mutex> lock(_original->mutex);
    _original->update(*_copy);
  }

private:

  CachePtr _copy;

  CachePtr _original;

};

//==============================================================================
class CacheManager
{
public:

  CacheManager(CachePtr cache)
    : _cache(std::move(cache))
  {
    // Do nothing
  }

  CacheHandle get() const
  {
    return CacheHandle(_cache);
  }

private:
  CachePtr _cache;

};

//==============================================================================
class DifferentialDriveCache : public Cache
{
public:

  DifferentialDriveCache(agv::Planner::Configuration config)
  {
    // Do nothing
  }

  CachePtr clone() const override final
  {
    // TODO(MXG): Fix this
    return nullptr;
  }

  void update(const Cache& newer_cache) override final
  {

  }

  agv::Plan plan(
      agv::Planner::Start start,
      agv::Planner::Goal goal,
      agv::Planner::Options options) override final
  {

  }


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

} // namespace planning
} // namespace internal
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNINGINTERNAL_HPP
