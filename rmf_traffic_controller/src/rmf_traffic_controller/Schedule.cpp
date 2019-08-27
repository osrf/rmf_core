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

#include <rmf_traffic_controller/Schedule.hpp>
#include <rmf_traffic_controller/Trajectory.hpp>

#include <string>
#include <unordered_set>
#include <vector>

namespace rmf_traffic_controller {

//==============================================================================
template<typename T>
struct Range
{
  T lower;
  T upper;
};

//==============================================================================
struct ChronoSpatialBlock
{
  Range<uint64_t> time;
  Range<double> x;
  Range<double> y;

  std::unordered_set<const Trajectory*> trajectories;
};


//==============================================================================
class ChronoBucket
{
public:

  bool intersection(const ChronoSpatialBlock& test) const;

private:

  std::vector<std::shared_ptr<ChronoSpatialBlock>> blocks;

};

//==============================================================================
/// \brief This class performs very simple rectangular broadphase collision
/// detection over time and 2D space.
class ChronoSpatialBroadphase
{
public:



private:

  std::vector<ChronoBucket> buckets;

};

//==============================================================================
class Schedule::Implementation
{
public:



private:

  std::vector<std::unique_ptr<Trajectory>> trajectories;

  // Each "floor" (a.k.a. "map", a.k.a. "layout") has its own broadphase
  // instance.
  std::vector<ChronoSpatialBroadphase> broadphases;

};

//==============================================================================



} // namespace rmf_traffic_controller
