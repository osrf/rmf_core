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

#ifndef SRC__RMF_TRAFFIC__AGV__INTERNAL_GRAPH_HPP
#define SRC__RMF_TRAFFIC__AGV__INTERNAL_GRAPH_HPP

#include <rmf_traffic/agv/Graph.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph::Implementation
{
public:

  std::vector<Waypoint> waypoints;
  std::vector<Lane> lanes;
  std::unordered_map<std::string, std::size_t> keys;

  // A map from a waypoint index to the set of lanes that can exit from it
  std::vector<std::vector<std::size_t>> lanes_from;
  std::vector<std::unordered_map<std::size_t, std::size_t>> lane_between;

  static Graph::Implementation& get(Graph& graph)
  {
    return *graph._pimpl;
  }

  static const Graph::Implementation& get(const Graph& graph)
  {
    return *graph._pimpl;
  }

};

} // namespace agv
} // namespace rmf_traffic
#endif // SRC__RMF_TRAFFIC__AGV__INTERNAL_GRAPH_HPP
