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

#ifndef SRC__RMF_TRAFFIC__AGV__SUPERGRAPH_HPP
#define SRC__RMF_TRAFFIC__AGV__SUPERGRAPH_HPP

#include "internal_Graph.hpp"
#include <rmf_traffic/Trajectory.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// A Supergraph is derived from a regular Graph. It analyzes the vertices and
/// edges of a regular Graph and adds in new edges and extra information that
/// may be useful for planners. The underlying graph cannot be changed without
/// reconstructing the Supergraph instance.
class Supergraph
{
public:

  Supergraph(Graph::Implementation original);

  const Graph::Implementation& original() const;

  struct FloorChange
  {
    std::size_t lane;
  };
  using FloorChanges = std::vector<FloorChange>;
  using DestinationFloorMap = std::unordered_map<std::string, FloorChanges>;
  using FloorChangeMap = std::unordered_map<std::string, DestinationFloorMap>;
  const FloorChangeMap& floor_change() const;

  // TODO(MXG): Consider having a reachability map in this class

private:
  Graph::Implementation _original;
  FloorChangeMap _floor_changes;
};

} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__SUPERGRAPH_HPP
