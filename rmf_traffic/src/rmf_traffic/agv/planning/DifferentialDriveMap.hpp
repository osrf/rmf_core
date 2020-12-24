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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEMAP_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEMAP_HPP

#include "Supergraph.hpp"

#include <memory>
#include <unordered_map>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
struct DifferentialDriveMapTypes
{
  struct Key
  {
    std::size_t start_lane;
    Orientation start_orientation;
    std::size_t goal_lane;
    Orientation goal_orientation;

    inline bool operator==(const Key& other) const
    {
      return
          start_lane == other.start_lane &&
          start_orientation == other.start_orientation &&
          goal_lane == other.goal_lane &&
          goal_orientation == other.goal_orientation;
    }
  };

  struct Node;
  using NodePtr = std::shared_ptr<const Node>;

  struct Node
  {
    std::size_t waypoint;
    double orientation;

    double remaining_cost_estimate;
    double current_cost;

    Graph::Lane::EventPtr event;
    std::vector<Route> route_from_parent;

    NodePtr parent;
  };

  struct Hash
  {
    Hash(std::size_t N_lanes)
    {
      _lane_shift = std::ceil(std::log2(N_lanes));
    }

    std::size_t operator()(const Key& key) const
    {
      return key.start_lane
          + (key.start_orientation << _lane_shift)
          + (key.goal_lane << (_lane_shift + 2))
          + (key.goal_orientation << 2*(_lane_shift + 2));
    }

  private:
    std::size_t _lane_shift;
  };
};

//==============================================================================
using DifferentialDriveMap =
  std::unordered_map<
    DifferentialDriveMapTypes::Key,
    DifferentialDriveMapTypes::NodePtr,
    DifferentialDriveMapTypes::Hash
  >;

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEMAP_HPP
