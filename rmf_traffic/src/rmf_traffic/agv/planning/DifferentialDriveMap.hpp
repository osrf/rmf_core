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

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
enum Orientation
{
  Forward = 0,
  Backward,
  Any
};

//==============================================================================
struct DifferentialDriveMapTypes
{
  // TODO(MXG): Consider refactoring Key into two Entry fields
  struct Key
  {
    std::size_t start_lane;
    Orientation start_orientation;
    std::size_t goal_lane;
    Orientation goal_orientation;

    inline bool operator==(const Key& other) const
    {
      if (goal_orientation == Orientation::Any)
      {
        return
            start_lane == other.start_lane &&
            start_orientation == other.start_orientation &&
            goal_orientation == other.goal_orientation;
      }

      return
          start_lane == other.start_lane &&
          start_orientation == other.start_orientation &&
          goal_lane == other.goal_lane &&
          goal_orientation == other.goal_orientation;
    }
  };

  struct Entry
  {
    std::size_t lane;
    Orientation orientation;

    inline bool operator==(const Entry& other) const
    {
      return
          lane == other.lane &&
          orientation == other.orientation;
    }
  };

  struct SolutionNode;
  using SolutionNodePtr = std::shared_ptr<const SolutionNode>;

  struct NodeInfo
  {
    // The entry field may be a nullopt if it is an intermediate node
    // e.g. a node that comes before or after an event to represent a waiting
    // period
    std::optional<Entry> entry;

    std::size_t waypoint;
    Eigen::Vector2d position;
    double yaw;

    double remaining_cost_estimate;
    double cost_from_parent;

    // An event that should occur when this node is reached,
    // i.e. after route_from_parent has been traversed
    Graph::Lane::EventPtr event;

    // The route to get to this node from the parent node
    std::vector<Route> route_from_parent;
    // TODO(MXG): Create a RelativeTrajectory class to store the trajectory
    // information for these nodes. The use of absolute trajectories may lead to
    // bugs if we forget to adjust the times for each case.
  };

  struct SolutionNode
  {
    NodeInfo info;
    SolutionNodePtr child;
  };

  struct KeyHash
  {
    KeyHash(std::size_t N_lanes)
    {
      _lane_shift = std::ceil(std::log2(N_lanes));
    }

    std::size_t operator()(const Key& key) const
    {
      if (key.goal_orientation == Orientation::Any)
      {
        return key.start_lane
            + (key.start_orientation << _lane_shift)
            + (key.goal_orientation << 2*(_lane_shift + 2));
      }

      return key.start_lane
          + (key.start_orientation << _lane_shift)
          + (key.goal_lane << (_lane_shift + 2))
          + (key.goal_orientation << 2*(_lane_shift + 2));
    }

  private:
    std::size_t _lane_shift;
  };

  struct EntryHash
  {
    EntryHash(std::size_t N_lanes)
    {
      _lane_shift = std::ceil(std::log2(N_lanes));
    }

    std::size_t operator()(const Entry& entry) const
    {
      return entry.lane + (entry.orientation << _lane_shift);
    }

  private:
    std::size_t _lane_shift;
  };
};

//==============================================================================
using DifferentialDriveMap =
  std::unordered_map<
    DifferentialDriveMapTypes::Key,
    DifferentialDriveMapTypes::SolutionNodePtr,
    DifferentialDriveMapTypes::KeyHash
  >;

//==============================================================================
using DifferentialDriveEntrySet =
  std::unordered_set<
    DifferentialDriveMapTypes::Entry,
    DifferentialDriveMapTypes::EntryHash
  >;

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEMAP_HPP
