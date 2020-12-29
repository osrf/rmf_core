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

// TODO(MXG): Move the inline function definitions in this header into their own
// .cpp file.

//==============================================================================
enum class Orientation
{
  Forward = 0,
  Backward,
  Any
};

//==============================================================================
inline std::ostream& operator<<(std::ostream& os, const Orientation orientation)
{
  if (orientation == Orientation::Forward)
    os << "Forward";
  else if(orientation == Orientation::Backward)
    os << "Backward";
  else if (orientation == Orientation::Any)
    os << "Any";
  else
    os << "Orientation::UNKNOWN";

  return os;
}

//==============================================================================
enum class Side
{
  Start = 0,
  Finish
};

//==============================================================================
inline std::ostream& operator<<(std::ostream& os, const Side side)
{
  if (side == Side::Start)
    os << "Start";
  else if (side == Side::Finish)
    os << "Finish";
  else
    os << "Side::UNKNOWN";

  return os;
}

//==============================================================================
struct DifferentialDriveMapTypes
{
  // TODO(MXG): Consider refactoring Key to use Entry
  struct Key
  {
    std::size_t start_lane;
    Orientation start_orientation;
    Side start_side; ///< Which side of the start lane is the vehicle on
    std::size_t goal_lane;
    Orientation goal_orientation;

    inline Key(
      const std::size_t start_lane_,
      const Orientation start_orientation_,
      const Side start_side_,
      const std::size_t goal_lane_,
      const Orientation goal_orientation_)
    : start_lane(start_lane_),
      start_orientation(start_orientation_),
      start_side(start_side_),
      goal_lane(goal_lane_),
      goal_orientation(goal_orientation_)
    {
      // Do nothing
    }

    inline bool operator==(const Key& other) const
    {
      return
          start_lane == other.start_lane &&
          start_orientation == other.start_orientation &&
          start_side == other.start_side &&
          goal_lane == other.goal_lane &&
          goal_orientation == other.goal_orientation;
    }
  };

  struct Entry
  {
    std::size_t lane;
    Orientation orientation;
    Side side;

    inline Entry(
      const std::size_t lane_,
      const Orientation orientation_,
      const Side side_)
    : lane(lane_),
      orientation(orientation_),
      side(side_)
    {
      // Do nothing
    }

    inline bool operator==(const Entry& other) const
    {
      return
          lane == other.lane &&
          orientation == other.orientation &&
          side == other.side;
    }

    inline bool operator!=(const Entry& other) const
    {
      return !(*this == other);
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
    std::optional<double> yaw;

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
      const std::size_t lane_shift = std::ceil(std::log2(N_lanes));
      _start_orientation_shift = lane_shift;
      _start_side_shift = _start_orientation_shift + 2;
      _goal_lane_shift = _start_side_shift + 1;
      _goal_orientation_shift = _goal_lane_shift + lane_shift;
    }

    std::size_t operator()(const Key& key) const
    {
      return
          key.start_lane
          + (static_cast<std::size_t>(key.start_orientation)
             << _start_orientation_shift)
          + (static_cast<std::size_t>(key.start_side)
             << _start_side_shift)
          + (key.goal_lane
             << _goal_lane_shift)
          + (static_cast<std::size_t>(key.goal_orientation)
             << _goal_orientation_shift);
    }

  private:
    std::size_t _start_orientation_shift;
    std::size_t _start_side_shift;
    std::size_t _goal_lane_shift;
    std::size_t _goal_orientation_shift;
  };

  struct EntryHash
  {
    EntryHash(std::size_t N_lanes)
    {
      _orientation_shift = std::ceil(std::log2(N_lanes));
      _side_shift = _orientation_shift + 2;
    }

    std::size_t operator()(const Entry& entry) const
    {
      return entry.lane
          + (static_cast<std::size_t>(entry.orientation) << _orientation_shift)
          + (static_cast<std::size_t>(entry.side) << _side_shift);
    }

  private:
    std::size_t _orientation_shift;
    std::size_t _side_shift;
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
using DifferentialDriveKeySet =
  std::unordered_set<
    DifferentialDriveMapTypes::Key,
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
