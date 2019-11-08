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

#ifndef RMF_TRAFFIC__AGV__GRAPH_HPP
#define RMF_TRAFFIC__AGV__GRAPH_HPP

#include <Eigen/Geometry>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Graph {
 public:
  class Waypoint {
   public:
    /// Set the position of this Waypoint
    Waypoint& set_position(const Eigen::Vector2d& position);

    /// Get the position of this Waypoint
    Eigen::Vector2d get_position() const;
  };

  class Edge {
   public:
    enum class Type {
      Free,
      Door,
      Lift,
    };
  };
};

}  // namespace agv
}  // namespace rmf_traffic

#endif  // RMF_TRAFFIC__AGV__GRAPH_HPP
