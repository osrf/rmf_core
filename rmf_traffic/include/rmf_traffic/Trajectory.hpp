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

#ifndef RMF_TRAFFIC__TRAJECTORY_HPP
#define RMF_TRAFFIC__TRAJECTORY_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <memory>
#include <vector>

namespace rmf_traffic {

class Trajectory
{
public:

  class Profile
  {
  public:

    enum Movement {
      /// The robot will follow the specified trajectory exactly.
      Strict = 0,

      /// The robot will autonomously navigate within the specified space.
      Autonomous,

      /// The robot is waiting in a queue, and will proceed after the preceding
      /// robot is gone.
      Queued,
    };

    //================================================
    // Collision table:
    // |=============================================|
    // | Movement   | Strict  | Autonomous | Queued  |
    // |------------+---------+------------+---------+
    // | Strict     | COLLIDE |   okay     | COLLIDE |
    // |------------+---------+------------+---------+
    // | Autonomous |  okay   |  COLLIDE   |  okay   |
    // |------------+---------+------------+---------+
    // | Queued     | COLLIDE |   okay     |  okay   |
    // |=============================================|
    //================================================

    Profile(const geometry::ConstConvexShapePtr& shape, Movement movement);

  private:

    class Implementation;
//    std::unique_ptr<

    geometry::ConstConvexShapePtr shape;

    Movement movement;
  };

  struct Segment
  {
    Profile profile;

  };




private:

  class Implementation;
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TRAJECTORY_HPP
