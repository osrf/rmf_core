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

#ifndef RMF_TRAFFIC__BLOCKADE__WRITER_HPP
#define RMF_TRAFFIC__BLOCKADE__WRITER_HPP

#include <Eigen/Geometry>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Writer
{
public:

  struct Checkpoint
  {
    Eigen::Vector2d position;
    rmf_traffic::Time departure_time;
    bool can_hold;
  };

  struct Sequence
  {
    std::string map_name;
    std::vector<Checkpoint> checkpoints;
  };

  struct Reservation
  {
    uint64_t reservation_id;
    std::vector<Sequence> sequences;
    double radius;
  };

  struct TimeUpdate
  {
    uint64_t reservation_id;
    uint64_t sequence_index;
    uint64_t checkpoint_index;
    rmf_traffic::Time departure_time;
  };

};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__WRITER_HPP
