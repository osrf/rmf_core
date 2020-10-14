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

  struct Item
  {
    uint64_t id;
    Eigen::Vector2d start;
    Eigen::Vector2d finish;
    double radius;
    std::array<rmf_traffic::Time, 2> time_range;
    bool can_hold_at_finish;
  };



};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__WRITER_HPP
