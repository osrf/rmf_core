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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNERINTERNAL_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNERINTERNAL_HPP

#include "GraphInternal.hpp"

namespace rmf_traffic {
namespace agv {
namespace internal {

std::vector<Trajectory> generate_plan(
    const Graph::Implementation& graph,
    std::size_t initial_waypoint,
    double initial_orientation,
    std::size_t final_waypoint,
    const double* final_orientation,
    const VehicleTraits& traits,
    const schedule::Viewer& viewer);

} // namespace internal
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNERINTERNAL_HPP
