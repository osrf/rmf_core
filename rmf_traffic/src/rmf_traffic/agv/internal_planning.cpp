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

#include "internal_planning.hpp"

#include "planning/DifferentialDrivePlanner.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
InterfacePtr make_planner_interface(Planner::Configuration config)
{
  if (const auto* differential = config.vehicle_traits().get_differential())
    return std::make_shared<DifferentialDrivePlanner>(std::move(config));

  throw std::runtime_error(
        "[rmf_traffic::agv::planning::make_planner_interface] The rmf_traffic "
        "Planner currently only supports differential drive vehicles.");
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
