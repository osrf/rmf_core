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

#include "NoControlFleetAdapter.hpp"

namespace rmf_fleet {
namespace adapter {

NoControlFleetAdapter::SharedPtr NoControlFleetAdapter::make(
    const std::string& _fleet_name)
{
  SharedPtr fleet_adapter(new NoControlFleetAdapter(_fleet_name));
  return fleet_adapter;
}

NoControlFleetAdapter::NoControlFleetAdapter(const std::string& _fleet_name)
: FleetAdapterNode(_fleet_name, FleetControlLevel::NoControl)
{}

NoControlFleetAdapter::~NoControlFleetAdapter()
{}

bool NoControlFleetAdapter::is_ready() const
{
  return true;
}

void NoControlFleetAdapter::start()
{
  fleet_state_sub = create_subscription<FleetState>(
      fleet_name + "/fleet_state", rclcpp::SystemDefaultsQoS(),
      [&](FleetState::UniquePtr msg)
  {
    fleet_state_cb(std::move(msg));
  });
}

void NoControlFleetAdapter::fleet_state_cb(FleetState::UniquePtr _msg)
{
  // parses through the fleet state
}

} // namespace adapter
} // namespace rmf_fleet
