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

#ifndef RMF_FLEET_ADAPTER__SRC__COMPONENTS__FLEETSTATELISTENER_HPP
#define RMF_FLEET_ADAPTER__SRC__COMPONENTS__FLEETSTATELISTENER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>

namespace rmf_fleet {
namespace adapter {

class FleetStateListener
{
public:

  FleetStateListener(rclcpp::Node& node, const std::string& fleet_name);

private:

  // TODO(MXG): Replace this with a std::optional as soon as we can use C++17
  std::unique_ptr<FleetComponents> components;

  using FleetState = rmf_fleet_msgs::msg::FleetState;

  void fleet_state_cb(FleetState::UniquePtr msg);

};

}
}


#endif // RMF_FLEET_ADAPTER__SRC__COMPONENTS__FLEETSTATELISTENER_HPP
