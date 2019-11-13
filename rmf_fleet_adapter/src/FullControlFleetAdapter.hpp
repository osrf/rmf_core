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

#ifndef RMF_FLEET_ADAPTER__SRC__FULLCONTROLFLEETADAPTER_HPP
#define RMF_FLEET_ADAPTER__SRC__FULLCONTROLFLEETADAPTER_HPP

#include "FleetAdapterNode.hpp"

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

namespace rmf_fleet {
namespace adapter {

class FullControlFleetAdapter : public FleetAdapterNode
{
public:

  using SharedPtr = std::shared_ptr<FullControlFleetAdapter>;

  static SharedPtr make(const std::string& fleet_name)
  {
    SharedPtr fleet_adapter(new FullControlFleetAdapter(fleet_name));
    return fleet_adapter;
  }

  // static std::shared_ptr<FullControlFleetAdapter> make(
  //     std::string fleet_name,
  //     const std::string& graph_file,
  //     rmf_traffic::agv::VehicleTraits vehicle_traits,
  //     rmf_traffic::Duration wait_time = std::chrono::seconds(10));

  ~FullControlFleetAdapter()
  {
    RCLCPP_INFO(get_logger(), "~FullControlFleetAdapter(): called.");
  }

  bool is_ready() const final
  {
    return true;
  }

private:

  FullControlFleetAdapter(std::string fleet_name)
  : FleetAdapterNode(fleet_name, FleetControlLevel::FullControl)
  {
    RCLCPP_INFO(get_logger(), "FullControlFleetAdapter(): called.");
  }

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  rclcpp::Subscription<FleetState>::SharedPtr fleet_state_sub;

  void fleet_state_cb();

};

} // adapter
} // rmf_fleet

#endif // RMF_FLEET_ADAPTER__SRC__FULLCONTROLFLEETADAPTER_HPP
