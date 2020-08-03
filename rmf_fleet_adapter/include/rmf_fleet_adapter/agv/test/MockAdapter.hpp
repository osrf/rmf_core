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

#include <memory>

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/TrafficLight.hpp>

#include <rclcpp/node.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

#include <rmf_traffic/schedule/Snapshot.hpp>

namespace rmf_fleet_adapter {
namespace agv {
namespace test {

//==============================================================================
/// This class is an alternative to the Adapter class, but made specifically for
/// testing. It does not try to connect to a Schedule Node or to any Negotiation
/// topics. It keeps its database internal.
class MockAdapter : public std::enable_shared_from_this<MockAdapter>
{
public:

  /// Create a mock adapter
  MockAdapter(
      const std::string& node_name,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  /// Add a fleet to test
  std::shared_ptr<FleetUpdateHandle> add_fleet(
      const std::string& fleet_name,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::agv::Graph navigation_graph);

  TrafficLight::UpdateHandlePtr add_traffic_light(
      std::shared_ptr<TrafficLight::CommandHandle> command,
      const std::string& fleet_name,
      const std::string& robot_name,
      rmf_traffic::agv::VehicleTraits traits,
      rmf_traffic::Profile profile);

  /// Get the rclcpp Node for this adapter
  std::shared_ptr<rclcpp::Node> node();

  /// const-qualified node()
  std::shared_ptr<const rclcpp::Node> node() const;

  /// Start spinning this adapter
  void start();

  /// Stop this adapter from spinning
  void stop();

  /// Submit a delivery request
  void request_delivery(const rmf_task_msgs::msg::Delivery& request);

  /// Submit a loop request
  void request_loop(const rmf_task_msgs::msg::Loop& request);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace test
} // namespace agv
} // namespace rmf_fleet_adapter
