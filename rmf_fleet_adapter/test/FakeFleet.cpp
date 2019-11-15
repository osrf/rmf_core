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

#include <vector>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

class FakeFleetDriver : public rclcpp::Node
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using RobotMode = rmf_fleet_msgs::msg::RobotMode;
  using Location = rmf_fleet_msgs::msg::Location;

  FakeFleetDriver(const std::string& _fleet_name)
  : fleet_name(_fleet_name),
    Node(_fleet_name + "_driver")
  {
    fleet_state_pub = create_publisher<FleetState>(
        fleet_name + "/fleet_state", rclcpp::SystemDefaultsQoS());

    using namespace std::chrono_literals;
    timer = create_wall_timer(
        1000ms, std::bind(&FakeFleetDriver::publish_fleet_state, this));

    steady_clock.reset(new rclcpp::Clock(RCL_STEADY_TIME));

    // fake fleet
    fleet_state_msg.name = fleet_name;

    // fake robot 1
    RobotState straight_robot;
    straight_robot.name = "straight_robot";
    straight_robot.model = "pepper";
    RobotMode sr_mode = RobotMode.MODE_MOVING;
    straight_robot.mode = sr_mode;
    straight_robot.battery_percent = 88.8;
    Location sr_location;
    sr_location.t = steady_clock->now();
    sr_location.x = 0.0;
    sr_location.y = 0.0;
    sr_location.yaw = 0.0;
    sr_location.level_name = "B1";
    straight_robot.location = sr_location;
    straight_robot.path.clear();
  }

private:

  std::string fleet_name;

  rclcpp::Publisher<FleetState>::SharedPtr fleet_state_pub;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<rclcpp::Clock> steady_clock;

  FleetState fleet_state_msg;

  void publish_fleet_state()
  {
    RCLCPP_INFO(get_logger(), "publishing fleet state!");
    fleet_state_pub->publish(fleet_state_msg);
  }

};

int main(int argc, char** argv)
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  rclcpp::spin(std::make_shared<FakeFleetDriver>("fake_fleet"));

  rclcpp::shutdown();
  std::cout << "all done" << std::endl;
  return 0;
}