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


#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>
#include <std_msgs/msg/string.hpp>

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>


class TestReadOnly : public rclcpp::Node
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using RobotState = rmf_fleet_msgs::msg::RobotState;
  using RobotMode = rmf_fleet_msgs::msg::RobotMode;
  using Location = rmf_fleet_msgs::msg::Location;
  using String = std_msgs::msg::String;

  TestReadOnly(
      std::string& fleet_name,
      std::string& viz_name)
  : Node("test_adapter_" + fleet_name),
    _fleet_name(fleet_name),
    _viz_name(viz_name)
  {
    // publisher to send FleetState messages to fleet adapter

    _fleet_state_pub = create_publisher<FleetState>(
        _fleet_name + "/fleet_state", rclcpp::SystemDefaultsQoS());
    
    // publisher to send String messages to rmf_schedule_visualizer node

    _viz_pub = create_publisher<String>(
        _viz_name + "/debug", rclcpp::SystemDefaultsQoS());

    // subscription to receive String messages which control execution of
    // this node

    _cmd_sub= create_subscription<String>(
        "test_adapter_" + _fleet_name+"/cmd",rclcpp::SystemDefaultsQoS(),
        [&](String::UniquePtr msg)
    {
      // receive commands through ros2 msg
      if (msg->data == "test_insert")
      {
        std::cout<<"Testing insert"<<std::endl;
        test_insert();
      }
      else if (msg->data == "test_advance")
      {
        std::cout<<"Testing advance"<<std::endl;
        test_advance();
      }
      else if (msg->data == "test_delay")
      {
        std::cout<<"Testing delay"<<std::endl;
        test_delay();
      }
      else if (msg->data == "test_replace")
      {
        std::cout<<"Testing replace"<<std::endl;
        test_replace();
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
        "Invalid cmd, try: test_insert, test_advance,test_delay, test_replace");
      }

    });

    RobotState robot;
    robot.name = "TestBot";
    robot.model = "1.0";
    RobotMode mode;
    mode.mode = mode.MODE_MOVING;
    robot.mode = mode;
    robot.battery_percent = 90.0;
    robot.location = get_location(
        rmf_traffic_ros2::convert(
            std::chrono::steady_clock::now()),
        0.0,
        0.0,
        0.0,
        "level1");


    // Robot moving to locations {1.0, 0} and {2.0, 0} 

    robot.path.push_back(
        get_location(
        modify_time(robot.location.t, 5),
        1.0,
        0.0,
        0.0,
        "level1"));

    robot.path.push_back(
        get_location(
        modify_time(robot.location.t, 10),
        2.0,
        0.0,
        0.0,
        "level1"));

    _fleet_state_msg.name = _fleet_name;
    _fleet_state_msg.robots.push_back(robot);

  }



private:
  Location get_location(
      builtin_interfaces::msg::Time t,
      double x,
      double y,
      double yaw,
      std::string level)
      {
        Location location;
        location.t = t;
        location.x = x;
        location.y = y;
        location.yaw = yaw;
        location.level_name = level;
        return location;
      }
  
  builtin_interfaces::msg::Time modify_time (
      builtin_interfaces::msg::Time t_, double delta)
      {
        t_.sec += delta;
        return t_; 
      }

  void test_insert()
  {
    if(!_inserted)
    {
      _fleet_state_pub->publish(_fleet_state_msg);

    }
  }
  void test_advance()
  {

  }
  void test_delay()
  {

  }
  void test_replace()
  {

  }

  std::string _fleet_name;
  std::string _viz_name;
  FleetState _fleet_state_msg;
  rclcpp::Publisher<FleetState>::SharedPtr _fleet_state_pub;
  rclcpp::Subscription<String>::SharedPtr _cmd_sub;
  rclcpp::Publisher<String>::SharedPtr _viz_pub;
  bool _inserted = false;

};


int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string fleet_name = "fleet1";
  std::string viz_name = "viz";
  rclcpp::spin(std::make_shared<TestReadOnly>(fleet_name,viz_name));

  rclcpp::shutdown();
  return 0;

}