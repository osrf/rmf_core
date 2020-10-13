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
#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>

#include <rmf_task_msgs/srv/post_task.hpp>
#include <rmf_task_msgs/srv/get_task.hpp>

// todo: create srv call for get/post req
class DispatcherNode : public rclcpp::Node
{
public: 
  static std::shared_ptr<DispatcherNode> make_node()
  {
    auto node = std::shared_ptr<DispatcherNode>(new DispatcherNode);
    node->_dispatcher = rmf_task_ros2::dispatcher::Dispatcher::make(node);
    return node;
  };

private:
  std::shared_ptr<rmf_task_ros2::dispatcher::Dispatcher> _dispatcher;

  DispatcherNode()
  : Node("rmf_task_dispatcher_node")
  {
    std::cout << "~Initializing Dispatcher Node~" << std::endl;
  };
};

//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = DispatcherNode::make_node();
  // auto node = rmf_task_ros2::dispatcher::DispatcherNode::make_node();

  RCLCPP_INFO(
    node->get_logger(),
    "Beginning task dispatcher node");

  rclcpp::spin(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Closing down task dispatcher node");

  rclcpp::shutdown();
}
