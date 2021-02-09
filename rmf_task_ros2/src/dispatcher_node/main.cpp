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

#include <rclcpp/rclcpp.hpp>
#include <rmf_task_ros2/Dispatcher.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "~Initializing Dispatcher Node~" << std::endl;
  auto node = rclcpp::Node::make_shared("rmf_dispatcher_node");

  // ros2 param
  const double bidding_time_window =
    node->declare_parameter<double>("bidding_time_window", 2.0);
  RCLCPP_INFO(node->get_logger(),
    "Declared Time Window Param as: [%f] secs", bidding_time_window);
  const int terminated_tasks_depth =
    node->declare_parameter<int>("terminated_tasks_depth", 50);
  RCLCPP_INFO(node->get_logger(),
    "Declared Terminated Tasks Depth Param as: [%d]",
    terminated_tasks_depth);
  const int evaluator_type_enum =
    node->declare_parameter<int>("evaluator_type_enum", 0);
  RCLCPP_INFO(node->get_logger(), "Type of Evaluator: [%d]",
    evaluator_type_enum);

  std::shared_ptr<rmf_task::Evaluator> eval_type;
  switch (evaluator_type_enum)
  {
    case 0:
      eval_type = std::make_shared<rmf_task::LeastFleetDiffCostEvaluator>();
      break;
    case 1:
      eval_type = std::make_shared<rmf_task::LeastFleetCostEvaluator>();
      break;
    case 2:
      eval_type = std::make_shared<rmf_task::QuickestFinishEvaluator>();
      break;
    default:
      RCLCPP_WARN(node->get_logger(), "Invalid Evaluator, use default: 0");
      eval_type = std::make_shared<rmf_task::LeastFleetDiffCostEvaluator>();
      break;
  }

  auto dispatcher = rmf_task_ros2::Dispatcher::make(
    node, eval_type, bidding_time_window, terminated_tasks_depth);

  RCLCPP_INFO(node->get_logger(), "Starting task dispatcher node");
  dispatcher->spin();
  RCLCPP_INFO(node->get_logger(), "Closing down task dispatcher");
  rclcpp::shutdown();
}
