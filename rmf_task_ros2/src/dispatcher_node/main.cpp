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

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task.hpp>

//==============================================================================
using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
using GetTaskSrv = rmf_task_msgs::srv::GetTask;

//==============================================================================

int main(int argc, char* argv[])
{
  // rclcpp::init(argc, argv);

  std::cout << "~Initializing Dispatcher Node~" << std::endl;

  auto dispatcher = rmf_task_ros2::dispatcher::Dispatcher::make(
    "rmf_dispatcher_node");

  const auto& node = dispatcher->node();
  RCLCPP_INFO(node->get_logger(), "Starting task dispatcher node");

  auto submit_task_srv = node->create_service<SubmitTaskSrv>(
    rmf_task_ros2::SubmitTaskSrvName,
    [&dispatcher, &node](
      const std::shared_ptr<SubmitTaskSrv::Request> request,
      std::shared_ptr<SubmitTaskSrv::Response> response)
    {
      // convert
      rmf_task_ros2::TaskProfileMsg msg;
      msg.type = request->type;
      msg.start_time = request->start_time;
      msg.params = request->params;

      auto id = dispatcher->submit_task(rmf_task_ros2::convert(msg));
      RCLCPP_WARN(node->get_logger(), "Submit New Task!!! ID %s", id.c_str());
      response->task_id = id;
      response->success = true;
    }
  );

  auto cancel_task_srv = node->create_service<CancelTaskSrv>(
    rmf_task_ros2::CancelTaskSrvName,
    [&dispatcher, &node](
      const std::shared_ptr<CancelTaskSrv::Request> request,
      std::shared_ptr<CancelTaskSrv::Response> response)
    {
      auto id = request->task_id;
      std::cout << "\n";
      RCLCPP_WARN(node->get_logger(), "Cancel Task!!! ID %s", id.c_str());
      response->success = dispatcher->cancel_task(id);
    }
  );

  auto get_task_srv = node->create_service<GetTaskSrv>(
    rmf_task_ros2::GetTaskSrvName,
    [&dispatcher, &node](
      const std::shared_ptr<GetTaskSrv::Request> request,
      std::shared_ptr<GetTaskSrv::Response> response)
    {
      auto ids = request->task_id;
      std::string printout = " | ID: ";
      for (auto id : ids)
        printout = printout + " " + id;

      // currently return all tasks
      std::cout << "\n - Active Tasks >>>> ";
      for (auto task : *dispatcher->active_tasks())
      {
        std::cout << " {" << task.first << " * "
                  << (int)task.second->state << "} ";
        response->active_tasks.push_back(
          rmf_task_ros2::convert(*(task.second)));
      }
      std::cout << std::endl;

      // Terminated Tasks
      std::cout << " - Teminated Tasks >>>> ";
      for (auto task : *dispatcher->terminated_tasks())
      {
        std::cout << " {" << task.first << " * "
                  << (int)task.second->state << "} ";
        response->terminated_tasks.push_back(
          rmf_task_ros2::convert(*(task.second)));
      }
      std::cout << std::endl;

      RCLCPP_WARN(node->get_logger(), "Get Task!!! ID %s | %d active | %d done",
      printout.c_str(),
      dispatcher->active_tasks()->size(),
      dispatcher->terminated_tasks()->size());

      response->success = true;
    }
  );

  dispatcher->spin();

  RCLCPP_INFO(node->get_logger(), "Closing down task dispatcher");
  rclcpp::shutdown();
}
