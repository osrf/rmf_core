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
#include <rmf_task_ros2/dispatcher/Dispatcher.hpp>

#include <rmf_task_msgs/srv/submit_task.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <rmf_task_msgs/srv/get_task_list.hpp>

//==============================================================================
using SubmitTaskSrv = rmf_task_msgs::srv::SubmitTask;
using CancelTaskSrv = rmf_task_msgs::srv::CancelTask;
using GetTaskListSrv = rmf_task_msgs::srv::GetTaskList;

//==============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
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
      RCLCPP_INFO(node->get_logger(), "Received task submission!");
      // convert
      rmf_task_ros2::TaskProfile task_profile;
      task_profile.task_type = request->task_type;
      task_profile.start_time = request->start_time;
      task_profile.clean = request->clean;
      task_profile.loop = request->loop;
      task_profile.delivery = request->delivery;
      task_profile.station = request->station;

      auto id = dispatcher->submit_task(task_profile);

      switch (request->evaluator)
      {
        using namespace rmf_task_ros2::bidding;
        case SubmitTaskSrv::Request::LOWEST_DIFF_COST_EVAL:
          {
            dispatcher->evaluator(
              std::make_shared<LeastFleetDiffCostEvaluator>());
            break;
          }
        case SubmitTaskSrv::Request::LOWEST_COST_EVAL:
          {
            dispatcher->evaluator(
              std::make_shared<LeastFleetCostEvaluator>());
            break;
          }
        case SubmitTaskSrv::Request::QUICKEST_FINISH_EVAL:
          {
            dispatcher->evaluator(
              std::make_shared<QuickestFinishEvaluator>());
            break;
          }
      }

      RCLCPP_DEBUG(node->get_logger(), "Generated ID is: %s", id.c_str());
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
      RCLCPP_WARN(node->get_logger(), "Cancel Task ID: %s", id.c_str());
      response->success = dispatcher->cancel_task(id);
    }
  );

  auto get_task_srv = node->create_service<GetTaskListSrv>(
    rmf_task_ros2::GetTaskListSrvName,
    [&dispatcher, &node](
      const std::shared_ptr<GetTaskListSrv::Request> request,
      std::shared_ptr<GetTaskListSrv::Response> response)
    {
      RCLCPP_WARN(
        node->get_logger(), "Get Task List: %d active | %d terminated tasks",
        dispatcher->active_tasks()->size(),
        dispatcher->terminated_tasks()->size());

      // currently return all tasks
      std::cout << "\n - Active Tasks >>>> ";
      for (auto task : *(dispatcher->active_tasks()))
      {
        std::cout << " {" << task.first << " * "
                  << (int)task.second->state << "} ";
        response->active_tasks.push_back(
          rmf_task_ros2::convert_status<rmf_task_msgs::msg::TaskSummary>(
            *(task.second)));
      }
      std::cout << std::endl;

      // Terminated Tasks
      std::cout << " - Teminated Tasks >>>> " << std::flush;
      for (auto task : *(dispatcher->terminated_tasks()))
      {
        std::cout << " {" << task.first << " * "
                  << (int)task.second->state << "} "
                  << std::flush;
        response->terminated_tasks.push_back(
          rmf_task_ros2::convert_status<rmf_task_msgs::msg::TaskSummary>(
            *(task.second)));
      }
      std::cout << std::endl;
      response->success = true;
    }
  );

  dispatcher->spin();

  RCLCPP_INFO(node->get_logger(), "Closing down task dispatcher");
  rclcpp::shutdown();
}
