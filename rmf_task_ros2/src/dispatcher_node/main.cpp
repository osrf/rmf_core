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
class DispatcherNode : public rclcpp::Node
{
public:
  static std::shared_ptr<DispatcherNode> make_node()
  {
    auto node = std::shared_ptr<DispatcherNode>(new DispatcherNode);
    node->_dispatcher = rmf_task_ros2::dispatcher::Dispatcher::make(node);
    
    node->_active_tasks_tracker = 
      std::move(node->_dispatcher->get_active_tasks());
    node->_terminated_tasks_tracker = 
      std::move(node->_dispatcher->get_terminated_tasks());

    return node;
  };

private:
  std::shared_ptr<rmf_task_ros2::dispatcher::Dispatcher> _dispatcher;

  rclcpp::Service<SubmitTaskSrv>::SharedPtr _submit_task_srv;
  rclcpp::Service<CancelTaskSrv>::SharedPtr _cancel_task_srv;
  rclcpp::Service<GetTaskSrv>::SharedPtr _get_task_srv;

  rmf_task_ros2::dispatcher::DispatchTasksPtr _active_tasks_tracker;
  rmf_task_ros2::dispatcher::DispatchTasksPtr _terminated_tasks_tracker;

  DispatcherNode()
  : Node("rmf_task_dispatcher_node")
  {
    std::cout << "~Initializing Dispatcher Node~" << std::endl;
    
    _submit_task_srv =create_service<SubmitTaskSrv>(
      rmf_task_ros2::SubmitTaskSrvName,
      [this]( 
          const std::shared_ptr<SubmitTaskSrv::Request> request,
          std::shared_ptr<SubmitTaskSrv::Response>      response)
      {
        // convert
        rmf_task_ros2::TaskProfileMsg msg;
        msg.type = request->type;
        msg.start_time = request->start_time;
        msg.params = request->params;

        auto id = _dispatcher->submit_task(rmf_task_ros2::convert(msg));
        RCLCPP_WARN(get_logger(),"Submit New Task!!! ID %s", id.c_str());
        response->task_id = id;
        response->success = true;
      }
    );

    _cancel_task_srv =create_service<CancelTaskSrv>(
      rmf_task_ros2::CancelTaskSrvName,
      [this](
          const std::shared_ptr<CancelTaskSrv::Request> request,
          std::shared_ptr<CancelTaskSrv::Response>      response)
      {
        auto id  = request->task_id;
        std::cout << "\n";
        RCLCPP_WARN(get_logger(),"Cancel Task!!! ID %s", id.c_str());       
        response->success = _dispatcher->cancel_task(id);
      }
    );

    _get_task_srv =create_service<GetTaskSrv>(
      rmf_task_ros2::GetTaskSrvName,
      [this]( 
        const std::shared_ptr<GetTaskSrv::Request> request,
        std::shared_ptr<GetTaskSrv::Response>      response)
      {
        auto ids = request->task_id;
        std::string printout = " | ID: ";
        for (auto id : ids)
          printout = printout + " " + id;

        // currently return all tasks
        std::cout << "\n - Active Tasks >>>> ";
        for(auto task : *_active_tasks_tracker)
        {
          std::cout << " {" << task.first << " * " 
                    << (int)task.second->state << "} ";
          response->active_tasks.push_back( 
            rmf_task_ros2::convert(*(task.second)));
        }
        std::cout << std::endl;
        
        // Terminated Tasks
        std::cout << " - Teminated Tasks >>>> ";
        for(auto task : *_terminated_tasks_tracker)
        {
          std::cout << " {" << task.first << " * " 
                    << (int)task.second->state << "} ";
          response->terminated_tasks.push_back( 
            rmf_task_ros2::convert(*(task.second)));
        }
        std::cout << std::endl;

        RCLCPP_WARN(get_logger(),"Get Task!!! ID %s | %d active | %d done",
          printout.c_str(), 
          _active_tasks_tracker->size(), 
          _terminated_tasks_tracker->size());
        
        response->success = true;
      }
    );
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
