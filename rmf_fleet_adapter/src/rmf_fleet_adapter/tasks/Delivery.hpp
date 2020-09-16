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

#ifndef SRC__RMF_FLEET_ADAPTER__TASKS__DELIVERY_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKS__DELIVERY_HPP

#include "../Task.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_utils/optional.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

struct DeliveryEstimate
{
  rmf_traffic::Time time = rmf_traffic::Time::max(); 
  agv::RobotContextPtr robot = nullptr;
  rmf_utils::optional<rmf_traffic::agv::Plan::Start> pickup_start;
  rmf_utils::optional<rmf_traffic::agv::Plan::Start> dropoff_start;
  rmf_utils::optional<rmf_traffic::agv::Plan::Start> finish;
};

// TODO(MXG): Come up with a better design for task dispatch
rmf_utils::optional<DeliveryEstimate> estimate_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const std::shared_ptr<agv::FleetUpdateHandle>& fleet);

//==============================================================================
// TODO(MXG): This is a sloppy design. We should have a task estimator + factory
// interface to handle the task dispatch and creation pipeline more elegantly.
std::shared_ptr<Task> make_delivery(
    const rmf_task_msgs::msg::Delivery& request,
    const agv::RobotContextPtr& context,
    rmf_traffic::agv::Plan::Start pickup_start,
    rmf_traffic::agv::Plan::Start dropoff_start);

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKS__DELIVERY_HPP
