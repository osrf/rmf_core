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

#ifndef RMF_TASK__REQUESTS__DELIVERY_HPP
#define RMF_TASK__REQUESTS__DELIVERY_HPP

#include <chrono>
#include <string>

#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_battery/MotionPowerSink.hpp>
#include <rmf_battery/DevicePowerSink.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_task/agv/State.hpp>
#include <rmf_task/Request.hpp>
#include <rmf_task/Estimate.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_request_item.hpp>

namespace rmf_task {
namespace requests {

class DeliveryDescription : public rmf_task::Request::Description
{
public:

  using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;
  using Start = rmf_traffic::agv::Planner::Start;

  static DescriptionPtr make(
    std::size_t pickup_waypoint,
    std::string pickup_dispenser,
    std::size_t dropoff_waypoint,
    std::string dropoff_ingestor,
    std::vector<DispenserRequestItem> items,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true);

  rmf_utils::optional<rmf_task::Estimate> estimate_finish(
    const agv::State& initial_state,
    const agv::Constraints& task_planning_constraints,
    const std::shared_ptr<EstimateCache> estimate_cache) const final;

  rmf_traffic::Duration invariant_duration() const final;

  /// Get the pickup waypoint in this request
  std::size_t pickup_waypoint() const;

  /// Get the name of the dispenser at the pickup waypoint
  const std::string& pickup_dispenser() const;

  /// Get the dropoff waypoint in this request
  std::size_t dropoff_waypoint() const;

  /// Get the name of the ingestor at the dropoff waypoint
  const std::string& dropoff_ingestor() const;

  /// Get the list of dispenser request items in this request
  const std::vector<DispenserRequestItem>&  items() const;

  /// Get the Start when the robot reaches the pickup_waypoint from an initial
  /// start
  Start dropoff_start(const Start& start) const;  

  class Implementation;
private:
  DeliveryDescription();

  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Delivery
{
public:

  using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;

  static ConstRequestPtr make(
    const std::string& id,
    std::size_t pickup_waypoint,
    std::string pickup_dispenser,
    std::size_t dropoff_waypoint,
    std::string dropoff_ingestor,
    std::vector<DispenserRequestItem> items,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    rmf_traffic::Time start_time,
    bool drain_battery = true,
    ConstPriorityPtr priority = nullptr);
};

} // namespace requests
} // namespace rmf_task

#endif // RMF_TASK__REQUESTS__DELIVERY_HPP
