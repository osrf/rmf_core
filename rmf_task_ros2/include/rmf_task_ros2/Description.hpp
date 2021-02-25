/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef RMF_TASK_ROS2__DESCRIPTION_HPP
#define RMF_TASK_ROS2__DESCRIPTION_HPP

#include <memory>
#include <iostream>
#include <iostream>

#include <rmf_task_msgs/msg/task_description.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task_ros2 {

//==============================================================================
// A description for user requested task
class Description
{
public:
  using TaskDescription = rmf_task_msgs::msg::TaskDescription;

  static std::shared_ptr<const Description> make_description(
    rmf_traffic::Time start_time,
    uint32_t type);

  rmf_traffic::Time start_time() const;

  uint32_t type() const;

  virtual TaskDescription to_msg() const;

protected:
  rmf_traffic::Time _start_time;
  rmf_task_msgs::msg::TaskType task_type;
};

using ConstDescriptionPtr = std::shared_ptr<const Description>;

//==============================================================================
namespace description {

class Delivery : public Description
{
public:
  using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;

  static std::shared_ptr<const Delivery> make(
    rmf_traffic::Time start_time,
    std::string pickup_place_name,
    std::string pickup_dispenser,
    std::string dropoff_place_name,
    std::string dropoff_ingestor,
    std::vector<DispenserRequestItem> items);

  /// use this only within internal implementation of RMF
  static std::shared_ptr<const Delivery> make_from_msg(TaskDescription msg);

  /// use this only within internal implementation of RMF
  TaskDescription to_msg() const final;

  /// Get the pickup_place_name
  const std::string& pickup_place_name() const;

  /// Get the dropoff_place_name
  const std::string& dropoff_place_name() const;

private:
  std::string _pickup_place_name;
  std::string pickup_dispenser;
  std::string _dropoff_place_name;
  std::string dropoff_ingestor;
  std::vector<DispenserRequestItem> items;
};

//==============================================================================
class Loop : public Description
{
public:
  static std::shared_ptr<const Loop> make(
    rmf_traffic::Time start_time,
    std::string start_name,
    std::string finish_name,
    std::size_t num_loops);

  static std::shared_ptr<const Loop> make_from_msg(TaskDescription msg);

  TaskDescription to_msg() const final;

  /// Get the start_name
  const std::string& start_name() const;

  /// Get the finish_name
  const std::string& finish_name() const;

private:
  std::string _start_name;
  std::string _finish_name;
  std::size_t num_loops;
};

//==============================================================================

class Clean : public Description
{
public:
  std::string _start_waypoint;

  static std::shared_ptr<const Clean> make(
    rmf_traffic::Time start_time,
    std::string start_waypoint);

  static std::shared_ptr<const Clean> make_from_msg(TaskDescription msg);

  TaskDescription to_msg() const final;

  /// Get the _start_waypoint
  const std::string& start_waypoint() const;
};

} // namespace description
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DESCRIPTION_HPP
