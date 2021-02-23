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

#include <rmf_task_ros2/Description.hpp>

namespace rmf_task_ros2 {

//==============================================================================
rmf_traffic::Time Description::start_time() const
{
  return _start_time;
}

//==============================================================================
uint32_t Description::type() const
{
  return task_type.type;
}

//==============================================================================
std::shared_ptr<const Description> Description::make_description(
  rmf_traffic::Time start_time,
  uint32_t type)
{
  auto desc = std::make_shared<Description>();
  desc->task_type.type = type;
  desc->_start_time = std::move(start_time);
  return desc;
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Description::to_msg() const
{
  rmf_task_msgs::msg::TaskDescription msg;
  msg.task_type = task_type;
  msg.start_time = rmf_traffic_ros2::convert(_start_time);
  return msg;
}

namespace description {

//==============================================================================
std::shared_ptr<const Delivery> Delivery::make(
  rmf_traffic::Time start_time,
  std::string pickup_place_name,
  std::string pickup_dispenser,
  std::string dropoff_place_name,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items)
{
  if (pickup_place_name.empty()  || pickup_dispenser.empty() ||
    dropoff_place_name.empty() || dropoff_ingestor.empty())
    return nullptr;

  auto delivery_desc = std::make_shared<Delivery>();

  delivery_desc->task_type.type = rmf_task_msgs::msg::TaskType::TYPE_DELIVERY;
  delivery_desc->_start_time = std::move(start_time);
  delivery_desc->_pickup_place_name = std::move(pickup_place_name);
  delivery_desc->pickup_dispenser = std::move(pickup_dispenser);
  delivery_desc->_dropoff_place_name = std::move(dropoff_place_name);
  delivery_desc->dropoff_ingestor = std::move(dropoff_ingestor);
  delivery_desc->items = std::move(items);

  return delivery_desc;
}

//==============================================================================
std::shared_ptr<const Delivery> Delivery::make_from_msg(
  TaskDescription msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_DELIVERY)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.delivery.pickup_place_name,
    msg.delivery.pickup_dispenser,
    msg.delivery.dropoff_place_name,
    msg.delivery.dropoff_ingestor,
    msg.delivery.items);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Delivery::to_msg() const
{
  rmf_task_msgs::msg::TaskDescription msg;
  msg.task_type = task_type;
  msg.start_time = rmf_traffic_ros2::convert(_start_time);
  msg.delivery.pickup_place_name = _pickup_place_name;
  msg.delivery.dropoff_place_name = _dropoff_place_name;
  msg.delivery.pickup_dispenser = pickup_dispenser;
  msg.delivery.dropoff_ingestor = dropoff_ingestor;
  return msg;
}

//==============================================================================
const std::string& Delivery::pickup_place_name() const
{
  return _pickup_place_name;
}

//==============================================================================
const std::string& Delivery::dropoff_place_name() const
{
  return _dropoff_place_name;
}

//==============================================================================
//==============================================================================
std::shared_ptr<const Loop> Loop::make(
  rmf_traffic::Time start_time,
  std::string start_name,
  std::string finish_name,
  std::size_t num_loops)
{
  if (start_name.empty()  || finish_name.empty() || num_loops == 0)
    return nullptr;

  auto loop_desc = std::make_shared<Loop>();
  loop_desc->task_type.type = rmf_task_msgs::msg::TaskType::TYPE_LOOP;
  loop_desc->_start_time = std::move(start_time);
  loop_desc->_start_name = std::move(start_name);
  loop_desc->_finish_name = std::move(finish_name);
  loop_desc->num_loops = num_loops;
  return loop_desc;
}

//==============================================================================
std::shared_ptr<const Loop> Loop::make_from_msg(TaskDescription msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_LOOP)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.loop.start_name,
    msg.loop.finish_name,
    msg.loop.num_loops);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Loop::to_msg() const
{
  TaskDescription msg;
  msg.task_type = task_type;
  msg.start_time = rmf_traffic_ros2::convert(_start_time);
  msg.loop.start_name = _start_name;
  msg.loop.finish_name = _finish_name;
  return msg;
}

//==============================================================================
const std::string& Loop::start_name() const
{
  return this->_start_name;
}

//==============================================================================
const std::string& Loop::finish_name() const
{
  return this->_finish_name;
}

//==============================================================================
//==============================================================================
std::shared_ptr<const Clean> Clean::make(
  rmf_traffic::Time start_time,
  std::string start_waypoint)
{
  if (start_waypoint.empty())
    return nullptr;

  auto clean_desc = std::make_shared<Clean>();
  clean_desc->task_type.type = rmf_task_msgs::msg::TaskType::TYPE_CLEAN;
  clean_desc->_start_time = std::move(start_time);
  clean_desc->_start_waypoint = std::move(start_waypoint);
  return clean_desc;
}

//==============================================================================
std::shared_ptr<const Clean> Clean::make_from_msg(TaskDescription msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_CLEAN)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.clean.start_waypoint);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Clean::to_msg() const
{
  TaskDescription msg;
  msg.task_type = task_type;
  msg.start_time = rmf_traffic_ros2::convert(_start_time);
  msg.clean.start_waypoint = _start_waypoint;
  return msg;
}

//==============================================================================
const std::string& Clean::start_waypoint() const
{
  return this->_start_waypoint;
}

} // namespace description
} // namespace rmf_task_ros2
