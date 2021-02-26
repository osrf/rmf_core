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
#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_task_ros2 {

//==============================================================================
class Description::Implementation
{
public:
  Implementation()
  {}

  rmf_traffic::Time start_time;
  rmf_task_msgs::msg::Priority priority;
  rmf_task_msgs::msg::TaskType task_type;
};

//==============================================================================
rmf_traffic::Time Description::start_time() const
{
  return _pimpl_base->start_time;
}

//==============================================================================
uint32_t Description::type() const
{
  return _pimpl_base->task_type.type;
}


//==============================================================================
uint64_t Description::priority() const
{
  return _pimpl_base->priority.value;
}

//==============================================================================
std::shared_ptr<const Description> Description::make_description(
  rmf_traffic::Time start_time,
  uint32_t type,
  uint64_t priority)
{
  std::shared_ptr<Description> desc(new Description());
  desc->_pimpl_base->task_type.type = type;
  desc->_pimpl_base->priority.value = priority;
  desc->_pimpl_base->start_time = std::move(start_time);
  return desc;
}

//==============================================================================
std::shared_ptr<const Description> Description::make_from_msg(
  const TaskDescription& msg)
{
  return make_description(rmf_traffic_ros2::convert(msg.start_time),
      msg.task_type.type, msg.priority.value);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Description::to_msg() const
{
  rmf_task_msgs::msg::TaskDescription msg;
  msg.task_type = _pimpl_base->task_type;
  msg.priority = _pimpl_base->priority;
  msg.start_time = rmf_traffic_ros2::convert(_pimpl_base->start_time);
  return msg;
}

//==============================================================================
Description::Description()
: _pimpl_base(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
//==============================================================================
namespace description {

class Delivery::Implementation
{
public:
  Implementation()
  {}

  std::string pickup_place_name;
  std::string pickup_dispenser;
  std::string dropoff_place_name;
  std::string dropoff_ingestor;
  std::vector<DispenserRequestItem> items;
};

//==============================================================================
std::shared_ptr<const Delivery> Delivery::make(
  rmf_traffic::Time start_time,
  std::string pickup_place_name,
  std::string pickup_dispenser,
  std::string dropoff_place_name,
  std::string dropoff_ingestor,
  std::vector<DispenserRequestItem> items,
  uint64_t priority)
{
  if (pickup_place_name.empty()  || pickup_dispenser.empty() ||
    dropoff_place_name.empty() || dropoff_ingestor.empty())
    return nullptr;

  std::shared_ptr<Delivery> delivery_desc(new Delivery());

  using TaskType = rmf_task_msgs::msg::TaskType;
  delivery_desc->_pimpl_base->task_type.type = TaskType::TYPE_DELIVERY;
  delivery_desc->_pimpl_base->start_time = std::move(start_time);
  delivery_desc->_pimpl_base->priority.value = priority;
  delivery_desc->_pimpl->pickup_place_name = std::move(pickup_place_name);
  delivery_desc->_pimpl->pickup_dispenser = std::move(pickup_dispenser);
  delivery_desc->_pimpl->dropoff_place_name = std::move(dropoff_place_name);
  delivery_desc->_pimpl->dropoff_ingestor = std::move(dropoff_ingestor);
  delivery_desc->_pimpl->items = std::move(items);

  return delivery_desc;
}

//==============================================================================
std::shared_ptr<const Delivery> Delivery::make_from_msg(
  const TaskDescription& msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_DELIVERY)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.delivery.pickup_place_name,
    msg.delivery.pickup_dispenser,
    msg.delivery.dropoff_place_name,
    msg.delivery.dropoff_ingestor,
    msg.delivery.items,
    msg.priority.value);
}

//==============================================================================
Delivery::Delivery()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Delivery::to_msg() const
{
  rmf_task_msgs::msg::TaskDescription msg;
  msg.task_type = _pimpl_base->task_type;
  msg.priority = _pimpl_base->priority;
  msg.start_time = rmf_traffic_ros2::convert(_pimpl_base->start_time);
  msg.delivery.pickup_place_name = _pimpl->pickup_place_name;
  msg.delivery.dropoff_place_name = _pimpl->dropoff_place_name;
  msg.delivery.pickup_dispenser = _pimpl->pickup_dispenser;
  msg.delivery.dropoff_ingestor = _pimpl->dropoff_ingestor;
  return msg;
}

//==============================================================================
const std::string& Delivery::pickup_place_name() const
{
  return _pimpl->pickup_place_name;
}

//==============================================================================
const std::string& Delivery::dropoff_place_name() const
{
  return _pimpl->dropoff_place_name;
}

//==============================================================================
//==============================================================================
class Loop::Implementation
{
public:
  Implementation()
  {}

  std::string start_name;
  std::string finish_name;
  std::size_t num_loops;
};

std::shared_ptr<const Loop> Loop::make(
  rmf_traffic::Time start_time,
  std::string start_name,
  std::string finish_name,
  std::size_t num_loops,
  uint64_t priority)
{
  if (start_name.empty()  || finish_name.empty() || num_loops == 0)
    return nullptr;

  std::shared_ptr<Loop> loop_desc(new Loop());

  using TaskType = rmf_task_msgs::msg::TaskType;
  loop_desc->_pimpl_base->task_type.type = TaskType::TYPE_LOOP;
  loop_desc->_pimpl_base->priority.value = priority;
  loop_desc->_pimpl_base->start_time = std::move(start_time);
  loop_desc->_pimpl->start_name = std::move(start_name);
  loop_desc->_pimpl->finish_name = std::move(finish_name);
  loop_desc->_pimpl->num_loops = num_loops;
  return loop_desc;
}

//==============================================================================
std::shared_ptr<const Loop> Loop::make_from_msg(
  const TaskDescription& msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_LOOP)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.loop.start_name,
    msg.loop.finish_name,
    msg.loop.num_loops,
    msg.priority.value);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Loop::to_msg() const
{
  TaskDescription msg;
  msg.task_type = _pimpl_base->task_type;
  msg.priority = _pimpl_base->priority;
  msg.start_time = rmf_traffic_ros2::convert(_pimpl_base->start_time);
  msg.loop.start_name = _pimpl->start_name;
  msg.loop.finish_name = _pimpl->finish_name;
  msg.loop.num_loops = _pimpl->num_loops;
  return msg;
}

//==============================================================================
Loop::Loop()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
const std::string& Loop::start_name() const
{
  return this->_pimpl->start_name;
}

//==============================================================================
const std::string& Loop::finish_name() const
{
  return this->_pimpl->finish_name;
}

//==============================================================================
//==============================================================================
class Clean::Implementation
{
public:
  Implementation()
  {}

  std::string start_waypoint;
};

std::shared_ptr<const Clean> Clean::make(
  rmf_traffic::Time start_time,
  std::string start_waypoint,
  uint64_t priority)
{
  if (start_waypoint.empty())
    return nullptr;

  std::shared_ptr<Clean> clean_desc(new Clean());

  using TaskType = rmf_task_msgs::msg::TaskType;
  clean_desc->_pimpl_base->task_type.type = TaskType::TYPE_CLEAN;
  clean_desc->_pimpl_base->start_time = std::move(start_time);
  clean_desc->_pimpl_base->priority.value = priority;
  clean_desc->_pimpl->start_waypoint = std::move(start_waypoint);
  return clean_desc;
}

//==============================================================================
std::shared_ptr<const Clean> Clean::make_from_msg(
  const TaskDescription& msg)
{
  if (msg.task_type.type != rmf_task_msgs::msg::TaskType::TYPE_CLEAN)
    return nullptr;

  return make(
    rmf_traffic_ros2::convert(msg.start_time),
    msg.clean.start_waypoint,
    msg.priority.value);
}

//==============================================================================
rmf_task_msgs::msg::TaskDescription Clean::to_msg() const
{
  TaskDescription msg;
  msg.task_type = _pimpl_base->task_type;
  msg.priority = _pimpl_base->priority;
  msg.start_time = rmf_traffic_ros2::convert(_pimpl_base->start_time);
  msg.clean.start_waypoint = _pimpl->start_waypoint;
  return msg;
}

//==============================================================================
Clean::Clean()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{
  // Do nothing
}

//==============================================================================
const std::string& Clean::start_waypoint() const
{
  return this->_pimpl->start_waypoint;
}

} // namespace description
} // namespace rmf_task_ros2
