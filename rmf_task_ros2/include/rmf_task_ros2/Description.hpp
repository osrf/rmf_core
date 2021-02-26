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

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

#include <rmf_task_msgs/msg/task_description.hpp>

namespace rmf_task_ros2 {

//==============================================================================
// A description for user requested task
class Description
{
public:
  using TaskDescription = rmf_task_msgs::msg::TaskDescription;

  /// Create a Description. This is only used when the task type is not 
  /// available as the derived class.
  ///
  /// \param [in] start_time
  ///
  /// \param [in] type
  static std::shared_ptr<const Description> make_description(
    rmf_traffic::Time start_time,
    uint32_t type,
    uint64_t priority = 0);

  /// Create a Description by providing a msg. This is only used when the task
  /// type is not available as the derived class.
  ///
  /// \param [in] msg
  ///   task description msg as argument
  static std::shared_ptr<const Description> make_from_msg(
    const TaskDescription& msg);

  /// Get the start_time of the task
  rmf_traffic::Time start_time() const;

  /// Get the type of the task
  uint32_t type() const;

  /// This is not created now
  uint64_t priority() const;

  /// Get the TaskDescription as msg
  virtual TaskDescription to_msg() const;

  class Implementation;
protected:
  Description();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl_base;
};

using ConstDescriptionPtr = std::shared_ptr<const Description>;

//==============================================================================
namespace description {

class Delivery : public Description
{
public:
  using DispenserRequestItem = rmf_dispenser_msgs::msg::DispenserRequestItem;

  /// Make a Delivery Task Description
  static std::shared_ptr<const Delivery> make(
    rmf_traffic::Time start_time,
    std::string pickup_place_name,
    std::string pickup_dispenser,
    std::string dropoff_place_name,
    std::string dropoff_ingestor,
    std::vector<DispenserRequestItem> items,
    uint64_t priority = 0);

  /// Create task description from description msg
  static std::shared_ptr<const Delivery> make_from_msg(
    const TaskDescription& msg);

  /// Get the TaskDescription as msg
  TaskDescription to_msg() const final;

  /// Get the pickup_place_name
  const std::string& pickup_place_name() const;

  /// Get the dropoff_place_name
  const std::string& dropoff_place_name() const;

  class Implementation;
private:
  Delivery();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Loop : public Description
{
public:
  /// Make a Loop Task Description
  static std::shared_ptr<const Loop> make(
    rmf_traffic::Time start_time,
    std::string start_name,
    std::string finish_name,
    std::size_t num_loops,
    uint64_t priority = 0);

  /// Create task description from description msg
  static std::shared_ptr<const Loop> make_from_msg(
    const TaskDescription& msg);

  /// Get the TaskDescription as msg
  TaskDescription to_msg() const final;

  /// Get the start_name
  const std::string& start_name() const;

  /// Get the finish_name
  const std::string& finish_name() const;

  class Implementation;
private:
  Loop();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Clean : public Description
{
public:
  /// Make a Clean Task Description
  static std::shared_ptr<const Clean> make(
    rmf_traffic::Time start_time,
    std::string start_waypoint,
    uint64_t priority = 0);

  /// Create task description from description msg
  static std::shared_ptr<const Clean> make_from_msg(
    const TaskDescription& msg);

  /// Get the TaskDescription as msg
  TaskDescription to_msg() const final;

  /// Get the start_waypoint of a clean
  const std::string& start_waypoint() const;

  class Implementation;
private:
  Clean();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace description
} // namespace rmf_task_ros2

#endif // RMF_TASK_ROS2__DESCRIPTION_HPP
