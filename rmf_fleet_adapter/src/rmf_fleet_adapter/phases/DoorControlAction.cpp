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

#include "DoorControlAction.hpp"
#include "SupervisorHasSession.hpp"
#include "RxOperators.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace rmf_fleet_adapter {
namespace phases {

//==============================================================================
DoorControlAction::DoorControlAction(
  std::string door_name,
  uint32_t target_mode,
  std::weak_ptr<rmf_rxcpp::Transport> transport,
  rxcpp::observable<rmf_door_msgs::msg::DoorState::SharedPtr> door_state_obs,
  rxcpp::observable<rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr> supervisor_heartbeat_obs)
  : _door_name{std::move(door_name)},
    _target_mode{target_mode},
    _transport{std::move(transport)},
    _door_state_obs{std::move(door_state_obs)},
    _supervisor_heartbeat_obs{std::move(supervisor_heartbeat_obs)}
{
  using rmf_door_msgs::msg::DoorState;
  using rmf_door_msgs::msg::SupervisorHeartbeat;

  auto transport_ = _transport.lock();
  if (!transport_)
    throw std::runtime_error("invalid transport state");
  // TODO: multiplex publisher?
  _publisher = transport_->create_publisher<rmf_door_msgs::msg::DoorRequest>(
    AdapterDoorRequestTopicName, 10);

  _session_id = boost::uuids::to_string(boost::uuids::random_generator{}());

  auto cancelled_obs = _cancelled.get_observable()
    .filter([](const auto& b) { return b; })
    .map([](const auto&)
    {
      Task::StatusMsg status;
      status.state = Task::StatusMsg::STATE_COMPLETED;
      status.status = "cancelled";
      return status;
    });

  using CombinedType = std::tuple<DoorState::SharedPtr, SupervisorHeartbeat::SharedPtr>;
  _obs = _door_state_obs.combine_latest(_supervisor_heartbeat_obs)
    .lift<CombinedType>(on_subscribe([this]() { _do_publish(); }))
    .map([this](const auto& v)
    {
      return _check_status(std::get<0>(v), std::get<1>(v));
    })
    .merge(cancelled_obs)
    .lift<Task::StatusMsg>(grab_while([this](const Task::StatusMsg& status)
    {
      if (
        status.state == Task::StatusMsg::STATE_COMPLETED ||
        status.state == Task::StatusMsg::STATE_FAILED)
      {
        _timer.reset();
        return false;
      }
      return true;
    }));
}

//==============================================================================
Task::StatusMsg DoorControlAction::_check_status(
  const rmf_door_msgs::msg::DoorState::SharedPtr& door_state,
  const rmf_door_msgs::msg::SupervisorHeartbeat::SharedPtr& heartbeat)
{
  Task::StatusMsg status{};
  status.state = Task::StatusMsg::STATE_ACTIVE;

  if (door_state->door_name != _door_name)
    return status;

  bool has_session = supervisor_has_session(*heartbeat, _session_id, _door_name);
  if (!_supervisor_received_publish)
  {
    _supervisor_received_publish = has_session;
    if (_supervisor_received_publish)
      _timer.reset();
  }
  else if (!_supervisor_finished_request)
    _supervisor_finished_request = !has_session;

  if (_supervisor_finished_request && door_state->current_mode.value == _target_mode)
    status.state = Task::StatusMsg::STATE_COMPLETED;

  return status;
}

//==============================================================================
void DoorControlAction::_do_publish()
{
  auto transport = _transport.lock();
  if (!transport)
    throw std::runtime_error("invalid transport state");

  rmf_door_msgs::msg::DoorRequest msg{};
  msg.door_name = _door_name;
  msg.request_time = transport->now();
  msg.requested_mode.value = _target_mode;
  msg.requester_id = _session_id;

  _publisher->publish(msg);
  _timer = transport->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]()
    {
      _do_publish();
    });
}

} // namespace phases
} // namespace rmf_fleet_adapter