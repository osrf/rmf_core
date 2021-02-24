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

#include <rmf_traffic/reservations/reservations.hpp>
#include <rmf_traffic_msgs/msg/reservation_cancel.hpp>
#include <rmf_traffic_msgs/msg/reservation_request.hpp>
#include <rmf_traffic_msgs/msg/reservation_response.hpp>

#include <rmf_traffic_ros2/reservation/Node.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include "NodeInternal.hpp"

namespace rmf_traffic_ros2 {
namespace reservation {

//=============================================================================
ReservationNode::ReservationNode(const rclcpp::NodeOptions& options):
  Node("choping_node")
{
  _resp_pub = create_publisher<ReservationResponse>(
    ReservationStatusTopicName,
    rclcpp::SystemDefaultsQoS()
  );

  _request_sub = create_subscription<ReservationRequest>(
    ReservationReqTopicName,
    rclcpp::SystemDefaultsQoS().best_effort(),
    [=](const ReservationRequest::UniquePtr msg)
    {
      on_request_reservation(*msg);
    }
  );

  _cancel_sub = create_subscription<ReservationCancel>(
    ReservationCancelTopicName,
    rclcpp::SystemDefaultsQoS().best_effort(),
    [=](const ReservationCancel::UniquePtr msg)
    {
      on_cancel_reservation(*msg);
    }
  );
}

//=============================================================================
void ReservationNode::on_request_reservation(ReservationRequest req)
{
  auto result = _reservation_system.reserve(
    convert(req.time),
    req.locations,
    req.finite_duration ? std::optional{convert(req.duration)} : std::nullopt);
  
  ReservationResponse resp;
  resp.task_id = req.task_id;
  
  if(result.has_value())
  {
    resp.status = ReservationResponse::STATUS_OK;
    resp.waypoint = result->waypoint();
    resp.reservation_id = result->reservation_id();
  }
  else
  {
    resp.status = ReservationResponse::STATUS_FAILED_TO_ALLOCATE;
  }
  
  _resp_pub->publish(resp);
}

//=============================================================================
void ReservationNode::on_cancel_reservation(ReservationCancel canc)
{
  _reservation_system.cancel_reservation(canc.reservation_id);
}

//=============================================================================
std::shared_ptr<rclcpp::Node> make_node(
  const rclcpp::NodeOptions& options)
{
  return std::make_shared<ReservationNode>(options);
}

} // namespace reservation
} // namespace rmf_traffic_ros2
