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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.s
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <rmf_traffic/reservations/reservations.hpp>
#include <rmf_traffic_msgs/msg/reservation_cancel.hpp>
#include <rmf_traffic_msgs/msg/reservation_request.hpp>
#include <rmf_traffic_msgs/msg/reservation_response.hpp>

#include <rmf_traffic_ros2/reservation/Node.hpp>


namespace rmf_traffic_ros2 {
namespace reservation {

class ReservationNode : public rclcpp::Node 
{
  using ReservationRequest = rmf_traffic_msgs::msg::ReservationRequest;
  void on_request_reservation(ReservationRequest request);
  rclcpp::Subscription<ReservationRequest>::SharedPtr _request_sub;

  using ReservationCancel = rmf_traffic_msgs::msg::ReservationCancel;
  void on_cancel_reservation(ReservationCancel cancel);
  rclcpp::Subscription<ReservationCancel>::SharedPtr _cancel_sub;

  using ReservationResponse = rmf_traffic_msgs::msg::ReservationResponse;
  rclcpp::Publisher<ReservationResponse>::SharedPtr _resp_pub;

  rmf_traffic::reservations::ReservationSystem _reservation_system;
public:
  ReservationNode(const rclcpp::NodeOptions& options);
};

} // end namespace reservation
} // end namespace rmf_traffic_ros2
