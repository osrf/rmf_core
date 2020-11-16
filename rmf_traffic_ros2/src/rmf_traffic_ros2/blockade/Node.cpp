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

#include <rmf_traffic_ros2/blockade/Node.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic/blockade/Moderator.hpp>

#include <rmf_traffic_msgs/msg/blockade_cancel.hpp>
#include <rmf_traffic_msgs/msg/blockade_heartbeat.hpp>
#include <rmf_traffic_msgs/msg/blockade_reached.hpp>
#include <rmf_traffic_msgs/msg/blockade_ready.hpp>
#include <rmf_traffic_msgs/msg/blockade_set.hpp>
#include <rmf_traffic_msgs/msg/blockade_status.hpp>

namespace rmf_traffic_ros2 {
namespace blockade {

//==============================================================================
class BlockadeNode : public rclcpp::Node
{
public:

  using Checkpoint = rmf_traffic::blockade::Writer::Checkpoint;
  using Reservation = rmf_traffic::blockade::Writer::Reservation;

  BlockadeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rmf_traffic_blockade_node", options),
      moderator(std::make_shared<rmf_traffic::blockade::Moderator>())
  {
    blockade_set_sub =
        create_subscription<SetMsg>(
          BlockadeSetTopicName,
          rclcpp::SystemDefaultsQoS().best_effort(),
          [=](const SetMsg::UniquePtr msg)
    {
      this->blockade_set(*msg);
    });

    blockade_ready_sub =
        create_subscription<ReadyMsg>(
          BlockadeReadyTopicName,
          rclcpp::SystemDefaultsQoS().best_effort(),
          [=](const ReadyMsg::UniquePtr msg)
    {
      this->blockade_ready(*msg);
    });

    blockade_reached_sub =
        create_subscription<ReachedMsg>(
          BlockadeReachedTopicName,
          rclcpp::SystemDefaultsQoS().best_effort(),
          [=](const ReachedMsg::UniquePtr msg)
    {
      this->blockade_reached(*msg);
    });

    blockade_cancel_sub =
        create_subscription<CancelMsg>(
          BlockadeCancelTopicName,
          rclcpp::SystemDefaultsQoS().best_effort(),
          [=](const CancelMsg::UniquePtr msg)
    {
      this->blockade_cancel(*msg);
    });

    heartbeat_pub = create_publisher<HeartbeatMsg>(
          BlockadeHeartbeatTopicName,
          rclcpp::SystemDefaultsQoS().reliable());

    heartbeat_timer = create_wall_timer(
          std::chrono::seconds(1),
          [this]()
    {
      this->publish_status();
    });
  }

  using SetMsg = rmf_traffic_msgs::msg::BlockadeSet;
  rclcpp::Subscription<SetMsg>::SharedPtr blockade_set_sub;
  void blockade_set(const SetMsg& set)
  {
    std::cout << "Participant [" << set.participant << "] set new path ["
              << set.reservation << "]" << std::endl;

    std::vector<Checkpoint> path;
    for (const auto& c : set.path)
    {
      path.push_back(
            Checkpoint{
              Eigen::Vector2d{c.position[0], c.position[1]},
              c.map_name,
              c.can_hold
            });
    }

    moderator->set(
          set.participant, set.reservation,
          Reservation{std::move(path), set.radius});

    check_for_updates();
  }

  using ReadyMsg = rmf_traffic_msgs::msg::BlockadeReady;
  rclcpp::Subscription<ReadyMsg>::SharedPtr blockade_ready_sub;
  void blockade_ready(const ReadyMsg& ready)
  {
    std::cout << "Participant [" << ready.participant
              << ":" << ready.reservation << "] ready for "
              << ready.checkpoint << std::endl;

    moderator->ready(ready.participant, ready.reservation, ready.checkpoint);

    check_for_updates();
  }

  using ReachedMsg = rmf_traffic_msgs::msg::BlockadeReached;
  rclcpp::Subscription<ReachedMsg>::SharedPtr blockade_reached_sub;
  void blockade_reached(const ReachedMsg& reached)
  {
    std::cout << "Participant [" << reached.participant
              << ":" << reached.reservation << "] reached "
              << reached.checkpoint << std::endl;

    try
    {
      moderator->reached(
            reached.participant, reached.reservation, reached.checkpoint);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
            get_logger(), "Exception due to [reached] update: %s", e.what());
    }

    check_for_updates();
  }

  using CancelMsg = rmf_traffic_msgs::msg::BlockadeCancel;
  rclcpp::Subscription<CancelMsg>::SharedPtr blockade_cancel_sub;
  void blockade_cancel(const CancelMsg& cancel)
  {
    if (cancel.all_reservations)
      moderator->cancel(cancel.participant);
    else
      moderator->cancel(cancel.participant, cancel.reservation);

    check_for_updates();
  }

  void check_for_updates()
  {
    print_status();
    const std::size_t current_version = moderator->assignments().version();
    if (current_version == last_assignment_version)
      return;

    last_assignment_version = current_version;
    publish_status();
  }

  using HeartbeatMsg = rmf_traffic_msgs::msg::BlockadeHeartbeat;
  rclcpp::Publisher<HeartbeatMsg>::SharedPtr heartbeat_pub;
  using StatusMsg = rmf_traffic_msgs::msg::BlockadeStatus;
  void publish_status()
  {
    const auto& ranges = moderator->assignments().ranges();

    std::vector<StatusMsg> statuses;
    for (const auto& s : moderator->statuses())
    {
      const std::size_t participant = s.first;
      const auto& range = ranges.at(participant);
      const auto& status = s.second;

      statuses.emplace_back(
            rmf_traffic_msgs::build<StatusMsg>()
            .participant(participant)
            .reservation(status.reservation)
            .any_ready(status.last_ready.has_value())
            .last_ready(status.last_ready.value_or(0))
            .last_reached(status.last_reached)
            .assignment_begin(range.begin)
            .assignment_end(range.end));
    }

    auto msg = rmf_traffic_msgs::build<HeartbeatMsg>()
        .statuses(std::move(statuses))
        .has_gridlock(moderator->has_gridlock());

    heartbeat_pub->publish(msg);
  }

  void print_status()
  {
    std::cout << " == NEW STATUS:\n";
    const auto& ranges = moderator->assignments().ranges();
    for (const auto& s : moderator->statuses())
    {
      const std::size_t participant = s.first;
      const auto& range = ranges.at(participant);
      const auto& status = s.second;

      std::cout << " >> " << participant << ": [";
      if (status.last_ready.has_value())
        std::cout << status.last_ready.value();
      else
        std::cout << "null";

      std::cout << "; " << status.last_reached << "; "
                << range.begin << ", " << range.end << "]\n";
    }
    std::cout << std::endl;
  }

  std::shared_ptr<rmf_traffic::blockade::Moderator> moderator;
  std::size_t last_assignment_version = 0;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
};

//==============================================================================
std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return std::make_shared<BlockadeNode>(options);
}

} // namespace blockade
} // namespace rmf_traffic_ros2
