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

#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/schedule_conflict_ack.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_repeat.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_notice.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_proposal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_conclusion.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
class Negotiation::Implementation
{
public:

  rclcpp::Node& node;

  using Repeat = rmf_traffic_msgs::msg::ScheduleConflictRepeat;
  using RepeatSub = rclcpp::Subscription<Repeat>;
  using RepeatPub = rclcpp::Publisher<Repeat>;
  RepeatSub::SharedPtr repeat_sub;
  RepeatPub::SharedPtr repeat_pub;

  using Notice = rmf_traffic_msgs::msg::ScheduleConflictNotice;
  using NoticeSub = rclcpp::Subscription<Notice>;
  using NoticePub = rclcpp::Publisher<Notice>;
  NoticeSub::SharedPtr notice_sub;
  NoticePub::SharedPtr notice_pub;

  using Proposal = rmf_traffic_msgs::msg::ScheduleConflictProposal;
  using ProposalSub = rclcpp::Subscription<Proposal>;
  using ProposalPub = rclcpp::Publisher<Proposal>;
  ProposalSub::SharedPtr proposal_sub;
  ProposalPub::SharedPtr proposal_pub;

  using Conclusion = rmf_traffic_msgs::msg::ScheduleConflictConclusion;
  using ConclusionSub = rclcpp::Subscription<Conclusion>;
  ConclusionSub::SharedPtr conclusion_sub;

  using Ack = rmf_traffic_msgs::msg::ScheduleConflictAck;
  using AckPub = rclcpp::Publisher<Ack>;
  AckPub::SharedPtr ack_pub;

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using NegotiatorPtr = std::unique_ptr<rmf_traffic::schedule::Negotiator>;
  using NegotiatorMap = std::unordered_map<ParticipantId, NegotiatorPtr>;
  using NegotiationMapPtr = std::shared_ptr<NegotiatorMap>;
  using WeakNegotiationMapPtr = std::weak_ptr<NegotiatorMap>;
  NegotiationMapPtr negotiators;

  using Version = rmf_traffic::schedule::Version;
  using Negotiation = rmf_traffic::schedule::Negotiation;
  using NegotiationMap = std::unordered_map<Version, Negotiation>;
  NegotiationMap negotiations;

  Implementation(rclcpp::Node& node_)
    : node(node_)
  {
    // TODO(MXG): Make the QoS configurable
    const auto qos = rclcpp::ServicesQoS();

    repeat_sub = node.create_subscription<Repeat>(
          ScheduleConflictRepeatTopicName, qos,
          [&](const Repeat::UniquePtr msg)
    {
      this->repeat(*msg);
    });

    repeat_pub = node.create_publisher<Repeat>(
          ScheduleConflictRepeatTopicName, qos);

    notice_sub = node.create_subscription<Notice>(
          ScheduleConflictNoticeTopicName, qos,
          [&](const Notice::UniquePtr msg)
    {
      this->notice(*msg);
    });

    notice_pub = node.create_publisher<Notice>(
          ScheduleConflictNoticeTopicName, qos);

    proposal_sub = node.create_subscription<Proposal>(
          ScheduleConflictProposalTopicName, qos,
          [&](const Proposal::UniquePtr msg)
    {
      this->proposal(*msg);
    });

    proposal_pub = node.create_publisher<Proposal>(
          ScheduleConflictProposalTopicName, qos);

    conclusion_sub = node.create_subscription<Conclusion>(
          ScheduleConflictConclusionTopicName, qos,
          [&](const Conclusion::UniquePtr msg)
    {
      this->conclusion(*msg);
    });

    ack_pub = node.create_publisher<Ack>(
          ScheduleConflictAckTopicName, qos);
  }

  void repeat(const Repeat& msg)
  {
    // Ignore if it's asking for a repeat of the conflict notice
    if (msg.table.empty())
      return;

    // Ignore if we aren't involved in this negotiation
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
      return;

    // Ignore if we aren't managing the relevant negotiator
    const ParticipantId for_participant = msg.table.back();
    const auto negotiator_it = negotiators->find(for_participant);
    if (negotiator_it == negotiators->end())
      return;

    auto to_accommodate = msg.table;
    to_accommodate.pop_back();

    const auto table = negotiate_it->second.table(
          for_participant, to_accommodate);
    if (!table)
    {
      std::string error =
          "[rmf_traffic_ros2::schedule::Negotiation] A repeat was requested "
          "for a table that does not exist. Negotiation ["
          + std::to_string(msg.conflict_version) + "], table [";
      for (const auto p : msg.table)
        error += " " + std::to_string(p) + " ";
      error += "]";

      RCLCPP_WARN(node.get_logger(), error);
      return;
    }


  }

  void notice(const Notice& msg)
  {

  }

  void proposal(const Proposal& msg)
  {

  }

  void conclusion(const Conclusion& msg)
  {

  }

  struct Handle
  {
    ParticipantId for_participant;
    WeakNegotiationMapPtr weak_map;

    ~Handle()
    {
      if (const auto map = weak_map.lock())
        map->erase(for_participant);
    }
  };

  std::shared_ptr<void> register_negotiator(
      const ParticipantId for_participant,
      NegotiatorPtr negotiator)
  {
    const auto insertion = negotiators->insert(
          std::make_pair(for_participant, std::move(negotiator)));

    if (!insertion.second)
    {
      throw std::runtime_error(
            "[rmf_traffic_ros2::schedule::Negotiaton] Attempt to register a "
            "duplicate negotiator for participant ["
            + std::to_string(for_participant) + "]");
    }

    return std::make_shared<Handle>(Handle{for_participant, negotiators});
  }
};

//==============================================================================
Negotiation::Negotiation(rclcpp::Node& node)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(node))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<void> Negotiation::register_negotiator(
    rmf_traffic::schedule::ParticipantId for_participant,
    std::unique_ptr<rmf_traffic::schedule::Negotiator> negotiator)
{
  return _pimpl->register_negotiator(for_participant, std::move(negotiator));
}

} // namespace schedule
} // namespace rmf_traffic_ros2
