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

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/schedule_conflict_ack.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_repeat.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_notice.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_proposal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_rejection.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_conclusion.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

std::string table_to_string(
    const std::vector<rmf_traffic::schedule::ParticipantId>& table)
{
  std::string output;
  for (const auto p : table)
    output += " " + std::to_string(p);
  return output;
}

//==============================================================================
class Negotiation::Implementation
{
public:

  class Responder : public rmf_traffic::schedule::Negotiator::Responder
  {
  public:

    Implementation* const impl;
    const rmf_traffic::schedule::Version conflict_version;
    const rmf_traffic::schedule::Negotiation::TablePtr table;

    Responder(
        Implementation* const impl_,
        const rmf_traffic::schedule::Version version_,
        rmf_traffic::schedule::Negotiation::TablePtr table_)
      : impl(impl_),
        conflict_version(version_),
        table(table_)
    {
      // Do nothing
      std::cout << " -- Requesting response for ["
                << table_to_string(table->sequence()) << " ]" << std::endl;
    }

    void submit(
        std::vector<rmf_traffic::Route> itinerary,
        std::function<void()> approval_callback) const final
    {
      const auto* last_version = table->version();
      table->submit(itinerary, last_version? *last_version+1 : 0);
      impl->approvals[conflict_version][table] = std::move(approval_callback);
      impl->publish_proposal(conflict_version, *table);
    }

    void reject() const final
    {
      const auto parent = table->parent();
      if (parent)
      {
        // We will reject the parent to communicate that this whole branch is
        // infeasible
        parent->reject();
        impl->publish_rejection(conflict_version, *parent);
      }
      else if (table->ongoing())
      {
        // This means that the whole negotiation is completely impossible.
        // TODO(MXG): Improve this by having the planner reveal what vehicle is
        // blocking progress so we can add it to the negotiation
        table->reject();
        impl->publish_rejection(conflict_version, *table);
      }
    }

  };

  rclcpp::Node& node;
  const rmf_traffic::schedule::Viewer& viewer;

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

  using Rejection = rmf_traffic_msgs::msg::ScheduleConflictRejection;
  using RejectionSub = rclcpp::Subscription<Rejection>;
  using RejectionPub = rclcpp::Publisher<Rejection>;
  RejectionSub::SharedPtr rejection_sub;
  RejectionPub::SharedPtr rejection_pub;

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
  struct Entry
  {
    bool participating;
    Negotiation negotiation;
  };

  using NegotiationMap = std::unordered_map<Version, Entry>;

  // The negotiations that this Negotiation class is involved in
  NegotiationMap negotiations;

  using TablePtr = rmf_traffic::schedule::Negotiation::TablePtr;
  using ApprovalCallbackMap = std::unordered_map<TablePtr, std::function<void()>>;
  using Approvals = std::unordered_map<Version, ApprovalCallbackMap>;
  Approvals approvals;

  Implementation(
      rclcpp::Node& node_,
      const rmf_traffic::schedule::Viewer& viewer_)
    : node(node_),
      viewer(viewer_),
      negotiators(std::make_shared<NegotiatorMap>())
  {
    // TODO(MXG): Make the QoS configurable
    const auto qos = rclcpp::ServicesQoS();

    repeat_sub = node.create_subscription<Repeat>(
          ScheduleConflictRepeatTopicName, qos,
          [&](const Repeat::UniquePtr msg)
    {
      this->receive_repeat_request(*msg);
    });

    repeat_pub = node.create_publisher<Repeat>(
          ScheduleConflictRepeatTopicName, qos);

    notice_sub = node.create_subscription<Notice>(
          ScheduleConflictNoticeTopicName, qos,
          [&](const Notice::UniquePtr msg)
    {
      this->receive_notice(*msg);
    });

    notice_pub = node.create_publisher<Notice>(
          ScheduleConflictNoticeTopicName, qos);

    proposal_sub = node.create_subscription<Proposal>(
          ScheduleConflictProposalTopicName, qos,
          [&](const Proposal::UniquePtr msg)
    {
      this->receive_proposal(*msg);
    });

    proposal_pub = node.create_publisher<Proposal>(
          ScheduleConflictProposalTopicName, qos);

    rejection_sub = node.create_subscription<Rejection>(
          ScheduleConflictRejectionTopicName, qos,
          [&](const Rejection::UniquePtr msg)
    {
      this->receive_rejection(*msg);
    });

    rejection_pub = node.create_publisher<Rejection>(
          ScheduleConflictRejectionTopicName, qos);

    conclusion_sub = node.create_subscription<Conclusion>(
          ScheduleConflictConclusionTopicName, qos,
          [&](const Conclusion::UniquePtr msg)
    {
      this->receive_conclusion(*msg);
    });

    ack_pub = node.create_publisher<Ack>(
          ScheduleConflictAckTopicName, qos);
  }

  void receive_repeat_request(const Repeat& msg)
  {
    // Ignore if it's asking for a repeat of the conflict notice
    if (msg.table.empty())
      return;

    // Ignore if we haven't heard of this negotiation
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
      return;

    // Ignore if we aren't participating in this negotiation
    if (!negotiate_it->second.participating)
      return;

    // Ignore if we aren't managing the relevant negotiator
    const ParticipantId for_participant = msg.table.back();
    const auto negotiator_it = negotiators->find(for_participant);
    if (negotiator_it == negotiators->end())
      return;

    auto to_accommodate = msg.table;
    to_accommodate.pop_back();

    const auto table = negotiate_it->second.negotiation.table(
          for_participant, to_accommodate);
    if (!table)
    {
      std::string error =
          "[rmf_traffic_ros2::schedule::Negotiation] A repeat was requested "
          "for a table that does not exist. Negotiation ["
          + std::to_string(msg.conflict_version) + "], table ["
          + table_to_string(msg.table) + " ]";

      RCLCPP_WARN(node.get_logger(), error);
      return;
    }

    publish_proposal(msg.conflict_version, *table);
  }

  void receive_notice(const Notice& msg)
  {
    std::cout << "\nReceived notice for negotation [" << msg.conflict_version
              << "]:";
    for (const auto p : msg.participants)
      std::cout << " " << p;
    std::cout << std::endl;

    bool relevant = false;
    for (const auto p : msg.participants)
    {
      if (negotiators->find(p) != negotiators->end())
      {
        relevant = true;
        break;
      }
    }

    const auto insertion = negotiations.insert(
        {msg.conflict_version,
         Entry{relevant, Negotiation(viewer, msg.participants)}});

    const bool is_new = insertion.second;
    bool& participating = insertion.first->second.participating;
    Negotiation& negotiation = insertion.first->second.negotiation;

    if (!is_new)
    {
      const auto& old_participants = negotiation.participants();
      for (const auto p : msg.participants)
      {
        if (old_participants.count(p) == 0)
        {
          std::cout << " -- Adding [" << p << "] to the negotiation" << std::endl;
          negotiation.add_participant(p);
        }
      }
    }

    if (!relevant)
    {
      std::cout << " -- No initial response needed ["
                << is_new << " | " << relevant << " | " << participating
                << "]" << std::endl;
      return;
    }

    // TODO(MXG): Is the participating flag even relevant?
    participating = true;

    std::vector<TablePtr> queue;
    for (const auto p : msg.participants)
    {
      const auto it = negotiators->find(p);
      if (it != negotiators->end())
      {
        const auto table = negotiation.table(p, {});
        if (!table->submission())
        {
          it->second->respond(
                table, Responder(this, msg.conflict_version, table));
        }

        queue.push_back(table);
      }
    }

    while (!queue.empty())
    {
      const auto top = queue.back();
      queue.pop_back();

      for (const auto& n : *negotiators)
      {
        const auto respond_to = top->respond(n.first);
        if (!respond_to)
          continue;

        if (!respond_to->submission())
        {
          n.second->respond(
                respond_to, Responder(this, msg.conflict_version, respond_to));
        }

        queue.push_back(respond_to);
      }
    }
  }

  void receive_proposal(const Proposal& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      RCLCPP_WARN(
            node.get_logger(),
            "[rmf_traffic_ros2::schedule::Negotiation::receive_proposal] "
            "Received a proposal for an unknown negotiation: "
            + std::to_string(msg.conflict_version));
      return;
    }

    const bool participating = negotiate_it->second.participating;
    Negotiation& negotiation = negotiate_it->second.negotiation;
    const auto table =
        negotiation.table(msg.for_participant, msg.to_accommodate);

    if (!table)
    {
      // TODO(MXG): Work out a scheme for caching inconsistent proposals
      // so that the negotiation can be reconstructed after requesting some
      // repeats.
      RCLCPP_WARN(
            node.get_logger(),
            "[rmf_traffic_ros2::schedule::Negotiation::receive_proposal] "
            "Receieved a proposal that builds on an unknown table");
      return;
    }

    std::cout << "Received proposal for ["
              << table_to_string(msg.to_accommodate) << " "
              << msg.for_participant << " ]" << std::endl;

    // We'll keep track of these negotiations whether or not we're participating
    // in them, because
    const bool updated =
        table->submit(convert(msg.itinerary), msg.proposal_version);

    if (!updated)
      return;

    if (!participating)
      return;

    for (const auto& n : *negotiators)
    {
      const ParticipantId participant = n.first;
      const auto& negotiator = n.second;

      if (const auto can_respond = table->respond(participant))
      {
        negotiator->respond(
              can_respond,
              Responder(this, msg.conflict_version, can_respond));
      }
    }
  }

  void receive_rejection(const Rejection& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about caching an unknown rejection, because it
      // is impossible for a proposal that was produced by this negotiation
      // instance to be rejected without us being aware of that proposal.
      return;
    }

    Negotiation& negotiation = negotiate_it->second.negotiation;
    const auto table = negotiation.table(msg.table);
    if (!table)
    {
      // TODO(MXG): Work out a scheme for caching inconsistent rejections so
      // that the negotiation can be reconstructed after requesting some
      // repeats.
      return;
    }

    table->reject();
    // TODO(MXG): Give the negotiator a chance to offer a new proposal.
  }

  void receive_conclusion(const Conclusion& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about concluding unknown negotiations
      return;
    }

    const bool participating = negotiate_it->second.participating;
    Negotiation& negotiation = negotiate_it->second.negotiation;

    if (participating)
    {
      const auto approval_callback_it = approvals.find(msg.conflict_version);
      if (msg.resolved)
      {
        assert(approval_callback_it != approvals.end());
        const auto& approval_callbacks = approval_callback_it->second;

        auto result = negotiation.table(msg.table);

        while (result)
        {
          const auto approve_it = approval_callbacks.find(result);
          if (approve_it != approval_callbacks.end())
            approve_it->second();

          result = result->parent();
        }
      }

      if (approval_callback_it != approvals.end())
        approvals.erase(approval_callback_it);

      std::vector<ParticipantId> participants;
      participants.reserve(negotiators->size());
      for (const auto p : negotiation.participants())
      {
        const auto n_it = negotiators->find(p);
        if (n_it != negotiators->end())
          participants.push_back(n_it->first);
      }

      // Acknowledge that we know about this conclusion
      Ack ack;
      ack.conflict_version = msg.conflict_version;
      ack.participants = std::move(participants);
      ack_pub->publish(ack);
      // TODO(MXG): Should we consider a more robust cache cleanup strategy?
    }

    // Erase these entries because the negotiation has concluded
    negotiations.erase(negotiate_it);
  }

  void publish_proposal(
      const Version conflict_version,
      const Negotiation::Table& table)
  {
    std::cout << " -- Publishing proposal for ["
              << table_to_string(table.sequence()) << " ]" << std::endl;

    Proposal msg;
    msg.conflict_version = conflict_version;
    assert(table.version());
    msg.proposal_version = *table.version();

    assert(table.submission());
    msg.itinerary = convert(*table.submission());
    msg.for_participant = table.participant();

    const auto& sequence = table.sequence();
    msg.to_accommodate.reserve(sequence.size()-1);
    for (std::size_t i=0; i < sequence.size()-1; ++i)
      msg.to_accommodate.push_back(sequence[i]);

    proposal_pub->publish(msg);
  }

  void publish_rejection(
      const Version conflict_version,
      const Negotiation::Table& table)
  {
    std::cout << " -- Publishing rejection for ["
              << table_to_string(table.sequence()) << " ]" << std::endl;

    Rejection msg;
    msg.conflict_version = conflict_version;
    msg.table = table.sequence();
    rejection_pub->publish(msg);
  }

  struct Handle
  {
    Handle(
        const ParticipantId for_participant_,
        NegotiationMapPtr map)
      : for_participant(for_participant_),
        weak_map(map)
    {
      // Do nothing
    }

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

    return std::make_shared<Handle>(for_participant, negotiators);
  }
};

//==============================================================================
Negotiation::Negotiation(
    rclcpp::Node& node,
    const rmf_traffic::schedule::Viewer& viewer)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(node, viewer))
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
