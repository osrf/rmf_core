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

#include "NegotiationRoom.hpp"

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/schedule_conflict_ack.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_repeat.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_notice.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_refusal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_forfeit.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_proposal.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_rejection.hpp>
#include <rmf_traffic_msgs/msg/schedule_conflict_conclusion.hpp>

#include <rclcpp/logging.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
std::string table_to_string(
  const std::vector<rmf_traffic::schedule::ParticipantId>& table)
{
  std::string output;
  for (const auto p : table)
    output += " " + std::to_string(p);
  return output;
}

//==============================================================================
template <typename T>
std::string ptr_to_string(T* ptr)
{
  std::stringstream str;
  str << ptr;
  return str.str();
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
    rmf_utils::optional<rmf_traffic::schedule::Version> table_version;

    const rmf_traffic::schedule::Negotiation::TablePtr parent;
    rmf_utils::optional<rmf_traffic::schedule::Version> parent_version;

    Responder(
      Implementation* const impl_,
      const rmf_traffic::schedule::Version version_,
      rmf_traffic::schedule::Negotiation::TablePtr table_)
    : impl(impl_),
      conflict_version(version_),
      table(table_),
      table_version(rmf_utils::pointer_to_opt(table->version())),
      parent(table->parent()),
      parent_version(
        parent? rmf_utils::pointer_to_opt(parent->version())
              : rmf_utils::nullopt)
    {
      // Do nothing
    }

    void submit(
      std::vector<rmf_traffic::Route> itinerary,
      std::function<UpdateVersion()> approval_callback) const final
    {
      const bool accepted = table->submit(itinerary,
          table_version? *table_version+1 : 0);
      assert(accepted);
      (void)(accepted);
      impl->approvals[conflict_version][table] = std::move(approval_callback);
      impl->publish_proposal(conflict_version, *table);
    }

    void reject(const Alternatives& alternatives) const final
    {
      if (parent)
      {
        // We will reject the parent to communicate that this whole branch is
        // infeasible
        assert(parent->version());
        parent->reject(*parent->version(), table->participant(), alternatives);
        impl->publish_rejection(
              conflict_version, *parent, table->participant(), alternatives);
      }
    }

    void forfeit(const std::vector<ParticipantId>& /*blockers*/) const final
    {
      // TODO(MXG): Consider using blockers to invite more participants into the
      // negotiation
      table->forfeit(table_version? *table_version+1 : 0);
      impl->publish_forfeit(conflict_version, *table);
    }

  };

  rclcpp::Node& node;
  std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer;

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

  using Refusal = rmf_traffic_msgs::msg::ScheduleConflictRefusal;
  using RefusalPub = rclcpp::Publisher<Refusal>;
  RefusalPub::SharedPtr refusal_pub;

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

  using Forfeit = rmf_traffic_msgs::msg::ScheduleConflictForfeit;
  using ForfeitSub = rclcpp::Subscription<Forfeit>;
  using ForfeitPub = rclcpp::Publisher<Forfeit>;
  ForfeitSub::SharedPtr forfeit_sub;
  ForfeitPub::SharedPtr forfeit_pub;

  using Conclusion = rmf_traffic_msgs::msg::ScheduleConflictConclusion;
  using ConclusionSub = rclcpp::Subscription<Conclusion>;
  ConclusionSub::SharedPtr conclusion_sub;

  using ParticipantAck = rmf_traffic_msgs::msg::ScheduleConflictParticipantAck;
  using Ack = rmf_traffic_msgs::msg::ScheduleConflictAck;
  using AckPub = rclcpp::Publisher<Ack>;
  AckPub::SharedPtr ack_pub;

  using NegotiationMapPtr = std::shared_ptr<NegotiatorMap>;
  using WeakNegotiationMapPtr = std::weak_ptr<NegotiatorMap>;
  NegotiationMapPtr negotiators;

  using Version = rmf_traffic::schedule::Version;
  using Negotiation = rmf_traffic::schedule::Negotiation;
  struct Entry
  {
    bool participating;
    NegotiationRoom room;
  };

  using NegotiationMap = std::unordered_map<Version, Entry>;

  // The negotiations that this Negotiation class is involved in
  NegotiationMap negotiations;

  using TablePtr = rmf_traffic::schedule::Negotiation::TablePtr;
  using ItineraryVersion = rmf_traffic::schedule::ItineraryVersion;
  using UpdateVersion = rmf_utils::optional<ItineraryVersion>;
  using ApprovalCallbackMap = std::unordered_map<TablePtr,
      std::function<UpdateVersion()>>;
  using Approvals = std::unordered_map<Version, ApprovalCallbackMap>;
  Approvals approvals;

  Implementation(
    rclcpp::Node& node_,
    std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer_)
  : node(node_),
    viewer(std::move(viewer_)),
    negotiators(std::make_shared<NegotiatorMap>())
  {
    // TODO(MXG): Make the QoS configurable
    const auto qos = rclcpp::ServicesQoS().reliable();

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

    refusal_pub = node.create_publisher<Refusal>(
      ScheduleConflictRefusalTopicName, qos);

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

    forfeit_sub = node.create_subscription<Forfeit>(
      ScheduleConflictForfeitTopicName, qos,
      [&](const Forfeit::UniquePtr msg)
      {
        this->receive_forfeit(*msg);
      });

    forfeit_pub = node.create_publisher<Forfeit>(
      ScheduleConflictForfeitTopicName, qos);

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

    const auto table = negotiate_it->second.room.negotiation.table(
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

  void respond_to_queue(
      std::vector<TablePtr> queue,
      Version conflict_version)
  {
    while (!queue.empty())
    {
      const auto top = queue.back();
      queue.pop_back();

      if (top->defunct())
        continue;

      if (!top->submission())
      {
        const auto n_it = negotiators->find(top->participant());
        if (n_it == negotiators->end())
          continue;

        // TODO(MXG): Make this limit configurable
        if (top->version() && *top->version() > 3)
        {
          // Give up on this table at this point to avoid an infinite loop
          top->forfeit(*top->version());
          publish_forfeit(conflict_version, *top);
          continue;
        }

        std::cout << " ======= Responding to:";
        for (const auto s : top->sequence())
          std::cout << " " << s;
        std::cout << std::endl;
        const auto& negotiator = n_it->second;
        negotiator->respond(
              top->viewer(), Responder(this, conflict_version, top));
      }

      if (top->submission())
      {
        for (const auto& c : top->children())
          queue.push_back(c);
      }
      else if (const auto& parent = top->parent())
      {
        if (parent->rejected())
          queue.push_back(parent);
      }
    }
  }

  void receive_notice(const Notice& msg)
  {
    bool relevant = false;
    for (const auto p : msg.participants)
    {
      if (negotiators->find(p) != negotiators->end())
      {
        relevant = true;
        break;
      }
    }


    auto new_negotiation = Negotiation::make(
          viewer->snapshot(), msg.participants);
    if (!new_negotiation)
    {
      // TODO(MXG): This is a temporary hack to deal with situations where a
      // conflict occurs before this node is aware of other relevant schedule
      // participants. We will refuse the negotiation and then expect it to be
      // retriggered some time later. Hopefully this node will know about the
      // schedule participant by then.
      //
      // A better system will be to use ReactiveX to observe the schedule and
      // try to open the negotiation again once the participant information is
      // available.
      Refusal refusal;
      refusal.conflict_version = msg.conflict_version;
      refusal_pub->publish(refusal);

      const auto n_it = negotiations.find(msg.conflict_version);
      if (n_it != negotiations.end())
        negotiations.erase(n_it);
      return;
    }

    const auto insertion = negotiations.insert(
      {msg.conflict_version, Entry{relevant, *std::move(new_negotiation)}});

    const bool is_new = insertion.second;
    bool& participating = insertion.first->second.participating;
    auto& room = insertion.first->second.room;
    Negotiation& negotiation = room.negotiation;

    if (!is_new)
    {
      const auto& old_participants = negotiation.participants();
      for (const auto p : msg.participants)
      {
        if (old_participants.count(p) == 0)
        {
          negotiation.add_participant(p);
        }
      }
    }

    if (!relevant)
    {
      // No response needed
      return;
    }

    // TODO(MXG): Is the participating flag even relevant?
    participating = true;

    for (const auto p : msg.participants)
    {
      const auto it = negotiators->find(p);
      if (it != negotiators->end())
      {
        const auto table = negotiation.table(p, {});
        if (!table->submission())
        {
          std::cout << " ======= Responding to:";
          for (const auto s : table->sequence())
            std::cout << " " << s;
          it->second->respond(
            table->viewer(), Responder(this, msg.conflict_version, table));
        }
      }
    }

    std::vector<TablePtr> queue;
    for (const auto p : negotiation.participants())
      queue.push_back(negotiation.table(p, {}));

    respond_to_queue(queue, msg.conflict_version);
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
    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto received_table =
      negotiation.table(msg.for_participant, msg.to_accommodate);

    if (!received_table)
    {
      std::string error =
        "[rmf_traffic_ros2::schedule::Negotiation::receive_proposal] "
        "Receieved a proposal for negotiation ["
        + std::to_string(msg.conflict_version) + "] that builds on an "
        "unknown table: [";
      for (const auto p : msg.to_accommodate)
        error += " " + std::to_string(p);
      error += " " + std::to_string(msg.for_participant) + " ]";

      RCLCPP_WARN(node.get_logger(), error);
      room.cached_proposals.push_back(msg);
      return;
    }

    std::cout << " ===== [" << msg.conflict_version << "] received proposal [";
    for (const auto s : received_table->sequence())
      std::cout << " " << s;
    std::cout << " ]  :" << msg.proposal_version << ":" << std::endl;

    // We'll keep track of these negotiations whether or not we're participating
    // in them, because one of our negotiators might get added to it in the
    // future
    const bool updated =
      received_table->submit(convert(msg.itinerary), msg.proposal_version);

    if (!updated)
    {
      std::cout << " ==== proposal out of date" << std::endl;
      return;
    }

    std::vector<TablePtr> queue = room.check_cache(*negotiators);

    if (!participating)
      return;

    for (const auto& n : *negotiators)
    {
      if (const auto respond_to = received_table->respond(n.first))
        queue.push_back(respond_to);
    }

    respond_to_queue(queue, msg.conflict_version);
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

    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto table = negotiation.table(msg.table);
    if (!table)
    {
      std::string error =
        "[rmf_traffic_ros2::schedule::Negotiation::receive_proposal] "
        "Receieved a rejection for negotiation ["
        + std::to_string(msg.conflict_version) + "] for an "
        "unknown table: [";
      for (const auto p : msg.table)
        error += " " + std::to_string(p);
      error += " ]";

      RCLCPP_WARN(node.get_logger(), error);

      room.cached_rejections.push_back(msg);
      return;
    }

    std::cout << " ===== [" << msg.conflict_version << "] received rejection [";
    for (const auto s : table->sequence())
      std::cout << " " << s;
    std::cout << " ]  :" << msg.proposal_version << ":" << std::endl;

    const bool updated = table->reject(
      msg.proposal_version, msg.rejected_by, convert(msg.alternatives));

    if (!updated)
    {
      std::cout << " ===== rejection did not update anything" << std::endl;
      return;
    }

    std::vector<TablePtr> queue = room.check_cache(*negotiators);

    if (!negotiate_it->second.participating)
    {
      std::cout << " ===== we are not participating in the rejection" << std::endl;
      return;
    }

    const auto n_it = negotiators->find(table->participant());
    if (n_it != negotiators->end())
    {
      std::cout << " ===== we have a negotiator for the rejection" << std::endl;
    }
    else
    {
      std::cout << " ===== no negotiator for the rejection" << std::endl;
    }

    queue.push_back(table);

    respond_to_queue(queue, msg.conflict_version);
  }

  void receive_forfeit(const Forfeit& msg)
  {
    const auto negotiate_it = negotiations.find(msg.conflict_version);
    if (negotiate_it == negotiations.end())
    {
      // We don't need to worry about caching an unknown rejection, because it
      // is impossible for a proposal that was produced by this negotiation
      // instance to be rejected without us being aware of that proposal.
      return;
    }

    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;
    const auto table = negotiation.table(msg.table);
    if (!table)
    {
      room.cached_forfeits.push_back(msg);
      return;
    }

    table->forfeit(msg.proposal_version);

    respond_to_queue(room.check_cache(*negotiators), msg.conflict_version);
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
    auto& room = negotiate_it->second.room;
    Negotiation& negotiation = room.negotiation;

    if (participating)
    {
      std::vector<ParticipantAck> acknowledgments;

      const auto approval_callback_it = approvals.find(msg.conflict_version);
      if (msg.resolved)
      {
        assert(approval_callback_it != approvals.end());
        const auto& approval_callbacks = approval_callback_it->second;

        for (std::size_t i = 1; i <= msg.table.size(); ++i)
        {
          const auto table = negotiation.table(
            std::vector<ParticipantId>(
              msg.table.begin(), msg.table.begin()+i));

          if (!table)
            break;

          const auto approve_it = approval_callbacks.find(table);
          if (approve_it != approval_callbacks.end())
          {
            ParticipantAck p_ack;
            p_ack.participant = table->participant();
            p_ack.updating = false;
            const auto& approval_cb = approve_it->second;
            if (approval_cb)
            {
              const auto update_version = approval_cb();
              if (update_version)
              {
                p_ack.updating = true;
                p_ack.itinerary_version = *update_version;
              }
            }

            acknowledgments.emplace_back(std::move(p_ack));
          }
        }
      }
      else
      {
        ParticipantAck p_ack;
        p_ack.updating = false;
        for (const auto p : negotiation.participants())
        {
          if (negotiators->count(p) != 0)
          {
            p_ack.participant = p;
            acknowledgments.push_back(p_ack);
          }
        }
      }

      // Acknowledge that we know about this conclusion
      Ack ack;
      ack.conflict_version = msg.conflict_version;
      ack.acknowledgments = std::move(acknowledgments);

      if (ack.acknowledgments.empty())
      {
        // If we are participating in this negotiation, then the acknowledgments
        // must not be empty, or else there is a bug somewhere.

        std::string err =
            "\n !!!!!!!!!!!! Impossible situation encountered for Negotiation ["
            + std::to_string(msg.conflict_version) + "] in node ["
            + node.get_name() + "]: No approval callbacks found?? Sequence: [";
        for (const auto s : msg.table)
          err += " " + std::to_string(s);
        err += " ] ";

        if (msg.resolved)
        {
          err += "Tables with acknowledgments for this negotiation:";

          const auto& approval_callbacks = approval_callback_it->second;
          for (const auto& cb : approval_callbacks)
          {
            const auto table = cb.first;
            err += "\n -- " + ptr_to_string(table.get()) + " | (";
            if (table->version())
              err += std::to_string(*table->version());
            else
              err += "null??";
            err += ")";

            for (const auto& s : table->sequence())
              err += " " + std::to_string(s);
          }

          err += "\nCurrent relevant tables in the negotiation:";
          for (std::size_t i = 1; i <= msg.table.size(); ++i)
          {
            const auto table = negotiation.table(
                  std::vector<ParticipantId>(
                    msg.table.begin(), msg.table.begin()+i));
            err += "\n -- " + ptr_to_string(table.get()) + " |";
            if (table && table->version())
              err += " (" + std::to_string(*table->version()) + ")";
            else if (table)
              err += " (null)";

            for (std::size_t j=0; j < i; ++j)
              err += " " + std::to_string(msg.table[j]);
          }
        }
        else
        {
          err += "Negotiation participants for this node:";
          for (const auto p : negotiation.participants())
          {
            if (negotiators->count(p) != 0)
              err += " " + std::to_string(p);
          }
        }

        std::cout << "\nAll tables in the negotiation:\n";
        print_negotiation_status(msg.conflict_version, negotiation);

        std::cout << err << "\n --- Fin --- \n" << std::endl;

        assert(!ack.acknowledgments.empty());
      }

      ack_pub->publish(ack);
      // TODO(MXG): Should we consider a more robust cache cleanup strategy?

      if (approval_callback_it != approvals.end())
        approvals.erase(approval_callback_it);
    }

    // Erase these entries because the negotiation has concluded
    negotiations.erase(negotiate_it);
  }

  void publish_proposal(
    const Version conflict_version,
    const Negotiation::Table& table)
  {
    std::cout << " -|-|-|-|- publishing proposal" << std::endl;
    Proposal msg;
    msg.conflict_version = conflict_version;
    assert(table.version());
    msg.proposal_version = *table.version();

    assert(table.submission());
    msg.itinerary = convert(*table.submission());
    msg.for_participant = table.participant();

    const auto& sequence = table.sequence();
    msg.to_accommodate.reserve(sequence.size()-1);
    for (std::size_t i = 0; i < sequence.size()-1; ++i)
      msg.to_accommodate.push_back(sequence[i]);

    proposal_pub->publish(msg);
  }

  void publish_rejection(
    const Version conflict_version,
    const Negotiation::Table& table,
    const ParticipantId rejected_by,
    const Negotiation::Alternatives& alternatives)
  {
    std::cout << " -|-|-|-|- publishing rejection" << std::endl;
    Rejection msg;
    msg.conflict_version = conflict_version;
    assert(table.version());
    msg.proposal_version = *table.version();
    msg.table = table.sequence();
    msg.rejected_by = rejected_by;
    msg.alternatives = convert(alternatives);

    rejection_pub->publish(msg);
  }

  void publish_forfeit(
    const Version conflict_version,
    const Negotiation::Table& table)
  {
    std::cout << " -|-|-|-|- publishing forfeit" << std::endl;
    Forfeit msg;
    msg.conflict_version = conflict_version;
    assert(table.version());
    msg.proposal_version = *table.version();
    msg.table = table.sequence();

    forfeit_pub->publish(msg);
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
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic_ros2::schedule::Negotiaton] Attempt to register a "
        "duplicate negotiator for participant ["
        + std::to_string(for_participant) + "]");
      // *INDENT-ON*
    }

    return std::make_shared<Handle>(for_participant, negotiators);
  }
};

//==============================================================================
Negotiation::Negotiation(
  rclcpp::Node& node,
  std::shared_ptr<const rmf_traffic::schedule::Snappable> viewer)
: _pimpl(rmf_utils::make_unique_impl<Implementation>(node, std::move(viewer)))
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
