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
#include <rclcpp/rclcpp.hpp>

#include <iostream>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::schedule::Negotiation::VersionedKeySequence convert(
  const std::vector<rmf_traffic_msgs::msg::NegotiationKey>& from)
{
  rmf_traffic::schedule::Negotiation::VersionedKeySequence output;
  output.reserve(from.size());
  for (const auto& key : from)
    output.push_back({key.participant, key.version});

  return output;
}

//==============================================================================
std::vector<rmf_traffic_msgs::msg::NegotiationKey> convert(
  const rmf_traffic::schedule::Negotiation::VersionedKeySequence& from)
{
  std::vector<rmf_traffic_msgs::msg::NegotiationKey> output;
  output.reserve(from.size());
  for (const auto& key : from)
  {
    rmf_traffic_msgs::msg::NegotiationKey msg;
    msg.participant = key.participant;
    msg.version = key.version;
    output.push_back(msg);
  }

  return output;
}

namespace schedule {

//==============================================================================
NegotiationRoom::NegotiationRoom(rmf_traffic::schedule::Negotiation negotiation_)
: negotiation(std::move(negotiation_))
{
  // Do nothing
}

//==============================================================================
std::vector<rmf_traffic::schedule::Negotiation::TablePtr> NegotiationRoom::
check_cache(const NegotiatorMap& negotiators)
{
  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> new_tables;

  bool recheck = false;
  do
  {
    recheck = false;

    for (auto it = cached_proposals.begin(); it != cached_proposals.end(); )
    {
      const auto& proposal = *it;
      const auto search = negotiation.find(
        proposal.for_participant, convert(proposal.to_accommodate));

      if (search.deprecated())
      {
        cached_proposals.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        const bool updated = table->submit(
          rmf_traffic_ros2::convert(
            proposal.itinerary), proposal.proposal_version);
        if (updated)
          new_tables.push_back(table);

        recheck = true;
        cached_proposals.erase(it++);
      }
      else
        ++it;
    }

    for (auto it = cached_rejections.begin(); it != cached_rejections.end(); )
    {
      const auto& rejection = *it;
      const auto search = negotiation.find(convert(rejection.table));

      if (search.deprecated())
      {
        cached_rejections.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        table->reject(
          rejection.table.back().version,
          rejection.rejected_by,
          rmf_traffic_ros2::convert(rejection.alternatives));
        recheck = true;
        cached_rejections.erase(it++);
      }
      else
        ++it;
    }

    for (auto it = cached_forfeits.begin(); it != cached_forfeits.end(); )
    {
      const auto& forfeit = *it;
      const auto search = negotiation.find(convert(forfeit.table));

      if (search.deprecated())
      {
        cached_forfeits.erase(it++);
        continue;
      }

      const auto table = search.table;
      if (table)
      {
        table->forfeit(forfeit.table.back().version);
        recheck = true;
        cached_forfeits.erase(it++);
      }
      else
        ++it;
    }

  } while (recheck);

  auto remove_it = std::remove_if(new_tables.begin(), new_tables.end(),
      [](const rmf_traffic::schedule::Negotiation::TablePtr& table)
      {
        return table->forfeited() || table->defunct();
      });
  new_tables.erase(remove_it, new_tables.end());

  std::vector<rmf_traffic::schedule::Negotiation::TablePtr> respond_to;
  for (const auto& new_table : new_tables)
  {
    for (const auto& n : negotiators)
    {
      const auto participant = n.first;
      if (const auto r = new_table->respond(participant))
        respond_to.push_back(r);
    }
  }

  return respond_to;
}

//==============================================================================
void print_negotiation_status(
  rmf_traffic::schedule::Version conflict_version,
  const rmf_traffic::schedule::Negotiation& negotiation)
{
  using Negotiation = rmf_traffic::schedule::Negotiation;

  std::cout << "\n[" << conflict_version << "] Active negotiation:";
  for (const auto p : negotiation.participants())
    std::cout << " " << p;
  std::cout << std::endl;

  std::vector<Negotiation::ConstTablePtr> terminal;
  std::vector<Negotiation::ConstTablePtr> queue;
  for (const auto p : negotiation.participants())
  {
    const auto p_table = negotiation.table(p, {});
    assert(p_table);
    queue.push_back(p_table);
  }

  while (!queue.empty())
  {
    Negotiation::ConstTablePtr top = queue.back();
    queue.pop_back();

    std::vector<Negotiation::ConstTablePtr> children = top->children();
    if (children.empty())
    {
      terminal.push_back(top);
    }
    else
    {
      for (const auto& child : children)
      {
        assert(child);
        queue.push_back(child);
      }
    }
  }

  std::cout << "    Current negotiation state";
  for (const auto& t : terminal)
  {
    std::cout << "\n --";
    const bool finished = static_cast<bool>(t->submission());
    const bool rejected = t->rejected();
    const bool forfeited = t->forfeited();
    const auto sequence = t->sequence();
    for (std::size_t i = 0; i < sequence.size(); ++i)
    {
      const auto& s = sequence[i];
      if (i == t->sequence().size()-1)
      {
        if (forfeited)
          std::cout << " <" << s.participant << ":" << s.version << ">";
        else if (rejected)
          std::cout << " {" << s.participant << ":" << s.version << "}";
        else if (!finished)
          std::cout << " [" << s.participant << ":" << s.version << "]";
        else
          std::cout << " >" << s.participant << ":" << s.version << "<";
      }
      else
      {
        std::cout << " " << s.participant << ":" << s.version;
      }
    }
  }
  std::cout << "\n" << std::endl;
}

rmf_traffic_msgs::msg::NegotiationStatus assemble_negotiation_status_msg(
  rmf_traffic::schedule::Version conflict_version,
  const rmf_traffic::schedule::Negotiation& negotiation)
{
  rmf_traffic_msgs::msg::NegotiationStatus msg;
  msg.conflict_version = conflict_version;

  //add participants
  for (const auto p : negotiation.participants())
    msg.participants.push_back(p);

  //do a breadth first traversal to assemble tables into an array
  struct Node
  {
    rmf_traffic::schedule::Negotiation::ConstTablePtr table_ptr;
    int parent_index = -1;
  };
  std::vector<Node> queue;

  for (const auto p : negotiation.participants())
  { 
    const auto root_table = negotiation.table(p, {});
    assert(root_table);
    assert(root_table->parent() == nullptr);

    Node t;
    t.table_ptr = root_table;
    queue.push_back(t);
  }

  //do breadth first processing
  uint current_depth = 0;
  std::vector<Node> next_depth_queue;
  do
  {
    for (auto node : queue)
    { 
      rmf_traffic_msgs::msg::NegotiationStatusTable table;

      for (const auto v : node.table_ptr->sequence())
        table.sequence.push_back(v.participant);
      table.depth = current_depth;
      table.parent_index = node.parent_index;

      table.finished = static_cast<bool>(node.table_ptr->submission());
      table.forfeited = node.table_ptr->forfeited();
      table.rejected = node.table_ptr->rejected();
      table.ongoing = node.table_ptr->ongoing();
      table.defunct = node.table_ptr->defunct();

      auto p = node.table_ptr->proposal();
      
      for (auto submission : p)
      {
        table.proposals_id.push_back(submission.participant);
        table.proposals.push_back(convert_itinerary(submission.itinerary));
      }
#if 0 //@TODO(ddengster): We could use table->viewer() to get a better view of itineraries
      auto viewer = node.table_ptr->viewer();
      std::cout << "base_proposal count: " <<viewer->base_proposals().size() << std::endl;
      for (auto proposal : viewer->base_proposals())
      {
        std::cout << "proposal id:" << proposal.participant << std::endl;
        rmf_traffic_msgs::msg::Itinerary itin_msg;
        itin_msg.routes = convert(proposal.itinerary);
        table.base_proposals.push_back(itin_msg);
      }

      std::cout << "alternatives count: " <<viewer->alternatives().size() << std::endl;
      for (auto alternative : viewer->alternatives())
      {

        ParticipantId participant = alternative.first;
        std::cout << "id :" << participant << std::endl;
        /*auto& alternatives_ptr = alternative.second;
        
        rmf_traffic_msgs::msg::Itinerary itin_msg;
        itin_msg = convert(*alternatives_ptr);
        table.alternatives.push_back(itin_msg);*/
      }
#endif
      msg.tables.push_back(table);

      auto children = node.table_ptr->children();
      for (auto child : children)
      {
        Node c;
        c.table_ptr = child;
        c.parent_index = msg.tables.size() - 1;
        next_depth_queue.push_back(c);
      }
    }
    queue = next_depth_queue;
    next_depth_queue.clear();
    ++current_depth;

  } while (!queue.empty());
  return msg;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
