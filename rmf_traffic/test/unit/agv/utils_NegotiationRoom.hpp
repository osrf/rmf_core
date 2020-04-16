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

#ifndef RMF_TRAFFIC__TEST__UNIT__AGV__UTILS_NEGOTIATIONROOM_HPP
#define RMF_TRAFFIC__TEST__UNIT__AGV__UTILS_NEGOTIATIONROOM_HPP

#include <rmf_traffic/agv/Negotiator.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_utils/catch.hpp>

#include <unordered_map>
#include <future>
#include <iostream>

//==============================================================================
class NegotiationRoom
{
public:

  using ParticipantId = rmf_traffic::schedule::ParticipantId;
  using Negotiator = rmf_traffic::agv::SimpleNegotiator;
  using Negotiation = rmf_traffic::schedule::Negotiation;
  using Responder = rmf_traffic::schedule::SimpleResponder;

  struct Intention
  {
    std::vector<rmf_traffic::agv::Planner::Start> start;
    rmf_traffic::agv::Planner::Goal goal;
    rmf_traffic::agv::Planner::Configuration configuration;

    Intention(
      std::vector<rmf_traffic::agv::Planner::Start> starts_,
      rmf_traffic::agv::Planner::Goal goal_,
      rmf_traffic::agv::Planner::Configuration config_)
    : start(std::move(starts_)),
      goal(std::move(goal_)),
      configuration(std::move(config_))
    {
      // Do nothing
    }

    Intention(
      rmf_traffic::agv::Planner::Start start_,
      rmf_traffic::agv::Planner::Goal goal_,
      rmf_traffic::agv::Planner::Configuration config_)
    : start({std::move(start_)}),
      goal(std::move(goal_)),
      configuration(std::move(config_))
    {
      // Do nothing
    }
  };

  using Intentions = std::unordered_map<ParticipantId, Intention>;

  NegotiationRoom(
    const rmf_traffic::schedule::Viewer& viewer,
    Intentions intentions,
    const bool print_failures_ = false)
  : negotiators(make_negotiations(intentions)),
    negotiation(std::make_shared<Negotiation>(
        viewer, get_participants(intentions))),
    print_failures(print_failures_)
  {
    // Do nothing
  }

  rmf_utils::optional<Negotiation::Proposal> solve()
  {
    std::vector<Negotiation::TablePtr> queue;
    for (const auto& p : negotiation->participants())
    {
      const auto table = negotiation->table(p, {});
      negotiators.at(p).respond(table, Responder(negotiation, p, {}));
      queue.push_back(table);
    }

    while (!queue.empty())
    {
      const auto top = queue.back();
      queue.pop_back();

      for (auto& n : negotiators)
      {
        const auto participant = n.first;
        auto& negotiator = n.second;
        const auto respond_to = top->respond(participant);
        if (respond_to)
        {
          bool interrupt = false;
          auto result = std::async(
            std::launch::async,
            [&]()
            {
              negotiator.respond(
                respond_to,
                Responder(negotiation, respond_to->sequence()),
                &interrupt);
            });

          using namespace std::chrono_literals;
          // Give up if a solution is not found within 10 seconds.
          if (result.wait_for(10s) != std::future_status::ready)
            interrupt = true;

          result.wait();

          if (print_failures && !respond_to->submission())
          {
            std::cout << "Failed to make a submission for [";
            for (const auto p : respond_to->sequence())
              std::cout << " " << p;
            std::cout << " ]" << std::endl;
          }
          queue.push_back(respond_to);
        }
      }
    }

    CHECK(negotiation->complete());
    if (!negotiation->ready())
      return rmf_utils::nullopt;

    return negotiation->evaluate(
      rmf_traffic::schedule::QuickestFinishEvaluator())->proposal();
  }

  static std::unordered_map<ParticipantId, Negotiator> make_negotiations(
    const std::unordered_map<ParticipantId, Intention>& intentions)
  {
    std::unordered_map<ParticipantId, Negotiator> negotiators;
    for (const auto& entry : intentions)
    {
      const auto participant = entry.first;
      const auto& intention = entry.second;
      negotiators.insert(
        std::make_pair(
          participant,
          rmf_traffic::agv::SimpleNegotiator(
            intention.start, intention.goal, intention.configuration)));
    }

    return negotiators;
  }

  static std::vector<ParticipantId> get_participants(
    const std::unordered_map<ParticipantId, Intention>& intentions)
  {
    std::vector<ParticipantId> participants;
    participants.reserve(intentions.size());
    for (const auto& entry : intentions)
      participants.push_back(entry.first);

    return participants;
  }

  std::unordered_map<ParticipantId, Negotiator> negotiators;
  std::shared_ptr<rmf_traffic::schedule::Negotiation> negotiation;
  bool print_failures = false;
};

#endif // UTILS_NEGOTIATIONROOM_HPP
