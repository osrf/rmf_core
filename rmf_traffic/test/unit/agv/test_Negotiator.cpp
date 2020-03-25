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

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/Negotiator.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

void print_proposal(
    const rmf_traffic::schedule::Negotiation::Proposal& proposals)
{
  for (const auto& proposal : proposals)
  {
    std::cout << "participant " << proposal.participant << ":\n";
    for (const auto& r : proposal.itinerary)
    {
      for (const auto& t : r->trajectory())
      {
        std::cout << t.position().transpose() << std::endl;
      }
    }
  }
}

SCENARIO("Test Plan Negotiation Between Two Participants")
{
  using namespace std::chrono_literals;

  rmf_traffic::schedule::Database database;

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0)
  };

  auto p1 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant 1",
          "test_Negotiator",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p2 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant 2",
          "test_Negotiator",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  auto p3 = rmf_traffic::schedule::make_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant 3",
          "test_Negotiator",
          rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
          profile
        },
        database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0}, true);  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}, true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}, true); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}, true); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}, true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                  10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
  {
    graph.add_lane(w0, w1);
    graph.add_lane(w1, w0);
  };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);

  // Create a conflict
  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const rmf_traffic::Duration wait_time = 1s;

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};
  rmf_traffic::agv::Planner planner{
    configuration,
    rmf_traffic::agv::Planner::Options{nullptr, wait_time}
  };

  auto start_time = std::chrono::steady_clock::now();

  const auto plan_1 = planner.plan(
        rmf_traffic::agv::Plan::Start(start_time, 3, 0.0),
        rmf_traffic::agv::Plan::Goal(7));
  REQUIRE(plan_1);
  p1.set(plan_1->get_itinerary());

  const auto plan_2 = planner.plan(
        rmf_traffic::agv::Plan::Start(start_time, 0, 90.0*M_PI/180.0),
        rmf_traffic::agv::Plan::Goal(10));
  REQUIRE(plan_2);
  p2.set(plan_2->get_itinerary());

  bool has_conflict = false;
  for (const auto& r1 : plan_1->get_itinerary())
  {
    for (const auto& r2 : plan_2->get_itinerary())
    {
      if (rmf_traffic::DetectConflict::between(
            profile, r1.trajectory(),
            profile, r2.trajectory()))
      {
        has_conflict = true;
        break;
      }
    }

    if (has_conflict)
      break;
  }

  CHECK(has_conflict);

  WHEN("Participants Crossing Paths")
  {
    auto negotiation = std::make_shared<rmf_traffic::schedule::Negotiation>(
          rmf_traffic::schedule::Negotiation{database, {p1.id(), p2.id()}});

    REQUIRE(negotiation->table(p1.id(), {}));
    CHECK_FALSE(negotiation->table(p1.id(), {p2.id()}));

    REQUIRE(negotiation->table(p2.id(), {}));
    CHECK_FALSE(negotiation->table(p2.id(), {p1.id()}));

    CHECK_FALSE(negotiation->ready());
    CHECK_FALSE(negotiation->complete());

    rmf_traffic::agv::SimpleNegotiator negotiator_1{
      plan_1->get_start(),
      plan_1->get_goal(),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(wait_time)
    };

    rmf_traffic::agv::SimpleNegotiator negotiator_2{
      plan_2->get_start(),
      plan_2->get_goal(),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(wait_time)
    };

    negotiator_1.respond(
          negotiation->table(p1.id(), {}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p1.id(), {}));

    REQUIRE(negotiation->table(p2.id(), {p1.id()}));

    negotiator_2.respond(
          negotiation->table(p2.id(), {}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p2.id(), {}));

    REQUIRE(negotiation->table(p1.id(), {p2.id()}));

    negotiator_1.respond(
          negotiation->table(p1.id(), {p2.id()}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p1.id(), {p2.id()}));

    CHECK(negotiation->ready());

    negotiator_2.respond(
          negotiation->table(p2.id(), {p1.id()}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p2.id(), {p1.id()}));

    CHECK(negotiation->complete());

    auto proposals = negotiation->evaluate(
            rmf_traffic::schedule::QuickestFinishEvaluator())->proposal();
    REQUIRE(proposals.size() == 2);

    const auto& proposal_1 = proposals.at(0);
    const auto& proposal_2 = proposals.at(1);
    for (const auto& r1 : proposal_1.itinerary)
    {
      for (const auto& r2 : proposal_2.itinerary)
      {
        CHECK_FALSE(rmf_traffic::DetectConflict::between(
                      profile, r1->trajectory(),
                      profile, r2->trajectory()));
      }
    }

//    print_proposal(*proposals);
  }

  WHEN("Participants Head-to-Head")
  {
    GIVEN("No third participant")
    {
      // Intentionally do nothing
    }

    GIVEN("A third participant")
    {
      const auto plan_3 = planner.plan(
            rmf_traffic::agv::Plan::Start(start_time, 0, 90.0*M_PI/180.0),
            rmf_traffic::agv::Plan::Goal(10));
      REQUIRE(plan_3);

      p3.set(plan_3->get_itinerary());
    }

    auto negotiation = std::make_shared<rmf_traffic::schedule::Negotiation>(
          rmf_traffic::schedule::Negotiation{database, {p1.id(), p2.id()}});

    rmf_traffic::agv::SimpleNegotiator negotiator_1{
      rmf_traffic::agv::Plan::Start(start_time, 3, 0.0),
      rmf_traffic::agv::Plan::Goal(7),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(wait_time)
    };

    rmf_traffic::agv::SimpleNegotiator negotiator_2{
      rmf_traffic::agv::Plan::Start(start_time, 7, 0.0),
      rmf_traffic::agv::Plan::Goal(3),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(wait_time)
    };

    negotiator_1.respond(
          negotiation->table(p1.id(), {}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p1.id(), {}));

    negotiator_2.respond(
          negotiation->table(p2.id(), {}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p2.id(), {}));

    CHECK_FALSE(negotiation->ready());
    CHECK_FALSE(negotiation->complete());

    negotiator_1.respond(
          negotiation->table(p1.id(), {p2.id()}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p1.id(), {p2.id()}));

    // ready() will be false here, because the ideal itinerary for
    // participant 2 makes it impossible for participant 1 to get out of the
    // way in enough time to avoid a conflict.
    // CHECK(negotiation->ready());

    negotiator_2.respond(
          negotiation->table(p2.id(), {p1.id()}),
          rmf_traffic::schedule::SimpleResponder(negotiation, p2.id(), {p1.id()}));

    CHECK(negotiation->complete());

    auto proposals = negotiation->evaluate(
            rmf_traffic::schedule::QuickestFinishEvaluator())->proposal();
    REQUIRE(proposals.size() == 2);

    const auto& proposal_1 = proposals.at(0);
    const auto& proposal_2 = proposals.at(1);
    for (const auto& r1 : proposal_1.itinerary)
    {
      for (const auto& r2 : proposal_2.itinerary)
      {
        CHECK_FALSE(rmf_traffic::DetectConflict::between(
                      profile, r1->trajectory(),
                      profile, r2->trajectory()));
      }
    }

//    print_proposal(*proposals);
  }
}
