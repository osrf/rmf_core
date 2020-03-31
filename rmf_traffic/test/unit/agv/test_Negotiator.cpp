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

#include <unordered_map>

#include <iostream>

//==============================================================================
void print_proposal(
    const rmf_traffic::schedule::Negotiation::Proposal& proposals)
{
  for (const auto& proposal : proposals)
  {
    std::cout << "\nparticipant " << proposal.participant << ":\n";
    const auto start_time =
        *proposal.itinerary.front()->trajectory().start_time();
    for (const auto& r : proposal.itinerary)
    {
      for (const auto& t : r->trajectory())
      {
        std::cout << "[" << rmf_traffic::time::to_seconds(t.time() - start_time)
                  << "]" << t.position().transpose() << std::endl;
      }
    }
  }
}

//==============================================================================
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
    rmf_traffic::agv::Planner::Start start;
    rmf_traffic::agv::Planner::Goal goal;
    rmf_traffic::agv::Planner::Configuration configuration;
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
          negotiator.respond(
                respond_to,
                Responder(negotiation, respond_to->sequence()));
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

//==============================================================================
SCENARIO("Multi-participant negotiation")
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
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        },
        database);

  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, { 0.0, -5.0}, true); // 0
  graph.add_waypoint(test_map_name, {-5.0,  0.0}, true); // 1
  graph.add_waypoint(test_map_name, { 0.0,  0.0}, true); // 2
//  graph.add_waypoint(test_map_name, { 5.0,  0.0}, true); // 3 // TODO(MXG): This edge case needs to be dealt with
  graph.add_waypoint(test_map_name, { 14.0,  0.0}, true); // 3
  graph.add_waypoint(test_map_name, { 0.0,  5.0}, true); // 4

  /*
   *         4
   *         |
   *         |
   *   1-----2-------3
   *         |
   *         |
   *         0
   */

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
  {
    graph.add_lane(w0, w1);
    graph.add_lane(w1, w0);
  };

  add_bidir_lane(0, 2);
  add_bidir_lane(1, 2);
  add_bidir_lane(3, 2);
  add_bidir_lane(4, 2);

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const auto time = std::chrono::steady_clock::now();

  rmf_traffic::agv::Planner::Configuration configuration{graph, traits};

  NegotiationRoom::Intentions intentions;
  intentions.insert({0, NegotiationRoom::Intention{
                       {time, 1, 0.0}, 3, configuration}});
  intentions.insert({1, NegotiationRoom::Intention{
                       {time, 4, M_PI/2.0}, 0, configuration}});
  intentions.insert({2, NegotiationRoom::Intention{
                       {time, 3, 0.0}, 1, configuration}});


  auto proposal = NegotiationRoom(database, intentions).solve();
  REQUIRE(proposal);

  //print_proposal(*proposal);
}






// (BH): Proposed Test Additions
//==============================================================================

// Helper Definitions
//==============================================================================
using VertexId = std::string;
using IsParkingSpot = bool;
using VertexMap = std::vector<std::tuple<VertexId, Eigen::Vector2d, IsParkingSpot>>;

using EdgeVertices = std::pair<VertexId, VertexId>;
using IsBidirectional = bool;
using EdgeMap = std::vector<std::pair<EdgeVertices, IsBidirectional>>;

using VertexIdtoIdxMap = std::unordered_map<VertexId, size_t>;

using ParticipantName = std::string;
using ParticipantIndex = size_t;

struct ParticipantConfig
{
  const rmf_traffic::Profile profile;
  const rmf_traffic::agv::VehicleTraits traits; 
  const rmf_traffic::schedule::ParticipantDescription description;
};

// Helper Functions
//==============================================================================
// Makes graph using text ids, and returns bookkeeping that maps ids to indices
// TODO(BH): Perhaps some sort of book keeping can be introduced in Graph itself?
std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap> 
generate_test_graph_data(std::string map_name, VertexMap vertices, EdgeMap edges)
{
  rmf_traffic::agv::Graph graph;
  // Maps vertex id to its corresponding id in the rmf_traffic::agv::Graph
  VertexIdtoIdxMap vertex_id_to_idx;
  size_t current_idx = 0; // vertex idxes are added in monotonic increments

  for(auto it = vertices.cbegin(); it != vertices.cend(); it++)
  {
    // Adding to rmf_traffic::agv::Graph
    graph.add_waypoint(map_name, std::get<1>(*it), std::get<2>(*it));
    // Book keeping
    vertex_id_to_idx.insert({std::get<0>(*it), current_idx});
    current_idx++;
  }

  for(auto it = edges.cbegin(); it != edges.cend(); it++)
  {
    auto source_vtx = vertex_id_to_idx[it->first.first];
    auto sink_vtx = vertex_id_to_idx[it->first.second];
    graph.add_lane(source_vtx, sink_vtx);
    if (it->second)
    {
      graph.add_lane(sink_vtx, source_vtx);
    }
  }
  return std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap>(graph, vertex_id_to_idx);
}

// Preset Robot Configurations
// Agent a(i) generates participant p(i), instantiated in tests
//==============================================================================
// a1
const rmf_traffic::Profile a1_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a1_traits{
  {0.7, 0.3},
  {1.0, 0.45},
  a1_profile
};

const rmf_traffic::schedule::ParticipantDescription a1_description = {
  "p1",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a1_profile
};

const ParticipantConfig a1_config = {
 a1_profile, a1_traits, a1_description
};

// Tests
//==============================================================================
SCENARIO("A Single Lane")
{
  using namespace std::chrono_literals;
  rmf_traffic::schedule::Database database; 
  const std::string test_map_name = "test_single_lane";
  VertexMap vertices;
  EdgeMap edges;

  /*    single_lane_map
   *
   *    A(p) <-> B(p) <-> C(p)
   */

  vertices.push_back({"A", {-1.0, 0.0}, IsParkingSpot(true)});
  vertices.push_back({"B", {0.0, 0.0}, IsParkingSpot(true)});
  vertices.push_back({"C", {1.0, 0.0}, IsParkingSpot(true)});

  edges.push_back({{"A", "B"}, IsBidirectional(true)});
  edges.push_back({{"B", "C"}, IsBidirectional(true)});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("1 Participant")
  {
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description, database);

    WHEN("Negotiate moving from A to C")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph, a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
          });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);
      }
    }
  }
}

