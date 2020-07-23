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

#include "utils_NegotiationRoom.hpp"

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/SimpleNegotiator.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

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

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

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
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
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
      *rmf_traffic::schedule::Negotiation::make(database, {p1.id(), p2.id()}));

    REQUIRE(negotiation->table(p1.id(), {}));
    CHECK_FALSE(negotiation->table(p1.id(), {p2.id()}));

    REQUIRE(negotiation->table(p2.id(), {}));
    CHECK_FALSE(negotiation->table(p2.id(), {p1.id()}));

    CHECK_FALSE(negotiation->ready());
//    CHECK_FALSE(negotiation->complete());

    rmf_traffic::agv::SimpleNegotiator negotiator_1{
      plan_1->get_start(),
      plan_1.get_goal(),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(
            nullptr, nullptr, rmf_utils::nullopt, rmf_utils::nullopt, wait_time)
    };

    rmf_traffic::agv::SimpleNegotiator negotiator_2{
      plan_2->get_start(),
      plan_2.get_goal(),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(
            nullptr, nullptr, rmf_utils::nullopt, rmf_utils::nullopt, wait_time)
    };

    auto next_table = negotiation->table(p1.id(), {});
    negotiator_1.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    REQUIRE(negotiation->table(p2.id(), {p1.id()}));

    next_table = negotiation->table(p2.id(), {});
    negotiator_2.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    REQUIRE(negotiation->table(p1.id(), {p2.id()}));

    next_table = negotiation->table(p1.id(), {p2.id()});
    negotiator_1.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    CHECK(negotiation->ready());

    next_table = negotiation->table(p2.id(), {p1.id()});
    negotiator_2.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

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
      *rmf_traffic::schedule::Negotiation::make(database, {p1.id(), p2.id()}));

    rmf_traffic::agv::SimpleNegotiator negotiator_1{
      rmf_traffic::agv::Plan::Start(start_time, 3, 0.0),
      rmf_traffic::agv::Plan::Goal(7),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(
            nullptr, nullptr, rmf_utils::nullopt, rmf_utils::nullopt, wait_time)
    };

    rmf_traffic::agv::SimpleNegotiator negotiator_2{
      rmf_traffic::agv::Plan::Start(start_time, 7, 0.0),
      rmf_traffic::agv::Plan::Goal(3),
      configuration,
      rmf_traffic::agv::SimpleNegotiator::Options(
            nullptr, nullptr, rmf_utils::nullopt, rmf_utils::nullopt, wait_time)
    };

    auto next_table = negotiation->table(p1.id(), {});
    negotiator_1.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    next_table = negotiation->table(p2.id(), {});
    negotiator_2.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    CHECK_FALSE(negotiation->ready());
    CHECK_FALSE(negotiation->complete());

    next_table = negotiation->table(p1.id(), {p2.id()});
    negotiator_1.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

    // ready() will be false here, because the ideal itinerary for
    // participant 2 makes it impossible for participant 1 to get out of the
    // way in enough time to avoid a conflict.
    // CHECK(negotiation->ready());

    next_table = negotiation->table(p2.id(), {p1.id()});
    negotiator_2.respond(
      next_table->viewer(),
      rmf_traffic::schedule::SimpleResponder::make(next_table));

//    CHECK(negotiation->complete());

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
SCENARIO("Multi-participant negotiation")
{
  using namespace std::chrono_literals;

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

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
  graph.add_waypoint(test_map_name, { 0.0, -5.0}); // 0
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 1
  graph.add_waypoint(test_map_name, { 0.0, 0.0}); // 2
  graph.add_waypoint(test_map_name, { 5.0,  0.0}); // 3
  graph.add_waypoint(test_map_name, { 0.0, 5.0}); // 4

  /*
   *         4
   *         |
   *         |
   *   1-----2-----3
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
        {time, 0, M_PI/2.0}, 4, configuration}});
  intentions.insert({2, NegotiationRoom::Intention{
        {time, 3, 0.0}, 1, configuration}});

  auto proposal = NegotiationRoom(database, intentions, 4.0)/*.print()*/.solve();
  REQUIRE(proposal);
}

// Helper Definitions
//==============================================================================
namespace {
using VertexId = std::string;
using IsHoldingSpot = bool;
using VertexMap = std::unordered_map<VertexId, std::pair<Eigen::Vector2d,
    IsHoldingSpot>>;

using EdgeId = std::string;
using EdgeVertices = std::pair<VertexId, VertexId>;
using IsBidirectional = bool;
using EdgeMap = std::unordered_map<EdgeId, std::pair<EdgeVertices,
    IsBidirectional>>;

using VertexIdtoIdxMap = std::unordered_map<VertexId, size_t>;

using ParticipantName = std::string;
using ParticipantIndex = size_t;

struct ParticipantConfig
{
  const rmf_traffic::Profile profile;
  const rmf_traffic::agv::VehicleTraits traits;
  const rmf_traffic::schedule::ParticipantDescription description;
};
}

// Helper Functions
//==============================================================================
// Makes graph using text ids, and returns bookkeeping that maps ids to indices
// TODO(BH): Perhaps some sort of book keeping can be introduced in Graph itself?
inline std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap>
generate_test_graph_data(std::string map_name, VertexMap vertices,
  EdgeMap edges)
{
  rmf_traffic::agv::Graph graph;
  // Maps vertex id to its corresponding id in the rmf_traffic::agv::Graph
  VertexIdtoIdxMap vertex_id_to_idx;
  size_t current_idx = 0; // vertex idxes are added in monotonic increments

  for (auto it = vertices.cbegin(); it != vertices.cend(); it++)
  {
    // Adding to rmf_traffic::agv::Graph
    graph.add_waypoint(map_name, it->second.first)
        .set_holding_point(it->second.second);

    // Book keeping
    vertex_id_to_idx.insert({it->first, current_idx});
    current_idx++;
  }

  for (auto it = edges.cbegin(); it != edges.cend(); it++)
  {
    auto source_vtx = vertex_id_to_idx[it->second.first.first];
    auto sink_vtx = vertex_id_to_idx[it->second.first.second];
    graph.add_lane(source_vtx, sink_vtx);
    if (it->second.second)
    {
      graph.add_lane(sink_vtx, source_vtx);
    }
  }
  return std::pair<rmf_traffic::agv::Graph, VertexIdtoIdxMap>(graph,
      vertex_id_to_idx);
}

inline rmf_utils::optional<rmf_traffic::schedule::Itinerary> get_participant_itinerary(
  rmf_traffic::schedule::Negotiation::Proposal proposal,
  rmf_traffic::schedule::ParticipantId participant_id)
{
  for (auto submission : proposal)
  {
    if (submission.participant == participant_id)
    {
      return submission.itinerary;
    }
  }
  return rmf_utils::nullopt;
}

// Preset Robot Configurations
// Agent a(i) generates participant p(i), instantiated in tests
//==============================================================================
// a0
const rmf_traffic::Profile a0_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a0_traits{
  {0.7, 0.3},
  {1.0, 0.45},
  a0_profile
};

const rmf_traffic::schedule::ParticipantDescription a0_description = {
  "p0",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a0_profile
};

const ParticipantConfig a0_config = {
  a0_profile, a0_traits, a0_description
};

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

// a2 - A slower version of a0 / a1
const rmf_traffic::Profile a2_profile{
  rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0)
};

const rmf_traffic::agv::VehicleTraits a2_traits{
  {0.3, 0.1},
  {0.5, 0.2},
  a2_profile
};

const rmf_traffic::schedule::ParticipantDescription a2_description = {
  "p2",
  "test_Negotiator",
  rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
  a2_profile
};

const ParticipantConfig a2_config = {
  a2_profile, a2_traits, a2_description
};

// Tests
//==============================================================================
SCENARIO("A Single Lane")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_lane";
  VertexMap vertices;
  EdgeMap edges;

  /*           single_lane_map
   *
   *       3     3     3
   *    A <-> B <-> C <-> D
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("1 Participant")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->D)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
      }
    }
  }

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    auto p2 = rmf_traffic::schedule::make_participant(a2_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->B), p1(D->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["D"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->B), p1(A->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("No Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE_FALSE(proposal);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->B), p1(B->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["B"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid proposal is found.")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B"].first);
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->B), p2(B->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["A"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["B"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p2_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid proposal is found.")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B"].first);
        auto p2_itinerary =
          get_participant_itinerary(*proposal, p2.id()).value();
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

    WHEN("Schedule:[p0(A->B)], Negotiation:[p1(D->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      rmf_traffic::agv::Planner a0_planner{
        p0_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a0_plan_0 = a0_planner.plan(
        {{time, vertex_id_to_idx["A"], 0.0}},
        {vertex_id_to_idx["B"]}
      );

      p0.set(a0_plan_0->get_itinerary());

      NegotiationRoom::Intentions intentions;

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {{time, vertex_id_to_idx["D"], 0.0}},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found.")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

    WHEN("Schedule:[p0(A->C->A)], Negotiation:[p1(D->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      rmf_traffic::agv::Planner a0_planner{
        p0_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a0_plan_0 = a0_planner.plan(
        {time, vertex_id_to_idx["A"], 0.0},
        {vertex_id_to_idx["C"]}
      );

      p0.set(a0_plan_0->get_itinerary());

      const auto a0_plan_1 = a0_planner.plan(
        {time + 8s, vertex_id_to_idx["C"], 0.0},
        {vertex_id_to_idx["A"]}
      );

      p0.extend(a0_plan_1->get_itinerary());

      NegotiationRoom::Intentions intentions;

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found.")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }
}

SCENARIO("A single lane, limited holding spaces")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_lane";
  VertexMap vertices;
  EdgeMap edges;

  /*           single_lane_map
   *
   *       3        3     3
   *    A <-> B(H) <-> C <-> D(H)
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    auto p2 = rmf_traffic::schedule::make_participant(a2_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[pO(A->C), p2(B->D)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p2_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(*proposal, p2.id()).value();
        //REQUIRE(p0_itinerary.front()->trajectory().find(rmf_traffic::Time(time + 3s))->position().segment(0, 2) != vertices["A"].first);
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
      }
    }

    // TODO(BH): A is not a holding area, however, 3 seconds after start time, the p0 is still in A
    // It is technically spinning in place and thus not "holding": but is this cheating?
    // I would expect p0 to move to B and wait there instead.
    WHEN("Schedule:[p2(B->D)], Negotiation:[pO(A->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      rmf_traffic::agv::Planner a2_planner{
        p2_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a2_plan = a2_planner.plan(
        {time, vertex_id_to_idx["B"], 0.0},
        {vertex_id_to_idx["D"]}
      );

      p2.set(a2_plan->get_itinerary());

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        //REQUIRE(p0_itinerary.front()->trajectory().find(rmf_traffic::Time(time + 3s))->position().segment(0, 2) != vertices["A"].first);
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }

}

SCENARIO("A single loop")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_loop";
  VertexMap vertices;
  EdgeMap edges;

  /*         single_loop
   *
   *         A(H) <----> B(H)
   *         ^           ^
   *         |           |
   *         v           v
   *         D(H)<------>C(H)
   */

  vertices.insert({"A", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{6.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{-0.0, -6.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[pO(A->C), p1(B->D)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[pO(C->B), p1(D->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

  }

}

SCENARIO("A single lane with an alcove holding space")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_lane_with_alcove";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *    single_lane_with_alcove
   *
   *             E(H)
   *             ^
   *             | 3
   *             |
   *        3    v    3           3
   *   A <-----> B <------> C <------> D(H)
   *
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"BE", {{"B", "E"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->E), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["E"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["E"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(E->D), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(B->D), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary = get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary = get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(E->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[p1(D->A)], Negotiation:[p0(B->D)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      rmf_traffic::agv::Planner a1_planner{
        p1_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a1_plan = a1_planner.plan(
        {time, vertex_id_to_idx["D"], 0.0},
        {vertex_id_to_idx["A"]}
      );

      p1.set(a1_plan->get_itinerary());

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
      }
    }

    WHEN("Schedule:[p1(A->D)], Negotiation:[p0(E->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      rmf_traffic::agv::Planner a1_planner{
        p1_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a1_plan = a1_planner.plan(
        {time, vertex_id_to_idx["A"], 0.0},
        {vertex_id_to_idx["D"]}
      );

      p1.set(a1_plan->get_itinerary());

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }

    WHEN("Schedule:[p1(D->A->D)], Negotiation:[p0(E->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      rmf_traffic::agv::Planner a1_planner{
        p1_planner_config,
        rmf_traffic::agv::Planner::Options{nullptr, 1s} // No route validator, holding time 1s
      };

      const auto a1_plan_0 = a1_planner.plan(
        {time, vertex_id_to_idx["D"], 0.0},
        {vertex_id_to_idx["A"]}
      );

      p1.set(a1_plan_0->get_itinerary());

      const auto a1_plan_1 = a1_planner.plan(
        {time + 16s, vertex_id_to_idx["A"], 0.0},
        {vertex_id_to_idx["D"]}
      );

      p1.extend(a1_plan_1->get_itinerary());

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }
}


SCENARIO("A single lane with a alternate one way path")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name =
    "test_single_lane_with_alternate one-way path";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *       test_single_lane_with_alternative_path
   *
   *                   E(H)<------+ F(H)
   *                   +             ^
   *                   | 3           | 3
   *                   |             |
   *              3    v       3     +       3
   *      A(H) <-----> B(H) <------> C(H) <------> D(H)
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"F", {{3.0, 3.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"CF", {{"C", "F"}, IsBidirectional(false)}});
  edges.insert({"FE", {{"F", "E"}, IsBidirectional(false)}});
  edges.insert({"EB", {{"E", "B"}, IsBidirectional(false)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(C->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions, 4.0).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions, 4.0).solve();
        REQUIRE(proposal);

        auto p0_itinerary = get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary = get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["A"].first);
      }
    }
  }

}

SCENARIO("A single lane with a alternate two way path")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name =
    "test_single_lane_with_alternate_two_way_path";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *       test_single_lane_with_alternative_two_way_path
   *
   *                   E(H) <------> F(H)
   *                   ^             ^
   *                   | 3           | 3
   *                   |             |
   *              3    v       3     v       3
   *      A(H) <-----> B(H) <------> C(H) <------> D(H)
   */

  vertices.insert({"A", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"F", {{3.0, 3.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"CF", {{"C", "F"}, IsBidirectional(true)}});
  edges.insert({"FE", {{"F", "E"}, IsBidirectional(true)}});
  edges.insert({"EB", {{"E", "B"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(C->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["A"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->D), p1(D->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary = get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary = get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["D"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0, 2) == vertices["A"].first);
      }
    }
  }

}

SCENARIO("A single loop with alcoves at each vertex")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_single_loop";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *    single_loop_with_alcoves
   *
   *      A'               B'
   *      ^ 3      6     3 ^
   *      +-->A <----> B<--+
   *          ^        ^
   *        6 |        | 6
   *          v        v
   *      +-->D<------>C<--+
   *      v 3      6     3 v
   *      D'               C'
   */

  vertices.insert({"A", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{6.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{-0.0, -6.0}, IsHoldingSpot(false)}});

  vertices.insert({"A'", {{-3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B'", {{9.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C'", {{9.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D'", {{-3.0, -6.0}, IsHoldingSpot(false)}});

  edges.insert({"AB", {{"A", "B"}, IsBidirectional(true)}});
  edges.insert({"BC", {{"B", "C"}, IsBidirectional(true)}});
  edges.insert({"CD", {{"C", "D"}, IsBidirectional(true)}});
  edges.insert({"AD", {{"A", "D"}, IsBidirectional(true)}});

  edges.insert({"AA'", {{"A", "A'"}, IsBidirectional(true)}});
  edges.insert({"BB'", {{"B", "B'"}, IsBidirectional(true)}});
  edges.insert({"CC'", {{"C", "C'"}, IsBidirectional(true)}});
  edges.insert({"DD'", {{"D", "D'"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(B'->D')]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C'"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D'"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(D'->B')]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C'"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B'"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[pO(B'->D'), p1(D'->B')]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D'"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B'"].first);
      }
    }

  }

  GIVEN("3 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    auto p2 = rmf_traffic::schedule::make_participant(a2_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[pO(A'->C'), p1(B'->D'), p2(D'->B')]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C'"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["B'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["D'"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["D'"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["B'"], // Goal Vertex
            p2_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions)/*.print()*/.solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(*proposal, p2.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C'"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["D'"].first);
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["B'"].first);
      }
    }

  }

}

SCENARIO("fan-in-fan-out bottleneck")
{
  using namespace std::chrono_literals;
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_fan_in_fan_out";
  VertexMap vertices;
  EdgeMap edges;

  /*
   *        fan_in_fan_out (All lengths are 3)
   *
   *        A       B(H)    C      D(H)    E
   *        ^       ^       ^      ^       ^
   *        |       |       |      |       |
   *        v       v       v      v       v
   *        A'<---> B'<---> C'<--->D'<---> E'
   *                        ^
   *                        |
   *                        v
   *                        F
   *                        ^
   *                        |
   *                        v
   *        V'<---> W'<---> X'<--->Y'<---> Z'
   *        ^       ^       ^      ^       ^
   *        |       |       |      |       |
   *        v       v       v      v       v
   *        V       W(H)    X      Y(H)    Z
   *
   *
   *
   */

  // Vertices
  vertices.insert({"A", {{-6.0, 6.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{-3.0, 6.0}, IsHoldingSpot(true)}});
  vertices.insert({"C", {{0.0, 6.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{3.0, 6.0}, IsHoldingSpot(true)}});
  vertices.insert({"E", {{6.0, 6.0}, IsHoldingSpot(false)}});

  vertices.insert({"A'", {{-6.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"B'", {{-3.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"C'", {{0.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"D'", {{3.0, 3.0}, IsHoldingSpot(false)}});
  vertices.insert({"E'", {{6.0, 3.0}, IsHoldingSpot(false)}});

  vertices.insert({"F", {{0.0, 0.0}, IsHoldingSpot(false)}});

  vertices.insert({"V'", {{-6.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"W'", {{-3.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"X'", {{0.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"Y'", {{3.0, -3.0}, IsHoldingSpot(false)}});
  vertices.insert({"Z'", {{6.0, -3.0}, IsHoldingSpot(false)}});

  vertices.insert({"V", {{-6.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"W", {{-3.0, -6.0}, IsHoldingSpot(true)}});
  vertices.insert({"X", {{0.0, -6.0}, IsHoldingSpot(false)}});
  vertices.insert({"Y", {{3.0, -6.0}, IsHoldingSpot(true)}});
  vertices.insert({"Z", {{6.0, -6.0}, IsHoldingSpot(false)}});

  // Edges
  edges.insert({"AA'", {{"A", "A'"}, IsBidirectional(true)}});
  edges.insert({"BB'", {{"B", "B'"}, IsBidirectional(true)}});
  edges.insert({"CC'", {{"C", "C'"}, IsBidirectional(true)}});
  edges.insert({"DD'", {{"D", "D'"}, IsBidirectional(true)}});
  edges.insert({"EE'", {{"E", "E'"}, IsBidirectional(true)}});

  edges.insert({"A'B'", {{"A'", "B'"}, IsBidirectional(true)}});
  edges.insert({"B'C'", {{"B'", "C'"}, IsBidirectional(true)}});
  edges.insert({"C'D'", {{"C'", "D'"}, IsBidirectional(true)}});
  edges.insert({"D'E'", {{"D'", "E'"}, IsBidirectional(true)}});

  edges.insert({"VV'", {{"V", "V'"}, IsBidirectional(true)}});
  edges.insert({"WW'", {{"W", "W'"}, IsBidirectional(true)}});
  edges.insert({"XX'", {{"X", "X'"}, IsBidirectional(true)}});
  edges.insert({"YY'", {{"Y", "Y'"}, IsBidirectional(true)}});
  edges.insert({"ZZ'", {{"Z", "Z'"}, IsBidirectional(true)}});

  edges.insert({"V'W'", {{"V'", "W'"}, IsBidirectional(true)}});
  edges.insert({"W'X'", {{"W'", "X'"}, IsBidirectional(true)}});
  edges.insert({"X'Y'", {{"X'", "Y'"}, IsBidirectional(true)}});
  edges.insert({"Y'Z'", {{"Y'", "Z'"}, IsBidirectional(true)}});

  edges.insert({"C'F", {{"C'", "F"}, IsBidirectional(true)}});
  edges.insert({"X'F", {{"X'", "F"}, IsBidirectional(true)}});

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  auto set_passthrough_point = [&](const std::string& wp)
  {
    graph.get_waypoint(vertex_id_to_idx[wp]).set_passthrough_point(true);
  };
  set_passthrough_point("A'");
  set_passthrough_point("B'");
  set_passthrough_point("C'");
  set_passthrough_point("D'");
  set_passthrough_point("E'");
  set_passthrough_point("F");
  set_passthrough_point("V'");
  set_passthrough_point("W'");
  set_passthrough_point("X'");
  set_passthrough_point("Y'");
  set_passthrough_point("Z'");

  GIVEN("1 Participant")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(X->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }

  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["V"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["V"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(V->E)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["V"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["E"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions)/*.print()*/.solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["E"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->X), p1(V->Z)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["X"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["V"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["X"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
      }
    }

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(X->C)")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }

  GIVEN("3 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);
    auto p2 = rmf_traffic::schedule::make_participant(a2_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(C->X)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["V"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["C"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["X"], // Goal Vertex
            p2_planner_config // Planner Configuration ( Preset )
          }
        });

      // We don't run this test in debug mode because it takes a long time
      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions, 2.0)/*.print()*/.solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(*proposal, p2.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["V"].first);
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["X"].first);
      }
    }
    WHEN("Schedule:[], Negotiation:[p0(A->Z), p1(E->V), p2(X->C)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};
      rmf_traffic::agv::Planner::Configuration p2_planner_config{graph,
        a2_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["Z"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["E"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["V"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p2.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["X"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["C"], // Goal Vertex
            p2_planner_config // Planner Configuration ( Preset )
          }
        });

      // We don't run this test in debug mode because it takes a long time
      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions, 2.3)/*.print()*/.solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        auto p1_itinerary =
          get_participant_itinerary(*proposal, p1.id()).value();
        auto p2_itinerary =
          get_participant_itinerary(*proposal, p2.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["Z"].first);
        REQUIRE(p1_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["V"].first);
        REQUIRE(p2_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["C"].first);
      }
    }
  }
}

#ifdef NDEBUG
// We do not run this test in Debug mode because it takes a long time to run
// due to the high branching factor
SCENARIO("Fully connected graph of 10 vertices")
{
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const std::string test_map_name = "test_fully_connected_graph_10_vertices";
  VertexMap vertices;
  EdgeMap edges;

  vertices.insert({"A", {{0.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"B", {{3.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"C", {{6.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"D", {{9.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"E", {{12.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"F", {{15.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"G", {{18.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"H", {{21.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"I", {{24.0, 0.0}, IsHoldingSpot(false)}});
  vertices.insert({"J", {{27.0, 0.0}, IsHoldingSpot(false)}});

  std::string vtxs = "ABCDEFGHIJ";
  for (char& v_source : vtxs)
  {
    for (char& v_dest : vtxs)
    {
      if (v_source == v_dest)
        continue;

      std::string v_source_str(1, v_source);
      std::string v_dest_str(1, v_dest);
      // Bidirectional is set to false since we double add anyway
      edges.insert({v_source_str + v_dest_str, {{v_source_str, v_dest_str}, IsBidirectional(
              false)}});
    }
  }

  auto graph_data = generate_test_graph_data(test_map_name, vertices, edges);
  auto graph = graph_data.first;
  auto vertex_id_to_idx = graph_data.second;

  GIVEN("1 Participant")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->J)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["J"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      THEN("Valid Proposal is found")
      {
        auto proposal = NegotiationRoom(database, intentions).solve();
        REQUIRE(proposal);

        auto p0_itinerary =
          get_participant_itinerary(*proposal, p0.id()).value();
        REQUIRE(p0_itinerary.back()->trajectory().back().position().segment(0,
          2) ==
          vertices["J"].first);
      }
    }
  }

  // Proposal not found.
  GIVEN("2 Participants")
  {
    auto p0 = rmf_traffic::schedule::make_participant(a0_config.description,
        database);
    auto p1 = rmf_traffic::schedule::make_participant(a1_config.description,
        database);

    WHEN("Schedule:[], Negotiation:[p0(A->J), p1(J->A)]")
    {
      const auto time = std::chrono::steady_clock::now();
      rmf_traffic::agv::Planner::Configuration p0_planner_config{graph,
        a0_config.traits};
      rmf_traffic::agv::Planner::Configuration p1_planner_config{graph,
        a1_config.traits};

      NegotiationRoom::Intentions intentions;
      intentions.insert({
          p0.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["A"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["J"], // Goal Vertex
            p0_planner_config // Planner Configuration ( Preset )
          }
        });

      intentions.insert({
          p1.id(),
          NegotiationRoom::Intention{
            {time, vertex_id_to_idx["J"], 0.0},  // Time, Start Vertex, Initial Orientation
            vertex_id_to_idx["A"], // Goal Vertex
            p1_planner_config // Planner Configuration ( Preset )
          }
        });

      // We don't run this test in debug mode because it takes a long time
      THEN("No valid proposal is found")
      {
        // The AGV planner assumes that waypoints are connected with simple
        // straight lines. Since all waypoints are colinear, it is impossible
        // for two vehicles to cross over each other.
        auto proposal = NegotiationRoom(database, intentions, 1.1).solve();
        CHECK_FALSE(proposal);
      }
    }
  }
}
#endif // NDEBUG
