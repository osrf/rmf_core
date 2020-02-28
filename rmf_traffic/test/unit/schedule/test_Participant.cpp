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

#include "utils_Database.hpp"

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include "../../../src/rmf_traffic/schedule/debug_Database.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

#include <unordered_map>

class FaultyWriter : public rmf_traffic::schedule::Writer

{
public:

  FaultyWriter(rmf_traffic::schedule::Database& database)
    : _database(database)
  {
    // Do nothing
  }

  bool drop_packets = false;

  void set(
      rmf_traffic::schedule::ParticipantId participant,
      const Input& itinerary,
      rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database.set(participant, itinerary, version);
  }

  void extend(
      rmf_traffic::schedule::ParticipantId participant,
      const Input& routes,
      rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database.extend(participant, routes, version);
  }

  void delay(
      rmf_traffic::schedule::ParticipantId participant,
      rmf_traffic::Time from,
      rmf_traffic::Duration delay,
      rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database.delay(participant, from, delay, version);
  }

  void erase(
      rmf_traffic::schedule::ParticipantId participant,
      rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database.erase(participant, version);
  }

  void erase(
      rmf_traffic::schedule::ParticipantId participant,
      const std::vector<rmf_traffic::RouteId>& routes,
      rmf_traffic::schedule::ItineraryVersion version) final
  {
    if (drop_packets)
      return;

    _database.erase(participant, routes, version);
  }

  rmf_traffic::schedule::ParticipantId register_participant(
      rmf_traffic::schedule::ParticipantDescription participant_info) final
  {
    // We assume participant registration is done over a reliable connection
    return _database.register_participant(participant_info);
  }

  void unregister_participant(
      rmf_traffic::schedule::ParticipantId participant) final
  {
    _database.set_current_time(std::chrono::steady_clock::now());
    _database.unregister_participant(participant);
  }

private:
  rmf_traffic::schedule::Database& _database;
};

//=============================================================================

inline void CHECK_EQUAL_TRAJECOTRY(
    const rmf_traffic::Trajectory& t1,
    const rmf_traffic::Trajectory& t2)
{
  REQUIRE(t1.size() == t2.size());
  REQUIRE(t1.start_time());
  REQUIRE(t1.finish_time());
  REQUIRE(t2.start_time());
  REQUIRE(t2.finish_time());
  
  auto t1_it = t1.begin();
  auto t2_it = t2.begin();

  for(; t1_it != t1.end(); ++t1_it, ++t2_it)
  {
    REQUIRE((t1_it->position() - t2_it->position()).norm() == Approx(0.0).margin(1e-6));
    REQUIRE((t1_it->velocity() - t2_it->velocity()).norm() == Approx(0.0).margin(1e-6));
    REQUIRE((t1_it->time() - t2_it->time()).count() == Approx(0.0));
  }
}

std::unordered_map<rmf_traffic::RouteId, rmf_traffic:ConstRoutePtr> convert_itinerary(
    rmf_traffic::schedule::Writer::Input& input)
{
  using RouteId = rmf_traffic::RouteId;
  using ConstRoutePtr = rmf_traffic::ConstRoutePtr;

  std::unordered_map<RouteId, ConstRoutePtr> itinerary;
  itinerary.reserve(input.size());

  for (const auto& item : input)
    itinerary.insert(std::make_pair(item.id, item.route));
  
  return itinerary;
}

inline void CHECK_ITINERARY(
    const rmf_traffic::schedule::Participant& p,
    const rmf_traffic::schedule::Database& db,
    std::string* parent = nullptr)
{
  using Debug = rmf_traffic::schedule::Database::Debug;

  REQUIRE(db.get_itinerary(p.id()));

  // const auto db_iti = db.get_itinerary(p.id()).value();
  const auto db_iti = Debug::get_itinerary(db, p.id()).value();
  const auto& p_iti = p.itinerary();
  REQUIRE(db_iti.size() == p_iti.size());

  // Create a map for each itinerary

  
  // for (std::size_t i = 0; i < db_iti.size(); i++)
  // {
  //   const auto& p_route = *p_iti[i].route;
  //   const auto& db_route = *db_iti[i].route;
    
  //   // Debugging 
  //   auto _parent = parent ? *parent : "";
  //   std::cout<< "Checking Item: " << i << " for " << _parent << std::endl;
  //   std::cout<< "P Map: " << p_route.map() << " DB Map: " << db_route.map()
  //     << std::endl;

  //   CHECK(p_route.map() == db_route.map());
  //   // CHECK_EQUAL_TRAJECOTRY(p_route.trajectory(), db_route.trajectory());  
  // }
}

//=============================================================================

// Symbol key for the test scenarios below
//
// S: accepted set change
// s: dropped set change
//
// X: accepted extend change
// x: dropped extend change
//
// D: accepted delay change
// d: dropped delay change
//
// E: accepted earse change
// e: dropped erase change
//
// C: accepted clear change
// c: dropped clear change

//=============================================================================

SCENARIO("Test Participant")
{
  using namespace std::chrono_literals;

  // Create a database
  rmf_traffic::schedule::Database db;
  rmf_traffic::schedule::Version dbv = 0;
  CHECK(db.latest_version() == dbv);
  REQUIRE(db.participant_ids().size() == 0);
  REQUIRE(db.get_participant(0) == nullptr);
  REQUIRE(db.inconsistencies().size() == 0);

  // Create a faulty writer
  FaultyWriter writer{db};

  // Create a simple rectifier factory
  rmf_traffic::schedule::DatabaseRectificationRequesterFactory rectifier{db};

  // Create a shape
  const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);

  // Create a participant
  auto p1 = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "participant 1",
      "test_Participant",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      rmf_traffic::Profile{shape}
    },
    writer,
    &rectifier);

  CHECK(p1.id() == 0);
  const auto& description = p1.description();
  CHECK(description.name() == "participant 1");
  CHECK(description.owner() == "test_Participant");
  CHECK(description.responsiveness() == rmf_traffic::schedule
    ::ParticipantDescription::Rx::Responsive);

  CHECK(db.participant_ids().size() == 1);
  REQUIRE(db.participant_ids().count(p1.id()) == 1);
  CHECK(db.latest_version() == ++dbv);
  REQUIRE(db.inconsistencies().begin() != db.inconsistencies().end());
  REQUIRE(db.get_itinerary(p1.id()));
  CHECK(db.get_itinerary(p1.id())->empty());

  const auto time = std::chrono::steady_clock::now();
  rmf_traffic::Trajectory t1;
  t1.insert(time, {0, 0, 0}, {0, 0, 0});
  t1.insert(time + 10s, {0, 10, 0}, {0, 0, 0});
  REQUIRE(t1.size() == 2);

  rmf_traffic::Trajectory t2;
  t2.insert(time + 12s, {0, 10, 0}, {0, 0, 0});
  t2.insert(time + 20s, {10, 10, 0}, {0, 0, 0});
  REQUIRE(t2.size() == 2);

  GIVEN ("Changes: S")
  {
    auto route_id = p1.set({rmf_traffic::Route{"test_map", t1}});
    CHECK(route_id == std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.last_route_id() == 0);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes:: SS")
  {
    auto route_id = p1.set({rmf_traffic::Route{"test_map", t1}});
    CHECK(route_id == std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.last_route_id() == 0);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    route_id = p1.set({
      rmf_traffic::Route{"test_map_2", t1},
      rmf_traffic::Route{"test_map_3", t2}
    });

    CHECK(route_id == 0);
    // The RouteIds continue from previous set()
    CHECK(p1.last_route_id() == 2);
    CHECK(p1.itinerary().size() == 2);
    CHECK(db.latest_version() == ++dbv);
    std::string parent = "SS";
    CHECK_ITINERARY(p1, db, &parent);
  }

  GIVEN("Changes: s")
  {
    writer.drop_packets = true;

    p1.set({rmf_traffic::Route{"test_map", t1}});
    CHECK(p1.itinerary().size() > 0);

    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());
  }

  GIVEN("Changes: SeE")
  {
    p1.set({rmf_traffic::Route{"test_map", t1}});

    CHECK(db.latest_version() == ++dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 1);

    // Tell the writer to start dropping packets
    writer.drop_packets = true;

    p1.extend({rmf_traffic::Route{"test_map_2", t1}});

    // Check that the database version did not change
    CHECK(db.latest_version() == dbv);

    writer.drop_packets = false;

    p1.extend({rmf_traffic::Route{"test_map_3", t1}});

    // Check that the database version still did not change
    CHECK(db.latest_version() == dbv);

    // Check that the database now sees that we have an inconsistency
    CHECK(db.inconsistencies().begin()->ranges.size() != 0);

    // Tell the rectifier to fix the inconsistencies
    rectifier.rectify();

    // Now the database should have updated with both changes
    CHECK(db.latest_version() == ++(++dbv));
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 3);
    const auto itinerary = db.get_itinerary(p1.id());
  }
}
