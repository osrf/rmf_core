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

#include "src/rmf_traffic/schedule/debug_Database.hpp"
#include "src/rmf_traffic/schedule/debug_Participant.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

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
namespace {

using RouteId = rmf_traffic::RouteId;
using ConstRoutePtr = rmf_traffic::ConstRoutePtr;

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

std::unordered_map<RouteId, ConstRoutePtr> convert_itinerary(
    rmf_traffic::schedule::Writer::Input input)
{
  std::unordered_map<RouteId, ConstRoutePtr> itinerary;
  itinerary.reserve(input.size());

  for (const auto& item : input)
  {
    const auto result = itinerary.insert(std::make_pair(item.id, item.route));
    assert(result.second);
  }
  return itinerary;
}

inline void CHECK_ITINERARY(
    const rmf_traffic::schedule::Participant& p,
    const rmf_traffic::schedule::Database& db)
{
  using Debug = rmf_traffic::schedule::Database::Debug;

  REQUIRE(db.get_itinerary(p.id()));

  auto db_iti = convert_itinerary(
      Debug::get_itinerary(db, p.id()).value());
  auto p_iti = convert_itinerary(p.itinerary());
  REQUIRE(db_iti.size() == p_iti.size());

  for (const auto& item : p_iti)
  {
    const auto db_it = db_iti.find(item.first);
    REQUIRE(db_it != db_iti.end());
    CHECK(item.second->map() == db_it->second->map());
    CHECK_EQUAL_TRAJECOTRY(item.second->trajectory(), db_it->second->trajectory());
  }
}

} // namespace anonymous

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
  using Route = rmf_traffic::Route;

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
  REQUIRE(p1.itinerary().size() == 0);

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

  rmf_traffic::Trajectory t3;
  t3.insert(time + 22s, {10, 10 ,0}, {0, 0, 0});
  t3.insert(time + 32s, {10, 0 , 0}, {0, 0, 0});
  REQUIRE(t3.size() == 2);

  GIVEN ("Changes: S")
  {
    auto route_id = p1.set({Route{"test_map", t1}});
    CHECK(route_id == std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.last_route_id() == 0);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    WHEN("When participant itinerary is set with an empty itinerary")
    {
      // THe itinerary should not change
      route_id = p1.set({});
      CHECK(route_id == 0);
      CHECK(p1.itinerary().size() == 1);
      // The databse version should not be incremented
      CHECK(db.latest_version() == dbv);
      CHECK_ITINERARY(p1, db);
    }
  }

  GIVEN("Changes:: SS")
  {
    auto route_id = p1.set({Route{"test_map", t1}});
    CHECK(route_id == std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.last_route_id() == 0);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    route_id = p1.set({
        Route{"test_map_2", t1},
        Route{"test_map_3", t2}
    });

    CHECK(route_id == 0);
    // The RouteIds continue from previous set()
    CHECK(p1.last_route_id() == 2);
    CHECK(p1.itinerary().size() == 2);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: X")
  {
    auto route_id = p1.extend({Route{"test_map", t1}});
    CHECK(route_id == std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.last_route_id() == 0);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: SX")
  {
    auto route_id = p1.set({Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    // Extend itinerary with two new routes
    std::vector<Route> additional_routes;
    additional_routes.emplace_back(Route{"test_map", t2});
    additional_routes.emplace_back(Route{"test_map_2", t1});

    route_id = p1.extend(additional_routes);
    CHECK(route_id == 0);
    CHECK(p1.itinerary().size() == 3);
    CHECK(p1.last_route_id() == 2);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: D")
  {
    p1.delay(time, 5s);
    CHECK(p1.last_route_id() ==
        std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.itinerary().size() == 0);

    // We do not need to transmit a delay when the itinerary is empty
    CHECK(db.latest_version() == dbv);

    CHECK(p1.last_route_id() ==
        std::numeric_limits<rmf_traffic::RouteId>::max());
  }

  GIVEN("Changes: SD")
  {
    p1.set({Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    // We make a copy of the itinerary for comparison after delay
    const auto old_itinerary = p1.itinerary();

    const auto delay_duration = 5s;
    p1.delay(time, delay_duration);
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(old_itinerary.front().id == p1.itinerary().front().id);
    CHECK(old_itinerary.front().route->map() ==
        p1.itinerary().front().route->map());

    auto old_it = old_itinerary.front().route->trajectory().begin();
    auto new_it = p1.itinerary().front().route->trajectory().begin();

    for (; new_it != p1.itinerary().front().route->trajectory().end();
        new_it++, old_it++)
    {
      CHECK((new_it->time() - (old_it->time() + delay_duration)).count()
          == Approx(0.0));
    }

    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: E")
  {
    // Erasing RouteId not in the itinerary
    p1.erase({0});
    CHECK(p1.last_route_id() ==
        std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.itinerary().size() == 0);
    // No change in database version
    CHECK(db.latest_version() == dbv);
  }

  GIVEN("Changes: SE")
  {
    p1.set({Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(p1.last_route_id() == 0);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    p1.erase({p1.last_route_id()});
    CHECK(p1.itinerary().size() == 0);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: C")
  {
    // Clearing an empty itinerary
    p1.clear();
    CHECK(p1.last_route_id() ==
        std::numeric_limits<rmf_traffic::RouteId>::max());
    CHECK(p1.itinerary().size() == 0);
    // No change in database version
    CHECK(db.latest_version() == dbv);
  }

  GIVEN("Changes: SC")
  {
    p1.set({Route{"test_map", t1}, Route{"test_map_2", t2}});
    REQUIRE(p1.itinerary().size() == 2);
    CHECK(p1.last_route_id() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    p1.clear();
    CHECK(p1.itinerary().size() == 0);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: s")
  {
    writer.drop_packets = true;

    p1.set({{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());
  }

  GIVEN("Changes sS")
  {
    writer.drop_packets = true;

    p1.set({{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    writer.drop_packets = false;

    p1.set({Route{"test_map", t2}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.get_itinerary(p1.id())->size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK(db.inconsistencies().begin()->participant == p1.id());
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);

    rectifier.rectify();
    // There is no need to fix anything, because the last change was a
    // nullifying change.
    CHECK(db.latest_version() == dbv);
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes sDdS")
  {
    writer.drop_packets = true;

    p1.set({{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    writer.drop_packets = false;

    p1.delay(time, 10s);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.inconsistencies().size() == 1);
    CHECK(db.inconsistencies().begin()->ranges.size() == 1);
    CHECK(db.inconsistencies().begin()->ranges.last_known_version() ==
          rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    writer.drop_packets = true;

    p1.delay(time, 10s);
    CHECK(db.latest_version() == dbv);
    CHECK(db.inconsistencies().begin()->ranges.last_known_version() + 1 ==
          rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    writer.drop_packets = false;

    p1.set({{"test_map", t2}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 1);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
    CHECK(db.inconsistencies().begin()->ranges.last_known_version() ==
          rmf_traffic::schedule::Participant::Debug::get_itinerary_version(p1));

    rectifier.rectify();
    // There is no need to fix anything, because the last change was a
    // nullifying change.
    CHECK(db.latest_version() == dbv);
    CHECK_ITINERARY(p1, db);
  }

  GIVEN("Changes: sE")
  {
    writer.drop_packets = true;

    // Set the itinerary
    p1.set({Route{"test_map", t1}, Route{"test_map", t2}});
    CHECK(db.latest_version() == dbv);

    writer.drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map_2", t2}});
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 0);
    CHECK(db.inconsistencies().begin()->participant == p1.id());
    CHECK(db.inconsistencies().begin()->ranges.size() != 0);

    // Fix inconsistencies
    rectifier.rectify();
    CHECK(db.latest_version() == ++(++dbv));
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: SxX")
  {
    p1.set({Route{"test_map", t1}});

    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    // Tell the writer to start dropping packets
    writer.drop_packets = true;

    p1.extend({Route{"test_map_2", t1}});

    // Check that the database version did not change
    CHECK(db.latest_version() == dbv);

    writer.drop_packets = false;

    p1.extend({Route{"test_map_3", t1}});

    // Check that the database version still did not change
    CHECK(db.latest_version() == dbv);

    // Check that the database now sees that we have an inconsistency
    REQUIRE(db.inconsistencies().size() > 0);
    CHECK(db.inconsistencies().begin()->participant == p1.id());
    CHECK(db.inconsistencies().begin()->ranges.size() != 0);

    // Tell the rectifier to fix the inconsistencies
    rectifier.rectify();

    // Now the database should have updated with both changes
    CHECK(db.latest_version() == ++(++dbv));
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 3);
    const auto itinerary = db.get_itinerary(p1.id());
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: sddxeX")
  {
    writer.drop_packets = true;

    // Set the participant itinerary
    p1.set({Route{"test_map", t1}});
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    // Add a delay to the itinerary
    p1.delay(time, 1s);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    // Add a second delay to the itinerary
    p1.delay(time, 1s);
    CHECK(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}, Route{"test_map", t3}});
    CHECK(p1.itinerary().size() == 3);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    // Erase the third route in the trajectory
    REQUIRE(p1.last_route_id() == 2);
    p1.erase({p1.last_route_id()});
    CHECK(p1.itinerary().size() == 2);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    writer.drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map_2", t3}});
    CHECK(p1.itinerary().size() == 3);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->empty());

    // Check for inconsistencies
    REQUIRE(db.inconsistencies().size() > 0);
    CHECK(db.inconsistencies().begin()->participant == p1.id());
    const auto inconsistency = db.inconsistencies().begin();
    CHECK(inconsistency->ranges.size() == 1);
    CHECK(inconsistency->ranges.last_known_version() == 5);
    CHECK(inconsistency->ranges.begin()->lower == 0);
    CHECK(inconsistency->ranges.begin()->upper == 4);

    // Fix inconsistencies
    rectifier.rectify();
    dbv += 6;
    CHECK(db.latest_version() == dbv);
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: SdDxX")
  {
    // Set the participant itinerary
    p1.set({Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    writer.drop_packets = true;

    // Add a delay
    p1.delay(time, 1s);
    CHECK(db.latest_version() == dbv);

    writer.drop_packets = false;

    // Add a delay
    p1.delay(time, 1s);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.inconsistencies().size() > 0);
    auto inconsistency = db.inconsistencies().begin();
    CHECK(inconsistency->participant == p1.id());
    REQUIRE(inconsistency->ranges.size() > 0);
    CHECK(inconsistency->ranges.last_known_version() == 2);
    CHECK(inconsistency->ranges.begin()->lower == 1);
    CHECK(inconsistency->ranges.begin()->upper == 1);

    writer.drop_packets = true;

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}});
    CHECK(p1.itinerary().size() == 2);
    CHECK(db.latest_version() == dbv);

    writer.drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map", t3}});
    REQUIRE(db.inconsistencies().size() > 0);
    CHECK(db.inconsistencies().begin()->participant == p1.id());
    inconsistency = db.inconsistencies().begin();
    // We expect two ranges of inconsistencies
    REQUIRE(inconsistency->ranges.size() > 1);
    CHECK(inconsistency->ranges.last_known_version() == 4);
    auto it = inconsistency->ranges.begin();
    CHECK(it->lower == 1);
    CHECK(it->upper == 1);
    ++it;
    REQUIRE(it != inconsistency->ranges.end());
    CHECK(it->lower == 3);
    CHECK(it->upper == 3);

    // Fix inconsistencies
    rectifier.rectify();
    dbv += 4;
    CHECK(db.latest_version() == dbv);
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Changes: ScX")
  {
    // Set the participant itinerary
    p1.set({Route{"test_map", t1}});
    REQUIRE(p1.itinerary().size() == 1);
    CHECK(db.latest_version() == ++dbv);
    CHECK_ITINERARY(p1, db);

    writer.drop_packets = true;

    // Clear the itinerary
    p1.clear();
    REQUIRE(p1.itinerary().size() == 0);
    CHECK(db.latest_version() == dbv);
    REQUIRE(db.get_itinerary(p1.id()));
    CHECK(db.get_itinerary(p1.id())->size() == 1);

    writer.drop_packets = false;

    // Extend the itinerary
    p1.extend({Route{"test_map", t2}, Route{"test_map", t3}});
    REQUIRE(p1.itinerary().size() == 2);
    CHECK(db.latest_version() == dbv);

    // Check for inconsistencies
    REQUIRE(db.inconsistencies().size() == 1);
    auto inconsistency = db.inconsistencies().begin();
    CHECK(inconsistency->participant == p1.id());
    REQUIRE(inconsistency->ranges.size() > 0);
    CHECK(inconsistency->ranges.last_known_version() == 2);
    CHECK(inconsistency->ranges.begin()->lower == 1);
    CHECK(inconsistency->ranges.begin()->upper == 1);

    // Fix inconsistency
    rectifier.rectify();
    dbv += 2;
    CHECK(db.latest_version() == dbv);
    CHECK_ITINERARY(p1, db);
    CHECK(db.inconsistencies().begin()->ranges.size() == 0);
  }

  GIVEN("Participant unregisters")
  {

    rmf_traffic::schedule::ParticipantId p2_id;

    {
      auto p2 = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription{
        "participant 2",
        "test_Participant",
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        rmf_traffic::Profile{shape}
      },
      writer,
      &rectifier);

      p2_id = p2.id();
      CHECK(db.latest_version() == ++dbv);
      CHECK(db.participant_ids().size() == 2);
      CHECK(db.get_participant(p2_id));
      REQUIRE(db.get_itinerary(p2_id));
      REQUIRE(db.inconsistencies().size() == 2);

      bool found = false;
      for (const auto& inconsistency : db.inconsistencies())
      {
        if (inconsistency.participant == p2_id)
        {
          found = true;
          CHECK(inconsistency.ranges.size() == 0);
        }
      }
      REQUIRE(found);
    }

    // Participant p2 should be unregistered when it is out of scope
    CHECK(db.latest_version() == ++dbv);
    CHECK(db.participant_ids().size() == 1);
    CHECK(db.get_participant(p2_id) == nullptr);
    CHECK_FALSE(db.get_itinerary(p2_id).has_value());
    CHECK(db.inconsistencies().size() == 1);
  }

}
