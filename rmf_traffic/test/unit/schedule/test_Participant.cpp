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

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

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

SCENARIO("Test Participant")
{
  // Create a database
  rmf_traffic::schedule::Database db;
  rmf_traffic::schedule::Version dbv = 0;
  CHECK(db.latest_version() == dbv);

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

  REQUIRE(db.participant_ids().count(p1.id()) == 1);
  CHECK(db.latest_version() == ++dbv);
  REQUIRE(db.inconsistencies().begin() != db.inconsistencies().end());
  REQUIRE(db.get_itinerary(p1.id()));
  CHECK(db.get_itinerary(p1.id())->empty());

  const auto time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory t1;
  t1.insert(time, {0, 0, 0}, {0, 0, 0});
  t1.insert(time, {0, 10, 0}, {0, 0, 0});

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
}