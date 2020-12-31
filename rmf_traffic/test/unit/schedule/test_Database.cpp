/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include "src/rmf_traffic/schedule/debug_Viewer.hpp"
#include "src/rmf_traffic/schedule/debug_Database.hpp"

#include <rmf_utils/catch.hpp>

using namespace std::chrono_literals;

SCENARIO("Test Database Conflicts")
{
  GIVEN("A Database with single trajectory")
  {
    rmf_traffic::schedule::Database db;
    rmf_traffic::schedule::Version dbv = 0;
    CHECK(db.latest_version() == dbv);

    //check for empty instantiation
    const rmf_traffic::schedule::Query query_all =
      rmf_traffic::schedule::query_all();

    rmf_traffic::schedule::Patch changes =
      db.changes(query_all, rmf_utils::nullopt);

    REQUIRE(changes.size() == 0);
    CHECK(changes.latest_version() == 0);

    //adding simple two-point trajecotry
    const double profile_scale = 1.0;
    const rmf_traffic::Time time = std::chrono::steady_clock::now();
    rmf_traffic::geometry::Box shape(profile_scale, profile_scale);
    rmf_traffic::geometry::FinalConvexShapePtr final_shape =
      rmf_traffic::geometry::make_final_convex(shape);

    const rmf_traffic::Profile profile{final_shape};

    const auto p1 = db.register_participant(
      rmf_traffic::schedule::ParticipantDescription{
        "test_participant",
        "test_Database",
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        profile
      });
    CHECK(db.latest_version() == ++dbv);

    rmf_traffic::Trajectory t1;
    t1.insert(time, Eigen::Vector3d{-5, 0, 0}, Eigen::Vector3d{0, 0, 0});
    t1.insert(time + 10s, Eigen::Vector3d{5, 0, 0}, Eigen::Vector3d{0, 0, 0});
    REQUIRE(t1.size() == 2);

    rmf_traffic::schedule::ItineraryVersion iv1 = 0;
    rmf_traffic::RouteId rv1 = 0;
    db.set(p1, create_test_input(rv1++, t1), iv1++);
    CHECK(db.latest_version() == ++dbv);

    // query for the changes after version 0
    changes = db.changes(query_all, 0);
    CHECK(changes.registered().size() == 1);
    CHECK(changes.unregistered().size() == 0);

    REQUIRE(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p1);
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->id == 0);
    CHECK_FALSE(changes.cull());
    CHECK(changes.latest_version() == db.latest_version());
    CHECK_TRAJECTORY_COUNT(db, 1, 1);

    // WHEN("Another Trajectory is inserted")
    {
      rmf_traffic::Trajectory t2;
      t2.insert(time, Eigen::Vector3d{0, -5, 0}, Eigen::Vector3d{0, 0, 0});
      t2.insert(time+10min, Eigen::Vector3d{0, 5, 0}, Eigen::Vector3d{0, 0, 0});
      REQUIRE(t2.size() == 2);

      db.extend(p1, create_test_input(rv1++, t2), iv1++);
      CHECK(db.latest_version() == ++dbv);
      CHECK_TRAJECTORY_COUNT(db, 1, 2);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 1);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      CHECK(changes.begin()->additions().items().size() == 2);
      CHECK(changes.begin()->delays().size() == 0);
      CHECK(changes.begin()->erasures().ids().size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      REQUIRE(changes.begin()->additions().items().size() == 1);
      CHECK(changes.begin()->additions().items().begin()->id == 1);
      CHECK(changes.begin()->delays().size() == 0);
      CHECK(changes.begin()->erasures().ids().size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());
    }

    // WHEN("Trajectory is delayed")
    {
      //introduce a delay after the first waypoint in t1
      const auto duration = 5s;
      db.delay(p1, duration, iv1++);
      CHECK(db.latest_version() == ++dbv);
      CHECK_TRAJECTORY_COUNT(db, 1, 2);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 1);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      REQUIRE(changes.begin()->additions().items().size() == 2);
      CHECK(changes.begin()->delays().size() == 0);
      CHECK(changes.begin()->erasures().ids().size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      CHECK(changes.begin()->additions().items().size() == 0);
      REQUIRE(changes.begin()->delays().size() == 1);
      CHECK(changes.begin()->delays().begin()->duration() == duration);
      CHECK(changes.begin()->erasures().ids().size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());
    }

    // WHEN("Trajectory is erased")
    {
      db.erase(p1, {1}, iv1++);
      CHECK(db.latest_version() == ++dbv);
      CHECK_TRAJECTORY_COUNT(db, 1, 1);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 1);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      REQUIRE(changes.begin()->additions().items().size() == 1);
      CHECK(changes.begin()->additions().items().begin()->id == 0);
      CHECK(changes.begin()->delays().size() == 0);
      CHECK(changes.begin()->erasures().ids().size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      CHECK(changes.begin()->additions().items().size() == 0);
      CHECK(changes.begin()->delays().size() == 0);
      REQUIRE(changes.begin()->erasures().ids().size() == 1);
      CHECK(changes.begin()->erasures().ids().front() == 1);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());
    }

    // WHEN("Itinerary is erased")
    {
      db.erase(p1, iv1++);
      CHECK(db.latest_version() == ++dbv);
      CHECK_TRAJECTORY_COUNT(db, 1, 0);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 1);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      REQUIRE(changes.size() == 1);
      CHECK(changes.begin()->participant_id() == p1);
      CHECK(changes.begin()->additions().items().size() == 0);
      CHECK(changes.begin()->delays().size() == 0);
      REQUIRE(changes.begin()->erasures().ids().size() == 1);
      CHECK(changes.begin()->erasures().ids().front() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());
    }

    // WHEN("Schedule is culled")
    {
      const auto cull_time = time + 5min;
      CHECK(rmf_traffic::schedule::Database::Debug::current_entry_history_count(
          db) == 2);
      const auto v = db.cull(cull_time);
      CHECK(rmf_traffic::schedule::Database::Debug::current_entry_history_count(
          db) == 1);
      CHECK(db.latest_version() == ++dbv);
      CHECK(v == db.latest_version());
      CHECK_TRAJECTORY_COUNT(db, 1, 0);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 1);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      REQUIRE(changes.cull());
      CHECK(changes.cull()->time() == cull_time);
      CHECK(changes.latest_version() == db.latest_version());
    }

    // WHEN("Unregistering a participant")
    {
      auto current_time = time + 10min;
      db.set_current_time(current_time);
      db.unregister_participant(p1);
      CHECK(db.latest_version() == ++dbv);
      CHECK(rmf_traffic::schedule::Database::Debug::current_removed_participant_count(
          db) == 1);

      // query from the start
      changes = db.changes(query_all, rmf_utils::nullopt);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 1);
      CHECK(changes.size() == 0);
      CHECK_FALSE(changes.cull());
      CHECK(changes.latest_version() == db.latest_version());

      // cull the unregistered participant
      const auto cull_time = current_time + 10min;
      db.cull(cull_time);
      CHECK(db.latest_version() == ++dbv);
      CHECK(rmf_traffic::schedule::Database::Debug::current_removed_participant_count(
          db) == 0);

      // query the diff
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      REQUIRE(changes.cull());
      CHECK(changes.cull()->time() == cull_time);
      CHECK(changes.latest_version() == db.latest_version());

      // query the diff minus two
      changes = db.changes(query_all, db.latest_version()-1);
      CHECK(changes.registered().size() == 0);
      CHECK(changes.unregistered().size() == 0);
      CHECK(changes.size() == 0);
      REQUIRE(changes.cull());
      CHECK(changes.cull()->time() == cull_time);
      CHECK(changes.latest_version() == db.latest_version());
    }
  }
}