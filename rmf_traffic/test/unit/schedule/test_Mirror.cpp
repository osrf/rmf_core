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
#include <rmf_traffic/geometry/Box.hpp>

#include "src/rmf_traffic/schedule/debug_Viewer.hpp"

#include <rmf_utils/catch.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>
#include <rmf_traffic/DetectConflict.hpp>

#include <unordered_map>

using namespace std::chrono_literals;

using IdMap = std::unordered_map<
    rmf_traffic::schedule::ParticipantId,
    std::unordered_set<rmf_traffic::RouteId>>;

SCENARIO("Test Mirror of a Database with two trajectories")
{

  // Creating database db and checking for empty initialziation
  rmf_traffic::schedule::Database db;
  const auto query_all = rmf_traffic::schedule::query_all();
  rmf_traffic::schedule::Patch changes =
      db.changes(query_all, rmf_utils::nullopt);
  CHECK(changes.size() == 0);
  rmf_traffic::schedule::Version dbv = 0;
  CHECK(changes.latest_version() == dbv);

  // Add two participants
  double profile_scale=1;
  const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(profile_scale, profile_scale);
  const rmf_traffic::Profile profile{shape};

  const rmf_traffic::schedule::ParticipantId p1 = db.register_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "test_participant_1",
          "test_Mirror",
          rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
          profile
        });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv1 = 0;
  rmf_traffic::RouteId rv1 = 0;

  const rmf_traffic::schedule::ParticipantId p2 = db.register_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "test_participant_2",
          "test_Mirror",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv2 = 0;
  rmf_traffic::RouteId rv2 = 0;

  // Creating Trajectories to insert
  const rmf_traffic::Time time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory t1;
  t1.insert(time, Eigen::Vector3d{-5,0,0}, Eigen::Vector3d{0,0,0});
  t1.insert(time + 10s, Eigen::Vector3d{5,0,0}, Eigen::Vector3d{0,0,0});
  REQUIRE(t1.size() == 2);

  rmf_traffic::Trajectory t2;
  t2.insert(time, Eigen::Vector3d{-5,10,0}, Eigen::Vector3d{0,0,0});
  t2.insert(time+10s, Eigen::Vector3d{5,10,0},Eigen::Vector3d{0,0,0});
  REQUIRE(t2.size() == 2);

  db.set(p1, create_test_input(rv1++, t1), iv1++);
  CHECK(db.latest_version() == ++dbv);

  db.set(p2, create_test_input(rv2++, t2), iv2++);
  CHECK(db.latest_version() == ++dbv);
  REQUIRE_FALSE(rmf_traffic::DetectConflict::between(profile, t1, profile, t2));

  rmf_traffic::schedule::Mirror mirror;
  // updating mirror
  changes = db.changes(query_all, rmf_utils::nullopt);
  CHECK(mirror.update(changes) == changes.latest_version());
  CHECK(mirror.latest_version() == changes.latest_version());

  WHEN("A trajectory is added to the database")
  {
    rmf_traffic::Trajectory t3;
    t3.insert(time, Eigen::Vector3d{-5, -10, 0}, Eigen::Vector3d{0, 0, 0});
    t3.insert(time+10s, Eigen::Vector3d{5, 10, 0},Eigen::Vector3d{0, 0, 0});

    db.extend(p1, create_test_input(rv1++, t3), iv1++);
    CHECK(db.latest_version() == ++dbv);
    CHECK_TRAJECTORY_COUNT(db, 2, 3);
    CHECK(mirror.latest_version() != db.latest_version());

    THEN("Updating the mirror should update its latest version")
    {
      changes = db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());
      CHECK_TRAJECTORY_COUNT(mirror, 2, 3);
    }
  }

  GIVEN("Create a trajectory that conflicts with the schedule")
  {
    rmf_traffic::Trajectory t3;
    t3.insert(time, Eigen::Vector3d{0, -5, 0}, Eigen::Vector3d{0, 0, 0});
    t3.insert(time+10s, Eigen::Vector3d{0, 5, 0}, Eigen::Vector3d{0, 0, 0});

    auto view = mirror.query(query_all);
    auto conflicting_trajectories =
        get_conflicting_trajectories(view, profile, t3);
    CHECK(conflicting_trajectories.size()==1);

    WHEN("Replacing conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      rmf_traffic::Trajectory t4;
      t4.insert(time, Eigen::Vector3d{-5, 0, 0}, Eigen::Vector3d{0, 0, 0});
      t4.insert(time+10s, Eigen::Vector3d{-2, 0, 0}, Eigen::Vector3d{0, 0, 0});

      db.set(p1, create_test_input(rv1++, t4), iv1++);
      CHECK(db.latest_version() == ++dbv);

      view = db.query(query_all);
      conflicting_trajectories =
          get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size() == 0);

      changes = db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
          get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size()==0);
    }

    WHEN("Erasing conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.erase(p1, {0}, iv1++);
      CHECK(db.latest_version() == ++dbv);
      changes= db.changes(query_all, mirror.latest_version());
      mirror.update(changes);
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
          get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size()==0);
    }

    WHEN("Delaying conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.delay(p1, time, 20s, iv1++);
      CHECK(db.latest_version() == ++dbv);
      changes = db.changes(query_all, mirror.latest_version());
      CHECK(mirror.update(changes) == changes.latest_version());
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
          get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size()==0);
    }

    WHEN("Culling conflicting trajectory in db and updating mirror should eliminate conflict")
    {
      db.cull(time + 11s);
      changes = db.changes(query_all, mirror.latest_version());
      CHECK(mirror.update(changes) == changes.latest_version());
      CHECK(mirror.latest_version() == db.latest_version());

      view = mirror.query(query_all);
      conflicting_trajectories =
          get_conflicting_trajectories(view, profile, t3);
      CHECK(conflicting_trajectories.size()==0);
     }
  }
}

SCENARIO("Testing specialized mirrors")
{
  rmf_traffic::schedule::Database db;
  rmf_traffic::schedule::Version dbv = 0;

  const auto query_all = rmf_traffic::schedule::query_all();
  rmf_traffic::schedule::Patch changes =
      db.changes(query_all, rmf_utils::nullopt);
  REQUIRE(changes.size() == 0);

  // Creating participants
  const double profile_scale = 1.0;
  const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Box>(profile_scale, profile_scale);
  const rmf_traffic::Profile profile{shape};

  const auto p1 = db.register_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_1",
          "test_Mirror",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile
        });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv1 = 0;
  rmf_traffic::RouteId rv1 = 0;

  const auto p2 = db.register_participant(
        rmf_traffic::schedule::ParticipantDescription{
          "participant_2",
          "test_Mirror",
          rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
          profile
        });
  CHECK(db.latest_version() == ++dbv);
  rmf_traffic::schedule::ItineraryVersion iv2 = 0;
  rmf_traffic::RouteId rv2 = 0;

  //Creating routes r1, r2, r3 in "test_map"
  const rmf_traffic::Time time = std::chrono::steady_clock::now();

  rmf_traffic::Trajectory t1;
  t1.insert(time, {-5, 0, 0}, {0, 0, 0});
  t1.insert(time + 10s, {5, 0, 0}, {0, 0, 0});
  REQUIRE(t1.size()==2);
  const auto r1 = std::make_shared<rmf_traffic::Route>("test_map", t1);

  rmf_traffic::Trajectory t2;
  t2.insert(time, {-5,10,0}, {0, 0, 0});
  t2.insert(time+11s, {5, 10, 0}, {0, 0, 0});
  REQUIRE(t2.size()==2);
  const auto r2 = std::make_shared<rmf_traffic::Route>("test_map", t2);

  rmf_traffic::Trajectory t3;
  t3.insert(time+11s, {0, -5, 0}, {0, 0, 0});
  t3.insert(time+20s, {0,5,0}, {0, 0, 0});
  REQUIRE(t3.size()==2);
  const auto r3 = std::make_shared<rmf_traffic::Route>("test_map", t3);

  // creating routes r4 and r5 in "test_map_2"
  rmf_traffic::Trajectory t4;
  t4.insert(time, {-5, 0, 0}, {0, 0, 0});
  t4.insert(time + 10s, {5, 0, 0}, {0, 0, 0});
  REQUIRE(t4.size()==2);
  const auto r4 = std::make_shared<rmf_traffic::Route>("test_map_2", t4);

  rmf_traffic::Trajectory t5;
  t5.insert(time, {-5, 10, 0}, {0, 0, 0});
  t5.insert(time+10s, {5, 10, 0}, {0, 0, 0});
  REQUIRE(t5.size() == 2);
  const auto r5 = std::make_shared<rmf_traffic::Route>("test_map_2", t5);

  db.set(p1, {{rv1++, r1}, {rv1++, r2}, {rv1++, r4}}, iv1++);
  CHECK(db.latest_version() == ++dbv);
  db.set(p2, {{rv2++, r3}, {rv2++, r5}}, iv2++);
  CHECK(db.latest_version() == ++dbv);

  // Check that there are no conflicts between the routes on test_map
  CHECK_FALSE(rmf_traffic::DetectConflict::between(profile, t1, profile, t2));
  CHECK_FALSE(rmf_traffic::DetectConflict::between(profile, t1, profile, t3));
  CHECK_FALSE(rmf_traffic::DetectConflict::between(profile, t2, profile, t3));

  // Check that there is no conflict between the routes on test_map_2
  CHECK_FALSE(rmf_traffic::DetectConflict::between(profile, t4, profile, t5));

  GIVEN("Query patch with spacetime region overlapping with t1")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    rmf_traffic::schedule::Query query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(10.0, 1.0);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);
    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+10s, spaces);
    query.spacetime().regions()->push_back(region);

    rmf_traffic::schedule::Patch changes =
        db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p1);
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->id == 0);
  }

  GIVEN("Query patch with spacetime region overlapping with t2")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();
    auto query= rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
    tf.translate(Eigen::Vector2d{0.0, 10.0});
    rmf_traffic::geometry::Space space(final_box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);
    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+10s, spaces);
    query.spacetime().regions()->push_back(region);

    //querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
        db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p1);
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->id == 1);
  }

//  // COMMENTED DUE TO NON-DETERMINISTIC BEHAVIOR OF FCL

//  GIVEN("Query patch with rotated spacetime region overlapping with t1")
//  {
//    auto time = std::chrono::steady_clock::now();
//    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

//    rmf_traffic::schedule::Query query= rmf_traffic::schedule::make_query(0,{});
//    REQUIRE(query.spacetime().regions() != nullptr);

//    //creating space to add to region
//    const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
//    const auto final_box = rmf_traffic::geometry::make_final_convex(box);
//    tf.rotate(Eigen::Rotation2Dd(M_PI_2));
//    rmf_traffic::geometry::Space space(final_box,tf);
//    std::vector<rmf_traffic::geometry::Space> spaces;
//    spaces.push_back(space);
//    //creating a region with time bounds and spaces that overlap with trajectory
//    rmf_traffic::Region region("test_map",time, time+10s,spaces);
//    query.spacetime().regions()->push_back(region);

//    //querying for Patch using defined spacetime query
//    rmf_traffic::schedule::Database::Patch changes= db.changes(query);
//    REQUIRE(changes.size() >0);
//    CHECK(changes.size() == 1);
//    CHECK(changes.begin()->id() == 1);
//  }

  GIVEN("Query patch with spacetime region overlapping with t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    //creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(1.0, 1.0);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    // creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time+10s, time+20s, spaces);
    query.spacetime().regions()->push_back(region);

    // querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
        db.changes(query, rmf_utils::nullopt);
    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 1);
    CHECK(changes.begin()->participant_id() == p2);
    REQUIRE(changes.begin()->additions().items().size() == 1);
    CHECK(changes.begin()->additions().items().begin()->id == 0);
  }

  GIVEN("Query patch with spacetime region overlapping with t1 and t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    // creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(10, 10);
    rmf_traffic::geometry::Space space(box,tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+20s,spaces);
    query.spacetime().regions()->push_back(region);

    //querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
        db.changes(query, rmf_utils::nullopt);

    // Only t1 and t3 are within range of the region. t2 runs along the line y=10,
    // which is outside the range of a 10x10 box that is centered at the origin,
    // because that box will only reach out to y=5.
    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 2);

    IdMap ids;
    for (const auto& c : changes)
    {
      for (const auto& i : c.additions().items())
        ids[c.participant_id()].insert(i.id);
    }

    IdMap expected_ids;
    expected_ids[p1] = {0};
    expected_ids[p2] = {0};

    CHECK(ids == expected_ids);
  }

  GIVEN("Query patch with spacetime region overlapping with t1, t2 & t3")
  {
    auto time = std::chrono::steady_clock::now();
    Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

    auto query = rmf_traffic::schedule::make_query({});
    REQUIRE(query.spacetime().regions() != nullptr);

    // creating space to add to region
    const auto box = rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(10, 20);
    rmf_traffic::geometry::Space space(box, tf);
    std::vector<rmf_traffic::geometry::Space> spaces;
    spaces.push_back(space);

    //creating a region with time bounds and spaces that overlap with trajectory
    rmf_traffic::Region region("test_map", time, time+20s,spaces);
    query.spacetime().regions()->push_back(region);

    // querying for Patch using defined spacetime query
    rmf_traffic::schedule::Patch changes =
        db.changes(query, rmf_utils::nullopt);

    REQUIRE(changes.size() > 0);
    CHECK(changes.size() == 2);

    IdMap ids;
    for (const auto& c : changes)
    {
      for (const auto& i : c.additions().items())
        ids[c.participant_id()].insert(i.id);
    }

    IdMap expected_ids;
    expected_ids[p1] = {0, 1};
    expected_ids[p2] = {0};
  }
}
