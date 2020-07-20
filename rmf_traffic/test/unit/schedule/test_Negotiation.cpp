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

#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Negotiation Unit Tests")
{
  const auto database = std::make_shared<rmf_traffic::schedule::Database>();

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

  rmf_traffic::schedule::Negotiation::TablePtr table;
  rmf_traffic::schedule::Negotiation::Table::ViewerPtr viewer;

  {
    auto negotiation = *rmf_traffic::schedule::Negotiation::make(
          database, {0, 1, 2});

    auto parent = negotiation.table(0, {});
    parent->submit({}, 1);

    table = negotiation.table(1, {0});
    CHECK_FALSE(table->defunct());

    viewer = table->viewer();
    CHECK_FALSE(viewer->defunct());

    parent->forfeit(2);
    CHECK(table->defunct());
    CHECK(viewer->defunct());

    negotiation.table(1, {})->submit({}, 1);

    table = negotiation.table(2, {1});
    CHECK_FALSE(table->defunct());

    viewer = table->viewer();
    CHECK_FALSE(viewer->defunct());
  }

  CHECK(table->defunct());
  CHECK(viewer->defunct());
}
