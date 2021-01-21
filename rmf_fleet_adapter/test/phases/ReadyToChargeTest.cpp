
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

#include "MockAdapterFixture.hpp"

#include <phases/ReadyToCharge.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_charger_msgs::msg::ChargerState;
using rmf_charger_msgs::msg::ChargerRequest;

SCENARIO_METHOD(MockAdapterFixture, "Charger Request Phase", "[phases]")
{
  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<ChargerRequest> received_requests;


  const auto info = add_robot();
  const auto& context = info.context;

  auto dispenser_request_pub = adapter->node()->create_publisher<ChargerRequest>(
    IngestorRequestTopicName, 10);
  
  auto request_guid = "test/charger";
  auto pending_phase = ReadyToCharge::make(
    context,
    request_guid
  );
  auto active_phase = pending_phase->begin();

  WHEN("it is started")
  {
    auto rcl_subscription = adapter->node()->create_subscription<ChargerRequest>(
    ChargerRequestTopicName,
    10,
    [&](ChargerRequest::UniquePtr ingestor_request)
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests.emplace_back(*ingestor_request);
      received_requests_cv.notify_all();
    });
    
    std::condition_variable status_updates_cv;
    std::list<Task::StatusMsg> status_updates;
    /*auto sub = active_phase->observe().subscribe(
      [&](const auto& status)
      {
        std::unique_lock<std::mutex> lk(m);
        status_updates.emplace_back(status);
        status_updates_cv.notify_all();
      });

    THEN("it should send charger request")
    {
      std::unique_lock<std::mutex> lk(m);
      if (received_requests.empty())
        received_requests_cv.wait(lk, [&]() { return !received_requests.empty(); });
      REQUIRE(received_requests.size() == 1);
    }*/

    THEN("it should continuously charger request")
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]() { return received_requests.size() >= 3; });
    }


    //sub.unsubscribe();
  }

  WHEN
}

} // namespace test
} // namespace phases
} // namespace rmf_fleet_adapter
