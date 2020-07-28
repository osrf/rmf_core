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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP

#include <rmf_fleet_adapter/agv/TrafficLight.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/schedule/Negotiator.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_traffic_ros2/schedule/Negotiation.hpp>

#include <rxcpp/rx.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class TrafficLight::UpdateHandle::Implementation
{
public:

  class Data
  {
  public:

    std::shared_ptr<CommandHandle> command;

    rmf_traffic::schedule::Participant itinerary;

    rmf_traffic::agv::VehicleTraits traits;

    rxcpp::schedulers::worker worker;

    std::size_t current_version = 0;

    std::vector<Waypoint> path;

    rmf_utils::optional<rmf_traffic::agv::Plan> plan;

    rmf_utils::optional<rmf_traffic::agv::Planner> planner;

    std::size_t processing_version = 0;

    void update_path(
        std::size_t version,
        const std::vector<Waypoint>& new_path);

    Data(
        std::shared_ptr<CommandHandle> command_,
        rmf_traffic::schedule::Participant itinerary_,
        rmf_traffic::agv::VehicleTraits traits_,
        rxcpp::schedulers::worker worker_)
      : command(std::move(command_)),
        itinerary(std::move(itinerary_)),
        traits(std::move(traits_)),
        worker(std::move(worker_))
    {
      // Do nothing
    }
  };

  class Negotiator : public rmf_traffic::schedule::Negotiator
  {
  public:

    Negotiator(std::shared_ptr<Data> data)
      : _data(std::move(data))
    {
      // Do nothing
    }

    void respond(
        const TableViewerPtr& table_viewer,
        const ResponderPtr& responder) final;

  private:
    std::weak_ptr<Data> _data;
  };

  std::size_t received_version = 0;

  std::shared_ptr<Data> data;

  std::shared_ptr<void> negotiation_license;

  Implementation(
      std::shared_ptr<CommandHandle> command_,
      rmf_traffic::schedule::Participant itinerary_,
      rmf_traffic::agv::VehicleTraits traits_,
      rxcpp::schedulers::worker worker_)
    : data(std::make_shared<Data>(
             std::move(command_),
             std::move(itinerary_),
             std::move(traits_),
             std::move(worker_)))
  {
    // Do nothing
  }

  static std::shared_ptr<UpdateHandle> make(
      std::shared_ptr<CommandHandle> command,
      rmf_traffic::schedule::Participant itinerary,
      rmf_traffic::agv::VehicleTraits traits,
      rxcpp::schedulers::worker worker,
      rmf_traffic_ros2::schedule::Negotiation& negotiation)
  {
    std::shared_ptr<UpdateHandle> handle = std::make_shared<UpdateHandle>();
    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
          std::move(command),
          std::move(itinerary),
          std::move(traits),
          std::move(worker));

    handle->_pimpl->negotiation_license = negotiation.register_negotiator(
          handle->_pimpl->data->itinerary.id(),
          std::make_unique<Negotiator>(handle->_pimpl->data));

    return handle;
  }

};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__INTERNAL_TRAFFICLIGHT_HPP
