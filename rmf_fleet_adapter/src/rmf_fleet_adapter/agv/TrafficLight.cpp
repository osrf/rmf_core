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

#include "internal_TrafficLight.hpp"

#include <rmf_utils/Modular.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Negotiator::respond(
    const TableViewerPtr& table_viewer,
    const ResponderPtr& responder)
{
  const auto data = _data.lock();
  if (!data || !data->planner || !data->plan)
  {
    // If we no longer have access to the traffic light data or there is no
    // plan being followed, then we simply forfeit the negotiation.
    return responder->forfeit({});
  }

  // FIXME TODO(MXG): Reply to the negotiation here.
  assert(false);
}

//==============================================================================
void TrafficLight::UpdateHandle::Implementation::Data::update_path(
    const std::size_t version,
    const std::vector<Waypoint>& new_path)
{
  if (rmf_utils::modular(version).less_than(processing_version))
    return;

  assert(version != processing_version);

  rmf_traffic::agv::Graph graph;
  for (const auto wp : new_path)
  {
    graph.add_waypoint(wp.map_name(), wp.position().block<2,1>(0,0))
        .set_passthrough_point(!wp.yield())
        .set_holding_point(wp.yield());
  }

//  rmf_traffic::agv::Plan::Configuration config(
//        )
}

//==============================================================================
std::size_t TrafficLight::UpdateHandle::update_path(
    const std::vector<Waypoint>& new_path)
{
  const std::size_t version = ++_pimpl->received_version;
  _pimpl->data->worker.schedule(
        [version, new_path, data = _pimpl->data](const auto&)
  {
    data->update_path(version, new_path);
  });

  return version;
}

} // namespace agv
} // namespace rmf_fleet_adapter
