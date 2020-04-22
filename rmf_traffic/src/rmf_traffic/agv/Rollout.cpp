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

#include <rmf_traffic/agv/Rollout.hpp>

#include "internal_Planner.hpp"

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Rollout::Implementation
{
public:

  Planner::Result result;

};

//==============================================================================
Rollout::Rollout(Planner::Result result)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(result)
             }))
{
  // Do nothing
}

//==============================================================================
std::vector<schedule::Itinerary> Rollout::expand(
    schedule::ParticipantId blocker,
    rmf_traffic::Duration span,
    const Planner::Options& options) const
{
  const auto& result = Planner::Result::Implementation::get(_pimpl->result);
  const auto& blocker_map = result.issues.blocked_nodes;

  const auto block_it = blocker_map.find(blocker);
  if (block_it == blocker_map.end())
    return {};

  if (block_it->second.empty())
    return {};

  return result.cache_mgr.get()->rollout(
        span,
        block_it->second,
        result.conditions.goal,
        options);
}

} // namespace agv
} // namespace rmf_traffic
