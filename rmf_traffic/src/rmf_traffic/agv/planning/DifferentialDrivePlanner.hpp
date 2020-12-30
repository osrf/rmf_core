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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEPLANNER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEPLANNER_HPP

#include "../internal_planning.hpp"

#include "DifferentialDriveHeuristic.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class DifferentialDrivePlanner : public Interface
{
public:

  DifferentialDrivePlanner(Planner::Configuration config);

  State initiate(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) const final;

  std::optional<Plan> plan(State& state) const final;

  std::vector<schedule::Itinerary> rollout(
    const Duration span,
    const Issues::BlockedNodes& nodes,
    const Planner::Goal& goal,
    const Planner::Options& options,
    std::optional<std::size_t> max_rollouts) const final;

  const Planner::Configuration& get_configuration() const final;

  std::unique_ptr<Debugger> debug_begin(
      const std::vector<Planner::Start>& starts,
      Planner::Goal goal,
      Planner::Options options) const final;

  std::optional<Plan> debug_step(Debugger& debugger) const final;

  std::optional<double> compute_heuristic(const Planner::Start& start) const;

private:
  Planner::Configuration _config;
  std::shared_ptr<const Supergraph> _supergraph;
  CacheManagerPtr<DifferentialDriveHeuristic> _cache;
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic


#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__DIFFERENTIALDRIVEPLANNER_HPP
