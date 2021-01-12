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

#ifndef SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNING_HPP
#define SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNING_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>

#include <memory>
#include <mutex>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
struct Conditions
{
  std::vector<agv::Planner::Start> starts;
  agv::Planner::Goal goal;
  agv::Planner::Options options;
};

//==============================================================================
struct Issues
{
  using BlockedNodes = std::unordered_map<std::shared_ptr<void>, Time>;
  using BlockerMap = std::unordered_map<schedule::ParticipantId, BlockedNodes>;

  BlockerMap blocked_nodes;
  bool interrupted = false;
  bool disconnected = false;
};

//==============================================================================
struct State
{
  Conditions conditions;
  Issues issues;
  std::optional<double> ideal_cost;

  class Internal
  {
  public:

    virtual std::optional<double> cost_estimate() const = 0;

    virtual std::size_t queue_size() const = 0;

    virtual std::size_t expansion_count() const = 0;

    virtual ~Internal() = default;
  };

  rmf_utils::impl_ptr<Internal> internal;
  std::size_t popped_count = 0;
};

//==============================================================================
struct PlanData
{
  std::vector<Route> routes;
  std::vector<agv::Plan::Waypoint> waypoints;
  agv::Planner::Start start;
  double cost;
};

//==============================================================================
class Interface
{
public:

  virtual State initiate(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) const = 0;

  virtual std::optional<PlanData> plan(State& state) const = 0;

  virtual std::vector<schedule::Itinerary> rollout(
    const Duration span,
    const Issues::BlockedNodes& nodes,
    const Planner::Goal& goal,
    const Planner::Options& options,
    std::optional<std::size_t> max_rollouts) const = 0;

  virtual const Planner::Configuration& get_configuration() const = 0;

  class Debugger
  {
  public:

    virtual const Planner::Debug::Node::SearchQueue& queue() const = 0;

    virtual const std::vector<Planner::Debug::ConstNodePtr>&
    expanded_nodes() const = 0;

    virtual const std::vector<Planner::Debug::ConstNodePtr>&
    terminal_nodes() const = 0;

    virtual ~Debugger() = default;
  };

  virtual std::unique_ptr<Debugger> debug_begin(
    const std::vector<Planner::Start>& starts,
    Planner::Goal goal,
    Planner::Options options) const = 0;

  virtual std::optional<PlanData> debug_step(Debugger& debugger) const = 0;

  virtual ~Interface() = default;
};

//==============================================================================
using InterfacePtr = std::shared_ptr<const Interface>;

//==============================================================================
InterfacePtr make_planner_interface(Planner::Configuration config);

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNING_HPP
