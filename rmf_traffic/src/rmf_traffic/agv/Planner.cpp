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

#include "GraphInternal.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <queue>

namespace rmf_traffic {
namespace agv {

//==============================================================================
struct ExpansionContext
{
  const Graph::Implementation& graph;
  const std::size_t final_waypoint;
  const double* const final_orientation;
  const VehicleTraits& traits;
  const Trajectory::ConstProfilePtr profile;
  const Duration holding_time;
  const schedule::Viewer& viewer;
};

//==============================================================================
Eigen::Vector3d to_3d(const Eigen::Vector2d& p, const double w)
{
  return Eigen::Vector3d(p[0], p[1], w);
}

//==============================================================================
struct BaseExpander
{
  const ExpansionContext context;

  const Eigen::Vector2d p_final;

  struct Node;
  // TODO(MXG): Consider replacing this with std::unique_ptr
  using NodePtr = std::shared_ptr<Node>;

  struct Node
  {
    double final_cost_estimate;

    double current_cost;

    std::size_t waypoint;

    double orientation;

    Trajectory trajectory_from_parent;

    NodePtr parent;
  };

  BaseExpander(ExpansionContext context)
    : context(std::move(context)),
      p_final(context.graph.waypoints[context.final_waypoint].get_location())
  {
    // Do nothing
  }

  double estimate_remaining_cost(std::size_t waypoint) const
  {
    const auto& p0 = context.graph.waypoints[waypoint].get_location();
    // TODO(MXG): When we want to support multiple floors, this estimate should
    // be improved by accounting for the need to use lifts.
    return (p_final - p0).norm()/context.traits.linear().get_nominal_velocity();
  }

  NodePtr make_initial_node(
      const std::size_t waypoint,
      const double orientation,
      const Time time) const
  {
    Trajectory initial(context.graph.waypoints[waypoint].get_map_name());
    initial.insert(
          time, context.profile,
          to_3d(context.graph.waypoints[waypoint].get_location(), orientation),
          Eigen::Vector3d::Zero());

    return std::make_shared<Node>(
          Node{estimate_remaining_cost(waypoint), 0.0, waypoint, orientation,
               initial, nullptr});
  }

  bool is_finished(const NodePtr& node) const
  {
    if(node->waypoint != context.final_waypoint)
      return false;

    if(context.final_orientation)
    {
      if(std::abs(node->orientation - *context.final_orientation) < 1e-6)
        return false;
    }

    return true;
  }
};

//==============================================================================
struct DifferentialDriveExpander : BaseExpander
{
  DifferentialDriveExpander(ExpansionContext ec)
    : BaseExpander(std::move(ec))
//    : ExpansionContext(std::move(ec))
  {
    // Do nothing
  }

  // TODO(MXG): Consider changing this to unique_ptr, or just using Node
  // directly in the vector
  using NodePtr = std::shared_ptr<Node>;

  struct Compare
  {
    bool operator()(const NodePtr& a, const NodePtr& b)
    {
      // Note(MXG): The priority queue puts the greater value first, so we
      // reverse the arguments in this comparison.
      return b->final_cost_estimate < a->final_cost_estimate;
    }
  };

  using SearchQueue =
      std::priority_queue<NodePtr, std::vector<NodePtr>, Compare>;

  void expand(const NodePtr& node, SearchQueue& queue)
  {
    const std::vector<std::size_t>& lanes =
        context.graph.lanes_from[node->waypoint];

    for(const std::size_t l : lanes)
    {
      const Graph::Lane& lane = context.graph.lanes[l];
      assert(lane.entry().waypoint_index() == node->waypoint);



      // TODO(MXG): Finish this expansion function
      // * Calculate an expansion from this waypoint to each of the other
      //   waypoints that connect to this waypoint by at least one lane
      // * When possible, calculate a multi-waypoint expansion (e.g. when there
      //   are straight lines connecting a sequence of waypoints, and no
      //   velocity constraint between them)
      // * Check each expansion against the schedule to make sure that it is
      //   indeed a valid expansion
    }
  }

};

//==============================================================================
struct HolonomicExpander
{
  const ExpansionContext context;

  HolonomicExpander(ExpansionContext ec)
    : context(std::move(ec))
  {
    // Do nothing
  }

  struct Node
  {
    double cost_estimate;
  };

  // TODO(MXG): Consider changing this to unique_ptr
  using NodePtr = std::shared_ptr<Node>;

  struct Compare
  {
    bool operator()(const NodePtr& a, const NodePtr& b)
    {
      // Note(MXG): The priority queue puts the greater value first, so we
      // reverse the arguments in this comparison.
      return b->cost_estimate < a->cost_estimate;
    }
  };

};

//==============================================================================
template<typename Expander>
std::vector<Trajectory> reconstruct_trajectory(
    const typename Expander::NodePtr& finish_node)
{
  using NodePtr = typename Expander::NodePtr;
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while(node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  std::vector<Trajectory> trajectories;
  trajectories.push_back(
      Trajectory{node_sequence.back()->trajectory_from_parent.get_map_name()});

  // We exclude the first node in the sequence, because it contains a dummy
  // trajectory which is not helpful.
  const auto stop_it = --node_sequence.rend();
  for(auto it = node_sequence.rbegin(); it != stop_it; ++it)
  {
    Trajectory& last_trajectory = trajectories.back();
    const Trajectory& next_trajectory = (*it)->trajectory_from_parent;
    if(next_trajectory.get_map_name() == last_trajectory.get_map_name())
    {
      for(const auto& segment : next_trajectory)
      {
        last_trajectory.insert(
              segment.get_finish_time(),
              segment.get_profile(),
              segment.get_finish_position(),
              segment.get_finish_velocity());
      }
    }
    else
    {
      trajectories.push_back(next_trajectory);
    }
  }

  return trajectories;
}

//==============================================================================
template<class Expander>
bool search(
    const Graph::Implementation& graph,
    const Time initial_time,
    const std::size_t initial_waypoint,
    const double initial_orientation,
    const std::size_t final_waypoint,
    const double* const final_orientation,
    const Planner::Options& options,
    std::vector<Trajectory> solution)
{
  using NodePtr = typename Expander::NodePtr;
  using SearchQueue = typename Expander::SearchQueue;

  Expander expander(ExpansionContext{
          graph, final_waypoint, final_orientation,
                      options.get_vehicle_traits(),
                      options.get_vehicle_traits().get_profile(),
                      options.get_minimum_holding_time(),
                      options.get_schedule_viewer()});

  SearchQueue queue;
  queue.push(expander.make_initial_node(
               initial_waypoint, initial_orientation, initial_time));

  while(!queue.empty())
  {
    NodePtr top = queue.top();
    queue.pop();

    if(expander.is_finished(top))
    {
      solution = reconstruct_trajectory<Expander>(top);
      return true;
    }

    expander.expand(top, queue);
  }

  // Could not find a solution!
  return false;
}

//==============================================================================
class Planner::Options::Implementation
{
public:

  VehicleTraits traits;

  Graph graph;

  const schedule::Viewer* viewer;

  Duration min_hold_time;
};

//==============================================================================
Planner::Options::Options(VehicleTraits vehicle_traits,
    Graph graph,
    const schedule::Viewer& viewer, Duration min_hold_time)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(vehicle_traits),
               std::move(graph),
               &viewer,
               min_hold_time}))
{
  // Do nothing
}

//==============================================================================
auto Planner::Options::set_vehicle_traits(VehicleTraits traits) -> Options&
{
  _pimpl->traits = std::move(traits);
  return *this;
}


//==============================================================================
VehicleTraits& Planner::Options::get_vehicle_traits()
{
  return _pimpl->traits;
}

//==============================================================================
const VehicleTraits& Planner::Options::get_vehicle_traits() const
{
  return _pimpl->traits;
}

//==============================================================================
auto Planner::Options::set_graph(Graph graph) -> Options&
{
  _pimpl->graph = std::move(graph);
  return *this;
}

//==============================================================================
Graph& Planner::Options::get_graph()
{
  return _pimpl->graph;
}

//==============================================================================
const Graph& Planner::Options::get_graph() const
{
  return _pimpl->graph;
}

//==============================================================================
auto Planner::Options::change_schedule_viewer(const schedule::Viewer& viewer)
-> Options&
{
  _pimpl->viewer = &viewer;
  return *this;
}

//==============================================================================
const schedule::Viewer& Planner::Options::get_schedule_viewer() const
{
  return *_pimpl->viewer;
}

//==============================================================================
auto Planner::Options::set_mininum_holding_time(Duration holding_time)
-> Options&
{
  _pimpl->min_hold_time = holding_time;
  return *this;
}

//==============================================================================
Duration Planner::Options::get_minimum_holding_time() const
{
  return _pimpl->min_hold_time;
}

//==============================================================================
bool Planner::solve(
    const Time initial_time,
    const std::size_t initial_waypoint,
    const double initial_orientation,
    const std::size_t final_waypoint,
    const double* const final_orientation,
    const Options& options,
    std::vector<Trajectory>& solution)
{
  solution.clear();
  const VehicleTraits& traits = options.get_vehicle_traits();
  const Graph::Implementation& graph =
      Graph::Implementation::get(options.get_graph());

  if(traits.get_steering() == VehicleTraits::Steering::Differential)
  {
    return search<DifferentialDriveExpander>(
          graph, initial_time, initial_waypoint, initial_orientation,
          final_waypoint, final_orientation, options, solution);
  }
//  else if(traits.get_steering() == VehicleTraits::Steering::Holonomic)
//  {
//    return search<HolonomicExpander>(
//          graph, initial_time, initial_waypoint, initial_orientation,
//          final_waypoint, final_orientation, traits, viewer, solution);
//  }

  throw std::runtime_error(
        std::string()
        + "[rmf_traffic::agv::internal::generate_plan] Unsupported steering "
        + "mode: [" + std::to_string(static_cast<int>(traits.get_steering()))
        + "]\n");
}

} // namespace agv
} // namespace rmf_traffic
