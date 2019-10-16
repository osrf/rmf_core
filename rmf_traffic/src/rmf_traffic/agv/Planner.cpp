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
#include "InterpolateInternal.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/Conflict.hpp>

#include <iostream>
#include <queue>

namespace rmf_traffic {
namespace agv {

namespace {
//==============================================================================
struct ExpansionContext
{
  const Graph::Implementation& graph;
  const std::size_t final_waypoint;
  const double* const final_orientation;
  const VehicleTraits& traits;
  const Trajectory::ConstProfilePtr profile;
  const Duration holding_time;
  const Interpolate::Options::Implementation interpolate;
  const schedule::Viewer::View view;
};

//==============================================================================
struct Node;
// TODO(MXG): Consider replacing this with std::unique_ptr
using NodePtr = std::shared_ptr<Node>;

//==============================================================================
struct Node
{
  double final_cost_estimate;

  double current_cost;

  std::size_t waypoint;

  double orientation;

  Trajectory trajectory_from_parent;

  NodePtr parent;
};

//==============================================================================
Eigen::Vector3d to_3d(const Eigen::Vector2d& p, const double w)
{
  return Eigen::Vector3d(p[0], p[1], w);
}

//==============================================================================
double compute_current_cost(
    const NodePtr& parent,
    const Trajectory& trajectory_from_parent)
{
  return parent->current_cost + time::to_seconds(
        *trajectory_from_parent.finish_time()
        - *trajectory_from_parent.start_time());
}

//==============================================================================
struct BaseExpander
{
  const ExpansionContext context;

  const Eigen::Vector2d p_final;

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
    //
    // TODO(MXG): It would almost certainly be better to precompute the best
    // path from each waypoint to the goal using a simpler time-agnostic A-star
    // per waypoint. These results can be cached and reused every time we need
    // to re-expand a waypoint, e.g. when a plan requires us to loop to avoid
    // a conflict with another trajectory.
    return (p_final - p0).norm()/context.traits.linear().get_nominal_velocity();
  }

  bool is_valid(const Trajectory& trajectory) const
  {
    for(const auto& check : context.view)
    {
      if(!DetectConflict::between(trajectory, check).empty())
        return false;
    }

    return true;
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
class DifferentialOrientationConstraint : public Graph::OrientationConstraint
{
public:

  DifferentialOrientationConstraint(
      const Eigen::Vector2d& forward,
      const bool reversible)
    : forward(forward),
      reversible(reversible)
  {
    // Do nothing
  }

  bool apply(
      Eigen::Vector3d& position,
      const Eigen::Vector2d& course_vector) const final
  {

  }

  std::unique_ptr<OrientationConstraint> clone() const final
  {
    return std::make_unique<DifferentialOrientationConstraint>(*this);
  }

  Eigen::Vector2d forward;
  bool reversible;

};

//==============================================================================
struct DifferentialDriveExpander : BaseExpander
{
  DifferentialOrientationConstraint differential_constraint;

  DifferentialDriveExpander(ExpansionContext ec)
    : BaseExpander(std::move(ec)),
      differential_constraint(
        context.traits.get_differential()->get_forward(),
        context.traits.get_differential()->is_reversible())
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

  struct TargetOrientation
  {
    enum class Result
    {
      Invalid,
      Changed,
      Unchanged
    };

    Result result;
    double value;

    void update(const TargetOrientation& other)
    {
      result = std::min(result, other.result);
      value = other.value;
    }
  };

  TargetOrientation apply_constraint(
      const Graph::OrientationConstraint* constraint,
      const Eigen::Vector2d& p,
      const double orientation,
      const Eigen::Vector2d& course,
      const double threshold)
  {
    Eigen::Vector3d position{p[0], p[1], orientation};
    if(!constraint->apply(position, course))
    {
      // Apparently the entry constraint for this lane cannot be satisfied in
      // this situation
      return {TargetOrientation::Result::Invalid, orientation};
    }

    if(std::abs(position[2] - orientation) < threshold)
      return {TargetOrientation::Result::Unchanged, position[2]};

    return {TargetOrientation::Result::Changed, position[2]};
  }

  TargetOrientation get_target_orientation(
      const Eigen::Vector2d& initial_p,
      const double initial_orientation,
      const Graph::Lane& lane)
  {
    const Eigen::Vector2d next_p =
        context.graph.waypoints[lane.exit().waypoint_index()]
        .get_location();
    const Eigen::Vector2d course = next_p - initial_p;
    const double thresh = context.interpolate.rotation_thresh;

    // NOTE(MXG): This implementation assumes that the user-specified
    // entry_constraint and exit_constraint will not violate the differential
    // drive constraint, because if they ever do then the lane is impossible to
    // traverse.
    //
    // TODO(MXG): Maybe that assumption should be encoded in an assertion.
    TargetOrientation target = apply_constraint(
          &differential_constraint, initial_p,
          initial_orientation, course, thresh);

    const auto* entry_constraint =
        lane.entry().orientation_constraint();
    if(entry_constraint)
    {
      target.update(apply_constraint(
            entry_constraint, initial_p, target.value, course, thresh));
      if(TargetOrientation::Result::Invalid == target.result)
        return target;
    }

    const auto* exit_constraint =
        lane.exit().orientation_constraint();
    if(exit_constraint)
    {
      const TargetOrientation exit_target = apply_constraint(
            exit_constraint, initial_p, target.value, course, thresh);
      if(TargetOrientation::Result::Invalid == exit_target.result)
        return exit_target;

      if(TargetOrientation::Result::Changed == exit_target.result)
      {
        if(entry_constraint)
        {
          // The solution that was given earlier by the entry_constraint did not
          // satisfy the exit_constraint. Let's see if the entry_constraint is
          // satisfied by the new exit_constraint's orientation.
          const TargetOrientation new_target = apply_constraint(
                entry_constraint, initial_p, exit_target.value, course, thresh);

          if(TargetOrientation::Result::Invalid == new_target.result
             || TargetOrientation::Result::Changed == new_target.result)
          {
            // The entry_constraint did not accept the output of the
            // exit_constraint, so we will give up now.
            //
            // TODO(MXG): This makes some assumptions about the behavior of the
            // constraint classes, and those assumptions are not explicit in the
            // documentation or the API. This should be fixed.
            return {TargetOrientation::Result::Invalid, initial_orientation};
          }

          return {TargetOrientation::Result::Changed, new_target.value};
        }
      }
    }

    return target;
  }

  struct LaneExpansionNode
  {
    std::size_t lane;
    Trajectory parent_trajectory;
  };

  void expand_lane(
      const NodePtr& initial_parent,
      const std::size_t initial_lane_index,
      SearchQueue& queue)
  {
    const Graph::Lane& initial_lane = context.graph.lanes[initial_lane_index];
    const std::size_t initial_waypoint = initial_parent->waypoint;
    assert(initial_lane.entry().waypoint_index() == initial_waypoint);
    const Eigen::Vector2d initial_p =
        context.graph.waypoints[initial_waypoint].get_location();
    const double orientation = initial_parent->orientation;

    const TargetOrientation orientation_change =
        get_target_orientation(initial_p, orientation, initial_lane);
    if(orientation_change.result == TargetOrientation::Result::Invalid)
      return;

    if(orientation_change.result == TargetOrientation::Result::Changed)
    {
      // We need to rotate before we can travel down this lane, so we'll expand
      // towards the rotation and then quit the expansion.
      expand_rotation(initial_parent, orientation_change.value, queue);
      return;
    }

    Trajectory initial_trajectory{
      context.graph.waypoints[initial_waypoint].get_map_name()};
    const Trajectory::Segment& last_seg =
        *(--initial_parent->trajectory_from_parent.end());
    initial_trajectory.insert(
          last_seg.get_finish_time(),
          context.profile,
          last_seg.get_finish_position(),
          Eigen::Vector3d::Zero());

    // The orientation constraints are satisfied, so we'll proceed down the lane
    std::vector<LaneExpansionNode> expansion_queue;
    expansion_queue.push_back(
        {initial_lane_index, std::move(initial_trajectory)});

    while(!expansion_queue.empty())
    {
      const LaneExpansionNode top = std::move(expansion_queue.back());
      expansion_queue.pop_back();

      const Graph::Lane& lane = context.graph.lanes[top.lane];
      const Graph::Waypoint& exit_waypoint =
          context.graph.waypoints[lane.exit().waypoint_index()];

      const Eigen::Vector2d& next_p = exit_waypoint.get_location();


    }


    // TODO(MXG): Finish this expansion function
    // * Calculate an expansion from this waypoint to each of the other
    //   waypoints that connect to this waypoint by at least one lane
    // * When possible, calculate a multi-waypoint expansion (e.g. when there
    //   are straight lines connecting a sequence of waypoints, and no
    //   velocity constraint between them)
    // * Check each expansion against the schedule to make sure that it is
    //   indeed a valid expansion
  }

  bool expand_rotation(
      const NodePtr& parent_node,
      const double target_orientation,
      SearchQueue& queue) const
  {
    const std::size_t waypoint = parent_node->waypoint;
    Trajectory trajectory{context.graph.waypoints[waypoint].get_map_name()};
    const Trajectory::Segment& last =
        *(--parent_node->trajectory_from_parent.end());
    const Eigen::Vector3d& p = last.get_finish_position();
    trajectory.insert(
          last.get_finish_time(),
          last.get_profile(),
          p,
          Eigen::Vector3d::Zero());

    internal::interpolate_rotation(
          trajectory,
          context.traits.rotational().get_nominal_velocity(),
          context.traits.rotational().get_nominal_acceleration(),
          last.get_finish_time(),
          p,
          Eigen::Vector3d(p[0], p[1], target_orientation));

    if(is_valid(trajectory))
    {
      queue.push(std::make_shared<Node>(
                   Node{
                     estimate_remaining_cost(waypoint),
                     compute_current_cost(parent_node, trajectory),
                     waypoint,
                     target_orientation,
                     trajectory,
                     parent_node
                   }));
      return true;
    }

    return false;
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    if(parent_node->waypoint == context.final_waypoint)
    {
      if(!context.final_orientation)
      {
        // We have already arrived at the solution, because the user does not
        // care about the final orientation.
        //
        // This should have been recognized by the is_finished() function, so
        // this is indicative of a bug.
        std::cerr << "[rmf_traffic::agv::DifferentialDriveExpander::expand] "
                  << "A bug has occurred. Please report this to the RMF "
                  << "developers." << std::endl;
        assert(false);
      }

      if(expand_rotation(parent_node, *context.final_orientation, queue))
        return;

      // Note: If the rotation was not valid, then some other trajectory is
      // blocking us from rotating, so we should keep expanding as usual.
    }

    const std::vector<std::size_t>& lanes =
        context.graph.lanes_from[parent_node->waypoint];

    for(const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);

    if(context.graph.waypoints[parent_node->waypoint].is_holding_point())
    {

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
                      options.get_interpolation(),
                      options.get_schedule_viewer().query(
                          schedule::query_everything())});

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
} // anonymous namespace

//==============================================================================
class Planner::Options::Implementation
{
public:

  VehicleTraits traits;

  Graph graph;

  const schedule::Viewer* viewer;

  Duration min_hold_time;

  Interpolate::Options interpolation;
};

//==============================================================================
Planner::Options::Options(
    VehicleTraits vehicle_traits,
    Graph graph,
    const schedule::Viewer& viewer,
    Duration min_hold_time,
    Interpolate::Options interpolation)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             Implementation{
               std::move(vehicle_traits),
               std::move(graph),
               &viewer,
               min_hold_time,
               std::move(interpolation)}))
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
auto Planner::Options::set_interpolation(Interpolate::Options interpolate)
-> Options&
{
  _pimpl->interpolation = std::move(interpolate);
  return *this;
}

//==============================================================================
Interpolate::Options& Planner::Options::get_interpolation()
{
  return _pimpl->interpolation;
}

//==============================================================================
const Interpolate::Options& Planner::Options::get_interpolation() const
{
  return _pimpl->interpolation;
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
