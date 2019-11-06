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

#include "../utils.hpp"

#include <iostream>
#include <queue>
#include <unordered_map>

using rmf_traffic::internal::wrap_to_pi;

namespace rmf_traffic {
namespace agv {

namespace {
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
  const auto stop_it = node_sequence.rend();
  for(auto it = ++node_sequence.rbegin(); it != stop_it; ++it)
  {
    Trajectory& last_trajectory = trajectories.back();
    const Trajectory& next_trajectory = (*it)->trajectory_from_parent;
    if(next_trajectory.get_map_name() == last_trajectory.get_map_name())
    {
      for(const auto& segment : next_trajectory)
        last_trajectory.insert(segment);
    }
    else
    {
      trajectories.push_back(next_trajectory);
    }
  }

  return trajectories;
}

//==============================================================================
template<typename Expander>
std::vector<Planner::Waypoint> reconstruct_waypoints(
    const typename Expander::NodePtr& finish_node,
    const Graph::Implementation& graph)
{
  using NodePtr = typename Expander::NodePtr;
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while(node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  std::vector<Planner::Waypoint> waypoints;
  for(auto it = node_sequence.rbegin(); it != node_sequence.rend(); ++it)
  {
    const auto& n = *it;
    const Eigen::Vector2d p = graph.waypoints[n->waypoint].get_location();
    const Eigen::Vector3d wp{p[0], p[1], n->orientation};
    const Time time{*n->trajectory_from_parent.finish_time()};
    waypoints.emplace_back(Planner::Waypoint{wp, time});
  }

  return waypoints;
}

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
  const schedule::Viewer& viewer;
  const Time initial_time;
};

//==============================================================================
struct Node;
// TODO(MXG): Consider replacing this with std::unique_ptr
using NodePtr = std::shared_ptr<Node>;

template<typename NodePtr>
struct Compare
{
  bool operator()(const NodePtr& a, const NodePtr& b)
  {
    // Note(MXG): The priority queue puts the greater value first, so we
    // reverse the arguments in this comparison.
    // TODO(MXG): Micro-optimization: consider saving the sum of these values
    // in the Node instead of needing to re-add them for every comparison.
    return b->remaining_cost_estimate + b->current_cost
         < a->remaining_cost_estimate + a->current_cost;
  }
};

//==============================================================================
struct Node
{
  double remaining_cost_estimate;

  double current_cost;

  std::size_t waypoint;

  double orientation;

  Trajectory trajectory_from_parent;

  NodePtr parent;
};

//==============================================================================
template<
    class Expander,
    class InitialNodeArgs = typename Expander::InitialNodeArgs,
    class NodePtr = typename Expander::NodePtr>
NodePtr search(
    typename Expander::ExpansionContext context,
    InitialNodeArgs initial_node_args)
{
  using SearchQueue = typename Expander::SearchQueue;

  Expander expander(std::move(context));

  SearchQueue queue;
  queue.push(expander.make_initial_node(initial_node_args));

  while(!queue.empty())
  {
    NodePtr top = queue.top();
    queue.pop();

    if(expander.is_finished(top))
      return top;

    expander.expand(top, queue);
  }

  // Could not find a solution!
  return nullptr;
}

//==============================================================================
template<class Expander>
bool search_and_construct(
    const Graph::Implementation& graph,
    const Time initial_time,
    const std::size_t initial_waypoint,
    const double initial_orientation,
    const std::size_t final_waypoint,
    const double* const final_orientation,
    const Planner::Options& options,
    std::vector<Trajectory>& solution,
    std::vector<Planner::Waypoint>* waypoints)
{
  NodePtr goal = search<Expander>(
        ExpansionContext{
          graph, final_waypoint, final_orientation,
          options.get_vehicle_traits(),
          options.get_vehicle_traits().get_profile(),
          options.get_minimum_holding_time(),
          options.get_interpolation(),
          options.get_schedule_viewer(),
          initial_time
        },
        typename Expander::InitialNodeArgs{
          initial_waypoint,
          initial_orientation
        });

  if(!goal)
    return false;

  solution = reconstruct_trajectory<Expander>(goal);
  if(waypoints)
    *waypoints = reconstruct_waypoints<Expander>(goal, graph);

  return true;
}

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
struct EuclideanExpander
{
  struct ExpansionContext
  {
    const Graph::Implementation& graph;
    const std::size_t final_waypoint;
  };

  const ExpansionContext context;

  const Eigen::Vector2d p_final;

  struct Node;
  using NodePtr = std::shared_ptr<EuclideanExpander::Node>;

  struct Node
  {
    double remaining_cost_estimate;
    double current_cost;
    std::size_t waypoint;
    Eigen::Vector2d location;
    NodePtr parent;
  };

  using Compare = agv::Compare<NodePtr>;

  using SearchQueue =
      std::priority_queue<NodePtr, std::vector<NodePtr>, Compare>;

  EuclideanExpander(ExpansionContext _context)
    : context(std::move(_context)),
      p_final(context.graph.waypoints[context.final_waypoint].get_location())
  {
    // Do nothing
  }

  double estimate_remaining_cost(const Eigen::Vector2d& p)
  {
    return (p_final - p).norm();
  }

  void expand_lane(
      const NodePtr& parent_node,
      const std::size_t lane_index,
      SearchQueue& queue)
  {
    const Graph::Lane& lane = context.graph.lanes[lane_index];
    assert(lane.entry().waypoint_index() == parent_node->waypoint);

    const Eigen::Vector2d p_start = parent_node->location;

    const std::size_t exit_waypoint_index = lane.exit().waypoint_index();
    const Eigen::Vector2d p_exit =
        context.graph.waypoints[exit_waypoint_index].get_location();

    const double cost = parent_node->current_cost + (p_exit - p_start).norm();
    queue.push(std::make_shared<Node>(
                 Node{
                   estimate_remaining_cost(p_start),
                   cost,
                   exit_waypoint_index,
                   p_exit,
                   parent_node
                 }));
  }

  bool is_finished(const NodePtr& node)
  {
    return node->waypoint == context.final_waypoint;
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    const std::size_t parent_waypoint = parent_node->waypoint;

    const std::vector<std::size_t>& lanes =
        context.graph.lanes_from[parent_waypoint];

    for(const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);
  }

  struct InitialNodeArgs
  {
    std::size_t waypoint;
  };

  NodePtr make_initial_node(const InitialNodeArgs& args)
  {
    const Eigen::Vector2d p_initial =
        context.graph.waypoints[args.waypoint].get_location();

    return std::make_shared<Node>(
          Node{estimate_remaining_cost(p_initial), 0.0, args.waypoint,
               context.graph.waypoints[args.waypoint].get_location(), nullptr});
  }
};

//==============================================================================
struct BaseExpander
{
  using ExpansionContext = agv::ExpansionContext;

  const ExpansionContext context;

  const Eigen::Vector2d p_final;

  schedule::Query query;

  std::unordered_map<std::size_t, double> cost_estimate;

  BaseExpander(ExpansionContext _context)
    : context(std::move(_context)),
      p_final(context.graph.waypoints[context.final_waypoint].get_location()),
      query(schedule::make_query({}, nullptr, nullptr))
  {
    // Do nothing
  }

  double estimate_remaining_cost(std::size_t waypoint)
  {
    // TODO(MXG): When we want to support multiple floors, this estimate should
    // be improved by accounting for the need to use lifts.
    //
    // TODO(MXG): We could probably improve the performance of the naive A-star
    // even further by caching the solutions for every node along the way to
    // the goal, but the current performance seems to be good enough for right
    // now.
    //
    // TODO(MXG): We could possibly make the search faster by having the cost
    // estimate account for the time required to rotate due to the orientation
    // constraints. However, it will take additional effort to account for those
    // constraints in the naive search, and it could (potentially) make it more
    // difficult to cache the naive search results if the initial orientation
    // needs to be a factor in the results cache map.

    auto estimate_it = cost_estimate.insert(
          std::make_pair(waypoint, std::numeric_limits<double>::infinity()));
    if(estimate_it.second)
    {
      // The pair was inserted, which implies that the cost estimate for this
      // waypoint has never been found before, and we should compute it now.
      const EuclideanExpander::NodePtr goal = search<EuclideanExpander>(
        EuclideanExpander::ExpansionContext{context.graph, context.final_waypoint},
        EuclideanExpander::InitialNodeArgs{waypoint});

      // TODO(MXG): Instead of asserting that the goal exists, we should
      // probably take this opportunity to shortcircuit the planner and return
      // that there is no solution.
      assert(goal != nullptr);

      std::vector<Eigen::Vector3d> positions;
      EuclideanExpander::NodePtr naive_node = goal;
      // Note: this constructs positions in reverse, but that's okay because
      // the Interpolate::positions function will produce a trajectory with the
      // same duration whether it is interpolating forward or backwards.
      while(naive_node)
      {
        const Eigen::Vector2d p = naive_node->location;
        positions.push_back({p[0], p[1], 0.0});
        naive_node = naive_node->parent;
      }

      // We pass in context.initial_time here because we don't actually care
      // about the Trajectory's start/end time being correct; we only care
      // about the difference between the two.
      const rmf_traffic::Trajectory estimate = Interpolate::positions(
            "", context.traits, context.initial_time, positions);

      const double cost_estimate = time::to_seconds(estimate.duration());
      estimate_it.first->second = cost_estimate;
    }

    return estimate_it.first->second;
  }

  bool is_valid(const Trajectory& trajectory)
  {
    assert(trajectory.size() > 1);
    query.spacetime().timespan()->set_lower_time_bound(
          *trajectory.start_time());
    query.spacetime().timespan()->set_upper_time_bound(
          *trajectory.finish_time());

    // TODO(MXG): When we start generating plans across multiple maps, we should
    // account for the trajectory's map name(s) here.
    const auto view = context.viewer.query(query);

    for(const auto& check : view)
    {
      assert(trajectory.size() > 1);
      assert(check.size() > 1);
      if(!DetectConflict::between(trajectory, check).empty())
        return false;
    }

    return true;
  }

  struct InitialNodeArgs
  {
    std::size_t waypoint;
    double orientation;
  };

  NodePtr make_initial_node(const InitialNodeArgs& args)
  {
    const std::string& map_name =
        context.graph.waypoints[args.waypoint].get_map_name();

    query.spacetime().timespan()->add_map(map_name);

    Trajectory initial(map_name);
    initial.insert(
          context.initial_time, context.profile,
          to_3d(context.graph.waypoints[args.waypoint].get_location(),
              args.orientation),
          Eigen::Vector3d::Zero());

    return std::make_shared<Node>(
          Node{estimate_remaining_cost(args.waypoint),
               0.0, args.waypoint, args.orientation,
               initial, nullptr});
  }

  bool is_finished(const NodePtr& node) const
  {
    if(node->waypoint != context.final_waypoint)
      return false;

    if(context.final_orientation)
    {
      if(std::abs(node->orientation - *context.final_orientation)
         > context.interpolate.rotation_thresh)
        return false;
    }

    return true;
  }
};

//==============================================================================
class DifferentialOrientationConstraint : public Graph::OrientationConstraint
{
public:

  static Eigen::Rotation2Dd compute_forward_offset(
      const Eigen::Vector2d& forward)
  {
    return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
  }

  static const Eigen::Rotation2Dd R_pi;

  DifferentialOrientationConstraint(
      const Eigen::Vector2d& forward,
      const bool reversible)
    : R_f(compute_forward_offset(forward)),
      R_f_inv(R_f.inverse()),
      reversible(reversible)
  {
    // Do nothing
  }

  Eigen::Rotation2Dd compute_R_final(
      const Eigen::Vector2d& course_vector,
      const Eigen::Vector2d& heading) const
  {
    const Eigen::Rotation2Dd R_c(
          std::atan2(course_vector[1], course_vector[0]));

    if(reversible)
    {
      if(heading.dot(course_vector) >= 0.0)
        return R_c * R_f_inv;
      else
        return R_pi * R_c * R_f_inv;
    }

    return R_c * R_f_inv;
  }

  bool apply(
      Eigen::Vector3d& position,
      const Eigen::Vector2d& course_vector) const final
  {
    const double initial_angle = position[2];
    const Eigen::Rotation2Dd R_r(initial_angle);

    const Eigen::Rotation2Dd R = compute_R_final(
          course_vector, R_f*R_r*Eigen::Vector2d::UnitX());

    position[2] = wrap_to_pi(R.angle());
    return true;
  }

  std::vector<double> get_orientations(
      const Eigen::Vector2d& course_vector) const
  {
    std::vector<double> orientations;
    orientations.reserve(2);

    const Eigen::Rotation2Dd R_c(
          std::atan2(course_vector[1], course_vector[0]));
    const Eigen::Rotation2Dd R_h = R_c * R_f_inv;

    orientations.push_back(wrap_to_pi(R_h.angle()));

    if(reversible)
      orientations.push_back(wrap_to_pi((R_pi * R_h).angle()));

    return orientations;
  }

  std::unique_ptr<OrientationConstraint> clone() const final
  {
    return std::make_unique<DifferentialOrientationConstraint>(*this);
  }

  Eigen::Rotation2Dd R_f;
  Eigen::Rotation2Dd R_f_inv;
  bool reversible;

};

//==============================================================================
const Eigen::Rotation2Dd DifferentialOrientationConstraint::R_pi =
    Eigen::Rotation2Dd(M_PI);

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

  using Compare = agv::Compare<NodePtr>;

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

    if(std::abs(wrap_to_pi(position[2] - orientation)) < threshold)
      return {TargetOrientation::Result::Unchanged, position[2]};

    return {TargetOrientation::Result::Changed, position[2]};
  }

  bool is_orientation_okay(
      const Eigen::Vector2d& initial_p,
      const double orientation,
      const Eigen::Vector2d& course,
      const Graph::Lane& lane) const
  {
    for(const auto* constraint : {
        lane.entry().orientation_constraint(),
        lane.exit().orientation_constraint()})
    {
      if(!constraint)
        continue;

      Eigen::Vector3d position{initial_p[0], initial_p[1], orientation};
      if(!constraint->apply(position, course))
        return false;

      if(std::abs(wrap_to_pi(orientation - position[2]))
         > context.interpolate.rotation_thresh)
        return false;
    }

    return true;
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
    for(const auto& parent : expand_rotations(initial_parent, initial_lane_index))
      expand_down_lane(parent, initial_lane_index, queue);
  }

  void expand_down_lane(
      const NodePtr& initial_parent,
      const std::size_t initial_lane_index,
      SearchQueue& queue)
  {
    const std::size_t initial_waypoint = initial_parent->waypoint;
    assert(context.graph.lanes[initial_lane_index].entry().waypoint_index()
           == initial_waypoint);
    const Eigen::Vector2d initial_p =
        context.graph.waypoints[initial_waypoint].get_location();
    const double orientation = initial_parent->orientation;

    Trajectory initial_trajectory{
      context.graph.waypoints[initial_waypoint].get_map_name()};

    const Trajectory::Segment& last_seg =
        initial_parent->trajectory_from_parent.back();

    const Eigen::Vector3d initial_position = last_seg.get_finish_position();

    initial_trajectory.insert(
          last_seg.get_finish_time(),
          context.profile,
          initial_position,
          Eigen::Vector3d::Zero());

    std::vector<LaneExpansionNode> expansion_queue;
    expansion_queue.push_back(
        {initial_lane_index, std::move(initial_trajectory)});

    while(!expansion_queue.empty())
    {
      const LaneExpansionNode top = std::move(expansion_queue.back());
      expansion_queue.pop_back();

      const Graph::Lane& lane = context.graph.lanes[top.lane];
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();
      const Graph::Waypoint& exit_waypoint =
          context.graph.waypoints[exit_waypoint_index];

      const Eigen::Vector2d& next_p = exit_waypoint.get_location();
      const Eigen::Vector3d next_position{next_p[0], next_p[1], orientation};

      // TODO(MXG): Figure out what to do if the trajectory spans across
      // multiple maps.
      Trajectory trajectory = top.parent_trajectory;
      const auto& continue_from = trajectory.back();
      internal::interpolate_translation(
            trajectory,
            context.traits.linear().get_nominal_velocity(),
            context.traits.linear().get_nominal_acceleration(),
            continue_from.get_finish_time(),
            continue_from.get_finish_position(), next_position,
            context.profile, context.interpolate.translation_thresh);

      // NOTE(MXG): We cannot move the trajectory in this function call, because
      // we may need to copy the trajectory later when we expand further down
      // other lanes.
      if(!add_if_valid(exit_waypoint_index, orientation, initial_parent,
                       trajectory, queue))
      {
        // This lane was not successfully added, so we should not try to expand
        // this any further.
        continue;
      }

      // If this lane was successfully added, we can try to find more lanes to
      // continue down, as a single expansion from the original parent.
      const std::vector<std::size_t>& lanes =
          context.graph.lanes_from[exit_waypoint_index];

      for(const std::size_t l : lanes)
      {
        const Graph::Lane& future_lane = context.graph.lanes[l];

        const Eigen::Vector2d future_p =
            context.graph.waypoints[future_lane.exit().waypoint_index()]
            .get_location();

        const Eigen::Vector2d course = future_p - initial_p;

        const bool check_orientation =
            is_orientation_okay(initial_p, orientation, course, future_lane);

        if(!check_orientation)
        {
          // The orientation needs to be changed to go down this lane, so we
          // should not expand in this direction
          continue;
        }

        const Eigen::Vector3d future_position{
            future_p[0], future_p[1], orientation};

        if(!internal::can_skip_interpolation(
             initial_position, next_position,
             future_position, context.interpolate))
        {
          continue;
        }

        expansion_queue.push_back({l, trajectory});
      }
    }
  }

  bool add_if_valid(
      const std::size_t waypoint,
      const double orientation,
      const NodePtr& parent_node,
      Trajectory trajectory,
      SearchQueue& queue,
      const double cost_factor = 1.0)
  {
    assert(trajectory.size() > 1);
    if(is_valid(trajectory))
    {
      const double cost = compute_current_cost(parent_node, trajectory);
      // TODO(MXG): Consider short-circuiting the rest of the search and
      // returning the solution if this Node solves the search problem. It could
      // be an optional behavior configurable from the Planner::Options.
      queue.push(std::make_shared<Node>(
                   Node{
                     estimate_remaining_cost(waypoint),
                     cost * cost_factor,
                     waypoint,
                     orientation,
                     std::move(trajectory),
                     parent_node
                   }));
      return true;
    }

    return false;
  }

  NodePtr expand_rotation(
      const NodePtr& parent_node,
      const double target_orientation)
  {
    // TODO(MXG): This function should be completely changed. Instead of
    // expanding towards the target orientation and creating a new node for it,
    // it should just create a trajectory to get the AGV facing the correct
    // direction and check whether that trajectory is feasible. Then it should
    // return that trajectory so it can be the start of an expansion that
    // actually goes down a lane. This should offer a significant reduction to
    // the branching factor of the search.
    //
    // TODO(MXG): For completeness, adding to the above TODO, we should expand
    // towards both forward and backward orientations every time (except when
    // constraints forbid it). Otherwise we could be missing out on potential
    // solutions (or getting sub-optimal solutions) for extreme edge cases.
    const std::size_t waypoint = parent_node->waypoint;
    Trajectory trajectory{context.graph.waypoints[waypoint].get_map_name()};
    const Trajectory::Segment& last =
        parent_node->trajectory_from_parent.back();

    const Eigen::Vector3d& p = last.get_finish_position();
    trajectory.insert(last);

    internal::interpolate_rotation(
          trajectory,
          context.traits.rotational().get_nominal_velocity(),
          context.traits.rotational().get_nominal_acceleration(),
          last.get_finish_time(),
          p,
          Eigen::Vector3d(p[0], p[1], target_orientation),
          context.profile,
          context.interpolate.rotation_thresh);

    if(is_valid(trajectory))
    {
      return std::make_shared<Node>(
            Node{
              estimate_remaining_cost(waypoint),
              compute_current_cost(parent_node, trajectory),
              waypoint,
              target_orientation,
              std::move(trajectory),
              parent_node
            });
    }

    return nullptr;
  }

  std::vector<NodePtr> expand_rotations(
      const NodePtr& parent_node,
      const std::size_t lane_index)
  {
    const Graph::Lane& lane = context.graph.lanes[lane_index];

    const Graph::Waypoint& initial_waypoint =
        context.graph.waypoints[lane.entry().waypoint_index()];
    const Eigen::Vector2d& initial_p = initial_waypoint.get_location();

    const Graph::Waypoint& next_waypoint =
        context.graph.waypoints[lane.exit().waypoint_index()];
    const Eigen::Vector2d& next_p = next_waypoint.get_location();

    const Eigen::Vector2d course = (next_p - initial_p).normalized();

    const std::vector<double> orientations =
        differential_constraint.get_orientations(course);

    std::vector<NodePtr> rotations;
    rotations.reserve(orientations.size());
    for(const double orientation : orientations)
    {
      if(!is_orientation_okay(initial_p, orientation, course, lane))
        continue;

      if(std::abs(wrap_to_pi(orientation - parent_node->orientation))
         < context.interpolate.rotation_thresh )
      {
        // No rotation is needed here
        rotations.push_back(parent_node);
      }
      else
      {
        const NodePtr rotation = expand_rotation(parent_node, orientation);
        if(rotation)
          rotations.push_back(rotation);
      }
    }

    return rotations;
  }

  void expand_holding(
      const std::size_t waypoint,
      const NodePtr& parent_node,
      SearchQueue& queue)
  {
    const Trajectory& parent_trajectory = parent_node->trajectory_from_parent;
    const auto& initial_segment = parent_trajectory.back();

    Trajectory trajectory{context.graph.waypoints[waypoint].get_map_name()};

    const Time initial_time = initial_segment.get_finish_time();
    const Eigen::Vector3d& initial_pos = initial_segment.get_finish_position();
    trajectory.insert(initial_segment);
    assert(trajectory.size() == 1);

    trajectory.insert(
          initial_time + context.holding_time,
          context.profile,
          initial_pos,
          Eigen::Vector3d::Zero());
    assert(trajectory.size() == 2);

    add_if_valid(
          waypoint, initial_pos[2], parent_node,
          std::move(trajectory), queue, 1.0);
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    const std::size_t parent_waypoint = parent_node->waypoint;
    if(parent_waypoint == context.final_waypoint)
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

      const double final_orientation = wrap_to_pi(*context.final_orientation);
      const auto final_node =
          expand_rotation(parent_node, final_orientation);
      if(final_node)
      {
        queue.push(final_node);
        return;
      }

      // Note: If the rotation was not valid, then some other trajectory is
      // blocking us from rotating, so we should keep expanding as usual.
      // If the rotation was valid, then the node that was added is a solution
      // node, so we should not expand anything else from this parent node.
      // We should, however, continue to expand other nodes, because a more
      // optimal solution could still exist.
    }

    const std::vector<std::size_t>& lanes =
        context.graph.lanes_from[parent_waypoint];

    for(const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);

    if(context.graph.waypoints[parent_waypoint].is_holding_point())
      expand_holding(parent_waypoint, parent_node, queue);
  }

};

//==============================================================================
// TODO(MXG): Implement this class. It should be even easier than the
// DifferentialDriveExpander.
//struct HolonomicExpander
//{
//  const ExpansionContext context;

//  HolonomicExpander(ExpansionContext ec)
//    : context(std::move(ec))
//  {
//    // Do nothing
//  }

//  struct Node
//  {
//    double cost_estimate;
//  };

//  // TODO(MXG): Consider changing this to unique_ptr
//  using NodePtr = std::shared_ptr<Node>;

//  struct Compare
//  {
//    bool operator()(const NodePtr& a, const NodePtr& b)
//    {
//      // Note(MXG): The priority queue puts the greater value first, so we
//      // reverse the arguments in this comparison.
//      return b->cost_estimate < a->cost_estimate;
//    }
//  };

//};
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
    std::vector<Trajectory>& solution,
    std::vector<Waypoint>* waypoints)
{
  solution.clear();
  const VehicleTraits& traits = options.get_vehicle_traits();
  const Graph::Implementation& graph =
      Graph::Implementation::get(options.get_graph());

  if(traits.get_steering() == VehicleTraits::Steering::Differential)
  {
    return search_and_construct<DifferentialDriveExpander>(
          graph, initial_time, initial_waypoint, initial_orientation,
          final_waypoint, final_orientation, options, solution, waypoints);
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
