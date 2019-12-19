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

#include "InterpolateInternal.hpp"
#include "internal_Planner.hpp"
#include "internal_planning.hpp"
#include "GraphInternal.hpp"

#include <rmf_utils/math.hpp>

#include <rmf_traffic/Conflict.hpp>

#include <iostream>
#include <map>
#include <unordered_map>
#include <queue>

namespace rmf_traffic {
namespace internal {
namespace planning {

//==============================================================================
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
Cache::Cache(const Cache&)
{
  // Do nothing
}

//==============================================================================
Cache::Cache(Cache&&)
{
  // Do nothing
}

//==============================================================================
Cache& Cache::operator=(const Cache&)
{
  // Do nothing
  return *this;
}

//==============================================================================
Cache& Cache::operator=(Cache&&)
{
  // Do nothing
  return *this;
}

//==============================================================================
CacheHandle::CacheHandle(CachePtr original)
  : _original(std::move(original))
{
  std::unique_lock<std::mutex> lock(_original->mutex);
  _copy = _original->clone();
}

//==============================================================================
rmf_utils::optional<Result> CacheHandle::plan(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options)
{
  return _copy->plan(starts, std::move(goal), std::move(options));
}

//==============================================================================
CacheHandle::~CacheHandle()
{
  // We are now done with the copy, so we will update the original cache with
  // whatever new searches have been accomplished by the copy.
  std::unique_lock<std::mutex> lock(_original->mutex);
  _original->update(*_copy);
}

//==============================================================================
CacheManager::CacheManager(CachePtr cache)
  : _cache(std::move(cache))
{
  // Do nothing
}

//==============================================================================
CacheHandle CacheManager::get() const
{
  return CacheHandle(_cache);
}

//==============================================================================
const agv::Planner::Configuration& CacheManager::get_configuration() const
{
  return _cache->get_configuration();
}

//==============================================================================
template<
    class Expander,
    class Context = typename Expander::Context,
    class InitialNodeArgs = typename Expander::InitialNodeArgs,
    class NodePtr = typename Expander::NodePtr>
NodePtr search(
    Context&& context,
    InitialNodeArgs&& initial_node_args,
    const bool* interrupt_flag)
{
  using SearchQueue = typename Expander::SearchQueue;

  Expander expander(context);

  SearchQueue queue;
  expander.make_initial_nodes(initial_node_args, queue);

  while(!queue.empty() && !(interrupt_flag && *interrupt_flag))
  {
    NodePtr top = queue.top();
    queue.pop();

    if(expander.is_finished(top))
      return top;

    expander.expand(top, queue);
  }

  return nullptr;
}

//==============================================================================
template<typename NodePtr>
std::vector<Trajectory> reconstruct_trajectories(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while (node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  std::vector<Trajectory> trajectories;
  trajectories.push_back(
        Trajectory{
          node_sequence.back()->trajectory_from_parent.get_map_name()
        });

  // We exclude the first node in the sequence, because it contains a dummy
  // trajectory which is not helpful.
  const auto stop_it = node_sequence.rend();
  for (auto it = ++node_sequence.rbegin(); it != stop_it; ++it)
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
template<typename NodePtr>
std::vector<agv::Plan::Waypoint> reconstruct_waypoints(
    const NodePtr& finish_node,
    const agv::Graph::Implementation& graph)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while(node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  std::vector<agv::Plan::Waypoint> waypoints;
  for (auto it = node_sequence.rbegin(); it != node_sequence.rend(); ++it)
  {
    const auto& n = *it;
    const Eigen::Vector2d p = n->waypoint?
          graph.waypoints[*n->waypoint].get_location() :
          n->trajectory_from_parent.back().get_finish_position()
            .template block<2,1>(0,0);
    const Time time{*n->trajectory_from_parent.finish_time()};
    waypoints.emplace_back(
          agv::Plan::Waypoint::Implementation::make(
            Eigen::Vector3d{p[0], p[1], n->orientation}, time,
            n->waypoint, n->event));
  }

  return waypoints;
}

//==============================================================================
template<typename NodePtr>
std::size_t find_start_index(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  while (node->parent)
    node = node->parent;

  return *node->start_set_index;
}

//==============================================================================
struct EuclideanExpander
{
  // TODO(MXG): We should be able to improve the performance of this Euclidean
  // search by making it two-sided (i.e. expanding from both the start and the
  // goal at the same time).

  // TODO(MXG): We can also make multi-floor planning more efficient by
  // explicitly expanding to/from the LiftShaft bottlenecks.

  struct Node;
  using NodePtr = std::shared_ptr<Node>;

  struct Node
  {
    std::size_t waypoint;
    double remaining_cost_estimate;
    double current_cost;
    Eigen::Vector2d location;
    NodePtr parent;
  };

  using SearchQueue =
      std::priority_queue<NodePtr, std::vector<NodePtr>, Compare<NodePtr>>;

  struct InitialNodeArgs
  {
    std::size_t waypoint;
  };

  struct Context
  {
    const agv::Graph::Implementation& graph;
    const std::size_t final_waypoint;
  };

  double estimate_remaining_cost(const Eigen::Vector2d& p)
  {
    return (p_final - p).norm();
  }

  EuclideanExpander(const Context& context)
  : context(context),
    p_final(context.graph.waypoints[context.final_waypoint].get_location())
  {
    // Do nothing
  }

  void make_initial_nodes(const InitialNodeArgs& args, SearchQueue& queue)
  {
    const Eigen::Vector2d location =
        context.graph.waypoints[args.waypoint].get_location();

    queue.emplace(std::make_shared<Node>(
          Node{
            args.waypoint,
            estimate_remaining_cost(location),
            0.0,
            location,
            nullptr
          }));
  }

  bool is_finished(const NodePtr& node)
  {
    return node->waypoint == context.final_waypoint;
  }

  static double lane_event_cost(const agv::Graph::Lane& lane)
  {
    double cost = 0.0;
    if(const auto* event = lane.entry().event())
      cost += time::to_seconds(event->duration());

    if(const auto* event = lane.exit().event())
      cost += time::to_seconds(event->duration());

    return cost;
  }

  void expand_lane(
      const NodePtr& parent_node,
      const std::size_t lane_index,
      SearchQueue& queue)
  {
    const agv::Graph::Lane& lane = context.graph.lanes[lane_index];
    assert(lane.entry().waypoint_index() == parent_node->waypoint);
    const std::size_t exit_waypoint_index = lane.exit().waypoint_index();
    if(expanded.count(exit_waypoint_index) > 0)
    {
      // This waypoint has already been expanded from, so there's no point in
      // expanding towards it again.
      return;
    }

    const Eigen::Vector2d p_start = parent_node->location;
    const Eigen::Vector2d p_exit =
        context.graph.waypoints[exit_waypoint_index].get_location();

    const double cost =
        parent_node->current_cost
        + lane_event_cost(lane)
        + (p_exit - p_start).norm();

    queue.push(std::make_shared<Node>(
                 Node{
                   exit_waypoint_index,
                   estimate_remaining_cost(p_exit),
                   cost,
                   p_exit,
                   parent_node
                 }));
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    const std::size_t parent_waypoint = parent_node->waypoint;
    expanded.insert(parent_waypoint);

    const std::vector<std::size_t>& lanes =
        context.graph.lanes_from[parent_waypoint];

    for(const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);
  }

private:
  const Context& context;
  Eigen::Vector2d p_final;
  std::unordered_set<std::size_t> expanded;
};

//==============================================================================
Eigen::Vector3d to_3d(const Eigen::Vector2d& p, const double w)
{
  return Eigen::Vector3d(p[0], p[1], w);
}

//==============================================================================
template<typename NodePtrT>
double compute_current_cost(
    const NodePtrT& parent,
    const Trajectory& trajectory_from_parent)
{
  return parent->current_cost + time::to_seconds(
        trajectory_from_parent.duration());
}

//==============================================================================
class DifferentialDriveConstraint
{
public:

  static Eigen::Rotation2Dd compute_forward_offset(
      const Eigen::Vector2d& forward)
  {
    return Eigen::Rotation2Dd(std::atan2(forward[1], forward[0]));
  }

  static const Eigen::Rotation2Dd R_pi;

  DifferentialDriveConstraint(
      const Eigen::Vector2d& forward,
      const bool reversible)
  : R_f_inv(compute_forward_offset(forward).inverse()),
    reversible(reversible)
  {
    // Do nothing
  }

  std::vector<double> get_orientations(
      const Eigen::Vector2d& course_vector)
  {
    std::vector<double> orientations;
    orientations.reserve(2);

    const Eigen::Rotation2Dd R_c(
          std::atan2(course_vector[1], course_vector[0]));
    const Eigen::Rotation2Dd R_h = R_c * R_f_inv;

    orientations.push_back(rmf_utils::wrap_to_pi(R_h.angle()));

    if(reversible)
      orientations.push_back(rmf_utils::wrap_to_pi((R_pi * R_h).angle()));

    return orientations;
  }

  Eigen::Rotation2Dd R_f_inv;
  bool reversible;

};

//==============================================================================
const Eigen::Rotation2Dd DifferentialDriveConstraint::R_pi =
    Eigen::Rotation2Dd(M_PI);

//==============================================================================
struct DifferentialDriveExpander
{
  struct Node;
  using NodePtr = std::shared_ptr<Node>;

  struct Node
  {
    double remaining_cost_estimate;
    double current_cost;
    rmf_utils::optional<std::size_t> waypoint;
    double orientation;
    Trajectory trajectory_from_parent;
    agv::Graph::Lane::EventPtr event;
    NodePtr parent;
    rmf_utils::optional<std::size_t> start_set_index = rmf_utils::nullopt;
  };

  using SearchQueue =
      std::priority_queue<NodePtr, std::vector<NodePtr>, Compare<NodePtr>>;

  class LaneEventExecutor : public agv::Graph::Lane::Executor
  {
  public:

    LaneEventExecutor& update(const NodePtr& parent)
    {
      _parent = parent;
      return *this;
    }

    void execute(const agv::Graph::Lane::DoorOpen& open) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(open);
    }

    void execute(const agv::Graph::Lane::DoorClose& close) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(close);
    }

    void execute(const agv::Graph::Lane::LiftDoorOpen& open) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(open);
    }

    void execute(const agv::Graph::Lane::LiftDoorClose& close) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(close);
    }

    void execute(const agv::Graph::Lane::LiftMove& move) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(move);
    }

    void execute(const agv::Graph::Lane::Dock& dock) final
    {
      assert(_parent);
      _event = agv::Graph::Lane::Event::make(dock);
    }

    NodePtr get(DifferentialDriveExpander* expander)
    {
      assert(_event);
      const auto duration = _event->duration();
      return expander->make_delay(
            // TODO(MXG): Somehow this was compiling without the dereference
            // operator. That should not be possible; please investigate.
            *_parent->waypoint,
            _parent,
            duration,
            std::move(_event));
    }

    LaneEventExecutor& add_if_valid(
        DifferentialDriveExpander* expander,
        SearchQueue& queue)
    {
      assert(_event);
      const auto duration = _event->duration();
      expander->expand_delay(
            *_parent->waypoint,
            _parent,
            duration,
            queue,
            std::move(_event));
      return *this;
    }

  private:
    NodePtr _parent;
    agv::Graph::Lane::EventPtr _event;
  };

  struct Context;

  struct InitialNodeArgs
  {
    const agv::Planner::StartSet& starts;
  };

  class Heuristic
  {
  public:

    double estimate_remaining_cost(
        const Context& context,
        const std::size_t waypoint)
    {
      auto estimate_it = known_costs.insert(
          {waypoint, std::numeric_limits<double>::infinity()});

      if(estimate_it.second)
      {
        // The pair was inserted, which implies that the cost estimate for this
        // waypoint has never been found before, and we should compute it now.
        const EuclideanExpander::NodePtr solution = search<EuclideanExpander>(
              EuclideanExpander::Context{context.graph, context.final_waypoint},
              EuclideanExpander::InitialNodeArgs{waypoint},
              nullptr);

        // TODO(MXG): Instead of asserting that the goal exists, we should
        // probably take this opportunity to shortcircuit the planner and return
        // that there is no solution.
        assert(solution != nullptr);

        std::vector<Eigen::Vector3d> positions;
        EuclideanExpander::NodePtr euclidean_node = solution;
        // Note: this constructs positions in reverse, but that's okay because
        // the Interpolate::positions function will produce a trajectory with the
        // same duration whether it is interpolating forward or backwards.
        while(euclidean_node)
        {
          const Eigen::Vector2d p = euclidean_node->location;
          positions.push_back({p[0], p[1], 0.0});
          euclidean_node = euclidean_node->parent;
        }

        // We pass in context.initial_time here because we don't actually care
        // about the Trajectory's start/end time being correct; we only care
        // about the difference between the two.
        const rmf_traffic::Trajectory estimate = agv::Interpolate::positions(
              "", context.traits, context.initial_time, positions);

        const double cost_esimate = time::to_seconds(estimate.duration());
        estimate_it.first->second = cost_esimate;

        // TODO(MXG): We could get significantly better performance if we
        // accounted for the cost of rotating when making this estimate, but
        // that would add considerable complexity to the caching, so we'll leave
        // that for a future improvement.
      }

      return estimate_it.first->second;
    }

    void update(const Heuristic& other)
    {
      for(const auto& wp_costs : other.known_costs)
        known_costs.insert(wp_costs);
    }

  private:
    std::unordered_map<std::size_t, double> known_costs;
  };

  struct Context
  {
    const agv::Graph::Implementation& graph;
    const agv::VehicleTraits& traits;
    const Trajectory::ConstProfilePtr& profile;
    const Duration holding_time;
    const agv::Interpolate::Options::Implementation& interpolate;
    const schedule::Viewer& viewer;
    const std::size_t final_waypoint;
    const double* const final_orientation;
    const rmf_traffic::Time initial_time;
    const bool* const interrupt_flag;
    const std::unordered_set<schedule::Version> ignore_schedule_ids;
    Heuristic& heuristic;
  };

  DifferentialDriveExpander(Context& context)
  : _context(context),
    _query(schedule::make_query({}, nullptr, nullptr)),
    _differential_constraint(
      _context.traits.get_differential()->get_forward(),
      _context.traits.get_differential()->is_reversible())
  {
    // Do nothing
  }

  void make_initial_nodes(const InitialNodeArgs& args, SearchQueue& queue)
  {
    const std::size_t N_starts = args.starts.size();
    for (std::size_t start_index=0; start_index < N_starts; ++start_index)
    {
      const auto& start = args.starts[start_index];

      const std::size_t initial_waypoint = start.waypoint();

      const double cost_estimate =
          _context.heuristic.estimate_remaining_cost(
            _context, initial_waypoint);

      const double initial_orientation = start.orientation();
      const std::string& map_name =
          _context.graph.waypoints[initial_waypoint].get_map_name();

      _query.spacetime().timespan()->add_map(map_name);

      const auto initial_time = start.time();

      const Eigen::Vector2d wp_location =
          _context.graph.waypoints[initial_waypoint].get_location();

      const auto& initial_location = start.location();
      if (initial_location)
      {
        const Eigen::Vector3d initial_position =
            to_3d(*initial_location, initial_orientation);

        Trajectory initial_trajectory{map_name};
        initial_trajectory.insert(
              initial_time,
              _context.profile,
              initial_position,
              Eigen::Vector3d::Zero());

        const auto initial_node = std::make_shared<Node>(
              Node{
                std::numeric_limits<double>::infinity(),
                0.0,
                rmf_utils::nullopt,
                initial_orientation,
                initial_trajectory,
                nullptr,
                nullptr,
                start_index
              });

        const Eigen::Vector2d course =
            (wp_location - *initial_location).normalized();

        const std::vector<double> orientations =
            _differential_constraint.get_orientations(course);

        const auto initial_lane = start.lane();

        for (const double orientation : orientations)
        {
          if (initial_lane)
          {
            const auto& lane = _context.graph.lanes[*initial_lane];
            const auto lane_exit = lane.exit().waypoint_index();
            if (lane_exit != initial_waypoint)
            {
              throw std::invalid_argument(
                    "[rmf_traffic::agv::Planner] Disagreement between initial "
                    "waypoint index [" + std::to_string(initial_waypoint)
                    + "] and the initial lane exit ["
                    + std::to_string(lane_exit) + "]");
            }

            if (!is_orientation_okay(
                  *initial_location, orientation, course, lane))
            {
              // We cannot approach the initial_waypoint with this orientation,
              // so we cannot use this orientation to start.
              continue;
            }
          }

          auto rotated_initial_node = initial_node;
          if (std::abs(rmf_utils::wrap_to_pi(orientation - initial_orientation))
              >= _context.interpolate.rotation_thresh)
          {
            const Eigen::Vector3d rotated_position =
                to_3d(*initial_location, orientation);

            Trajectory rotation_trajectory = initial_trajectory;

            const auto& rotational = _context.traits.rotational();
            // TODO(MXG): Consider refactoring this with the other spots where
            // we use interpolate_rotation
            agv::internal::interpolate_rotation(
                  rotation_trajectory,
                  rotational.get_nominal_velocity(),
                  rotational.get_nominal_acceleration(),
                  initial_time,
                  initial_position,
                  rotated_position,
                  _context.profile,
                  _context.interpolate.rotation_thresh);

            if (!is_valid(rotation_trajectory))
            {
              // The rotation trajectory is not feasible, so we cannot use this
              // orientation to start.
              continue;
            }

            const double rotation_cost =
                rmf_traffic::time::to_seconds(rotation_trajectory.duration());

            auto rotated_initial_node = std::make_shared<Node>(
                  Node{
                    std::numeric_limits<double>::infinity(),
                    rotation_cost,
                    rmf_utils::nullopt,
                    orientation,
                    std::move(rotation_trajectory),
                    nullptr,
                    initial_node
                  });
          }

          Trajectory approach_trajectory{map_name};
          approach_trajectory.insert(
                rotated_initial_node->trajectory_from_parent.back());

          agv::internal::interpolate_translation(
                approach_trajectory,
                _context.traits.linear().get_nominal_velocity(),
                _context.traits.linear().get_nominal_acceleration(),
                *approach_trajectory.start_time(),
                to_3d(*initial_location, orientation),
                to_3d(wp_location, orientation),
                _context.profile,
                _context.interpolate.translation_thresh);

          if (!is_valid(approach_trajectory))
          {
            // The approach trajectory is not feasible, so we cannot use this
            // orientation to start.
            continue;
          }

          const double current_cost =
              rmf_traffic::time::to_seconds(approach_trajectory.duration())
              + rotated_initial_node->current_cost;

          queue.push(std::make_shared<Node>(
                       Node{
                         cost_estimate,
                         current_cost,
                         initial_waypoint,
                         orientation,
                         std::move(approach_trajectory),
                         nullptr,
                         rotated_initial_node
                       }));
        }
      }
      else
      {
        Trajectory initial_trajectory{map_name};
        initial_trajectory.insert(
              initial_time,
              _context.profile,
              to_3d(wp_location, initial_orientation),
              Eigen::Vector3d::Zero());

        queue.push(std::make_shared<Node>(
                     Node{
                       cost_estimate,
                       0.0,
                       initial_waypoint,
                       initial_orientation,
                       std::move(initial_trajectory),
                       nullptr,
                       nullptr,
                       start_index
                     }));
      }
    }
  }

  bool is_finished(const NodePtr& node) const
  {
    if(*node->waypoint != _context.final_waypoint)
      return false;

    if(_context.final_orientation)
    {
      if(std::abs(node->orientation - *_context.final_orientation)
         > _context.interpolate.rotation_thresh)
        return false;
    }

    return true;
  }

  bool is_valid(const Trajectory& trajectory)
  {
    assert(trajectory.size() > 1);
    _query.spacetime().timespan()->set_lower_time_bound(
          *trajectory.start_time());
    _query.spacetime().timespan()->set_upper_time_bound(
          *trajectory.finish_time());

    // TODO(MXG): When we start generating plans across multiple maps, we should
    // account for the trajectory's map name(s) here.
    const auto view = _context.viewer.query(_query);

    const auto& ignore_schedule_ids = _context.ignore_schedule_ids;
    if (ignore_schedule_ids.empty())
    {
      for (const auto& check : view)
      {
        assert(trajectory.size() > 1);
        assert(check.trajectory.size() > 1);
        if(!DetectConflict::between(trajectory, check.trajectory, true).empty())
          return false;
      }
    }
    else
    {
      for (const auto& check : view)
      {
        assert(trajectory.size() > 1);
        assert(check.trajectory.size() > 1);
        if (ignore_schedule_ids.count(check.id) > 0)
          continue;

        if(!DetectConflict::between(trajectory, check.trajectory, true).empty())
          return false;
      }
    }

    return true;
  }

  NodePtr expand_rotation(
      const NodePtr& parent_node,
      const double target_orientation)
  {
    const std::size_t waypoint = *parent_node->waypoint;
    Trajectory trajectory{_context.graph.waypoints[waypoint].get_map_name()};
    const Trajectory::Segment& last =
        parent_node->trajectory_from_parent.back();

    const Eigen::Vector3d& p = last.get_finish_position();
    trajectory.insert(last);

    // TODO(MXG): Consider storing these traits as POD member fields of the
    // context to reduce the dereferencing cost here.
    const auto& rotational = _context.traits.rotational();
    agv::internal::interpolate_rotation(
          trajectory,
          rotational.get_nominal_velocity(),
          rotational.get_nominal_acceleration(),
          last.get_finish_time(),
          p,
          Eigen::Vector3d(p[0], p[1], target_orientation),
          _context.profile,
          _context.interpolate.rotation_thresh);

    if(is_valid(trajectory))
    {
      return std::make_shared<Node>(
            Node{
              _context.heuristic.estimate_remaining_cost(_context, waypoint),
              compute_current_cost(parent_node, trajectory),
              waypoint,
              target_orientation,
              std::move(trajectory),
              nullptr,
              parent_node
            });
    }

    return nullptr;
  }

  bool is_orientation_okay(
      const Eigen::Vector2d& initial_p,
      const double orientation,
      const Eigen::Vector2d& course,
      const agv::Graph::Lane& lane) const
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

      if(std::abs(rmf_utils::wrap_to_pi(orientation - position[2]))
         > _context.interpolate.rotation_thresh)
        return false;
    }

    return true;
  }

  std::vector<NodePtr> expand_rotations(
      const NodePtr& parent_node,
      const std::size_t lane_index)
  {
    const agv::Graph::Lane& lane = _context.graph.lanes[lane_index];

    const agv::Graph::Waypoint& initial_waypoint =
        _context.graph.waypoints[lane.entry().waypoint_index()];
    const Eigen::Vector2d& initial_p = initial_waypoint.get_location();

    const agv::Graph::Waypoint& next_waypoint =
        _context.graph.waypoints[lane.exit().waypoint_index()];
    const Eigen::Vector2d& next_p = next_waypoint.get_location();

    const Eigen::Vector2d course = (next_p - initial_p).normalized();

    const std::vector<double> orientations =
        _differential_constraint.get_orientations(course);

    std::vector<NodePtr> rotations;
    rotations.reserve(orientations.size());
    for(const double orientation : orientations)
    {
      if(!is_orientation_okay(initial_p, orientation, course, lane))
        continue;

      if(std::abs(rmf_utils::wrap_to_pi(orientation - parent_node->orientation))
         < _context.interpolate.rotation_thresh)
      {
        // No rotation is needed to reach this orientation
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

  NodePtr make_if_valid(
      const std::size_t waypoint,
      const double orientation,
      const NodePtr& parent_node,
      Trajectory trajectory,
      agv::Graph::Lane::EventPtr event = nullptr)
  {
    assert(trajectory.size() > 1);
    if(is_valid(trajectory))
    {
      return std::make_shared<Node>(
            Node{
              _context.heuristic.estimate_remaining_cost(_context, waypoint),
              compute_current_cost(parent_node, trajectory),
              waypoint,
              orientation,
              std::move(trajectory),
              std::move(event),
              parent_node
            });
    }

    return nullptr;
  }

  bool add_if_valid(
      const std::size_t waypoint,
      const double orientation,
      const NodePtr& parent_node,
      Trajectory trajectory,
      SearchQueue& queue,
      agv::Graph::Lane::EventPtr event = nullptr)
  {
    const auto node = make_if_valid(
          waypoint,
          orientation,
          parent_node,
          std::move(trajectory),
          std::move(event));

    if(node)
    {
      // TODO(MXG): Consider short-circuiting the rest of the search and
      // returning the solution if this Node solves the search problem. It could
      // be an optional behavior configurable from the Planner::Options.
      queue.push(node);
      return true;
    }

    return false;
  }

  struct LaneExpansionNode
  {
    std::size_t lane;
  };

  void expand_down_lane(
      NodePtr initial_parent,
      const std::size_t initial_lane_index,
      SearchQueue& queue)
  {
    const std::size_t initial_waypoint = *initial_parent->waypoint;
    assert(_context.graph.lanes[initial_lane_index].entry().waypoint_index()
           == initial_waypoint);
    const Eigen::Vector2d initial_p =
        _context.graph.waypoints[initial_waypoint].get_location();
    const double orientation = initial_parent->orientation;

    const auto& initial_lane = _context.graph.lanes[initial_lane_index];
    if (const auto* entry_event = initial_lane.entry().event())
    {
      initial_parent = entry_event->execute(
            _executor.update(initial_parent)).get(this);

      if (!initial_parent)
      {
        // The entry event was not feasible, so we will stop expanding
        return;
      }
    }

    const std::string map_name =
        _context.graph.waypoints[initial_waypoint].get_map_name();

    const Trajectory::Segment& initial_seg =
        initial_parent->trajectory_from_parent.back();

    const Time initial_time = initial_seg.get_finish_time();
    const Eigen::Vector3d initial_position = initial_seg.get_finish_position();

    std::vector<LaneExpansionNode> lane_expansion_queue;
    lane_expansion_queue.push_back({initial_lane_index});

    while (!lane_expansion_queue.empty())
    {
      const LaneExpansionNode top = std::move(lane_expansion_queue.back());
      lane_expansion_queue.pop_back();

      const agv::Graph::Lane& lane = _context.graph.lanes[top.lane];
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();
      const agv::Graph::Waypoint& exit_waypoint =
          _context.graph.waypoints[exit_waypoint_index];

      const Eigen::Vector2d& next_p = exit_waypoint.get_location();
      const Eigen::Vector3d next_position{next_p[0], next_p[1], orientation};

      // TODO(MXG): Figure out what to do if the trajectory spans across
      // multiple maps.
      Trajectory trajectory{map_name};
      trajectory.insert(initial_seg);
      agv::internal::interpolate_translation(
            trajectory,
            _context.traits.linear().get_nominal_velocity(),
            _context.traits.linear().get_nominal_acceleration(),
            initial_time,
            initial_position,
            next_position,
            _context.profile,
            _context.interpolate.translation_thresh);

      if (const auto* event = lane.exit().event())
      {
        if(!is_valid(trajectory))
          continue;

        auto parent_to_event = std::make_shared<Node>(
              Node{
                _context.heuristic.estimate_remaining_cost(
                    _context, exit_waypoint_index),
                compute_current_cost(initial_parent, trajectory),
                exit_waypoint_index,
                orientation,
                std::move(trajectory),
                nullptr,
                initial_parent
              });

        event->execute(_executor.update(parent_to_event))
            .add_if_valid(this, queue);

        continue;
      }
      // NOTE(MXG): We cannot move the trajectory in this function call, because
      // we may need to copy the trajectory later when we expand further down
      // other lanes.
      else if (!add_if_valid(exit_waypoint_index, orientation, initial_parent,
                        trajectory, queue))
      {
        // This lane was not successfully added, so we should not try to expand
        // this any further.
        continue;
      }

      // If this lane was successfully added, we can try to find more lanes to
      // continue down, as a single expansion from the original parent.
      const std::vector<std::size_t>& lanes =
          _context.graph.lanes_from[exit_waypoint_index];

      for (const std::size_t l : lanes)
      {
        const agv::Graph::Lane& future_lane = _context.graph.lanes[l];

        const Eigen::Vector2d future_p =
            _context.graph.waypoints[future_lane.exit().waypoint_index()]
            .get_location();

        const Eigen::Vector2d course = future_p - initial_p;

        const bool check_orientation =
            is_orientation_okay(initial_p, orientation, course, future_lane);

        if (!check_orientation)
        {
          // The orientation needs to be changed to go down this lane, so we
          // should not expand in this direction
          continue;
        }

        if (future_lane.entry().event())
        {
          // An event needs to take place before proceeding down this lane, so
          // we should not expand in this direction
          continue;
        }

        const Eigen::Vector3d future_position{
          future_p[0], future_p[1], orientation
        };

        if(!agv::internal::can_skip_interpolation(
             initial_position, next_position,
             future_position, _context.interpolate))
        {
          continue;
        }

        lane_expansion_queue.push_back({l});
      }
    }
  }

  void expand_lane(
      const NodePtr& initial_parent,
      const std::size_t initial_lane_index,
      SearchQueue& queue)
  {
    const auto rotations = expand_rotations(initial_parent, initial_lane_index);
    for (const auto& parent : rotations)
      expand_down_lane(parent, initial_lane_index, queue);
  }

  NodePtr make_delay(
      const std::size_t waypoint,
      const NodePtr& parent_node,
      const Duration delay,
      agv::Graph::Lane::EventPtr event = nullptr)
  {
    const Trajectory& parent_trajectory = parent_node->trajectory_from_parent;
    const auto& initial_segment = parent_trajectory.back();

    Trajectory trajectory{_context.graph.waypoints[waypoint].get_map_name()};

    const Time initial_time = initial_segment.get_finish_time();
    const Eigen::Vector3d& initial_pos = initial_segment.get_finish_position();
    trajectory.insert(initial_segment);

    trajectory.insert(
          initial_time + delay,
          _context.profile,
          initial_pos,
          Eigen::Vector3d::Zero());
    assert(trajectory.size() == 2);

    return make_if_valid(
          waypoint, initial_pos[2], parent_node,
          std::move(trajectory), std::move(event));
  }

  void expand_delay(
      const std::size_t waypoint,
      const NodePtr& parent_node,
      const Duration delay,
      SearchQueue& queue,
      agv::Graph::Lane::EventPtr event = nullptr)
  {
    const auto node = make_delay(
          waypoint, parent_node, delay, std::move(event));

    if(node)
      queue.push(node);
  }

  void expand_holding(
      const std::size_t waypoint,
      const NodePtr& parent_node,
      SearchQueue& queue)
  {
    expand_delay(waypoint, parent_node, _context.holding_time, queue);
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    const std::size_t parent_waypoint = *parent_node->waypoint;
    if(parent_waypoint == _context.final_waypoint)
    {
      if(!_context.final_orientation)
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

      const double final_orientation =
          rmf_utils::wrap_to_pi(*_context.final_orientation);
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
        _context.graph.lanes_from[parent_waypoint];

    for (const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);

    if (_context.graph.waypoints[parent_waypoint].is_holding_point())
      expand_holding(parent_waypoint, parent_node, queue);
  }


private:

  Context& _context;
  schedule::Query _query;
  DifferentialDriveConstraint _differential_constraint;
  LaneEventExecutor _executor;
};

//==============================================================================
namespace {
class DifferentialDriveCache : public Cache
{
public:

  using Heuristic = DifferentialDriveExpander::Heuristic;
  using NodePtr = DifferentialDriveExpander::NodePtr;

  DifferentialDriveCache(agv::Planner::Configuration config)
  : _config(std::move(config)),
    _graph(agv::Graph::Implementation::get(_config.graph())),
    _traits(_config.vehicle_traits()),
    _profile(_traits.get_profile()),
    _interpolate(agv::Interpolate::Options::Implementation::get(
                   _config.interpolation()))
  {
    // Do nothing
  }

  CachePtr clone() const final
  {
    return std::make_shared<DifferentialDriveCache>(*this);
  }

  void update(const Cache& newer_cache) override final
  {
    const auto& newer = static_cast<const DifferentialDriveCache&>(newer_cache);

    for(const auto& h : newer._heuristics)
    {

      auto& heuristic = _heuristics.insert(
            std::make_pair(h.first, Heuristic{})).first->second;

      heuristic.update(h.second);
    }
  }

  rmf_utils::optional<Result> plan(
      const std::vector<agv::Planner::Start>& starts,
      agv::Planner::Goal goal,
      agv::Planner::Options options) final
  {
    if (starts.empty())
      return rmf_utils::nullopt;

    const std::size_t goal_waypoint = goal.waypoint();
    Heuristic& h = _heuristics.insert(
          std::make_pair(goal_waypoint, Heuristic{})).first->second;
    const bool* const interrupt_flag = options.interrupt_flag();

    const NodePtr solution = search<DifferentialDriveExpander>(
          DifferentialDriveExpander::Context{
            _graph,
            _traits,
            _profile,
            options.minimum_holding_time(),
            _interpolate,
            options.schedule_viewer(),
            goal_waypoint,
            goal.orientation(),
            starts.front().time(),
            interrupt_flag,
            options.ignore_schedule_ids(),
            h
          },
          DifferentialDriveExpander::InitialNodeArgs{starts},
          interrupt_flag);

    if (!solution)
      return rmf_utils::nullopt;

    auto trajectories = reconstruct_trajectories(solution);
    auto waypoints = reconstruct_waypoints(solution, _graph);
    auto start_index = find_start_index(solution);

    return Result{
        std::move(trajectories),
        std::move(waypoints),
        starts[start_index],
        std::move(goal),
        std::move(options)
    };
  }

  const agv::Planner::Configuration& get_configuration() const final
  {
    return _config;
  }

private:

  agv::Planner::Configuration _config;

  const agv::Graph::Implementation& _graph;
  const agv::VehicleTraits& _traits;
  const Trajectory::ConstProfilePtr& _profile;
  const agv::Interpolate::Options::Implementation& _interpolate;

  // This maps from a goal waypoint to the cached Heuristic object that tries to
  // plan to that goal waypoint.
  using HeuristicDatabase = std::unordered_map<std::size_t, Heuristic>;
  HeuristicDatabase _heuristics;
};
} // anonymous namespace

//==============================================================================
CacheManager make_cache(agv::Planner::Configuration config)
{
  if(config.vehicle_traits().get_differential())
  {
    return CacheManager(std::make_shared<DifferentialDriveCache>(
                          std::move(config)));
  }

  throw std::runtime_error(
        "[rmf_traffic::agv::Planner] Planning utilities are currently only "
        "implemented for AGVs that use a differential drive.");
}

} // namespace planning
} // namespace internal
} // namespace rmf_traffic
