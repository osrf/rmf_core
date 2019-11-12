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
#include "internal_planning.hpp"
#include "GraphInternal.hpp"

#include "../utils.hpp"

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
  _copy = original->clone();
}

//==============================================================================
Result CacheHandle::plan(
    agv::Planner::Start start,
    agv::Planner::Goal goal,
    agv::Planner::Options options)
{
  return _copy->plan(std::move(start), std::move(goal), std::move(options));
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
template<
    class Expander,
    class Context = typename Expander::Context,
    class InitialNodeArgs = typename Expander::InitialNodeArgs,
    class NodePtr = typename Expander::NodePtr>
NodePtr search(
    Context&& context,
    InitialNodeArgs&& initial_node_args)
{
  using SearchQueue = typename Expander::SearchQueue;

  Expander expander(context);

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

  return nullptr;
}


//==============================================================================
struct EuclideanExpander
{
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

  NodePtr make_initial_node(const InitialNodeArgs& args)
  {
    const Eigen::Vector2d location =
        context.graph.waypoints[args.waypoint].get_location();

    return std::make_shared<Node>(
          Node{
            args.waypoint,
            estimate_remaining_cost(location),
            0.0,
            location,
            nullptr
          });
  }

  bool is_finished(const NodePtr& node)
  {
    return node->waypoint == context.final_waypoint;
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

    const double cost = parent_node->current_cost + (p_exit - p_start).norm();
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
struct DifferentialDriveExpander
{
  struct Node;
  using NodePtr = std::shared_ptr<Node>;

  struct Node
  {
    double remaining_cost_estimate;
    double current_cost;
    std::size_t waypoint;
    double orientation;
    Trajectory trajectory_from_parent;
    NodePtr parent;
  };

  using SearchQueue =
      std::priority_queue<NodePtr, std::vector<NodePtr>, Compare<NodePtr>>;

  struct Context;

  struct InitialNodeArgs
  {
    std::size_t waypoint;
    double orientation;
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
              EuclideanExpander::InitialNodeArgs{waypoint});

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
    const Trajectory::ConstProfilePtr profile;
    const Duration holding_time;
    const agv::Interpolate::Options::Implementation interpolate;
    const schedule::Viewer& viewer;
    const std::size_t final_waypoint;
    const double* const final_orientation;
    const rmf_traffic::Time initial_time;
    Heuristic& heuristic;
  };

  DifferentialDriveExpander(Context& context)
  : _context(context),
    _query(schedule::make_query({}, nullptr, nullptr))
  {
    // Do nothing
  }

  NodePtr make_initial_node(const InitialNodeArgs& args)
  {
    const std::size_t waypoint = args.waypoint;

    const std::string& map_name =
        _context.graph.waypoints[args.waypoint].get_map_name();

    // TODO(MXG): We should be able to make this query more efficient, e.g.
    // add a lower bound on time.
    _query.spacetime().timespan()->add_map(map_name);

    Trajectory initial(map_name);
    initial.insert(
          _context.initial_time, _context.profile,
          to_3d(_context.graph.waypoints[args.waypoint].get_location(),
                args.orientation),
          Eigen::Vector3d::Zero());

    return std::make_shared<Node>(
          Node{
            _context.heuristic.estimate_remaining_cost(_context, waypoint),
            0.0,
            waypoint,
            args.orientation,
            initial,
            nullptr
          });
  }

  bool is_finished(const NodePtr& node) const
  {
    if(node->waypoint != _context.final_waypoint)
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

    for(const auto& check : view)
    {
      assert(trajectory.size() > 1);
      assert(check.size() > 1);
      if(!DetectConflict::between(trajectory, check).empty())
        return false;
    }

    return true;
  }

  NodePtr expand_rotation(
      const NodePtr& parent_node,
      const double target_orientation)
  {
    const std::size_t waypoint = parent_node->waypoint;
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
              parent_node
            });
    }

    return nullptr;
  }

  void expand(const NodePtr& parent_node, SearchQueue& queue)
  {
    const std::size_t parent_waypoint = parent_node->waypoint;
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

      const double final_orientation = wrap_to_pi(*_context.final_orientation);
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
  }


private:

  Context& _context;
  schedule::Query _query;
};

//==============================================================================
class DifferentialDriveCache : public Cache
{
public:

  using Heuristic = DifferentialDriveExpander::Heuristic;

  DifferentialDriveCache(agv::Planner::Configuration config)
  : graph(agv::Graph::Implementation::get(config.graph())),
    traits(config.vehicle_traits())
  {
    // Do nothing
  }

  CachePtr clone() const override final
  {
    return std::make_shared<DifferentialDriveCache>(*this);
  }

  void update(const Cache& newer_cache) override final
  {
    const auto& newer = static_cast<const DifferentialDriveCache&>(newer_cache);

    for(const auto& h : newer.heuristics)
    {

      auto& heuristic =
          heuristics.insert(std::make_pair(h.first, Heuristic{})).first->second;

      heuristic.update(h.second);
    }
  }

  Result plan(
      agv::Planner::Start start,
      agv::Planner::Goal goal,
      agv::Planner::Options options) override final
  {
    const std::string& map_name =
        graph.waypoints[start.waypoint()].get_map_name();

    Result result{false, {map_name}, {}, start, goal, options};
    const std::size_t goal_waypoint = goal.waypoint();
    Heuristic& h = heuristics.insert(
          std::make_pair(goal_waypoint, Heuristic{})).first->second;

    const DifferentialDriveExpander::NodePtr solution =
        search<DifferentialDriveExpander>(
          DifferentialDriveExpander::Context{
            graph,
            traits,
            goal_waypoint,
            goal.orientation(),
            start.time(),
            h
          },
          DifferentialDriveExpander::InitialNodeArgs{
            start.waypoint(),
            start.orientation()
          });


  }

private:

  const agv::Graph::Implementation& graph;
  const agv::VehicleTraits& traits;

  // This maps from a goal waypoint to the cached Heuristic object that tries to
  // plan to that goal waypoint.
  using HeuristicDatabase = std::unordered_map<std::size_t, Heuristic>;
  HeuristicDatabase heuristics;
};

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

//==============================================================================


} // namespace planning
} // namespace internal
} // namespace rmf_traffic
