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

#include "internal_planning.hpp"
#include "GraphInternal.hpp"

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
    class InitialNodeArgs = typename Expander::InitialNodeArgs,
    class NodePtr = typename Expander::NodePtr>
NodePtr search(
    const typename Expander::Context& context,
    typename Expander::Heuristic& heuristic,
    InitialNodeArgs initial_node_args)
{
  using SearchQueue = typename Expander::SearchQueue;

  Expander expander(std::move(context), heuristic);

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

  class Heuristic
  {
  public:

    Heuristic(const Context& context)
      : p_final(context.graph.waypoints[context.final_waypoint].get_location())
    {
      // Do nothing
    }

    double estimate_remaining_cost(const Eigen::Vector3d& p)
    {
      return (p_final - p).norm();
    }

  private:
    Eigen::Vector2d p_final;
  };

  EuclideanExpander(
      const Context& context,
      Heuristic& heuristic)
    : context(context),
      heuristic(heuristic)
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
            heuristic.estimate_remaining_cost(location),
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
    const Eigen::Vector3d p_exit =
        context.graph.waypoints[exit_waypoint_index].get_location();

    const double cost = parent_node->current_cost + (p_exit - p_start).norm();
    queue.push(std::make_shared<Node>(
                 Node{
                   exit_waypoint_index,
                   heuristic.estimate_remaining_cost(p_exit),
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
  Heuristic& heuristic;
  std::unordered_set<std::size_t> expanded;
};

//==============================================================================
struct DifferentialDriverExpander
{
  struct Node
  {
    std::size_t waypoint;
    double orientation;
  };

  struct Context
  {
    const agv::Graph::Implementation& graph;
    const std::size_t final_waypoint;
    const double* const final_orientation;
  };

  class Heuristic
  {
  public:

    double estimate_remaining_cost(const Node& node)
    {
      auto estimate_it = known_costs.insert(
            {node.waypoint, std::numeric_limits<double>::infinity()});

      if(estimate_it.second)
      {
        const EuclideanExpander::Context c{
          context.graph, context.final_waypoint};
        EuclideanExpander::Heuristic h{c};
        const EuclideanExpander::NodePtr goal = search<EuclideanExpander>(
              c, h, EuclideanExpander::InitialNodeArgs{node.waypoint});


      }

    }

    void update(const Heuristic& other)
    {
      for(const auto& wp_costs : other.known_costs)
        known_costs.insert(wp_costs);
    }

  private:
    std::unordered_map<std::size_t, double> known_costs;
    const Context& context;
  };
};

//==============================================================================
class DifferentialDriveCache : public Cache
{
public:

  using Heuristic = DifferentialDriverExpander::Heuristic;

  DifferentialDriveCache(agv::Planner::Configuration config)
    : graph(agv::Graph::Implementation::get(config.graph()))
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

  }

private:

  const agv::Graph::Implementation& graph;

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
