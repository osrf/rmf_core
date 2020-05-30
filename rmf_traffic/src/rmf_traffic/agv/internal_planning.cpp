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

#include "../RouteInternal.hpp"

#include <rmf_utils/math.hpp>

#include <rmf_traffic/DetectConflict.hpp>

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
Cache* CacheHandle::operator->()
{
  return _copy.get();
}

//==============================================================================
const Cache* CacheHandle::operator->() const
{
  return _copy.get();
}

//==============================================================================
Cache& CacheHandle::operator*() &
{
  return *_copy.get();
}

//==============================================================================
const Cache& CacheHandle::operator*() const&
{
  return *_copy.get();
}

//==============================================================================
CacheHandle::~CacheHandle()
{
  if (!_original)
    return;

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
  class SearchQueue = typename Expander::SearchQueue,
  class NodePtr = typename Expander::NodePtr>
NodePtr search(
  Expander& expander,
  SearchQueue& queue,
  const bool* interrupt_flag)
{
  while (!queue.empty() && !(interrupt_flag && *interrupt_flag))
  {
    NodePtr top = queue.top();
    if (expander.quit(top))
      return nullptr;

    // This pop must only happen after we have decided whether or not we are
    // quitting. If we pop before quitting, then we will lose this node forever.
    queue.pop();

    if (expander.is_finished(top))
      return top;

    expander.expand(top, queue);
  }

  return nullptr;
}

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while (node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  return node_sequence;
}

namespace {
//==============================================================================
template<typename NodePtr>
void reparent_node_for_holding(
    const NodePtr& low_node,
    const NodePtr& high_node,
    RouteData route)
{
  high_node->parent = low_node;
  high_node->route_from_parent = std::move(route);
}

//==============================================================================
template<typename NodePtr>
struct OrientationTimeMap
{
  struct Element
  {
    using TimeMap = std::map<rmf_traffic::Time, NodePtr>;
    using TimePair = std::pair<
      typename TimeMap::iterator,
      typename TimeMap::iterator
    >;

    double orientation;
    TimeMap time_map;

    void squash(const agv::RouteValidator* validator)
    {
      assert(!time_map.empty());
      if (time_map.size() <= 2)
        return;

      std::vector<TimePair> queue;
      queue.push_back({time_map.begin(), --time_map.end()});

      while (!queue.empty())
      {
        const auto top = queue.back();
        queue.pop_back();
        const auto it_low = top.first;
        const auto it_high = top.second;

        assert(it_low != time_map.end());
        assert(it_high != time_map.end());

        if (it_high->first <= it_low->first)
          continue;

        if (it_low == --typename TimeMap::iterator(it_high))
        {
          // If the iterators are perfectly next to each other, then the only
          // action between them must be a holding.
          continue;
        }

        assert(it_high != time_map.begin());

        const auto& node_low = it_low->second;
        const auto& node_high = it_high->second;
        assert(node_low->route_from_parent.map
               == node_high->route_from_parent.map);

        const auto& start_wp = node_low->route_from_parent.trajectory.back();
        const auto& end_wp = node_high->route_from_parent.trajectory.back();

        RouteData new_route;
        new_route.map = node_low->route_from_parent.map;
        new_route.trajectory.insert(start_wp);
        new_route.trajectory.insert(
              end_wp.time(),
              end_wp.position(),
              Eigen::Vector3d::Zero());

        if (!validator || !validator->find_conflict(RouteData::make(new_route)))
        {
          reparent_node_for_holding(node_low, node_high, std::move(new_route));
          time_map.erase(++typename TimeMap::iterator(it_low), it_high);
          queue.clear();
          if (time_map.size() > 2)
            queue.push_back({time_map.begin(), --time_map.end()});

          continue;
        }

        const auto next_high = --typename TimeMap::iterator(it_high);
        const auto next_low = ++typename TimeMap::iterator(it_low);

        if (next_high != it_low)
          queue.push_back({it_low, next_high});

        if (next_low != it_high)
          queue.push_back({next_low, it_high});
      }
    }
  };

  void insert(NodePtr node)
  {
    const auto orientation = node->orientation;
    const auto time = *node->route_from_parent.trajectory.finish_time();

    auto it = elements.begin();
    for (; it != elements.end(); ++it)
    {
      if (std::abs(it->orientation - orientation) < 15.0*M_PI/180.0)
        break;
    }

    if (it == elements.end())
      elements.push_back({orientation, {{time, node}}});
    else
      it->time_map.insert({time, node});
  }

  std::vector<Element> elements;
};
} // anonymous namespace

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(
    const NodePtr& finish_node,
    const agv::RouteValidator* validator)
{
  auto node_sequence = reconstruct_nodes(finish_node);

  // Remove "cruft" from plans. This means making sure vehicles don't do any
  // unnecessary motions.
  std::unordered_map<
    std::size_t,
    OrientationTimeMap<NodePtr>
  > cruft_map;

  for (const auto& node : node_sequence)
  {
    if (!node->waypoint)
      continue;

    const auto wp = *node->waypoint;
    cruft_map[wp].insert(node);
  }

  for (auto& cruft : cruft_map)
  {
    for (auto& duplicate : cruft.second.elements)
      duplicate.squash(validator);
  }

  return reconstruct_nodes(finish_node);
}

//==============================================================================
template<typename NodePtr>
std::vector<Route> reconstruct_routes(
    const std::vector<NodePtr>& node_sequence,
    rmf_utils::optional<rmf_traffic::Duration> span = rmf_utils::nullopt)
{
  if (node_sequence.size() == 1)
  {
    // If there is only one node in the sequence, then it is a start node.
    if (span)
    {
      // When performing a rollout, it is important that at least one route with
      // two waypoints is provided. We use the span value to creating a
      // stationary trajectory when the robot is already starting out at a
      // holding point.
      std::vector<Route> output;
      Route simple_route =
          RouteData::make(node_sequence.back()->route_from_parent);
      if (simple_route.trajectory().size() < 2)
      {
        const auto& wp = simple_route.trajectory().back();
        simple_route.trajectory().insert(
              wp.time() + *span,
              wp.position(),
              Eigen::Vector3d::Zero());
      }

      output.emplace_back(std::move(simple_route));
    }

    // When there is only one node, we should return an empty itinerary to 
    // indicate that the AGV does not need to go anywhere.
    return {};
  }

  std::vector<RouteData> routes;
  routes.push_back(RouteData{node_sequence.back()->route_from_parent.map, {}});

  // We exclude the first node in the sequence, because it contains a dummy
  // trajectory which is not helpful.
  const auto stop_it = node_sequence.rend();
  for (auto it = ++node_sequence.rbegin(); it != stop_it; ++it)
  {
    RouteData& last_route = routes.back();
    const RouteData& next_route = (*it)->route_from_parent;
    if (next_route.map == last_route.map)
    {
      for (const auto& waypoint : next_route.trajectory)
        last_route.trajectory.insert(waypoint);
    }
    else
    {
      routes.push_back(next_route);
    }
  }

  std::vector<Route> output;
  output.reserve(routes.size());
  for (RouteData& route : routes)
    output.emplace_back(RouteData::make(std::move(route)));

  return output;
}

//==============================================================================
template<typename NodePtr>
std::vector<agv::Plan::Waypoint> reconstruct_waypoints(
  const std::vector<NodePtr>& node_sequence,
  const agv::Graph::Implementation& graph)
{
  std::vector<agv::Plan::Waypoint> waypoints;
  for (auto it = node_sequence.rbegin(); it != node_sequence.rend(); ++it)
  {
    const auto& n = *it;
    const Eigen::Vector2d p = n->waypoint ?
      graph.waypoints[*n->waypoint].get_location() :
      n->route_from_parent.trajectory.back().position()
      .template block<2, 1>(0, 0);
    const Time time{*n->route_from_parent.trajectory.finish_time()};
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

  bool quit(const NodePtr&) const
  {
    return false;
  }

  static double lane_event_cost(const agv::Graph::Lane& lane)
  {
    double cost = 0.0;
    if (const auto* event = lane.entry().event())
      cost += time::to_seconds(event->duration());

    if (const auto* event = lane.exit().event())
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
    if (expanded.count(exit_waypoint_index) > 0)
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

    for (const std::size_t l : lanes)
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

    if (reversible)
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
    RouteData route_from_parent;
    agv::Graph::Lane::EventPtr event;
    NodePtr parent;
    rmf_utils::optional<agv::Plan::Start> start = rmf_utils::nullopt;
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

      if (estimate_it.second)
      {
        // The pair was inserted, which implies that the cost estimate for this
        // waypoint has never been found before, and we should compute it now.
        auto euclidean_context = EuclideanExpander::Context{
          context.graph,
          context.final_waypoint
        };
        EuclideanExpander expander(euclidean_context);

        EuclideanExpander::SearchQueue euclidean_queue;
        expander.make_initial_nodes(
              EuclideanExpander::InitialNodeArgs{waypoint},
              euclidean_queue);

        const EuclideanExpander::NodePtr solution =
            search<EuclideanExpander>(expander, euclidean_queue, nullptr);

        // TODO(MXG): Instead of asserting that the goal exists, we should
        // probably take this opportunity to shortcircuit the planner and return
        // that there is no solution.
        assert(solution != nullptr);

        std::vector<Eigen::Vector3d> positions;
        EuclideanExpander::NodePtr euclidean_node = solution;
        // Note: this constructs positions in reverse, but that's okay because
        // the Interpolate::positions function will produce a trajectory with the
        // same duration whether it is interpolating forward or backwards.
        while (euclidean_node)
        {
          const Eigen::Vector2d p = euclidean_node->location;
          positions.push_back({p[0], p[1], 0.0});
          euclidean_node = euclidean_node->parent;
        }

        // We pass in context.initial_time here because we don't actually care
        // about the Trajectory's start/end time being correct; we only care
        // about the difference between the two.
        const rmf_traffic::Trajectory estimate = agv::Interpolate::positions(
          context.traits, context.initial_time, positions);

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
      for (const auto& wp_costs : other.known_costs)
        known_costs.insert(wp_costs);
    }

  private:
    std::unordered_map<std::size_t, double> known_costs;
  };

  struct Context
  {
    const agv::Graph::Implementation& graph;
    const agv::VehicleTraits& traits;
    const Profile& profile;
    const Duration holding_time;
    const agv::Interpolate::Options::Implementation& interpolate;
    const agv::RouteValidator* const validator;
    const std::size_t final_waypoint;
    const rmf_utils::optional<double> final_orientation;
    const rmf_utils::optional<double> maximum_cost_estimate;
    const bool* const interrupt_flag;
    Heuristic& heuristic;
    Issues::BlockerMap& blockers;
    const bool simple_lane_expansion; // reduces branching factor when true
    const rmf_traffic::Time initial_time = rmf_traffic::Time(rmf_traffic::Duration(0));
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

  // TODO(MXG): This will only ever return 0, 1, or 2 routes, so a bounded
  // vector would be preferable.
  std::vector<RouteData> make_start_approach_routes(
      const agv::Planner::Start& start)
  {
    std::vector<RouteData> output;

    const std::size_t initial_waypoint = start.waypoint();

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

      RouteData initial_route;
      initial_route.map = map_name;
      Trajectory& initial_trajectory = initial_route.trajectory;
      initial_trajectory.insert(
        initial_time,
        initial_position,
        Eigen::Vector3d::Zero());

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
            // *INDENT-OFF*
            throw std::invalid_argument(
              "[rmf_traffic::agv::Planner] Disagreement between initial "
              "waypoint index [" + std::to_string(
                initial_waypoint)
              + "] and the initial lane exit ["
              + std::to_string(lane_exit) + "]");
            // *INDENT-ON*
          }

          if (!is_orientation_okay(
              *initial_location, orientation, course, lane))
          {
            // We cannot approach the initial_waypoint with this orientation,
            // so we cannot use this orientation to start.
            continue;
          }
        }

        auto approach_route = initial_route;
        Trajectory& approach_trajectory = approach_route.trajectory;
        if (std::abs(rmf_utils::wrap_to_pi(orientation - initial_orientation))
          >= _context.interpolate.rotation_thresh)
        {
          const Eigen::Vector3d rotated_position =
            to_3d(*initial_location, orientation);

          const auto& rotational = _context.traits.rotational();

          // TODO(MXG): Consider refactoring this with the other spots where
          // we use interpolate_rotation
          agv::internal::interpolate_rotation(
            approach_trajectory,
            rotational.get_nominal_velocity(),
            rotational.get_nominal_acceleration(),
            initial_time,
            initial_position,
            rotated_position,
            _context.interpolate.rotation_thresh);
        }

        agv::internal::interpolate_translation(
          approach_trajectory,
          _context.traits.linear().get_nominal_velocity(),
          _context.traits.linear().get_nominal_acceleration(),
          *approach_trajectory.finish_time(),
          to_3d(*initial_location, orientation),
          to_3d(wp_location, orientation),
          _context.interpolate.translation_thresh);

        output.emplace_back(std::move(approach_route));
      }
    }

    return output;
  }

  void expand_start_routes(
      const NodePtr& start_node,
      const std::vector<RouteData>& routes,
      SearchQueue& queue)
  {
    assert(start_node->start);

    const std::size_t waypoint = start_node->start->waypoint();

    const double remaining_cost_estimate =
        _context.heuristic.estimate_remaining_cost(
          _context, waypoint);

    for (const auto& route : routes)
    {
      const double current_cost =
          rmf_traffic::time::to_seconds(route.trajectory.duration());

      // TODO(MXG): Should we consider lane entry events here?

      if (!is_valid(route, start_node))
        continue;

      queue.push(std::make_shared<Node>(
        Node{
          remaining_cost_estimate,
          current_cost,
          waypoint,
          route.trajectory.back().position()[2],
          route,
          nullptr,
          start_node
        }));
    }

    RouteData wait_route;
    wait_route.map = start_node->route_from_parent.map;
    Trajectory& wait_trajectory = wait_route.trajectory;
    wait_trajectory.insert(
      start_node->route_from_parent.trajectory.back());
    wait_trajectory.insert(
      wait_trajectory.back().time() + _context.holding_time,
      wait_trajectory.back().position(),
      Eigen::Vector3d::Zero());

    const double wait_cost =
        start_node->current_cost
        + rmf_traffic::time::to_seconds(wait_route.trajectory.duration());

    if (is_valid(wait_route, start_node))
    {
      queue.push(std::make_shared<Node>(
        Node{
          start_node->remaining_cost_estimate,
          wait_cost,
          start_node->waypoint,
          start_node->orientation,
          std::move(wait_route),
          nullptr,
          start_node,
          start_node->start
        }));
    }
  }

  void expand_start_node(
    const NodePtr& start_node,
    SearchQueue& queue)
  {
    expand_start_routes(
      start_node,
      make_start_approach_routes(*start_node->start),
      queue);
  }

  void make_initial_nodes(const InitialNodeArgs& args, SearchQueue& queue)
  {
    const std::size_t N_starts = args.starts.size();
    for (std::size_t start_index = 0; start_index < N_starts; ++start_index)
    {
      const auto& start = args.starts[start_index];
      const std::size_t initial_waypoint = start.waypoint();
      auto initial_routes = make_start_approach_routes(start);
      const double remaining_cost_estimate =
          _context.heuristic.estimate_remaining_cost(
            _context, initial_waypoint);

      double shortest_route_cost = initial_routes.empty()?
            0.0 : std::numeric_limits<double>::infinity();
      for (const auto& initial_route : initial_routes)
      {
        const double route_cost =
            rmf_traffic::time::to_seconds(initial_route.trajectory.duration());
        shortest_route_cost = std::min(shortest_route_cost, route_cost);
      }

      const auto& wp = _context.graph.waypoints[initial_waypoint];
      RouteData starting_point;
      starting_point.map = wp.get_map_name();
      Eigen::Vector3d location;
      if (start.location())
      {
        location.block<2,1>(0,0) = *start.location();
      }
      else
      {
        location.block<2,1>(0,0) = wp.get_location();
      }
      location[2] = start.orientation();

      starting_point.trajectory.insert(
          start.time(),
          location,
          Eigen::Vector3d::Zero());

      rmf_utils::optional<std::size_t> start_node_wp = rmf_utils::nullopt;
      if (!start.location())
        start_node_wp = initial_waypoint;

      queue.push(std::make_shared<Node>(
        Node{
          shortest_route_cost + remaining_cost_estimate,
          0.0,
          start_node_wp,
          start.orientation(),
          starting_point,
          nullptr,
          nullptr,
          start,
          start_index
        }));
    }
  }

  bool is_finished(const NodePtr& node) const
  {
    if (!node->waypoint)
      return false;

    if (*node->waypoint != _context.final_waypoint)
      return false;

    if (_context.final_orientation)
    {
      if (std::abs(node->orientation - *_context.final_orientation)
        > _context.interpolate.rotation_thresh)
        return false;
    }

    return true;
  }

  bool is_holding_point(rmf_utils::optional<std::size_t> waypoint) const
  {
    if (!waypoint)
      return false;

    return _context.graph.waypoints.at(*waypoint).is_holding_point();
  }

  bool quit(const NodePtr& node) const
  {
    if (!_context.maximum_cost_estimate.has_value())
      return false;

    const double cost_estimate =
        node->current_cost + node->remaining_cost_estimate;

    return (*_context.maximum_cost_estimate < cost_estimate);
  }

  bool is_valid(
      const RouteData& route,
      const NodePtr& parent)
  {
    if (_context.validator)
    {
      auto conflict = _context.validator->find_conflict(
            Route::Implementation::make(route));

      if (conflict)
      {
        auto time_it =
            _context.blockers[conflict->participant]
            .insert({parent, conflict->time});

        if (!time_it.second)
        {
          time_it.first->second =
              std::max(time_it.first->second, conflict->time);
        }

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
    RouteData route;
    route.map = _context.graph.waypoints[waypoint].get_map_name();
    Trajectory& trajectory = route.trajectory;
    const Trajectory::Waypoint& last =
      parent_node->route_from_parent.trajectory.back();

    const Eigen::Vector3d& p = last.position();
    trajectory.insert(last);

    // TODO(MXG): Consider storing these traits as POD member fields of the
    // context to reduce the dereferencing cost here.
    const auto& rotational = _context.traits.rotational();
    agv::internal::interpolate_rotation(
      trajectory,
      rotational.get_nominal_velocity(),
      rotational.get_nominal_acceleration(),
      last.time(),
      p,
      Eigen::Vector3d(p[0], p[1], target_orientation),
      _context.interpolate.rotation_thresh);

    if (is_valid(route, parent_node))
    {
      return std::make_shared<Node>(
        Node{
          _context.heuristic.estimate_remaining_cost(_context, waypoint),
          compute_current_cost(parent_node, trajectory),
          waypoint,
          target_orientation,
          std::move(route),
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
    for (const auto* constraint : {
        lane.entry().orientation_constraint(),
        lane.exit().orientation_constraint()
      })
    {
      if (!constraint)
        continue;

      Eigen::Vector3d position{initial_p[0], initial_p[1], orientation};
      if (!constraint->apply(position, course))
        return false;

      if (std::abs(rmf_utils::wrap_to_pi(orientation - position[2]))
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
    for (const double orientation : orientations)
    {
      if (!is_orientation_okay(initial_p, orientation, course, lane))
        continue;

      if (std::abs(rmf_utils::wrap_to_pi(orientation -
        parent_node->orientation))
        < _context.interpolate.rotation_thresh)
      {
        // No rotation is needed to reach this orientation
        rotations.push_back(parent_node);
      }
      else
      {
        const NodePtr rotation = expand_rotation(parent_node, orientation);
        if (rotation)
          rotations.push_back(rotation);
      }
    }

    return rotations;
  }

  NodePtr make_if_valid(
    const std::size_t waypoint,
    const double orientation,
    const NodePtr& parent_node,
    RouteData route,
    agv::Graph::Lane::EventPtr event = nullptr)
  {
    assert(route.trajectory.size() > 1);
    if (is_valid(route, parent_node))
    {
      return std::make_shared<Node>(
        Node{
          _context.heuristic.estimate_remaining_cost(_context, waypoint),
          compute_current_cost(parent_node, route.trajectory),
          waypoint,
          orientation,
          std::move(route),
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
    RouteData route,
    SearchQueue& queue,
    agv::Graph::Lane::EventPtr event = nullptr)
  {
    const auto node = make_if_valid(
      waypoint,
      orientation,
      parent_node,
      std::move(route),
      std::move(event));

    if (node)
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

    const Trajectory::Waypoint& initial_wp =
      initial_parent->route_from_parent.trajectory.back();

    const Time initial_time = initial_wp.time();
    const Eigen::Vector3d initial_position = initial_wp.position();

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
      RouteData route;
      route.map = map_name;
      Trajectory& trajectory = route.trajectory;
      trajectory.insert(initial_wp);
      agv::internal::interpolate_translation(
        trajectory,
        _context.traits.linear().get_nominal_velocity(),
        _context.traits.linear().get_nominal_acceleration(),
        initial_time,
        initial_position,
        next_position,
        _context.interpolate.translation_thresh);

      if (const auto* event = lane.exit().event())
      {
        if (!is_valid(route, initial_parent))
          continue;

        auto parent_to_event = std::make_shared<Node>(
          Node{
            _context.heuristic.estimate_remaining_cost(
              _context, exit_waypoint_index),
            compute_current_cost(initial_parent, trajectory),
            exit_waypoint_index,
            orientation,
            std::move(route),
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
        route, queue))
      {
        // This lane was not successfully added, so we should not try to expand
        // this any further.
        continue;
      }

//      std::cout << "Expanded down a lane" << std::endl;
      if (_context.simple_lane_expansion)
      {
        // This indicates that we do not want to spend time exploring every
        // possible way that a vehicle may move down a corridor. The planner
        // will assume that the vehicle always comes to a complete stop at every
        // waypoint that it passes over. This results in a plan that is not as
        // close to optimal, but it significantly reduces the branching factor.
        // This is currently only used for the Rollout feature to avoid
        // needlessly enormous sets of itinerary alternatives.
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

        if (!agv::internal::can_skip_interpolation(
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
    const Trajectory& parent_trajectory =
      parent_node->route_from_parent.trajectory;
    const auto& initial_segment = parent_trajectory.back();

    RouteData route;
    route.map = _context.graph.waypoints[waypoint].get_map_name();
    Trajectory& trajectory = route.trajectory;

    const Time initial_time = initial_segment.time();
    const Eigen::Vector3d& initial_pos = initial_segment.position();
    trajectory.insert(initial_segment);

    trajectory.insert(
      initial_time + delay,
      initial_pos,
      Eigen::Vector3d::Zero());
    assert(trajectory.size() == 2);

    return make_if_valid(
      waypoint, initial_pos[2], parent_node,
      std::move(route), std::move(event));
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

    if (node)
    {
//      std::cout << "Expand holding" << std::endl;
      queue.push(node);
    }
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
    const bool has_waypoint = parent_node->waypoint.has_value();
    if (has_waypoint)
    {
      const std::size_t parent_waypoint = *parent_node->waypoint;
      if (parent_waypoint == _context.final_waypoint)
      {
        if (!_context.final_orientation)
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
        if (final_node)
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

    if (parent_node->start && !has_waypoint)
    {
      expand_start_node(parent_node, queue);
      return;
    }

    assert(has_waypoint);
    const std::size_t parent_waypoint = *parent_node->waypoint;

    const std::vector<std::size_t>& lanes =
      _context.graph.lanes_from[parent_waypoint];

    for (const std::size_t l : lanes)
      expand_lane(parent_node, l, queue);

    if (!_context.graph.waypoints[parent_waypoint].is_passthrough_point())
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
  using Node = DifferentialDriveExpander::Node;

  DifferentialDriveCache(agv::Planner::Configuration config)
  : _config(std::move(config)),
    _graph(agv::Graph::Implementation::get(_config.graph())),
    _traits(_config.vehicle_traits()),
    _profile(_traits.profile()),
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

    for (const auto& h : newer._heuristics)
    {
      auto& heuristic = _heuristics.insert(
        std::make_pair(h.first, Heuristic{})).first->second;

      heuristic.update(h.second);
    }
  }

  class InternalState : public State::Internal
  {
  public:
    DifferentialDriveExpander::SearchQueue queue;

    rmf_utils::optional<double> cost_estimate() const final
    {
      if (queue.empty())
        return rmf_utils::nullopt;

      const auto& top = queue.top();
      return top->current_cost + top->remaining_cost_estimate;
    }
  };

  State initiate(
      const std::vector<agv::Planner::Start>& starts,
      agv::Planner::Goal goal,
      agv::Planner::Options options) final
  {
    State state{
      Conditions{
        starts,
        std::move(goal),
        std::move(options)
      },
      Issues{},
      rmf_utils::make_derived_impl<State::Internal, InternalState>()
    };

    auto context = make_context(
          state.conditions.goal,
          state.conditions.options,
          state.issues.blocked_nodes,
          false);

    DifferentialDriveExpander expander(context);
    auto& queue = static_cast<InternalState*>(state.internal.get())->queue;

    expander.make_initial_nodes(
          DifferentialDriveExpander::InitialNodeArgs{
            state.conditions.starts
          }, queue);

    return state;
  }

  rmf_utils::optional<Plan> plan(State& state) final
  {
    if (state.conditions.starts.empty())
      return rmf_utils::nullopt;

    auto context = make_context(
          state.conditions.goal,
          state.conditions.options,
          state.issues.blocked_nodes,
          false);

    DifferentialDriveExpander expander(context);
    auto& queue = static_cast<InternalState*>(state.internal.get())->queue;
    const bool* interrupt_flag = state.conditions.options.interrupt_flag();

    const NodePtr solution =
        search<DifferentialDriveExpander>(expander, queue, interrupt_flag);

    if (interrupt_flag && *interrupt_flag)
      state.issues.interrupted = true;

    if (!solution)
      return rmf_utils::nullopt;

    return make_plan(state.conditions.starts, solution, context.validator);
  }

  struct RolloutEntry
  {
    Time initial_time;
    NodePtr node;

    bool operator==(const RolloutEntry& r) const
    {
      return node == r.node;
    }

    rmf_traffic::Duration span() const
    {
      return *node->route_from_parent.trajectory.finish_time() - initial_time;
    }

    struct Compare
    {
      bool operator()(const RolloutEntry& a, const RolloutEntry& b) const
      {
        return b.span() < a.span();
      }
    };

    struct Hash
    {
      std::size_t operator()(const RolloutEntry& r) const
      {
        return std::hash<NodePtr>()(r.node);
      }
    };
  };

  std::vector<schedule::Itinerary> rollout(
      const Duration max_span,
      const Issues::BlockedNodes& nodes,
      const agv::Planner::Goal& goal,
      const agv::Planner::Options& options,
      rmf_utils::optional<std::size_t> max_rollouts) final
  {
    std::vector<RolloutEntry> rollout_queue;
    for (const auto& void_node : nodes)
    {
      bool skip = false;
      const auto original_node =
          std::static_pointer_cast<Node>(void_node.first);

      const auto original_t = void_node.second;

      // TODO(MXG): This filtering approach is not reliable as it could be.
      // Certain blockages will only be expanded half as far past the blockage
      // as the API implies. It would be better if each type of node expansion
      // could have a unique identifier so we could both avoid redundant
      // expansions while still expanding a blockage out as far as the API
      // says that it will.
//      const auto merge_span = max_span/2.0;

      auto ancestor = original_node->parent;
      while (ancestor)
      {
        if (nodes.count(ancestor) > 0)
        {
          // TODO(MXG): Consider if we should account for the time difference
          // between these conflicts so that we get a broader rollout.
//          const auto t = *ancestor->route_from_parent.trajectory.finish_time();
//          if (t - original_t < merge_span)
          {
            skip = true;
          }

          break;
        }

        ancestor = ancestor->parent;
      }

      if (skip)
        continue;

      rollout_queue.emplace_back(
            RolloutEntry{
              original_t,
              original_node
            });

      // TODO(MXG): Consider making this configurable, or making a more
      // meaningful decision on how to prune the initial rollout queue.
      if (rollout_queue.size() > 5)
        break;
    }

    std::unordered_map<NodePtr, ConstRoutePtr> route_map;
    std::vector<schedule::Itinerary> alternatives;

    Issues::BlockerMap temp_blocked_nodes;
    auto context = make_context(goal, options, temp_blocked_nodes, true);
    DifferentialDriveExpander expander(context);

    const bool* interrupt_flag = options.interrupt_flag();

    DifferentialDriveExpander::SearchQueue search_queue;
    DifferentialDriveExpander::SearchQueue finished_rollouts;

    while (!rollout_queue.empty() && !(interrupt_flag && *interrupt_flag))
    {
      const auto top = rollout_queue.back();
      rollout_queue.pop_back();

      const auto current_span =
          top.node->route_from_parent.trajectory.back().time()
          - top.initial_time;

      const bool stop_expanding =
             (max_span < current_span)
          || expander.is_finished(top.node)
          || expander.is_holding_point(top.node->waypoint);

      if (stop_expanding)
      {
        finished_rollouts.push(top.node);

        if (max_rollouts && *max_rollouts <= finished_rollouts.size())
          break;

        continue;
      }

      expander.expand(top.node, search_queue);
      while (!search_queue.empty())
      {
        rollout_queue.emplace_back(
          RolloutEntry{
            top.initial_time,
            search_queue.top()
          });

        search_queue.pop();
      }
    }

    while (!finished_rollouts.empty())
    {
      auto node = finished_rollouts.top();
      finished_rollouts.pop();

      schedule::Itinerary itinerary;
      auto routes = reconstruct_routes(reconstruct_nodes(node), max_span);
      for (auto& r : routes)
      {
        assert(r.trajectory().size() > 0);
        itinerary.emplace_back(std::make_shared<Route>(std::move(r)));
      }

      assert(!itinerary.empty());
      alternatives.emplace_back(std::move(itinerary));
    }

    return alternatives;
  }

  const agv::Planner::Configuration& get_configuration() const final
  {
    return _config;
  }

  class Debugger : public Cache::Debugger
  {
  public:
    const agv::Planner::Debug::Node::SearchQueue& queue() const final
    {
      return queue_;
    }

    const std::vector<agv::Planner::Debug::ConstNodePtr>&
    expanded_nodes() const final
    {
      return expanded_nodes_;
    }

    const std::vector<agv::Planner::Debug::ConstNodePtr>&
    terminal_nodes() const final
    {
      return terminal_nodes_;
    }

    DifferentialDriveExpander::NodePtr convert(
        agv::Planner::Debug::ConstNodePtr from)
    {
      auto output = _from_debug[from];
      assert(output);

      return output;
    }

    agv::Planner::Debug::ConstNodePtr convert(
        DifferentialDriveExpander::NodePtr from)
    {
      const auto it = _to_debug.find(from);
      if (it != _to_debug.end())
        return it->second;

      std::vector<DifferentialDriveExpander::NodePtr> queue;
      queue.push_back(from);
      while (!queue.empty())
      {
        const auto node = queue.back();

        const auto parent = node->parent;
        agv::Planner::Debug::ConstNodePtr debug_parent = nullptr;
        if (parent)
        {
          const auto parent_it = _to_debug.find(parent);
          if (parent_it == _to_debug.end())
          {
            queue.push_back(parent);
            continue;
          }

          debug_parent = parent_it->second;
        }

        auto new_debug_node = std::make_shared<agv::Planner::Debug::Node>(
              agv::Planner::Debug::Node{
                debug_parent,
                RouteData::make(node->route_from_parent),
                node->remaining_cost_estimate,
                node->current_cost,
                node->waypoint,
                node->orientation,
                node->event,
                node->start_set_index
              });

        _to_debug[node] = new_debug_node;
        _from_debug[new_debug_node] = node;
        queue.pop_back();
      }

      return _to_debug[from];
    }

    agv::Planner::Debug::Node::SearchQueue queue_;
    std::vector<agv::Planner::Debug::ConstNodePtr> expanded_nodes_;
    std::vector<agv::Planner::Debug::ConstNodePtr> terminal_nodes_;
    Issues::BlockerMap blocked_nodes_;

    std::vector<agv::Planner::Start> starts_;
    agv::Planner::Goal goal_;
    agv::Planner::Options options_;

    Debugger(
        std::vector<agv::Planner::Start> starts,
        agv::Planner::Goal goal,
        agv::Planner::Options options)
    : starts_(std::move(starts)),
      goal_(std::move(goal)),
      options_(std::move(options))
    {
      // Do nothing
    }

  private:
    std::unordered_map<
        agv::Planner::Debug::ConstNodePtr,
        DifferentialDriveExpander::NodePtr> _from_debug;

    std::unordered_map<
        DifferentialDriveExpander::NodePtr,
        agv::Planner::Debug::ConstNodePtr> _to_debug;
  };

  std::unique_ptr<Cache::Debugger> debug_begin(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) final
  {
    auto debugger = std::make_unique<Debugger>(
          starts,
          std::move(goal),
          std::move(options));

    auto context = make_context(
          debugger->goal_, debugger->options_, debugger->blocked_nodes_, false);

    DifferentialDriveExpander expander(context);

    DifferentialDriveExpander::SearchQueue queue;
    expander.make_initial_nodes(
          DifferentialDriveExpander::InitialNodeArgs{starts}, queue);

    while (!queue.empty())
    {
      debugger->queue_.push(debugger->convert(queue.top()));
      queue.pop();
    }

    return debugger;
  }

  rmf_utils::optional<Plan> debug_step(Cache::Debugger& input_debugger) final
  {
    Debugger& debugger = static_cast<Debugger&>(input_debugger);

    auto top = debugger.convert(debugger.queue_.top());
    debugger.queue_.pop();
    debugger.expanded_nodes_.push_back(debugger.convert(top));

    auto context = make_context(
          debugger.goal_, debugger.options_, debugger.blocked_nodes_, false);

    DifferentialDriveExpander expander(context);
    if (expander.is_finished(top))
      return make_plan(debugger.starts_, top, context.validator);

    DifferentialDriveExpander::SearchQueue queue;
    expander.expand(top, queue);
    if (queue.empty())
      debugger.terminal_nodes_.push_back(debugger.convert(top));
    else
    {
      while (!queue.empty())
      {
        debugger.queue_.push(debugger.convert(queue.top()));
        queue.pop();
      }
    }

    return rmf_utils::nullopt;
  }

private:

  Plan make_plan(
      const std::vector<agv::Planner::Start>& starts,
      const NodePtr& solution,
      const agv::RouteValidator* validator) const
  {
    auto nodes = reconstruct_nodes(solution, validator);
    auto routes = reconstruct_routes(nodes);
    auto waypoints = reconstruct_waypoints(nodes, _graph);
    auto start_index = find_start_index(solution);

    return Plan{
      std::move(routes),
      std::move(waypoints),
      starts[start_index],
      solution->current_cost
    };
  }

  DifferentialDriveExpander::Context make_context(
      const agv::Planner::Goal& goal,
      const agv::Planner::Options& options,
      Issues::BlockerMap& blocked_nodes,
      const bool simple_lane_expansion)
  {
    const std::size_t goal_waypoint = goal.waypoint();
    const auto goal_orientation = rmf_utils::pointer_to_opt(goal.orientation());

    Heuristic& h = _heuristics.insert(
          std::make_pair(goal_waypoint, Heuristic{})).first->second;
    const bool* const interrupt_flag = options.interrupt_flag();

    return DifferentialDriveExpander::Context{
      _graph,
      _traits,
      _profile,
      options.minimum_holding_time(),
      _interpolate,
      options.validator().get(),
      goal_waypoint,
      goal_orientation,
      options.maximum_cost_estimate(),
      interrupt_flag,
      h,
      blocked_nodes,
      simple_lane_expansion
    };
  }

  agv::Planner::Configuration _config;

  const agv::Graph::Implementation& _graph;
  const agv::VehicleTraits& _traits;
  const Profile& _profile;
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
  if (config.vehicle_traits().get_differential())
  {
    return CacheManager(std::make_shared<DifferentialDriveCache>(
          std::move(config)));
  }

  // *INDENT-OFF*
  throw std::runtime_error(
    "[rmf_traffic::agv::Planner] Planning utilities are currently only "
    "implemented for AGVs that use a differential drive.");
  // *INDENT-ON*
}

} // namespace planning
} // namespace internal
} // namespace rmf_traffic
