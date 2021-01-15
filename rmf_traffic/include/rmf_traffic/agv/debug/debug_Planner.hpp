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

#ifndef RMF_TRAFFIC__AGV__DEBUG__DEBUG_PLANNER_HPP
#define RMF_TRAFFIC__AGV__DEBUG__DEBUG_PLANNER_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <queue>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// This class exists only for debugging purposes. It is not to be used in live
/// production, and its API is to be considered unstable at all times. Any minor
/// version increment
class Planner::Debug
{
public:

  struct Node;
  using ConstNodePtr = std::shared_ptr<const Node>;

  /// A Node in the planning search. A final Planning solution will be a chain
  /// of these Nodes, aggregated into a Plan data structure.
  struct Node
  {
    /// The parent of this Node. If this is a nullptr, then this was a starting
    /// node.
    ConstNodePtr parent;

    /// The route that goes from the parent Node to this Node.
    std::vector<Route> route_from_parent;

    /// An estimate of the remaining cost, based on the heuristic.
    double remaining_cost_estimate;

    /// The actual cost that has accumulated on the way to this Node.
    double current_cost;

    /// The waypoint that this Node stops on.
    rmf_utils::optional<std::size_t> waypoint;

    /// The orientation that this Node ends with.
    double orientation;

    /// A pointer to an event that occured on the way to this Node.
    agv::Graph::Lane::EventPtr event;

    /// If this is a starting node, then this will be the index.
    rmf_utils::optional<std::size_t> start_set_index;

    struct Compare
    {
      bool operator()(const ConstNodePtr& a, const ConstNodePtr& b)
      {
        return b->current_cost + b->remaining_cost_estimate
          < a->current_cost + a->remaining_cost_estimate;
      }
    };

    using SearchQueue =
      std::priority_queue<ConstNodePtr, std::vector<ConstNodePtr>, Compare>;

    using Vector = std::vector<ConstNodePtr>;
  };

  class Progress
  {
  public:

    /// Step the planner forward one time. This will expand the current highest
    /// priority Node in the queue and move it to the back of expanded_nodes.
    /// The nodes that result from the expansion will all be added to the queue.
    rmf_utils::optional<Plan> step();

    /// Implicitly cast the Progress instance to a boolean. The value will be
    /// true if the plan can keep expanding, and it will be false if it cannot
    /// expand any further.
    ///
    /// After finding a solution, it may be possible to continue expanding, but
    /// there is no point because the first solution returned is guaranteed to
    /// be the optimal one.
    operator bool() const
    {
      return !queue().empty();
    }

    /// A priority queue of unexpanded Nodes. They are sorted based on g(n)+h(n)
    /// in ascending order (see Node::Compare).
    const Node::SearchQueue& queue() const;

    /// The set of Nodes that have been expanded. They are sorted in the order
    /// that they were chosen for expansion.
    const Node::Vector& expanded_nodes() const;

    /// The set of Nodes which terminated, meaning it was not possible to expand
    /// from them.
    const Node::Vector& terminal_nodes() const;

    class Implementation;
  private:
    Progress();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  /// Create a debugger for a planner.
  Debug(const Planner& planner);

  /// Begin debugging a plan. Call step() on the Progress object until it
  /// returns a plan or until the queue is empty (the Progress object can be
  /// treated as a boolean for this purpose).
  Progress begin(
    const std::vector<Start>& starts,
    Goal goal,
    Options options) const;

  /// Get the current size of the frontier queue of a Planner Result
  static std::size_t queue_size(const Planner::Result& result);

  /// Get the number of search nodes that have been expanded for a Planner
  /// Result
  static std::size_t expansion_count(const Planner::Result& result);

  /// Get the current number of nodes that have been created for this Planner
  /// Result. This is equal to queue_size(r) + expansion_count(r).
  static std::size_t node_count(const Planner::Result& result);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__DEBUG__DEBUG_PLANNER_HPP
