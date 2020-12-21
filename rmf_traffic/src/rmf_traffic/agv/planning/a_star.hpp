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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__A_STAR_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__A_STAR_HPP

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<
    class Expander,
    class SearchQueue = typename Expander::SearchQueue,
    class NodePtr = typename Expander::NodePtr>
NodePtr a_star_search(
    Expander& expander,
    SearchQueue& queue)
{
  while (!queue.empty())
  {
    NodePtr top = queue.top();

    if (expander.quit(top, queue))
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
template<typename NodePtrT>
struct SimpleCompare
{
  bool operator()(const NodePtrT& a, const NodePtrT& b)
  {
    // Note(MXG): The priority queue puts the greater value first, so we
    // reverse the arguments in this comparison.
    // TODO(MXG): Micro-optimization: consider saving the sum of these values
    // in the Node instead of needing to re-add them for every comparison.
    return b->remaining_cost_estimate + b->current_cost
        < a->remaining_cost_estimate + a->current_cost;
  }
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__A_STAR_HPP
