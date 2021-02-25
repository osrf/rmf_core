/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <vector>

#ifndef SRC__RMF_TASK__INVARIANTHEURISTICQUEUE_HPP
#define SRC__RMF_TASK__INVARIANTHEURISTICQUEUE_HPP

namespace rmf_task {

// Sorts and distributes tasks among agents based on the earliest finish time
// possible for each task (i.e. not accounting for any variant costs). Guaranteed
// to underestimate actual cost when the earliest start times for each task are
// similar (enforced by the segmentation_threshold).
class InvariantHeuristicQueue
{
public:

  InvariantHeuristicQueue(std::vector<double> initial_values);

  void add(const double earliest_start_time, const double earliest_finish_time);

  double compute_cost() const;

private:
  struct element { double start; double end; };
  std::vector<std::vector<element>> _stacks;
};

} // namespace rmf_task

#endif // SRC__RMF_TASK__INVARIANTHEURISTICQUEUE_HPP
