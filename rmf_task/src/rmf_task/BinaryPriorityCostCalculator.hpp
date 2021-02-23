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

#ifndef SRC__RMF_TASK__BINARYPRIORITYCOSTCALCULATOR_HPP
#define SRC__RMF_TASK__BINARYPRIORITYCOSTCALCULATOR_HPP

#include "CostCalculator.hpp"

namespace rmf_task {

// Sample implementation for binary prioritization scheme
class BinaryPriorityCostCalculator : public CostCalculator
{
public:

  /// Constructor
  BinaryPriorityCostCalculator(
    double priority_penalty = 10000);

  /// Documentation inherited
  double compute_cost(
    const Node& n,
    rmf_traffic::Time time_now,
    bool check_priority) const final;

  /// Compute the cost of assignments
  double compute_cost(
    rmf_task::agv::TaskPlanner::Assignments assignments) const final;

private:
  using TaskPlanner = rmf_task::agv::TaskPlanner;
  using Assignments = TaskPlanner::Assignments;

  double _priority_penalty;

  double compute_g_assignment(const TaskPlanner::Assignment& assignment) const;

  double compute_g(const Assignments& assigned_tasks) const;

  double compute_g(const Node& node) const;

  double compute_h(const Node& node, const rmf_traffic::Time time_now) const;

  bool valid_assignment_priority(const Node& node) const;

};

} // namespace rmf_task

#endif // SRC__RMF_TASK__BINARYPRIORITYCOSTCALCULATOR_HPP
