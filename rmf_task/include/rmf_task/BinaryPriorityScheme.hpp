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

#ifndef RMF_TASK__BINARYPRIORITYSCHEME_HPP
#define RMF_TASK__BINARYPRIORITYSCHEME_HPP

#include <memory>

namespace rmf_task {

// Forward declare abstract interfaces. The definition will remain internal
class Priority;
class CostCalculator;
using PriorityPtr = std::shared_ptr<Priority>;
using ConstPriorityPtr = std::shared_ptr<const Priority>;

/// This is for a binary prioritization scheme. Tasks are assigned either high priority or low priority.
class BinaryPriorityScheme
{
public:

  /// Use these to assign the task priority
  static std::shared_ptr<Priority> make_low_priority();
  static std::shared_ptr<Priority> make_high_priority();

  /// Use this to give the appropriate cost calculator to the task planner
  static std::shared_ptr<CostCalculator> make_cost_calculator();
};

} // namespace rmf_task

#endif // RMF_TASK__BINARYPRIORITYSCHEME_HPP
