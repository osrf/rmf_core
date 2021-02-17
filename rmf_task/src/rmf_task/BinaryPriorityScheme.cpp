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

#include <rmf_task/BinaryPriorityScheme.hpp>

#include "Priority.hpp"
#include "CostCalculator.hpp"

namespace rmf_task {

//==============================================================================
std::shared_ptr<Priority> BinaryPriorityScheme::make_low_priority()
{
  return nullptr;
}

//==============================================================================
std::shared_ptr<Priority> BinaryPriorityScheme::make_high_priority()
{
 return std::make_shared<BinaryPriority>(1);
}

//==============================================================================
std::shared_ptr<CostCalculator> BinaryPriorityScheme::make_cost_calculator()
{
  return std::make_shared<BinaryPriorityCostCalculator>();
}

}