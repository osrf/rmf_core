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

#include "BinaryPriority.hpp"

namespace rmf_task {

//==============================================================================
BinaryPriority::BinaryPriority(std::size_t value)
: _value(value)
{
  // Do nothing
}

//==============================================================================
std::size_t BinaryPriority::value() const
{
  return _value;
}

}