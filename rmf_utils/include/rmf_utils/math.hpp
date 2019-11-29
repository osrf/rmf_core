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

#ifndef RMF_UTILS__MATH_HPP
#define RMF_UTILS__MATH_HPP

#include <cmath>

namespace rmf_utils {

//==============================================================================
inline double wrap_to_pi(double value)
{
  while(value < -M_PI)
    value += 2.0*M_PI;

  while(M_PI < value)
    value -= 2.0*M_PI;

  return value;
}

} // namespace rmf_utils

#endif // RMF_UTILS__MATH_HPP
