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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__CHANGEINTERNAL_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__CHANGEINTERNAL_HPP

#include <rmf_traffic/schedule/Change.hpp>

#include <rmf_utils/optional.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Change::Delay::Implementation
{
public:

  Time from;
  Duration duration;

};

//==============================================================================
rmf_utils::optional<Trajectory> apply_delay(
  const Trajectory& old_trajectory,
  Time from,
  Duration delay);

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__CHANGEINTERNAL_HPP
