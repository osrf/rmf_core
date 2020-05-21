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

#include <rmf_traffic/Time.hpp>

namespace rmf_traffic {
namespace time {

//==============================================================================
double to_seconds(const Duration delta_t)
{
  using Sec64 = std::chrono::duration<double>;
  return std::chrono::duration_cast<Sec64>(delta_t).count();
}

//==============================================================================
Duration from_seconds(double delta_t)
{
  using Sec64 = std::chrono::duration<double>;
  return std::chrono::duration_cast<Duration>(Sec64(delta_t));
}

//==============================================================================
Time apply_offset(const Time start_time, const double delta_seconds)
{
  using Sec64 = std::chrono::duration<double>;
  using NanoInt = std::chrono::steady_clock::duration;
  return start_time + std::chrono::duration_cast<NanoInt>(Sec64(delta_seconds));
}

} // namespace time
} // namespace rmf_traffic
