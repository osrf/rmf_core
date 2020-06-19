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

#ifndef RMF_TRAFFIC__TIME_HPP
#define RMF_TRAFFIC__TIME_HPP

#include <chrono>

namespace rmf_traffic {

/// Specifies a specific point in time, with nanosecond precision.
///
/// Conventionally this will be represented relative to the Unix Epoch.
//
// NOTE(MXG): An int64 nanosecond representation of time since the Unix Epoch
// will not overflow until the year 2262. Please be sure to change this
// implementation before then.
using Time = std::chrono::steady_clock::time_point;
// TODO(MXG): We should seriously consider a complete overhaul of how we are
// using time in this library.


/// Specifies a change in time, with nanosecond precision.
using Duration = std::chrono::steady_clock::duration;

namespace time {

/// Change the given duration from a nanosecond count to a double-precision
/// floating-point representation in seconds.
double to_seconds(Duration delta_t);

/// Chance the given duration from a double-precision floating-point
/// representation to a nanosecond count.
Duration from_seconds(double delta_t);

/// Return the given start_time, offset by the number of seconds given.
///
/// \param[in] start_time
///   The time to start from
///
/// \param[in] delta_seconds
///   The number of seconds to add to the start_time
Time apply_offset(Time start_time, double delta_seconds);

} // namespace time


// TODO(MXG): Make user-friendly interfaces for interacting with the STL
// time points and durations. They have good semantics from a technical point
// of view, but they tend to be difficult for users to do tests and experiments
// with, so it might be good to provide some APIs to help people use it.

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__TIME_HPP
