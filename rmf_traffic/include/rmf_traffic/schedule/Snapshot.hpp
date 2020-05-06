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

#ifndef RMF_TRAFFIC__SCHEDULE__SNAPSHOT_HPP
#define RMF_TRAFFIC__SCHEDULE__SNAPSHOT_HPP

#include <rmf_traffic/schedule/Viewer.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Snapshot : public Viewer
{
public:

  // The Snapshot class is just a more specific name for Viewer.

};

//==============================================================================
/// This is a pure abstract interface class that can be inherited by any
/// schedule Viewer that wants to be able to provide a frozen snapshot of its
/// schedule.
class Snappable
{
public:

  /// Get a snapshot of the schedule
  virtual std::shared_ptr<const Snapshot> snapshot() const = 0;

  // Virtual destructor
  virtual ~Snappable() = default;

};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__SNAPSHOT_HPP
