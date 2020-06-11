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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCIESINTERNAL_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCIESINTERNAL_HPP

#include <rmf_traffic/schedule/Inconsistencies.hpp>

#include "InconsistencyTracker.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
using ApiMap = std::unordered_map<ParticipantId, Inconsistencies::Element>;

//==============================================================================
class Inconsistencies::Implementation
{
public:

  static std::unique_ptr<InconsistencyTracker> register_participant(
    Inconsistencies& inconsistencies,
    ParticipantId id);

  void unregister_participant(ParticipantId id);

private:
  friend Inconsistencies;
  friend InconsistencyTracker;

  /// The map of all inconsistencies
  InconsistenciesMap _inconsistencies;

  /// The memory footprint that contains the objects which the API will give to
  /// library users by reference. We need to store these objects internally so
  /// that the API doesn't need to do excessive copying when giving information
  /// to users.
  ApiMap _api;

  using raw_iterator = ApiMap::const_iterator;
  static const_iterator make_iterator(raw_iterator it);

};


} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INCONSISTENCIESINTERNAL_HPP
