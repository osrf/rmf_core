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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__DEBUG_DATABASE_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__DEBUG_DATABASE_HPP

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

class Database::Debug
{
public:

  /// Returns how many entry histories are still in the database. An entry
  /// history is the chain of changes tied to the route ID of a single
  /// participant. The full history of an entry is preserved until the database
  /// is given a cull command, and the entry qualifies for the cull.
  static std::size_t current_entry_history_count(const Database& database);

  /// Returns how many participants remain in the record of participants that
  /// have been removed (aka unregistered). These entries should gradually
  /// disappear when the database is culled from a time that comes after the
  /// unregistering time.
  static std::size_t current_removed_participant_count(const Database& db);

};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__DEBUG_DATABASE_HPP
