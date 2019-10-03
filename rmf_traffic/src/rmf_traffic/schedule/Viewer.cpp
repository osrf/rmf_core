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

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <map>

namespace rmf_traffic {
namespace schedule {

namespace {

struct Entry
{
  std::unique_ptr<Trajectory> trajectory;
  std::size_t version;
};

using EntryPtr = std::shared_ptr<Entry>;

} // anonymous namespace

//==============================================================================
class Viewer::Implementation
{
public:

  using Bucket = std::vector<EntryPtr>;
  using Timeline = std::map<Time, Bucket>;
  using MapToTimeline = std::unordered_map<std::string, Timeline>;

  MapToTimeline maps;

  using EntryMap = std::map<std::size_t, EntryPtr>;

  struct ChangeData
  {
    Database::Change::Mode mode;

    // Used in all modes
    std::size_t id;

    // Used in Insert and Replace mode
    EntryPtr entry;

    // Used in Replace and Erase mode
    std::size_t original_id;

    // Used in Cull mode
    std::vector<std::size_t> culled_ids;
  };

  using ChangeMap = std::map<std::size_t, ChangeData>;

};

//==============================================================================


} // namespace schedule
} // namespace rmf_traffic
