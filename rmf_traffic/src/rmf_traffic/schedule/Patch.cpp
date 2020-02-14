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

#include <rmf_traffic/schedule/Patch.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Patch::Implementation
{
public:

  std::vector<Change> changes;

  Version latest_version;

  Implementation()
  {
    // Do nothing
  }

  Implementation(std::vector<Change> _changes, Version _latest_version)
    : changes(std::move(_changes)),
      latest_version(_latest_version)
  {
    // Do nothing
  }

};

//==============================================================================
class Patch::IterImpl
{
public:

  std::vector<Change>::const_iterator iter;

};

//==============================================================================
Patch::Patch(std::vector<Participant> changes, Version latest_version)
  : _pimpl(rmf_utils::make_impl<Implementation>(
             std::move(changes), latest_version))
{
  // Do nothing
}

//==============================================================================
auto Patch::begin() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.begin()});
}

//==============================================================================
auto Patch::end() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.end()});
}

//==============================================================================
std::size_t Patch::size() const
{
  return _pimpl->changes.size();
}

//==============================================================================
Version Patch::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
Patch::Patch()
{
  // Do nothing
}

} // namespace schedule


namespace detail {

template class bidirectional_iterator<
    const schedule::Change,
    schedule::Patch::IterImpl,
    schedule::Patch
>;

} // namespace detail


} // namespace rmf_traffic
