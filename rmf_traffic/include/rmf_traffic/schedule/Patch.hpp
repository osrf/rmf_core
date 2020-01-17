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

#ifndef RMF_TRAFFIC__SCHEDULE__PATCH_HPP
#define RMF_TRAFFIC__SCHEDULE__PATCH_HPP

#include <rmf_traffic/schedule/Change.hpp>

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A container of Database changes
class Patch
{
public:

  template<typename E, typename I, typename F>
  using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  class IterImpl;
  using const_iterator = base_iterator<const Change, IterImpl, Patch>;

  Patch(std::vector<Change> changes, Version latest_version);

  /// Returns an iterator to the first element of the Patch.
  const_iterator begin() const;

  /// Returns an iterator to the element following the last element of the
  /// Patch. This iterator acts as a placeholder; attempting to dereference it
  /// results in undefined behavior.
  const_iterator end() const;

  /// Get the number of elements in this Patch.
  std::size_t size() const;

  /// Get the latest version of the Database that informed this Patch.
  Version latest_version() const;

  class Implementation;
private:
  Patch();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};


} // namespace schedule
} // namespace rmf_traffic

namespace detail {

extern template class bidirectional_iterator<
    const schedule::Database::Change,
    schedule::Database::Patch::IterImpl,
    schedule::Database::Patch
>;

}

#endif // RMF_TRAFFIC__SCHEDULE__PATCH_HPP
