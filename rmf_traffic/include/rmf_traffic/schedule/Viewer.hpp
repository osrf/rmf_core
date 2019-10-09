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

#ifndef RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
#define RMF_TRAFFIC__SCHEDULE__VIEWER_HPP

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_traffic/schedule/Query.hpp>

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/macros.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A class that allows users to see Trajectories that are part of a schedule.
///
/// This class cannot be instantiated directly. To get a Viewer, you must
/// instantiate an rmf_traffic::schedule::Database or an
/// rmf_traffic::schedule::Mirror object.
class Viewer
{
public:

  /// A read-only view of some Trajectories in a Database or Mirror.
  ///
  /// It is undefined behavior to modify a Database or patch a Mirror while
  /// reading Trajectories from this view. The user of this class is responsible
  /// for managing access to reads vs access to writes.
  class View
  {
  public:

    template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

    class IterImpl;
    using const_iterator = base_iterator<const Trajectory, IterImpl, View>;
    using iterator = const_iterator;

    /// Returns an iterator to the first element of the View
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// View.
    const_iterator end() const;

    /// Returns the number of elements in this View.
    std::size_t size() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Query this Viewer to get a View of the Trajectories inside of it that
  /// match the Query parameters.
  View query(const Query& parameters) const;


  class Implementation;
protected:
  // These constructors and operators are protected so that users can only
  // constructor or assign using classes that are derived from Viewer.
  Viewer();
  RMF_UTILS__DEFAULT_COPY_MOVE(Viewer);
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule

namespace detail {

extern template class bidirectional_iterator<
    const Trajectory,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__VIEWER_HPP
