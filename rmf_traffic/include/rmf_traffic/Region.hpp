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

#ifndef RMF_TRAFFIC__REGION_HPP
#define RMF_TRAFFIC__REGION_HPP

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

#include <rmf_traffic/geometry/Space.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <vector>

namespace rmf_traffic {

//==============================================================================
/// A class to describe a region within spacetime.
///
/// This specifies the map whose coordinates should be used, a lower and
/// upper bound to define a time range, and a set of geometry::Space objects
/// to define regions with space.
///
/// For the geometry::Space objects, this class acts like an STL container
/// and provides an iterator interface to specify, access, and modify them.
class Region
{
public:

  using Space = geometry::Space;
  template<typename E, typename I, typename F>
  using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;

  /// Construct a region given the parameters.
  ///
  /// \param[in] map
  ///   The map whose coordinates will be used to define the regions in
  ///   space.
  ///
  /// \param[in] lower_bound
  ///   The lower bound for the time range.
  ///
  /// \param[in] upper_bound
  ///   The upper bound for the time range.
  ///
  /// \param[in] spaces
  ///   A vector of geometry::Space objects to define the desired regions
  ///   in space.
  Region(
      std::string map,
      Time lower_bound,
      Time upper_bound,
      std::vector<Space> spaces);

  /// Construct a region with no time constraints.
  ///
  /// \param[in] map
  ///   The map whose coordinates will be used to define the regions in
  ///   space.
  ///
  /// \param[in] spaces
  ///   A vector of geometry::Space objects to define the desired regions
  ///   in space.
  Region(
      std::string map,
      std::vector<Space> spaces);

  /// Get the name of the map that this Spacetime refers to.
  const std::string& get_map() const;

  /// Set the name of the map that this Spacetime refers to.
  Region& set_map(std::string map);

  /// Get the lower bound for the time range.
  ///
  /// If there is no lower bound for the time range, then this returns
  /// a nullptr.
  const Time* get_lower_time_bound() const;

  /// Set the lower bound for the time range.
  Region& set_lower_time_bound(Time time);

  /// Remove the lower bound for the time range.
  Region& remove_lower_time_bound();

  /// Get the upper bound for the time range.
  ///
  /// If there is no upper bound for the time range, then this returns
  /// a nullptr.
  const Time* get_upper_time_bound() const;

  /// Set the upper bound for the time range.
  Region& set_upper_time_bound(Time time);

  /// Remove the upper bound for the time range.
  Region& remove_upper_time_bound();

  class IterImpl;
  using iterator = base_iterator<Space, IterImpl, Region>;
  using const_iterator = base_iterator<const Space, IterImpl, Region>;

  /// Add a region of space.
  void push_back(Space space);

  /// Remove the last region of space that was added.
  void pop_back();

  /// Erase a specific region of space based on its iterator.
  iterator erase(iterator it);

  /// Erase a specific sets of regions of space based on their iterators.
  iterator erase(iterator first, iterator last);

  /// Get the beginning iterator for the regions of space.
  iterator begin();

  /// const-qualified begin()
  const_iterator begin() const;

  /// Explicitly const-qualified alternative for begin()
  const_iterator cbegin() const;

  /// Get the one-past-the-end iterator for the regions of space.
  iterator end();

  /// const-qualified end()
  const_iterator end() const;

  /// Explicitly const-qualified alternative for end()
  const_iterator cend() const;

  /// Get the number of Space regions in this Spacetime region.
  std::size_t num_spaces() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

namespace detail {

//==============================================================================
extern template class bidirectional_iterator<
    geometry::Space, Region::IterImpl, Region
>;

//==============================================================================
extern template class bidirectional_iterator<
    const geometry::Space, Region::IterImpl, Region
>;

} // namespace detail

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__REGION_HPP
