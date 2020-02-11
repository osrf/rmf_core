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

#ifndef RMF_TRAFFIC__DETAIL__FORWARD_ITERATOR_HPP
#define RMF_TRAFFIC__DETAIL__FORWARD_ITERATOR_HPP

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace detail {

//==============================================================================
/// This class is used so we can provide iterators for various container classes
/// without exposing any implementation details about what kind of STL container
/// we are using inside of our container class. This allows us to guarantee ABI
/// stability, even if we decide to change what STL container we use inside of
/// our implementation.
///
/// This class is designed to offer only the most basic features of a
/// forward iterator.
template<typename ElementType, typename ImplementationType, typename Friend>
class forward_iterator
{
public:

  using Element = ElementType;
  using Implementation = ImplementationType;

  /// Dereference operator
  Element& operator*() const;

  /// Drill-down operator
  Element* operator->() const;

  /// Pre-increment operator: ++it
  ///
  /// \note This is more efficient than the post-increment operator.
  ///
  /// \return a reference to the iterator that was operated on
  forward_iterator& operator++();

  /// Post-increment operator: it++
  ///
  /// \return a copy of the iterator before it was incremented
  forward_iterator operator++(int);

  /// Equality comparison operator
  bool operator==(const forward_iterator& other) const;

  /// Inequality comparison operator
  bool operator!=(const forward_iterator& other) const;

  // Allow implicit conversion to const_iterator type
  operator forward_iterator<const Element, Implementation, Friend>() const;

  // Allow typical copying and moving
  forward_iterator(const forward_iterator&) = default;
  forward_iterator(forward_iterator&&) = default;
  forward_iterator& operator=(const forward_iterator&) = default;
  forward_iterator& operator=(forward_iterator&&) = default;

  // Default constructor. This will leave the iterator uninitialized, so it is
  // UNDEFINED BEHAVIOR (most likely a segfault) to use it without using one of
  // the container's functions (like insert, find, begin, end, etc) to
  // initialize it first.
  forward_iterator();

private:
  forward_iterator(ImplementationType impl);
  friend Friend;
  template<typename, typename, typename> friend class forward_iterator;
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__DETAIL__FORWARD_ITERATOR_HPP
