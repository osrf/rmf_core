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

#ifndef RMF_TRAFFIC__DETAIL__BIDIRECTIONAL_ITERATOR_HPP
#define RMF_TRAFFIC__DETAIL__BIDIRECTIONAL_ITERATOR_HPP

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
/// bidirectional iterator.
template<typename ElementType, typename ImplementationType, typename Friend>
class bidirectional_iterator
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
  bidirectional_iterator& operator++();

  /// Pre-decrement operator: --it
  ///
  /// \note This is more efficient than the post-decrement operator
  ///
  /// \return a reference to the iterator that was operated on
  bidirectional_iterator& operator--();

  /// Post-increment operator: it++
  ///
  /// \return a copy of the iterator before it was incremented
  bidirectional_iterator operator++(int);

  /// Post-decrement operator: it--
  ///
  /// \return a copy of the iterator before it was decremented
  bidirectional_iterator operator--(int);

  /// Equality comparison operator
  bool operator==(const bidirectional_iterator& other) const;

  /// Inequality comparison operator
  bool operator!=(const bidirectional_iterator& other) const;

  // Allow implicit conversion to const_iterator type
  operator bidirectional_iterator<const Element, Implementation,
    Friend>() const;

  // Allow typical copying and moving
  bidirectional_iterator(const bidirectional_iterator&) = default;
  bidirectional_iterator(bidirectional_iterator&&) = default;
  bidirectional_iterator& operator=(const bidirectional_iterator&) = default;
  bidirectional_iterator& operator=(bidirectional_iterator&&) = default;

  // Default constructor. This will leave the iterator uninitialized, so it is
  // UNDEFINED BEHAVIOR (most likely a segfault) to use it without using one of
  // the container's functions (like insert, find, begin, end, etc) to
  // initialize it first.
  bidirectional_iterator();

private:
  bidirectional_iterator(ImplementationType impl);
  friend Friend;
  template<typename, typename, typename> friend class bidirectional_iterator;
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace detail
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__DETAIL__BIDIRECTIONAL_ITERATOR_HPP
