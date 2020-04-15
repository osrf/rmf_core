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

#ifndef SRC__RMF_TRAFFIC__DETAIL__INTERNAL_BIDIRECTIONAL_ITERATOR_HPP
#define SRC__RMF_TRAFFIC__DETAIL__INTERNAL_BIDIRECTIONAL_ITERATOR_HPP

#include <rmf_traffic/detail/bidirectional_iterator.hpp>

namespace rmf_traffic {
namespace detail {

//==============================================================================
template<typename E, typename I, typename F>
E& bidirectional_iterator<E, I, F>::operator*() const
{
  return *_pimpl->iter;
}

//==============================================================================
template<typename E, typename I, typename F>
E* bidirectional_iterator<E, I, F>::operator->() const
{
  return &(*_pimpl->iter);
}

//==============================================================================
template<typename E, typename I, typename F>
auto bidirectional_iterator<E, I, F>::operator++() -> bidirectional_iterator&
{
  ++_pimpl->iter;
  return *this;
}

//==============================================================================
template<typename E, typename I, typename F>
auto bidirectional_iterator<E, I, F>::operator--() -> bidirectional_iterator&
{
  --_pimpl->iter;
  return *this;
}

//==============================================================================
template<typename E, typename I, typename F>
auto bidirectional_iterator<E, I, F>::operator++(int) -> bidirectional_iterator
{
  bidirectional_iterator original{*this};
  ++_pimpl->iter;
  return original;
}

//==============================================================================
template<typename E, typename I, typename F>
auto bidirectional_iterator<E, I, F>::operator--(int) -> bidirectional_iterator
{
  bidirectional_iterator original{*this};
  --_pimpl->iter;
  return original;
}

//==============================================================================
template<typename E, typename I, typename F>
bool bidirectional_iterator<E, I, F>::operator==(
  const bidirectional_iterator& other) const
{
  return _pimpl->iter == other._pimpl->iter;
}

//==============================================================================
template<typename E, typename I, typename F>
bool bidirectional_iterator<E, I, F>::operator!=(
  const bidirectional_iterator& other) const
{
  return _pimpl->iter != other._pimpl->iter;
}

//==============================================================================
template<typename E, typename I, typename F>
bidirectional_iterator<E, I, F>::bidirectional_iterator()
{
  // Do nothing
}

//==============================================================================
template<typename E, typename I, typename F>
bidirectional_iterator<E, I, F>::bidirectional_iterator(I impl)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{std::move(impl)}))
{
  // Do nothing
}

//==============================================================================
template<typename E, typename I, typename F>
bidirectional_iterator<E, I, F>
::operator bidirectional_iterator<const E, I, F>() const
{
  bidirectional_iterator<const E, I, F> result;
  result._pimpl = rmf_utils::make_impl<I>(I{_pimpl->iter});
  return result;
}

} // namespace detail
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__DETAIL__INTERNAL_BIDIRECTIONAL_ITERATOR_HPP
