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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__MODULAR_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__MODULAR_HPP

#include <stdexcept>
#include <limits>
#include <type_traits>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// This class allows us to correctly handle version number overflow. Since the
/// schedule needs to continue running for an arbitrarily long time, we cannot
/// expect its versions numbers to get reset before it reaches the limit of its
/// integer representation. This class allows us to compare version numbers that
/// could have overflowed at some point.
template<typename V>
class Modular
{
public:

  Modular(V basis)
  : _basis(basis)
  {
    // Do nothing
  }

  /// Compare rhs (right-hand side of the less-than comparison) directly to the
  /// basis value. This comparison is only permitted if the value of rhs is
  /// within half of the maximum modular distance of the basis. If rhs is
  /// further than that distance from the basis (either from above or below)
  /// then an exception will be thrown.
  bool less_than(const V rhs) const
  {
    // "distance" here means how far the RHS is from the basis, adjusted back by
    // half of the maximum window size.
    const V distance = rhs - _basis + HalfWindow;

    // If the distance from the adjusted basis is greater than the maximum
    // window size, then we have an exception.
    if (Window < distance)
    {
      throw std::runtime_error(
            "[rmf_traffic::schedule::Modular] modular distance between value ["
            + std::to_string(rhs) + "] and basis [" + std::to_string(_basis)
            + "] is too big [" + std::to_string(distance) + "]. Maximum is "
            + std::to_string(Window));
    }

    // If the distance from the adjusted basis is less than half the window
    // size, then RHS is below (i.e. less than) the original basis. Otherwise it
    // is not less than the original basis.
    return distance < HalfWindow;
  }

  /// Modify less(V) to also return true when equal.
  bool less_than_or_equal(const V rhs) const
  {
    return (_basis == rhs) || less_than(rhs);
  }

  /// Compare lhs (left-hand side) to rhs (right-hand side), and return true if
  /// and only if lhs is less than rhs.
  ///
  /// This assumes that both lhs and rhs are greater than the basis, i.e.
  /// neither of these values has spilled past (overlapped) the basis value.
  /// This assumption allows us to do this comparison without any risk of an
  /// excpetion.
  bool less_than(const V lhs, const V rhs) const
  {
    return (lhs - _basis) < (rhs - _basis);
  }

  /// Modify less(V, V) to also return true when lhs and rhs are equal.
  bool less_than_or_equal(const V lhs, const V rhs) const
  {
    return (lhs == rhs) || less_than(lhs, rhs);
  }

private:

  /// This constant represents the maximum modular distance that we will accept
  /// between modular values when comparing a value directly against the basis.
  /// Templates were used here so that this value will automatically be set to
  /// an appropriate value based on the maximum size of V. Converting V to a
  /// signed integer type cuts its maximum size in half, which is the desired
  /// window size that we want to allow.
  ///
  /// Considering the size of this window, it is not feasible that enough
  /// inbound messages would get dropped (~9 quintillion when using 64-bits) for
  /// us to witness this large of a gap between updates.
  ///
  constexpr static V Window =
      static_cast<V>(std::numeric_limits<std::make_signed_t<V>>::max());

  constexpr static V HalfWindow = Window/2;

  V _basis = 0;

};

/// This function gives a convenient way to instantiate a Modular<V> object.
/// The rules of C++ template instantiation do not allow a class constructor to
/// infer the template type based on argument values, but function templates are
/// able to make such an inference. This allows someone to call modular(v)
/// instead of Modular<MyIntegerValue>(v) to get an object of type
/// Modular<MyIntegerValue>.
template<typename V>
Modular<V> modular(V value)
{
  return Modular<V>(value);
}

/// This class wraps up the Modular::less_than operation into a "binary
/// predicate" class type so it can be used in template metaprogramming.
template<typename V>
struct ModularLess
{
  bool operator()(const V& lhs, const V& rhs) const
  {
    return modular(lhs).less_than(rhs);
  }
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__MODULAR_HPP
