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

#ifndef RMF_UTILS__OPTIONAL_HPP
#define RMF_UTILS__OPTIONAL_HPP

#include <utility>
#include <stdexcept>

namespace rmf_utils {

struct nullopt { };

struct bad_optional_access : std::logic_error
{
  bad_optional_access()
    : std::logic_error("bad optional access")
  {
    // Do nothing
  }
};

// TODO(MXG): Replace this with a std::optional when we're able to support C++17
template<typename T>
class optional
{
public:

  optional()
  : _has_value(false),
    _storage(nullopt())
  {
    // Do nothing
  }
  optional(nullopt)
  : _has_value(false),
    _storage(nullopt())
  {
    // Do nothing
  }
  optional& operator=(nullopt)
  {
    _has_value = false;
    return *this;
  }


  optional(const T& value)
  : _has_value(true),
    _storage(T(value))
  {
    // Do nothing
  }
  optional(T&& value)
  : _has_value(true),
    _storage(T(std::move(value)))
  {
    // Do nothing
  }
  optional& operator=(const T& value)
  {
    _has_value = true;
    _storage = value;
    return *this;
  }
  optional& operator=(T&& value)
  {
    _has_value = true;
    _storage = std::move(value);
  }


  bool has_value() const
  {
    return _has_value;
  }
  operator bool() const
  {
    return _has_value;
  }


  T& value()
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }
  const T& value() const
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }

  T* operator->()
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }
  const T* operator->() const
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }

  T& operator*() &
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }
  const T& operator*() const&
  {
    if (!_has_value)
      throw bad_optional_access();

    return _storage._value;
  }

  T&& operator*() &&
  {
    if (_has_value)
      throw bad_optional_access();

    return std::move(_storage._value);
  }
  const T&& operator*() const&&
  {
    if (_has_value)
      throw bad_optional_access();

    return std::move(_storage._value);
  }

  ~optional()
  {
    if(_has_value)
      _storage.~T();
  }

private:

  bool _has_value;
  union {
    nullopt _null;
    T _value;
  } _storage;

};

} // namespace rmf_utils

#endif // RMF_UTILS__OPTIONAL_HPP
