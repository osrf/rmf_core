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

struct nullopt_t { };

inline constexpr nullopt_t nullopt{};

struct bad_optional_access : std::logic_error
{
  bad_optional_access()
    : std::logic_error("bad optional access")
  {
    // Do nothing
  }
};

// TODO(MXG): Replace this with a std::optional when we're able to support C++17
// TODO(MXG): Consider removing the safety check for the dereference operators
template<typename T>
class optional
{
public:

  optional()
  : _has_value(false),
    _storage(nullopt)
  {
    // Do nothing
  }
  optional(nullopt_t)
  : _has_value(false),
    _storage(nullopt)
  {
    // Do nothing
  }
  optional& operator=(nullopt_t)
  {
    if (_has_value)
      _storage._value.~T();

    _has_value = false;
    _storage._null = nullopt;
    return *this;
  }

  optional(const optional& other)
  : _has_value(other._has_value),
    _storage(other._has_value, other._storage)
  {
    // Do nothing
  }

  optional(optional&& other)
  : _has_value(other._has_value),
    _storage(other._has_value, std::move(other._storage))
  {
    // Do nothing
  }

  optional& operator=(const optional& other)
  {
    if (other._has_value)
    {
      *this = other._storage._value;
      return *this;
    }

    if (_has_value)
    {
      // If the other does not have a value but this one does,
      // then destruct the value contained in this object's storage.
      _storage._value.~T();
    }

    _has_value = false;
    _storage._null = nullopt;
    return *this;
  }

  optional& operator=(optional&& other)
  {
    if (other._has_value)
    {
      *this = std::move(other._storage._value);
      return *this;
    }

    if (_has_value)
    {
      // If the other does not have a value but this one does,
      // then destruct the value contained in this object's storage.
      _storage._value.~T();
    }

    _has_value = false;
    _storage._null = nullopt;
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
    if (_has_value)
    {
      _storage._value = value;
    }
    else
    {
      new (&_storage._value) T(value);
    }

    _has_value = true;
    return *this;
  }
  optional& operator=(T&& value)
  {
    if (_has_value)
    {
      _storage._value = std::move(value);
    }
    else
    {
      new (&_storage._value) T(std::move(value));
    }

    _has_value = true;
    return *this;
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

    return &_storage._value;
  }
  const T* operator->() const
  {
    if (!_has_value)
      throw bad_optional_access();

    return &_storage._value;
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
    if (!_has_value)
      throw bad_optional_access();

    return std::move(_storage._value);
  }
  const T&& operator*() const&&
  {
    if (!_has_value)
      throw bad_optional_access();

    return std::move(_storage._value);
  }

  ~optional()
  {
    if(_has_value)
      _storage._value.~T();
  }

private:

  bool _has_value;
  union Storage {

    Storage(nullopt_t)
    : _null(nullopt)
    {
      // Do nothing
    }

    Storage(const T& other)
    : _value(other)
    {
      // Do nothing
    }

    Storage(T&& other)
    : _value(std::move(other))
    {
      // Do nothing
    }

    Storage(bool has_value, const Storage& other)
    : _null(nullopt)
    {
      if (has_value)
      {
        new (&_value) T(other._value);
      }
    }

    Storage(bool has_value, Storage&& other)
    : _null(nullopt)
    {
      if (has_value)
      {
        new (&_value) T(std::move(other._value));
      }
    }

    nullopt_t _null;
    T _value;

    ~Storage() { }
  } _storage;

};

} // namespace rmf_utils

#endif // RMF_UTILS__OPTIONAL_HPP
