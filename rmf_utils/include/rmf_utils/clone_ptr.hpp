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

#ifndef RMF_UTILS__CLONE_PTR_HPP
#define RMF_UTILS__CLONE_PTR_HPP

#include <memory>
#include <type_traits>

namespace rmf_utils {

//==============================================================================
template<typename T>
class Cloneable
{
public:

  std::unique_ptr<T> clone() const = 0;

};

//==============================================================================
template<typename T>
class clone_ptr;

//==============================================================================
template<typename T, typename... Args>
clone_ptr<T> make_clone(Args&&... args);

//==============================================================================
template<typename T>
class clone_ptr
{
public:

  clone_ptr()
  : _ptr(nullptr)
  {
    // Do nothing
  }

  explicit clone_ptr(T* ptr)
  : _ptr(ptr)
  {
    // Do nothing
  }

  clone_ptr(std::nullptr_t)
  : _ptr(nullptr)
  {
    // Do nothing
  }

  clone_ptr(const clone_ptr& other)
  {
    if (other._ptr)
      _ptr = std::unique_ptr<T>(static_cast<T*>(other->clone().release()));
  }

  template<typename U>
  clone_ptr(const clone_ptr<U>& other)
  {
    if (other._ptr)
      _ptr = std::unique_ptr<T>(other->clone().release());
  }

  clone_ptr(clone_ptr&& other)
  : _ptr(std::move(other._ptr))
  {
    // Do nothing
  }

  template<typename U>
  clone_ptr(clone_ptr<U>&& other)
  : _ptr(std::move(other._ptr))
  {
    // Do nothing
  }

  clone_ptr& operator=(const clone_ptr& other)
  {
    _ptr = other._ptr? other._ptr->clone().release() : nullptr;
    return *this;
  }

  template<typename U>
  clone_ptr& operator=(const clone_ptr<U>& other)
  {
    _ptr = other._ptr? other._ptr->clone().release() : nullptr;
    return *this;
  }

  clone_ptr& operator=(clone_ptr&& other)
  {
    _ptr = std::move(other._ptr);
    return *this;
  }

  template<typename U>
  clone_ptr& operator=(clone_ptr<U>&& other)
  {
    _ptr = std::move(other._ptr);
    return *this;
  }

  T* get() const
  {
    return _ptr.get();
  }

  T* release()
  {
    return _ptr.release();
  }

  T& operator*() const
  {
    return *_ptr;
  }

  T* operator->() const
  {
    return _ptr.get();
  }

  operator bool() const
  {
    return static_cast<bool>(_ptr);
  }

  // TODO(MXG): Consider the spaceship operator when we can use C++20

  bool operator==(const clone_ptr& other) const
  {
    return _ptr == other._ptr;
  }

  bool operator==(std::nullptr_t) const
  {
    return _ptr == nullptr;
  }

  bool operator!=(const clone_ptr& other) const
  {
    return _ptr != other._ptr;
  }

  bool operator!=(std::nullptr_t) const
  {
    return _ptr != nullptr;
  }

  bool operator<(const clone_ptr& other) const
  {
    return _ptr < other._ptr;
  }

  bool operator>(const clone_ptr& other) const
  {
    return _ptr > other._ptr;
  }

  bool operator<=(const clone_ptr& other) const
  {
    return _ptr <= other._ptr;
  }

  bool operator>=(const clone_ptr& other) const
  {
    return _ptr >= other._ptr;
  }

private:

  template<typename U>
  friend class clone_ptr;

  std::unique_ptr<T> _ptr;
};

//==============================================================================
template<typename T, typename... Args>
clone_ptr<T> make_clone(Args&&... args)
{
  return clone_ptr<T>(new T(std::forward<Args>(args)...));
}

} // namespace rmf_utils

#endif // RMF_UTILS__CLONE_PTR_HPP
