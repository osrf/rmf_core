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

#ifndef RMF_UTILS__IMPL_PTR_HPP
#define RMF_UTILS__IMPL_PTR_HPP

#include <memory>
#include <type_traits>
#include <cassert>

// These utilities were derived from https://github.com/oliora/samples/blob/master/spimpl.h
// Which uses the following boost license:
/*
    Copyright (c) 2015 Andrey Upadyshev (oliora@gmail.com)
    Distributed under the Boost Software License, Version 1.0.
    See http://www.boost.org/LICENSE_1_0.txt
 */


namespace rmf_utils {
namespace details {

//==============================================================================
template<class T>
T* default_copy(const T* original)
{
  static_assert(sizeof(T) > 0 && !std::is_void<T>::value,
                "default_copy cannot copy an incomplete type");
  return new T(*original);
}

//==============================================================================
template<class T>
void default_delete(T* ptr)
{
  static_assert(sizeof(T) > 0 && !std::is_void<T>::value,
                "default_delete cannot delete an incomplete type");
  delete ptr;
}

//==============================================================================
template<class T>
using default_deleter_t = void (*)(T*);

//==============================================================================
template<class T>
using default_copier_t = T* (*)(const T*);

template<class T, class D, class C = default_copier_t<T>>
struct is_default_manageable : public std::integral_constant<bool,
    std::is_same<D, default_deleter_t<T>>::value &&
    std::is_same<C, default_copier_t<T>>::value
> { };

} // namespace details

//==============================================================================
template<class T, class Deleter = details::default_deleter_t<T>>
class unique_impl_ptr
{
protected:
  static_assert(
      !std::is_array<T>::value,
      "unique_impl_ptr specialization for arrays is not implemented");
  struct dummy_t_ {int dummy__;};

public:
  using pointer = T*;
  using const_pointer = typename std::add_const<T>::type *;
  using reference = T&;
  using const_reference = typename std::add_const<T>::type &;
  using element_type = T;
  using deleter_type = typename std::decay<Deleter>::type;
  using unique_ptr_type = std::unique_ptr<T, deleter_type>;
  using is_default_manageable = details::is_default_manageable<T, deleter_type>;

  constexpr unique_impl_ptr() noexcept
  : ptr_(nullptr, deleter_type{}) {}

  constexpr unique_impl_ptr(std::nullptr_t) noexcept
  : unique_impl_ptr() {}

  template<class D>
  unique_impl_ptr(pointer p, D&& d,
           typename std::enable_if<
              std::is_convertible<D, deleter_type>::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : ptr_(std::move(p), std::forward<D>(d)) {}

  template<class U>
  unique_impl_ptr(U *u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && is_default_manageable::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : unique_impl_ptr(u, &details::default_delete<T>, &details::default_copy<T>) {}

  unique_impl_ptr(unique_impl_ptr&& r) noexcept = default;


  template<class U>
  unique_impl_ptr(std::unique_ptr<U>&& u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && is_default_manageable::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : ptr_(u.release(), &details::default_delete<T>) {}

  template<class U, class D>
  unique_impl_ptr(std::unique_ptr<U, D>&& u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && std::is_convertible<D, deleter_type>::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : ptr_(std::move(u)) {}

  template<class U, class D>
  unique_impl_ptr(unique_impl_ptr<U, D>&& u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && std::is_convertible<D, deleter_type>::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : ptr_(std::move(u.ptr_)) {}

  unique_impl_ptr(const unique_impl_ptr<T, Deleter>&) = delete;

  unique_impl_ptr& operator= (unique_impl_ptr&& r) noexcept = default;

  template<class U>
  typename std::enable_if<
      std::is_convertible<U*, pointer>::value
          && is_default_manageable::value,
      unique_impl_ptr&
  >::type operator= (std::unique_ptr<U>&& u) noexcept
  {
      return operator=(unique_impl_ptr(std::move(u)));
  }

  reference operator=(const unique_impl_ptr<T, Deleter>&) = delete;

  reference operator*() { return *ptr_; }
  const_reference operator*() const { return *ptr_; }

  pointer operator->() noexcept { return get(); }
  const_pointer operator->() const noexcept { return get(); }

  pointer get() noexcept { return ptr_.get(); }
  const_pointer get() const noexcept { return ptr_.get(); }

  void swap(unique_impl_ptr& u) noexcept
  {
      using std::swap;
      ptr_.swap(u.ptr_);
  }

  pointer release() noexcept { return ptr_.release(); }

  unique_ptr_type release_unique() noexcept { return std::move(ptr_); }

  explicit operator bool() const noexcept { return static_cast<bool>(ptr_); }

  typename std::remove_reference<deleter_type>::type& get_deleter() noexcept { return ptr_.get_deleter(); }
  const typename std::remove_reference<deleter_type>::type& get_deleter() const noexcept { return ptr_.get_deleter(); }

protected:
  unique_ptr_type ptr_;
};


template<class T, class D>
inline void swap(unique_impl_ptr<T, D>& l, unique_impl_ptr<T, D>& r) noexcept
{
  l.swap(r);
}


template <class T1, class D1, class T2, class D2>
inline bool operator==(const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  return l.get() == r.get();
}

template <class T1, class D1, class C1, class T2, class D2>
inline bool operator!=(const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  return !(l == r);
}

template <class T1, class D1, class T2, class D2>
inline bool operator< (const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  using P1 = typename unique_impl_ptr<T1, D1>::pointer;
  using P2 = typename unique_impl_ptr<T2, D2>::pointer;
  using CT = typename std::common_type<P1, P2>::type;
  return std::less<CT>()(l.get(), r.get());
}

template <class T1, class D1, class T2, class D2>
inline bool operator> (const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  return r < l;
}

template <class T1, class D1, class T2, class D2>
inline bool operator<=(const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  return !(r < l);
}

template <class T1, class D1, class T2, class D2>
inline bool operator>=(const unique_impl_ptr<T1, D1>& l, const unique_impl_ptr<T2, D2>& r)
{
  return !(l < r);
}

template <class T, class D>
inline bool operator==(const unique_impl_ptr<T, D>& p, std::nullptr_t) noexcept
{
  return !p;
}

template <class T, class D>
inline bool operator==(std::nullptr_t, const unique_impl_ptr<T, D>& p) noexcept
{
  return !p;
}

template <class T, class D>
inline bool operator!=(const unique_impl_ptr<T, D>& p, std::nullptr_t) noexcept
{
  return static_cast<bool>(p);
}

template <class T, class D>
inline bool operator!=(std::nullptr_t, const unique_impl_ptr<T, D>& p) noexcept
{
  return static_cast<bool>(p);
}

template <class T, class D>
inline bool operator< (const unique_impl_ptr<T, D>& l, std::nullptr_t)
{
  using P = typename unique_impl_ptr<T, D>::pointer;
  return std::less<P>()(l.get(), nullptr);
}

template <class T, class D>
inline bool operator< (std::nullptr_t, const unique_impl_ptr<T, D>& p)
{
  using P = typename unique_impl_ptr<T, D>::pointer;
  return std::less<P>()(nullptr, p.get());
}

template <class T, class D>
inline bool operator> (const unique_impl_ptr<T, D>& p, std::nullptr_t)
{
  return nullptr < p;
}

template <class T, class D>
inline bool operator> (std::nullptr_t, const unique_impl_ptr<T, D>& p)
{
  return p < nullptr;
}

template <class T, class D>
inline bool operator<=(const unique_impl_ptr<T, D>& p, std::nullptr_t)
{
  return !(nullptr < p);
}

template <class T, class D>
inline bool operator<=(std::nullptr_t, const unique_impl_ptr<T, D>& p)
{
  return !(p < nullptr);
}

template <class T, class D>
inline bool operator>=(const unique_impl_ptr<T, D>& p, std::nullptr_t)
{
  return !(p < nullptr);
}

template <class T, class D>
inline bool operator>=(std::nullptr_t, const unique_impl_ptr<T, D>& p)
{
  return !(nullptr < p);
}


template<class T, class... Args>
inline unique_impl_ptr<T> make_unique_impl(Args&&... args)
{
  return unique_impl_ptr<T>(new T(std::forward<Args>(args)...), &details::default_delete<T>);
}


// Helpers to manage unique impl, stored in std::unique_ptr
template<class T, class Deleter = details::default_deleter_t<T>, class Copier = details::default_copier_t<T>>
class impl_ptr : public unique_impl_ptr<T, Deleter>
{
  using base_type = unique_impl_ptr<T, Deleter>;
  using dummy_t_ = typename base_type::dummy_t_;
public:
  using pointer = typename base_type::pointer;
  using const_pointer = typename base_type::const_pointer;
  using reference = typename base_type::reference;
  using const_reference = typename base_type::const_reference;
  using element_type = typename base_type::element_type;
  using deleter_type = typename base_type::deleter_type;
  using unique_ptr_type = typename base_type::unique_ptr_type;
  using copier_type = typename std::decay<Copier>::type;
  using is_default_manageable = details::is_default_manageable<T, deleter_type, copier_type>;

  constexpr impl_ptr() noexcept
  : base_type(nullptr, deleter_type{}), copier_(copier_type{}) {}

  constexpr impl_ptr(std::nullptr_t) noexcept
  : impl_ptr() {}

  template<class D, class C>
  impl_ptr(pointer p, D&& d, C&& c,
           typename std::enable_if<
              std::is_convertible<D, deleter_type>::value
                  && std::is_convertible<C, copier_type>::value,
              typename base_type::dummy_t_
           >::type = typename base_type::dummy_t_()) noexcept
  : base_type(std::move(p), std::forward<D>(d)), copier_(std::forward<C>(c)) {}

  template<class U>
  impl_ptr(U *u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && is_default_manageable::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : impl_ptr(u, &details::default_delete<T>, &details::default_copy<T>) {}

  impl_ptr(const impl_ptr& r)
  : impl_ptr(r.clone()) {}

  impl_ptr(impl_ptr&& r) noexcept = default;

  template<class U>
  impl_ptr(std::unique_ptr<U>&& u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && is_default_manageable::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : base_type(u.release(), &details::default_delete<T>) {}

  template<class U, class D, class C>
  impl_ptr(std::unique_ptr<U, D>&& u, C&& c,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && std::is_convertible<D, deleter_type>::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : base_type(std::move(u)), copier_(std::forward<C>(c)) {}

  template<class U, class D, class C>
  impl_ptr(impl_ptr<U, D, C>&& u,
           typename std::enable_if<
              std::is_convertible<U*, pointer>::value
                  && std::is_convertible<D, deleter_type>::value,
              dummy_t_
           >::type = dummy_t_()) noexcept
  : base_type(std::move(u.ptr_)), copier_(std::move(u.copier_)) {}

  impl_ptr& operator= (const impl_ptr& r)
  {
    if (this == &r)
        return *this;

    return operator=(r.clone());
  }

  impl_ptr& operator= (impl_ptr&& r) noexcept = default;

  template<class U, class D, class C>
  typename std::enable_if<
      std::is_convertible<U*, pointer>::value
          && std::is_convertible<D, deleter_type>::value
          && std::is_convertible<C, copier_type>::value,
      impl_ptr&
  >::type operator= (const impl_ptr<U, D, C>& u)
  {
    return operator=(u.clone());
  }

  //

  template<class U>
  typename std::enable_if<
      std::is_convertible<U*, pointer>::value
          && is_default_manageable::value,
      impl_ptr&
  >::type operator= (std::unique_ptr<U>&& u) noexcept
  {
    return operator=(impl_ptr(std::move(u)));
  }

  template<class U, class D, class C>
  typename std::enable_if<
      std::is_convertible<U*, pointer>::value
          && std::is_convertible<D, deleter_type>::value
          && std::is_convertible<C, copier_type>::value,
      impl_ptr&
  >::type operator= (impl_ptr<U, D, C>&& u) noexcept
  {
    base_type::ptr_ = std::move(u.ptr_);
    copier_ = std::move(u.copier_);
    return *this;
  }

  void swap(impl_ptr& u) noexcept
  {
    using std::swap;
    base_type::ptr_.swap(u.ptr_);
    swap(copier_, u.copier_);
  }

  impl_ptr clone() const
  {
    return impl_ptr(
        base_type::ptr_ ? copier_(base_type::ptr_.get()) : nullptr,
        base_type::ptr_.get_deleter(),
        copier_);
  }

  const typename std::remove_reference<copier_type>::type& get_copier() const noexcept { return copier_; }
  typename std::remove_reference<copier_type>::type& get_copier() noexcept { return copier_; }
private:
  copier_type copier_;
};

template<class T, class... Args>
inline impl_ptr<T> make_impl(Args&&... args)
{
  return impl_ptr<T>(new T(std::forward<Args>(args)...), &details::default_delete<T>, &details::default_copy<T>);
}

template<class U, class D, class... Args>
impl_ptr<U> make_derived_impl(Args&&... args)
{
  return impl_ptr<U>(
        new D(std::forward<Args>(args)...),
        [](U* ptr) -> void { delete static_cast<D*>(ptr); },
        [](const U* ptr) -> U* { return new D(*static_cast<const D*>(ptr)); });
}

template<class T, class D, class C>
inline void swap(impl_ptr<T, D, C>& l,impl_ptr<T, D, C>& r) noexcept
{
  l.swap(r);
}
} // namespace rmf_utils

namespace std {
template <class T, class D>
struct hash<rmf_utils::unique_impl_ptr<T, D>>
{
  using argument_type = rmf_utils::unique_impl_ptr<T, D> ;
  using result_type = size_t;

  result_type operator()(const argument_type& p) const noexcept
  {
    return hash<typename argument_type::pointer>()(p.get());
  }
};

template <class T, class D, class C>
struct hash<rmf_utils::impl_ptr<T, D, C>>
{
  using argument_type = rmf_utils::impl_ptr<T, D, C> ;
  using result_type = size_t;

  result_type operator()(const argument_type& p) const noexcept
  {
    return hash<typename argument_type::pointer>()(p.get());
  }
};
} // namespace std

#endif // RMF_UTILS__IMPL_PTR_HPP
