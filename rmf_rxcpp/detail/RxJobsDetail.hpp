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

#ifndef RMF_RXCPP__RXJOBSDETAIL_HPP
#define RMF_RXCPP__RXJOBSDETAIL_HPP

#include <rxcpp/rx.hpp>

namespace detail {
template<typename Job, typename Subscriber>
using IsAsyncJob = std::is_constructible<std::function<void(Subscriber, int)>, typename Job::element_type>;

template<typename Job, typename Subscriber>
void schedule_job(const Job& j, const Subscriber& s, const rxcpp::schedulers::worker w, typename std::enable_if_t<IsAsyncJob<Job, Subscriber>::value>* = 0)
{
  w.schedule([j, s, w](const auto&) { (*j)(s, w); });
}

template<typename Job, typename Subscriber>
void schedule_job(const Job& j, const Subscriber& s, const rxcpp::schedulers::worker w, typename std::enable_if_t<!IsAsyncJob<Job, Subscriber>::value>* = 0)
{
  w.schedule([j, s](const auto&) { (*j)(s); });
}

} // namespace detail

#endif //RMF_RXCPP__RXJOBSDETAIL_HPP
