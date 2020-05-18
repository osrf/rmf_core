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

#ifndef RMF_RXCPP__RXJOBS_HPP
#define RMF_RXCPP__RXJOBS_HPP

#include "detail/RxJobsDetail.hpp"
#include <rxcpp/rx.hpp>
#include <memory>

/**
 * Creates an observable from a job, the observable runs the job in an event loop until it has
 * completed or cancelled. Each progress update on a job is queued at the back of the event loop
 * so that other jobs has a chance to start before earlier jobs are finished.
 *
 * @tparam Job
 * @tparam Result
 * @param job
 * @return
 */
template<typename T, typename Job>
auto make_observable(const Job& job)
{
  static auto event_loop = rxcpp::schedulers::make_event_loop();
  return rxcpp::observable<>::create<T>([job](const auto& s)
  {
    auto worker = event_loop.create_worker();
    detail::schedule_job(job, s, worker);
  });
}

template<typename T, typename Job0, typename Job1, typename... Jobs>
auto make_observable(Job0&& j0, Job1&& j1, Jobs&&... js)
{
  // serialize_event_loop makes sure that the next progress update is only queue after the current
  // has been received and processed. The decision to continue or stop the job may depend on the
  // outcome of the result, so we need to wait until the subscriber processed the result before
  // continue.
  //
  // Other options are `observe_on_event_loop` and `synchronize_event_loop`, they do not wait
  // for the result to be processed so are not suitable. No coordination also waits for results
  // to be processed but does not provide thread safety.
  auto o0 = make_observable<T>(j0);
  auto o1 = make_observable<T>(j1);
  return o0.merge(rxcpp::serialize_event_loop(), o1, make_observable<T>(js)...);
}

template<typename T, typename JobsIterable>
auto make_observable_from_job_list(const JobsIterable& jobs)
{
  // needed to prevent dynamic observables, which reduces performance
  using Observable = decltype(make_observable<T>(*jobs.begin()));

  return rxcpp::observable<>::create<Observable>([&jobs](const auto& s)
  {
    for (auto& j : jobs)
      s.on_next(make_observable<T>(j));
    s.on_completed();
  }).merge(rxcpp::serialize_event_loop());
}

template<typename T, typename... JobsAndHandler, size_t... Is, typename... SubscriberArgs>
inline void run_jobs_impl(const std::tuple<JobsAndHandler...>& js, const std::index_sequence<Is...>&, SubscriberArgs&&... args)
{
  make_observable<T>(std::get<Is>(js)...).subscribe(args...);
}

template<typename T, typename... JobsAndHandler, size_t... Is, typename... SubscriberArgs>
inline void run_jobs_blocking_impl(const std::tuple<JobsAndHandler...>& js, const std::index_sequence<Is...>&, SubscriberArgs&&... args)
{
  make_observable<T>(std::get<Is>(js)...).as_blocking().subscribe(args...);
}

template<typename T, typename Job, typename... SubscriberArgs>
inline auto run_job(const Job& job, SubscriberArgs&&... args)
{
  make_observable<T>(job).subscribe(args...);
}

template<typename T, typename... Jobs, typename... SubscriberArgs>
inline void run_jobs(const std::tuple<Jobs...>& jobs, SubscriberArgs&&... args)
{
  constexpr auto jobs_len = std::tuple_size<std::tuple<Jobs...>>::value;
  run_jobs_impl<T>(jobs, std::make_index_sequence<jobs_len>(), args...);
}

template<typename T, typename JobsIterable, typename... SubscriberArgs>
inline void run_jobs_list(const JobsIterable& jobs, SubscriberArgs&&... args)
{
  make_observable_from_job_list<T>(jobs).subscribe(args...);
}

template<typename T, typename Job, typename... SubscriberArgs>
inline auto run_job_blocking(const Job& job, SubscriberArgs&&... args)
{
  make_observable<T>(job).as_blocking().subscribe(args...);
}

template<typename T, typename... Jobs, typename... SubscriberArgs>
inline void run_jobs_blocking(const std::tuple<Jobs...>& jobs, SubscriberArgs&&... args)
{
  constexpr auto jobs_len = std::tuple_size<std::tuple<Jobs...>>::value;
  run_jobs_blocking_impl<T>(jobs, std::make_index_sequence<jobs_len>(), args...);
}

template<typename T, typename JobsIterable, typename... SubscriberArgs>
inline void run_jobs_list_blocking(const JobsIterable& jobs, SubscriberArgs&&... args)
{
  make_observable_from_job_list<T>(jobs).as_blocking().subscribe(args...);
}

inline auto make_subscription()
{
  return rxcpp::composite_subscription{};
}

template<typename F>
inline auto make_job(const F& f)
{
  return std::make_shared<F>(f);
}

#endif //RMF_RXCPP__RXJOBS_HPP
