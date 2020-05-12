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

// A POC of wrapping rxcpp into a generic job system.

#include <rxcpp/rx.hpp>
#include <memory>

template<typename Job>
struct JobProgress
{
  Job& job;
  decltype(job.process()) result;
  bool& cancelled;

  inline void cancel() const {
    cancelled = true;
  }
};

/**
 * Recursively run a job on a worker until it is completed or cancelled, each iteration is re-queued
 * at the back of the worker job list.
 *
 * @tparam Subscriber
 * @tparam Job
 * @param worker
 * @param s
 * @param j
 */
template<typename Subscriber, typename Job>
void schedule_job(const rxcpp::schedulers::worker& worker, const Subscriber& s, Job& j)
{
  worker.schedule([s, &j, worker](const auto&)
  {
    // TODO: rxcpp can't pass by non-const reference to the subscriber, using a reference to the
    // cancelled state as a work around. Another possibly is to make it a stateful observer
    bool cancelled;
    auto progress = JobProgress<Job>{j, j.process(), cancelled};
    s.on_next(progress);
    if (!j.completed() && !progress.cancelled)
      schedule_job(worker, s, j);
    else
      s.on_completed();
  });
}

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
template<typename Job>
auto make_observable(Job& job)
{
  static auto event_loop = rxcpp::schedulers::make_event_loop();
  return rxcpp::observable<>::create<JobProgress<Job>>([&job](const auto& s)
  {
    auto worker = event_loop.create_worker();
    schedule_job(worker, s, job);
  });
}

/**
 * Runs a group of jobs in parallel in an event loop. A job is a type that contains the following
 * `Result Job::process()`
 * `bool Job::completed()`
 *
 * @tparam Job
 * @tparam Handler
 * @param jobs
 * @param handler
 */
template<typename Job, typename Handler>
void run_jobs(std::vector<std::shared_ptr<Job>>& jobs, const Handler& handler)
{
  // needed to prevent dynamic observables, which reduces performance
  using Observable = decltype(make_observable(*jobs[0]));

  auto obs = rxcpp::observable<>::create<Observable>([&jobs](const auto& s)
  {
    for (auto& j : jobs) {
      s.on_next(make_observable(*j));
    }
    s.on_completed();
  })
    // serialize_event_loop makes sure that the next progress update is only queue after the current
    // has been received and processed. The decision to continue or stop the job may depend on the
    // outcome of the result, so we need to wait until the subscriber processed the result before
    // continue.
    //
    // Other options are `observe_on_event_loop` and `synchronize_event_loop`, they do not wait
    // for the result to be processed so are not suitable. No coordination also waits for results
    // to be processed but does not provide thread safety.
    .merge(rxcpp::serialize_event_loop());

  // Instead of creating a higher order observable, create an observable of jobs and map them to
  // an observable.
  //
  // This doesn't work for some reason, using `serialize_event_loop` causes an infinite loop.
  // other coordination causes compile errors.
//  auto obs = rxcpp::observable<>::iterate(jobs).map([](const auto& j)
//  {
//    return make_observable(*j);
//  }).merge(rxcpp::serialize_event_loop());

  obs.as_blocking().subscribe(handler);
}
