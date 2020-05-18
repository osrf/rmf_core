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

#include <rmf_utils/catch.hpp>

#include <RxJobs.hpp>

TEST_CASE("run simple job", "[Jobs]")
{
  bool ran = false;
  auto j = make_job([](const auto& s)
  {
    s.on_next(1);
    s.on_completed();
  });
  run_job_blocking<int>(j, [&ran](const auto&) { ran = true; });
  REQUIRE(ran);
}

TEST_CASE("run multiple jobs in parallel", "[Jobs]")
{
  bool job1_success = false;
  bool job2_success = false;
  auto job1 = make_job([&job1_success, &job2_success](const auto& s) {
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < timeout)
    {
      if (job2_success)
        break;
    }
    job1_success = job2_success;
    s.on_completed();
  });
  auto job2 = make_job([&job2_success](const auto& s) {
    job2_success = true;
    s.on_completed();
  });

  run_jobs_blocking<int>(std::make_tuple(job1, job2), [](const auto&) {});
  REQUIRE(job1_success);
}

struct AsyncCounterJob
{
  int counter = 0;

  template<typename Subscriber, typename Worker>
  void operator()(const Subscriber& s, const Worker& w)
  {
    s.on_next(++counter);
    if (counter >= 10)
    {
      s.on_completed();
      return;
    }
    w.schedule([this, s, w](const auto&)
    {
      (*this)(s, w);
    });
  }
};

TEST_CASE("async job", "[Jobs]")
{
  auto j = std::make_shared<AsyncCounterJob>();
  run_job_blocking<int>(j);
  REQUIRE(j->counter == 10);
}

TEST_CASE("job completion handler is called", "[Jobs]")
{
  bool called = false;
  auto j = make_job([](const auto& s)
  {
    s.on_completed();
  });
  run_job_blocking<int>(j, [](const auto&) {}, [&called]()
  {
    called = true;
  });
  REQUIRE(called);
}

TEST_CASE("nested job", "[Jobs]")
{
  auto job1 = make_job([](const auto& s)
  {
    s.on_next(std::string{"hello"});
    s.on_completed();
  });
  auto job2 = make_job([&job1](const auto& s)
  {
    run_job<std::string>(job1, [s](const auto& val)
    {
      s.on_next(val + " world");
      s.on_completed();
    });
  });
  run_job_blocking<std::string>(job2, [](const auto& val)
  {
    REQUIRE(val == "hello world");
  });
}

TEST_CASE("cancelling job", "[Jobs]")
{
  int counter = 0;
  auto j = make_job([&counter](const auto& s)
  {
    for (int i = 1; i <= 10; i++)
    {
      if (!s.is_subscribed())
        break;
      counter = i;
      s.on_next(i);
    }
    s.on_completed();
  });
  auto subscription = make_subscription();
  run_job_blocking<int>(j, subscription, [&subscription](int i)
  {
    if (i >= 5)
      subscription.unsubscribe();
  });
  REQUIRE(counter == 5);
}