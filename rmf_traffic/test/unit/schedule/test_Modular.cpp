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

#include "src/rmf_traffic/schedule/Modular.hpp"

#include <rmf_utils/catch.hpp>

TEMPLATE_TEST_CASE("Test Modular", "[modular]", uint8_t, uint16_t, uint64_t)
{
  auto max_value = std::numeric_limits<TestType>::max();
  auto min_value = std::numeric_limits<TestType>::min();

  auto _max_value = max_value - 1;
  auto min_value_ = min_value + 1;

  // Here we check less_than() returns false when lhs = rhs
  CHECK_FALSE(rmf_traffic::schedule::modular(max_value).less_than(max_value));
  CHECK_FALSE(rmf_traffic::schedule::modular(min_value).less_than(min_value));

  // Here we check less_than_or_equal() returns true when lhs = rhs
  CHECK(rmf_traffic::schedule::modular(max_value).less_than_or_equal(max_value));
  CHECK(rmf_traffic::schedule::modular(min_value).less_than_or_equal(min_value));

  // Here we check _max_value < max_value < min_value < min_value_
  CHECK(rmf_traffic::schedule::modular(_max_value).less_than(max_value));
  CHECK(rmf_traffic::schedule::modular(max_value).less_than(min_value));
  CHECK(rmf_traffic::schedule::modular(min_value).less_than(min_value_));

  // Here we check lhs < rhs when basis < lhs
  auto __max_value = _max_value - 1;
  CHECK(rmf_traffic::schedule::modular(__max_value).less_than(_max_value, max_value));
  CHECK(rmf_traffic::schedule::modular(max_value).less_than(min_value, min_value_));

  // Here we check lhs = rhs when basis < lhs
  CHECK(rmf_traffic::schedule::modular(_max_value).less_than_or_equal(max_value, max_value));
  CHECK(rmf_traffic::schedule::modular(max_value).less_than_or_equal(min_value, min_value));
}
