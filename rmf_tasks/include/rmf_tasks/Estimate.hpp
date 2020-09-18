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

#ifndef INCLUDE__RMF_TASKS__ESTIMATE_HPP
#define INCLUDE__RMF_TASKS__ESTIMATE_HPP

#include <rmf_tasks/agv/State.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_tasks {

/// Estimates for requests
class Estimate
{
public:

  Estimate(agv::State finish_state, rmf_traffic::Time wait_until);

  agv::State finish_state() const;

  Estimate& finish_state(agv::State new_finish_state);

  rmf_traffic::Time wait_until() const;

  Estimate& wait_until(rmf_traffic::Time new_wait_until);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_tasks

#endif // INCLUDE__RMF_TASKS__ESTIMATE_HPP
