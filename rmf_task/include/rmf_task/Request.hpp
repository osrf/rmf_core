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

#ifndef RMF_TASK__TASK_HPP
#define RMF_TASK__TASK_HPP

#include <memory>

#include <rmf_task/Estimate.hpp>
#include <rmf_task/agv/State.hpp>
#include <rmf_task/agv/Constraints.hpp>
#include <rmf_task/Priority.hpp>

#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_task {

/// Implement this for new type of requests.
class Request
{
public:
  class Description
  {
  public:

    /// The unique id for this request
    virtual const std::string& id() const = 0;

    /// Estimate the state of the robot when the request is finished along with the
    /// time the robot has to wait before commencing the request
    virtual std::optional<Estimate> estimate_finish(
      const agv::State& initial_state,
      const agv::Constraints& task_planning_constraints,
      const std::shared_ptr<EstimateCache> estimate_cache) const = 0;
    
    /// Estimate the invariant component of the request's duration
    virtual rmf_traffic::Duration invariant_duration() const = 0;

    virtual ~Description() = default;
  };
  using DescriptionPtr = std::shared_ptr<Description>;

  /// Constructor
  ///
  /// \param[in] earliest_start_time
  ///   The earliest time this request should begin execution. This is usually the
  ///   requested start time for the request.
  ///
  /// \param[in] priority
  ///   The priority for this request. This is provided by the Priority Scheme. For
  ///   requests that do not have any priority this is a nullptr.
  ///
  /// \param[in] description
  ///   The description for this request
  Request(
    rmf_traffic::Time earliest_start_time,
    ConstPriorityPtr priority
    DescriptionPtr description);

  /// Get the earliest time that this request may begin
  rmf_traffic::Time earliest_start_time() const;

  /// Get the priority of this request
  ConstPriorityPtr priority() const;

  /// Get the description of this request
  const DescriptionPtr& description() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RequestPtr = std::shared_ptr<Request>;
using ConstRequestPtr = std::shared_ptr<const Request>;

} // namespace rmf_task

#endif // RMF_TASK__REQUEST_HPP
