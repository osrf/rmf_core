/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rmf_task/Request.hpp>

namespace rmf_task {

//==============================================================================
class Request::Implementation
{
public:
  std::string id;
  rmf_traffic::Time earliest_start_time;
  rmf_task::ConstPriorityPtr priority;
  DescriptionPtr description;
};

//==============================================================================
Request::Request(
  const std::string& id,
  rmf_traffic::Time earliest_start_time,
  ConstPriorityPtr priority,
  DescriptionPtr description)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation {
          id,
          earliest_start_time,
          std::move(priority),
          std::move(description)
      }))
{
    // Do nothing
}

//==============================================================================
const std::string& Request::id() const
{
  return _pimpl->id;
}

//==============================================================================
rmf_traffic::Time Request::earliest_start_time() const
{
  return _pimpl->earliest_start_time;
}

//==============================================================================
ConstPriorityPtr Request::priority() const
{
  return _pimpl->priority;
}

//==============================================================================
const Request::DescriptionPtr& Request::description() const
{
  return _pimpl->description;
}

} // namespace rmf_task