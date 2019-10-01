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

#ifndef SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP
#define SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <list>
#include <map>

namespace rmf_traffic {
namespace internal {

//==============================================================================
struct SegmentElement;
using SegmentList = std::list<SegmentElement>;
using OrderMap = std::map<Time, SegmentList::iterator>;

struct SegmentElement
{
  struct Data
  {
    Time finish_time;
    Trajectory::ConstProfilePtr profile;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
  };

  Data data;

  // We store a Trajectory::Segment in this struct so that we can always safely
  // return a reference to a Trajectory::Segment object. As long as this
  // SegmentData is alive, any Trajectory::Segment reference that refers to it
  // will remain valid.
  std::unique_ptr<Trajectory::Segment> myself;

  SegmentElement(Data input_data)
    : data(std::move(input_data))
  {
    // Do nothing
  }

  SegmentElement(const SegmentElement& other)
    : data(other.data)
  {
    // Do nothing
  }

  SegmentElement& operator=(const SegmentElement& other)
  {
    data = other.data;
    return *this;
  }

  SegmentElement(SegmentElement&&) = default;
  SegmentElement& operator=(SegmentElement&&) = default;
};

} // namespace internal
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__TRAJECTORYINTERNAL_HPP
