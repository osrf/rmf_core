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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP

#include <rmf_traffic/schedule/Viewer.hpp>

#include <vector>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Viewer::View::Implementation
{
public:

  struct Storage
  {
    ParticipantId participant;
    ConstRoutePtr route;
  };

  std::vector<Storage> storage;
  std::vector<Element> elements;

  static View make_view(std::vector<Storage> input)
  {
    std::vector<Element> elements;
    elements.reserve(input.size());
    for (const auto& s : input)
    {
      assert(s.route);
      elements.emplace_back(Element{s.participant, *s.route});
    }

    View view;
    view._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            std::move(input),
            std::move(elements)
          });
    return view;
  }
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
