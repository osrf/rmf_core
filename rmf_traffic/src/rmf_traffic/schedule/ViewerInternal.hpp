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
    RouteId route_id;
    ConstRoutePtr route;
    std::shared_ptr<const ParticipantDescription> description;
  };

  std::vector<Storage> storage;
  std::vector<Element> elements;

  static View make_view(std::vector<Storage> input)
  {
    std::vector<Element> elements = make_elements(input);

    View view;
    view._pimpl = rmf_utils::make_impl<Implementation>(
          Implementation{
            std::move(input),
            std::move(elements)
          });
    return view;
  }

  static void append_to_view(View& view, std::vector<Storage> input)
  {
    append_to_elements(view._pimpl->elements, input);
    view._pimpl->storage.insert(
          view._pimpl->storage.end(),
          std::make_move_iterator(input.begin()),
          std::make_move_iterator(input.end()));
  }

  static std::vector<Element> make_elements(
      const std::vector<Storage>& input)
  {
    std::vector<Element> elements;
    append_to_elements(elements, input);
    return elements;
  }

  static void append_to_elements(
      std::vector<Element>& elements,
      const std::vector<Storage>& input)
  {
    elements.reserve(elements.size() + input.size());
    for (const auto& s : input)
    {
      assert(s.route);
      elements.emplace_back(
            Element{s.participant, s.route_id, *s.route, *s.description});
    }
  }
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__VIEWERINTERNAL_HPP
