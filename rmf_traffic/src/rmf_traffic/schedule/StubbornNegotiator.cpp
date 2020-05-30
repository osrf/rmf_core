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

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class StubbornNegotiator::Implementation
{
public:

  const Participant* participant;

};

//==============================================================================
void StubbornNegotiator::respond(
    const schedule::Negotiation::Table::ViewerPtr& table_viewer,
    const Responder& responder,
    const bool*)
{
  const auto& itinerary = _pimpl->participant->itinerary();
  const auto& profile = _pimpl->participant->description().profile();
  const auto& proposal = table_viewer->base_proposals();

  for (const auto& p : proposal)
  {
    const auto other_participant = table_viewer->get_description(p.participant);
    assert(other_participant);
    const auto& other_profile = other_participant->profile();

    for (const auto& other_route : p.itinerary)
    {
      for (const auto& item : itinerary)
      {
        if (item.route->map() != other_route->map())
          continue;

        if (rmf_traffic::DetectConflict::between(
              profile,
              item.route->trajectory(),
              other_profile,
              other_route->trajectory()))
        {
          rmf_traffic::schedule::Itinerary alternative;
          alternative.reserve(itinerary.size());
          for (const auto& item : itinerary)
            alternative.emplace_back(item.route);

          return responder.reject({std::move(alternative)});
        }
      }
    }
  }

  std::vector<rmf_traffic::Route> submission;
  for (const auto& item : itinerary)
    submission.push_back(*item.route);

  responder.submit(std::move(submission));
}

} // namespace schedule
} // namespace rmf_traffic
