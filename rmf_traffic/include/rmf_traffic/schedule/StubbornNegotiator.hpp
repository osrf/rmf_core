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

#ifndef RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP
#define RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP

#include <rmf_traffic/schedule/Negotiator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A StubbornNegotiator will only accept plans that accommodate the current
/// itinerary of the
class StubbornNegotiator : public Negotiator
{
public:

  /// Constructor
  ///
  /// \note We take a const-reference to the Participant with the expectation
  /// that the Participant instance will outlive this StubbornNegotiator
  /// instance. The StubbornNegotiator costs very little to construct, so it is
  /// okay to use a pattern like
  ///
  /// \code
  /// StubbornNegotiator(participant).respond(table_view, responder);
  /// \endcode
  ///
  /// \param[in] participant
  ///   The Participant who wants to be stubborn.
  StubbornNegotiator(const Participant& participant);

  void respond(
      const schedule::Negotiation::Table::ViewerPtr& table_viewer,
      const ResponderPtr& responder) final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP
