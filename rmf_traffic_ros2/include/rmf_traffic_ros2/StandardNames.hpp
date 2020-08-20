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

#ifndef RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP
#define RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP

#include <string>

namespace rmf_traffic_ros2 {

const std::string Prefix = "rmf_traffic/";
const std::string ItinerarySetTopicName = Prefix + "itinerary_set";
const std::string ItineraryExtendTopicName = Prefix + "itinerary_extend";
const std::string ItineraryDelayTopicName = Prefix + "itinerary_delay";
const std::string ItineraryEraseTopicName = Prefix + "itinerary_erase";
const std::string ItineraryClearTopicName = Prefix + "itinerary_clear";
const std::string RegisterParticipantSrvName = Prefix + "register_participant";
const std::string UnregisterParticipantSrvName = Prefix +
  "unregister_participant";
const std::string RegisterQueryServiceName = Prefix + "register_query";
const std::string UnregisterQueryServiceName = Prefix + "unregister_query";
const std::string MirrorUpdateServiceName = Prefix + "mirror_update";
const std::string MirrorWakeupTopicName = Prefix + "mirror_wakeup";
const std::string ScheduleInconsistencyTopicName = Prefix +
  "schedule_inconsistency";
const std::string NegotiationAckTopicName = Prefix +
  "negotiation_ack";
const std::string NegotiationRepeatTopicName = Prefix +
  "negotiation_repeat";
const std::string NegotiationNoticeTopicName = Prefix +
  "negotiation_notice";
const std::string NegotiationRefusalTopicName = Prefix +
  "negotiation_refusal";
const std::string NegotiationProposalTopicName = Prefix +
  "negotiation_proposal";
const std::string NegotiationRejectionTopicName = Prefix +
  "negotiation_rejection";
const std::string NegotiationForfeitTopicName = Prefix +
  "negotiation_forfeit";
const std::string NegotiationConclusionTopicName = Prefix +
  "negotiation_conclusion";

const std::string EmergencyTopicName = "fire_alarm_trigger";

} // namespace rmf_traffic_ros2

#endif // RMF_TRAFFIC_ROS2__STANDARDNAMES_HPP
