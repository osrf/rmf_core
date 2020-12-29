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

#ifndef RMF_TRAFFIC_ROS2__PARTICIPANT_REGISTRY_HPP
#define RMF_TRAFFIC_ROS2__PARTICIPANT_REGISTRY_HPP


#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace rmf_traffic_ros2 { 


using Database = rmf_traffic::schedule::Database; 
using ParticipantId = rmf_traffic::schedule::ParticipantId;
using ParticipantDescription = rmf_traffic::schedule::ParticipantDescription;  

//=============================================================================
struct AtomicOperation
{
  enum class OpType : uint8_t
  {
    Add = 0,
    Remove
  };
  OpType operation;
  ParticipantDescription description;
};

//=============================================================================
class AbstractParticipantLogger
{
public:
  virtual ~AbstractParticipantLogger() {};
  virtual void write_operation(AtomicOperation operation) = 0;
  //TODO: Use an iterator
  virtual std::optional<AtomicOperation> read_next_record() = 0;
};

//=============================================================================
/// Adds a persistance layer to the participant ids. This allows the scheduler 
/// to restart without the need to restart fleet adapters. 
/// Internally, this class implements a an append only journal. This makes it 
/// independent of any id generation inside the database, as long as the said 
/// database id generation algorithm is deterministic.
class ParticipantRegistry
{
public:
  ParticipantRegistry(
    AbstractParticipantLogger* file_name,
    std::shared_ptr<Database> database);

  /// Adds a participant
  /// \param[in] description - The description of the participant that one
  ///   wishes to register.
  /// 
  /// \throws std::runtime_error if a participant with the same name and owner
  ///   are already in the registry.
  ParticipantId add_participant(ParticipantDescription description);
  
  /// Removes a participant from the registry.
  /// \param[in] id - participant to remove
  void remove_participant(ParticipantId id);
  
  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//=============================================================================
ParticipantDescription::Rx responsiveness(std::string response);

//=============================================================================
uint8_t shapetype(YAML::Node node);

//=============================================================================
rmf_traffic_msgs::msg::ConvexShape convexshape(YAML::Node node);

//=============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext shapecontext(YAML::Node node);

//=============================================================================
rmf_traffic::Profile profile(YAML::Node node);

//=============================================================================
ParticipantDescription participant_description(YAML::Node node);

//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext context);

//=============================================================================
std::string serialize_shape_type(uint8_t shape_type);

//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape);

//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext shape);

//=============================================================================
YAML::Node serialize(rmf_traffic::Profile profile);

//=============================================================================
std::string serialize_responsiveness(ParticipantDescription::Rx resp);

//=============================================================================
YAML::Node serialize(ParticipantDescription participant);

} // end namespace rmf_traffic_ros2

#endif