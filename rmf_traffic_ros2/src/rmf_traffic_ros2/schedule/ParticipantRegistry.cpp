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

#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

namespace rmf_traffic_ros2 {

using Database = rmf_traffic::schedule::Database;
using ParticipantId = rmf_traffic::schedule::ParticipantId;
using ParticipantDescription = rmf_traffic::schedule::ParticipantDescription;

//=============================================================================
ParticipantDescription::Rx responsiveness(std::string response)
{
  if(response == "Invalid")
    return ParticipantDescription::Rx::Invalid;
  if(response == "Unresponsive")
    return ParticipantDescription::Rx::Unresponsive;
  if(response == "Responsive")
    return ParticipantDescription::Rx::Responsive;
  throw std::runtime_error("Responsiveness field contains invalid identifier");
}

//=============================================================================
uint8_t shapetype(YAML::Node node)
{
  auto type = node.as<std::string>();
  if(type == "None")
    return rmf_traffic_msgs::msg::ConvexShape::NONE;
  if(type == "Box")
    return rmf_traffic_msgs::msg::ConvexShape::BOX;
  if(type == "Circle")
    return rmf_traffic_msgs::msg::ConvexShape::CIRCLE;

  throw std::runtime_error("Shape type must be one of None, Box, Circle");
}

//=============================================================================
rmf_traffic_msgs::msg::ConvexShape convexshape(YAML::Node node)
{
  if(!node.IsMap())
    throw std::runtime_error("Profile information malformatted");

  if(!node["type"])
    throw std::runtime_error("Profile information missing footprint");

  if(!node["index"])
    throw std::runtime_error("Profile information missing footprint");

  rmf_traffic_msgs::msg::ConvexShape shape;
  shape.type = shapetype(node["type"]);
  shape.index = node["index"].as<uint8_t>();
  return shape;
}

//=============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext shapecontext(YAML::Node node)
{
  //For now since only circles are supported, so I'm just going to store their
  //Radii. In future this should change.
  rmf_traffic_msgs::msg::ConvexShapeContext shape_context;
  if(!node.IsSequence())
    throw std::runtime_error("Expected a list");

  for(auto item: node)
  {
    double radius = item.as<double>();
    rmf_traffic_msgs::msg::Circle circle;
    circle.radius = radius;
    shape_context.circles.push_back(circle);
  }

  return shape_context;
}

//=============================================================================
rmf_traffic::Profile profile(YAML::Node node)
{
  if(!node.IsMap())
    throw std::runtime_error("Profile information malformatted");

  if(!node["footprint"])
    throw std::runtime_error("Profile information missing footprint");

  if(!node["vicinity"])
    throw std::runtime_error("Profile information missing footprint");

  if(!node["shapecontext"])
    throw std::runtime_error("Profile information missing footprint");
  
  rmf_traffic_msgs::msg::Profile profile_msg;
  auto footprint = convexshape(node["footprint"]);
  auto vicinity = convexshape(node["vicinity"]);
  auto context = shapecontext(node["shapecontext"]);

  profile_msg.footprint = footprint;
  profile_msg.vicinity = vicinity;
  profile_msg.shape_context = context;
  
  rmf_traffic::Profile profile = convert(profile_msg);
  return profile;
}


//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShapeContext context)
{
  //For now since only circles are supported, so I'm just going to store their
  //Radii. In future this should change.
  YAML::Node node;
  for(auto circle: context.circles)
  {
    node.push_back(circle.radius);
  }
  return node;
}

//=============================================================================
std::string serialize_shape_type(uint8_t shape_type)
{

}

//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape)
{
  
}

//=============================================================================
YAML::Node serialize(rmf_traffic::Profile profile)
{
  
}

//=============================================================================
std::string serialize_responsiveness(ParticipantDescription::Rx resp)
{
  if(resp == ParticipantDescription::Rx::Invalid)
    return "Invalid";
  if(resp == ParticipantDescription::Rx::Unresponsive)
    return "Unresponsive";
  if(resp == ParticipantDescription::Rx::Responsive)
    return "Responsive";
  throw std::runtime_error("Failed to seriallize responsiveness");
}

//=============================================================================
YAML::Node serialize(ParticipantDescription participant)
{
  YAML::Node node;

  node["name"] = participant.name();
  node["owner"] = participant.owner();
  node["responsiveness"] = serialize_responsiveness(participant.responsiveness());
  node["profile"] = serialize(participant.profile());

  return node;
}

//=============================================================================
class ParticipantRegistry::Implementation
{
public:
  bool _writeable;
  std::unordered_map<ParticipantId, ParticipantDescription> _descriptions;
  
  //===========================================================================
  Implementation(std::string file_name, bool writeable)
  {
    auto node = YAML::LoadFile(file_name);
    if(!node.IsMap()) {
      throw std::runtime_error(
        "Malformatted YAML file. Expected a map"  
      );
    }
    
    for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it)
    {
      ParticipantId id = it->first.as<uint64_t>();
      if(!it->second.IsMap())
      {
        throw std::runtime_error(
          "Malformatted YAML file. Expected a map under participant id: " +
          std::to_string(id) 
        );
      }
  
      auto description = it->second;
      if(!description["name"]) {
        throw std::runtime_error(
          "Malformatted YAML file. Expected a name under participant id: " +
          std::to_string(id) 
        );
      }
  
      if(!description["owner"]) {
        throw std::runtime_error(
          "Malformatted YAML file. Expected a owner under participant id: " +
          std::to_string(id) 
        );
      }
  
      if(!description["responsiveness"]) {
        throw std::runtime_error(
          "Malformatted YAML file. Expected a responsiveness field under participant id: " +
          std::to_string(id) 
        );
      }
  
      if(!description["profile"]) {
        throw std::runtime_error(
          "Malformatted YAML file. Expected a  under participant id: " +
          std::to_string(id) 
        );
      }
  
  
      ParticipantDescription final_desc(
          description["name"].as<std::string>(),
          description["owner"].as<std::string>(),
          responsiveness(description["responsiveness"].as<std::string>()),
          profile(description["profile"])
      );
  
      _descriptions.insert({id, final_desc});
    }
    _writeable = writeable;
  }

  //===========================================================================
  void write_to_file()
  {

  }
};

//=============================================================================
ParticipantRegistry::ParticipantRegistry(std::string file_name, bool writeable)
:_pimpl(rmf_utils::make_unique_impl<Implementation>(file_name, writeable))
{
 
}

//=============================================================================
void ParticipantRegistry::initiallize_database(Database& db)
{
  //_descriptions
  //db.initiallize_with_static_participants();
}

//=============================================================================
void ParticipantRegistry::add_participant(ParticipantId id,
  ParticipantDescription decription)
{

}

//=============================================================================
void ParticipantRegistry::remove_participant(ParticipantId id)
{

}

}//end namespace rmf_traffic_ros2
