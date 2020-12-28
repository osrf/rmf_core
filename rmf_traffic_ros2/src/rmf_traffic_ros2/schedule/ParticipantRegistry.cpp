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

#include <mutex>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

namespace rmf_traffic_ros2 {

using Database = rmf_traffic::schedule::Database;
using ParticipantId = rmf_traffic::schedule::ParticipantId;
using ParticipantDescription = rmf_traffic::schedule::ParticipantDescription;

//=============================================================================
struct UniqueId
{
  std::string name;
  std::string owner;

  bool operator==(const UniqueId &other) const
  {
    return name == other.name && owner == other.owner;
  }
};

//=============================================================================
struct UniqueIdHasher
{
  std::size_t operator()(UniqueId id) const
  {
    return std::hash<std::string>{}(id.name + id.owner);
  }
};

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
ParticipantDescription participant_description(YAML::Node node)
{

  if(node.IsMap())
  {
    throw std::runtime_error("Malformatted YAML file. Expected a map");
  }
  
  if(!node["name"]) {
    throw std::runtime_error("Malformatted YAML file. Expected a name.");
  }
  
  if(!node["owner"]) {
    throw std::runtime_error("Malformatted YAML file. Expected a owner.");
  }
  
  if(!node["responsiveness"]) {
    throw std::runtime_error(
      "Malformatted YAML file. Expected a responsiveness field"
    );
  }
  
  if(!node["profile"]) {
    throw std::runtime_error("Malformatted YAML file. Expected a profile");
  }
  
  std::string name = node["name"].as<std::string>();
  std::string owner = node["owner"].as<std::string>();
  ParticipantDescription final_desc(
    name,
    owner,
    responsiveness(node["responsiveness"].as<std::string>()),
    profile(node["profile"])
  );

  return final_desc;
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
  if(shape_type == rmf_traffic_msgs::msg::ConvexShape::NONE)
    return "None";
  if(shape_type == rmf_traffic_msgs::msg::ConvexShape::BOX)
    return "Box";
  if(shape_type == rmf_traffic_msgs::msg::ConvexShape::CIRCLE)
    return "Circle";

  throw std::runtime_error("Shape type must be one of None, Box, Circle");
}

//=============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape)
{
  YAML::Node node;
  node["type"] = serialize_shape_type(shape.type);
  node["index"] = shape.index;
  return node;
}

//=============================================================================
YAML::Node serialize(rmf_traffic::Profile profile)
{
  YAML::Node node;
  rmf_traffic_msgs::msg::Profile profile_msg= convert(profile);
  node["footprint"] = serialize(profile_msg.footprint);
  node["vicinity"] = serialize(profile_msg.vicinity);
  node["shapecontext"] = serialize(profile_msg.shape_context);
  return node;
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
  //===========================================================================
  Implementation(std::string file_name, bool writeable)
  {
    _writeable = writeable;
    auto node = YAML::LoadFile(file_name);
    init(node);
  }

  //===========================================================================
  Implementation(YAML::Node node)
  {
    _writeable = false;
    init(node);
  }

  //===========================================================================
  Implementation()
  {
    _next_id = 0;
    _writeable = false;
  }

  //===========================================================================
  void restore_database(Database& db)
  {

  }
  
  //===========================================================================
  YAML::Node to_yaml()
  {
    YAML::Node node;
    
    std::lock_guard<std::mutex> lock(mutex);
    for(auto participant: _descriptions)
    {
      auto _id = participant.first;
      node[_id] = serialize(participant.second);
    }
    return node;
  }

  //===========================================================================
  void write_to_file()
  {
    if(!_writeable) return;
  }

  //===========================================================================
  void add_participant(ParticipantDescription description)
  {
    ParticipantId curr_id;
    {
      std::lock_guard<std::mutex> lock(mutex);
      UniqueId key = {description.name(), description.owner()};

      if(_id_from_name.count(key))
      {
        throw std::runtime_error("Participant with name: "+description.name()
          + "and owner: "+description.owner() + "already exists" );
      }

      _descriptions.insert({_next_id, description});
      _id_from_name[key] = _next_id;
      curr_id = _next_id;
      _next_id++;
    }
  }

  //===========================================================================
  void remove_participant(ParticipantId id)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto description = _descriptions.find(id);
      if(description == _descriptions.end())
      {
        throw std::runtime_error("Participant with id " + std::to_string(id)
          + " does not exist");
      }
      
      UniqueId key = {description->second.name(), 
        description->second.owner()};

      _descriptions.erase(id);
      _id_from_name.erase(key);
    }
    write_to_file();
  }
private:

  //===========================================================================
  void init(YAML::Node node)
  {
    _next_id = 0;

    if(!node.IsMap()) {
      throw std::runtime_error(
        "Malformatted YAML file. Expected a map"  
      );
    }

    for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it)
    {
      ParticipantId id = it->first.as<uint64_t>();

      auto final_desc = participant_description(it->second);    
  
      UniqueId key = {final_desc.name(), final_desc.owner()}; 
      _descriptions.insert({id, final_desc});
      _id_from_name.insert({key, id});

      if(id >= _next_id)
      {
        _next_id += 1;
      }
    }
   
  }

  //==========================================================================
  bool _writeable;
  ParticipantId _next_id; ///Holds the next id
  std::unordered_map<ParticipantId, ParticipantDescription> _descriptions;
  std::unordered_map<UniqueId, 
    ParticipantId, UniqueIdHasher> _id_from_name;
  std::vector<AtomicOperation> _journal;
  std::mutex mutex, file_mutex;
};

//=============================================================================
ParticipantRegistry::ParticipantRegistry(std::string file_name, bool writeable)
:_pimpl(rmf_utils::make_unique_impl<Implementation>(file_name, writeable))
{
  
}

//=============================================================================
ParticipantRegistry::ParticipantRegistry(YAML::Node node)
:_pimpl(rmf_utils::make_unique_impl<Implementation>(node))
{

}

//=============================================================================
ParticipantRegistry::ParticipantRegistry()
:_pimpl(rmf_utils::make_unique_impl<Implementation>())
{

}

//=============================================================================
void ParticipantRegistry::restore_database(Database& db)
{
  _pimpl->restore_database(db);
}

//=============================================================================
void ParticipantRegistry::add_participant(
  ParticipantId id,
  ParticipantDescription description)
{
  _pimpl->add_participant(description);
}

//=============================================================================
void ParticipantRegistry::remove_participant(ParticipantId id)
{
  return _pimpl->remove_participant(id);
}

//=============================================================================
YAML::Node ParticipantRegistry::to_yaml()
{
  return _pimpl->to_yaml();
}
}//end namespace rmf_traffic_ros2
