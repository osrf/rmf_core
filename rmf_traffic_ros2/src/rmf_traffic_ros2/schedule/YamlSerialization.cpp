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

#include "internal_YamlSerialization.hpp"

namespace rmf_traffic_ros2 {
namespace schedule {

//==============================================================================
ParticipantDescription::Rx responsiveness(YAML::Node node)
{
  auto response = node.as<std::string>();
  if(response == "Unresponsive")
    return ParticipantDescription::Rx::Unresponsive;
  if(response == "Responsive")
    return ParticipantDescription::Rx::Responsive;

  throw YAML::ParserException(node.Mark(),
    "Responsiveness field contains unknown identifier");
}

//==============================================================================
uint8_t shape_type(YAML::Node node)
{
  auto type = node.as<std::string>();

  if(type == "Box")
    return rmf_traffic_msgs::msg::ConvexShape::BOX;

  if(type == "Circle")
    return rmf_traffic_msgs::msg::ConvexShape::CIRCLE;

  throw YAML::ParserException(node.Mark(),
    "Shape type must be one of Box, Circle");
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShape convex_shape(YAML::Node node)
{
  if(!node.IsMap())
    throw YAML::ParserException(node.Mark(),
      "Profile information should be a map");

  if(!node["type"])
    throw YAML::ParserException(node.Mark(),
      "Profile information missing footprint");

  if(!node["index"])
    throw YAML::ParserException(node.Mark(),
      "Profile information missing footprint");

  rmf_traffic_msgs::msg::ConvexShape shape;
  shape.type = shape_type(node["type"]);
  shape.index = node["index"].as<uint8_t>();
  return shape;
}

//==============================================================================
rmf_traffic_msgs::msg::ConvexShapeContext shape_context(YAML::Node node)
{
  //For now since only circles are supported, so I'm just going to store their
  //Radii. In future this should change.
  rmf_traffic_msgs::msg::ConvexShapeContext shape_context;
  if(!node.IsSequence())
    throw YAML::ParserException(node.Mark(),
      "Shape context should be a list");

  for(auto item: node)
  {
    double radius = item.as<double>();
    rmf_traffic_msgs::msg::Circle circle;
    circle.radius = radius;
    shape_context.circles.push_back(circle);
  }

  return shape_context;
}

//==============================================================================
rmf_traffic::Profile profile(YAML::Node node)
{
  if(!node.IsMap())
    throw YAML::ParserException(node.Mark(),
      "Profile information should be a map");

  if(!node["footprint"])
    throw YAML::ParserException(node.Mark(),
      "Profile information missing footprint");

  if(!node["vicinity"])
    throw YAML::ParserException(node.Mark(),
      "Profile information missing vicinity");

  if(!node["shapecontext"])
    throw YAML::ParserException(node.Mark(),
      "Profile information missing shape context");

  rmf_traffic_msgs::msg::Profile profile_msg;
  auto footprint = convex_shape(node["footprint"]);
  auto vicinity = convex_shape(node["vicinity"]);
  auto context = shape_context(node["shapecontext"]);

  profile_msg.footprint = footprint;
  profile_msg.vicinity = vicinity;
  profile_msg.shape_context = context;

  rmf_traffic::Profile profile = convert(profile_msg);
  return profile;
}

//==============================================================================
ParticipantDescription participant_description(YAML::Node node)
{

  if(!node.IsMap())
  {
    throw YAML::ParserException(node.Mark(),
      "Participant description should be a map field.");
  }

  if(!node["name"]) {
    throw YAML::ParserException(node.Mark(),
      "Participant description missing name field.");
  }

  if(!node["owner"]) {
    throw YAML::ParserException(node.Mark(),
      "Participant description missing owner field.");
  }

  if(!node["responsiveness"]) {
    throw YAML::ParserException(node.Mark(),
      "Participant description missing responsiveness field"
    );
  }

  if(!node["profile"]) {
    throw YAML::ParserException(node.Mark(),
      "Participant description missing a profile field");
  }

  std::string name = node["name"].as<std::string>();
  std::string owner = node["owner"].as<std::string>();
  ParticipantDescription final_desc(
    name,
    owner,
    responsiveness(node["responsiveness"]),
    profile(node["profile"])
  );

  return final_desc;
}

//==============================================================================
AtomicOperation atomic_operation(YAML::Node node)
{
  if(!node.IsMap())
    throw YAML::ParserException(node.Mark(),
      "Malformatted atomic operation Expected a map");

  AtomicOperation::OpType op_type;

  if(!node["operation"])
    throw YAML::ParserException(node.Mark(), "Expected an operation field.");

  if(node["operation"].as<std::string>() == "Add")
  {
    op_type = AtomicOperation::OpType::Add;
  }
  else
  {
    throw YAML::ParserException(node.Mark(), "Invalid operation.");
  }

  if(!node["participant_description"])
    throw YAML::ParserException(node.Mark(),
      "Expected a participant_description field");

  auto description = participant_description(node["participant_description"]);

  return {op_type, description};
}

//==============================================================================
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

//==============================================================================
std::string serialize_shape_type(uint8_t shape_type)
{
  if(shape_type == rmf_traffic_msgs::msg::ConvexShape::BOX)
    return "Box";
  if(shape_type == rmf_traffic_msgs::msg::ConvexShape::CIRCLE)
    return "Circle";

  throw std::runtime_error("Shape type must be one of Box, Circle");
}

//==============================================================================
YAML::Node serialize(rmf_traffic_msgs::msg::ConvexShape shape)
{
  YAML::Node node;
  node["type"] = serialize_shape_type(shape.type);
  node["index"] = shape.index;
  return node;
}

//==============================================================================
YAML::Node serialize(rmf_traffic::Profile profile)
{
  YAML::Node node;
  rmf_traffic_msgs::msg::Profile profile_msg= convert(profile);
  node["footprint"] = serialize(profile_msg.footprint);
  node["vicinity"] = serialize(profile_msg.vicinity);
  node["shapecontext"] = serialize(profile_msg.shape_context);
  return node;
}

//==============================================================================
std::string serialize_responsiveness(ParticipantDescription::Rx resp)
{
  if(resp == ParticipantDescription::Rx::Unresponsive)
    return "Unresponsive";
  if(resp == ParticipantDescription::Rx::Responsive)
    return "Responsive";
  throw std::runtime_error("Failed to seriallize responsiveness");
}

//==============================================================================
YAML::Node serialize(ParticipantDescription participant)
{
  YAML::Node node;

  node["name"] = participant.name();
  node["owner"] = participant.owner();
  node["responsiveness"] = serialize_responsiveness(participant.responsiveness());
  node["profile"] = serialize(participant.profile());

  return node;
}

//==============================================================================
YAML::Node serialize(AtomicOperation atomOp)
{
  YAML::Node node;

  if(atomOp.operation == AtomicOperation::OpType::Add)
  {
    node["operation"] = "Add";
  }
  else
  {
    throw std::runtime_error("Found an invalid operation");
  }
  node["participant_description"] = serialize(atomOp.description);

  return node;
}

} // namespace schedule
} // namespace rmf_traffic_ros2
