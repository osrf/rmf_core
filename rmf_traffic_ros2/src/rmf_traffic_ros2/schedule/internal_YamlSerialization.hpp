#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace rmf_traffic_ros2 {

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
AtomicOperation atomic_operation(YAML::Node node);

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

//=============================================================================
YAML::Node serialize(AtomicOperation atomOp);

}
