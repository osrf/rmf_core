#include <rmf_utils/catch.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

SCENARIO("Test idempotency of shape type")
{
  using namespace rmf_traffic_ros2;
  YAML::Node node;

  // Check "None"
  auto serialized = serialize_shape_type(rmf_traffic_msgs::msg::ConvexShape::NONE);
  node["type"] = serialized;  
  CHECK(shapetype(node["type"]) == rmf_traffic_msgs::msg::ConvexShape::NONE);

  // Check "Box"
  serialized = serialize_shape_type(rmf_traffic_msgs::msg::ConvexShape::BOX);
  node["type"] = serialized;
  CHECK(shapetype(node["type"]) == rmf_traffic_msgs::msg::ConvexShape::BOX);

  // Check "Circle"
  serialized = serialize_shape_type(rmf_traffic_msgs::msg::ConvexShape::CIRCLE);
  node["type"] = serialized;
  CHECK(shapetype(node["type"]) == rmf_traffic_msgs::msg::ConvexShape::CIRCLE);

  REQUIRE_THROWS(serialize_shape_type(42));
  node["type"] = "not a shape";
  REQUIRE_THROWS(shapetype(node["type"]));
}

SCENARIO("Test idempotency of ParticipantDescription.")
{
  using namespace rmf_traffic_ros2;
  YAML::Node node;

  
}