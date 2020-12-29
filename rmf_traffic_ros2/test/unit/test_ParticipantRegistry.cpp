#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

#include <rmf_utils/catch.hpp>

using namespace rmf_traffic_ros2;
SCENARIO("Test idempotency of shape type")
{
  
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

  
}


class TestOperationLogger: public AbstractParticipantLogger
{
  std::vector<AtomicOperation> _journal;
  std::size_t _counter;
public:
  
  TestOperationLogger()
  {
    _counter = 0;
  }

  void write_operation(AtomicOperation operation) override
  {
    _journal.push_back(operation);
  }

  std::optional<AtomicOperation> read_next_record() override
  {
    if(_counter >= _journal.size())
    {
      return std::nullopt;
    }
    return {_journal[_counter++]};    
  }
};

SCENARIO("Participant registry restores participants from logger")
{
  using Database = rmf_traffic::schedule::Database;
  using ParticipantId = rmf_traffic::schedule::ParticipantId;

  
  
  TestOperationLogger* logger = new TestOperationLogger;

  //Lets create a bunch of participants
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  rmf_traffic::schedule::ParticipantDescription p1(
    "participant 1",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  rmf_traffic::schedule::ParticipantDescription p2(
    "participant 2",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});
  
  rmf_traffic::schedule::ParticipantDescription p3(
    "participant 3",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});
  
  ParticipantId participant_id1, participant_id2, participant_id3;
  
  {
    auto db1 = std::make_shared<Database>();
    ParticipantRegistry registry1(logger, db1);
    participant_id1 = registry1.add_participant(p1);
    participant_id2 = registry1.add_participant(p2);
    registry1.remove_participant(participant_id2);
    participant_id3 = registry1.add_participant(p3);
  }

  
  {
    auto db2 = std::make_shared<Database>();
    ParticipantRegistry registry2(logger, db2);
    REQUIRE(db2->participant_ids().size() == 2);
  }
  delete logger;

}