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


bool operator==(const rmf_traffic::Profile p1, const rmf_traffic::Profile p2) 
{
  return convert(p1) == convert(p2);
}

bool operator==(
  const rmf_traffic::schedule::ParticipantDescription desc1, 
  const rmf_traffic::schedule::ParticipantDescription desc2)  
{
  return desc1.name() == desc2.name()
    && desc1.owner() == desc2.owner()
    && desc1.responsiveness() ==  desc2.responsiveness()
    && desc1.profile() == desc2.profile();
}

SCENARIO("Test idempotency of ParticipantDescription.")
{
  //Lets create a bunch of participants
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);

  rmf_traffic::schedule::ParticipantDescription p1(
    "participant 1",
    "test_Participant",
    rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
    rmf_traffic::Profile{shape});

  auto serialized = serialize(p1);
  
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
  
  GIVEN("A stubbed out logger")
  {
    TestOperationLogger* logger = new TestOperationLogger;
    WHEN("Creating a new DB without errors")
    {
      auto db1 = std::make_shared<Database>();
      ParticipantRegistry registry1(logger, db1);
      participant_id1 = registry1.add_participant(p1);
      participant_id2 = registry1.add_participant(p2);
      registry1.remove_participant(participant_id2);
      participant_id3 = registry1.add_participant(p3);
  
      THEN("Restoring DB")
      {
        auto db2 = std::make_shared<Database>();
        ParticipantRegistry registry2(logger, db2);
        auto restored_participants = db2->participant_ids();
        REQUIRE(restored_participants.count(participant_id1) > 0);
        REQUIRE(restored_participants.count(participant_id2) == 0);
        REQUIRE(restored_participants.count(participant_id3) > 0);
        REQUIRE(restored_participants.size() == 2);
  
        auto _p1 = db2->get_participant(participant_id1);
        auto _p3 = db2->get_participant(participant_id3);
  
        REQUIRE(*_p1 == p1);
        REQUIRE(*_p3 == p3);
      }
    }
  
    WHEN("Creating a new DB with erroneous remove request")
    {
      auto db1 = std::make_shared<Database>();
      ParticipantRegistry registry1(logger, db1);
      participant_id1 = registry1.add_participant(p1);
      REQUIRE_THROWS(registry1.remove_participant(1000));
      participant_id2 = registry1.add_participant(p2);
      registry1.remove_participant(participant_id2);
      participant_id3 = registry1.add_participant(p3);
  
      THEN("Restoring DB")
      {
        auto db2 = std::make_shared<Database>();
        ParticipantRegistry registry2(logger, db2);
        auto restored_participants = db2->participant_ids();
        REQUIRE(restored_participants.count(participant_id1) > 0);
        REQUIRE(restored_participants.count(participant_id2) == 0);
        REQUIRE(restored_participants.count(participant_id3) > 0);
        REQUIRE(restored_participants.size() == 2);
  
        auto _p1 = db2->get_participant(participant_id1);
        auto _p3 = db2->get_participant(participant_id3);
  
        REQUIRE(*_p1 == p1);
        REQUIRE(*_p3 == p3);
      }
    }
    delete logger;
  }
}

SCENARIO("Test file logger")
{

}