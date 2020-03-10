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

#include <rmf_traffic_ros2/schedule/Writer.hpp>

#include <rmf_traffic/geometry/Box.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

//==============================================================================
class ParticipantNode : public rclcpp::Node
{
public:

  ParticipantNode()
    : rclcpp::Node("schedule_participant_node")
  {
    writer = rmf_traffic_ros2::schedule::Writer::make(*this);
  }

  rmf_traffic_ros2::schedule::WriterPtr writer;

  rmf_utils::optional<rmf_traffic::schedule::Participant> participant;

};

//==============================================================================
std::shared_ptr<ParticipantNode> make_node()
{
  auto node = std::make_shared<ParticipantNode>();

  while(!node->writer->ready())
    rclcpp::spin_some(node);

  using namespace std::chrono_literals;
  const auto now = std::chrono::steady_clock::now();

  rmf_traffic::schedule::ParticipantDescription description{
    "some participant",
    "schedule_participant_node",
    rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Box>(1.0, 2.0)
    }
  };

  rmf_traffic::Trajectory t;
  t.insert(now, {0, 0, 0}, {0, 0, 0});
  t.insert(now + 10s, {0, 10, 0}, {0, 0, 0});

  node->writer->async_make_participant(
        std::move(description),
        [node, t = std::move(t)](rmf_traffic::schedule::Participant participant)
  {
    node->participant = std::move(participant);
    std::cout << "Sending trajectory" << std::endl;
    node->participant->set({{"test map", std::move(t)}});
  });

  return node;
}

//==============================================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_node());
}
